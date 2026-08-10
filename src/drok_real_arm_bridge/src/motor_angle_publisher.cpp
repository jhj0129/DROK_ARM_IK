#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <iomanip>
#include <map>
#include <algorithm>
#include <functional>
#include <sstream>
#include <cmath> // M_PI

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Linux SocketCAN
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cerrno>

// --- Configuration ---
#define CMD_READ_ANGLE 0x92
const std::chrono::milliseconds UPDATE_INTERVAL(10);
const std::chrono::microseconds SINGLE_READ_TIMEOUT_US(5000);

struct MotorInfo {
    std::string name;
    uint32_t can_id;
    std::string interface_name;
    int64_t raw_angle_internal;
    double angle_degrees_internal;
    bool updated_this_cycle_internal;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher;

    MotorInfo(std::string n, uint32_t id, std::string if_name)
        : name(std::move(n)), can_id(id), interface_name(std::move(if_name)),
          raw_angle_internal(0), angle_degrees_internal(0.0), updated_this_cycle_internal(false),
          angle_publisher(nullptr) {}
};

struct CanInterface {
    std::string name;
    int socket_fd = -1;
    std::vector<uint32_t> motor_ids;
    std::map<uint32_t, size_t> motor_id_to_all_motors_index;

    CanInterface(std::string n, std::vector<uint32_t> ids)
        : name(std::move(n)), motor_ids(std::move(ids)) {}
};

// (옵션) 모터별 각도 스케일을 한곳에서 관리
static inline double scale_deg_per_cnt(
    uint32_t can_id,
    const std::string& ifname)
{
    // Publish OUTPUT-SHAFT angle in degrees.
    // can11 0x143/0x144 are RMD X4-P6 (6:1).
    if (ifname == "can10") {
        return 0.01 / 36.0;
    }

    if (ifname == "can11") {
        if (can_id == 0x141 || can_id == 0x142) {
            return 0.01;
        }

        if (can_id == 0x143 || can_id == 0x144) {
            return 0.01 / 6.0;
        }
    }

    return 0.01;
}

class MotorControllerNode : public rclcpp::Node {
public:
    MotorControllerNode() : Node("motor_angle_publisher") {
        RCLCPP_INFO(this->get_logger(), "Motor Angle Publisher Node starting...");

        initialize_interfaces_and_motors();

        if (all_motors_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No motors configured or CAN interfaces failed.");
            return;
        }

        // JointState 퍼블리셔
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos_profile);

        RCLCPP_INFO(this->get_logger(), "Configured to read %zu motor angles.", all_motors_.size());

        // 공통 요청 프레임
        frame_tx_.can_dlc = 8;
        frame_tx_.data[0] = CMD_READ_ANGLE;
        for (int i = 1; i < 8; ++i) frame_tx_.data[i] = 0x00;

        // 주기 타이머
        timer_ = this->create_wall_timer(
            UPDATE_INTERVAL, std::bind(&MotorControllerNode::update_and_publish, this));
    }

    ~MotorControllerNode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down Motor Angle Publisher Node.");
        for (auto& cif : can_interfaces_config_) {
            if (cif.socket_fd != -1) {
                close(cif.socket_fd);
                RCLCPP_INFO(this->get_logger(), "Closed socket for %s", cif.name.c_str());
            }
        }
    }

private:
    void initialize_interfaces_and_motors() {
        // 인터페이스 및 모터 ID 구성
        can_interfaces_config_.emplace_back("can10", std::vector<uint32_t>{0x141, 0x142, 0x143, 0x144});
        can_interfaces_config_.emplace_back("can11", std::vector<uint32_t>{0x141, 0x142, 0x143, 0x144});

        for (auto& cif_config : can_interfaces_config_) {
            if (!initialize_can_socket(cif_config)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN socket for %s.", cif_config.name.c_str());
            }

            for (uint32_t motor_id : cif_config.motor_ids) {
                std::stringstream ss_hex_id;
                ss_hex_id << std::hex << motor_id;
                std::string motor_ros_name_part = cif_config.name + "_motor_0x" + ss_hex_id.str();

                all_motors_.emplace_back(motor_ros_name_part, motor_id, cif_config.name);
                MotorInfo& current_motor = all_motors_.back();

                if (cif_config.socket_fd != -1) {
                    std::string topic_name = "motor_angles/" + motor_ros_name_part;
                    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
                    qos_profile.reliable();
                    current_motor.angle_publisher = this->create_publisher<std_msgs::msg::Float64>(topic_name, qos_profile);

                    RCLCPP_INFO(this->get_logger(),
                                "Created publisher for motor '%s' (CAN ID: 0x%X) on topic '%s'",
                                current_motor.name.c_str(),
                                current_motor.can_id,
                                current_motor.angle_publisher->get_topic_name());
                } else {
                    RCLCPP_WARN(this->get_logger(),
                                "Publisher not created for motor '%s' due to failed CAN interface '%s'.",
                                current_motor.name.c_str(), cif_config.name.c_str());
                }
                cif_config.motor_id_to_all_motors_index[motor_id] = all_motors_.size() - 1;
            }
        }
    }

    bool initialize_can_socket(CanInterface& cif) {
        cif.socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (cif.socket_fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error opening socket for %s: %s",
                         cif.name.c_str(), strerror(errno));
            return false;
        }

        struct ifreq ifr {};
        strncpy(ifr.ifr_name, cif.name.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';

        if (ioctl(cif.socket_fd, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "ioctl failed for %s: %s",
                         cif.name.c_str(), strerror(errno));
            close(cif.socket_fd);
            cif.socket_fd = -1;
            return false;
        }

        struct sockaddr_can addr {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(cif.socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed for %s: %s",
                         cif.name.c_str(), strerror(errno));
            close(cif.socket_fd);
            cif.socket_fd = -1;
            return false;
        }

        struct timeval tv {};
        tv.tv_sec = 0;
        tv.tv_usec = SINGLE_READ_TIMEOUT_US.count();
        if (setsockopt(cif.socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv) < 0) {
            RCLCPP_WARN(this->get_logger(), "setsockopt SO_RCVTIMEO failed for %s.", cif.name.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Successfully initialized CAN socket for: %s (fd: %d)",
                    cif.name.c_str(), cif.socket_fd);
        return true;
    }

    static inline bool write_all_retry_eintr(int fd, const void* buf, size_t len) {
        const uint8_t* p = static_cast<const uint8_t*>(buf);
        size_t sent = 0;
        while (sent < len) {
            ssize_t n = ::write(fd, p + sent, len - sent);
            if (n < 0) {
                if (errno == EINTR) continue;
                return false;
            }
            sent += static_cast<size_t>(n);
        }
        return true;
    }

    void update_and_publish() {
        for (auto& motor : all_motors_) motor.updated_this_cycle_internal = false;

        // 요청 송신
        for (const auto& cif : can_interfaces_config_) {
            if (cif.socket_fd == -1) continue;
            for (uint32_t motor_id_to_send : cif.motor_ids) {
                frame_tx_.can_id = motor_id_to_send;
                if (!write_all_retry_eintr(cif.socket_fd, &frame_tx_, sizeof(struct can_frame))) {
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "CAN TX error on %s for ID 0x%X: %s",
                        cif.name.c_str(), motor_id_to_send, strerror(errno));
                }
            }
        }

        auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(7);

        // 응답 수신
        for (auto& cif : can_interfaces_config_) {
            if (cif.socket_fd == -1) continue;
            int received_count_this_interface = 0;

            while (std::chrono::steady_clock::now() < receive_deadline) {
                struct can_frame frame_rx {};
                ssize_t nbytes_rx = ::read(cif.socket_fd, &frame_rx, sizeof(struct can_frame));

                if (nbytes_rx < 0) {
                    if (errno == EINTR) continue; // 시그널에 의해 깨어났으면 재시도
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break; // 타임아웃
                    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "CAN RX error on %s: %s", cif.name.c_str(), strerror(errno));
                    break;
                } else if (static_cast<size_t>(nbytes_rx) < sizeof(struct can_frame)) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Incomplete CAN frame on %s", cif.name.c_str());
                    continue;
                }

                uint32_t received_id = frame_rx.can_id & CAN_SFF_MASK; // 플래그 제거
                uint32_t lookup_id = received_id;

                // === can11 응답 매핑 (요구사항) ===
                if (cif.name == "can11") {
                    switch (received_id) {
                        case 0x241: lookup_id = 0x141; break; // id1
                        case 0x242: lookup_id = 0x142; break; // id2
                        case 0x143: lookup_id = 0x143; break; // id3: 동일
                        case 0x144: lookup_id = 0x144; break; // id4: 동일
                        default: break; // 기타는 그대로/무시
                    }
                }

                auto it_motor_map_entry = cif.motor_id_to_all_motors_index.find(lookup_id);
                if (it_motor_map_entry == cif.motor_id_to_all_motors_index.end()) {
                    continue; // 관심 없는 ID
                }

                if (frame_rx.data[0] != CMD_READ_ANGLE) {
                    continue; // 다른 명령 응답은 스킵
                }

                MotorInfo& motor_internal_state = all_motors_[it_motor_map_entry->second];
                bool parsed_successfully = false;

                // 인터페이스별 데이터 해석
                if (motor_internal_state.interface_name == "can10") {
                    // 6 바이트(1..6) 사용 + sign extend
                    if (frame_rx.can_dlc >= 7) {
                        int64_t motorAngleRaw =
                              (int64_t)frame_rx.data[1]
                            | ((int64_t)frame_rx.data[2] << 8)
                            | ((int64_t)frame_rx.data[3] << 16)
                            | ((int64_t)frame_rx.data[4] << 24)
                            | ((int64_t)frame_rx.data[5] << 32)
                            | ((int64_t)frame_rx.data[6] << 40);
                        if (frame_rx.data[6] & 0x80) motorAngleRaw |= 0xFFFF000000000000LL; // sign extend
                        motor_internal_state.raw_angle_internal = motorAngleRaw;
                        motor_internal_state.angle_degrees_internal =
                            (double)motorAngleRaw * scale_deg_per_cnt(motor_internal_state.can_id,
                                                                      motor_internal_state.interface_name);
                        motor_internal_state.updated_this_cycle_internal = true;
                        parsed_successfully = true;
                    }
                } else { // can11
                    if (motor_internal_state.can_id == 0x141 || motor_internal_state.can_id == 0x142) {
                        // 4바이트(4..7)
                        if (frame_rx.can_dlc >= 8) {
                            int32_t motorAngleRaw_i32 =
                                  (int32_t)frame_rx.data[4]
                                | ((int32_t)frame_rx.data[5] << 8)
                                | ((int32_t)frame_rx.data[6] << 16)
                                | ((int32_t)frame_rx.data[7] << 24);
                            motor_internal_state.raw_angle_internal = motorAngleRaw_i32;
                            motor_internal_state.angle_degrees_internal =
                                (double)motorAngleRaw_i32 * scale_deg_per_cnt(motor_internal_state.can_id,
                                                                              motor_internal_state.interface_name);
                            motor_internal_state.updated_this_cycle_internal = true;
                            parsed_successfully = true;
                        }
                    } else { // 0x143, 0x144: 6바이트(1..6)
                        if (frame_rx.can_dlc >= 7) {
                            int64_t motorAngleRaw =
                                  (int64_t)frame_rx.data[1]
                                | ((int64_t)frame_rx.data[2] << 8)
                                | ((int64_t)frame_rx.data[3] << 16)
                                | ((int64_t)frame_rx.data[4] << 24)
                                | ((int64_t)frame_rx.data[5] << 32)
                                | ((int64_t)frame_rx.data[6] << 40);
                            if (frame_rx.data[6] & 0x80) motorAngleRaw |= 0xFFFF000000000000LL; // sign extend
                            motor_internal_state.raw_angle_internal = motorAngleRaw;
                            motor_internal_state.angle_degrees_internal =
                                (double)motorAngleRaw * scale_deg_per_cnt(motor_internal_state.can_id,
                                                                          motor_internal_state.interface_name);
                            motor_internal_state.updated_this_cycle_internal = true;
                            parsed_successfully = true;
                        }
                    }
                }

                if (parsed_successfully && motor_internal_state.angle_publisher) {
                    auto angle_msg = std::make_unique<std_msgs::msg::Float64>();
                    angle_msg->data = motor_internal_state.angle_degrees_internal;
                    motor_internal_state.angle_publisher->publish(std::move(angle_msg));
                    received_count_this_interface++;
                }

                if (received_count_this_interface >= (int)cif.motor_ids.size()) break;
            }
        }

        publish_joint_states();
    }

    void publish_joint_states() {
        auto msg = std::make_unique<sensor_msgs::msg::JointState>();
        msg->header.stamp = this->get_clock()->now();

        // 8개 고정
        const std::vector<std::string> joint_names = {
            "joint1","joint2","joint3","joint4","joint5","joint6","joint7","joint8"
        };
        const std::vector<std::string> motor_ros_names = {
            "can10_motor_0x141", // joint1
            "can10_motor_0x142", // joint2
            "can10_motor_0x143", // joint3
            "can10_motor_0x144", // joint4
            "can11_motor_0x141", // joint5
            "can11_motor_0x142", // joint6
            "can11_motor_0x143", // joint7
            "can11_motor_0x144", // joint8
        };

        if (joint_names.size() != motor_ros_names.size()) {
            RCLCPP_ERROR_ONCE(this->get_logger(), "Mismatch between joint_names and motor_ros_names size!");
            return;
        }

        msg->name = joint_names;
        msg->position.reserve(motor_ros_names.size());

        for (size_t i = 0; i < motor_ros_names.size(); ++i) {
            const auto& target = motor_ros_names[i];
            const MotorInfo* found = nullptr;
            for (const auto& m : all_motors_) {
                if (m.name == target) { found = &m; break; }
            }

            double angle_rad = 0.0;
            if (found) {
                angle_rad = found->angle_degrees_internal * M_PI / 180.0;

                // (필요시) 개별 부호/오프셋 보정
                // 예시:
                // if (target == "can10_motor_0x144") angle_rad = -angle_rad;
                // if (target == "can11_motor_0x144") angle_rad = -angle_rad;

            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "No motor info for '%s'; pushing 0.0", target.c_str());
            }
            msg->position.push_back(angle_rad);
        }

        joint_state_publisher_->publish(std::move(msg));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<CanInterface> can_interfaces_config_;
    std::vector<MotorInfo> all_motors_;
    struct can_frame frame_tx_{};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
