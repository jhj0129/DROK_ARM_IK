#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
// ARM: 차체 -> 팔 enable, 팔 -> 차체 done 신호용 Bool 메시지 추가
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp> // IMU 메시지 추가
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class SummerNode : public rclcpp::Node {
public:
    SummerNode() : Node("Summer_Node") {
        // 전방 카메라 구독
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, 
            std::bind(&SummerNode::process_front, this, std::placeholders::_1));

        // IMU 데이터 구독 추가
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->current_imu_linear_x = msg->linear_acceleration.x;
            });

        yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_bbox_raw", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            std::string data = msg->data;
            auto status_msg = std_msgs::msg::String();

            if (data.find("red_light") != std::string::npos) {
                is_red_light = true;
                status_msg.data = "STOP";
                status_pub_->publish(status_msg);
            } 
            else if (data.find("green_light") != std::string::npos) {
                is_red_light = false;
                status_msg.data = "GO";
                status_pub_->publish(status_msg);
            }
            
            // ARM: 기존 supply_box -> 20초 is_waiting 시작 부분을 팔 미션 시작 인터락으로 연결
            if (data.find("supply_box") != std::string::npos && !is_waiting) {
                start_arm_sequence_if_needed();
            }
        });

        nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { 
                this->current_nav = *msg; 
            });

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        ui_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/ui_combined_vision", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/light_status", 10);

        // ARM: q.py가 발행하는 supply_box 문자열을 직접 받아 차체 정지/팔 시작에 사용
        // ARM: 기존 /yolo_bbox_raw 구독은 신호등 및 기존 기능 유지를 위해 그대로 둠
        arm_yolo_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/yolo_detected_object", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.find("supply_box") != std::string::npos) {
                    start_arm_sequence_if_needed();
                }
            });

        // ARM: Summer -> DROK_ARM_IK. True를 한 번 발행하면 팔이 카메라 XYZ/TF 기반 grasp 시작
        arm_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/drok_arm_auto/enable", 10);

        // ARM: DROK_ARM_IK가 grasp -> lift -> HOME을 완료하면 DONE=True를 발행
        arm_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drok_arm_auto/done", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && is_waiting && arm_started) {
                    // ARM: 팔이 HOME까지 복귀한 뒤에만 차체 주행을 다시 허용
                    is_waiting = false;
                    arm_started = false;
                    arm_mission_completed = true;

                    RCLCPP_INFO(
                        this->get_logger(),
                        "[ARM] GRASP DONE + HOME confirmed. Chassis driving resumed.");
                }
            });
    }

private:
    // ARM: supply_box가 여러 프레임 연속 검출되어도 한 번만 팔 미션을 시작
    // ARM: 현재 대회 흐름에서는 supply_box 파지 성공 후 물체를 계속 잡고 있다고 가정하므로
    // ARM: arm_mission_completed=true 이후에는 같은 Summer 실행 중 재파지를 하지 않음
    void start_arm_sequence_if_needed() {
        if (is_waiting || arm_started || arm_mission_completed) {
            return;
        }

        is_waiting = true;
        arm_started = false;
        wait_start_time = this->now();

        RCLCPP_INFO(
            this->get_logger(),
            "[ARM] supply_box detected. Chassis STOP interlock active.");
    }

    void process_front(const sensor_msgs::msg::Image::SharedPtr mf) {
        try {
            cv::Mat img_f = cv_bridge::toCvCopy(mf, "bgr8")->image;
            cv::Size target_size(640, 480);
            cv::resize(img_f, img_f, target_size);

            auto out_msg = geometry_msgs::msg::Twist();

            // 1. 신호등 및 보급상자 제어 로직
            if (is_red_light) {
                out_msg.linear.x = 0.0; 
                out_msg.angular.z = 0.0;
            } 
            // ARM: 기존 20초 대기 대신 팔 DONE=True가 올 때까지 차체를 계속 정지
            else if (is_waiting) {
                out_msg.linear.x = 0.0;
                out_msg.angular.z = 0.0;
            } 
            else {
                out_msg = current_nav;
            }

            // ARM: supply_box 감지 직후 차체에 STOP 명령을 먼저 준 뒤 0.5초 정착시간을 확보
            // ARM: enable=True는 한 번만 발행하며 이후 DONE=True가 올 때까지 /cmd_vel=0 유지
            if (is_waiting && !arm_started) {
                double elapsed = (this->now() - wait_start_time).seconds();

                if (elapsed >= ARM_CHASSIS_SETTLE_SEC) {
                    auto arm_enable_msg = std_msgs::msg::Bool();
                    arm_enable_msg.data = true;
                    arm_enable_pub_->publish(arm_enable_msg);
                    arm_started = true;

                    RCLCPP_INFO(
                        this->get_logger(),
                        "[ARM] Chassis settled. /drok_arm_auto/enable=True published.");
                }
            }
            
            // 2. IMU linear x가 0.1보다 작으면 토크(속도) 2배 증가 (정지 상태가 아닐 때만 적용)
            if (out_msg.linear.x != 0.0 && std::abs(current_imu_linear_x) < 0.1) {
                out_msg.linear.x *= 2.0;
            }

            cmd_pub_->publish(out_msg);

            // 시각화
            cv::Mat ui_view; 
            cv::resize(img_f, ui_view, cv::Size(), 0.5, 0.5);
            auto msg = cv_bridge::CvImage(mf->header, "bgr8", ui_view).toImageMsg();
            ui_image_pub_->publish(*msg);
            
            cv::imshow("Summer_Front_Vision", ui_view); 
            cv::waitKey(1);

        } catch (cv::Exception& e) { 
            RCLCPP_ERROR(this->get_logger(), "OpenCV Error: %s", e.what()); 
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // IMU 구독자 추가
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;

    // ARM: q.py supply_box 감지 토픽 구독
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_yolo_sub_;
    // ARM: 팔 시작 명령 publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_enable_pub_;
    // ARM: 팔 grasp/HOME 완료 subscriber
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_done_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ui_image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    geometry_msgs::msg::Twist current_nav;
    double current_imu_linear_x = 0.0; // IMU 가속도 저장 변수
    bool is_red_light = false;
    bool is_waiting = false;
    rclcpp::Time wait_start_time;

    // ARM: 차체가 실제로 정지할 시간을 확보한 뒤에만 팔을 움직임
    static constexpr double ARM_CHASSIS_SETTLE_SEC = 0.5;
    // ARM: enable=True 중복 발행 방지
    bool arm_started = false;
    // ARM: 이번 Summer 실행에서 보급상자 파지 완료 후 재트리거 방지
    bool arm_mission_completed = false;
};

int main(int argc, char **argv) { 
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<SummerNode>()); 
    rclcpp::shutdown(); 
    return 0; 
}
