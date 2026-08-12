# DROK_ARM_IK

DROK ARM 실제 로봇용 독립형 FK/IK + 실제 RMD 제어 + TOP-DOWN grasp workspace입니다.

## 현재 기준 상태 (2026-08-12)

- 외부 `IK_solver_MuJoCo` workspace 불필요
- MuJoCo viewer 없이 실제 grasp 실행
- `ARM_BASE_LINK`: +X 전방, +Y 왼쪽, +Z 위
- 연습모드 기본값: `USE_FIXED_PRACTICE_TARGET = True`
- 현재 연습 목표: `(0.400, 0.000, -0.100) m`
- 시작 HOME에서는 JOINT1을 현재 위치에 유지하고 JOINT2~6만 HOME으로 이동
- IK 동작에서는 JOINT1을 정상 사용
- grasp/lift 후 최종 HOME 복귀는 기존 `HOME_Q` 사용
- gripper TCP: `gripper_center`에서 local +X `0.05 m` 보정
- CAN interface state/bitrate 변경 없음
- Motor ROM/limit write 없음

## Repository

```text
DROK_ARM_IK/
├── README.md
├── DROK_ARM_code.txt              # 사용자가 정리한 실행/파라미터 메모
├── src/
│   ├── drok_arm_kinematics/
│   └── drok_real_arm_bridge/
├── tools/
│   ├── go_home.sh
│   ├── run_real.sh
│   ├── drok_auto_grasp_prototype1.py
│   ├── interactive_box_ik_grasp_v11.py
│   ├── drok_gripper_opening_calibration_jog.py
│   └── ...
└── integration/
    └── chassis_supply_box/
        ├── summer_node_ARM_integrated.cpp
        ├── yolo_xyz_publisher_q.py
        ├── DROK_ARM_SUMMER_INTEGRATION_GUIDE.txt
        └── summer_node_ARM.diff
```

`integration/chassis_supply_box/`는 차체 Summer + supply_box 연동용 자료이며 ARM 단독 실행에는 필요하지 않습니다.

## 처음 한 번 빌드

```bash
cd ~/DROK_ARM_IK
source /opt/ros/humble/setup.bash
chmod +x tools/*.sh tools/*.py
bash tools/first_setup.sh
```

GitHub ZIP으로 받은 폴더명이 `DROK_ARM_IK-main`이면 위 경로의 `DROK_ARM_IK`를 `DROK_ARM_IK-main`으로 바꾸면 됩니다.

## 기본 점검 / 실행

### 1. 모터 통신

```bash
bash ~/DROK_ARM_IK-main/tools/run_real.sh
```

### 2. HOME — JOINT1 HOLD

전원 ON 후 JOINT1을 사용자가 물리적 정면에 맞춘 뒤:

```bash
bash ~/DROK_ARM_IK-main/tools/go_home.sh
```

### 3. 연습 grasp 노드

```bash
source ~/DROK_ARM_IK-main/tools/source_env.sh
bash ~/DROK_ARM_IK-main/tools/run_drok_auto_grasp_prototype1.sh
```

### 4. 연습 grasp 시작

```bash
source ~/DROK_ARM_IK-main/tools/source_env.sh
bash ~/DROK_ARM_IK-main/tools/trigger_fixed_ik_grasp.sh
```

### 5. 그리퍼 재보정

```bash
source ~/DROK_ARM_IK-main/tools/source_env.sh
python3 ~/DROK_ARM_IK-main/tools/drok_gripper_opening_calibration_jog.py
```

## 주요 파라미터

### `tools/drok_auto_grasp_prototype1.py`

```python
USE_FIXED_PRACTICE_TARGET = True   # True=연습, False=YOLO+TF
FIXED_GRASP_X_M = 0.4000
FIXED_GRASP_Y_M = 0.0000
FIXED_GRASP_Z_M = -0.1000

ROBOT_OFFSET_FORWARD_CM = 0.0
ROBOT_OFFSET_RIGHT_CM = 0.0
ROBOT_OFFSET_UP_CM = 0.0

START_HOME_SEC = 3.0*2
RETURN_HOME_SEC = 3.0*2
```

### `tools/interactive_box_ik_grasp_v11.py`

```python
NEAR_STANDOFF_M = 0.09
LIFT_HEIGHT_M = 0.05
REAL_CURRENT_TO_PREALIGN_SEC = 1.2
REAL_APPROACH1_SEC = 6.0*2
REAL_APPROACH2_SEC = 3.0*2
REAL_GRASP_TO_LIFT_SEC = 3.0*2
REAL_GRIPPER_CLOSE_SEC = 3.0*2
```

현재 업로드본에 저장된 gripper calibration:

```text
OPEN  : gap=14.6 cm, protocol=+105.110 deg
GRASP : gap=9.7 cm,  protocol=+1172.960 deg
```

### TCP 파지점 보정

`src/drok_arm_kinematics/config/drok_arm_kinematics_only.urdf`

```xml
<origin xyz="0.05 0 0" rpy="0 0 0" />
```

`src/drok_arm_kinematics/config/robot_geometry.yaml`에도 동일한 `0.05 m`가 적용되어 있습니다.

## CAMERA / YOLO 모드

`tools/drok_auto_grasp_prototype1.py`:

```python
USE_FIXED_PRACTICE_TARGET = False
```

입력 토픽:

```text
/yolo_detected_object   std_msgs/String
/yolo_object_xyz        geometry_msgs/Vector3Stamped
```

YOLO XYZ는 `camera_link` 기준이고 ARM 노드가 TF를 이용해 `ARM_BASE_LINK`로 변환합니다.

## GitHub에 올리지 않는 파일

```text
build/
install/
log/
__pycache__/
*.pyc
*.pyo
*.bak_*
```

## 안전

이 저장소의 실행/보정 흐름에서 자동으로 다음을 변경하지 않습니다.

```text
CAN interface UP/DOWN
CAN bitrate
Motor ROM
Motor limit
```
