# DROK_ARM_IK

DROK ARM 실제 로봇용 **고정 연습위치 IK Grasp** 코드입니다.

현재 기본 모드는 **고정 연습위치 사용**입니다.

- YOLO 물체 XYZ / Camera TF 좌표는 사용하지 않음
- `/drok_arm_auto/enable=True` 입력 시 고정된 위치의 물체를 잡음
- 시작 시 `EXACT HOME_Q` 이동
- 그리퍼 `FULL OPEN`
- Top-Down IK 계산
- `APPROACH1 → APPROACH2 → GRASP → LIFT`
- 물체를 잡은 상태로 `HOME` 복귀

---

## 1. 현재 기본 설정

현재 고정 목표 위치는 `ARM_BASE_LINK` 기준입니다.

```text
X = +0.4000 m
Y = +0.0000 m
Z = -0.0325 m
```

현재 코드 설정:

```python
USE_FIXED_PRACTICE_TARGET = True

FIXED_GRASP_X_M = 0.4000
FIXED_GRASP_Y_M = 0.0000
FIXED_GRASP_Z_M = -0.0325

START_ENABLED = False
```

필수 외부 IK Workspace:

```text
https://github.com/jhj0129/IK_solver_MuJoCo
```

---

# 2. 처음 파일을 받은 뒤 압축 풀기

GitHub에서 **Download ZIP**으로 받았다고 가정합니다.

기본 다운로드 파일명:

```text
~/Downloads/DROK_ARM_IK-main.zip
```

### Terminal 1

```bash
cd ~

unzip ~/Downloads/DROK_ARM_IK-main.zip

mv ~/DROK_ARM_IK-main ~/DROK_ARM_IK

cd ~/DROK_ARM_IK
```

압축이 정상적으로 풀렸는지 확인:

```bash
ls ~/DROK_ARM_IK
```

정상적인 주요 항목:

```text
README.md
src
tools
```

이 저장소는 **빌드 전 소스만 포함**합니다.

따라서 아래 폴더는 GitHub에 포함하지 않습니다.

```text
build/
install/
log/
```

---

# 3. IK_solver_MuJoCo 준비

이 프로젝트는 IK 계산을 위해 별도의 `IK_solver_MuJoCo` Workspace를 사용합니다.

기본 위치:

```text
~/IK_solver_MuJoCo
```

아직 없다면:

### Terminal 1

```bash
cd ~

git clone https://github.com/jhj0129/IK_solver_MuJoCo.git
```

IK Workspace가 다른 위치에 있다면:

```bash
export DROK_IK_ROOT=/원하는/경로/IK_solver_MuJoCo
```

예:

```bash
export DROK_IK_ROOT=/home/robot/IK_solver_MuJoCo
```

---

# 4. 처음 한 번 빌드

새 컴퓨터에서 처음 사용할 때 한 번 실행합니다.

### Terminal 1

```bash
cd ~/DROK_ARM_IK

chmod +x tools/*.sh
chmod +x tools/*.py

bash tools/first_setup.sh
```

`first_setup.sh`는 다음 작업을 수행합니다.

```text
ROS 2 Humble 확인
→ IK_solver_MuJoCo 확인
→ 필요 시 IK Workspace 빌드
→ DROK_ARM_IK Workspace 빌드
```

직접 빌드하려면:

```bash
cd ~/DROK_ARM_IK

source /opt/ros/humble/setup.bash
source ~/IK_solver_MuJoCo/install/setup.bash

bash tools/build.sh
```

---

# 5. 새 터미널마다 환경 소싱

빌드가 끝난 뒤 새 터미널을 열 때마다 아래 한 줄을 실행합니다.

```bash
source ~/DROK_ARM_IK/tools/source_env.sh
```

이 스크립트가 자동으로 다음 환경을 source 합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/IK_solver_MuJoCo/install/setup.bash
source ~/DROK_ARM_IK/install/setup.bash
```

---

# 6. 고정 연습위치 물체 잡기

현재 설정에서는 **물체 좌표 토픽이 들어오지 않아도 됩니다.**

IK 모드가 켜지면 아래 위치를 사용합니다.

```text
ARM_BASE_LINK 기준

X = +0.4000 m
Y = +0.0000 m
Z = -0.0325 m
```

## Terminal 1 - 실제 팔 Bridge 실행

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/run_real.sh
```

이 터미널은 계속 켜둡니다.

---

## Terminal 2 - 자동 IK Grasp 노드 실행

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/run_drok_auto_grasp_prototype1.sh
```

정상 시작 로그:

```text
TARGET MODE       : FIXED PRACTICE
Fixed target [ARM_BASE_LINK m]: (+0.4000, +0.0000, -0.0325)
IK mode trigger   : /drok_arm_auto/enable=True
YOLO XYZ / TF     : IGNORED
Auto start enabled: False
```

이 상태에서는 아직 로봇이 잡기 동작을 시작하지 않습니다.

---

## Terminal 3 - IK 모드 ON / 물체 잡기 1회 실행

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/trigger_fixed_ik_grasp.sh
```

이 명령이 실행되면 내부적으로:

```text
/drok_arm_auto/enable = True
```

가 한 번 publish 됩니다.

실제 동작 순서:

```text
IK MODE ON
→ 고정 연습위치 사용
→ EXACT HOME_Q
→ GRIPPER FULL OPEN
→ IK 계산
→ PREALIGN
→ APPROACH1
→ APPROACH2
→ GRASP / CLOSE
→ LIFT
→ HOME
```

---

# 7. 물체 위치 수정

수정 파일:

```text
~/DROK_ARM_IK/tools/drok_auto_grasp_prototype1.py
```

파일 위쪽의 다음 부분을 수정합니다.

```python
USE_FIXED_PRACTICE_TARGET = True

FIXED_GRASP_X_M = 0.4000
FIXED_GRASP_Y_M = 0.0000
FIXED_GRASP_Z_M = -0.0325
```

좌표계:

```text
+X = 로봇 전방
+Y = 로봇 왼쪽
+Z = 위쪽
```

## 예시 1 - 물체를 5 cm 더 앞으로

기존:

```python
FIXED_GRASP_X_M = 0.4000
```

수정:

```python
FIXED_GRASP_X_M = 0.4500
```

---

## 예시 2 - 물체를 현재보다 10 cm 더 높게

기존:

```python
FIXED_GRASP_Z_M = -0.0325
```

10 cm 위로 이동:

```python
FIXED_GRASP_Z_M = 0.0675
```

---

## 예시 3 - 물체를 오른쪽 5 cm로 이동

`+Y`가 왼쪽이므로 오른쪽은 `-Y`입니다.

```python
FIXED_GRASP_Y_M = -0.0500
```

Python 설정값만 수정한 경우 **colcon 재빌드는 필요 없습니다.**

자동 IK 노드를 종료한 뒤 다시 실행하면 새 값이 적용됩니다.

---

# 8. 오프셋 수정

수정 파일:

```text
~/DROK_ARM_IK/tools/drok_auto_grasp_prototype1.py
```

설정 위치:

```python
ROBOT_OFFSET_FORWARD_CM = 0.0
ROBOT_OFFSET_RIGHT_CM = 0.0
ROBOT_OFFSET_UP_CM = 0.0
```

오프셋 방향:

```text
FORWARD +  → 최종 X 증가
RIGHT   +  → 최종 Y 감소
UP      +  → 최종 Z 증가
```

현재 버전에서는 오프셋이:

```text
고정 연습위치 모드
카메라 / YOLO 모드
```

둘 다에 적용됩니다.

## 예시 - 2 cm 앞으로 보정

```python
ROBOT_OFFSET_FORWARD_CM = 2.0
```

## 예시 - 오른쪽으로 3 cm 보정

```python
ROBOT_OFFSET_RIGHT_CM = 3.0
```

## 예시 - 위쪽으로 1.5 cm 보정

```python
ROBOT_OFFSET_UP_CM = 1.5
```

최종 목표 위치는:

```text
고정 물체 위치 + 오프셋
```

으로 계산됩니다.

---

# 9. 그리퍼 FULL OPEN 위치로 자동 이동

먼저 **Terminal 1에서 실제 팔 Bridge가 실행 중이어야 합니다.**

### Terminal 2

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/gripper_open.sh
```

현재 저장된 `FULL OPEN` 위치로 이동합니다.

현재 기본 저장값:

```text
OPEN GAP      = 14.60 cm
OPEN PROTOCOL = -1640.890 deg
```

---

# 10. 그리퍼 GRASP 위치로 자동 이동

먼저 **Terminal 1에서 실제 팔 Bridge가 실행 중이어야 합니다.**

### Terminal 2

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/gripper_grasp.sh
```

현재 저장된 `GRASP / CLOSE` 위치로 이동합니다.

현재 기본 저장값:

```text
GRASP GAP      = 9.70 cm
GRASP PROTOCOL = -545.910 deg
```

---

# 11. 현재 그리퍼 OPEN / GRASP 저장값 확인

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

python3 ~/DROK_ARM_IK/tools/drok_gripper_preset.py status
```

출력 예:

```text
OPEN : 14.60 cm
GRASP: 9.70 cm
```

---

# 12. 그리퍼 OPEN / GRASP 위치 새로 보정하기

먼저 실제 팔 Bridge를 실행합니다.

## Terminal 1

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/run_real.sh
```

## Terminal 2

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

python3 ~/DROK_ARM_IK/tools/drok_gripper_opening_calibration_jog.py
```

보정 프로그램에서 사용할 수 있는 주요 명령:

```text
-1
```

현재 feedback 기준으로 OPEN 방향으로 1 topic-degree 이동.

```text
+1
```

현재 feedback 기준으로 CLOSE 방향으로 1 topic-degree 이동.

```text
s
```

현재 feedback 확인.

```text
saved
```

현재 코드에 저장된 OPEN / GRASP 값 확인.

---

## FULL OPEN 위치 저장

실제 그리퍼를 원하는 완전 열림 위치까지 이동시킨 뒤:

```text
gripper> save open
```

실측 간격까지 같이 저장하려면:

```text
gripper> save open 14.6
```

---

## GRASP 위치 저장

실제 물체를 잡는 위치까지 이동시킨 뒤:

```text
gripper> save grasp
```

실측 간격까지 같이 저장하려면:

```text
gripper> save grasp 9.7
```

저장하면 아래 파일이 자동으로 수정됩니다.

```text
~/DROK_ARM_IK/tools/interactive_box_ik_grasp_v11.py
```

수정되는 값:

```python
GRIPPER_OPEN_GAP_CM
GRIPPER_OPEN_PROTOCOL_DEG

GRIPPER_CLOSE_GAP_CM
GRIPPER_CLOSE_PROTOCOL_DEG
```

저장 전에 자동 Backup도 생성됩니다.

기존 저장값과 현재 feedback 차이가 매우 큰 경우:

```text
SAVE
```

를 다시 입력하도록 요구할 수 있습니다.

실제 그리퍼 위치를 직접 확인한 경우에만 저장하십시오.

---

# 13. 고정 연습위치 모드 / 카메라 모드 전환

수정 파일:

```text
~/DROK_ARM_IK/tools/drok_auto_grasp_prototype1.py
```

현재 설정:

```python
USE_FIXED_PRACTICE_TARGET = True
```

이 상태에서는:

```text
YOLO XYZ 무시
Camera TF 무시
IK 모드 ON → 고정 연습위치 사용
```

추후 실제 카메라 좌표를 사용하려면:

```python
USE_FIXED_PRACTICE_TARGET = False
```

로 변경합니다.

---

# 14. 주요 파일

```text
DROK_ARM_IK/
├── README.md
│
├── src/
│   └── drok_real_arm_bridge/
│
└── tools/
    ├── interactive_box_ik_grasp_v11.py
    ├── drok_auto_grasp_prototype1.py
    ├── drok_manual_box_grasp_practice.py
    ├── drok_gripper_opening_calibration_jog.py
    ├── drok_gripper_preset.py
    ├── gripper_open.sh
    ├── gripper_grasp.sh
    ├── run_real.sh
    ├── run_drok_auto_grasp_prototype1.sh
    ├── trigger_fixed_ik_grasp.sh
    ├── first_setup.sh
    └── source_env.sh
```

---

# 15. GitHub에 올리지 않는 파일

아래 항목은 빌드 생성물이므로 저장소에 올리지 않습니다.

```text
build/
install/
log/
__pycache__/
*.pyc
```

`.gitignore`에 포함되어 있습니다.

---

# 16. 안전 관련

이 Workspace의 실행 및 설치 스크립트는 다음 설정을 변경하지 않습니다.

```text
CAN interface UP / DOWN
CAN bitrate
Motor ROM
Motor limit
```

실제 CAN 장치는 프로그램 실행 전에 시스템에서 이미 정상적으로 준비되어 있어야 합니다.

---

# 17. 현재 확인된 실제 로봇 실행 상태

실제 로봇에서 아래 과정까지 동작을 확인했습니다.

```text
EXACT HOME
→ FULL OPEN
→ TOP-DOWN IK
→ APPROACH1
→ APPROACH2
→ GRASP / CLOSE
→ LIFT
```

최근 테스트에서는 마지막 `LIFT → HOME` 과정에서 `JOINT6` 오차가 약 `1.37°` 남아 현재 도착 허용오차 `1.0°`를 넘으면서 timeout이 발생한 적이 있습니다.

따라서 실제 장비에서는 마지막 HOME 복귀 완료 status까지 확인하십시오.
