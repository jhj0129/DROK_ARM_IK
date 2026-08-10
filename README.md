# DROK_ARM_IK

DROK ARM 실제 로봇용 **독립형 FK/IK + 고정 연습위치 Grasp Workspace**입니다.

## 핵심

이제 별도의 `IK_solver_MuJoCo` Workspace가 필요하지 않습니다.

기존에 사용하던 FK/IK 중 실제 grasp에 필요한 부분만:

```text
src/drok_arm_kinematics/
```

안으로 통합했습니다.

현재 실제 고정위치 grasp에는:

```text
MuJoCo Viewer       필요 없음
MuJoCo Simulation   필요 없음
외부 IK Workspace   필요 없음
```

입니다.

---

# 1. GitHub ZIP 압축 풀기

```bash
cd ~

unzip ~/Downloads/DROK_ARM_IK-main.zip

mv ~/DROK_ARM_IK-main ~/DROK_ARM_IK

cd ~/DROK_ARM_IK
```

---

# 2. 처음 한 번 준비 / 빌드

필요한 C++ library:

```bash
sudo apt update

sudo apt install -y \
  libeigen3-dev \
  libyaml-cpp-dev
```

그다음:

```bash
cd ~/DROK_ARM_IK

chmod +x tools/*.sh
chmod +x tools/*.py

bash tools/first_setup.sh
```

한 Workspace에서 다음 두 package가 같이 빌드됩니다.

```text
drok_arm_kinematics
drok_real_arm_bridge
```

---

# 3. 새 터미널 소싱

```bash
source ~/DROK_ARM_IK/tools/source_env.sh
```

외부 `IK_solver_MuJoCo/install/setup.bash`는 더 이상 source하지 않습니다.

---

# 4. 고정 연습위치 물체 잡기

현재 목표:

```text
ARM_BASE_LINK
X = +0.4000 m
Y = +0.0000 m
Z = -0.0325 m
```

## Terminal 1 - 실제 팔

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/run_real.sh
```

## Terminal 2 - 자동 IK 노드

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/run_drok_auto_grasp_prototype1.sh
```

## Terminal 3 - IK 모드 ON

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/trigger_fixed_ik_grasp.sh
```

동작:

```text
EXACT HOME_Q
→ GRIPPER FULL OPEN
→ FK / IK
→ PREALIGN
→ APPROACH1
→ APPROACH2
→ GRASP
→ LIFT
→ HOME
```

---

# 5. 물체 위치 수정

파일:

```text
~/DROK_ARM_IK/tools/drok_auto_grasp_prototype1.py
```

```python
USE_FIXED_PRACTICE_TARGET = True

FIXED_GRASP_X_M = 0.4000
FIXED_GRASP_Y_M = 0.0000
FIXED_GRASP_Z_M = -0.0325
```

좌표:

```text
+X = 전방
+Y = 왼쪽
+Z = 위
```

---

# 6. 오프셋 수정

같은 파일:

```python
ROBOT_OFFSET_FORWARD_CM = 0.0
ROBOT_OFFSET_RIGHT_CM = 0.0
ROBOT_OFFSET_UP_CM = 0.0
```

```text
FORWARD + → X 증가
RIGHT   + → Y 감소
UP      + → Z 증가
```

---

# 7. 그리퍼 FULL OPEN

실제 bridge를 먼저 실행한 상태에서:

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/gripper_open.sh
```

현재:

```text
OPEN = 14.60 cm
protocol = -1640.890 deg
```

---

# 8. 그리퍼 GRASP

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/gripper_grasp.sh
```

현재:

```text
GRASP = 9.70 cm
protocol = -545.910 deg
```

---

# 9. 그리퍼 재보정

Terminal 1:

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

bash ~/DROK_ARM_IK/tools/run_real.sh
```

Terminal 2:

```bash
source ~/DROK_ARM_IK/tools/source_env.sh

python3 \
  ~/DROK_ARM_IK/tools/drok_gripper_opening_calibration_jog.py
```

완전 열림 저장:

```text
gripper> save open
```

잡기 저장:

```text
gripper> save grasp
```

---

# 10. 이제 IK가 저장소 내부에서 사용하는 파일

Geometry:

```text
src/drok_arm_kinematics/config/robot_geometry.yaml
```

Joint limit URDF:

```text
src/drok_arm_kinematics/config/drok_arm_kinematics_only.urdf
```

Nearest IK helper:

```text
tools/baseline_nearest_ik_core.py
```

C++ IK:

```text
src/drok_arm_kinematics/src/solve_ik_pose.cpp
```

C++ FK:

```text
src/drok_arm_kinematics/src/test_fk.cpp
```

따라서 현재 실제 grasp 계산에 필요한 FK/IK가 모두 이 저장소에 포함됩니다.

---

# 11. MuJoCo

현재 고정 연습위치 실제 grasp에는 MuJoCo가 필요하지 않습니다.

기존 MuJoCo simulation / MJCF / scene 파일은 이 저장소에 포함하지 않았습니다.

---

# 12. GitHub에 올리지 않는 파일

```text
build/
install/
log/
__pycache__/
*.pyc
```

---

# 13. 안전

자동으로 다음을 변경하지 않습니다.

```text
CAN interface UP / DOWN
CAN bitrate
Motor ROM
Motor limit
```
