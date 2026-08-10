DROK_ARM_IK
===========

DROK ARM 실제 로봇용 고정 연습위치 IK grasp 코드입니다.

현재 기본 모드
--------------
- 고정 연습위치 사용
- YOLO 물체 XYZ / camera TF 좌표는 사용하지 않음
- `/drok_arm_auto/enable=True`가 들어오면 고정된 위치의 물체를 잡음
- 시작 시 EXACT HOME_Q 이동
- 그리퍼 FULL OPEN
- TOP-DOWN IK
- APPROACH1 -> APPROACH2 -> GRASP -> LIFT
- 물체를 잡은 상태로 HOME 복귀

현재 고정 목표:
  X = +0.4000 m
  Y = +0.0000 m
  Z = -0.0325 m

필수 외부 IK workspace:
  https://github.com/jhj0129/IK_solver_MuJoCo

============================================================
1. 처음 GitHub 파일을 받은 뒤 압축 푸는 방법
============================================================

GitHub에서 "Download ZIP"으로 받았다고 가정합니다.

기본 다운로드 파일명:
  ~/Downloads/DROK_ARM_IK-main.zip

Terminal:

  cd ~

  unzip ~/Downloads/DROK_ARM_IK-main.zip

  mv ~/DROK_ARM_IK-main ~/DROK_ARM_IK

  cd ~/DROK_ARM_IK

이 저장소는 "빌드 전 소스"만 포함합니다.
즉 build/, install/, log/는 GitHub에 올리지 않습니다.

압축을 푼 뒤 확인:

  ls ~/DROK_ARM_IK

정상적인 주요 폴더:
  src
  tools

============================================================
2. IK_solver_MuJoCo 준비
============================================================

이 프로젝트는 IK 계산을 위해 별도의 IK_solver_MuJoCo workspace를 사용합니다.

기본 위치:
  ~/IK_solver_MuJoCo

아직 없다면:

  cd ~
  git clone https://github.com/jhj0129/IK_solver_MuJoCo.git

이미 다른 위치에 있다면:

  export DROK_IK_ROOT=/원하는/경로/IK_solver_MuJoCo

============================================================
3. 처음 한 번 빌드
============================================================

Terminal:

  cd ~/DROK_ARM_IK

  chmod +x tools/*.sh
  chmod +x tools/*.py

  bash tools/first_setup.sh

first_setup.sh는:
  - ROS 2 Humble 확인
  - IK_solver_MuJoCo 확인
  - 필요 시 IK workspace 빌드
  - DROK_ARM_IK의 drok_real_arm_bridge 빌드
를 수행합니다.

직접 빌드하려면:

  cd ~/DROK_ARM_IK

  source /opt/ros/humble/setup.bash
  source ~/IK_solver_MuJoCo/install/setup.bash

  bash tools/build.sh

============================================================
4. 이후 새 터미널에서 환경 소싱
============================================================

각 새 터미널에서:

  source ~/DROK_ARM_IK/tools/source_env.sh

이 한 줄이 기본적으로:
  /opt/ros/humble/setup.bash
  ~/IK_solver_MuJoCo/install/setup.bash
  ~/DROK_ARM_IK/install/setup.bash
를 source합니다.

============================================================
5. 고정 연습위치 물체 잡기
============================================================

현재 설정에서는 물체 좌표 토픽을 받지 않아도 됩니다.

고정 연습위치:
  ARM_BASE_LINK 기준
  (+0.4000, +0.0000, -0.0325) m

----------------------------
Terminal 1 - 실제 팔 bridge
----------------------------

  source ~/DROK_ARM_IK/tools/source_env.sh

  bash ~/DROK_ARM_IK/tools/run_real.sh

이 터미널은 계속 켜둡니다.

---------------------------------
Terminal 2 - 자동 IK grasp 노드
---------------------------------

  source ~/DROK_ARM_IK/tools/source_env.sh

  bash ~/DROK_ARM_IK/tools/run_drok_auto_grasp_prototype1.sh

정상 시작 로그 예:

  TARGET MODE       : FIXED PRACTICE
  Fixed target [ARM_BASE_LINK m]: (+0.4000, +0.0000, -0.0325)
  IK mode trigger   : /drok_arm_auto/enable=True
  YOLO XYZ / TF     : IGNORED
  Auto start enabled: False

--------------------------------
Terminal 3 - IK 모드 1회 실행
--------------------------------

  source ~/DROK_ARM_IK/tools/source_env.sh

  bash ~/DROK_ARM_IK/tools/trigger_fixed_ik_grasp.sh

이 명령은 내부적으로 다음 토픽을 한 번 보냅니다:

  /drok_arm_auto/enable = True

동작 순서:

  IK MODE ON
  -> 고정 연습위치
  -> EXACT HOME_Q
  -> GRIPPER FULL OPEN
  -> IK 계산
  -> PREALIGN
  -> APPROACH1
  -> APPROACH2
  -> GRASP/CLOSE
  -> LIFT
  -> HOME

============================================================
6. 고정 물체 위치 수정
============================================================

파일:
  ~/DROK_ARM_IK/tools/drok_auto_grasp_prototype1.py

파일 위쪽 USER CONFIGURATION 부분:

  USE_FIXED_PRACTICE_TARGET = True

  FIXED_GRASP_X_M = 0.4000
  FIXED_GRASP_Y_M = 0.0000
  FIXED_GRASP_Z_M = -0.0325

좌표계:
  +X = 로봇 전방
  +Y = 로봇 왼쪽
  +Z = 위쪽

예시 1:
물체를 5 cm 더 앞으로 설정:

  FIXED_GRASP_X_M = 0.4500

예시 2:
물체를 현재보다 10 cm 더 높게 설정:

  기존 Z = -0.0325
  +0.10 m

  FIXED_GRASP_Z_M = 0.0675

예시 3:
물체를 오른쪽 5 cm 위치로 설정:

  +Y가 왼쪽이므로 오른쪽은 음수.

  FIXED_GRASP_Y_M = -0.0500

수정 후 Python 파일만 바뀐 경우 colcon 재빌드는 필요 없습니다.
자동 IK 노드를 종료한 뒤 다시 실행하면 새 값이 적용됩니다.

============================================================
7. 오프셋 수정
============================================================

같은 파일:
  ~/DROK_ARM_IK/tools/drok_auto_grasp_prototype1.py

설정 위치:

  ROBOT_OFFSET_FORWARD_CM = 0.0
  ROBOT_OFFSET_RIGHT_CM = 0.0
  ROBOT_OFFSET_UP_CM = 0.0

이번 GitHub 버전에서는 이 오프셋을:
  - 고정 연습위치 모드
  - 추후 CAMERA/YOLO 모드
둘 다에 적용하도록 구성했습니다.

방향:
  FORWARD +  -> 최종 X 증가
  RIGHT   +  -> 최종 Y 감소
  UP      +  -> 최종 Z 증가

예시:
고정 좌표는 그대로 두고 실제 grasp 위치만 2 cm 앞으로 미세 조정:

  ROBOT_OFFSET_FORWARD_CM = 2.0

오른쪽으로 3 cm 보정:

  ROBOT_OFFSET_RIGHT_CM = 3.0

위쪽으로 1.5 cm 보정:

  ROBOT_OFFSET_UP_CM = 1.5

최종 목표는:

  fixed target + offset

형태로 계산됩니다.

============================================================
8. 그리퍼 저장된 OPEN / GRASP 위치로 자동 이동
============================================================

먼저 Terminal 1의 실제 팔 bridge가 켜져 있어야 합니다.

FULL OPEN으로 바로 이동:

  source ~/DROK_ARM_IK/tools/source_env.sh

  bash ~/DROK_ARM_IK/tools/gripper_open.sh

GRASP/CLOSE 저장 위치로 바로 이동:

  source ~/DROK_ARM_IK/tools/source_env.sh

  bash ~/DROK_ARM_IK/tools/gripper_grasp.sh

현재 저장값만 확인:

  source ~/DROK_ARM_IK/tools/source_env.sh

  python3 ~/DROK_ARM_IK/tools/drok_gripper_preset.py status

현재 기본 저장값:

  OPEN
    gap      = 14.60 cm
    protocol = -1640.890 deg

  GRASP
    gap      = 9.70 cm
    protocol = -545.910 deg

============================================================
9. 그리퍼 OPEN / GRASP 위치를 새로 보정해서 저장
============================================================

실제 팔 bridge를 먼저 실행:

Terminal 1:

  source ~/DROK_ARM_IK/tools/source_env.sh

  bash ~/DROK_ARM_IK/tools/run_real.sh

Terminal 2:

  source ~/DROK_ARM_IK/tools/source_env.sh

  python3 ~/DROK_ARM_IK/tools/drok_gripper_opening_calibration_jog.py

보정 프로그램 안의 명령:

  -1
    현재 feedback 기준 OPEN 방향으로 1 topic-degree 이동

  +1
    현재 feedback 기준 CLOSE 방향으로 1 topic-degree 이동

  s
    현재 feedback 확인

  saved
    현재 코드에 저장된 OPEN / GRASP 값 확인

실제 FULL OPEN 위치까지 맞춘 후:

  gripper> save open

실제 gap까지 같이 저장하려면 예:

  gripper> save open 14.6

실제 물체를 잡는 GRASP 위치까지 맞춘 후:

  gripper> save grasp

실제 gap까지 같이 저장하려면 예:

  gripper> save grasp 9.7

저장 시:
  ~/DROK_ARM_IK/tools/interactive_box_ik_grasp_v11.py

안의 다음 값이 자동으로 수정됩니다:

  GRIPPER_OPEN_GAP_CM
  GRIPPER_OPEN_PROTOCOL_DEG

  GRIPPER_CLOSE_GAP_CM
  GRIPPER_CLOSE_PROTOCOL_DEG

저장 전 자동 backup도 생성됩니다.

주의:
기존 저장값과 현재 feedback이 크게 다르면 프로그램이
정확히 "SAVE"를 다시 입력하도록 요구합니다.
실제 그리퍼 위치를 눈으로 확인한 경우에만 저장합니다.

============================================================
10. 고정모드 / 카메라모드 전환
============================================================

파일:
  ~/DROK_ARM_IK/tools/drok_auto_grasp_prototype1.py

현재:

  USE_FIXED_PRACTICE_TARGET = True

따라서:
  YOLO XYZ 무시
  TF 무시
  IK 모드 ON 시 고정위치 사용

추후 실제 카메라 좌표를 쓰려면:

  USE_FIXED_PRACTICE_TARGET = False

로 변경하면 기존 YOLO + TF 경로를 사용할 수 있습니다.

============================================================
11. 주요 파일
============================================================

  src/drok_real_arm_bridge/
    실제 RMD motor bridge 및 real_mapping.yaml

  tools/interactive_box_ik_grasp_v11.py
    IK / FK / TOP-DOWN 경로 / 실제 RMD / gripper 핵심 코드

  tools/drok_auto_grasp_prototype1.py
    자동 IK 모드 및 고정/카메라 목표 선택

  tools/drok_manual_box_grasp_practice.py
    수동 연습용

  tools/drok_gripper_opening_calibration_jog.py
    그리퍼 OPEN / GRASP 위치 보정 및 저장

  tools/drok_gripper_preset.py
    저장된 OPEN / GRASP 위치로 즉시 이동

  tools/gripper_open.sh
    저장된 OPEN 위치로 즉시 이동

  tools/gripper_grasp.sh
    저장된 GRASP 위치로 즉시 이동

  tools/run_real.sh
    실제 팔 bridge 실행

  tools/run_drok_auto_grasp_prototype1.sh
    자동 IK 노드 실행

  tools/trigger_fixed_ik_grasp.sh
    IK 모드 ON / 고정위치 grasp 1회 실행

  tools/first_setup.sh
    새 컴퓨터 첫 빌드

  tools/source_env.sh
    ROS + IK + workspace 환경 소싱

============================================================
12. GitHub에 올리지 않는 파일
============================================================

다음은 빌드 생성물이므로 저장소에 올리지 않습니다:

  build/
  install/
  log/
  __pycache__/
  *.pyc

.gitignore에 포함되어 있습니다.

============================================================
13. 안전 관련
============================================================

이 workspace의 실행/설치 스크립트는:
  - CAN interface up/down 상태를 변경하지 않습니다.
  - CAN bitrate를 변경하지 않습니다.
  - 모터 ROM/limit을 쓰지 않습니다.

실제 CAN 장치는 실행 전에 시스템에서 이미 정상 준비되어 있어야 합니다.

============================================================
14. 현재 확인된 실행 상태
============================================================

실제 로봇에서 다음 과정까지 확인했습니다:

  EXACT HOME
  FULL OPEN
  TOP-DOWN IK
  APPROACH1
  APPROACH2
  GRASP/CLOSE
  LIFT

최근 테스트에서는 마지막 LIFT -> HOME에서 JOINT6 오차가 약 1.37 deg
남아 현재 arrival tolerance 1.0 deg를 넘으면서 timeout이 발생한 적이 있습니다.

따라서 실제 장비에서 HOME 복귀 완료 status까지 반드시 확인하십시오.
