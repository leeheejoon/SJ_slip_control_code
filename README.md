# RH56E2 T1 촉각 미끄러짐 제어 테스트

Inspire RH56E2 T1 손의 촉각 배열을 이용해 파지 중 접촉 변화를 감지하고 손가락을 추가로 조입니다. 이 절차에는 촉각 센서가 달린 RH56E2 T1, USB–RS485 어댑터, 손 전원, 가벼운 테스트 물체와 받침대가 필요합니다.

## 다운로드와 빌드

Ubuntu 24.04에서 ROS 2 Jazzy, `colcon`, `rosdep`을 준비합니다. 처음 사용하는 환경이라면 `sudo rosdep init`과 `rosdep update`로 rosdep을 초기화합니다.

```bash
git clone https://github.com/leeheejoon/SJ_slip_control_code.git rh56e2_slip_ws
cd rh56e2_slip_ws
source /opt/ros/jazzy/setup.bash
sudo apt install libeigen3-dev libmodbus-dev

mapfile -t package_paths < <(colcon list --paths-only --packages-up-to \
  rh56e2_control inspire_description modbus_ros2_control basic_joint_controller)
rosdep install --from-paths "${package_paths[@]}" --ignore-src \
  --rosdistro jazzy -t build -t buildtool -t build_export -t buildtool_export -t exec \
  --skip-keys lina_planning -y

colcon build --packages-up-to \
  rh56e2_control inspire_description modbus_ros2_control basic_joint_controller \
  --cmake-args -DBUILD_TESTING=OFF
```

## 장비에서 실행

터미널 1에서 손을 연결합니다. `/dev/ttyUSB0`는 사용하는 장치 경로로 바꿉니다. 왼손이면 런치 명령에 `direction:=1`을 추가합니다. 기본값은 오른손입니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rh56e2_control real.launch.py serial_port:=/dev/ttyUSB0
```

`FSM initialization complete.`가 출력된 후, 물체와 손이 닿지 않은 상태에서 터미널 2를 엽니다. 두 터미널 모두 워크스페이스 루트에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rh56e2_control hand_command_node
```

1. `촉각 무접촉 보정 완료`와 `손 힘 영점 계산 완료` 로그가 나온 뒤, 가벼운 물체를 받침대 위에 놓고 파지 위치에 맞춥니다. 손가락 작동 범위에서 손을 빼고 진행합니다. 명령 노드는 보정 뒤 자동으로 파지를 시작합니다.
2. `초기 파지 완료` 후 접촉한 영역의 `촉각 기준 준비` 로그를 확인합니다. 물체가 떨어져도 손상되지 않도록 아래에 받칠 공간을 두고 받침대를 낮춥니다.
3. `미끄러짐 발생 → 추가 조임` 로그의 감지 영역, 조임량과 목표각을 확인합니다. 물체가 실제로 내려갔는지, 조임 뒤 멈췄는지, 완전히 이탈했는지 따로 기록합니다.
4. 비교 실험에서는 명령 노드에 `--ros-args -p slip_all_fingers:=false`를 추가해 감지한 손가락만 추가로 조입니다. 기본 모드에서는 엄지 굴곡을 포함한 다섯 손가락을 함께 조입니다.

초기 파지 힘 범위는 영점 보정된 센서값 120–180입니다. 한 번의 추가 조임은 0.01–0.03 rad 범위에서 계산하며 관절각 상한을 적용합니다. 기본 설정에서 누적 추가 조임량 제한은 꺼져 있습니다. 명령 노드를 종료해도 제어기가 마지막 목표를 계속 유지할 수 있습니다. 실험 종료 시 장비의 정지 절차를 사용합니다.
