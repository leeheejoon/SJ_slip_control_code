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

초기 파지 힘 범위는 영점 보정된 센서값 기준 검지 100–160, 나머지 네 손가락 80–140입니다. 한 번의 추가 조임은 0.01–0.03 rad 범위에서 계산하며 관절각 상한을 적용합니다. 기본 설정에서 누적 추가 조임량 제한은 꺼져 있습니다. 명령 노드를 종료해도 제어기가 마지막 목표를 계속 유지할 수 있습니다. 실험 종료 시 장비의 정지 절차를 사용합니다.


## 현재 초기 파지 조건

초기 힘 제어는 접근 중 0.5초마다 각 손가락에 적용합니다. 전체 정지 확정 후에는 1초 주기로 복귀합니다.

| 보정된 힘값 | 목표각 변화 |
|---|---|
| 60 미만 | +0.015 rad |
| 60 이상, 80 미만 | +0.001 rad |
| 80~140 | 현재각 유지, 다음 주기에 재판정 |
| 140 초과, 160 이하 | -0.001 rad |
| 160 초과 | -0.015 rad |

검지는 목표 힘 100~160이며 90~100/160~170에서 미세 보정합니다.
`index_force_min`, `index_force_max`, `index_force_margin` 파라미터로 설정할 수 있습니다.
다른 네 손가락은 표의 범위를 적용합니다. 같은 판단 시점에 각자의 목표 범위에 모두 들어오면 전체 정지를 확정합니다.
이후 힘 범위를 재확인하지 않고 3초간 각도를 유지한 뒤 촉각 기준을 수집합니다.
촉각 변화 조건은 새 프레임 2개 이상에서 0.15초 지속, 추가 조임 후 대기는 0.20초입니다.
센서값 변화에 따른 추정이므로 로그 문구만으로 실제 미끄러짐이나 억제 성공을 확정하지 않습니다.

촉각 기본 영역은 `index_pad,middle_pad,thumb_distal`입니다. 다른 영역의 `미수신`은
기본 수집 대상에서 제외된 결과입니다. `tactile_regions:=all`로 전체를 선택할 수 있으나,
각 영역 수신 간격이 길어질 수 있습니다. 무접촉 보정 대상은 명령 노드의
`tactile_calibration_regions` 파라미터로 지정합니다.

실험 원본과 개별 조임 방식의 소스 백업은 로컬 워크스페이스에 별도로 보관합니다.


검지 범위만 바꾸려면 명령 노드 실행 시 다음 파라미터를 사용합니다.

```bash
ros2 run rh56e2_control hand_command_node --ros-args \
  -p index_force_min:=100.0 \
  -p index_force_max:=160.0 \
  -p index_force_margin:=10.0
```

검지는 힘 90 미만에서 +0.015rad, 90~100에서 +0.001rad,
100~160에서 현재각 유지, 160 초과~170에서 -0.001rad,
170 초과에서 -0.015rad로 보정합니다. 다른 손가락은 위 표를 사용합니다.
엄지 굴곡만 먼저 고정하는 조건은 없으며, 모두 각자의 목표 범위에 들어오면
전체 정지를 확정합니다. 엄지 회전 1.0rad 고정은 유지합니다.
