# RH56E2 실제 손 제어

이 패키지는 팔 없이 RH56E2 손을 제어한다.

- `launch/real.launch.py`: 손 하드웨어, 관절 상태 발행기, 손 제어기와 FSM 준비.
- `config/real_hand_controllers.yaml`: 손 관절 6개와 제어기 설정.
- `src/fsm_init_node.cpp`: HOLD → MOVEJ 준비.
- `src/hand_command_node.cpp`: 사용자가 따로 실행하는 손 동작 명령.
- `models/`: 보존한 상자·받침대 모델. 실제 손 런치에서는 생성하지 않는다.

기존 팔+손 시뮬레이션 런치와 팔 제어 노드는 제거했다.
남아 있는 Gazebo 피드백 코드와 파지 로직은 현재 실제 손 런치에서 실행하지 않는다.
명령 노드 종료만으로 제어기의 마지막 목표가 취소되지는 않는다.
