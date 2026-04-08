# 📜 Biped Bike Robot Scripts

이 디렉토리는 이족보행 로봇의 **보행(Walking)**, **변신(Transformation)**, 그리고 **자동차 모드 조종(Teleoperation)**을 담당하는 핵심 파이썬 스크립트들을 포함하고 있습니다.

## 🚶 이족보행 모드 (Biped Mode)
*   **`op3_walker.py`**: ROBOTIS-OP3의 기구학 모델을 기반으로 한 고성능 보행 엔진입니다. 가제보 시뮬레이션 환경에서 로봇이 안정적으로 걷도록 최적화 하고 있습니다.
*   **`ready_posture.py`**: 로봇의 모든 관절을 기본 차렷 자세(Ready Posture)로 초기화합니다. 보행을 시작하거나 변신을 하기 전 가장 먼저 실행하는 스크립트입니다.

## 🔄 변신 시퀀스 (Transformation)
*   **`transform_bike.py`**: 로봇을 **3轮(Three-wheel) 바이크** 모드로 변신시킵니다. 물리적 무게중심과 각 관절의 지렛대 원리를 이용한 6단계의 정교한 시퀀스로 구성되어 있어, 로봇이 넘어지지 않고 안전하게 변신합니다.
*   **`revert_bike.py`**: 바이크 모드에서 다시 이족보행 모드로 복귀합니다. 바닥에 밀착된 상체를 팔 관절의 강력한 반동(Push-up)을 이용해 밀어 올려 기상하는 6단계 역변신 로직을 수행합니다.

## 🏎️ 자동차 모드 조종 (Car Mode Control)
*   **`bike_teleop.py`**: 바이크 모드로 변신한 로봇을 키보드(**WASD**)로 조종할 수 있게 해주는 텔레오퍼레이션 스크립트입니다. 차동 주행(Differential Drive) 방식을 채택하여 바이크처럼 드리프트와 회전이 가능합니다.

## 🛠️ 유틸리티 (Utilities)
*   **`patch_urdf.py`**: 솔리드웍스(SolidWorks)에서 내보낸 원본 URDF 파일을 ROS 2 및 Gazebo 환경에 맞게 자동으로 보정(조인트 이름, 메시 경로, 제어기 설정 등)해 주는 워크플로우 자동화 도구입니다.

---
### 🚀 실행 방법 예시
```bash
# 0. 가제보 실행
ros2 launch biped_bike_robot gazebo.launch.py 

# 1. 초기 자세 설정
ros2 run biped_bike_robot ready_posture.py

# 2. 바이크 변신
ros2 run biped_bike_robot transform_bike.py

# 3. 키보드 조종
ros2 run biped_bike_robot bike_teleop.py
```
