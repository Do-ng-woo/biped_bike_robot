# Launch Files

이 폴더는 RViz 표시, Gazebo 시뮬레이션, 실제 Dynamixel 하드웨어 연동 런치를 담고 있습니다.

## 공통 준비

```bash
cd ~/biped_bike_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select biped_bike_robot
source install/setup.bash
```

## `display.launch.py`

URDF를 RViz에서 확인하고 `joint_state_publisher_gui` 슬라이더로 관절을 움직여 보는 순수 표시용 런치입니다. 실제 모터에는 연결하지 않습니다.

```bash
ros2 launch biped_bike_robot display.launch.py
```

실행 노드:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`

## `gazebo.launch.py`

Gazebo에서 로봇과 `ros2_control` controller를 실행합니다. IK walker, ready posture, transform script를 시뮬레이션에서 먼저 검증할 때 사용합니다.

```bash
ros2 launch biped_bike_robot gazebo.launch.py
```

보행 실행:

```bash
ros2 run biped_bike_robot ik_walker.py
```

실행 구성:

- `ros_gz_sim`
- `robot_state_publisher`
- `ros_gz_bridge` clock bridge
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `wheel_velocity_controller`

## `hardware_display.launch.py`

RViz, GUI, Dynamixel bridge를 함께 실행하는 실제 하드웨어 연동 런치입니다. OpenCR에는 `arduino/opencr_usb_to_dxl/opencr_usb_to_dxl.ino`가 올라가 있어야 하고, Dynamixel Wizard나 Arduino Serial Monitor가 `/dev/ttyACM0`을 잡고 있으면 안 됩니다.

실기 브릿지는 `JointTrajectory`를 현재 모터 위치부터 8ms 주기로 선형
보간합니다. 따라서 `ready_posture.py`의 3초 도착 시간과 워커의 시작 전환
시간이 실제 모터에도 적용됩니다. 단, 아래의 `center_on_start` 중앙정렬은
기존처럼 런치 직후 즉시 전송됩니다.

기본 실행:

```bash
ros2 launch biped_bike_robot hardware_display.launch.py
```

기본값으로 `center_on_start:=true`입니다. 실행하면 위치 모터에 2048 tick 정자세 명령을 보냅니다.

### 방향 확인 모드

작은 범위만 허용하고 GUI 슬라이더를 실제 모터로 보냅니다.

```bash
ros2 launch biped_bike_robot hardware_display.launch.py \
  max_abs_position_rad:=0.1 \
  enable_joint_state_commands:=true \
  enable_trajectory_commands:=false
```

### 실기 보행 모드

GUI 명령은 끄고 `JointTrajectory`만 받습니다.

터미널 1:

```bash
ros2 launch biped_bike_robot hardware_display.launch.py \
  max_abs_position_rad:=2.2 \
  center_on_start:=false \
  startup_ready_posture_on_start:=true \
  startup_forward_lean_deg:=10.0 \
  startup_shoulder_pitch_deg:=-70.0 \
  enable_joint_state_commands:=false \
  enable_trajectory_commands:=true
```

터미널 2:

```bash
ros2 run biped_bike_robot ready_posture.py

ros2 run biped_bike_robot ik_walker.py
```

첫 실물 검증은 로봇을 지지한 상태에서 다음처럼 한 사이클만 5배 느리게
실행합니다.

```bash
ros2 run biped_bike_robot ik_walker.py --ros-args \
  -p num_cycles:=1 \
  -p trajectory_time_scale:=5.0 \
  -p startup_duration_sec:=3.0
```

### 모터 로그 모드

Dynamixel telemetry를 CSV로 저장합니다. 기본 저장 위치는 아래입니다.

```text
src/biped_bike_robot/motor_logs/dxl_telemetry_YYYYMMDD_HHMMSS.csv
```

전체 모터를 10초간 5Hz로 기록:

```bash
ros2 launch biped_bike_robot hardware_display.launch.py \
  log_telemetry:=true \
  max_abs_position_rad:=2.2 \
  center_on_start:=false \
  startup_ready_posture_on_start:=true \
  startup_forward_lean_deg:=10.0 \
  startup_shoulder_pitch_deg:=-70.0 \
  enable_joint_state_commands:=false \
  enable_trajectory_commands:=true
```

롤축 4개만 20초간 10Hz로 기록:

```bash
ros2 launch biped_bike_robot hardware_display.launch.py \
  log_telemetry:=true \
  telemetry_rate_hz:=10.0 \
  telemetry_duration_sec:=20.0 \
  telemetry_motor_ids:=2,6,9,13 \
  max_abs_position_rad:=2.2 \
  center_on_start:=false \
  startup_ready_posture_on_start:=true \
  startup_forward_lean_deg:=10.0 \
  startup_shoulder_pitch_deg:=-70.0 \
  enable_joint_state_commands:=false \
  enable_trajectory_commands:=true
```

로그 컬럼:

- `goal_tick`: 브릿지가 마지막으로 보낸 목표 위치
- `present_position_tick`: 실제 encoder 위치
- `position_error_tick`: `goal_tick - present_position_tick`
- `present_pwm_percent`: 모터 출력 PWM 비율
- `present_load_percent`: XL430 내부 추정 load
- `voltage_v`: 입력 전압
- `temperature_c`: 모터 온도

판단 기준:

- `position_error_tick`이 커지고 `present_pwm_percent`가 90~100%에 가까우면 토크/기구 한계 가능성이 큽니다.
- `position_error_tick`이 커지는 순간 `voltage_v`가 크게 떨어지면 전원/배선 문제가 의심됩니다.
- `present_pwm_percent`가 낮은데 오차가 크면 gain, profile, 명령 주기, 기구 유격을 확인합니다.

## `hardware_display.launch.py` 파라미터

- `torque_on_start`  
  기본값 `true`. 시작 시 위치 모터 torque를 켭니다.

- `center_on_start`  
  기본값 `true`. 시작 시 위치 모터를 2048 tick으로 보냅니다. 현재 자세가 많이 틀어져 있으면 갑자기 움직일 수 있으므로 로봇을 잡거나 지지대에 올린 상태에서 켭니다.

- `startup_ready_posture_on_start`  
  기본값 `false`. 시작 시 2048 tick 영점 대신 앞으로 살짝 기울어진 ready 자세를 보냅니다. 실물 보행 시작 전에는 `center_on_start:=false`와 함께 켭니다.

- `startup_forward_lean_deg`  
  기본값 `5.0`. `startup_ready_posture_on_start`에서 좌우 hip pitch에 추가하는 전방 기울임입니다. 현재 실물 보행 기본 예시는 `10.0`을 사용합니다.

- `startup_shoulder_pitch_deg`  
  기본값 `-70.0`. `startup_ready_posture_on_start`에서 어깨 pitch를 뒤쪽으로 접어 유지하는 각도입니다. 팔꿈치 pitch는 0도로 유지합니다.

- `max_abs_position_rad`  
  기본값 `0.35`. 이 절대값보다 큰 위치 명령은 무시합니다. 실기 보행에서는 보통 `2.2` 정도로 올립니다.

- `log_joint_states`  
  기본값 `true`. `/joint_states`에서 받은 joint 명령을 터미널에 출력합니다.

- `enable_joint_state_commands`  
  기본값 `true`. GUI 슬라이더 `/joint_states` 명령을 실제 모터로 보냅니다.

- `enable_trajectory_commands`  
  기본값 `true`. walker/ready/transform 스크립트의 `JointTrajectory` 명령을 실제 모터로 보냅니다.

- `enable_velocity_commands`  
  기본값 `true`. bike teleop의 바퀴 속도 명령을 ID 7과 14에 전달합니다.

- `max_wheel_velocity_rad_s`  
  기본값 `2.0`. 실물 바퀴별 속도 명령의 절대 상한입니다.

- `wheel_command_timeout_sec`  
  기본값 `0.5`. 이 시간 동안 새 속도 명령이 없으면 두 바퀴를 자동 정지합니다.

- `log_telemetry`  
  기본값 `false`. Dynamixel telemetry CSV 기록을 켭니다.

- `telemetry_log_path`  
  기본값 빈 문자열. 비워두면 `src/biped_bike_robot/motor_logs`에 자동 파일명으로 저장합니다.

- `telemetry_rate_hz`  
  기본값 `5.0`. telemetry 기록 주기입니다.

- `telemetry_duration_sec`  
  기본값 `10.0`. 이 시간이 지나면 telemetry 기록을 멈춥니다. `0`이면 종료할 때까지 계속 기록합니다.

- `telemetry_motor_ids`  
  기본값 빈 문자열. 비워두면 모든 활성 모터를 기록합니다. 예: `2,6,9,13`.

## 안전 메모

- 실제 로봇 테스트는 처음에는 반드시 로봇을 손으로 잡거나 지지대에 올려서 합니다.
- `center_on_start`는 편하지만 시작 즉시 정자세 명령이 나갑니다.
- Dynamixel Wizard, Arduino IDE Serial Monitor, 다른 ROS 노드가 같은 포트를 열면 브릿지가 실패합니다.
- 로그 CSV는 커질 수 있으므로 기본 제한을 유지하고 필요한 모터만 `telemetry_motor_ids`로 골라 기록합니다.
