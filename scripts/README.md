# Biped Bike Robot Scripts

이 폴더는 로봇의 자세 전환, 보행 궤적 생성, 바이크 모드 주행, URDF 보정, 실기 브릿지를 담당하는 실행 스크립트를 담고 있습니다.

런치 파일 실행법과 하드웨어 파라미터는 `launch/README.md`를 봅니다.

## 보행

### `op3_walker.py`

OP3 walking module 기반의 12자유도 다리 보행 엔진입니다. 계산된 17개 관절 목표를 `JointTrajectory`로 만들어 `/joint_trajectory_controller/joint_trajectory`에 publish합니다.

실행 예:

```bash
ros2 run biped_bike_robot op3_walker.py
```

자주 쓰는 파라미터:

```bash
ros2 run biped_bike_robot op3_walker.py --ros-args \
  -p z_move_amplitude:=0.025 \
  -p y_swap_amplitude:=-0.030 \
  -p period_time:=3.0 \
  -p dsp_ratio:=0.50 \
  -p pelvis_offset_deg:=0.0
```

주요 파라미터 의미:

- `x_move_amplitude`: 전후 보폭입니다. 현재 로봇 기준 전진은 음수입니다.
- `z_move_amplitude`: 발을 들어 올리는 높이입니다.
- `y_swap_amplitude`: 상체/무게중심을 좌우로 이동시키는 양입니다.
- `period_time`: 한 보행 주기 시간입니다. 크게 할수록 느리고 부드럽습니다.
- `dsp_ratio`: 양발 지지 구간 비율입니다. 크게 할수록 발을 들기 전에 무게 이동 시간이 길어집니다.
- `pelvis_offset_deg`: 골반 롤 오프셋입니다. 상체 수평 우선 테스트에서는 `0.0`을 권장합니다.
- `num_cycles`: 생성할 보행 주기 수입니다.

## 준비 자세와 변신

### `ready_posture.py`

보행 전 사용할 기본 준비 자세를 보냅니다. 무릎을 약간 굽히고 로봇을 안정적인 시작 자세로 정렬할 때 사용합니다.

```bash
ros2 run biped_bike_robot ready_posture.py
```

### `transform_bike.py`

이족보행 모드에서 바이크 모드로 변형하는 시퀀스를 보냅니다.

```bash
ros2 run biped_bike_robot transform_bike.py
```

### `revert_bike.py`

바이크 모드에서 다시 이족보행 모드로 돌아오는 시퀀스를 보냅니다.

```bash
ros2 run biped_bike_robot revert_bike.py
```

### `transform_bike_reverse.py`

바이크 변형 방향을 반대로 확인하거나, 역방향 변형 테스트에 쓰는 보조 스크립트입니다.

## 바이크 주행

### `bike_teleop.py`

바이크 모드에서 바퀴 속도 명령을 보내는 텔레옵 스크립트입니다. 바퀴 모터는 `dynamixel_hardware.yaml`에서 velocity mode로 관리합니다.

```bash
ros2 run biped_bike_robot bike_teleop.py
```

## 실기 브릿지

### `dxl_joint_state_bridge.py`

ROS 명령을 Dynamixel XL430 명령으로 변환하는 실기 브릿지입니다.

받는 명령:

- `/joint_states`: RViz GUI 방향 확인용 위치 명령
- `/joint_trajectory_controller/joint_trajectory`: walker/ready/transform 스크립트의 trajectory 명령

하는 일:

- `config/dynamixel_hardware.yaml`에서 모터 ID, joint 이름, 방향, gain, mode를 읽습니다.
- URDF 기준 radian 값을 Dynamixel tick으로 변환합니다.
- 위치 모터에는 `Goal Position`을 sync write합니다.
- 옵션을 켜면 Dynamixel telemetry를 CSV로 기록합니다.

브릿지는 보통 직접 실행하지 않고 `hardware_display.launch.py`로 실행합니다.

## URDF 보정

### `patch_urdf.py`

SolidWorks export URDF를 ROS 2/Gazebo에서 쓰기 좋게 보정하는 스크립트입니다. mesh 경로, joint/controller 연동, Gazebo 사용에 필요한 부분을 정리할 때 사용합니다.

## 하드웨어 기준

현재 실기 기준:

- Dynamixel `XL430-W250`
- TTL bus
- Protocol `2.0`
- Baudrate `1000000`
- OpenCR USB-to-Dynamixel passthrough
- `2048 tick == URDF 0 rad`
- 위치 관절 Operating Mode `3`
- 바퀴 모터 Operating Mode `1`

모터 ID, 방향, gain, telemetry 주소는 `config/dynamixel_hardware.yaml`에서 관리합니다.

