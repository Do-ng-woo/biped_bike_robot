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

### 실기에서 잘 걷는 검증 보정값

아래 조합은 실제 하드웨어에서 보행이 잘 되는 것으로 확인한 보정값입니다.

```bash
ros2 run biped_bike_robot op3_walker.py --ros-args \
  -p num_cycles:=5 \
  -p support_hip_roll_lift_deg:=20.0 \
  -p support_ankle_roll_lift_deg:=10.0 \
  -p support_ankle_roll_lift_sign:=1.0 \
  -p pelvis_pitch_forward_lift_deg:=30.0 \
  -p pelvis_pitch_forward_lift_sign:=1.0 \
  -p trajectory_time_scale:=5.0
```

중요 포인트:

- `num_cycles`: 한 사이클이 좌/우 2보입니다. `num_cycles:=5`면 총 10보 앞으로 갑니다.
- `support_hip_roll_lift_deg`: 한 발 지지 때 힙 롤로 좌우 처짐을 보정합니다.
- `support_ankle_roll_lift_deg`: 지지발 발목/발 롤로 바닥 반력을 보정합니다.
- `pelvis_pitch_forward_lift_deg`: 실기에서 골반 면이 뒤로 눕는 문제를 보정하기 위해, 발이 나가는 동안 골반 면을 앞으로 숙입니다.
- `trajectory_time_scale`: 같은 궤적을 느리게 재생합니다. 위 조합은 5배 느린 재생입니다.
- 보행 끝에는 별도의 복귀 스텝을 만들지 않습니다. 요청한 사이클 수만큼 걷고 마지막 자세를 유지하므로, 발 방향이 바뀌는 어색한 마무리 동작을 피할 수 있습니다.

자주 쓰는 파라미터:

```bash
ros2 run biped_bike_robot op3_walker.py --ros-args \
  -p z_move_amplitude:=0.070 \
  -p y_swap_amplitude:=-0.050 \
  -p x_move_amplitude:=-0.025 \
  -p x_swap_forward_bias:=-0.006 \
  -p x_swap_time_advance_ratio:=0.08 \
  -p init_x_offset:=-0.030 \
  -p init_z_offset:=0.025 \
  -p hip_pitch_offset_deg:=10.0 \
  -p period_time:=3.0 \
  -p dsp_ratio:=0.50 \
  -p pelvis_offset_deg:=0.0 \
  -p support_hip_roll_lift_deg:=10.0 \
  -p support_ankle_roll_lift_deg:=0.0 \
  -p swing_foot_pitch_lift_deg:=0.0 \
  -p swing_ankle_pitch_lift_deg:=0.0 \
  -p pelvis_pitch_forward_lift_deg:=5.0 \
  -p trajectory_time_scale:=1.5
```

주요 파라미터 의미:

- `x_move_amplitude`: 전후 보폭입니다. 현재 로봇 기준 전진은 음수입니다.
- `step_fb_ratio`: 보폭에 비례해 상체/골반 기준점을 앞뒤로 흔드는 비율입니다. 키우면 앞뒤 스윙 전체가 커집니다.
- `x_swap_forward_bias`: 앞뒤 스윙의 중심을 전진 방향으로 옮기는 값입니다. 현재 로봇 기준 전진은 음수이므로, `-0.006`은 전체 스윙을 약 6mm 앞으로 보냅니다.
- `x_swap_time_advance_ratio`: 앞뒤 무게중심 이동 타이밍을 보행 주기 대비 얼마나 앞당길지 정합니다. `0.08`은 한 주기의 8%만큼 먼저 움직입니다.
- `init_x_offset`: 준비 자세부터 적용되는 골반/상체 전후 기준점입니다. 현재 기본값 `-0.030`은 약 3cm 전방입니다.
- `init_z_offset`: 준비 자세부터 적용되는 몸 높이 보정입니다. 값을 키우면 다리가 더 접히고 무게중심이 낮아집니다.
- `hip_pitch_offset_deg`: 기본 hip pitch 전방 기울임입니다. 앞으로 넘어지면 값을 줄여 테스트합니다.
- `z_move_amplitude`: 발을 들어 올리는 높이입니다. 현재 기본값 `0.070`은 약 7cm입니다.
- `y_swap_amplitude`: 상체/무게중심을 좌우로 이동시키는 양입니다. 현재 기본값 `-0.050`은 중심 기준 약 5cm, 왕복 약 10cm입니다.
- `period_time`: 보행 엔진 내부 한 주기 시간입니다. 발 궤적의 phase 계산 자체가 바뀌므로 단순 슬로우모션에는 `trajectory_time_scale`을 사용합니다.
- `trajectory_time_scale`: 생성된 같은 보행 궤적의 timestamp만 늘리거나 줄입니다. `2.0`이면 2배 느린 슬로우모션입니다.
- `x_move_start_scale`: 출발 첫 보행의 보폭 비율입니다. 현재 기본값은 `1.0`이라 처음부터 100% 보폭입니다.
- `x_move_ramp_per_cycle`: 보행 주기마다 보폭을 늘리는 비율입니다. 현재 기본값은 `0.0`이라 ramp 없이 고정 보폭입니다.
- `dsp_ratio`: 양발 지지 구간 비율입니다. 크게 할수록 발을 들기 전에 무게 이동 시간이 길어집니다.
- `pelvis_offset_deg`: 골반 롤 오프셋입니다. 상체 수평 우선 테스트에서는 `0.0`을 권장합니다.
- `support_hip_roll_lift_deg`: 실기 보정값입니다. 한쪽 발이 들릴 때 지지발 hip roll에 점진적으로 더하는 최대 각도입니다.
- `support_hip_roll_lift_sign`: 지지발 hip roll 보정 방향입니다. 보정 후 더 기울어지면 `-1.0`으로 바꿔 테스트합니다.
- `support_ankle_roll_lift_deg`: 실기 보정값입니다. 한쪽 발이 들릴 때 지지발 ankle/foot roll에 점진적으로 더하는 최대 각도입니다.
- `support_ankle_roll_lift_sign`: 지지발 ankle/foot roll 보정 방향입니다. 보정 후 더 기울어지면 `-1.0`으로 바꿔 테스트합니다.
- `swing_foot_pitch_lift_deg`: 실기 보정값입니다. 진행발이 들릴 때 스윙발 pitch에 점진적으로 더하는 최대 각도입니다. 뒤꿈치가 끌리면 부호를 바꿔가며 테스트합니다.
- `swing_foot_pitch_lift_sign`: 스윙발 pitch 보정 방향입니다.
- `swing_ankle_pitch_lift_deg`: 실기 보정값입니다. 진행발이 들릴 때 스윙발 ankle pitch 관절에 직접 더하는 최대 각도입니다. 앞꿈치가 과하게 들리고 뒤꿈치가 끌릴 때 5도 정도부터 테스트합니다.
- `swing_ankle_pitch_lift_sign`: 스윙발 ankle pitch 직접 보정 방향입니다. 뒤꿈치가 더 끌리면 `-1.0`으로 바꿔 테스트합니다.
- `pelvis_pitch_forward_lift_deg`: 실기 보정값입니다. 한쪽 발이 나가는 동안 골반 면을 앞으로 숙이는 hip pitch 보정 최대 각도입니다.
- `pelvis_pitch_forward_lift_sign`: 골반 pitch 보정 방향입니다. 보정 후 골반이 더 뒤로 누우면 `-1.0`으로 바꿔 테스트합니다.
- `num_cycles`: 생성할 보행 주기 수입니다. 한 주기는 좌/우 2보이므로 `1`이면 2보, `5`이면 10보입니다.

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
