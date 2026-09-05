# Biped Bike Robot Scripts

이 폴더는 로봇의 자세 전환, 보행 궤적 생성, 바이크 모드 주행, URDF 보정, 실기 브릿지를 담당하는 실행 스크립트를 담고 있습니다.

런치 파일 실행법과 하드웨어 파라미터는 `launch/README.md`를 봅니다.

## 보행

### `ik_walker.py`

12자유도 다리 보행 엔진입니다. 사인파 기반 발 궤적과 해석적 IK로 계산한 17개 관절 목표를 `JointTrajectory`로 만들어 `/joint_trajectory_controller/joint_trajectory`에 publish합니다.

실행 예:

```bash
ros2 run biped_bike_robot ik_walker.py
```

워커는 현재 실물 자세에서 보행 시작 자세까지 기본 3초 동안 부드럽게
이동한 뒤 첫 스텝을 시작합니다. 이 시간은 보행 슬로모션 배율과 별개입니다.

```bash
ros2 run biped_bike_robot ik_walker.py --ros-args \
  -p startup_duration_sec:=3.0
```

### 실기에서 잘 걷는 검증 보정값

아래 조합은 실제 하드웨어에서 보행이 잘 되는 것으로 확인한 보정값입니다.

```bash
ros2 run biped_bike_robot ik_walker.py --ros-args \
  -p num_cycles:=1 \
  -p support_hip_roll_lift_deg:=20.0 \
  -p support_ankle_roll_lift_deg:=10.0 \
  -p support_ankle_roll_lift_sign:=1.0 \
  -p pelvis_pitch_forward_lift_deg:=30.0 \
  -p pelvis_pitch_forward_lift_sign:=1.0 \
  -p trajectory_time_scale:=4.0
```

중요 포인트:

- `num_cycles`: 한 사이클이 좌/우 2보입니다. 위 검증 명령은 `1`이라 총 2보만 테스트합니다.
- `support_hip_roll_lift_deg`: 한 발 지지 때 힙 롤로 좌우 처짐을 보정합니다.
- `support_ankle_roll_lift_deg`: 지지발 발목/발 롤로 바닥 반력을 보정합니다.
- `pelvis_pitch_forward_lift_deg`: 한발만 땅에 닿고 반대발을 뻗는 동안, 땅에 닿아 있는 지지발 hip pitch에만 들어가는 동적 전방 보정입니다. 위 검증값은 `30도`입니다.
- `trajectory_time_scale`: 같은 궤적을 느리게 재생합니다. 위 조합은 4배 느린 재생입니다.
- `startup_duration_sec`: 현재 자세에서 워커 시작 자세로 이동하는 시간입니다. 기본값은 3초이며 `trajectory_time_scale`의 영향을 받지 않습니다.
- 보행 끝에는 별도의 복귀 스텝을 만들지 않습니다. 요청한 사이클 수만큼 걷고 마지막 자세를 유지하므로, 발 방향이 바뀌는 어색한 마무리 동작을 피할 수 있습니다.

자주 쓰는 파라미터:

```bash
ros2 run biped_bike_robot ik_walker.py --ros-args \
  -p z_move_amplitude:=0.070 \
  -p y_swap_amplitude:=-0.047 \
  -p x_move_amplitude:=-0.025 \
  -p x_swap_forward_bias:=-0.006 \
  -p x_swap_time_advance_ratio:=0.08 \
  -p init_x_offset:=-0.030 \
  -p init_z_offset:=0.050 \
  -p hip_pitch_offset_deg:=10.0 \
  -p period_time:=3.0 \
  -p dsp_ratio:=0.50 \
  -p foot_lift_delay_ratio:=0.20 \
  -p pelvis_offset_deg:=0.0 \
  -p support_hip_roll_lift_deg:=10.0 \
  -p support_ankle_roll_lift_deg:=0.0 \
  -p swing_foot_pitch_lift_deg:=0.0 \
  -p swing_ankle_pitch_lift_deg:=0.0 \
  -p pelvis_pitch_forward_lift_deg:=5.0 \
  -p arm_shoulder_pitch_deg:=-70.0 \
  -p trajectory_time_scale:=1.5
```

주요 파라미터 의미:

- `x_move_amplitude`: 전후 보폭입니다. 현재 로봇 기준 전진은 음수입니다.
- `step_fb_ratio`: 보폭에 비례해 상체/골반 기준점을 앞뒤로 흔드는 비율입니다. 키우면 앞뒤 스윙 전체가 커집니다.
- `x_swap_forward_bias`: 앞뒤 스윙의 중심을 전진 방향으로 옮기는 값입니다. 현재 로봇 기준 전진은 음수이므로, `-0.006`은 전체 스윙을 약 6mm 앞으로 보냅니다.
- `x_swap_time_advance_ratio`: 앞뒤 무게중심 이동 타이밍을 보행 주기 대비 얼마나 앞당길지 정합니다. `0.08`은 한 주기의 8%만큼 먼저 움직입니다.
- `init_x_offset`: 준비 자세부터 적용되는 골반/상체 전후 기준점입니다. 현재 기본값 `-0.030`은 약 3cm 전방입니다.
- `init_z_offset`: 준비 자세부터 적용되는 몸 높이 보정입니다. 값을 키우면 무릎/발목 pitch가 더 접히고 무게중심이 낮아집니다. ver5 긴 정강이 기준 기본값은 `0.050`입니다.
- `hip_pitch_offset_deg`: 기본 hip pitch 전방 기울임입니다. 앞으로 넘어지면 값을 줄여 테스트합니다.
- `z_move_amplitude`: 발을 들어 올리는 높이입니다. 현재 기본값 `0.070`은 약 7cm입니다.
- `y_swap_amplitude`: 상체/무게중심을 좌우로 이동시키는 양입니다. 현재 기본값 `-0.047`은 중심 기준 약 4.7cm, 왕복 약 9.4cm입니다.
- `period_time`: 보행 엔진 내부 한 주기 시간입니다. 발 궤적의 phase 계산 자체가 바뀌므로 단순 슬로우모션에는 `trajectory_time_scale`을 사용합니다.
- `foot_lift_delay_ratio`: 한 발 지지 구간 시작 후, 이 비율만큼 무게중심을 먼저 옮긴 뒤 발을 듭니다. 골반은 지지발 쪽에 머문 상태로 발을 들어 올리고 내려놓으며, 발이 착지한 뒤에 다음 방향으로 이동합니다. 발을 늦게 든 만큼 착지 완료 시점도 뒤로 밀어 보행 주기를 늘리므로, 내려놓는 궤적을 압축하지 않습니다.
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
- `pelvis_pitch_forward_lift_deg`: 실기 보정값입니다. 한쪽 발이 나가는 동안에만 지지발 hip pitch에 적용되는 보정 최대 각도입니다.
- `pelvis_pitch_forward_lift_sign`: 골반 pitch 보정 방향입니다. 보정 후 골반이 더 뒤로 누우면 `-1.0`으로 바꿔 테스트합니다.
- `arm_shoulder_pitch_deg`: 보행 중 유지할 어깨 pitch 각도입니다. 실물 기본값은 뒤쪽으로 접은 `-70.0`이며, 팔꿈치 pitch는 0도로 유지합니다.
- `num_cycles`: 생성할 보행 주기 수입니다. 한 주기는 좌/우 2보이므로 `1`이면 2보, `5`이면 10보입니다.

## 준비 자세와 변신

### `ready_posture.py`

보행 전 사용할 기본 준비 자세를 보냅니다. 무릎을 약간 굽히고, 뒤쪽이 무거운 실물 기준으로 hip pitch를 앞으로 5도 더 기울이며, 어깨 pitch를 뒤쪽으로 70도 접어 로봇을 안정적인 시작 자세로 정렬할 때 사용합니다. 팔꿈치 pitch는 0도로 유지합니다.

```bash
ros2 run biped_bike_robot ready_posture.py
```

현재 자세에서 레디 자세까지 기본 3초 동안 선형 보간합니다. 시간을 바꾸려면:

```bash
ros2 run biped_bike_robot ready_posture.py --ros-args \
  -p move_duration_sec:=4.0
```

실물 기본 보정값:

- `forward_lean_deg`: 기본값 `5.0`. 좌우 hip pitch에 추가하는 전방 기울임입니다.
- `arm_shoulder_pitch_deg`: 기본값 `-70.0`. ready/walker 중 어깨 pitch를 뒤쪽으로 접어 유지하는 각도입니다. 팔꿈치 pitch는 0도로 유지합니다.

### `transform_bike.py`

이족보행 모드에서 바이크 모드로 변형하는 시퀀스를 보냅니다.
안정적으로 일어나는 `revert_bike.py`의 경로를 역순으로 사용하며, 몸통을
접고 펴는 동안 어깨 관절을 기구 한계인 `-25도`(`-0.436332 rad`)로
유지해 전방 낙하를 막습니다. 이 값보다 뒤로 가는 명령은 실물 브릿지와
Gazebo URDF에서 모두 차단됩니다.
손목은 pitch를 아래로 접기 전에 `arm_wrist_roll_jnt`를 먼저 `180도` 돌린
뒤, 그 상태를 유지한 채 claw pitch를 접습니다.
변신 중 팔꿈치 pitch가 열려 있는 지지/완성 자세는 기존 `10도` 대신
`20도`를 사용하고, 완전한 바이크 자세의 어깨 pitch는 뒤쪽 `-70도` 자세를
유지합니다.
완전한 바이크 자세에서는 hip yaw도 안쪽으로 모아 왼쪽 `+5도`, 오른쪽
`-5도`를 적용합니다. 리버트는 이 자세에서 시작한 뒤 다음 지지 waypoint로
넘어가며 hip yaw가 다시 0도로 풀립니다.

```bash
ros2 run biped_bike_robot transform_bike.py
```

각 단계의 기본 시간은 3초입니다. Gazebo에서 더 느리게 확인하려면:

```bash
ros2 run biped_bike_robot transform_bike.py --ros-args \
  -p stage_duration_sec:=5.0
```

### `revert_bike.py`

바이크 모드에서 다시 이족보행 모드로 돌아오는 시퀀스를 보냅니다.
변신 코드와 공용 자세 정의를 사용하며, 리버트에는 기립 안정화를 위한 중간
waypoint가 포함됩니다. 반복되는 어깨 값은 별도 제약이 아니라 지지 자세를 계속
유지하는 위치 명령입니다.
리버트 초반에는 접힌 claw 상태에서 `arm_wrist_roll_jnt`의 `180도` 자세를
유지한 채 wrist pitch를 먼저 펴고, 이후 arm yaw 복귀 단계에서 wrist roll도
원위치로 돌아옵니다.

기립 구간은 8.0 이후에 별도 waypoint로 나뉩니다. 8.0에서는 arm yaw만
복귀하고 deep squat 자세를 유지합니다. 8.5에서는 무릎/발목은 그대로 둔 채
hip pitch만 READY 방향으로 약 `10도` 먼저 복귀해 뒤쪽 체중을 만듭니다.
이후 9.0, 9.6, 10.2에서 무릎/발목은 각각 `30%`, `65%`, `85%` 펴지고,
hip pitch는 deep squat 기준 누적 `25도`, `38도`, `43도` 복귀합니다.
마지막 10.7에서 `HARDWARE_READY` 자세로 정착합니다.

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

키 조작:

- `w`: 전진 속도를 0.5 rad/s씩 증가
- `s`: 감속 후 후진 속도를 0.5 rad/s씩 증가
- `a`, `d`: 좌·우 회전 성분 조절
- `q` 또는 `Space`: 즉시 정지
- `Ctrl-C`: 정지 명령을 보낸 뒤 종료

기본 최대 바퀴 속도는 `2.0 rad/s`입니다. 실물 브릿지는 ID 7과 14를
velocity mode로 초기화하고 앞의 두 속도값을 전달합니다. 명령이 0.5초 이상
끊기면 watchdog이 두 바퀴를 자동 정지합니다. 첫 시험은 로봇을 들어 바퀴가
바닥에 닿지 않은 상태에서 `w`를 한 번만 눌러 방향을 확인합니다.

더 낮은 속도로 텔레옵을 실행하려면:

```bash
ros2 run biped_bike_robot bike_teleop.py --ros-args \
  -p speed_step:=0.25 \
  -p max_wheel_speed:=1.0
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
- `JointTrajectory`의 점 사이를 8ms 주기로 선형 보간합니다. 첫 점은 모터의 현재 위치에서 시작하므로, 도착 시간까지 기다렸다가 목표로 한 번에 점프하지 않습니다.
- 보행용 전역 명령 한도는 `max_abs_position_rad`를 사용하지만, 180도 회전이 필요한 `arm_base_yaw_jnt`는 하드웨어 YAML의 관절별 한도 `3.14159 rad`를 우선 적용합니다.
- `/wheel_velocity_controller/commands`를 받아 두 실물 바퀴의 Goal Velocity에 sync write하고, 명령 timeout 시 속도를 0으로 만듭니다.
- `startup_ready_posture_on_start:=true`를 켜면 시작 시 hip pitch를 앞으로 더 기울인 ready 자세와 `startup_shoulder_pitch_deg` 어깨 각도를 바로 보냅니다. 실물 보행 기본값은 `startup_forward_lean_deg:=10.0`, `startup_shoulder_pitch_deg:=-70.0`이며 팔꿈치 pitch는 0도입니다.

브릿지는 보통 직접 실행하지 않고 `hardware_display.launch.py`로 실행합니다.

#### OpenCR IMU 데이터 형식과 강화학습 입력 기준

현재 실제 로봇 제어 중 IMU는 OpenCR 내장 IMU를 사용합니다. OpenCR은 Dynamixel TTL 브리지 역할을 하면서, 가상 Dynamixel ID `200`으로 IMU 값을 응답합니다. PC 쪽 `dxl_joint_state_bridge.py`가 이 값을 읽어서 ROS 표준 메시지인 `sensor_msgs/Imu`로 변환해 `/opencr/imu`에 publish합니다.

실행 경로:

```bash
ros2 launch biped_bike_robot hardware_display.launch.py \
  enable_opencr_imu:=true \
  enable_imu_tf:=true \
  use_joint_state_gui:=false \
  publish_present_joint_states:=true \
  enable_joint_state_commands:=false \
  enable_trajectory_commands:=true \
  startup_ready_posture_on_start:=true \
  center_on_start:=false
```

확인은 다음처럼 합니다.

```bash
ros2 topic echo /opencr/imu
ros2 run tf2_ros tf2_echo world base_link
```

웹 컨트롤러의 하드웨어 ON 버튼도 위 설정을 기본으로 실행합니다.

OpenCR 스케치 `arduino/opencr_dxl_bridge_with_imu/opencr_dxl_bridge_with_imu.ino`는 주소 `100`부터 `68 byte`짜리 IMU 블록을 만듭니다. PC에서는 Dynamixel Protocol 2.0 read로 이 블록을 읽습니다.

| Offset | Type | 이름 | 단위/의미 |
|---:|---|---|---|
| 100 | `uint32` | `time_ms` | OpenCR 부팅 후 ms |
| 104 | `float` | `qw` | OpenCR IMU quaternion w |
| 108 | `float` | `qx` | OpenCR IMU quaternion x |
| 112 | `float` | `qy` | OpenCR IMU quaternion y |
| 116 | `float` | `qz` | OpenCR IMU quaternion z |
| 120 | `float` | `roll_deg` | roll, degree |
| 124 | `float` | `pitch_deg` | pitch, degree |
| 128 | `float` | `yaw_deg` | yaw, degree |
| 132 | `float` | `gyro_x_dps` | gyro x, degree/sec |
| 136 | `float` | `gyro_y_dps` | gyro y, degree/sec |
| 140 | `float` | `gyro_z_dps` | gyro z, degree/sec |
| 144 | `float` | `acc_x_g` | accel x, g |
| 148 | `float` | `acc_y_g` | accel y, g |
| 152 | `float` | `acc_z_g` | accel z, g |
| 156 | `int16` | `gyro_x_adc` | raw gyro x |
| 158 | `int16` | `gyro_y_adc` | raw gyro y |
| 160 | `int16` | `gyro_z_adc` | raw gyro z |
| 162 | `int16` | `acc_x_adc` | raw accel x |
| 164 | `int16` | `acc_y_adc` | raw accel y |
| 166 | `int16` | `acc_z_adc` | raw accel z |

PC 쪽에서는 이 블록을 다음 형식으로 unpack합니다.

```python
struct.unpack("<I13f6h", bytes(data))
```

`/opencr/imu`는 `sensor_msgs/Imu`입니다. 강화학습에서 ROS 토픽을 바로 쓰면 이 형식을 기준으로 받으면 됩니다.

```text
header.frame_id: body_link
orientation: quaternion, x/y/z/w
angular_velocity: rad/s
linear_acceleration: m/s^2
```

변환 규칙은 다음과 같습니다.

| ROS 필드 | 원본 OpenCR 값 | 변환 |
|---|---|---|
| `orientation.w/x/y/z` | `qw/qx/qy/qz` | 그대로 사용 |
| `angular_velocity.x/y/z` | `gyro_*_dps` | `deg/s * pi / 180` |
| `linear_acceleration.x/y/z` | `acc_*_g` | `g * 9.80665` |
| `orientation_covariance` | 고정값 | 대각선 `0.0025` |
| `angular_velocity_covariance` | 고정값 | 대각선 `0.02` |
| `linear_acceleration_covariance` | 고정값 | 대각선 `0.04` |

중요한 점은 `/opencr/imu`의 quaternion은 OpenCR 보드 기준 원본 자세라는 것입니다. 보드가 로봇 등 뒤에 수직으로 달려 있어서, 이 값을 그대로 쓰면 로봇의 roll/pitch/yaw 감각과 다르게 보입니다.

로봇 기준 자세 시각화는 `imu_base_tf.py`가 담당합니다. 이 노드는 `/opencr/imu`를 받아서 `world -> base_link` TF를 publish합니다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `imu_mount_roll_deg` | `-90.0` | OpenCR 보드 장착 roll 보정 |
| `imu_mount_pitch_deg` | `0.0` | 장착 pitch 보정 |
| `imu_mount_yaw_deg` | `0.0` | 장착 yaw 보정 |
| `imu_rpy_remap` | `YRp` | roll/pitch/yaw 축 재매핑. 대문자는 부호 반전 |
| `imu_zero_yaw_on_start` | `true` | 켤 때 yaw를 정면 0으로 잡음 |
| `imu_yaw_zero_samples` | `10` | 처음 10개 샘플 평균으로 yaw zero 계산 |
| `imu_pivot_x/y/z` | `-0.066549, -0.076779, 0.018299` | base_link 중앙 기준 회전 pivot 보정 |

즉 RViz와 웹 뷰어에서 로봇이 기울어지는 자세는 원본 `/opencr/imu`가 아니라 보정된 `/tf`의 `world -> base_link`를 보고 있습니다. 강화학습에서 로봇 몸통 기준 roll/pitch/yaw를 쓰고 싶으면 이 보정된 TF 또는 동일한 보정식을 적용한 quaternion을 쓰는 것이 안전합니다.

강화학습 observation 추천:

```text
base_orientation_quat: [x, y, z, w]      # 보정된 world -> base_link quaternion 권장
base_angular_velocity: [wx, wy, wz]      # /opencr/imu angular_velocity, rad/s
base_linear_accel: [ax, ay, az]          # /opencr/imu linear_acceleration, m/s^2
joint_position: 현재 /joint_states position
joint_velocity: 현재 /joint_states velocity가 안정적으로 읽히면 사용
```

정책 입력으로 yaw 절대각이 필요 없으면 quaternion 대신 gravity vector를 body frame으로 회전한 값이나 roll/pitch만 쓰는 편이 더 안정적입니다. yaw는 시작할 때 `imu_zero_yaw_on_start:=true`로 0을 잡지만, 장시간 운용에서는 IMU yaw drift가 생길 수 있습니다.

주의할 점:

- `/opencr/imu`의 `linear_acceleration`에는 중력 성분이 포함됩니다. 서 있을 때도 한 축에 약 `9.8 m/s^2` 크기가 나오는 것이 정상입니다.
- 로봇이 모터를 움직이는 중에도 IMU는 같은 OpenCR-Dynamixel 포트로 읽습니다. `enable_opencr_imu:=true`일 때 bridge가 주기적으로 가상 ID `200`을 읽기 때문에 로봇 동작과 동시에 사용할 수 있습니다.
- IMU publish rate 기본값은 `30 Hz`입니다. 더 빠르게 읽으면 DXL 제어 주기와 버스 부하에 영향을 줄 수 있으니, RL 로깅은 처음에는 `30 Hz` 기준으로 맞추는 것이 좋습니다.
- 실험 전에는 `ros2 topic hz /opencr/imu`로 실제 주기를 확인합니다.

IMU 숫자 형식을 단독으로 보고 싶을 때만 `arduino/opencr_imu_stream/opencr_imu_stream.ino`를 올립니다. 이 모드는 USB serial에 CSV를 출력하므로 Dynamixel 제어와 동시에 쓰는 모드가 아닙니다.

```bash
python3 src/biped_bike_robot/scripts/opencr_imu_reader.py --port /dev/opencr --rows 20
```

CSV 컬럼은 다음 순서입니다.

```text
time_ms,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,gyro_x_dps,gyro_y_dps,gyro_z_dps,acc_x_g,acc_y_g,acc_z_g,gyro_x_adc,gyro_y_adc,gyro_z_adc,acc_x_adc,acc_y_adc,acc_z_adc
```

실제 로봇 제어로 돌아가려면 다시 `arduino/opencr_dxl_bridge_with_imu/opencr_dxl_bridge_with_imu.ino`를 OpenCR에 업로드해야 합니다.

### `web_control.py`

실물 테스트용 로컬 웹 인터페이스입니다. 브라우저 버튼으로 하드웨어 브릿지를 켜고 끄거나, 검증된 보행/변신 명령을 실행합니다.

```bash
ros2 run biped_bike_robot web_control.py
```

실행 후 브라우저에서 엽니다.

```text
http://127.0.0.1:8080
```

`ros2 run`에서 새 스크립트를 못 찾으면 워크스페이스 루트에서 직접 실행합니다.

```bash
python3 src/biped_bike_robot/scripts/web_control.py
```

버튼 동작:

- `Hardware ON`: 아래 하드웨어 브릿지 명령을 실행하고 계속 유지합니다.
- `Hardware OFF`: 실행 중인 하드웨어 브릿지를 종료합니다.
- `Run Walk`: 입력한 `cycles` 값으로 실기 보행 보정 명령을 실행합니다.
- `Run Transform`: 입력한 `sec/stage` 값으로 `transform_bike.py`를 실행합니다.
- `Teleop ON/OFF`: 바이크 모드 키보드 주행 publisher를 켜고 끕니다.
- `Set Speed`: 키보드 주행 속도를 `0.1`에서 `5.0 rad/s` 사이에서 지정합니다.
- 키보드 주행: `W/S`는 전진/후진, `A/D`는 좌/우 회전, `Q` 또는 `Space`는 바퀴 정지입니다. 웹 페이지에 현재 눌린 키와 현재 속도가 표시됩니다.

웹 패널의 `Hardware ON` 명령:

```bash
ros2 launch biped_bike_robot hardware_display.launch.py \
  enable_opencr_imu:=true \
  enable_imu_tf:=true \
  use_joint_state_gui:=false \
  publish_present_joint_states:=true \
  enable_joint_state_commands:=false \
  enable_trajectory_commands:=true \
  startup_ready_posture_on_start:=true \
  center_on_start:=false
```

웹 패널의 `Run Walk` 명령:

```bash
ros2 run biped_bike_robot ik_walker.py --ros-args \
  -p num_cycles:=1 \
  -p support_hip_roll_lift_deg:=20.0 \
  -p support_ankle_roll_lift_deg:=10.0 \
  -p support_ankle_roll_lift_sign:=1.0 \
  -p pelvis_pitch_forward_lift_deg:=30.0 \
  -p pelvis_pitch_forward_lift_sign:=1.0 \
  -p trajectory_time_scale:=4.0
```

웹 패널의 `Run Transform` 명령:

```bash
ros2 run biped_bike_robot transform_bike.py --ros-args \
  -p stage_duration_sec:=5.0
```

웹 패널의 바이크 주행 명령은 별도 터미널 teleop 대신 브라우저 키 입력을 받아 `/wheel_velocity_controller/commands`로 직접 publish합니다.

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
