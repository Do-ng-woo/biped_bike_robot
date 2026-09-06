# Biped Bike Manipulator (native ROS 2)

이 폴더는 Docker 없이 ROS 2 Jazzy 호스트에서 다음 기능을 한 번에 실행한다.

- 기존 12축 하체를 변신 시퀀스의 `DEEP_SQUAT_ARMS_UP` 하체 각도로 이동하고 지속 고정
- OpenRB에 연결된 OMX-L 리더 6축(ID 21~26)을 실제 팔로워 6축(ID 15~20)으로 텔레오퍼레이션
- U20CAM 영상, 실제 팔 관절각, 리더 목표각을 에피소드 단위로 기록
- 로컬 GPU에서 영상 기반 지도학습(behavior cloning)
- 학습 모델을 불러와 실제 팔에 추론 명령 전송
- 브라우저에서 상태·카메라·기록·학습·추론 제어

다른 `biped_bike_robot` 파일은 수정하지 않는다. 하체는 외부 PD 루프를 사용하지 않는다. 활성 모드에서는 하나의 command arbiter가 매 주기 하체 고정 12축과 팔/그리퍼 목표 6축을 합친 18축 trajectory를 발행하고, Dynamixel 내부 position PID가 이를 추종한다. 휠 명령은 모든 모드에서 계속 0으로 보낸다.

## 안전 및 현재 범위

팔로워의 기존 5축 뒤에 `arm_gripper_jnt`를 추가해 OpenCR ID 20과 연결한다. 리더 `gripper_joint_1`(OpenRB ID 26)의 시작 위치를 팔로워 ID 20의 시작 위치에 자동 정렬한 뒤 상대 각도를 추종한다. 실행 시 원본 하드웨어 YAML을 수정하지 않고 `data/runtime/dynamixel_hardware_with_gripper.yaml`을 생성한다. ID 20은 다른 팔 관절과 동일한 일반 Position Control Mode(Operating Mode 3)를 사용한다. Wizard에서 측정한 팔로워 기구 끝점은 닫힘 181.14도(2061 tick), 열림 108.63도(1236 tick)다. 닫힘은 기구 끝점에서 약 0.5도 여유를 둔 2055 tick, 열림은 1.22 rad 범위로 사용한다. 기록에서 열림 값이 증가하는 리더 ID 26은 `+1`, 열릴 때 tick이 감소하는 팔로워 ID 20의 하드웨어 변환은 `-1`로 두어 서로 반대인 물리 회전 방향을 맞춘다. 리더의 더 큰 gripper 회전 범위에는 0.72 배율을 적용해 팔로워 개폐율로 환산하므로 중간 영역이 양 끝값에 포화되지 않는다. ID 20 PWM limit은 다른 관절과 같은 885다.

실시간 텔레옵에서는 기존 브리지의 18개 개별 위치 읽기를 사용하지 않는다. 이 폴더의 `dxl_joint_state_bridge_streaming.py`가 원본 브리지를 읽어 확장하고, 20개 모터 상태를 ID 1~7, 8~14, 15~20의 세 GroupSyncRead로 나눠 20 Hz 수집한다. 한 하체 그룹의 패킷이 빠져도 팔과 그리퍼 피드백은 독립적으로 유지된다. 30 Hz로 들어오는 단일 목표점은 매번 보간을 재시작하지 않고 즉시 GroupSyncWrite하며 1 tick 변화부터 반영한다. Ready/Kneeling 같은 다점 궤적만 기존 시간 보간을 유지한다. 원본 `scripts/dxl_joint_state_bridge.py`는 수정하지 않는다.

처음에는 로봇을 공중에 띄우거나 비상 정지가 가능한 상태에서 낮은 속도로 확인한다. `config/system.yaml`의 `arm.direction`, `arm.offset`, 관절 한계가 실제 장착 방향과 맞는지 축별로 확인해야 한다. 기존 주행/변신 제어 프로그램과 이 프로그램을 동시에 실행하면 같은 모터 명령 토픽을 두 노드가 사용할 수 있으므로 동시에 실행하지 않는다.

## 1. 환경 준비

ROS와 기존 두 워크스페이스를 먼저 불러온다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/dongwoo/install/setup.bash
source /home/dongwoo/biped_bike_ws/install/setup.bash
cd /home/dongwoo/biped_bike_ws/src/biped_bike_robot/manipulator
```

학습 환경은 최초 한 번만 설치한다. PyTorch와 OpenCV가 이 폴더의 `.venv-training`에 설치되고 호스트 ROS 패키지는 읽기 전용으로 공유되며, 다른 Python 환경을 바꾸지 않는다.

```bash
./setup_native.sh
```

장치까지 점검한다.

```bash
./validate_native.py --hardware --leader --camera --training
```

`/dev/opencr`가 없다면 기존 biped 하드웨어 설정에서 사용하는 udev 별칭을 먼저 준비한다. 카메라 번호가 달라졌다면 안정적인 `/dev/v4l/by-id/...-video-index0` 경로로 `config/system.yaml`의 `camera.device`를 바꾸는 것이 좋다.

## 2. 실행

현재 Docker 버전이 카메라나 OpenRB를 잡고 있다면 먼저 해당 컨테이너를 중지한다. 그 뒤:

```bash
./run_native.sh --hardware --leader --camera --ready-on-start
```

브라우저에서 `http://localhost:8000`을 연다. 같은 네트워크의 다른 PC에서는 `http://노트북_IP:8000`으로 접속한다.

하드웨어 없이 UI와 ROS 노드만 확인하려면:

```bash
./run_native.sh
```

`--ready-on-start`를 준 경우에만 OpenCR 브리지가 초기화된 뒤 ID 20 gripper를 포함한 완전한 18축 `/joint_states`를 먼저 기다린다. 그 실측 자세에서 기존 `scripts/bike_transform_sequence.py`의 `HARDWARE_READY`와 gripper 중립값 0 rad까지, MJLab `run_ros_bridge.py`와 같은 cubic smoothstep(`a=t²(3-2t)`) 프로파일을 50 Hz/151개 궤적점으로 만들어 3초 동안 이동한다. 따라서 3초를 기다렸다가 목표로 튀는 방식이 아니며 시작과 끝 속도도 완만하다. 브리지 자체의 즉시 startup posture는 항상 끈다. 실제 로봇을 지지하고 주변을 비운 상태에서 사용한다. 옵션을 생략하면 시작 시 자동 자세 이동은 하지 않는다. 초기 고정 모드에서는 OpenCR IMU를 사용하지 않는다.

하체는 ROS overlay에서 발견되는 오래된 설치본을 사용하지 않고 `config/system.yaml`에 지정된 현재 biped 소스의 `dxl_joint_state_bridge.py`를 직접 실행한다. 원본 `dynamixel_hardware.yaml`에 이 모듈의 ID 20 설정만 합친 런타임 YAML을 사용한다. 리더도 크래시 이력이 있는 `omx_l_leader_ai.launch.py`/`controller_manager` 대신 이 모듈의 `leader_openrb_reader.py`가 OpenRB ID 21~26의 Present Position만 읽는다. 이 리더 노드는 위치 및 토크 명령을 전송하지 않는다.

## 3. 텔레오퍼레이션

웹에서 다음 두 버튼을 순서대로 누른다.

1. **1. Kneeling Stable**: 휠 속도를 0으로 만들고 전체 18축을 기존 `DEEP_SQUAT_ARMS_UP`과 gripper 중립 자세로 4초 동안 smoothstep 이동한다. 실제 하체 관절이 목표 오차 0.12 rad 안에서 0.5초 유지돼야 상태가 `STABLE`이 된다.
2. **2. Manipulating Teleop**: `STABLE` 상태에서만 활성화된다. 리더와 팔로워의 현재 위치를 6축 모두 자동 정렬한 뒤 팔과 gripper 명령을 허용한다.

Record와 Inference도 동일하게 먼저 `STABLE` 확인이 필요하다. 안정화 전 직접 요청해도 웹 API와 ROS 상태 머신 양쪽에서 거부한다.

전환이 15초 안에 완료되지 않으면 `FAULT`가 되고 팔 명령은 차단된다. 활성화 후에도 하체 오차와 joint-state 수신을 계속 감시하며, 허용 오차가 0.3초 이상 지속되거나 상태 수신이 끊기면 `FAULT`로 전환한다. **Arm Stop / Idle**은 새 팔 명령을 끊지만 하체는 마지막 고정 목표를 유지한다.

자세 숫자는 이 폴더에 복사해 두지 않고 기존 `scripts/bike_transform_sequence.py`에서 매 실행 시 직접 읽는다.

| 관절 | Ready | Manipulation kneeling |
|---|---:|---:|
| 왼 hip pitch | 0° | -44.48° |
| 오른 hip pitch | 0° | +44.48° |
| 양 knee pitch | -17.19° | -120° |
| 양 ankle pitch | +18.59° | +85° |
| arm shoulder pitch | -70° | -70° |
| arm elbow pitch | 0° | +20° |

무릎 전환 timeout 시 웹 상태의 `lower_error_rad`와 `largest errors`에 목표에서 가장 멀리 남은 관절이 표시된다. `Skipped ... trajectory joints over their absolute command limits`가 나오면 현재 관절 zero/center 설정을 먼저 확인한다. 실행 로그의 Dynamixel config 경로는 이 모듈 아래 `manipulator/data/runtime/dynamixel_hardware_with_gripper.yaml`이어야 한다.

## 4. 데이터 기록

웹의 Dataset recording 항목을 채우고 **Start**를 누른다. UI가 먼저 고정 무릎 자세 완료를 기다린 뒤 기록을 시작한다.

- `Warmup`: 준비 시간
- `Episode sec`: 한 에피소드 기록 시간
- `Reset timeout sec`: 팔을 기존 `DEEP_SQUAT_ARMS_UP`의 팔 자세로 복귀시키는 제한 시간
- `Stop`: 현재 에피소드를 저장하고 일시 정지
- `Retry`: 현재/직전 에피소드를 버리고 다시 기록
- `Next`: 현재 에피소드를 저장하고 다음 에피소드로 이동
- `Finish`: 남은 유효 샘플을 저장하고 데이터셋 확정

에피소드 사이에는 팔로워를 설정된 reset 자세로 이동시키고 실제 관절 오차가 허용 범위에서 0.5초 유지되는지 확인한다. 그 뒤 리더 기준을 현재 팔로워 자세에 다시 정렬하고 다음 warmup을 시작한다.

데이터는 이 폴더 아래 `data/datasets/<user>/<name_timestamp>/`에만 저장한다. 각 에피소드에는 JPEG 프레임, `camera.mp4`, `samples.npz`, `episode.json`이 생긴다. `samples.npz`에는 카메라 시간, 실제 팔 관절 시간·각도(`observation_state`), 리더 목표 시간·각도(`action`)가 저장된다. 카메라를 기준으로 가장 가까운 관절/명령 샘플을 선택하고 기본 50 ms를 초과한 조합은 기록하지 않는다.

## 5. 학습과 실행

**Finish** 후 Training의 Refresh를 누르고 데이터셋을 선택한 뒤 **Start Training**을 누른다. 기본 모델은 카메라 영상과 현재 팔 관절각에서 다음 팔 목표각을 예측하는 작은 supervised behavior-cloning 모델이다. 결과는 `data/models/<output>/best.pt`에 저장되고 로그는 같은 폴더의 `console.log`와 `training.jsonl`에 남는다.

학습/검증 데이터는 프레임이 아니라 에피소드 단위로 분리하므로 같은 시연의 인접 프레임이 양쪽에 섞이지 않는다. 정규화 통계도 학습 에피소드로만 계산한다.

학습 후 Inference model에서 `best.pt`를 선택해 **Load Model**, 이어서 **Manipulating Inference**를 누른다. 모델의 6축 이름·순서, 입출력 차원, 정규화 값을 실제 로봇과 비교하고 추론 노드의 `LOADED` 응답을 받은 경우에만 INFERENCE 진입을 허용한다. 기존 5축 데이터셋과 모델은 새 6축 계약과 호환되지 않으므로 새로 기록·학습해야 한다. 실제 로봇 적용 전에는 낮은 `arm.max_velocity_rad_s`로 시작하고 반드시 사람이 비상 정지할 수 있어야 한다.

이 초기 모델은 ACT나 diffusion policy가 아닌 검증용 1-step BC 기준선이다. 수집·안전·모드 전환 인터페이스를 그대로 유지하면서 나중에 `learning/model.py`와 `nodes/policy_inference.py`만 ACT 등으로 교체할 수 있다.

## ROS 인터페이스

| 용도 | 토픽 |
|---|---|
| 실제 전체 관절 상태 | `/joint_states` |
| 기존 하드웨어 trajectory | `/joint_trajectory_controller/joint_trajectory` |
| 리더 입력 | `/leader/joint_trajectory` |
| 매핑된 텔레옵 목표 | `/manipulator/teleop_target` |
| 추론 목표 | `/manipulator/inference_target` |
| 모드 요청/상태 | `/manipulator/mode/request`, `/manipulator/mode/state` |
| 기록 요청/상태 | `/manipulator/record/command`, `/manipulator/record/state` |
| 모델 요청/로드 상태 | `/manipulator/inference/model`, `/manipulator/inference/model_state` |
| 카메라 | `/camera1/image_raw/compressed` |

모든 배선, 자세, 한계, 속도, 저장 위치는 `config/system.yaml`에서 관리한다.
