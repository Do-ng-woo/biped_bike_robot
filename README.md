# 🤖 Biped Bike Robot

> **biped_bike_robot_print_ver6** — SolidWorks에서 설계하고 실물 구동으로 검증 중인 **이족보행 ↔ 2구동 휠 + 볼캐스터 변환 로봇**  
> ROS 2 Jazzy + Gazebo Harmonic 환경

### ✨ 현재 URDF 반영 상태
- **최신 URDF**: `urdf/biped_bike_robot.urdf`
- **원본 익스포트**: `solidworks_export/urdf/biped_bike_robot_print_ver6.urdf`
- **구조**: 20 links, 19 joints = 17 actuated revolute joints + 2 continuous knee wheel joints
- **총 질량**: 약 **1.917 kg** (`base_link`에 메인 프레임, 배터리, 제어보드 질량 반영)
- **구동 모드**: 12자유도 이족보행, 팔 지지 기반 변신, 2륜 주행 모드

---

## 🎥 실물 구동 데모

### 1. 이족 보행 (Bipedal Walking)
![Bipedal Walking Demo](media/walking.gif)

### 2. 차륜형 변신 (Transform to Wheeled Mode)
![Transform to Wheeled Mode](media/transform.gif)

### 3. 이족 보행 모드 복귀 (Reverse Transform)
![Reverse Transform](media/revert_transform.gif)

### 4. 바이크 모드 주행 (Wheeled Driving)
![Wheeled Driving Demo](media/drive.gif)

---

## 🎬 시뮬레이션 데모

### 1. 시뮬레이션 보행
![Simulation Walking Demo](media/walking_demo.gif)

### 2. 시뮬레이션 변신
![Simulation Transform Demo](media/transform0506.gif)

### 3. 시뮬레이션 역변신
![Simulation Reverse Transform Demo](media/reverse_transform0506.gif)

---

## 📐 관절 트리 다이어그램 (Joint Tree)

```
base_link (골반/허리 프레임)
├── [L] l_hip_yaw ──────── l_hip_yaw (좌측 고관절 요)
│   └── l_hip_roll ─────── l_hip_roll (좌측 고관절 롤)
│       └── l_hip_pitch ── l_hip_pitch (좌측 고관절 피치)
│           └── l_knee_pitch ── l_knee_pitch (좌측 무릎)
│               ├── l_ankle_pitch ── l_ankle_pitch (좌측 발목)
│               │   └── l_foot_roll ── l_foot_roll (좌측 발)
│               └── l_knee_pitch_wheel ── l_knee_pitch_wheel (좌측 무릎 휠) 🔄
│
├── [R] r_hip_yaw ──────── r_hip_yaw (우측 고관절 요)
│   └── r_hip_roll ─────── r_hip_roll (우측 고관절 롤)
│       └── r_hip_pitch ── r_hip_pitch (우측 고관절 피치)
│           └── r_knee_pitch ── r_knee_pitch (우측 무릎)
│               ├── r_ankle_pitch ── r_ankle_pitch (우측 발목)
│               │   └── r_foot_roll ── r_foot_roll (우측 발)
│               └── r_knee_pitch_wheel ── r_knee_pitch_wheel (우측 무릎 휠) 🔄
│
└── [U] arm_base_yaw ──── arm_base_yaw (팔 베이스 요)
    └── arm_shoulder_pitch ── arm_shoulder_pitch (어깨 피치)
        ├── arm_elbow_pitch ── arm_elbow_pitch (팔꿈치 피치)
        │   └── arm_wrist_pitch ── arm_wrist_pitch (손목 피치)
        │       └── arm_wrist_roll ── arm_wrist_roll (손목 롤)
```

> 🔄 = `continuous` 타입 (무한 회전 가능한 패시브 휠 관절)
> 현재 구조에서는 팔 끝 지지를 회전 휠 조인트가 아니라 볼캐스터 구조로 처리합니다.

---

## 📊 링크 목록 (20개)

| # | 링크 이름 | 위치 | 질량 (kg) | 설명 |
|---|-----------|------|-----------|------|
| 1 | `base_link` | 중심 | 0.626 | 골반/메인 프레임 (메인 기판 및 배터리 반영) |
| 2 | `l_hip_yaw` | 좌측 | 0.027 | 좌측 고관절 요 회전부 |
| 3 | `l_hip_roll` | 좌측 | 0.118 | 좌측 고관절 롤 링크 |
| 4 | `l_hip_pitch` | 좌측 | 0.014 | 좌측 고관절 피치 링크 |
| 5 | `l_knee_pitch` | 좌측 | 0.146 | 좌측 허벅지 (상부 다리) |
| 6 | `l_ankle_pitch` | 좌측 | 0.118 | 좌측 정강이 (하부 다리) |
| 7 | `l_foot_roll` | 좌측 | 0.075 | 좌측 발 |
| 8 | `l_knee_pitch_wheel` | 좌측 | 0.017 | 좌측 무릎 구동 휠 |
| 9 | `r_hip_yaw` | 우측 | 0.027 | 우측 고관절 요 회전부 |
| 10 | `r_hip_roll` | 우측 | 0.118 | 우측 고관절 롤 링크 |
| 11 | `r_hip_pitch` | 우측 | 0.014 | 우측 고관절 피치 링크 |
| 12 | `r_knee_pitch` | 우측 | 0.146 | 우측 허벅지 (상부 다리) |
| 13 | `r_ankle_pitch` | 우측 | 0.118 | 우측 정강이 (하부 다리) |
| 14 | `r_foot_roll` | 우측 | 0.075 | 우측 발 |
| 15 | `r_knee_pitch_wheel` | 우측 | 0.017 | 우측 무릎 구동 휠 |
| 16 | `arm_base_yaw` | 상체 | 0.053 | 팔 베이스 요 회전부 |
| 17 | `arm_shoulder_pitch` | 상체 | 0.105 | 상완 (어깨 피치) |
| 18 | `arm_elbow_pitch` | 상체 | 0.044 | 전완 (팔꿈치 피치) |
| 19 | `arm_wrist_pitch` | 상체 | 0.020 | 손목 피치 |
| 20 | `arm_wrist_roll` | 상체 | 0.039 | 손목 롤 / 엔드이펙터 |
> **총 질량**: 약 **1.917 kg** (URDF inertial mass 합산)

---

## 🔩 관절 목록 (19개)

### 좌측 다리 (Left Leg) — 6 액추에이터 + 1 패시브

| # | 관절 이름 | 타입 | 축 | 부모 → 자식 |
|---|-----------|------|-----|-------------|
| 1 | `l_hip_yaw` | revolute | Z(-) | base_link → l_hip_yaw |
| 2 | `l_hip_roll` | revolute | Z(-) | l_hip_yaw → l_hip_roll |
| 3 | `l_hip_pitch` | revolute | Z(+) | l_hip_roll → l_hip_pitch |
| 4 | `l_knee_pitch` | revolute | Z(-) | l_hip_pitch → l_knee_pitch |
| 5 | `l_ankle_pitch` | revolute | Z(-) | l_knee_pitch → l_ankle_pitch |
| 6 | `l_foot_roll` | revolute | Z(-) | l_ankle_pitch → l_foot_roll |
| 7 | `l_knee_pitch_wheel` | **continuous** | Z(-) | l_knee_pitch → l_knee_pitch_wheel |

### 우측 다리 (Right Leg) — 6 액추에이터 + 1 패시브

| # | 관절 이름 | 타입 | 축 | 부모 → 자식 |
|---|-----------|------|-----|-------------|
| 8 | `r_hip_yaw` | revolute | Z(-) | base_link → r_hip_yaw |
| 9 | `r_hip_roll` | revolute | Z(-) | r_hip_yaw → r_hip_roll |
| 10 | `r_hip_pitch` | revolute | Z(-) | r_hip_roll → r_hip_pitch |
| 11 | `r_knee_pitch` | revolute | Z(-) | r_hip_pitch → r_knee_pitch |
| 12 | `r_ankle_pitch` | revolute | Z(-) | r_knee_pitch → r_ankle_pitch |
| 13 | `r_foot_roll` | revolute | Z(-) | r_ankle_pitch → r_foot_roll |
| 14 | `r_knee_pitch_wheel` | **continuous** | Z(+) | r_knee_pitch → r_knee_pitch_wheel |

### 상체 (Upper Body) — 5 액추에이터

| # | 관절 이름 | 타입 | 축 | 부모 → 자식 |
|---|-----------|------|-----|-------------|
| 15 | `arm_base_yaw` | revolute | Z(-) | base_link → arm_base_yaw |
| 16 | `arm_shoulder_pitch` | revolute | Z(+) | arm_base_yaw → arm_shoulder_pitch |
| 17 | `arm_elbow_pitch` | revolute | Z(+) | arm_shoulder_pitch → arm_elbow_pitch |
| 18 | `arm_wrist_pitch` | revolute | Z(-) | arm_elbow_pitch → arm_wrist_pitch |
| 19 | `arm_wrist_roll` | revolute | Z(-) | arm_wrist_pitch → arm_wrist_roll |

---

## 🦿 설계 특징

### 1. 좌우 대칭 이족 구조
- 각 다리: **6 DOF** (Hip Yaw → Hip Roll → Hip Pitch → Knee Pitch → Ankle Pitch → Foot Roll)
- 좌우 다리는 `base_link`에서 Y축 방향으로 오프셋 (좌: y=-0.01425, 우: y=-0.13925)
- 고관절 롤(Roll)은 `rpy="0 -π/2 0"`로 90° 회전하여 측면 운동축을 구현

### 2. 차륜 모드를 위한 휠과 볼캐스터
- `l_knee_pitch_wheel_jnt`, `r_knee_pitch_wheel_jnt` — 2개의 **continuous** 구동 휠 관절
- 팔 끝 지지는 `arm_wheel_pitch_` 조인트 없이 볼캐스터 구조로 처리

### 3. 단일 팔 지지 구조
- `arm_base_yaw` → `arm_shoulder_pitch` → `arm_elbow_pitch` → `arm_wrist_pitch` → `arm_wrist_roll`
- `arm_wrist_roll`이 자전거 핸들을 잡는 최종 엔드이펙터

### 4. 서보 모터 기반 소형 로봇
- 총 질량 약 **1.917 kg**
- 모든 메시가 STL 형식 (SolidWorks 직접 내보내기)
- URDF에는 Gazebo friction/contact 및 `ros2_control` 인터페이스가 포함되어 있습니다.

---

## 📁 패키지 구조

```
biped_bike_robot/
├── CMakeLists.txt
├── package.xml
├── README.md                    ← 이 문서
├── config/
│   ├── controllers.yaml         ← ros2_control 컨트롤러 설정
│   └── dynamixel_hardware.yaml  ← 실물 Dynamixel ID/방향/제한 설정
├── launch/
│   ├── display.launch.py        ← RViz 모델 표시
│   ├── gazebo.launch.py         ← Gazebo 시뮬레이션 런치
│   └── hardware_display.launch.py ← 실물 브릿지 + RViz/robot_state_publisher
├── media/
│   ├── walking.gif              ← 실물 보행 데모
│   ├── transform.gif            ← 실물 변신 데모
│   ├── revert_transform.gif     ← 실물 역변신 데모
│   ├── drive.gif                ← 실물 주행 데모
│   ├── walking_demo.gif         ← 시뮬레이션 보행 데모
│   ├── transform0506.gif        ← 시뮬레이션 변신 데모
│   └── reverse_transform0506.gif ← 시뮬레이션 역변신 데모
├── meshes/                      ← 메시 파일 (20개 STL)
│   ├── base_link.STL
│   ├── l_hip_yaw.STL
│   ├── l_hip_roll.STL
│   ├── l_hip_pitch.STL
│   ├── l_knee_pitch.STL
│   ├── l_ankle_pitch.STL
│   ├── l_foot_roll.STL
│   ├── l_knee_pitch_wheel.STL
│   ├── r_hip_yaw.STL
│   ├── r_hip_roll.STL
│   ├── r_hip_pitch.STL
│   ├── r_knee_pitch.STL
│   ├── r_ankle_pitch.STL
│   ├── r_foot_roll.STL
│   ├── r_knee_pitch_wheel.STL
│   ├── arm_base_yaw.STL
│   ├── arm_shoulder_pitch.STL
│   ├── arm_elbow_pitch.STL
│   ├── arm_wrist_pitch.STL
│   └── arm_wrist_roll.STL
├── urdf/
│   └── biped_bike_robot.urdf    ← 최신 보정 URDF (print ver6 기반)
├── scripts/
│   ├── ik_walker.py             ← IK 기반 보행 엔진
│   ├── transform_bike.py        ← 바이크 모드 변신 시퀀스
│   ├── revert_bike.py           ← 이족보행 모드 복구 시퀀스
│   ├── bike_teleop.py           ← 키보드(WASD) 주행 컨트롤러
│   ├── dxl_joint_state_bridge.py ← 실물 Dynamixel 브릿지
│   ├── imu_base_tf.py           ← 실물 자세 TF 보정
│   ├── web_control.py           ← 실물 테스트용 웹 컨트롤 패널
│   ├── ready_posture.py         ← 기본 준비 자세 초기화
│   └── patch_urdf.py            ← SolidWorks→ROS 2 자동 보정 도구
└── solidworks_export/             ← SolidWorks 원본 (수정 금지)
    ├── README.md
    ├── urdf/biped_bike_robot_print_ver6.urdf
    └── meshes/
```

---

## 🔗 관절 연쇄 다이어그램 (Kinematic Chain)

```mermaid
graph TD
    B["base_link<br/>골반 프레임<br/>626g"]

    B -->|l_hip_yaw| LHY["l_hip_yaw<br/>27g"]
    LHY -->|l_hip_roll| LHR["l_hip_roll<br/>118g"]
    LHR -->|l_hip_pitch| LHP["l_hip_pitch<br/>14g"]
    LHP -->|l_knee_pitch| LK["l_knee_pitch<br/>146g"]
    LK -->|l_ankle_pitch| LA["l_ankle_pitch<br/>118g"]
    LA -->|l_foot_roll| LF["l_foot_roll<br/>75g"]
    LK -.->|wheel| LKW["l_knee_pitch_wheel 🔄<br/>17g"]

    B -->|r_hip_yaw| RHY["r_hip_yaw<br/>27g"]
    RHY -->|r_hip_roll| RHR["r_hip_roll<br/>118g"]
    RHR -->|r_hip_pitch| RHP["r_hip_pitch<br/>14g"]
    RHP -->|r_knee_pitch| RK["r_knee_pitch<br/>146g"]
    RK -->|r_ankle_pitch| RA["r_ankle_pitch<br/>118g"]
    RA -->|r_foot_roll| RF["r_foot_roll<br/>75g"]
    RK -.->|wheel| RKW["r_knee_pitch_wheel 🔄<br/>17g"]

    B -->|arm_base_yaw| ABY["arm_base_yaw<br/>53g"]
    ABY -->|shoulder_pitch| ASP["arm_shoulder_pitch<br/>105g"]
    ASP -->|elbow_pitch| AEP["arm_elbow_pitch<br/>44g"]
    AEP -->|wrist_pitch| AWP["arm_wrist_pitch<br/>20g"]
    AWP -->|wrist_roll| AWR["arm_wrist_roll<br/>39g"]

    style B fill:#4a90d9,color:#fff
    style LF fill:#2ecc71,color:#fff
    style RF fill:#2ecc71,color:#fff
    style AWR fill:#e74c3c,color:#fff
    style LKW fill:#f39c12,color:#fff
    style RKW fill:#f39c12,color:#fff
```

---
