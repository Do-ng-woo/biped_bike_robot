# 🦿 OP3 Walking Engine — 기술 문서

> **op3_walker.py** — ROBOTIS-OP3 보행 엔진을 biped_bike_robot ver4에 맞게 완전 이식 및 최적화한 이족보행 코드

---

## 📐 아키텍처 개요

```
┌─────────────────────────────────────────────────┐
│              OP3WalkerNode (ROS 2)              │
│  ┌───────────┐    ┌──────────────────────────┐  │
│  │WalkingParam│───▶│   OP3WalkingEngine       │  │
│  │ (설정값)   │    │                          │  │
│  └───────────┘    │  1. 4구간 사인파 생성     │  │
│                   │  2. 평행사변형 CoM 이동   │  │
│                   │  3. 6-DOF Analytical IK   │  │
│                   │  4. 관절 각도 출력        │  │
│                   └──────────┬───────────────┘  │
│                              │ 17 DOF angles    │
│                              ▼                  │
│                   /joint_trajectory_controller   │
└─────────────────────────────────────────────────┘
```

| 모듈 | 역할 |
|------|------|
| `WalkingParam` | 보행 파라미터 (보폭, 주기, 발높이, 무게이동 등) |
| `OP3WalkingEngine` | 사인파 기반 궤적 생성 + IK 계산 (OP3 C++ 완전 이식) |
| `analytical_ik_leg()` | 해석적 6-DOF 역기구학 (OP3 `calcInverseKinematicsForLeg` 이식) |
| `OP3WalkerNode` | ROS 2 노드 — 궤적을 `JointTrajectory` 메시지로 퍼블리시 |

---

## 🔑 핵심 원리: 평행사변형 무게중심 이동

이족보행에서 한 발을 들기 위해서는 **무게중심(CoM)이 반드시 지지발 위에 있어야** 합니다.

```
       ◀── y_swap ──▶

  ═══════════════════════  ← 골반 (항상 바닥과 평행!)
           ╱    ╱
     좌다리╱    ╱우다리     ← 두 다리가 평행사변형의 빗변
        ╱    ╱
  ─────╱───╱───────────  ← 지면 (고정)
         [지지발]
```

### 왜 평행사변형인가?

1. **골반 윗면은 무조건 바닥과 평행** — 상체의 CoM이 흔들리면 안 됨
2. **다리만 기울어짐** — hip_roll을 통해 두 다리가 같은 방향으로 기울어지면, 골반은 수평을 유지하면서 좌우로 **병진(translation)** 이동
3. **지면-좌다리-골반-우다리**가 **평행사변형**을 형성

> ⚠️ **절대 하면 안 되는 것**: 발목 롤을 기울여서 CoM을 이동시키는 것.  
> 하체에서 2cm 기울어진 각도가 상체에서는 6cm 이상으로 증폭되어 즉시 넘어짐.

---

## 🏗️ 보행 4구간 (Walking Phases)

한 주기(`period_time`)는 4개의 구간으로 나뉩니다:

```
시간 ──────────────────────────────────────────────▶
     │  DSP₁  │    Left SSP    │  DSP₂  │   Right SSP   │
     │ (양발) │  (왼발 스윙)   │ (양발) │ (오른발 스윙) │
     0       0.15            0.85      1.15           1.85    2.0s
              ▲                         ▲
        CoM → 우측 이동           CoM → 좌측 이동
        (오른발 지지)             (왼발 지지)
```

| 구간 | Phase | 설명 |
|------|-------|------|
| DSP₁ | Phase 0 | 양발 지지, 무게중심 이동 시작 |
| Left SSP | Phase 1 | 왼발 스윙 (오른발 지지), CoM이 오른발 위 |
| DSP₂ | Phase 2 | 양발 지지, 무게중심 반대로 이동 |
| Right SSP | Phase 3 | 오른발 스윙 (왼발 지지), CoM이 왼발 위 |

> **DSP** = Double Support Phase (양발 지지)  
> **SSP** = Single Support Phase (한발 지지)

---

## 📊 사인파 신호 구성

OP3 엔진은 모든 움직임을 **사인파(`_wsin`)의 조합**으로 생성합니다:

```python
_wsin(time, period, phase_shift, magnitude, magnitude_shift)
= magnitude × sin(2π/period × time − phase_shift) + magnitude_shift
```

### Swap (양발 공통 진동) — 골반 움직임

| 신호 | 용도 | 주기 |
|------|------|------|
| `swap_x` | 골반 전후 흔들림 | period / 2 |
| `swap_y` | 골반 좌우 병진 **(평행사변형)** | period |
| `swap_z` | 골반 상하 흔들림 | period / 2 |

### Move (좌/우 독립) — 발 궤적

| 신호 | 용도 | 주기 |
|------|------|------|
| `left/right_x` | 발 전후 스윙 | SSP 시간 |
| `left/right_y` | 발 좌우 이동 | SSP 시간 |
| `left/right_z` | 발 리프트 (들기) | SSP / 2 |
| `left/right_yaw` | 발 회전 | SSP 시간 |

---

## 🤖 이 로봇에 맞춘 핵심 보정 사항

### 1. Y축 반전 (`y_swap_amplitude = -0.040`)

이 로봇의 URDF 좌표계:
- 좌측 힙: `Y = -0.01425` (0에 가까움)
- 우측 힙: `Y = -0.13925` (음수 방향)

OP3와 **Y축 방향이 반대**이므로 `y_swap_amplitude`를 **음수**로 설정해야 지지발 쪽으로 CoM이 이동합니다.

### 2. 전진 방향 반전 (`x_move_amplitude = -0.030`)

이 로봇의 전진 방향은 **-X** (OP3는 +X). 보폭을 음수로 설정.

### 3. 넓은 힙 간격 보상 (`y_swap = 4cm`)

| | OP3 | biped_bike_robot |
|---|-----|-----------------|
| 힙 간격 | ~7cm | **12.5cm** |
| 필요 CoM 이동 | ~2cm | **~4cm** |
| `y_swap_amplitude` | 0.020 | **-0.040** |
| 다리 길이 | ~17cm | **14.7cm** |
| 로봇 질량 | ~3.5kg | **~1.4kg** |

---

## ⚙️ 최종 파라미터 설정

```python
param = WalkingParam()
param.init_x_offset    = -0.010   # 골반 약간 전방 (전도 방지)
param.init_z_offset    =  0.020   # 무릎 살짝 굽혀 충격 흡수
param.period_time      =  2.0     # 보행 주기 2초 (안정적)
param.dsp_ratio        =  0.3     # DSP 비율 30%
param.x_move_amplitude = -0.030   # 보폭 3cm 전진
param.z_move_amplitude =  0.040   # 발 리프트 4cm
param.y_swap_amplitude = -0.040   # 평행사변형 CoM 병진 4cm
param.pelvis_offset    =  5.0°    # 골반 롤 오프셋
param.hip_pitch_offset = 13.0°    # 고관절 전방 기울임
```

---

## 🔧 역기구학 (Inverse Kinematics)

`analytical_ik_leg()` 함수는 OP3의 `calcInverseKinematicsForLeg`을 완전 이식한 **해석적 6-DOF IK**입니다.

**입력**: 발 끝점 `(x, y, z, roll, pitch, yaw)` — 힙 기준 상대 좌표  
**출력**: 6개 관절 각도 `[hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]`

```
힙(Hip) ──── θ₁(yaw) ──── θ₂(roll) ──── θ₃(pitch)
                                           │
                                      허벅지 (5.9cm)
                                           │
                                      θ₄(knee)
                                           │
                                      정강이 (8.775cm)
                                           │
                                  θ₅(ankle_pitch) ──── θ₆(ankle_roll)
                                           │
                                         [발]
```

### 관절 축 방향 보정 (`JOINT_AXIS_DIR`)

SolidWorks 익스포트 URDF의 관절 축 방향이 OP3와 다르므로, IK 출력에 방향 보정을 적용:

```python
JOINT_AXIS_DIR = [
    -1,  1, -1, -1, -1,  1,   # Right leg
    -1,  1,  1, -1, -1,  1,   # Left leg
]
```

---

## 🚀 실행 방법

```bash
# 1. 가제보 실행
ros2 launch biped_bike_robot gazebo.launch.py

# 2. 기본 자세 잡기
ros2 run biped_bike_robot ready_posture.py

# 3. 보행 시작 (1 주기)
ros2 run biped_bike_robot op3_walker.py
```

### 파라미터 튜닝 가이드

| 증상 | 조정할 파라미터 |
|------|----------------|
| 넘어짐 (좌우) | `y_swap_amplitude` 크기 조정 |
| 넘어짐 (전후) | `init_x_offset` 또는 `hip_pitch_offset` |
| 발이 안 떨어짐 | `z_move_amplitude` 증가 또는 `y_swap_amplitude` 크기 증가 |
| 보행 불안정 | `period_time` 증가 (느리게) |
| 발이 끌림 | `z_move_amplitude` 증가 |
| 보폭 조절 | `x_move_amplitude` (음수=전진) |

---

## 📚 참조

- **원본 코드**: ROBOTIS-OP3 `op3_walking_module.cpp` (1151줄 C++)
- **저자**: Kayman (ROBOTIS CO., LTD.)
- **이식 대상**: biped_bike_robot ver4 (ROS 2 Jazzy + Gazebo Harmonic)
- **다리 기구학**: 6-DOF — Hip Yaw → Hip Roll → Hip Pitch → Knee → Ankle Pitch → Foot Roll
