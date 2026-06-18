#!/usr/bin/env python3
"""
SolidWorks URDF 패치 스크립트 (ver3 대응)

SolidWorks에서 익스포트한 원본 URDF를 ROS 2 + Gazebo 호환으로 패치합니다.

사용법:
    cd ~/biped_bike_ws/src/biped_bike_robot
    python3 scripts/patch_urdf.py

작동 방식:
    1. solidworks_export/ 또는 urdf/ 에서 원본 URDF를 읽음
    2. 패치 적용 (package 경로, joint 이름, rpy 보정, 모터 속성 등)
    3. urdf/ 와 meshes/ 에 결과 저장
    4. launch 파일, 제어 코드 등은 전혀 건드리지 않음

ver3 변경사항:
    - 다리: 6 DOF (hip_yaw 추가) — l/r_hip_yaw, l/r_hip_roll, l/r_hip_pitch,
      l/r_knee_pitch, l/r_ankle_pitch, l/r_foot_roll
    - 팔: 5 DOF — arm_base_yaw, arm_shoulder_pitch, arm_elbow_pitch,
      arm_wrist_pitch, arm_wrist_roll
    - 패시브 휠: l/r_knee_pitch_wheel, arm_wheel_pitch_
"""

import os
import re
import shutil
import glob
import sys

# ===== 설정 =====
PACKAGE_NAME = "biped_bike_robot"

# 이 스크립트의 위치 기준으로 패키지 루트 결정
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(SCRIPT_DIR)  # scripts/ 의 상위

EXPORT_DIR = os.path.join(PKG_ROOT, "solidworks_export")
OUTPUT_URDF_DIR = os.path.join(PKG_ROOT, "urdf")
OUTPUT_MESH_DIR = os.path.join(PKG_ROOT, "meshes")


# ===== 패치 규칙 정의 =====
# 여기에 새로운 패치를 추가하면 다음 익스포트부터 자동 적용됩니다.

# 고관절 롤 rpy 보정 (SolidWorks 좌표계 설정 문제)
# ver3: joint 이름이 l_hip_roll, r_hip_roll 로 변경됨
HIP_ROLL_RPY_PATCHES = {
    "l_hip_roll": {"from": "0 -1.5708 0", "to": "0 -1.5708 3.14159"},
    "r_hip_roll": {"from": "0 -1.5708 0", "to": "0 -1.5708 3.14159"},
}


def find_export_urdf():
    """solidworks_export/ 또는 urdf/ 에서 URDF 파일 찾기"""
    # 먼저 solidworks_export/urdf/ 확인
    urdf_dir = os.path.join(EXPORT_DIR, "urdf")
    if os.path.isdir(urdf_dir):
        urdf_files = glob.glob(os.path.join(urdf_dir, "*.urdf"))
        if urdf_files:
            return urdf_files[0]

    # solidworks_export 루트 확인
    urdf_files = glob.glob(os.path.join(EXPORT_DIR, "*.urdf"))
    if urdf_files:
        return urdf_files[0]

    # urdf/ 디렉토리에서 원본 찾기 (사용자가 직접 넣은 경우)
    urdf_files = glob.glob(os.path.join(OUTPUT_URDF_DIR, "*ver*.urdf"))
    if urdf_files:
        return urdf_files[0]

    print(f"❌ URDF 파일을 찾을 수 없습니다: {EXPORT_DIR} 또는 {OUTPUT_URDF_DIR}")
    sys.exit(1)


def find_export_meshes():
    """solidworks_export/meshes/ 또는 메인 meshes/ 에서 STL 파일들 찾기"""
    # solidworks_export/meshes/ 우선 확인
    mesh_dir = os.path.join(EXPORT_DIR, "meshes")
    if os.path.isdir(mesh_dir):
        stls = glob.glob(os.path.join(mesh_dir, "*.STL")) + \
               glob.glob(os.path.join(mesh_dir, "*.stl"))
        if stls:
            return stls

    # 이미 메인 meshes/ 에 있으면 복사 불필요
    stls = glob.glob(os.path.join(OUTPUT_MESH_DIR, "*.STL")) + \
           glob.glob(os.path.join(OUTPUT_MESH_DIR, "*.stl"))
    if stls:
        print("  ℹ️  메시 파일이 이미 meshes/ 에 있습니다 (복사 건너뜀)")
        return []

    print(f"⚠️  meshes 디렉토리 없음: {mesh_dir}")
    return []


def patch_crlf(content):
    """[패치 1] CRLF → LF 변환"""
    return content.replace('\r\n', '\n').replace('\r', '\n')


def patch_package_name(content):
    """[패치 2] package:// URI를 메인 패키지명으로 변경"""
    # package://어쩌구_ver3/meshes/ → package://biped_bike_robot/meshes/
    return re.sub(
        r'package://[^/]+/meshes/',
        f'package://{PACKAGE_NAME}/meshes/',
        content
    )


def patch_joint_names(content):
    """[패치 3] joint 이름 충돌 해결 (Gazebo SDF 호환)

    SolidWorks 익스포터가 joint와 child link에 동일한 이름을 부여하는 문제 해결.
    <joint> 태그의 name 속성에만 _jnt 접미사를 추가.
    <link>, <parent>, <child> 태그는 건드리지 않음.
    """
    def replace_joint_name(match):
        indent = match.group(1)
        joint_name = match.group(2)
        rest = match.group(3)
        return f'{indent}<joint\n{indent}  name="{joint_name}_jnt"\n{indent}  {rest}'

    # <joint\n    name="X"\n    type="..."> 패턴 매칭
    pattern = r'(\s*)<joint\s*\n\s*name="([^"]+)"\s*\n\s*(type="[^"]+")'
    return re.sub(pattern, replace_joint_name, content)


def patch_hip_roll_rpy(content):
    """[패치 4] 고관절 롤 rpy 보정

    SolidWorks 익스포트 시 좌표계 설정 문제로 다리가 180도 뒤집혀 나오는 문제 수정.
    joint 이름을 기준으로 해당 joint의 origin rpy를 수정.
    패치 3 이후에 실행되므로 _jnt 접미사가 붙은 이름으로 검색.
    """
    for joint_base_name, rpy_patch in HIP_ROLL_RPY_PATCHES.items():
        joint_name = f"{joint_base_name}_jnt"  # 패치 3에서 _jnt가 추가됨
        old_rpy = rpy_patch["from"]
        new_rpy = rpy_patch["to"]

        # joint 이름 뒤에 나오는 rpy 값을 찾아서 교체
        pattern = (
            f'(name="{re.escape(joint_name)}".*?'
            f'rpy="){re.escape(old_rpy)}(")'
        )
        content = re.sub(pattern, rf'\g<1>{new_rpy}\g<2>', content, flags=re.DOTALL)

    return content


def patch_joint_limits_and_dynamics(content):
    """[패치 5] Dynamixel XL430-W250-T 모터 속성 및 동역학 적용
    
    - effort: 1.5 (N.m), velocity: 6.38 (rad/s), 위치 범위: -pi ~ pi
    - 시뮬레이션 안정성을 위한 dynamics (damping=0.1, friction=0.1) 추가
    """
    # 1. 기존 Revolute 관절의 limit 속성을 치환하고 dynamics 태그 추가
    pattern_revolute = r'<limit\s+lower="0"\s+upper="0"\s+effort="0"\s+velocity="0"\s*/>'
    replacement_revolute = (
        '<limit\n'
        '      lower="-3.14159"\n'
        '      upper="3.14159"\n'
        '      effort="1.5"\n'
        '      velocity="6.38" />\n'
        '    <dynamics\n'
        '      damping="0.1"\n'
        '      friction="0.1" />'
    )
    content = re.sub(pattern_revolute, replacement_revolute, content)

    # The shoulder link mechanically interferes beyond about 25 degrees backward.
    shoulder_pattern = (
        r'(<joint\s+name="arm_shoulder_pitch_jnt".*?<limit\s+lower=")'
        r'-3\.14159(")'
    )
    content = re.sub(
        shoulder_pattern,
        r'\g<1>-0.436332\g<2>',
        content,
        flags=re.DOTALL,
    )

    # 2. Continuous 관절은 limit이 아예 없으므로 </joint> 앞에 삽입
    def replace_continuous(match):
        joint_body = match.group(0)
        insert = (
            '  <limit\n'
            '      effort="1.5"\n'
            '      velocity="6.38" />\n'
            '    <dynamics\n'
            '      damping="0.1"\n'
            '      friction="0.1" />\n'
            '  </joint>'
        )
        return joint_body.replace('</joint>', insert)

    pattern_continuous = r'<joint[^>]*type="continuous">.*?</joint>'
    content = re.sub(pattern_continuous, replace_continuous, content, flags=re.DOTALL)
    
    return content

def patch_self_collision(content):
    """[패치 6] 가제보 내 링크 간 자가 충돌(Self-collision) 활성화
    
    모든 <link name="..."> 태그에 해당하는 Gazebo <self_collide>true</self_collide> 속성 부여
    """
    links = re.findall(r'<link\s+name="([^"]+)">', content)
    
    gazebo_tags = "\n".join([
        f'  <gazebo reference="{link}">\n'
        f'    <self_collide>true</self_collide>\n'
        f'  </gazebo>' for link in links
    ])
    
    # </robot> 태그 직전에 삽입
    content = content.replace('</robot>', f'{gazebo_tags}\n</robot>')
    return content

def patch_ros2_control(content):
    """[패치 7] ros2_control 제어기 및 Gazebo 플러그인 추가

    모든 작동하는 관절(revolute, continuous)을 ros2_control 하드웨어 인터페이스로 등록하고,
    Gazebo Harmonic (gz_ros2_control) 시스템 플러그인을 활성화합니다.
    """
    # 동작하는 조인트 추출
    joints = re.findall(r'<joint\s*name="([^"]+)"\s*type="(revolute|continuous)">', content)
    
    ros2_control_xml = [
        '  <ros2_control name="GazeboSystem" type="system">',
        '    <hardware>',
        '      <plugin>gz_ros2_control/GazeboSimSystem</plugin>',
        '    </hardware>'
    ]

    for joint_name, _ in joints:
        ros2_control_xml.extend([
            f'    <joint name="{joint_name}">',
            '      <command_interface name="position"/>',
            '      <state_interface name="position">',
            '        <param name="initial_value">0.0</param>',
            '      </state_interface>',
            '      <state_interface name="velocity"/>',
            '      <state_interface name="effort"/>',
            '    </joint>'
        ])
    
    ros2_control_xml.append('  </ros2_control>')

    # 가제보 플러그인 설정
    ros2_control_xml.extend([
        '  <gazebo>',
        '    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">',
        '      <parameters>$(find biped_bike_robot)/config/controllers.yaml</parameters>',
        '    </plugin>',
        '  </gazebo>'
    ])

    ros2_control_xml_str = "\n".join(ros2_control_xml)
    
    content = content.replace('</robot>', f'{ros2_control_xml_str}\n</robot>')
    return content

def copy_meshes(mesh_files):
    """메시 파일을 메인 패키지의 meshes/로 복사"""
    os.makedirs(OUTPUT_MESH_DIR, exist_ok=True)

    # 기존 STL 파일 정리
    for old_stl in glob.glob(os.path.join(OUTPUT_MESH_DIR, "*.STL")) + \
                   glob.glob(os.path.join(OUTPUT_MESH_DIR, "*.stl")):
        os.remove(old_stl)

    copied = 0
    for mesh_file in mesh_files:
        basename = os.path.basename(mesh_file)
        dest = os.path.join(OUTPUT_MESH_DIR, basename)
        shutil.copy2(mesh_file, dest)
        copied += 1

    return copied


def main():
    print("=" * 50)
    print(f"🔧 SolidWorks URDF 패치 시작 (ver3)")
    print(f"   원본: {EXPORT_DIR}")
    print(f"   출력: {PKG_ROOT}")
    print("=" * 50)

    # 원본 URDF 읽기
    urdf_path = find_export_urdf()
    print(f"\n📄 원본 URDF: {urdf_path}")

    with open(urdf_path, 'r', encoding='utf-8', errors='replace') as f:
        content = f.read()

    # 패치 순서대로 적용
    print("\n[1/7] CRLF → LF 변환...")
    content = patch_crlf(content)
    print("  ✅ 완료")

    print("[2/7] package:// URI 변경...")
    content = patch_package_name(content)
    print(f"  ✅ → package://{PACKAGE_NAME}/meshes/")

    print("[3/7] Joint 이름 충돌 해결 (_jnt 접미사)...")
    content = patch_joint_names(content)
    print("  ✅ 완료")

    print("[4/7] 고관절 롤 rpy 보정...")
    content = patch_hip_roll_rpy(content)
    for name, patch in HIP_ROLL_RPY_PATCHES.items():
        print(f"  ✅ {name}: rpy {patch['from']} → {patch['to']}")

    print("[5/7] 다이나믹셀 모터 속성 및 동역학 적용...")
    content = patch_joint_limits_and_dynamics(content)
    print("  ✅ limit, dynamics (XL430-W250-T) 추가 완료")

    print("[6/7] 가제보 자가 충돌(Self-collide) 활성화...")
    content = patch_self_collision(content)
    print("  ✅ 모든 링크에 <self_collide>true</self_collide> 추가 완료")

    print("[7/7] ros2_control 플러그인 추가...")
    content = patch_ros2_control(content)
    print("  ✅ 컨트롤러 설정(hardware interface 및 plugin) 추가 완료")

    # 패치된 URDF 저장
    os.makedirs(OUTPUT_URDF_DIR, exist_ok=True)
    output_urdf = os.path.join(OUTPUT_URDF_DIR, f"{PACKAGE_NAME}.urdf")
    with open(output_urdf, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"\n💾 패치된 URDF 저장: {output_urdf}")

    # 메시 파일 복사
    mesh_files = find_export_meshes()
    if mesh_files:
        copied = copy_meshes(mesh_files)
        print(f"📦 메시 파일 {copied}개 복사 완료")
    else:
        print("ℹ️  메시 파일 복사 건너뜀")

    print("\n" + "=" * 50)
    print("✅ 패치 완료!")
    print(f"   colcon build && source install/setup.bash")
    print(f"   ros2 launch {PACKAGE_NAME} gazebo.launch.py")
    print("=" * 50)


if __name__ == "__main__":
    main()
