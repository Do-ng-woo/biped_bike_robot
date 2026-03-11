#!/usr/bin/env python3
"""
SolidWorks URDF 패치 스크립트

SolidWorks에서 익스포트한 원본 URDF를 ROS 2 + Gazebo 호환으로 패치합니다.

사용법:
    cd ~/biped_bike_ws/src/biped_bike_robot
    python3 scripts/patch_urdf.py

작동 방식:
    1. solidworks_export/ 에서 원본 URDF와 meshes를 읽음
    2. 패치 적용 (joint 이름, rpy 보정, 줄바꿈 등)
    3. urdf/ 와 meshes/ 에 결과 저장
    4. launch 파일, 제어 코드 등은 전혀 건드리지 않음
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
# joint 이름 기준으로 rpy를 수정
HIP_ROLL_RPY_PATCHES = {
    "l_hip_roll_joint": {"from": "0 -1.5708 0", "to": "0 -1.5708 3.14159"},
    "r_hip_roll_joint": {"from": "0 -1.5708 0", "to": "0 -1.5708 3.14159"},
}


def find_export_urdf():
    """solidworks_export/urdf/ 에서 URDF 파일 찾기"""
    urdf_dir = os.path.join(EXPORT_DIR, "urdf")
    if not os.path.isdir(urdf_dir):
        # urdf 하위 폴더가 없으면 export 루트에서 직접 찾기
        urdf_files = glob.glob(os.path.join(EXPORT_DIR, "*.urdf"))
    else:
        urdf_files = glob.glob(os.path.join(urdf_dir, "*.urdf"))

    if not urdf_files:
        print(f"❌ URDF 파일을 찾을 수 없습니다: {EXPORT_DIR}")
        sys.exit(1)

    return urdf_files[0]


def find_export_meshes():
    """solidworks_export/meshes/ 에서 STL 파일들 찾기"""
    mesh_dir = os.path.join(EXPORT_DIR, "meshes")
    if not os.path.isdir(mesh_dir):
        print(f"⚠️  meshes 디렉토리 없음: {mesh_dir}")
        return []
    return glob.glob(os.path.join(mesh_dir, "*.STL")) + \
           glob.glob(os.path.join(mesh_dir, "*.stl"))


def patch_crlf(content):
    """[패치 1] CRLF → LF 변환"""
    return content.replace('\r\n', '\n').replace('\r', '\n')


def patch_package_name(content):
    """[패치 2] package:// URI를 메인 패키지명으로 변경"""
    # package://어쩌구_ver1/meshes/ → package://biped_bike_robot/meshes/
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
    print(f"🔧 SolidWorks URDF 패치 시작")
    print(f"   원본: {EXPORT_DIR}")
    print(f"   출력: {PKG_ROOT}")
    print("=" * 50)

    # 원본 URDF 읽기
    urdf_path = find_export_urdf()
    print(f"\n📄 원본 URDF: {os.path.basename(urdf_path)}")

    with open(urdf_path, 'r', encoding='utf-8', errors='replace') as f:
        content = f.read()

    # 패치 순서대로 적용
    print("\n[1/4] CRLF → LF 변환...")
    content = patch_crlf(content)
    print("  ✅ 완료")

    print("[2/4] package:// URI 변경...")
    content = patch_package_name(content)
    print(f"  ✅ → package://{PACKAGE_NAME}/meshes/")

    print("[3/4] Joint 이름 충돌 해결 (_jnt 접미사)...")
    content = patch_joint_names(content)
    print("  ✅ 완료")

    print("[4/4] 고관절 롤 rpy 보정...")
    content = patch_hip_roll_rpy(content)
    for name, patch in HIP_ROLL_RPY_PATCHES.items():
        print(f"  ✅ {name}: rpy {patch['from']} → {patch['to']}")

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
        print("⚠️  메시 파일 없음 (수동으로 meshes/에 넣어주세요)")

    print("\n" + "=" * 50)
    print("✅ 패치 완료!")
    print(f"   colcon build && source install/setup.bash")
    print(f"   ros2 launch {PACKAGE_NAME} display.launch.py")
    print("=" * 50)


if __name__ == "__main__":
    main()
