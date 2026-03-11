# SolidWorks Export 폴더

이 폴더에 SolidWorks URDF Exporter로 내보낸 **원본 파일**을 넣으세요.

## 구조
```
solidworks_export/
├── urdf/
│   └── <패키지명>.urdf    ← SolidWorks에서 내보낸 URDF
└── meshes/
    └── *.STL              ← SolidWorks에서 내보낸 메시 파일들
```

## 사용법

1. SolidWorks 익스포트 결과물의 `urdf/`와 `meshes/` 폴더를 여기에 **통째로 덮어쓰기**
2. 패키지 루트에서 패치 스크립트 실행:
   ```bash
   cd ~/biped_bike_ws/src/biped_bike_robot
   python3 scripts/patch_urdf.py
   ```
3. 빌드:
   ```bash
   cd ~/biped_bike_ws
   colcon build && source install/setup.bash
   ```

> ⚠️ 이 폴더의 파일을 **직접 수정하지 마세요**.
> 수정이 필요하면 `scripts/patch_urdf.py`에 패치 규칙을 추가하세요.
