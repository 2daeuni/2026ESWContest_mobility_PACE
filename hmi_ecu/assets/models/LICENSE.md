# 3D 모델 출처

- 자차 / 주변 차 모델: "Car Concept" — Khronos glTF Sample Assets
  - © 2024 Darmstadt Graphics Group GmbH (Eric Chadwick), [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)
  - 원본: https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/CarConcept
  - 변경: 실내·번호판(로고) 제거, 재질별 분리, 주변 차용 단순화 (`tools/build_vehicle_models.py`)
- 트럭 / 버스 / 이륜차 / 보행자: `tools/build_vehicle_models.py` 가 생성한 단순 형상 (자체 제작)
- 환경 조명 `assets/env/*.hdr`, 접지 그림자, 차선 점선 텍스처: 자체 생성 (`tools/make_env_hdr.py`)

다른 차 모델로 바꾸기: 원하는 `.glb` 를 받아서
`python3 tools/build_vehicle_models.py --car 자차.glb --traffic 주변차.glb` 실행 → 이 폴더의 `.mesh` 가 교체됨
