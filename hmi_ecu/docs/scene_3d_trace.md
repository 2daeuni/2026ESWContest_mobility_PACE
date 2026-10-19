# HMI 3D 런타임 경로 추적

사용자 실주행 로그에서 ego-local 데이터 경로는 정상이며, 차량 body pitch 기반 경사 판정이
실제 오르막을 flat으로 오판정하는 문제가 확인됐다. 좌표 수식은 유지하고 road grade 생성과
slopeMode 입력만 변경했다. 이번 검증은 mock이며 실제 CARLA를 실행하지 않았다.

| 단계 | 위치/자세 데이터 |
| --- | --- |
| bridge `build_packet` | `lane.center3d`, 차량 `local_x/y/z`, `relative_yaw/pitch/roll` 생성. 기존 `local_yaw/pitch/roll` 별칭 유지 |
| JSON / UDP receiver | JSON을 변환 없이 backend `sink("carla", packet, timestamp)`에 전달 |
| backend | 원본을 `_carla_store["data"]`에 저장하고 `scene.build_scene(carla=...)`에 직접 전달 |
| state_merge | 속도/기어/거리 등 UI 상태만 보충. geometry가 통과하는 경로가 아님 |
| scene | local/relative 필드를 보존. `center3d`로 `lines3d`, `road_left3d/right3d` 생성. ego pitch/roll은 관측용으로 보존 |
| Main.qml | `World3D.sc: hmi.state.scene` |
| LeftCluster | `DriveVis3D.world`로 동일한 World3D를 참조 |
| World3D | 위치 `(local_y, local_z + 0, -local_x)`, 회전 `(relative_pitch, -relative_yaw, -relative_roll)` |
| geometry3d | 입력 `(local_x, local_y, local_z)` → 정점 `(local_y, local_z + lift, -local_x)` |

현재 읽은 코드에는 root/camera에 ego pitch/roll을 다시 적용하는 코드가 없었다.
기존 local 위치 경로도 연결되어 있었다. 발견한 계약 차이는 `relative_*` 필드가 없고
`local_*` 자세 별칭만 사용하던 점이며, 이 자체가 기존 실차 증상을 설명한다고 단정하지 않는다.
local 위치가 있는데 상대 자세가 없을 때 absolute pitch/roll로 되돌아가던 경로는 이제 0으로 처리한다.
완전한 구형 패킷은 기존 평면 위치/자세 fallback을 유지한다.

## 로그

bridge PC와 HMI에서 각각 0.5초마다 한 줄을 출력한다. 적용한 프로세스를 다시 시작해야 한다.
브리지 로그는 stdout, HMI QML 로그는 Qt console(stderr)에 출력된다.

- `[3DDBG] stage=bridge`: 송신 성공 후 seq, frame, egoPitch, roadGrade, center25, vehicleId/local 좌표, 객체 수.
- `[3DDBG] stage=qml`: 수신 원본 raw25/rawVehicle, scene center25, 실제 delegate scenePosition(final),
  mode, 수신/scene 차량 수, rawLocal 수, mapRoad/mapBoxes, root/ego/camera transform, has3D.
- `frame=ego_local_3d_v1`: 이번 bridge 표식. `legacy`는 표식이 없는 송신자이며 이전 버전일 수 있다.
- `roadMode=center3d`, `mode=local/relative`: 신규 경로.
- `mode=local/local-alias`: 이전 full-local 버전의 상대 자세 별칭 사용.
- `rawLocal=0`, `roadMode=legacy`: 필드가 도착하지 않거나 구형 패킷인 경우.
- `has3D=0`: Qt Software fallback이며 World3D GPU 표시 경로가 아니다.
- vehicle0는 scene 정렬 후 첫 번째 차량이다. bridge의 첫 차량과 ID가 다를 수 있으므로 ID로 비교한다.
- `roadQml`은 center25의 축 매핑 결과, `final`은 실제 차량 delegate의 scenePosition이다.
  차량 assets는 지면 Y=0으로 제작돼 고정 MODEL_GROUND_OFFSET은 0이다.

도로 visibility는 최종 carla_scene JSON 최상위의 `road_grade_deg`를 동일한 key로 scene/QML에 전달해 결정한다.
이전 `lane.road_grade_deg`는 유지하며 HMI의 구형 패킷 fallback으로만 읽는다.
bridge 로그는 반드시 `packet["road_grade_deg"]`를 그대로 출력한다.
현재 Driving Waypoint의 앞뒤 4m 월드 좌표로 `degrees(atan2(dz, horizontal))`를 계산한다.
도로 끝에서는 가능한 한쪽 구간을 사용하고, 유효한 수평 구간이 없으면 0.0을 사용한다.
역방향 자차는 앞뒤를 바꿔 경사 부호를 맞춘다. 차량 body pitch와 center3d/local_z는 사용하지 않는다.

표시 상태는 `MAP_MODE`와 `ROAD3D_MODE` 두 개다. 초기 상태는 MAP이다.
MAP에서 `abs(road_grade_deg) >= 0.40°`이면 ROAD3D로 들어가며 그 순간의
`ego.z`(CARLA world Z)를 `baseFlatZ`에 저장한다. ROAD3D 동안에는 기준 높이를 바꾸지 않는다.
높은 평탄부에서 grade가 0이 되어도 ROAD3D를 유지한다.

ROAD3D에서 `abs(road_grade_deg) <= 0.15°`와 `abs(ego.z - baseFlatZ) <= 0.20m`가
500ms 연속 유지될 때만 MAP으로 복귀한다. 중간에 grade/높이 조건이 깨지거나
데이터가 누락되면 대기 타이머를 취소한다. 복귀 후 기준 높이는 진단용으로 보존하며,
다음 MAP→ROAD3D 진입 시에만 새 기준 높이를 저장한다. ego world Z는 상태 판정용이며
차량 위치나 geometry에는 추가 적용하지 않는다.

- 평지 + 유효한 맵: 전체 Map 도로/차선/반대 차선/중앙선/교차로를 표시하고,
  scene road surface는 숨긴다. scene 차선은 현재 차선 강조선만 유지한다.
- ROAD3D(높은 평탄부 포함): 평면 Map 도로/차선/큰 바닥/정적 boxes를 숨기고 center3d 도로/차선을 표시한다.
- 동적 차량과 ego는 두 상태 모두 표시하며, Map 전체 root는 끄지 않는다.
- 맵이 없거나 버전이 일치하지 않으면 평지에서도 scene 도로로 fallback한다.

0.5초 QML 로그에 `mode`(MAP/ROAD3D), `egoZ`, `baseFlatZ`, `heightDelta`,
`slopeMode`, `egoPitch`, `roadGrade`, `mapRoadVisible`, `mapLaneVisible`,
`sceneRoad3DVisible`을 포함한다. 예:

```text
[3DDBG] ... egoPitch=0.2 roadGrade=0.1 ... slopeMode=0 ... mapRoadVisible=1 mapLaneVisible=1 sceneRoad3DVisible=0 ...
[3DDBG] ... egoPitch=0.15 roadGrade=6.0 ... slopeMode=1 ... mapRoadVisible=0 mapLaneVisible=0 sceneRoad3DVisible=1 ...
```

실제 로그에서 먼저 송신/수신 frame과 local 필드 존재를 확인하고, raw → scene → final 순서로
같은 차량 ID를 비교한다. 로그 없이 원격 CARLA PC의 실행 파일/버전 불일치를 확정할 수는 없다.
