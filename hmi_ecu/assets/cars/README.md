# 차량 이미지 폴더

`carla_tools/capture_vehicle_sprites.py` 가 CARLA 차량을 직접 렌더링해서 배경을 투명하게 잘라낸 PNG 를 여기에 저장함.

- `<blueprint id>_rear.png`  : 뒤에서 본 모습 (도로 장면의 앞차/주변차/자차)
- `<blueprint id>_front.png` : 앞에서 본 모습 (마주 오는 차)
- `<blueprint id>_hero.png`  : 앞 3/4 모습 (왼쪽 클러스터 자차)
- `default_*.png`            : 모르는 차종일 때 사용 (캡처 스크립트가 --default 차종으로 자동 생성)

직접 찍은 사진을 써도 됨: 배경 투명 PNG, 차가 이미지 좌우 끝까지 꽉 차게 잘라서 같은 이름으로 넣으면 됨.
이미지가 없으면 HMI 가 기본 도형으로 그림.
