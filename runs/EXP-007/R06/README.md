# EXP-007/R06 — TLN / PF / 장애물 1개 / 속도 배율 1.00

실행 결과: 3랩 무충돌 완료 후 장애물충돌1회, 벽충돌0회. PF출발검사통과.

- 맵 hyhall12, seed2026, 장애물 1개, 현재 장애물 크기 설정 0.30m
- 모델 TLN_aug.keras, mapping=false, pose_source=pf
- TLN 기본 범위 3.0~6.5m/s 대비 배율 1.00
- v_min=3.0, v_max=6.5 m/s. 나머지 추론 설정은 tln_params.yaml에 고정.
- /speed_scaler의 global_limit은 TLN 속도를 제어하지 않음.
- 공유 launch 기본값과 컨트롤러 소스는 변경하지 않음.
- 시작 1→3→4→2, 종료 1→2→3→4.
- 출발 전 PF 검사. 실제 속도는 GT odometry로 측정.
- 실제 장애물 배치는 /sim/obstacles 및 markers로 대조. 원본 bag 로컬 보존.
- 장애물 1개 조건끼리 속도 비교. R01은 장애물 2개였음.

랩타임9.661/9.970/9.450초, 평균9.694초. scorer기준34.050초에장애물충돌, GT(-2.530,6.479). 목표3랩완료와전체충돌1회구분. 상세bag분석은아직미완료. 다음R06-retry1은동일설정반복.
