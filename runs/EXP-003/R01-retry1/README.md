# EXP-003/R01-retry1 — PP / GT / 배율 0.25 / 장애물 1개

2026-10-21, hyhall12, seed 2026. 시작 로그 global_limit=0.25 확인. spawner 로그에서 한 변 0.30m 장애물 매 랩 1개 확인. 배치는 랩마다 변경된다.

결과: 3연속 랩 무충돌. 랩타임 25.231 / 25.138 / 25.550초, 평균 25.306초. 벽·장애물 충돌 0회, teleport 0회.

원본 telemetry와 결과 JSON 보존. bag에는 /sim/obstacles 및 /sim/obstacles/markers가 없어 실제 전체 배치 좌표·방향은 직접 기록되지 않았다. 장애물 후보 ID와 조건은 obstacle_spawner.log에 보존했다. 실제 추종 오차는 아직 분석하지 않았다. 연속 3랩은 독립 반복 3회가 아니다.
