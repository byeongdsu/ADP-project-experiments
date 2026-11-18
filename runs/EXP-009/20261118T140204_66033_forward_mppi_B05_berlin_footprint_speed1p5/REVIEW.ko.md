# B05 분석 — 2026-11-18

기본 MPPI footprint_v1, Berlin/GT/fixed1.5m/s/4obstacles/seed2026/7lap target/600s. bag metadata 있음, recorder 및 simulator/controller 종료 확인. raw bag 변경/재색인 안 함. 종료 후 speed0, stop20회 확인.

## 결과

첫 랩33.049초 무충돌. 두 번째 랩 벽 충돌로 종료, 목표7랩 실패. runner 시작 상대 약51.135초, 누적76.225m. GT 충돌좌표(-.4041,-2.6012). obstacle 직접 충돌0, wall1 (simulator iTTC beam label이며 별도 rectangle intersection으로 검증한 판정 아님).

B03기존MPPI:0랩/18.704초 벽충돌, B04기존SVG:0랩/18.990초 벽충돌. B05는 첫배치 네 장애물이 B03와 동일하며 첫랩통과. 단1회이므로 충돌검사 변경이 안정적으로 해결했다고 단정하지 않음. 기존 baseline/수정판에서 여유 거리 계산과 grid외 차체 처리도 변경됐으므로 yaw반영 하나의 기여로 분리 불가. 첫랩후 실제장애물배치변경확인: 두번째랩 첫코너부근 장애물중심(-1.4614,-3.8818), yaw -2.173rad. 첫랩 같은구간 장애물중심(-1.9786,-3.9280)과 다름. 따라서 첫랩 성공과 두번째랩 실패는 같은 장애물배치 반복이 아님.

## 실패 직전

충돌 -2초 후보충돌비율 .975, -1초~0초 표본1.0. 마지막2초 평균 .99186. 이 비율은 horizon내 어느점에서든 양의 충돌비용이 있는 후보 비중이며 실제 충돌확률99%의 뜻이 아님. 실제/명령속도1.5유지, 장애물옆통과후 왼쪽기준경로 굽이를 따르지못하고 바깥벽으로 접근. 조향명령 -0.372에서+.323 등 큰반전, 선택예측도 벽방향으로 나감. scan/costmap 최근성 정상(최종costmap수신약26ms전), 장애물배치수신약.737초전이며 layout변경은17.58초전. 자동감속/충돌후보 hard reject 없음이 코드와 기록에서 확인됨. 후보 자체가 부족한지, 비용절충/평균화 영향이 얼마인지는 별도검증 필요. 연산지연만으로 실패원인을 단정할 근거없음.

## 시간

solver mean13.125ms/p9520.339ms/max32.614ms. 전체cycle mean14.922ms/p9522.302ms/max37.056ms,25ms초과3.082%. 새footprint cache 구축은solver timer밖이며 전체cycle에 포함. 이전실행과 관찰구간/길이가 다르므로 평균차이를 같은장면의계산증가량으로 단정금지. 입력감사 input_audit.json은 기존legacy collision_layer/scan에대한기하일관성, raw footprint 안전성을증명하는것아님.

## 다음

공통footprint_v1로 SVG도 같은Berlin1.5m/s/실제첫배치/GT/seed2026 조건에 실행해야 공통비용수정후 알고리즘 비교가 됨. 기존B04를새B05의공정SVG대조로사용하지않음. SVG가후보탐색을개선해도자동감속을추가하는것은아님. 이후규칙기반위험감속/가변속도인터페이스를마련하고PPO속도선택과비교. 오늘분석으로새주행/파라미터변경/학습시작안함.

그림 collision_detail.png, 원본JSON collision_detail.json, 비교표 comparison_after_run.json. 원본bag보존.
