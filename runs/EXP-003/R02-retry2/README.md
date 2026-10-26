# EXP-003/R02-retry2 — PP / GT / 0.25 / 장애물 2개

결과: 0랩, 충돌0회, teleport0회. 장애물 앞 정지가 재현됐다. 고정 GT 출발 pose와 초기 속도0 확인. 장애물 좌표는 R02와 동일.

/state에서 GB_TRACK → TRAILING → OVERTAKE → TRAILING 전환 확인. 마지막 TRAILING 진입은 첫 drive 이후 약16.55초. 이후 drive 속도0, GT 정지. 로컬 경로에는 약1.30m/s 목표속도가 존재한다. stop_diagnostics.json에 전환 시각과 정지 수치를 보존했다.

정지 상태가 TRAILING임은 확인됐다. 소스상 정적 장애물에 대한 간격 제어가 속도를0으로 제한할 수 있다. OVERTAKE 종료 원인은 회피 경로 유효성 또는 점유 판정 후보이나 관련 내부 입력 미기록으로 확정하지 않는다. /trailing/gap_data는 기록 대상으로 등록됐지만 표본0개다. FTG fallback은 소스상 ftg_disabled=True. 파라미터 변경 없음.

사용자가 촬영 완료를 알림. 영상 파일은 아직 전달받지 않아 영상 분석은 하지 않았다. 원본 bag과 장애물 토픽 보존.

## 후속 원인 확인
추가로 보존한 state_machine.log에서 마지막 전환 시각의 O_FREE False 확인. 두 번째 장애물이 회피 경로의 점유 기준에 들어간 것이 전환 원인이다. 상세: ../../../analyses/EXP-003/REPORT.md. 앞 절의 미확정 판단을 이 확인 결과로 갱신한다.
