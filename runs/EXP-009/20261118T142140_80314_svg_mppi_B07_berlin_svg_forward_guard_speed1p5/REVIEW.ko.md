# B07 주행 분석 — 2026-11-18

SVG/forward_guard_v1/Berlin/GT/nominal1.5m/s/4obs/seed2026/7lap target/600s. 종료 reason guard_stop, status no_collision_free_forward_candidate. 0랩/충돌0/누적10.264m/관측9.071초. 구동시작6.967초후guard,출발후반대방향관측없음. 이전B06의U턴위치(진행약25m)보다훨씬이른첫장애물구간(약10m)에서정지했으므로U턴장면을정상회피했다고말할수없음. 이번결과는거부/정지경로검증이지회피성공/완주아님.

## 정지

bag guard수신GT약(2.67934,-18.60536),직전GT수신0.2ms전속도1.5. guard후|v|<.05까지수신시간약.2314초/이동거리근사.1273m. 최종(2.53555,-18.61241),종료후속도2.27e-16. 참조방향90도초과없음(direction_detail.json, 전체일부정지브레이크구간을제외한비영명령구간분석). 지도가없거나초기방향오류때문이아님. 첫장애물중심(2.9128,-18.2891)부근. rawbagmetadata정상/활성프로세스없음/rawbag수정안함.

정지사유는충돌없음+정방향조건을동시에통과한표본이없다는뜻. 현재기록만으로모든후보가충돌때문인지방향때문인지개별건수는분리불가. 지도에통로가없다는증거아님. 1000개최종표본에서유효해부족/2.2초전체hard검사보수성/현재자세와속도/계산지연등이가능. 이버전은정지후낮은속도로재계획하거나자동복구하는기능이없어latch상태로종료하는설계. 실패상황이해결됐다고표현금지.

## 시간 회귀

solver n50 mean125.698ms/p95220.891/max246.058. cycle n51 mean135.031ms/p95231.388/max254.253,25ms초과100%. 이전B06 cyclemean24.119/p9537.616초과34.43%와달리새버전이훨씬느림. 관찰구간과선택궤적이다르나새검증경로에서큰비용증가가확인됨. 이전에검증추가의실시간비용을충분히확인하지못했고회귀로별도기록한다. 이결과를원본SVG의고유연산한계나SVG개선필요성증거로합치지않는다.

읽기전용 빌드검사: core는-O3, node는최적화옵션없음. check_forward/OrientedFootprint::cost가header inline weak symbol로양쪽에서사용되며node의비최적화구현이실제바이너리에포함. 동일O3core library와동일고정입력에main TU만O0/O3로바꾼진단실행에서 wall1.568s대.685s,출력trace동일여부linkage_full_benchmark.json참조. 원래실험과다른빈rawmap의한번씩대조이므로실주행5배악화전체를이원인으로확정/비율일반화금지. diagnostics/forward_guard_20261118/benchmark_linkage_full.py/linkage_full_benchmark.json. 실험binary/source는변경안함. 그외CPU자원기록보존.

## 다음 권장

추가속도상향/학습/같은조건반복전에새검증구현의빌드/호출경로회귀를고치고offline동일입력으로비용/출력동등성검증. 이어서정지직전실제입력에서정방향불합격/충돌불합격을분리계측하고,모델과실제목표속도를맞춘저속재계획에유효후보가생기는지확인. hard조건을무작정꺼서역주행허용으로돌아가지않음. 낮은속도로회피가가능한지확인한뒤규칙감속과PPO목표속도선택으로연결. 이번사용자요청은분석이며새주행/설정/실행binary변경없음.

그림direction_detail.png,방향JSONdirection_detail.json,정지guard_stop_detail.json,원본summary.json. 기존B05/B06보존. GitHub게시안함.
