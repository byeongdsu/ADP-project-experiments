B04:SVG-MPPI 비교. B03과속도1.5/GT/지도/출발/장애물4/seed2026/7랩/600초/예측2.2초/softmargin.30및비용5동일.
YAML변경은mpc_mode만. SVG고유기본설정활성화,튜닝없음.
SVG특정공유gradient합산race는별도ws_svg_repro에서작업별저장/고정순서합산으로수정.단일스레드15solve전체control trace유지/4threads15pairedsolve0불일치.수정전4threads15/15불일치.
common MPCBase/비용지도/입력/forward solver는기존consistent_v1과바이트동일.
샘플batch1000이같아도SVGguide계산추가로동일연산량아님.현재팽창지도한계공유,개선보장없음.
평가:0랩벽충돌이었던B03대비충돌/역방향/조향/이격/시간측정,독립반복후판단.
