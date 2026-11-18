B01 Berlin map connection, upstream map commit a34fb4b04843ad8ebdb77c26e4e1929d4f1eb606.
시뮬레이터/RViz/reference/scorer/obstacle_spawner/결과plot에서같은Berlin맵과IQP경로사용. map assets SHA기록/변경감지.
초기1m/s GT consistent_v1,장애물4 seed2026,최대7랩/600초.최적화설정/바이너리변경없음.
공개speed_scaling global_limit .50은파일보존및해시기록.현재고정속도1이므로공개구간별속도배율은drive에미적용.
출발위치는IQP484번점(별도작성start_pose.yaml),교수님공식출발위치아님.
정지상태도메인78에서scan/GT/collision+clearance costmap/reference489점/Berlin600x600map/4장애물/단일drive publisher/controller ready_disabled 검증.입력정렬 distance.0446m,heading약0,selfcost0.
검증중enable발행없고차량이동없음.주행완주성능미검증.기존HYHALL12세션fallback보존.
