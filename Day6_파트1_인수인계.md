# Day6 파트1 인수인계 (센아 → 은경)

## 담당 파일
`patrol_robot/patrol_robot/schedule_manager_node.py`

## 오늘 목표
`schedule_manager_node.py`에 **위험 이벤트 대응 로직**을 추가하고, Nav2 순찰 로직과 통합.

## 구현 내용

### 1. 로봇 현재 위치 추적
- `/amcl_pose` 구독 → `(x, y, yaw)` 형태로 저장 (`_current_pose`)
- 위험 지점 좌표 계산에 사용

### 2. 위험 이벤트 대응
- `/risk_events` 구독 (은경님 `risk_detector_node`가 발행하는 토픽)
- 이벤트 수신 시 처리 흐름:
  1. 현재 순찰 goal `cancel_goal_async()`로 취소
  2. 로봇 현재 위치 + 이벤트의 `distance`/`angle_rad`로 위험 지점의 map 절대좌표 계산 (삼각함수)
     - 로봇 상대각도(`angle_rad`) + 로봇 현재 yaw = 절대각도
     - 장애물 코앞까지 가지 않도록 `SAFETY_MARGIN = 0.3m`만큼 거리에서 빼줌
  3. 계산된 좌표로 새 goal 전송
  4. 도착 후 3초 대기 (확인 시간)
  5. 원래 순찰 스케줄로 복귀 — **중단된 지점부터 이어서 재개** (처음부터 다시 X)

### 3. 순찰 진행 상태 기억 (`_patrol_progress`)
- `(block_id, zone_index, waypoint_index)` 형태로 저장
- 위험 대응으로 순찰이 중단되면 이 상태를 기록해두고, 복귀 시 그 지점부터 이어서 진행
- (초기 버전은 매번 patrol_order 처음부터 순회해서 항상 첫 waypoint에만 머무르는 버그가 있었음 → 오늘 수정)

## 오늘 겪은 버그 & 해결 정리

| 문제 | 원인 | 해결 |
|---|---|---|
| `schedule.json` 로드 실패 (`FileNotFoundError`) | `setup.py`의 `data_files`에 config 폴더 등록 누락 | `(os.path.join('share', package_name, 'config'), glob('config/*.json'))` 추가 |
| 위험지점 goal이 장애물 코앞(또는 안쪽)으로 잡힘 | 좌표 계산 시 `distance`(장애물까지 거리)를 안전여유 없이 그대로 사용 | `SAFETY_MARGIN = 0.3` 적용, `approach_distance = max(distance - SAFETY_MARGIN, 0.05)` |
| `/amcl_pose` 메시지를 계속 못 받음 (`amcl_pose 대기 중` 경고 반복) | QoS 불일치. AMCL 발행 QoS: `RELIABLE + TRANSIENT_LOCAL`, 구독 쪽은 기본값(`VOLATILE`) | 구독 시 AMCL과 동일한 QoS(`TRANSIENT_LOCAL`, depth=1)로 맞춤 |
| 순찰이 항상 첫 waypoint(a_1)에만 머무름 | 위험 대응 후 복귀 시 `run_once()`가 매번 `patrol_order`를 처음부터 순회 | `_patrol_progress` 상태 추가, 중단 지점부터 재개하도록 수정 |
| Gazebo `spawn_entity` 서비스 타임아웃 | WSL2에서 gzserver 초기화가 느려 launch의 30초 제한을 넘김 (서비스 자체는 늦게라도 정상적으로 뜸) | 타임아웃 나면 수동으로 `ros2 run gazebo_ros spawn_entity.py ...` 재실행 |
| `schedule_manager_node` 좀비 프로세스 여러 개 동시 실행 | 재시작 시 이전 터미널 프로세스가 완전히 안 죽고 남음 | 재실행 전 `pkill -9 -f schedule_manager_node`로 정리하는 습관 필요 |

## 아직 남은 이슈 (다음에 같이 봐야 할 것)

1. **`risk_detector_node`의 threshold(현재 0.5m)가 다소 민감함**
   - 좁은 통로를 지나갈 때도 계속 위험 이벤트가 발생해서 순찰이 너무 자주 끊김
   - 제안: threshold를 0.25~0.3m 정도로 낮추거나, 같은 지점에서 반복 트리거되는 걸 막는 디바운스/쿨다운 로직 추가
2. **`night` time_block(18:00~08:00) 처리 로직 없음**
   - 문자열 비교(`start_time <= now < end_time`)라 자정 넘는 구간에서 실패
3. **`schedule.json`의 `operation` 블록 `end_time`을 테스트용으로 `23:59`로 임시 확장해둔 상태**
   - 실제 배포 전에 원래 값(`16:00`)으로 되돌려야 함

## 확인된 정상 동작
- 위험 이벤트 수신 → goal 취소 → 위험지점 이동 → 3초 대기 → 복귀 → 다음 waypoint로 진행하는 전체 사이클이 통합 테스트에서 정상 작동 확인됨 (WSL2 + Gazebo Classic + Nav2 환경)

## 최신 코드 위치
https://github.com/se-ny/patrol-robot/tree/day6-risk-response
(병합 저장소 `k-eungyeong/safety-patrol-robot` 기준)
