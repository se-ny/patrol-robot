"""
risk_logic.py
ROS2/로봇 의존성 없는 순수 위험감지 로직.
Day4에서 risk_detector_node.py가 이 함수들을 그대로 가져다 씀.
"""

import math
from dataclasses import dataclass


@dataclass
class RiskEvent:
    distance: float
    angle_rad: float
    zone_id: str | None = None

    def to_dict(self) -> dict:
        return {
            "type": "obstacle_proximity",
            "distance": round(self.distance, 3),
            "angle_rad": round(self.angle_rad, 3),
            "zone_id": self.zone_id,
        }


def filter_valid_ranges(
    ranges: list[float],
    range_min: float = 0.0,
    range_max: float = float("inf"),
) -> list[float]:
    """
    inf, nan, 0 이하 값 제거 (라이더 노이즈/무효값 처리)
    range_min/range_max를 지정하면 센서 스펙 밖의 값(자기 몸체 반사 등으로
    비정상적으로 가까운 값 등)도 함께 걸러낸다.
    """
    return [
        r
        for r in ranges
        if not math.isinf(r) and not math.isnan(r) and range_min <= r <= range_max
    ]


def find_min_range(
    ranges: list[float],
    range_min: float = 0.0,
    range_max: float = float("inf"),
) -> tuple[float, int] | None:
    """유효 범위 중 최소 거리와 그 인덱스를 반환. 유효값 없으면 None."""
    valid = filter_valid_ranges(ranges, range_min=range_min, range_max=range_max)
    if not valid:
        return None
    min_val = min(valid)
    # 원본 ranges 기준 인덱스를 찾아야 각도 계산이 맞음
    min_index = ranges.index(min_val)
    return min_val, min_index


def detect_risk(
    ranges: list[float],
    angle_min: float,
    angle_increment: float,
    threshold: float = 0.5,
    zone_id: str | None = None,
    sensor_range_min: float = 0.0,
    sensor_range_max: float = float("inf"),
) -> RiskEvent | None:
    """
    라이더 스캔 결과에서 임계값보다 가까운 장애물이 있으면 RiskEvent 반환.
    없으면 None.
    sensor_range_min/max: 센서 스펙(LaserScan.range_min/range_max) 기준
    유효하지 않은 값(자기 몸체 반사 등)은 오탐 방지를 위해 무시한다.
    """
    result = find_min_range(ranges, range_min=sensor_range_min, range_max=sensor_range_max)
    if result is None:
        return None

    min_range, min_index = result
    if min_range >= threshold:
        return None

    angle = angle_min + min_index * angle_increment
    return RiskEvent(distance=min_range, angle_rad=angle, zone_id=zone_id)


# 실제 TurtleBot3 burger LDS 센서(ros2 topic echo /scan --once로 확인) 파라미터
REAL_ANGLE_MIN = 0.0
REAL_ANGLE_INCREMENT = 0.017492700368165970  # 360도 / 약 360개 샘플
REAL_RANGE_MIN = 0.12
REAL_RANGE_MAX = 3.5
REAL_SAMPLE_COUNT = 360


def make_all_inf_scan(count: int = REAL_SAMPLE_COUNT) -> list[float]:
    """빈 공간(장애물 없음)을 흉내낸 실전 스타일 스캔 데이터"""
    return [float("inf")] * count


def make_scan_with_obstacle(
    obstacle_index: int, obstacle_distance: float, count: int = REAL_SAMPLE_COUNT
) -> list[float]:
    """실전처럼 대부분 inf인데 특정 인덱스만 가까운 장애물이 있는 스캔"""
    ranges = make_all_inf_scan(count)
    ranges[obstacle_index] = obstacle_distance
    return ranges


if __name__ == "__main__":
    # 목업 테스트 케이스들 (실전 파라미터 기준)
    test_cases = [
        {
            "name": "정상 (빈 월드, 전부 inf)",
            "ranges": make_all_inf_scan(),
            "expect_risk": False,
        },
        {
            "name": "위험 (정면 90도 지점에 0.3m 장애물)",
            "ranges": make_scan_with_obstacle(obstacle_index=90, obstacle_distance=0.3),
            "expect_risk": True,
        },
        {
            "name": "임계값 경계값 (0.5m 정확히, 위험 아님)",
            "ranges": make_scan_with_obstacle(obstacle_index=180, obstacle_distance=0.5),
            "expect_risk": False,
        },
        {
            "name": "range_min보다 가까운 값 (0.05m, 센서 스펙 밖 → 오탐 방지로 무시)",
            "ranges": make_scan_with_obstacle(obstacle_index=45, obstacle_distance=0.05),
            "expect_risk": False,
        },
        {
            "name": "range_min 직후 유효값 (0.15m, 진짜 위험)",
            "ranges": make_scan_with_obstacle(obstacle_index=45, obstacle_distance=0.15),
            "expect_risk": True,
        },
    ]

    print("=== 위험감지 로직 목업 테스트 (실전 파라미터 기준) ===\n")
    for tc in test_cases:
        event = detect_risk(
            ranges=tc["ranges"],
            angle_min=REAL_ANGLE_MIN,
            angle_increment=REAL_ANGLE_INCREMENT,
            threshold=0.5,
            zone_id="zone_a",
            sensor_range_min=REAL_RANGE_MIN,
            sensor_range_max=REAL_RANGE_MAX,
        )
        got_risk = event is not None
        status = "PASS" if got_risk == tc["expect_risk"] else "FAIL"
        print(f"[{status}] {tc['name']}")
        if event:
            print(f"       -> {event.to_dict()}")
        print()
