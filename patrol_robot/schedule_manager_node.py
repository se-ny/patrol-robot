#!/usr/bin/env python3
"""
schedule_manager_node.py (Day3 초안)

schedule.json을 읽어서, 현재 시간대(time_block)에 맞는 zone들을
patrol_order 순서대로 순회하며 Nav2에 goal을 순차 전송한다.

주의: 오늘은 "초안" 단계라 아래는 아직 안 됨 (다음 Day에서 채울 부분):
- 위험감지 이벤트에 따른 동적 재조정 (Day6)
- 순찰 완료 후 다음 time_block으로 자동 전환 로직 정교화
- 야간(night) 블록처럼 자정을 넘는 시간대 처리
"""

import json
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


DEFAULT_SCHEDULE_PATH = (
    Path(__file__).resolve().parent.parent / "config" / "schedule.json"
)


class ScheduleManagerNode(Node):
    def __init__(self, schedule_path: Path = DEFAULT_SCHEDULE_PATH):
        super().__init__("schedule_manager_node")

        self.declare_parameter("schedule_path", str(schedule_path))
        resolved_path = Path(self.get_parameter("schedule_path").value)

        self.schedule = self._load_schedule(resolved_path)
        self.zones_by_id = {z["zone_id"]: z for z in self.schedule["zones"]}

        self.navigator = BasicNavigator()
        self.get_logger().info("Nav2 활성화 대기 중...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 준비 완료. 스케줄 매니저 시작.")

    def _load_schedule(self, path: Path) -> dict:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.get_logger().info(
            f"스케줄 로드 완료: {path} (time_blocks={len(data['time_blocks'])})"
        )
        return data

    def _get_current_time_block(self) -> dict | None:
        """
        현재 시각 기준으로 해당하는 time_block을 찾는다.
        주의: night 블록처럼 end_time < start_time(자정 넘김)인 경우는
        아직 처리 못 함 -- Day3 초안 단계의 알려진 한계.
        """
        now_str = datetime.now().strftime("%H:%M")
        for block in self.schedule["time_blocks"]:
            if block["start_time"] <= now_str < block["end_time"]:
                return block
        return None

    def _waypoint_to_pose(self, waypoint: dict) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = waypoint["x"]
        pose.pose.position.y = waypoint["y"]

        q = quaternion_from_euler(0.0, 0.0, waypoint["yaw"])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def _zone_waypoints(self, zone_id: str) -> list[dict]:
        zone = self.zones_by_id.get(zone_id)
        if zone is None:
            self.get_logger().warn(f"알 수 없는 zone_id: {zone_id}, 스킵")
            return []
        return zone["waypoints"]

    def run_once(self):
        """
        현재 time_block의 patrol_order를 한 바퀴만 순회하며 goal 순차 전송.
        (초안: 반복 루프나 동적 재조정은 아직 없음)
        """
        block = self._get_current_time_block()
        if block is None:
            self.get_logger().warn("현재 시각에 해당하는 time_block이 없습니다.")
            return

        self.get_logger().info(
            f"현재 블록: {block['block_name']} ({block['block_id']}), "
            f"순찰 순서: {block['patrol_order']}"
        )

        for zone_id in block["patrol_order"]:
            waypoints = self._zone_waypoints(zone_id)
            for wp in waypoints:
                self._go_to_waypoint(zone_id, wp)

    def _go_to_waypoint(self, zone_id: str, waypoint: dict):
        pose = self._waypoint_to_pose(waypoint)
        self.get_logger().info(
            f"[{zone_id}] goal 전송: {waypoint['waypoint_id']} "
            f"(x={waypoint['x']}, y={waypoint['y']})"
        )
        self.navigator.goToPose(pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"  이동 중... 남은 거리: "
                    f"{feedback.distance_remaining:.2f}m",
                    throttle_duration_sec=2.0,
                )
            time.sleep(0.5)

        result = self.navigator.getResult()
        self.get_logger().info(f"  결과: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = ScheduleManagerNode()
    try:
        node.run_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()