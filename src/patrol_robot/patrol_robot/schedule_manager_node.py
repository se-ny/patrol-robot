#!/usr/bin/env python3
"""
schedule_manager_node.py (Day6 업데이트)

Day3 초안에서 추가된 것:
- /risk_events 구독 → 위험 감지 시 현재 goal 취소하고 위험 좌표로 재전송
- /amcl_pose 구독 → 위험 좌표(로봇 기준 상대좌표) 계산에 필요한 로봇 현재 위치 추적

동작 방식:
1. 평소엔 스케줄대로 순찰 (기존 run_once 로직)
2. /risk_events가 오면:
   a. 지금 가고 있던 goal을 취소
   b. distance/angle_rad + 로봇 현재 pose로 위험 지점의 map 좌표 계산
   c. 그 좌표로 새 goal 전송 (확인차 이동)
   d. 도착하면 잠깐 대기 후 원래 순찰 스케줄로 복귀
"""

import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


DEFAULT_SCHEDULE_PATH = (
    Path(get_package_share_directory("patrol_robot")) / "config" / "schedule.json"
)

# 위험 지점 확인 후 대기 시간(초) - 너무 짧으면 확인이 무의미, 너무 길면 순찰 지연
RISK_INVESTIGATE_WAIT_SEC = 3.0


class ScheduleManagerNode(Node):
    def __init__(self, schedule_path: Path = DEFAULT_SCHEDULE_PATH):
        super().__init__("schedule_manager_node")

        self.declare_parameter("schedule_path", str(schedule_path))
        resolved_path = Path(self.get_parameter("schedule_path").value)

        self.schedule = self._load_schedule(resolved_path)
        self.zones_by_id = {z["zone_id"]: z for z in self.schedule["zones"]}

        # --- Nav2 액션 클라이언트 (Day4에서 확인한 방식 그대로 적용) ---
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._current_goal_handle = None
        self._interrupted_by_risk = False

        # 위험 대응으로 순찰이 중단됐을 때 "어디까지 갔는지" 기억해두는 상태.
        # (block_id, zone_index, waypoint_index) 튜플, 없으면 처음부터 순찰.
        # 이게 없으면 위험 이벤트가 걸릴 때마다 run_once()가 patrol_order를
        # 처음부터 다시 순회해서 항상 첫 waypoint에만 머무르게 됨.
        self._patrol_progress: tuple[str, int, int] | None = None

        # --- 로봇 현재 위치 추적 ---
        # AMCL은 /amcl_pose를 RELIABLE + TRANSIENT_LOCAL QoS로 발행함.
        # 기본 QoS(depth=10, VOLATILE)로 구독하면 durability 불일치로
        # 메시지를 아예 못 받는 문제가 있어 AMCL과 동일한 QoS로 맞춰줌.
        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._current_pose = None  # (x, y, yaw) 튜플
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_callback, amcl_qos
        )

        # --- 위험 이벤트 구독 ---
        self.create_subscription(String, "/risk_events", self._risk_event_callback, 10)

        self.get_logger().info("Nav2 액션 서버 대기 중...")
        self._nav_client.wait_for_server()
        self.get_logger().info("Nav2 준비 완료. 스케줄 매니저 시작.")

    # ------------------------------------------------------------------
    # 초기화/유틸
    # ------------------------------------------------------------------

    def _load_schedule(self, path: Path) -> dict:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.get_logger().info(
            f"스케줄 로드 완료: {path} (time_blocks={len(data['time_blocks'])})"
        )
        return data

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._current_pose = (x, y, yaw)

    def _get_current_time_block(self) -> dict | None:
        """
        알려진 한계: night처럼 end_time < start_time(자정 넘김)인 블록은
        아직 처리 못 함. (Day3부터 이어지는 이슈, 별도로 고칠 예정)
        """
        now_str = datetime.now().strftime("%H:%M")
        for block in self.schedule["time_blocks"]:
            if block["start_time"] <= now_str < block["end_time"]:
                return block
        return None

    def _waypoint_to_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        q = quaternion_from_euler(0.0, 0.0, yaw)
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

    # ------------------------------------------------------------------
    # 위험 이벤트 처리 (Day6 핵심)
    # ------------------------------------------------------------------

    def _risk_event_callback(self, msg: String):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"위험 이벤트 파싱 실패: {msg.data}")
            return

        if self._current_pose is None:
            self.get_logger().warn(
                "위험 이벤트 수신했지만 로봇 현재 위치를 아직 모름 (amcl_pose 대기 중). 무시."
            )
            return

        if self._interrupted_by_risk:
            self.get_logger().info("이미 위험 대응 중, 새 이벤트 무시.")
            return

        distance = event.get("distance")
        angle_rad = event.get("angle_rad")
        if distance is None or angle_rad is None:
            self.get_logger().warn(f"위험 이벤트에 distance/angle_rad 없음: {event}")
            return

        self.get_logger().warn(
            f"⚠️ 위험 이벤트 수신: distance={distance:.2f}m, angle_rad={angle_rad:.2f}"
        )

        danger_x, danger_y = self._compute_danger_coordinate(distance, angle_rad)
        self._interrupted_by_risk = True
        self._cancel_current_goal_and_investigate(danger_x, danger_y)

    def _compute_danger_coordinate(self, distance: float, angle_rad: float) -> tuple[float, float]:
        """
        /risk_events의 angle_rad는 로봇(base_scan) 기준 상대각도.
        여기에 로봇의 현재 map 기준 yaw를 더해서 절대각도를 구하고,
        로봇의 현재 map 좌표에서 그 방향으로 (distance - 안전여유)만큼
        떨어진 지점을 계산 (장애물 코앞까지 가지 않도록).
        """
        robot_x, robot_y, robot_yaw = self._current_pose
        absolute_angle = robot_yaw + angle_rad

        # 장애물 바로 앞에서 멈추도록 안전 여유(standoff) 적용
        SAFETY_MARGIN = 0.3  # 미터, 로봇 반경 + 여유
        approach_distance = max(distance - SAFETY_MARGIN, 0.05)

        danger_x = robot_x + approach_distance * math.cos(absolute_angle)
        danger_y = robot_y + approach_distance * math.sin(absolute_angle)
        return danger_x, danger_y

    def _cancel_current_goal_and_investigate(self, danger_x: float, danger_y: float):
        if self._current_goal_handle is not None:
            self.get_logger().info("현재 순찰 goal 취소 요청.")
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda f: self._on_cancel_done(f, danger_x, danger_y)
            )
        else:
            self._send_investigate_goal(danger_x, danger_y)

    def _on_cancel_done(self, future, danger_x: float, danger_y: float):
        self.get_logger().info("현재 goal 취소 완료. 위험 지점으로 이동 시작.")
        self._send_investigate_goal(danger_x, danger_y)

    def _send_investigate_goal(self, x: float, y: float):
        pose = self._waypoint_to_pose(x, y, yaw=0.0)
        self.get_logger().info(f"위험 지점으로 이동: ({x:.2f}, {y:.2f})")

        send_goal_future = self._nav_client.send_goal_async(
            self._make_goal_msg(pose)
        )
        send_goal_future.add_done_callback(self._on_investigate_goal_response)

    def _on_investigate_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("위험 지점 goal이 거부됨. 순찰로 바로 복귀.")
            self._interrupted_by_risk = False
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_investigate_result)

    def _on_investigate_result(self, future):
        self.get_logger().info(
            f"위험 지점 도착/처리 완료. {RISK_INVESTIGATE_WAIT_SEC}초 대기 후 순찰 복귀."
        )
        time.sleep(RISK_INVESTIGATE_WAIT_SEC)
        self._interrupted_by_risk = False
        self._current_goal_handle = None
        self.get_logger().info("순찰 스케줄로 복귀.")
        self.run_once()

    # ------------------------------------------------------------------
    # 기존 순찰 로직 (Day3, goal 전송 부분만 액션클라이언트 직접 호출로 교체)
    # ------------------------------------------------------------------

    def _make_goal_msg(self, pose: PoseStamped) -> NavigateToPose.Goal:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        return goal_msg

    def run_once(self):
        block = self._get_current_time_block()
        if block is None:
            self.get_logger().warn("현재 시각에 해당하는 time_block이 없습니다.")
            self._patrol_progress = None
            return

        patrol_order = block["patrol_order"]

        # 이전에 위험 대응으로 중단된 지점이 있고, 같은 time_block이면
        # 그 지점부터 이어서 순찰. 블록이 바뀌었으면 처음부터.
        start_zone_idx = 0
        start_wp_idx = 0
        if self._patrol_progress is not None and self._patrol_progress[0] == block["block_id"]:
            _, start_zone_idx, start_wp_idx = self._patrol_progress
            self.get_logger().info(
                f"중단된 지점부터 순찰 재개: zone_index={start_zone_idx}, waypoint_index={start_wp_idx}"
            )
        else:
            self.get_logger().info(
                f"현재 블록: {block['block_name']} ({block['block_id']}), "
                f"순찰 순서: {patrol_order}"
            )

        for zone_idx in range(start_zone_idx, len(patrol_order)):
            zone_id = patrol_order[zone_idx]

            if self._interrupted_by_risk:
                self.get_logger().info("위험 대응 중이라 순찰 중단.")
                self._patrol_progress = (block["block_id"], zone_idx, 0)
                return

            waypoints = self._zone_waypoints(zone_id)
            wp_start = start_wp_idx if zone_idx == start_zone_idx else 0

            for wp_idx in range(wp_start, len(waypoints)):
                if self._interrupted_by_risk:
                    self._patrol_progress = (block["block_id"], zone_idx, wp_idx)
                    return
                self._go_to_waypoint(zone_id, waypoints[wp_idx])

        # patrol_order 끝까지 중단 없이 순회 완료 → 이번 블록 순찰 완주
        self._patrol_progress = None
        self.get_logger().info(f"{block['block_name']} 순찰 완주.")

    def _go_to_waypoint(self, zone_id: str, waypoint: dict):
        pose = self._waypoint_to_pose(waypoint["x"], waypoint["y"], waypoint["yaw"])
        self.get_logger().info(
            f"[{zone_id}] goal 전송: {waypoint['waypoint_id']} "
            f"(x={waypoint['x']}, y={waypoint['y']})"
        )

        send_goal_future = self._nav_client.send_goal_async(self._make_goal_msg(pose))
        goal_handle = self._spin_until_future_complete(send_goal_future)

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(f"goal 거부됨: {waypoint['waypoint_id']}")
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self._spin_until_future_complete(result_future)
        self._current_goal_handle = None
        if not self._interrupted_by_risk:
            self.get_logger().info(f"[{zone_id}] {waypoint['waypoint_id']} 도착 완료.")

    def _spin_until_future_complete(self, future):
        """위험 이벤트 콜백도 계속 처리되도록 spin_until_future_complete 사용."""
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = ScheduleManagerNode()
    try:
        node.run_once()
        rclpy.spin(node)  # run_once 끝나도 위험 이벤트는 계속 감시
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()