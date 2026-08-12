#!/usr/bin/env python3
"""
risk_detector_node.py (Day4 + Day6 Discord 알림 연동)
/scan 토픽을 구독해서 risk_logic.detect_risk()로 위험을 판단하고,
위험 감지 시 /risk_events 토픽으로 이벤트를 퍼블리시한다.

Day6 추가 (은경, discord_notifier.py 연동):
- 위험 감지 시점에 Discord #위험알림 채널로 실시간 알림 전송
- 1초에 최대 한 번만 전송되도록 throttle 처리 (로그 출력 주기와 동일)
"""
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from patrol_robot.risk_logic import detect_risk
from .discord_notifier import send_discord_alert
class RiskDetectorNode(Node):
    def __init__(self):
        super().__init__("risk_detector_node")
        # 위험 판단 임계값(m) - 파라미터로 빼서 나중에 실행 시 조정 가능하게
        self.declare_parameter("threshold", 0.5)
        self.threshold = self.get_parameter("threshold").value
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, 10
        )
        self.publisher = self.create_publisher(String, "/risk_events", 10)
        self._last_alert_time = 0.0
        self.get_logger().info(
            f"위험감지 노드 시작 (threshold={self.threshold}m). /scan 구독 중..."
        )
    def _scan_callback(self, msg: LaserScan):
        event = detect_risk(
            ranges=list(msg.ranges),
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            threshold=self.threshold,
            sensor_range_min=msg.range_min,
            sensor_range_max=msg.range_max,
        )
        if event is None:
            return
        payload = json.dumps(event.to_dict(), ensure_ascii=False)
        out_msg = String()
        out_msg.data = payload
        self.publisher.publish(out_msg)
        self.get_logger().warn(
            f"위험 감지! 거리={event.distance:.2f}m, 각도={event.angle_rad:.2f}rad",
            throttle_duration_sec=1.0,
        )
        now = time.time()
        if now - self._last_alert_time >= 1.0:
            send_discord_alert(
                f"🚨 위험 감지! 거리={event.distance:.2f}m, 각도={event.angle_rad:.2f}rad"
            )
            self._last_alert_time = now
def main(args=None):
    rclpy.init(args=args)
    node = RiskDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()
