#!/usr/bin/env python3
"""
risk_detector_node.py (Day4 + Day6 Discord 알림 연동 + Day8 쿨다운 튜닝)
/scan 토픽을 구독해서 risk_logic.detect_risk()로 위험을 판단하고,
위험 감지 시 /risk_events 토픽으로 이벤트를 퍼블리시한다.

Day6 추가 (은경, discord_notifier.py 연동):
- 위험 감지 시점에 Discord #위험알림 채널로 실시간 알림 전송

Day8 추가 (센아, 쿨다운 튜닝):
- 원래는 /scan 콜백마다(초당 5~6회) /risk_events를 매번 publish하고 있었음.
  로그는 throttle_duration_sec으로 화면 출력만 억제됐을 뿐, 실제 publish와
  Discord 전송은 계속 나가서 schedule_manager가 너무 자주 위험 대응에
  들어가고, Discord도 rate limit(429 Too Many Requests)에 걸리는 문제가 있었음.
- /risk_events 자체를 RISK_EVENT_COOLDOWN_SEC 간격으로만 publish하도록 변경.
- Discord 전송 쿨다운도 별도로 더 길게(DISCORD_ALERT_COOLDOWN_SEC) 둬서
  429를 방지.
"""
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from patrol_robot.risk_logic import detect_risk
from .discord_notifier import send_discord_alert

# 같은 위험 상황이 계속될 때 /risk_events를 다시 publish하기까지 최소 간격(초).
# 너무 짧으면 schedule_manager가 위험 대응 goal을 취소/재전송하는 걸 반복하게 됨.
RISK_EVENT_COOLDOWN_SEC = 2.0

# Discord webhook 자체 rate limit(429) 방지용 별도 쿨다운.
# risk_events 쿨다운보다 더 길게 둬서 스팸성 알림을 추가로 억제.
DISCORD_ALERT_COOLDOWN_SEC = 5.0


class RiskDetectorNode(Node):
    def __init__(self):
        super().__init__("risk_detector_node")
        # 위험 판단 임계값(m) - 파라미터로 빼서 나중에 실행 시 조정 가능하게
        # Day8 튜닝: 0.5m는 좁은 통로 정상 주행에서도 너무 자주 걸려서
        # 실측 테스트 후 0.3m로 낮춤. 필요하면 실행 시 여전히
        # --ros-args -p threshold:=<값> 으로 덮어쓸 수 있음.
        self.declare_parameter("threshold", 0.3)
        self.threshold = self.get_parameter("threshold").value
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, 10
        )
        self.publisher = self.create_publisher(String, "/risk_events", 10)
        self._last_publish_time = 0.0
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

        now = time.time()
        if now - self._last_publish_time < RISK_EVENT_COOLDOWN_SEC:
            # 같은 위험이 계속 감지되는 중이라도 쿨다운 안에는 재발행하지 않음
            return
        self._last_publish_time = now

        payload = json.dumps(event.to_dict(), ensure_ascii=False)
        out_msg = String()
        out_msg.data = payload
        self.publisher.publish(out_msg)
        self.get_logger().warn(
            f"위험 감지! 거리={event.distance:.2f}m, 각도={event.angle_rad:.2f}rad",
            throttle_duration_sec=1.0,
        )

        if now - self._last_alert_time >= DISCORD_ALERT_COOLDOWN_SEC:
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

