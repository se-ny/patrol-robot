#!/usr/bin/env python3
"""
api_server.py (Day9)

ROS2 토픽을 구독하면서 FastAPI로 REST API를 제공하는 브릿지 서버.
- /patrol_status  → 순찰 상태 (schedule_manager_node가 발행)
- /risk_events    → 위험 이벤트 (risk_detector_node가 발행)
- /amcl_pose      → 로봇 현재 위치 (AMCL이 발행)

실행 방법 (safety-patrol-robot 워크스페이스 source 후):
    python3 api_server.py
기본적으로 http://localhost:8000 에서 뜸.
API 문서는 http://localhost:8000/docs 에서 자동 확인 가능 (FastAPI 기본 기능).
"""
import json
import threading
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

# 대시보드에서 보여줄 위험 이벤트 로그 최대 개수 (너무 많이 쌓이지 않도록 제한)
MAX_RISK_EVENT_LOG = 100


class DashboardBridgeNode(Node):
    """ROS2 토픽을 구독해서 최신 상태를 메모리에 들고 있는 노드."""

    def __init__(self):
        super().__init__("dashboard_bridge_node")

        # schedule_manager_node의 /patrol_status와 동일한 QoS로 맞춰야
        # 늦게 켜져도(대시보드 서버가 로봇보다 나중에 실행돼도) 마지막 상태를 받을 수 있음.
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        amcl_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.latest_status: dict | None = None
        self.latest_position: dict | None = None
        self.risk_event_log: deque = deque(maxlen=MAX_RISK_EVENT_LOG)

        self.create_subscription(String, "/patrol_status", self._on_status, status_qos)
        self.create_subscription(String, "/risk_events", self._on_risk_event, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, amcl_qos
        )

        self.get_logger().info("대시보드 브릿지 노드 시작. /patrol_status, /risk_events, /amcl_pose 구독 중...")

    def _on_status(self, msg: String):
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"patrol_status 파싱 실패: {msg.data}")

    def _on_risk_event(self, msg: String):
        try:
            event = json.loads(msg.data)
            event["received_at"] = datetime.now().isoformat()
            self.risk_event_log.append(event)
        except json.JSONDecodeError:
            self.get_logger().warn(f"risk_event 파싱 실패: {msg.data}")

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.latest_position = {"x": x, "y": y, "yaw": yaw}


# --- ROS2 노드를 백그라운드 스레드에서 spin ---
rclpy.init()
bridge_node = DashboardBridgeNode()


def _spin_ros():
    rclpy.spin(bridge_node)


ros_thread = threading.Thread(target=_spin_ros, daemon=True)
ros_thread.start()

# --- FastAPI 앱 ---
app = FastAPI(title="Safety Patrol Robot Dashboard API")

# Streamlit이 다른 포트(보통 8501)에서 이 API를 호출하니 CORS 허용
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/status")
def get_status():
    """현재 순찰 상태 (idle / patrolling / risk_response)."""
    return bridge_node.latest_status or {"state": "unknown"}


@app.get("/api/risk-events")
def get_risk_events(limit: int = 20):
    """최근 위험 이벤트 로그 (최신순)."""
    events = list(bridge_node.risk_event_log)[-limit:]
    events.reverse()
    return {"count": len(events), "events": events}


@app.get("/api/position")
def get_position():
    """로봇 현재 위치 (map 기준 x, y, yaw)."""
    return bridge_node.latest_position or {"x": None, "y": None, "yaw": None}


@app.get("/api/health")
def health_check():
    return {"ok": True}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)