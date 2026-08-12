#!/usr/bin/env python3
"""
dashboard_app.py (Day9)

api_server.py(FastAPI, http://localhost:8000)를 폴링해서
순찰 상태 / 위험 이벤트 로그 / 로봇 실시간 위치를 보여주는 Streamlit 대시보드.

실행 방법:
    streamlit run dashboard_app.py
(api_server.py가 먼저 실행되어 있어야 함)
"""
import time

import pandas as pd
import requests
import streamlit as st

API_BASE = "http://localhost:8000"
REFRESH_INTERVAL_SEC = 2

st.set_page_config(page_title="Safety Patrol Robot 대시보드", layout="wide")
st.title("🤖 Safety Patrol Robot 대시보드")


def fetch(path: str, default=None):
    try:
        resp = requests.get(f"{API_BASE}{path}", timeout=2)
        resp.raise_for_status()
        return resp.json()
    except requests.exceptions.RequestException:
        return default


status = fetch("/api/status", default={"state": "unknown"})
position = fetch("/api/position", default={"x": None, "y": None, "yaw": None})
risk_data = fetch("/api/risk-events", default={"count": 0, "events": []})

if status.get("state") == "unknown" and position.get("x") is None:
    st.warning(
        "⚠️ API 서버에 연결할 수 없습니다. api_server.py가 실행 중인지, "
        "ROS2 노드(schedule_manager_node, risk_detector_node)가 켜져 있는지 확인해주세요."
    )

# --- 1. 순찰 상태 카드 ---
st.subheader("📍 순찰 상태")

STATE_LABELS = {
    "idle": "🔴 대기 중 (해당 시간대 없음)",
    "patrolling": "🟢 순찰 중",
    "risk_response": "🟠 위험 대응 중",
    "unknown": "⚪ 알 수 없음",
}

col1, col2, col3, col4 = st.columns(4)
col1.metric("상태", STATE_LABELS.get(status.get("state"), status.get("state", "-")))
col2.metric("현재 블록", status.get("block_name") or "-")
col3.metric("현재 구역", status.get("zone_id") or "-")
col4.metric("현재 웨이포인트", status.get("waypoint_id") or "-")

if status.get("state") == "risk_response":
    st.info(
        f"위험 대응 중 — 거리={status.get('distance', '?')}m, "
        f"각도={status.get('angle_rad', '?')}rad, "
        f"위험 지점=({status.get('danger_x', '?')}, {status.get('danger_y', '?')})"
    )

# --- 2. 로봇 실시간 위치 ---
st.subheader("🗺️ 로봇 실시간 위치")

if position.get("x") is not None:
    pos_df = pd.DataFrame([{"x": position["x"], "y": position["y"]}])
    col_a, col_b = st.columns([1, 2])
    with col_a:
        st.metric("x", f"{position['x']:.2f} m")
        st.metric("y", f"{position['y']:.2f} m")
        st.metric("yaw", f"{position['yaw']:.2f} rad")
    with col_b:
        st.scatter_chart(pos_df, x="x", y="y", height=300)
else:
    st.write("위치 정보 없음 (AMCL이 아직 pose를 발행하지 않았을 수 있습니다)")

# --- 3. 위험 이벤트 로그 ---
st.subheader("🚨 최근 위험 이벤트 로그")

events = risk_data.get("events", [])
if events:
    df = pd.DataFrame(events)
    cols_order = [c for c in ["received_at", "distance", "angle_rad"] if c in df.columns]
    other_cols = [c for c in df.columns if c not in cols_order]
    df = df[cols_order + other_cols]
    st.dataframe(df, use_container_width=True, height=300)
else:
    st.write("아직 기록된 위험 이벤트가 없습니다.")

st.caption(f"{REFRESH_INTERVAL_SEC}초마다 자동 새로고침됩니다.")

time.sleep(REFRESH_INTERVAL_SEC)
st.rerun()