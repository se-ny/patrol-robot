import os
import requests
from dotenv import load_dotenv

# .env 파일에서 DISCORD_WEBHOOK_URL 불러오기
load_dotenv()
DISCORD_WEBHOOK_URL = os.getenv("DISCORD_WEBHOOK_URL")


def send_discord_alert(message: str) -> bool:
    """
    Discord 채널로 위험 알림 메시지를 전송한다.

    Args:
        message: 전송할 알림 내용 (예: "위험 감지! 거리=0.29m")

    Returns:
        전송 성공 여부 (True/False)
    """
    if not DISCORD_WEBHOOK_URL:
        print("[discord_notifier] DISCORD_WEBHOOK_URL이 설정되지 않았습니다 (.env 확인)")
        return False

    payload = {"content": message}

    try:
        response = requests.post(DISCORD_WEBHOOK_URL, json=payload, timeout=3)
        response.raise_for_status()
        return True
    except requests.exceptions.RequestException as e:
        print(f"[discord_notifier] 전송 실패: {e}")
        return False
