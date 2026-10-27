import requests
import tempfile
import subprocess
import base64
import shutil
import uuid

# 接口配置
API_URL = "https://openspeech.bytedance.com/api/v1/tts"
HEADERS = {
    "x-api-key": "b584ac77-03f6-4e56-9e12-de6429bfab3d"
}

# 请求参数
PAYLOAD = {
    "app": {"cluster": "volcano_tts"},
    "user": {"uid": "豆包语音"},
    "audio": {
        "voice_type": "BV002",
        "encoding": "mp3",
        "speed_ratio": 0.75,
        "volume_ratio": 1.0,
        "pitch_ratio": 1.0
    },
    "request": {
        "reqid": "021774350691800fdbd400a04000000ffffffff00000993fe578d",
        "text": "你好呀",  # 修改播报文字
        "operation": "query"
    }
}

def play_tts(text=" 您好，我叫艾德机器人"):
    try:
        payload = {
            "app": dict(PAYLOAD["app"]),
            "user": dict(PAYLOAD["user"]),
            "audio": dict(PAYLOAD["audio"]),
            "request": dict(PAYLOAD["request"]),
        }
        payload["request"]["text"] = text
        payload["request"]["reqid"] = uuid.uuid4().hex

        # 1. 获取语音数据
        print("⏳ 获取语音数据...")
        response = requests.post(API_URL, headers=HEADERS, json=payload, timeout=30)
        response.raise_for_status()

        audio_bytes = b""
        suffix = ".mp3"

        content_type = response.headers.get("Content-Type", "")
        if "application/json" in content_type:
            data = response.json()
            if data.get("code") != 3000:
                print(f"❌ TTS 失败：{data}")
                return

            b64_audio = data.get("data")
            if not b64_audio:
                print(f"❌ TTS 返回成功但没有音频数据：{data}")
                return

            audio_bytes = base64.b64decode(b64_audio)
        else:
            audio_bytes = response.content

        # 根据文件头简单判断格式
        if audio_bytes.startswith(b"RIFF"):
            suffix = ".wav"
        elif audio_bytes.startswith(b"ID3") or (len(audio_bytes) > 2 and audio_bytes[0] == 0xFF):
            suffix = ".mp3"

        # 2. 临时文件（播放后自动删除，不留痕迹）
        with tempfile.NamedTemporaryFile(suffix=suffix, delete=True) as temp_file:
            temp_file.write(audio_bytes)
            temp_file.flush()
            
            print("🔊 正在播放语音...")
            if suffix == ".wav":
                subprocess.run(["paplay", temp_file.name], check=True)
            else:
                if shutil.which("mpg123"):
                    subprocess.run(["mpg123", "-q", temp_file.name], check=True)
                elif shutil.which("ffplay"):
                    subprocess.run(["ffplay", "-nodisp", "-autoexit", "-loglevel", "error", temp_file.name], check=True)
                else:
                    print("❌ 当前系统只有 paplay，不能直接播放 mp3。")
                    print("请安装播放器：sudo apt install -y mpg123")
                    return

        print("✅ 播放完成！")

    except requests.exceptions.RequestException as e:
        print(f"❌ 请求失败：{str(e)}")
    except subprocess.CalledProcessError as e:
        print(f"❌ 播放失败：{e}")

if __name__ == "__main__":
    play_tts()