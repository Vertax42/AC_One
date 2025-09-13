#!/usr/bin/env python3
"""
测试语音功能
"""

import pyttsx3
import threading
import time

# 创建语音引擎
print("初始化语音引擎...")
try:
    voice_engine = pyttsx3.init()
    print("语音引擎初始化成功!")
except Exception as e:
    print(f"语音引擎初始化失败: {e}")
    exit(1)

# 设置语音属性
print("设置语音属性...")
try:
    voice_engine.setProperty("voice", "en")
    voice_engine.setProperty("rate", 120)  # 设置语速
    print("语音属性设置成功!")
except Exception as e:
    print(f"语音属性设置失败: {e}")

# 创建锁
voice_lock = threading.Lock()


def voice_process(voice_engine, line):
    """语音处理函数"""
    print(f"准备播放: {line}")
    with voice_lock:
        try:
            voice_engine.say(line)
            voice_engine.runAndWait()
            print(f"播放完成: {line}")
        except Exception as e:
            print(f"播放失败: {e}")
    return


def test_voice():
    """测试语音功能"""
    print("=" * 50)
    print("开始语音测试")
    print("=" * 50)

    # 测试1: 基本语音
    print("\n测试1: 基本语音")
    voice_process(voice_engine, "Hello, this is a voice test")
    time.sleep(1)

    # 测试2: 数字
    print("\n测试2: 数字")
    voice_process(voice_engine, "1, 2, 3, 4, 5")
    time.sleep(1)

    # 测试3: 中文（如果支持）
    print("\n测试3: 中文测试")
    try:
        voice_process(voice_engine, "你好，这是语音测试")
    except:
        print("中文语音可能不支持，跳过")
    time.sleep(1)

    # 测试4: 短句
    print("\n测试4: 短句")
    voice_process(voice_engine, "Go")
    time.sleep(1)

    # 测试5: 检查可用语音
    print("\n测试5: 检查可用语音")
    voices = voice_engine.getProperty("voices")
    print(f"可用语音数量: {len(voices) if voices else 0}")
    if voices:
        for i, voice in enumerate(voices):
            print(f"语音 {i}: {voice.name} ({voice.languages})")

    print("\n语音测试完成!")


if __name__ == "__main__":
    test_voice()
