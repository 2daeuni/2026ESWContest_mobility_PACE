"""
청각 경고 (다중감각: 시각 + 햅틱 + 청각)

단계별 경고음 패턴
  L1 주의  : 차임 1회 (단계 진입 시)
  L2 경고  : 2연속 비프, 1.2 s 마다 반복
  L3 위험  : 3연속 고음 비프, 끊김 없이 반복 (약 0.5 s 주기)
  Fail-Safe: 낮은 2음 1회 (진입 시)
  충돌(DEMO): 긴 저음 1회
  → 단계가 높을수록 높고 빠르고 촘촘해짐 (긴급도 구분)

소리 파일은 시작할 때 assets/sounds/*.wav 로 직접 생성 (외부 음원 저작권 문제 없음).
재생: Qt Multimedia(QSoundEffect) → 없으면 aplay → 둘 다 없으면 무음 (화면 동작에는 영향 없음)

주의: 라즈베리파이 5 는 3.5 mm 오디오 단자가 없음 → HDMI 모니터 스피커 / USB 스피커 / 블루투스 사용
"""

import math
import os
import shutil
import struct
import subprocess
import time
import wave

ASSET_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "sounds")
RATE = 48000   # 대부분 오디오 장치 기본 형식 (48 kHz, 16 bit, 스테레오)

# 패턴: (파일, 반복 주기 s 또는 None=진입 시 1회)
PATTERNS = {
    "caution": ("caution.wav", None),
    "warning": ("warning.wav", 1.2),
    "danger": ("danger.wav", 0.5),
    "fault": ("fault.wav", None),
    "collision": ("collision.wav", None),
}


# -----------------------------------------------------------------
# 톤 생성
# -----------------------------------------------------------------
def _tone(freq, ms, volume=0.6, square=0.0):
    n = int(RATE * ms / 1000)
    fade = int(RATE * 0.006)
    out = []
    for i in range(n):
        t = i / RATE
        s = math.sin(2 * math.pi * freq * t)
        if square:
            s = (1 - square) * s + square * (1.0 if s >= 0 else -1.0)
        # 클릭 방지 페이드 인/아웃
        env = min(1.0, i / fade, (n - 1 - i) / fade) if fade else 1.0
        out.append(s * env * volume)
    return out


def _silence(ms):
    return [0.0] * int(RATE * ms / 1000)


def _chime(f1, f2, ms, volume=0.5):
    """두 배음이 섞이고 서서히 줄어드는 부드러운 차임."""
    n = int(RATE * ms / 1000)
    return [
        (math.sin(2 * math.pi * f1 * i / RATE) * 0.7 + math.sin(2 * math.pi * f2 * i / RATE) * 0.3)
        * math.exp(-4.0 * i / n) * min(1.0, i / 200.0) * volume
        for i in range(n)
    ]


def _write(path, samples):
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(RATE)
        frames = []
        for s in samples:
            v = struct.pack("<h", int(max(-1.0, min(1.0, s)) * 32000))
            frames.append(v + v)
        w.writeframes(b"".join(frames))


def generate_sounds(directory=ASSET_DIR, force=False):
    os.makedirs(directory, exist_ok=True)
    specs = {
        "caution.wav": lambda: _chime(880, 1320, 420),
        "warning.wav": lambda: _tone(1050, 120) + _silence(80) + _tone(1050, 120),
        "danger.wav": lambda: (_tone(1500, 90, 0.75, 0.3) + _silence(55)) * 3 + _silence(20),
        "fault.wav": lambda: _tone(520, 220, 0.55) + _silence(40) + _tone(390, 320, 0.55),
        "collision.wav": lambda: _tone(260, 900, 0.7, 0.6),
    }
    for name, fn in specs.items():
        path = os.path.join(directory, name)
        if force or not os.path.exists(path):
            _write(path, fn())
    return directory


# -----------------------------------------------------------------
# 어떤 경고음을 낼지
# -----------------------------------------------------------------
def choose_pattern(s):
    if s.get("demo_collided") or s.get("carla_collision"):
        return "collision"
    if (not s.get("failsafe_active") and s.get("normal_braking")
            and s.get("risk_cause") == "RECOVERING"):
        return None
    level = s.get("risk_level")
    if s.get("brake_now") or level == 3:
        return "danger"
    if level == 2:
        return "warning"
    if level == 1:
        return "caution"
    if s.get("failsafe_active"):
        return "fault"
    return None


class _Player:
    """QSoundEffect → aplay → 무음 순서로 사용 가능한 재생기."""

    def __init__(self, directory):
        self.directory = directory
        self.backend = "none"
        self.effects = {}
        self.process = None
        try:
            from PySide6.QtCore import QUrl
            from PySide6.QtMultimedia import QSoundEffect
            for key, (fname, _period) in PATTERNS.items():
                eff = QSoundEffect()
                eff.setSource(QUrl.fromLocalFile(os.path.join(directory, fname)))
                eff.setVolume(0.9)
                self.effects[key] = eff
            self.backend = "QtMultimedia"
        except Exception:
            self.effects = {}
        self.has_aplay = shutil.which("aplay") is not None
        if not self.effects and self.has_aplay:
            self.backend = "aplay"

    def play(self, key, volume):
        if self.backend == "QtMultimedia":
            eff = self.effects.get(key)
            if eff is not None:
                try:
                    from PySide6.QtMultimedia import QSoundEffect
                    broken = eff.status() == QSoundEffect.Status.Error
                except Exception:
                    broken = False
                if not broken:
                    eff.setVolume(volume)
                    eff.play()
                    return
                # Qt 쪽 장치/형식 문제 → aplay 로 전환
                self.backend = "aplay" if self.has_aplay else "none"
        if self.backend == "aplay":
            path = os.path.join(self.directory, PATTERNS[key][0])
            try:
                self.stop()
                self.process = subprocess.Popen(
                    ["aplay", "-q", path], stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL)
            except OSError:
                pass

    def stop(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=0.02)
                except subprocess.TimeoutExpired:
                    pass
            self.process = None
        for eff in self.effects.values():
            try:
                eff.stop()
            except Exception:
                pass


class AlertSound:
    """backend 의 매 틱(30 Hz)마다 update(state) 호출."""

    def __init__(self, directory=ASSET_DIR, enabled=True):
        generate_sounds(directory)
        self.player = _Player(directory) if enabled else None
        self.muted = not enabled
        self.volume = 0.9
        self.current = None
        self.last_play = 0.0
        self.played_count = 0

    @property
    def backend(self):
        return self.player.backend if self.player else "off"

    def set_muted(self, muted):
        self.muted = bool(muted)
        if self.muted and self.player:
            self.player.stop()

    def update(self, state, now=None):
        now = time.monotonic() if now is None else now
        key = choose_pattern(state)
        if key != self.current:
            self.current = key
            self.last_play = 0.0
            if self.player:
                self.player.stop()
            if key is not None:
                self._play(key, now)
            return key
        if key is not None:
            period = PATTERNS[key][1]
            if period is not None and now - self.last_play >= period:
                self._play(key, now)
        return key

    def _play(self, key, now):
        self.last_play = now
        if self.muted or self.player is None:
            return
        self.played_count += 1
        self.player.play(key, self.volume)
