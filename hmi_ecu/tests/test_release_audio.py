from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hmi.audio import AlertSound, _Player


class ReleaseAudioTests(unittest.TestCase):
    def test_l3_to_l0_stops_current_sound_and_prevents_replay(self):
        with tempfile.TemporaryDirectory() as directory:
            sound = AlertSound(directory=directory, enabled=False)
            sound.player = Mock()
            sound.muted = False
            sound.update({"risk_level": 3}, now=1.0)
            sound.player.play.assert_called_once()
            sound.update({"risk_level": 0}, now=1.01)
            self.assertIsNone(sound.current)
            self.assertEqual(sound.player.stop.call_count, 2)
            for now in (2.0, 3.0, 4.0):
                sound.update({"risk_level": 0}, now=now)
            sound.player.play.assert_called_once()

    def test_failsafe_audio_remains_when_general_risk_is_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            sound = AlertSound(directory=directory, enabled=False)
            sound.update({"risk_level": 3}, now=1.0)
            self.assertEqual(sound.update({"risk_level": 0,
                                           "failsafe_active": True}, now=1.01), "fault")

    def test_aplay_process_is_terminated_on_off(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(sys.modules, {"PySide6.QtCore": None,
                                          "PySide6.QtMultimedia": None}):
                player = _Player(directory)
            player.backend = "aplay"
            player.effects = {}
            process = Mock()
            process.poll.return_value = None
            with patch("hmi.audio.subprocess.Popen", return_value=process):
                player.play("danger", 0.9)
            player.stop()
            process.terminate.assert_called_once()
            self.assertIsNone(player.process)


if __name__ == "__main__":
    unittest.main()
