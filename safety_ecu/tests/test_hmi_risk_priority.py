from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import hmi_sender


class HmiRiskPriorityTests(unittest.TestCase):
    def test_risk_change_replaces_queued_snapshot_without_waiting_for_period(self):
        sender = hmi_sender.HmiSender()
        state = SimpleNamespace(risk_level=3)
        with patch.object(hmi_sender, "build_packet",
                          side_effect=lambda state, seq, **values: dict(
                              risk_level=state.risk_level, seq=seq)):
            with patch.object(hmi_sender.time, "monotonic", return_value=1.0):
                sender.publish(state)
            state.risk_level = 2
            with patch.object(hmi_sender.time, "monotonic", return_value=1.01):
                sender.publish(state)
            self.assertEqual(sender._queue.get_nowait()["risk_level"], 2)
            self.assertTrue(sender._queue.empty())


if __name__ == "__main__":
    unittest.main()
