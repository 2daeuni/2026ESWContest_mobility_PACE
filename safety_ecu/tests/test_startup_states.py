import json
import os
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carla.receiver_v2 import CarlaReceiverV2
from control.failsafe import FailSafeManager
from control.state import SystemState


class StartupStateTest(unittest.TestCase):
    def setUp(self):
        self.receiver = CarlaReceiverV2()
        self.receiver.sock = Mock()
        self.state = SystemState()
        self.failsafe = FailSafeManager()

    def receive(self, sequence, now, *, distance=None, closing=None, ttc=None, obstacle_id=None):
        packet = dict(source="CARLA", sequence=sequence, vehicle_speed_kmh=0,
                      distance_m=distance, closing_speed_mps=closing,
                      ttc_s=ttc, obstacle_id=obstacle_id, gear="D")
        self.receiver.sock.recvfrom.return_value = (
            json.dumps(packet).encode(), ("192.168.50.1", 5006))
        with patch("carla.receiver_v2.time.monotonic", return_value=now):
            accepted = self.receiver.receive()
        if accepted:
            self.state.update_carla(self.receiver)
        return accepted

    def test_start_waits_without_false_fault_or_ready(self):
        self.assertFalse(self.receiver.check_freshness())
        self.state.invalidate_carla(self.receiver.link_state)
        self.assertEqual(self.receiver.link_state, "WAITING")
        self.assertFalse(self.failsafe.evaluate(self.state, "CARLA"))
        self.assertEqual(self.failsafe.waiting_reason, "CARLA_WAITING")
        self.receive(1, 100)
        self.assertFalse(self.state.carla_connected)
        self.receive(2, 100.01)
        self.assertFalse(self.state.carla_connected)
        self.receive(3, 100.02)
        self.assertTrue(self.state.carla_connected)
        self.assertFalse(self.failsafe.evaluate(self.state, "CARLA"))

    def test_timeout_and_three_packet_recovery(self):
        for sequence in range(1, 4):
            self.receive(sequence, 100 + sequence * .01)
        with patch("carla.receiver_v2.time.monotonic", return_value=101):
            self.assertFalse(self.receiver.check_freshness())
        self.state.invalidate_carla(self.receiver.link_state)
        self.assertEqual(self.receiver.link_state, "LOST")
        self.assertTrue(self.failsafe.evaluate(self.state, "CARLA"))
        self.assertEqual(self.failsafe.reason, "CARLA_COMMUNICATION_ERROR")
        for sequence in (4, 5):
            self.receive(sequence, 101 + sequence * .01)
            self.assertEqual(self.receiver.link_state, "RECOVERING")
            self.assertFalse(self.state.carla_connected)
            self.assertTrue(self.failsafe.evaluate(self.state, "CARLA"))
        self.receive(6, 101.06)
        self.assertEqual(self.receiver.link_state, "READY")
        self.assertFalse(self.failsafe.evaluate(self.state, "CARLA"))

    def test_stationary_contact_packet_is_valid(self):
        self.assertTrue(self.receive(1, 100, distance=0, closing=0, ttc=0, obstacle_id=7))
        self.assertEqual(self.receiver.ttc, 0)

    def test_real_mode_does_not_require_carla(self):
        self.state.pedal_connected = True
        self.state.pedal_sensor_status = 0
        self.state.carla_link_state = "LOST"
        self.assertFalse(self.failsafe.evaluate(self.state, "HARDWARE"))
        self.assertIsNone(self.failsafe.reason)

    def test_pedal_wait_then_actual_timeout(self):
        self.state.carla_connected = True
        self.state.carla_link_state = "READY"
        self.assertFalse(self.failsafe.evaluate(self.state, "CARLA"))
        self.assertEqual(self.failsafe.waiting_reason, "PEDAL_WAITING")
        self.state.pedal_ever_connected = True
        self.assertTrue(self.failsafe.evaluate(self.state, "CARLA"))
        self.assertEqual(self.failsafe.reason, "PEDAL_COMMUNICATION_ERROR")

    def test_second_receiver_cannot_share_udp_port(self):
        first = CarlaReceiverV2(host="127.0.0.1", port=0)
        first.start()
        second = CarlaReceiverV2(host="127.0.0.1", port=first.sock.getsockname()[1])
        try:
            with self.assertRaises(OSError):
                second.start()
            self.assertIsNone(second.sock)
        finally:
            first.close()
            second.close()


if __name__ == "__main__":
    unittest.main()
