import unittest
from controller_core import Controller

class ObstacleTests(unittest.TestCase):
    def test_obstacle_cancels_auto_and_requires_fresh_keypress(self):
        sent = []
        c = Controller(sent.append, lambda: 0)
        c.connected = c.ready = c.enabled = True
        c.press('Up')
        c.receive('SAFETY OBSTACLE 20')
        self.assertEqual(c.keys, set())
        self.assertIn('Up', c.blocked)
        c.start_test('Full lab')
        c.receive('MOTION_BLOCKED')
        self.assertIsNone(c.plan)
        self.assertFalse(c.waiting_test)
        self.assertTrue(c.ready)
        self.assertIn('obstacle', c.sensor_status.lower())

    def test_clear_does_not_restart_keys_or_test(self):
        c = Controller(lambda _: None, lambda: 0)
        c.connected = c.ready = c.enabled = True
        c.press('Up')
        c.receive('SAFETY OBSTACLE 20')
        c.receive('SAFETY CLEAR 50')
        self.assertFalse(c.keys)
        self.assertIsNone(c.plan)
        self.assertIn('clear', c.sensor_status.lower())
