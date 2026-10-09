import unittest
from controller_core import Controller, wheel_command

class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.sent = []
        self.c = Controller(self.sent.append, lambda: self.now)
        self.c.connect()
        self.now = 2.1
        self.c.tick()
        self.c.receive('ROBOT_READY 2')
        self.c.receive('CONFIG_OK')
        self.c.enable()

    def test_handshake_required(self):
        c = Controller(self.sent.append, lambda: self.now)
        c.connect(); c.enable(); c.press('Up'); c.tick()
        self.assertFalse(c.enabled)
        self.assertFalse(any(s.startswith('DRIVE') for s in self.sent))

    def test_arrow_release_and_focus_stop(self):
        self.c.press('Up'); self.c.tick()
        self.assertIn('DRIVE 255 255', self.sent)
        self.c.release('Up'); self.assertEqual(self.sent[-1], 'STOP')
        self.c.press('Left'); self.c.tick()
        self.assertEqual(wheel_command({'Left'}, 180), (-180, 180))
        self.c.disable(); self.assertEqual(self.sent[-1], 'STOP')
        self.assertFalse(self.c.enabled)
        self.assertFalse(self.c.keys)

    def test_space_requires_new_keypress(self):
        self.c.press('Up'); self.c.stop()
        self.c.press('Up'); self.c.tick()
        self.assertEqual(self.sent[-1], 'STOP')
        self.c.release('Up'); self.c.press('Up'); self.c.tick()
        self.assertEqual(self.sent[-1], 'DRIVE 255 255')

    def test_opposites_stop_and_diagonal_curves(self):
        self.assertEqual(wheel_command({'Up', 'Down'}, 180), (0, 0))
        self.assertEqual(wheel_command({'Up', 'Left'}, 180), (90, 180))
        self.assertEqual(wheel_command({'Down', 'Right'}, 180), (-180, -90))
        self.assertEqual(wheel_command({'Left', 'Right'}, 180), (0, 0))

    def test_key_repeat_does_not_repeat_preset(self):
        self.c.press('F1'); self.c.press('F1'); self.c.tick()
        self.assertEqual(len([s for s in self.sent if s.startswith('TEST ')]), 1)
        self.c.stop(); self.now += 5; self.c.tick()
        self.assertIsNone(self.c.plan)

    def test_suite_advances_only_on_matching_completion(self):
        self.c.start_test('Wheels'); self.c.tick()
        req = self.c.test_id
        self.c.receive(f'TEST_DONE {req-1}')
        self.assertIsNone(self.c.gap_until)
        self.c.receive(f'TEST_DONE {req}')
        self.now += 0.9; self.c.receive('ALIVE'); self.c.tick()
        self.assertEqual(self.c.test_id, req)
        self.now += 0.2; self.c.receive('ALIVE'); self.c.tick()
        self.assertEqual(self.c.test_id, req+1)

    def test_heartbeat_and_link_loss(self):
        self.now += 0.11; self.c.tick()
        self.assertIn('HEARTBEAT', self.sent)
        self.now += 1.1; self.c.tick()
        self.assertFalse(self.c.ready)
        self.assertFalse(self.c.enabled)
        self.assertEqual(self.sent[-1], 'STOP')

    def test_config_disables_until_acknowledged(self):
        self.c.configure(80, 100, False, True)
        self.assertFalse(self.c.enabled)
        self.assertFalse(self.c.ready)
        self.assertEqual(self.sent[-1], 'CONFIG 80 100 0 1')
        self.c.receive('CONFIG_OK'); self.assertTrue(self.c.ready)
        self.assertFalse(self.c.enabled)

    def test_timeout_from_firmware_disarms(self):
        self.c.receive('CONTROL_TIMEOUT: stopped; reconnect controller')
        self.assertFalse(self.c.ready)
        self.assertFalse(self.c.enabled)

if __name__ == '__main__': unittest.main()
