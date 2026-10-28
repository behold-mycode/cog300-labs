import unittest
from robot_link import RobotLink, validate_settings, wheel_command

class SerialFake:
    def __init__(self): self.sent = []; self.incoming = bytearray(); self.closed = False
    def write(self, data): self.sent.append(data.decode().strip()); return len(data)
    @property
    def in_waiting(self): return len(self.incoming)
    def read(self, count):
        data = bytes(self.incoming[:count]); del self.incoming[:count]; return data
    def close(self): self.closed = True
    def feed(self, line): self.incoming.extend((line + '\n').encode())

class LinkTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.; self.serial = SerialFake()
        self.link = RobotLink(clock=lambda: self.now, serial_factory=lambda port: self.serial)
        self.owner = self.link.connect('fake')
    def ready(self):
        self.link.frame(self.owner, 1, [], 'MANUAL')
        self.now = 2.; self.link.frame(self.owner, 2, [], 'MANUAL'); self.link.tick()
        self.serial.feed('ROBOT_READY 4'); self.link.tick()
        self.serial.feed('OK CONFIG'); self.link.tick()
        self.assertTrue(self.link.ready)
    def test_handshake_and_drive(self):
        self.ready(); self.link.frame(self.owner, 3, ['Up'], 'MANUAL'); self.now += .1; self.link.tick()
        self.assertIn('DRIVE 255 255', self.serial.sent)
        self.link.frame(self.owner, 4, [], 'MANUAL'); self.now += .1; self.link.tick()
        self.assertEqual(self.serial.sent[-1], 'STOP')
    def test_out_of_order_and_other_owner(self):
        self.ready(); self.link.frame(self.owner, 10, [], 'MANUAL')
        with self.assertRaises(ValueError): self.link.frame(self.owner, 9, ['Up'], 'MANUAL')
        with self.assertRaises(ValueError): self.link.frame('other', 11, ['Up'], 'MANUAL')
        self.assertEqual(self.link.keys, [])
    def test_browser_loss_stops_autonomy_and_requires_reconnect(self):
        self.ready(); self.link.frame(self.owner, 3, [], 'FOLLOW'); self.now += .1; self.link.tick()
        self.assertIn('MODE FOLLOW', self.serial.sent)
        self.now += .6; self.link.tick()
        self.assertFalse(self.link.ready); self.assertTrue(self.serial.closed)
        self.assertIn('STOP', self.serial.sent)
        with self.assertRaises(ValueError): self.link.frame(self.owner, 4, [], 'FOLLOW')
    def test_serial_loss_despite_live_browser(self):
        self.ready(); self.now += 1.1
        self.link.frame(self.owner, 3, [], 'MANUAL'); self.link.tick()
        self.assertFalse(self.link.ready)
    def test_config_and_validation(self):
        self.ready(); settings = dict(power=120, minimum=40, invertLeft=True, invertRight=False, followCm=25, wallCm=30)
        self.link.configure(self.owner, 3, settings)
        self.assertFalse(self.link.ready)
        self.assertIn('CONFIG 120 40 1 0 25 30', self.serial.sent)
        for value in [256, -1, True, 20.5, '120']:
            with self.assertRaises(ValueError): validate_settings(dict(settings, power=value))
    def test_partial_and_oversized_serial_lines(self):
        self.ready(); self.serial.feed('DATA,100,MANUAL,STOPPED,25.5,-1,0,0'); self.link.tick()
        self.assertEqual(self.link.telemetry['front'], 25.5)
        self.serial.incoming.extend(b'x' * 5000); self.link.tick(); self.link.tick()
        self.assertFalse(self.link.ready)
    def test_keyboard_mapping(self):
        self.assertEqual(wheel_command(['Up','Left'],255),(127,255))
        self.assertEqual(wheel_command(['Down'],255),(-255,-255))
        self.assertEqual(wheel_command(['Left'],255),(-255,255))
        self.assertEqual(wheel_command(['Up','Down'],255),(0,0))

if __name__ == '__main__': unittest.main()
