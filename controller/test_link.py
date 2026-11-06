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
        self.serial.feed('ROBOT_READY 6'); self.link.tick()
        self.serial.feed('OK CONFIG'); self.link.tick()
        self.assertTrue(self.link.ready)
    def test_default_direction_calibration(self):
        self.ready()
        self.assertIn('CONFIG 255 255 1 0 20 10 -1', self.serial.sent)

    def test_line_mode_telemetry_and_stop(self):
        self.ready()
        self.link.frame(self.owner, 3, [], 'LINE'); self.now += .1; self.link.tick()
        self.assertIn('MODE LINE', self.serial.sent)
        self.serial.feed('IR,0,1,0,1')
        self.serial.feed('DATA,100,LINE,LINE_TRACKING,25,20,140,140')
        self.link.tick()
        self.assertEqual(self.link.telemetry['ir'], [0,1,0])
        self.assertEqual(self.link.telemetry['mode'], 'LINE')
        self.serial.feed('DATA,110,MANUAL,LINE_TIMEOUT,25,20,0,0')
        self.link.tick()
        self.assertTrue(self.link.ready)
        self.assertFalse(self.serial.closed)
        self.assertEqual(self.link.mode, 'MANUAL')

    def test_maze_mode_and_front_stop(self):
        self.ready()
        self.link.frame(self.owner, 3, [], 'MAZE'); self.now += .1; self.link.tick()
        self.assertIn('MODE MAZE', self.serial.sent)
        self.serial.feed('DATA,100,MAZE,MAZE_WALL,50,10,140,140'); self.link.tick()
        self.assertEqual(self.link.telemetry['mode'], 'MAZE')
        self.serial.feed('DATA,110,MANUAL,FRONT_TOO_CLOSE,9,10,0,0'); self.link.tick()
        self.assertTrue(self.link.ready)
        self.assertFalse(self.serial.closed)
        self.assertEqual(self.link.mode, 'MANUAL')

    def test_line_polarity_settings(self):
        from robot_link import DEFAULTS
        self.assertEqual(DEFAULTS['lineTapeLevel'], -1)
        for level in [-1, 0, 1]:
            self.assertEqual(validate_settings(dict(DEFAULTS, lineTapeLevel=level))['lineTapeLevel'], level)
        for level in [2, True, '1']:
            with self.assertRaises(ValueError): validate_settings(dict(DEFAULTS, lineTapeLevel=level))

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
        self.ready(); settings = dict(power=120, minimum=40, invertA=True, invertB=False, followCm=25, wallCm=30)
        self.link.configure(self.owner, 3, settings)
        self.assertFalse(self.link.ready)
        self.assertIn('CONFIG 120 40 1 0 25 30 -1', self.serial.sent)
        for value in [256, -1, True, 20.5, '120']:
            with self.assertRaises(ValueError): validate_settings(dict(settings, power=value))
    def test_partial_and_oversized_serial_lines(self):
        self.ready(); self.serial.feed('DATA,100,MANUAL,STOPPED,25.5,-1,0,0'); self.link.tick()
        self.assertEqual(self.link.telemetry['front'], 25.5)
        self.serial.incoming.extend(b'x' * 5000); self.link.tick(); self.link.tick()
        self.assertFalse(self.link.ready)
    def test_navigation_faults_preserve_connection(self):
        for state in ['TURN_TIMEOUT', 'SEARCH_TIMEOUT', 'ALIGN_TIMEOUT']:
            with self.subTest(state=state):
                self.setUp(); self.ready()
                self.serial.feed('DATA,100,MANUAL,%s,60,15,0,0' % state)
                self.link.tick()
                self.assertTrue(self.link.ready)
                self.assertFalse(self.serial.closed)
                self.assertEqual(self.link.mode, 'MANUAL')

    def test_navigation_calibration_ack_and_validation(self):
        self.ready()
        for bad in ({'kind':3,'invertL':0,'invertR':0}, {'kind':True,'invertL':0,'invertR':0}):
            with self.assertRaises(ValueError): self.link.action(self.owner,3,'encoders',bad)
        self.link.action(self.owner,3,'encoders',dict(kind=2,invertL=0,invertR=1))
        self.assertIn('ENC 2 0 1',self.serial.sent);self.assertFalse(self.link.ready)
        self.link.frame(self.owner,4,['Up'],'MAZE');self.assertEqual(self.link.mode,'MANUAL')
        self.serial.feed('OK ENC');self.link.tick();self.assertTrue(self.link.ready)
        self.serial.feed('COUNTS,120,-80,2');self.link.tick()
        self.assertEqual(self.link.telemetry['encoderCounts'],[120,-80])

    def test_map_export_locks_movement_and_validates_links(self):
        self.ready();self.link.action(self.owner,3,'map_export',{})
        self.link.frame(self.owner,4,['Up'],'MAZE');self.assertEqual(self.link.mode,'MANUAL')
        with self.assertRaises(ValueError): self.link.action(self.owner,5,'map_reset',{})
        for line in ('MAP_BEGIN,2,1','MAP_NODE,0,0,0,0:0:0','MAP_NODE,1,100,0,3.14:0:0','MAP_EDGE,0,0,0,1,0,100,1','MAP_END'):
            self.serial.feed(line)
        self.link.tick();self.assertTrue(self.link.maze_map['complete']);self.assertFalse(self.link.map_exporting)
        for line in ('MAP_BEGIN,1,1','MAP_NODE,0,0,0,0:0:0','MAP_EDGE,0,0,0,5,0,100,1','MAP_END'):
            self.serial.feed(line)
        self.link.tick();self.assertFalse(self.link.maze_map['complete'])

    def test_reconnect_adopts_existing_map_calibration(self):
        self.link.frame(self.owner,1,[],'MANUAL');self.now=2
        self.link.frame(self.owner,2,[],'MANUAL');self.link.tick()
        self.serial.feed('CONFIG_REPORT,255,255,1,0,20,10,1')
        self.serial.feed('ROBOT_READY 6');self.link.tick()
        self.assertIn('CONFIG 255 255 1 0 20 10 1',self.serial.sent)
        self.serial.feed('OK CONFIG');self.link.tick();self.assertTrue(self.link.ready)

    def test_belief_and_observation_capture(self):
        self.ready();self.link.action(self.owner,3,'trace',dict(enabled=1))
        self.serial.feed('OK TRACE');self.link.tick();self.assertTrue(self.link.ready)
        self.serial.feed('BELIEF,1,0.1,0.6,0.1,0.1,0.1,50,51,0.05,10')
        self.serial.feed('OBS,100,1,2,0,0,0,5,5,300,200,10,20,255,255,1')
        self.serial.feed('OBS,invalid');self.link.tick()
        self.assertEqual(len(self.link.trace_rows),1)
        self.assertTrue(self.link.telemetry['dynamics'])
        self.assertEqual(self.link.telemetry['wheelSpeed'],[50,51])
        self.assertIn('100,1,2,',self.link.trace_csv())
        self.link.action(self.owner,4,'trace',dict(enabled=0));self.serial.feed('OK TRACE');self.link.tick()
        self.assertFalse(self.link.trace_recording);self.assertEqual(len(self.link.trace_rows),1)

    def test_keyboard_mapping(self):
        self.assertEqual(wheel_command(['Up','Left'],255),(0,255))
        self.assertEqual(wheel_command(['Down'],255),(-255,-255))
        self.assertEqual(wheel_command(['Left'],255),(-255,255))
        self.assertEqual(wheel_command(['Up','Down'],255),(0,0))

if __name__ == '__main__': unittest.main()
