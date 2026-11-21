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
    def test_line_control_telemetry(self):
        self.serial.feed('LINECTRL,1,240,230,0,1.2,40,200,210')
        self.link.tick()
        self.assertEqual(self.link.telemetry['lineControl']['cruise'], 240)
        self.assertEqual(self.link.telemetry['lineControl']['speedR'], 210)
    def ready(self):
        self.link.frame(self.owner, 1, [], 'MANUAL')
        self.now = 2.; self.link.frame(self.owner, 2, [], 'MANUAL'); self.link.tick()
        self.serial.feed('CAPS ARENA_1'); self.serial.feed('ROBOT_READY 7'); self.link.tick()
        self.serial.feed('OK CONFIG'); self.link.tick()
        self.assertTrue(self.link.ready)
    def test_old_firmware_blocks_arena_commands(self):
        self.serial.feed('ROBOT_READY 7'); self.link.tick()
        for seq, mode in enumerate(('OBJECT','RADAR_SCAN','RADAR_ARM'),1):
            with self.assertRaisesRegex(ValueError,'scanner firmware'):
                self.link.frame(self.owner, seq, [], mode)
        with self.assertRaisesRegex(ValueError,'scanner firmware'):
            self.link.action(self.owner,4,'radar_profile',dict(minMm=50,maxMm=90))
        self.assertNotIn('MODE OBJECT',self.serial.sent)
        self.assertFalse(any(s.startswith('RADAR') for s in self.serial.sent))

    def test_radar_commands_and_partial_sweeps(self):
        self.ready()
        self.link.action(self.owner, 3, 'radar_profile', dict(minMm=50, maxMm=100))
        self.assertEqual(self.serial.sent[-1], 'RADAR PROFILE 50 100')
        self.serial.feed('OK RADAR PROFILE'); self.link.tick()
        self.assertEqual(self.link.radar_snapshot()['settings']['maxMm'],100)
        self.link.frame(self.owner, 4, [], 'RADAR_SCAN'); self.now += .11; self.link.tick()
        self.assertIn('RADAR SCAN', self.serial.sent)
        for line in ('RADAR,7,-5,510,1','RADAR,7,0,-1,0','RADAR,7,5,530,2','LOCK,5,530,60'):
            self.serial.feed(line)
        self.link.tick()
        radar = self.link.radar_snapshot()
        self.assertEqual(len(radar['samples']),3)
        self.assertEqual(radar['lock'],[5,530,60])
        self.serial.feed('RADAR,8,4,500,1'); self.serial.feed('RADAR,7,9,510,2'); self.link.tick()
        self.assertEqual(self.link.radar_snapshot()['samples'],[[4,500,1]])
        self.now += 2.1
        self.assertEqual(self.link.radar_snapshot()['samples'],[])
        self.link._receive('DATA,3000,OBJECT,TURNING,20,30,90,-90')
        self.assertIsNone(self.link.radar_snapshot()['lock'])
        self.link.frame(self.owner, 5, [], 'MANUAL')
        self.serial.feed('ALIVE'); self.link.tick()
        self.assertEqual(self.serial.sent[-1],'STOP')

    def test_bench_widths_require_both_neighbors(self):
        self.ready()
        self.link.sent_mode = 'RADAR_SCAN'
        for angle,mm,group in [(-60,400,1),(-57,400,1),(-54,-1,0),(-51,500,2),(-48,500,2),(-45,-1,0),(-39,500,3),(-36,500,3),(-33,-1,0)]:
            self.link._receive(f'RADAR,1,{angle},{mm},{group}')
        widths = self.link.radar_snapshot()['bench']
        self.assertEqual([w['status'] for w in widths],['edge clipped','bounded','incomplete'])
        self.assertEqual(widths[1]['width'],52)
        self.link._receive('RADAR,2,-60,-1,0')
        self.assertEqual(self.link.radar_snapshot()['previousBench']['contours'],widths)
        with self.assertRaises(ValueError):
            self.link.action(self.owner,3,'radar_servo',dict(rightUs=900,centerUs=1500,leftUs=1490))
        self.link.action(self.owner,3,'radar_servo',dict(rightUs=1950,centerUs=1500,leftUs=1000))
        self.assertEqual(self.serial.sent[-1],'RADAR SERVO 1950 1500 1000')

    def test_wall_mode_and_calibration(self):
        self.ready()
        self.link.action(self.owner, 3, 'sonar', dict(enabled=1, interval=25))
        self.assertEqual(self.serial.sent[-1], 'SONAR 1 25')
        self.serial.feed('OK SONAR'); self.link.tick()
        self.link.frame(self.owner, 4, [], 'WALL_MAZE'); self.now += .1; self.link.tick()
        self.assertIn('MODE WALL_MAZE', self.serial.sent)
        self.serial.feed('WALLNAV,1,1,1,1,2,-1,17,0.9,0.8,0.2,40,50,39,48')
        self.link.tick()
        self.assertEqual(self.link.telemetry['wall']['leftRange'], 17)
        self.assertEqual(self.link.telemetry['wall']['current'], 2)

    def test_race_calibration_mode_and_watchdog(self):
        self.ready()
        self.link.action(self.owner, 3, 'race_config', dict(speed=50, offset=100, angle=1.8))
        self.assertEqual(self.serial.sent[-1], 'RACECFG 50 100 1.8')
        self.assertFalse(self.link.ready)
        self.serial.feed('OK RACECFG'); self.link.tick()
        self.assertTrue(self.link.ready)
        self.serial.feed('RACENAV,1,1,50,100,1.8,1'); self.link.tick()
        self.assertEqual(self.link.telemetry['race']['speed'], 50)
        self.link.frame(self.owner, 4, [], 'RACE'); self.now += .11; self.link.tick()
        self.assertIn('MODE RACE', self.serial.sent)
        self.assertNotIn('RUN START', self.serial.sent)
        self.serial.feed('DATA,100,RACE,RACE_SEARCHING_LINE,60,15,20,-20');self.link.tick()
        self.assertEqual(self.link.telemetry['mode'], 'RACE')
        self.now += .7;self.link.tick()
        self.assertFalse(self.link.snapshot()['ready']);self.assertIn('STOP', self.serial.sent)

    def test_saved_calibration_actions(self):
        self.ready()
        self.link.action(self.owner, 3, 'save_calibration', {})
        self.assertEqual(self.serial.sent[-2:], ['STOP','SAVE'])
        self.serial.feed('OK SAVE');self.link.tick();self.assertTrue(self.link.ready)
        self.link.action(self.owner, 4, 'load_calibration', {})
        self.serial.feed('CONFIG_REPORT,200,100,0,1,25,15,0')
        self.serial.feed('OK LOAD');self.link.tick()
        self.assertTrue(self.link.ready);self.assertEqual(self.link.settings['power'],200)
        self.assertEqual(self.link.settings['lineTapeLevel'],0)

    def test_startup_status_cannot_interrupt_handshake(self):
        self.serial.feed('DATA,10,MANUAL,HARDWARE_DIMENSIONS_REQUIRED,50,20,0,0')
        self.link.tick()
        self.assertEqual(self.link.phase,'Waiting for firmware 7')
        self.ready()

    def test_default_direction_calibration(self):
        self.ready()
        self.assertFalse(any(cmd.startswith('CONFIG ') for cmd in self.serial.sent))

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
    def test_browser_loss_stops_without_closing_usb_or_resuming_stale_input(self):
        self.ready(); self.link.frame(self.owner, 3, [], 'FOLLOW'); self.now += .1; self.link.tick()
        self.assertIn('MODE FOLLOW', self.serial.sent)
        self.now += .6; self.serial.feed('ALIVE'); self.link.tick()
        self.assertFalse(self.link.snapshot()['ready'])
        self.assertFalse(self.serial.closed)
        self.assertIn('STOP', self.serial.sent)
        self.assertEqual(self.link.mode, 'MANUAL')
        self.serial.sent.clear()
        # Several minutes with no browser, but a healthy firmware connection.
        for _ in range(1800):
            self.now += .1; self.serial.feed('ALIVE'); self.link.tick()
        self.assertFalse(self.serial.closed)
        self.assertIn('HEARTBEAT', self.serial.sent)
        for seq, keys, mode in [(4, [], 'FOLLOW'), (5, ['Up'], 'MANUAL')]:
            self.link.frame(self.owner, seq, keys, mode)
            self.now += .11; self.serial.feed('ALIVE'); self.link.tick()
            self.assertFalse(self.link.snapshot()['ready'])
        self.assertFalse(any(c.startswith(('MODE FOLLOW', 'DRIVE')) for c in self.serial.sent))
        # UI sees not-ready, clears local commands, and acknowledges neutral.
        self.link.frame(self.owner, 6, [], 'MANUAL')
        self.assertTrue(self.link.snapshot()['ready'])
        self.assertEqual(self.link.mode, 'MANUAL')
        self.link.frame(self.owner, 7, [], 'FOLLOW')
        self.now += .11; self.link.tick()
        self.assertIn('MODE FOLLOW', self.serial.sent)

    def test_serial_loss_despite_live_browser(self):
        self.ready(); self.now += 1.1
        self.link.frame(self.owner, 3, [], 'MANUAL'); self.link.tick()
        self.assertFalse(self.link.ready)
    def test_silent_firmware_keeps_port_and_retries_handshake(self):
        self.ready()
        self.now += 1.1; self.link.frame(self.owner, 3, [], 'LINE'); self.link.tick()
        self.assertFalse(self.serial.closed)
        self.assertFalse(self.link.snapshot()['ready'])
        for seq in range(4, 30):
            self.now += 1
            self.link.frame(self.owner, seq, [], 'LINE'); self.link.tick()
        self.assertFalse(self.serial.closed)
        self.assertGreater(self.serial.sent.count('HELLO'), 5)
        self.serial.feed('ROBOT_READY 7'); self.link.tick()
        self.assertFalse(self.link.snapshot()['ready'])  # Still needs neutral acknowledgement.
        self.link.frame(self.owner, 30, [], 'MANUAL'); self.link.tick()
        self.assertTrue(self.link.snapshot()['ready'])
        self.assertNotIn('MODE LINE', self.serial.sent)

    def test_board_reset_and_watchdog_recover_without_losing_owner(self):
        for notice in ('ROBOT_BOOT 7', 'CONTROL_TIMEOUT'):
            with self.subTest(notice=notice):
                self.setUp(); self.ready()
                self.serial.feed(notice); self.link.tick()
                self.assertFalse(self.serial.closed)
                self.assertEqual(self.link.owner, self.owner)
                self.assertFalse(self.link.snapshot()['ready'])
                self.now += 2.1; self.link.tick()
                self.serial.feed('ROBOT_READY 7'); self.link.tick()
                self.link.frame(self.owner, 3, [], 'MANUAL')
                self.assertTrue(self.link.snapshot()['ready'])

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
        self.serial.feed('ROBOT_READY 7');self.link.tick()
        self.assertFalse(any(cmd.startswith('CONFIG ') for cmd in self.serial.sent))
        self.assertEqual(self.link.settings['lineTapeLevel'],1)
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
        self.assertEqual(len(self.link.trace_csv().splitlines()[1].split(',')),25)
        self.serial.feed('OBS,200,2,2,0,0,0,5,5,300,200,10,20,255,255,1,150,20,0.9,0.8,0.7,40,40,39,41,1')
        self.link.tick();self.assertEqual(len(self.link.trace_rows),2)
        self.link.action(self.owner,4,'trace',dict(enabled=0));self.serial.feed('OK TRACE');self.link.tick()
        self.assertFalse(self.link.trace_recording);self.assertEqual(len(self.link.trace_rows),2)

    def test_keyboard_mapping(self):
        self.assertEqual(wheel_command(['Up','Left'],255),(0,255))
        self.assertEqual(wheel_command(['Down'],255),(-255,-255))
        self.assertEqual(wheel_command(['Left'],255),(-255,255))
        self.assertEqual(wheel_command(['Up','Down'],255),(0,0))

if __name__ == '__main__': unittest.main()
