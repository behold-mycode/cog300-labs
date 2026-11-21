"""One serial connection and one browser owner. Caller holds a lock around calls."""
from collections import deque
import math
import secrets
import time

DEFAULTS = dict(power=255, minimum=255, invertA=True, invertB=False,
                followCm=20, wallCm=10, lineTapeLevel=-1)
ARROWS = {'Up', 'Down', 'Left', 'Right'}
MODES = {'MANUAL', 'FOLLOW', 'WALL', 'LINE', 'MAZE', 'STOP_ON_TAPE', 'STOP_AT_END', 'WALL_MAZE', 'RACE', 'OBJECT', 'CALIBRATE'}
RADAR_RUNS = {'RADAR_SCAN': 'RADAR SCAN', 'RADAR_ARM': 'RADAR ARM'}
BROWSER_TIMEOUT = .6


def validate_settings(values):
    if isinstance(values, dict) and 'lineTapeLevel' not in values:
        values = dict(values, lineTapeLevel=-1)
    if not isinstance(values, dict) or set(values) != set(DEFAULTS):
        raise ValueError('Settings must include power, minimum, motor A/B directions and both distances.')
    for name, low, high in [('power', 1, 255), ('minimum', 0, 255),
                            ('followCm', 5, 100), ('wallCm', 5, 80), ('lineTapeLevel', -1, 1)]:
        if type(values[name]) is not int or not low <= values[name] <= high:
            raise ValueError(f'{name} must be an integer from {low} to {high}.')
    if values['minimum'] > values['power']:
        raise ValueError('Minimum moving PWM cannot exceed power.')
    if any(type(values[name]) is not bool for name in ('invertA', 'invertB')):
        raise ValueError('Wheel reversal settings must be true or false.')
    return values.copy()


def wheel_command(keys, power):
    keys = set(keys)
    if {'Up', 'Down'} <= keys or {'Left', 'Right'} <= keys:
        return 0, 0
    direction = 1 if 'Up' in keys else -1 if 'Down' in keys else 0
    if direction:
        left = 0 if 'Left' in keys else power
        right = 0 if 'Right' in keys else power
        return direction * left, direction * right
    if 'Left' in keys: return -power, power
    if 'Right' in keys: return power, -power
    return 0, 0


def open_serial(port):
    import serial
    return serial.Serial(port, 115200, timeout=0, write_timeout=.1)


class RobotLink:
    def __init__(self, clock=time.monotonic, serial_factory=open_serial):
        self.clock, self.serial_factory = clock, serial_factory
        self.serial = None
        self.owner = None
        self.ready = False
        self.browser_paused = False
        self.phase = 'Disconnected'
        self.settings = DEFAULTS.copy()
        self.keys, self.mode = [], 'MANUAL'
        self.telemetry = {}
        self.navigation = {}
        self.map_exporting = False
        self.pending_navigation = None
        self.maze_map = dict(nodes={}, edges={}, complete=False)
        self.trace_rows=[];self.trace_recording=False
        self.history = deque(maxlen=600)
        self.arena_capable = False
        self.radar_settings = {}
        self.pending_radar_setting = None
        self.buffer = bytearray()
        self.reset_radar()

    def reset_radar(self):
        self.previous_bench = None
        self.sweep = None
        self.samples = {}
        self.radar_lock = None
        self.lock_updated = None
        self.radar_updated = None
        self.radar_moving = False

    def bench_contours(self):
        # Both neighboring bearings must have been observed to bound a surface.
        spacing = self.radar_settings.get('step',3)
        points = self.samples
        contours = []
        run = []
        def finish():
            if len(run) < 2:
                return
            first, last = run[0][0], run[-1][0]
            distance = sum(p[1] for p in run)/len(run)
            left, right = points.get(first-spacing), points.get(last+spacing)
            if first == -60 or last == 60:
                status = 'edge clipped'
            elif left is None or right is None:
                status = 'incomplete'
            else:
                status = 'bounded'
            width = round(2*distance*math.sin(math.radians(last-first+spacing)/2))
            contours.append(dict(bearing=(first+last)/2, range=round(distance),
                                 width=width, status=status))
        for angle, distance, segment in sorted(points.values()):
            if run and (angle != run[-1][0]+spacing or segment != run[-1][2] or distance <= 0):
                finish()
                run = []
            if distance > 0 and segment > 0:
                run.append((angle, distance, segment))
        finish()
        return contours

    def bench_active(self):
        return self.sent_mode == 'RADAR_SCAN' or self.telemetry.get('mode') == 'RADAR_SCAN'

    def radar_record(self, line):
        try:
            kind, *fields = line.split(',')
            if kind == 'RADAR':
                sweep, angle, distance, segment = map(int, fields)
                if not (0 <= sweep <= 0xffff and -90 <= angle <= 90 and
                        (distance == -1 or 0 < distance <= 10000) and 0 <= segment <= 255):
                    return
                if self.radar_moving:
                    return
                if self.sweep != sweep:
                    if self.sweep is not None and ((sweep - self.sweep) & 0xffff) >= 0x8000:
                        return
                    if self.sweep is not None and self.bench_active():
                        self.previous_bench = dict(sweep=self.sweep, contours=self.bench_contours())
                    self.samples = {}
                    self.sweep = sweep
                self.samples[angle] = [angle, distance, segment]
                self.radar_updated = self.clock()
            elif kind == 'LOCK':
                angle, distance, width = map(int, fields)
                self.radar_lock = None
                if not self.radar_moving and self.samples and -90 <= angle <= 90 and 0 < distance <= 10000 and 0 < width <= 4000:
                    self.radar_lock = [angle, distance, width]
                    self.lock_updated = self.clock()
        except ValueError:
            return

    def radar_snapshot(self):
        age = None if self.radar_updated is None else max(0, self.clock() - self.radar_updated)
        fresh = age is not None and age <= 2 and not self.radar_moving
        bench = fresh and self.bench_active()
        return dict(capable=self.arena_capable, bench=self.bench_contours() if bench else None,
                    previousBench=self.previous_bench if bench else None,
                    settings=self.radar_settings.copy(), sweep=self.sweep, age=age, moving=self.radar_moving,
                    samples=list(self.samples.values()) if fresh else [],
                    lock=self.radar_lock if fresh and self.lock_updated is not None and self.clock()-self.lock_updated <= 2 else None)

    def log(self, text):
        self.history.append(time.strftime('%H:%M:%S') + ' ' + text)

    def connect(self, port):
        if self.serial is not None:
            raise ValueError('A controller is already connected. Disconnect it first.')
        if not isinstance(port, str) or not port or len(port) > 256:
            raise ValueError('Choose a serial port.')
        self.serial = self.serial_factory(port)
        self.owner = secrets.token_urlsafe(24)
        self.ready = False
        self.browser_paused = False
        self.phase = 'Waiting for firmware 7'
        self.keys, self.mode = [], 'MANUAL'
        self.reset_radar()
        self.arena_capable = False
        self.radar_settings = {}
        self.pending_radar_setting = None
        self.buffer.clear(); self.telemetry = {}; self.navigation = {}
        self.map_exporting = False
        self.pending_navigation = None
        self.maze_map = dict(nodes={}, edges={}, complete=False)
        self.seq = -1
        self.started = self.last_browser = self.last_rx = self.clock()
        self.last_tx = self.last_hello = -100.
        self.sent_mode, self.sent_motion = 'MANUAL', (0, 0)
        self.log('Connected port ' + port)
        return self.owner

    def _authorize(self, owner, seq):
        if not self.serial or owner != self.owner:
            raise ValueError('Connection ended. Connect again.')
        if type(seq) is not int or not self.seq < seq < 2**53:
            raise ValueError('Stale controller request ignored.')
        self.seq = seq

    def frame(self, owner, seq, keys, mode):
        if (not isinstance(keys, list) or len(keys) > 4 or
                any(not isinstance(k, str) or k not in ARROWS for k in keys) or
                not isinstance(mode, str) or mode not in MODES | RADAR_RUNS.keys()):
            raise ValueError('Invalid keyboard or mode input.')
        self._authorize(owner, seq)
        if self.ready and (mode == 'OBJECT' or mode in RADAR_RUNS) and not self.arena_capable:
            self.keys, self.mode = [], 'MANUAL'
            self._send('STOP')
            raise ValueError('Upload the arena scanner firmware to start this mode.')
        if mode=='OBJECT':
            raise ValueError('Object finding starts at the maze exit tape.')
        self.last_browser = self.clock()
        if self.browser_paused:
            if keys or mode != 'MANUAL': return
            self.browser_paused = False
        # Never queue movement while waiting for handshake/settings acknowledgement.
        self.keys = keys if self.ready and mode == 'MANUAL' else []
        self.mode = mode if self.ready else 'MANUAL'
        if self.map_exporting: self.keys,self.mode=[], 'MANUAL'

    def _send(self, command):
        data = (command + '\n').encode('ascii')
        if self.serial.write(data) != len(data):
            raise OSError('Incomplete serial write')
        if command != 'HEARTBEAT' and not command.startswith('DRIVE '):
            self.log('> ' + command)

    def _config(self):
        s = self.settings
        self._send('CONFIG %d %d %d %d %d %d %d' % (
            s['power'], s['minimum'], s['invertA'], s['invertB'], s['followCm'], s['wallCm'], s['lineTapeLevel']))

    def configure(self, owner, seq, settings):
        settings = validate_settings(settings)
        self._authorize(owner, seq)
        if not self.ready: raise ValueError('Wait for the connection to finish.')
        if self.map_exporting: raise ValueError('Wait for map export to finish.')
        self._send('STOP')
        self.settings = settings
        self.keys, self.mode = [], 'MANUAL'
        self.ready = False; self.phase = 'Applying settings'
        self.last_browser = self.last_rx = self.clock()
        self._config()

    def action(self, owner, seq, action, values):
        commands = {
            'radar_step': ('RADAR STEP', [('step',1,5)]),
            'radar_servo': ('RADAR SERVO', [('rightUs',800,2200),('centerUs',800,2200),('leftUs',800,2200)]),
            'radar_profile': ('RADAR PROFILE', [('minMm',1,2000),('maxMm',1,2000)]),
            'radar_clearance': ('RADAR CLEARANCE', [('radius',0,2000)]),
            'race_config': ('RACECFG', [('speed',1,300),('offset',1,999),('angle',.5,3.1)]),
            'save_calibration': ('SAVE', []), 'load_calibration': ('LOAD', []),
            'sonar': ('SONAR', [('enabled',0,1),('interval',25,100)]),
            'body': ('BODY', [('front',1,1000),('rear',1,1000),('halfWidth',1,1000),('frontSensor',0,1000),('sideX',-1000,1000),('sideY',0,1000)]),
            'wall_config': ('WALLCFG', [('cruise',1,500),('turnSpeed',1,300),('margin',1,200),('deceleration',1,10000),('reaction',.1,5),('matchRadius',1,500),('corridorWidth',10,5000)]),
            'speed_config': ('SPEEDCFG', [('fullLeft',1,3000),('fullRight',1,3000),('kp',0,10),('ki',0,10),('acceleration',1,3000)]),
            'encoders': ('ENC', [('kind',0,2),('invertL',0,1),('invertR',0,1)]),
            'odometry': ('ODOM', [('left',.001,100),('right',.001,100),('track',10,1000)]),
            'geometry': ('GEOMETRY', [('offset',1,1000),('width',1,500),('match',1,2000),('clearance',1,1000)]),
            'trace': ('TRACE', [('enabled',0,1)]),
            'dynamics': ('DYNAMICS', [('deceleration',1,10000),('reaction',.001,5),('margin',1,500)]),
            'model': ('MODEL', [('hit',.501,.999),('falsePositive',.001,.499),('change',.001,.999)]),
            'map_export': ('MAP DUMP', []), 'map_reset': ('MAP RESET', []),
            'map_confirm': ('MAP CONFIRM', [('node',0,63)])}
        if not isinstance(action,str) or action not in commands or not isinstance(values,dict):
            raise ValueError('Unknown navigation action.')
        prefix, fields = commands[action]
        if set(values) != {f[0] for f in fields}: raise ValueError('Incomplete calibration fields.')
        args=[]
        for name,low,high in fields:
            v=values[name]
            if type(v) not in (int,float) or not math.isfinite(v) or not low <= v <= high:
                raise ValueError(name + ' is outside its valid range.')
            if action in ('encoders','map_confirm','trace','sonar','radar_profile','radar_clearance','radar_servo','radar_step') and type(v) is not int:
                raise ValueError(name + ' must be an integer.')
            args.append(format(v,'.6g'))
        if action == 'radar_step' and values['step'] not in (1,2,3,5):
            raise ValueError('Bench spacing must be 1, 2, 3 or 5 degrees.')
        if action == 'radar_servo':
            step1 = values['centerUs'] - values['rightUs']
            step2 = values['leftUs'] - values['centerUs']
            if step1 * step2 <= 0 or min(abs(step1),abs(step2)) < 100:
                raise ValueError('Servo pulses must increase or decrease in steps of at least 100 µs.')
        if action == 'radar_profile' and values['maxMm'] < values['minMm']:
            raise ValueError('Maximum width must be at least the minimum width.')
        if action=='sonar' and values['enabled']==0 and values['interval']<35:
            raise ValueError('Two sensors need at least 35 ms spacing.')
        if action=='body' and (values['frontSensor']>values['front'] or values['sideY']>values['halfWidth'] or not -values['rear']<=values['sideX']<=values['front']):
            raise ValueError('Sonar offsets must lie within the measured footprint.')
        command=' '.join([prefix]+args)
        if len(command)>90: raise ValueError('Calibration command too long.')
        self._authorize(owner,seq)
        if not self.ready: raise ValueError('Connect before configuring navigation.')
        if action.startswith('radar_') and not self.arena_capable:
            raise ValueError('Upload the arena scanner firmware to configure the radar.')
        if self.map_exporting: raise ValueError('Wait for map export to finish.')
        self.keys,self.mode=[], 'MANUAL'; self.sent_mode='MANUAL'; self.sent_motion=(0,0)
        self.last_browser=self.clock()
        if action in ('radar_profile','radar_clearance','radar_servo','radar_step'):
            self.pending_radar_setting = values.copy()
        if action=='trace':
            self.trace_recording=bool(values['enabled'])
            if self.trace_recording: self.trace_rows=[]
        self._send('STOP');self._send(command)
        if action == 'map_export':
            self.map_exporting = True; self.phase = 'Exporting map — motors stopped'
        else:
            self.pending_navigation = 'OK ' + prefix
            self.ready = False; self.phase = 'Applying navigation settings'
        if action in ('map_export','map_reset'):
            self.maze_map=dict(nodes={},edges={},complete=False)

    def _map_record(self, line):
        try:
            p=line.split(',')
            if p[0]=='MAP_BEGIN' and len(p)==3:
                n,e=map(int,p[1:])
                if not (0<=n<=64 and 0<=e<=128): return
                self.maze_map=dict(nodes={},edges={},complete=False,expectedNodes=n,expectedEdges=e)
            elif p[0]=='MAP_NODE' and 5<=len(p)<=10:
                n=int(p[1]);x,y=map(float,p[2:4])
                if not 0<=n<64 or not all(math.isfinite(v) for v in (x,y)): return
                ports=[]
                for item in p[4:]:
                    h,edge,blocked=map(float,item.split(':'))
                    if not math.isfinite(h) or not edge.is_integer() or not -1<=edge<128 or blocked not in (0,1): return
                    ports.append(dict(bearing=h,edge=int(edge),blocked=bool(blocked)))
                self.maze_map['nodes'][n]=dict(id=n,x=x,y=y,ports=ports)
            elif p[0]=='MAP_EDGE' and len(p)==8:
                i,a,ap,b,bp=map(int,p[1:6]);length=float(p[6]);visits=int(p[7])
                if not (0<=i<128 and 0<=a<64 and 0<=b<64 and 0<=ap<6 and 0<=bp<6 and math.isfinite(length) and length>0 and visits>0): return
                self.maze_map['edges'][i]=dict(id=i,a=a,ap=ap,b=b,bp=bp,length=length,traversals=visits)
            elif p[0]=='MAP_END':
                m=self.maze_map
                valid=(set(m['nodes'])==set(range(m.get('expectedNodes',-1))) and
                       set(m['edges'])==set(range(m.get('expectedEdges',-1))) and 'expectedNodes' in m)
                for edge in m['edges'].values():
                    for node,port in ((edge['a'],edge['ap']),(edge['b'],edge['bp'])):
                        valid = valid and node in m['nodes'] and port<len(m['nodes'][node]['ports'])
                        if valid: valid = m['nodes'][node]['ports'][port]['edge']==edge['id']
                for node in m['nodes'].values():
                    for i,port in enumerate(node['ports']):
                        if port['edge']==-1: continue
                        edge=m['edges'].get(port['edge'])
                        valid=valid and edge is not None and (node['id'],i) in ((edge['a'],edge['ap']),(edge['b'],edge['bp']))
                m['complete']=valid;self.map_exporting=False
                self.phase='Map exported' if valid else 'Incomplete map export — retry'
        except (ValueError,OverflowError): pass

    def disconnect(self, reason='Disconnected'):
        if self.serial:
            try: self._send('STOP')
            except (OSError, ValueError): pass
            try: self.serial.close()
            except OSError: pass
        self.reset_radar()
        self.trace_recording=False
        self.arena_capable = False
        self.serial = self.owner = None
        self.browser_paused = False
        self.ready = False; self.keys, self.mode = [], 'MANUAL'
        self.map_exporting = False; self.pending_navigation = None
        self.phase = reason
        self.log(reason)

    def _recover(self, reason):
        """Keep the port/owner, but require a fresh stopped firmware session."""
        self.ready = False
        self.browser_paused = True
        self.keys, self.mode = [], 'MANUAL'
        self.sent_mode, self.sent_motion = 'MANUAL', (0, 0)
        self.map_exporting = False; self.pending_navigation = None
        self.reset_radar()
        self.arena_capable = False
        self.radar_settings = {}
        self.pending_radar_setting = None
        self.trace_recording = False
        self.phase = 'Waiting for firmware 7'
        self.started = self.clock() - 2
        self.last_hello = -100.
        self._send('STOP')
        self.log(reason + ' — retrying firmware handshake; USB retained.')

    def _receive(self, line):
        if line != 'ALIVE' and not line.startswith(('OBS,','RADAR,')): self.log('< ' + line)
        if line.startswith('CONFIG_REPORT,') and (self.phase == 'Waiting for firmware 7' or self.pending_navigation == 'OK LOAD'):
            try:
                v=[int(x) for x in line.split(',')[1:]]
                if len(v)!=7 or v[2] not in (0,1) or v[3] not in (0,1): return
                self.settings=validate_settings(dict(power=v[0],minimum=v[1],invertA=bool(v[2]),invertB=bool(v[3]),followCm=v[4],wallCm=v[5],lineTapeLevel=v[6]))
            except ValueError: pass
        elif self.pending_navigation and line == self.pending_navigation:
            if self.pending_radar_setting is not None:
                self.radar_settings.update(self.pending_radar_setting)
                self.pending_radar_setting = None
            self.pending_navigation=None;self.ready=True;self.phase='Navigation settings applied'
        elif line.startswith('BUSY '):
            self.phase=line.replace('_',' ')
        elif line == 'CAPS ARENA_1' and self.phase == 'Waiting for firmware 7':
            self.arena_capable = True
        elif line == 'ROBOT_READY 7' and self.phase == 'Waiting for firmware 7':
            self.ready = True; self.phase = 'Connected'; self.last_rx = self.clock()
            # Firmware owns calibration. Connecting must never overwrite it.
            self.sent_mode, self.sent_motion = 'MANUAL', (0, 0)
        elif line.startswith('ROBOT_READY ') and line != 'ROBOT_READY 7':
            self.disconnect('Upload Robot firmware 7, then reconnect.')
        elif line == 'OK CONFIG' and self.phase == 'Applying settings':
            self.ready = True; self.phase = 'Connected — hold arrows or WASD'
            self.last_rx = self.clock()
            self.sent_mode, self.sent_motion = 'MANUAL', (0, 0)
        elif line == 'ALIVE':
            self.last_rx = self.clock()
        elif line.startswith('CONTROL_TIMEOUT'):
            self._recover('Firmware watchdog expired')
        elif line.startswith('ERROR ') and (self.sent_mode in RADAR_RUNS or self.pending_radar_setting is not None):
            self._send('STOP')
            self.pending_radar_setting = self.pending_navigation = None
            self.keys, self.mode = [], 'MANUAL'
            self.sent_mode, self.sent_motion = 'MANUAL', (0, 0)
            self.ready, self.browser_paused = True, True
            self.phase = line[6:].replace('_', ' ').capitalize()
        elif line.startswith('ERROR'):
            self.disconnect('Firmware error: ' + line)
        elif line.startswith('ROBOT_BOOT') and self.phase != 'Waiting for firmware 7':
            self._recover('Board reset')
        elif line.startswith(('RADAR,','LOCK,')):
            self.radar_record(line)
        elif line in ('OK RADAR SCAN','OK RADAR ARM'):
            self.phase = 'Scanning with wheels stopped' if line.endswith('SCAN') else 'Exit marker armed'
        elif line.startswith('NAV,'):
            try:
                v = [float(x) for x in line.split(',')[1:]]
                if len(v) == 11 and all(math.isfinite(x) for x in v):
                    self.navigation = dict(zip(('encoders','odometry','geometry','localized','nodes','edges','current','candidate','x','y','heading'), v))
            except ValueError: pass
        elif line.startswith('MAP_'):
            self._map_record(line)
        elif line.startswith('OBS,'):
            try:
                fields=line.split(',')[1:];values=[float(x) for x in fields]
                if self.trace_recording and len(values) in (15,25) and all(math.isfinite(x) for x in values):
                    self.trace_rows.append(','.join(fields)+(','*10 if len(fields)==15 else ''))
                    if len(self.trace_rows)>=20000:
                        self.trace_recording=False;self._send('TRACE 0')
                        self.log('Trace stopped at 20,000 samples; download before starting another.')
            except ValueError: pass
        elif line.startswith('RACENAV,'):
            try:
                v=[float(x) for x in line.split(',')[1:]]
                if len(v)==6 and all(math.isfinite(x) for x in v) and v[0] in (0,1) and v[1] in (0,1):
                    self.telemetry['race']=dict(zip(('configured','saved','speed','offset','angle','stage'),v))
            except ValueError: pass
        elif line.startswith('WALLNAV,'):
            try:
                v=[float(x) for x in line.split(',')[1:]]
                if len(v)==14 and all(math.isfinite(x) for x in v):
                    self.telemetry['wall']=dict(zip(('sonar','calibrated','localized','ownsMap','current','candidate','leftRange','frontWall','rightWall','leftWall','targetL','targetR','speedL','speedR'),v))
            except ValueError: pass
        elif line.startswith('BELIEF,'):
            try:
                v=[float(x) for x in line.split(',')[1:]]
                if len(v)==10 and all(math.isfinite(x) for x in v):
                    self.telemetry.update(dynamics=bool(v[0]),belief=v[1:6],wheelSpeed=v[6:8],headingUncertainty=v[8],beliefUpdates=v[9])
            except ValueError: pass
        elif line.startswith('COUNTS,'):
            try:
                v=[int(x) for x in line.split(',')[1:]]
                if len(v)==3 and v[2] in (0,1,2) and all(-2**31<=x<2**31 for x in v[:2]):
                    previous = self.telemetry.get('encoderCounts')
                    if previous is not None and previous != v[:2]:
                        self.reset_radar()
                    self.telemetry.update(encoderCounts=v[:2],encoderKind=v[2])
            except ValueError: pass
        elif line.startswith('IR,'):
            try:
                raw = [int(v) for v in line.split(',')[1:]]
                if len(raw) == 4 and all(v in (0, 1) for v in raw[:3]) and raw[3] in (-1, 0, 1):
                    self.telemetry.update(ir=raw[:3], tapeLevel=raw[3])
            except ValueError: pass
        elif line.startswith('LINECTRL,'):
            try:
                values = [float(v) for v in line.split(',')[1:]]
                if len(values) == 8 and all(math.isfinite(v) for v in values):
                    self.telemetry['lineControl'] = dict(zip(
                        ('phase','cruise','minimum','saturated','heading','remaining','speedL','speedR'),values))
            except ValueError: pass
        elif line.startswith('DATA,'):
            parts = line.split(',')
            if len(parts) != 8 or parts[2] not in MODES | {'RADAR_SCAN'}: return
            try:
                values = [float(p) for p in parts[4:]]
                if not all(math.isfinite(v) for v in values): return
                if values[2] or values[3]:
                    self.reset_radar()
                    self.radar_moving = True
                else:
                    self.radar_moving = False
                self.telemetry.update(ms=int(parts[1]), mode=parts[2], state=parts[3],
                                      front=values[0], right=values[1], left=values[2], motorRight=values[3])
            except ValueError: return
            if self.ready and parts[2] == 'MANUAL' and parts[3] not in ('STOPPED','DRIVING','TIMED_TEST'):
                self.keys, self.mode = [], 'MANUAL'
                self.sent_mode = 'MANUAL'
                self.phase = 'Stopped: ' + parts[3].replace('_',' ').lower()


    def tick(self):
        if not self.serial: return
        try:
            now = self.clock()
            if self.ready and self.mode == 'MANUAL' and now - self.last_browser > BROWSER_TIMEOUT and not self.browser_paused:
                self._send('STOP')
                self.browser_paused = True
                self.keys, self.mode = [], 'MANUAL'
                self.sent_mode, self.sent_motion = 'MANUAL', (0, 0)
                self.log('Browser inactive — motors stopped; USB connection retained.')
            self.buffer.extend(self.serial.read(min(self.serial.in_waiting, 2048)))
            if len(self.buffer) > 4096:
                self.disconnect('Invalid serial stream — reconnect.'); return
            for _ in range(40):
                if b'\n' not in self.buffer: break
                raw, _, self.buffer = self.buffer.partition(b'\n')
                if len(raw) > 384:
                    self.disconnect('Invalid serial line — reconnect.'); return
                self._receive(raw.decode('ascii', errors='replace').strip())
                if not self.serial: return
            if len(self.buffer) > 384 and b'\n' not in self.buffer:
                self.disconnect('Invalid serial line — reconnect.'); return
            if self.phase == 'Waiting for firmware 7':
                if now - self.started >= 2 and now - self.last_hello >= .5:
                    self._send('HELLO'); self.last_hello = now
                return
            if now - self.last_rx > 1:
                self._recover('Firmware response delayed'); return
            if now - self.last_tx < .1: return
            self.last_tx = now
            self._send('HEARTBEAT')
            if not self.ready: return
            if self.mode != self.sent_mode:
                self.reset_radar()
                if self.mode == 'MANUAL' and self.sent_mode in RADAR_RUNS:
                    self._send('STOP')
                else:
                    if self.mode == 'RADAR_ARM' and self.telemetry.get('mode') not in ('WALL', 'WALL_MAZE'):
                        self._send('STOP')
                    if self.mode == 'RADAR_SCAN':
                        # A previous session may have used another bench step.
                        step = self.radar_settings.get('step',3)
                        self._send(f'RADAR STEP {step}')
                    self._send(RADAR_RUNS.get(self.mode, 'MODE ' + self.mode))
                self.sent_mode = self.mode; self.sent_motion = (0, 0)
            if self.mode == 'MANUAL':
                motion = wheel_command(self.keys, self.settings['power'])
                if motion != (0, 0): self._send('DRIVE %d %d' % motion)
                elif self.sent_motion != (0, 0): self._send('STOP')
                self.sent_motion = motion
        except (OSError, ValueError) as error:
            self.disconnect('Serial error: ' + str(error))

    def trace_csv(self):
        header='ms,sequence,tapeMask,xMm,yMm,headingRad,leftMm,rightMm,frontMm,rightMmRange,frontAgeMs,rightAgeMs,leftPWM,rightPWM,odometryConfigured,leftRangeMm,leftAgeMs,frontWallProbability,rightWallProbability,leftWallProbability,targetLeftMmS,targetRightMmS,measuredLeftMmS,measuredRightMmS,wallState'
        return header+'\n'+'\n'.join(self.trace_rows)+'\n'

    def snapshot(self):
        return dict(connected=self.serial is not None, ready=self.ready and not self.browser_paused,
                    status=(self.phase + ' — USB open; motors disabled') if not self.ready and self.serial else
                           'Browser inactive — motors stopped; USB connected' if self.browser_paused else self.phase,
                    mode=self.mode, settings=self.settings, radar=self.radar_snapshot(), telemetry=self.telemetry.copy(), navigation=self.navigation, mazeMap=self.maze_map, mapExporting=self.map_exporting,
                    traceRows=len(self.trace_rows),traceRecording=self.trace_recording,log=list(self.history))


class DemoSerial:
    """Protocol simulation for UI checks. Never opens or controls hardware."""
    def __init__(self, _port):
        self.pending = bytearray(); self.mode = 'MANUAL'; self.left = self.right = 0
        self.last_log = 0.
        self.demo_ray = 0
        self.demo_step = 3
    def write(self, data):
        cmd = data.decode().strip()
        if cmd == 'HELLO': self.pending.extend(b'CAPS ARENA_1\nROBOT_READY 7\n')
        elif cmd == 'MAP DUMP': self.pending.extend(b'MAP_BEGIN,0,0\nMAP_END\n')
        elif cmd in ('SAVE','LOAD'):
            self.pending.extend(('OK '+cmd+'\n').encode())
        elif cmd.startswith(('RACECFG ','SONAR ','BODY ','WALLCFG ','SPEEDCFG ','ENC ','ODOM ','GEOMETRY ','DYNAMICS ','MODEL ','TRACE ','MAP ')):
            prefix=' '.join(cmd.split()[:2]) if cmd.startswith('MAP ') else cmd.split()[0]
            self.pending.extend(('OK '+prefix+'\n').encode())
        elif cmd.startswith('RADAR '):
            action = cmd.split()[1]
            if action == 'SCAN':
                self.mode = 'RADAR_SCAN'
                self.demo_ray = 0
                self.left = self.right = 0
            elif action == 'STEP':
                self.demo_step = int(cmd.split()[2])
            elif action == 'ARM':
                self.mode = 'WALL'
            self.pending.extend(('OK RADAR ' + action + '\n').encode())
        elif cmd.startswith('CONFIG '): self.pending.extend(b'OK CONFIG\n')
        elif cmd == 'HEARTBEAT':
            self.pending.extend(b'ALIVE\n')
            if self.mode == 'RADAR_SCAN':
                sweep, index = divmod(self.demo_ray, 120 // self.demo_step + 1)
                angle = index * self.demo_step - 60
                distance = 510 if -9 <= angle <= 9 else 780 if 27 <= angle <= 45 else -1
                segment = 1 if distance == 510 else 2 if distance == 780 else 0
                self.pending.extend(f'RADAR,{sweep},{angle},{distance},{segment}\n'.encode())
                self.demo_ray += 1
            if time.monotonic() - self.last_log >= .5:
                self.last_log = time.monotonic()
                self.pending.extend(f'DATA,1000,{self.mode},DEMO,30,20,{self.left},{self.right}\n'.encode())
        elif cmd == 'STOP': self.mode = 'MANUAL'; self.left = self.right = 0
        elif cmd.startswith('MODE '): self.mode = cmd.split()[1]
        elif cmd.startswith('DRIVE '): self.left, self.right = map(int, cmd.split()[1:])
        return len(data)
    @property
    def in_waiting(self): return len(self.pending)
    def read(self, count):
        result = bytes(self.pending[:count]); del self.pending[:count]; return result
    def close(self): pass
