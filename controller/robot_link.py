"""One serial connection and one browser owner. Caller holds a lock around calls."""
from collections import deque
import math
import secrets
import time

DEFAULTS = dict(power=255, minimum=255, invertA=True, invertB=False,
                followCm=20, wallCm=10, lineTapeLevel=-1)
ARROWS = {'Up', 'Down', 'Left', 'Right'}
MODES = {'MANUAL', 'FOLLOW', 'WALL', 'LINE', 'MAZE', 'STOP_ON_TAPE', 'STOP_AT_END'}
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
        self.buffer = bytearray()

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
        self.phase = 'Waiting for firmware 6'
        self.keys, self.mode = [], 'MANUAL'
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
                not isinstance(mode, str) or mode not in MODES):
            raise ValueError('Invalid keyboard or mode input.')
        self._authorize(owner, seq)
        self.last_browser = self.clock()
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
            if action in ('encoders','map_confirm','trace') and type(v) is not int:
                raise ValueError(name + ' must be an integer.')
            args.append(format(v,'.6g'))
        command=' '.join([prefix]+args)
        if len(command)>90: raise ValueError('Calibration command too long.')
        self._authorize(owner,seq)
        if not self.ready: raise ValueError('Connect before configuring navigation.')
        if self.map_exporting: raise ValueError('Wait for map export to finish.')
        self.keys,self.mode=[], 'MANUAL'; self.sent_mode='MANUAL'; self.sent_motion=(0,0)
        self.last_browser=self.clock()
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
        self.trace_recording=False
        self.serial = self.owner = None
        self.ready = False; self.keys, self.mode = [], 'MANUAL'
        self.map_exporting = False; self.pending_navigation = None
        self.phase = reason
        self.log(reason)

    def _receive(self, line):
        if line != 'ALIVE' and not line.startswith('OBS,'): self.log('< ' + line)
        if line.startswith('CONFIG_REPORT,') and self.phase == 'Waiting for firmware 6':
            try:
                v=[int(x) for x in line.split(',')[1:]]
                if len(v)!=7 or v[2] not in (0,1) or v[3] not in (0,1): return
                self.settings=validate_settings(dict(power=v[0],minimum=v[1],invertA=bool(v[2]),invertB=bool(v[3]),followCm=v[4],wallCm=v[5],lineTapeLevel=v[6]))
            except ValueError: pass
        elif self.pending_navigation and line == self.pending_navigation:
            self.pending_navigation=None;self.ready=True;self.phase='Navigation settings applied'
        elif line.startswith('BUSY '):
            self.phase=line.replace('_',' ')
        elif line == 'ROBOT_READY 6' and self.phase == 'Waiting for firmware 6':
            self.phase = 'Applying settings'; self.last_rx = self.clock()
            self._config()
        elif line.startswith('ROBOT_READY ') and line != 'ROBOT_READY 6':
            self.disconnect('Upload Robot firmware 6, then reconnect.')
        elif line == 'OK CONFIG' and self.phase == 'Applying settings':
            self.ready = True; self.phase = 'Connected — hold arrows or WASD'
            self.last_rx = self.clock()
            self.sent_mode, self.sent_motion = 'MANUAL', (0, 0)
        elif line == 'ALIVE':
            self.last_rx = self.clock()
        elif line.startswith(('CONTROL_TIMEOUT', 'ERROR')):
            self.disconnect('Firmware stopped — reconnect. ' + line)
        elif line.startswith('ROBOT_BOOT') and self.ready:
            self.disconnect('Board reset — reconnect.')
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
                if self.trace_recording and len(values)==15 and all(math.isfinite(x) for x in values):
                    self.trace_rows.append(','.join(fields))
                    if len(self.trace_rows)>=20000:
                        self.trace_recording=False;self._send('TRACE 0')
                        self.log('Trace stopped at 20,000 samples; download before starting another.')
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
                    self.telemetry.update(encoderCounts=v[:2],encoderKind=v[2])
            except ValueError: pass
        elif line.startswith('IR,'):
            try:
                raw = [int(v) for v in line.split(',')[1:]]
                if len(raw) == 4 and all(v in (0, 1) for v in raw[:3]) and raw[3] in (-1, 0, 1):
                    self.telemetry.update(ir=raw[:3], tapeLevel=raw[3])
            except ValueError: pass
        elif line.startswith('DATA,'):
            parts = line.split(',')
            if len(parts) != 8 or parts[2] not in MODES: return
            try:
                values = [float(p) for p in parts[4:]]
                if not all(math.isfinite(v) for v in values): return
                self.telemetry.update(ms=int(parts[1]), mode=parts[2], state=parts[3],
                                      front=values[0], right=values[1], left=values[2], motorRight=values[3])
            except ValueError: return
            if parts[2] == 'MANUAL' and parts[3] not in ('STOPPED','DRIVING','TIMED_TEST'):
                self.keys, self.mode = [], 'MANUAL'
                self.sent_mode = 'MANUAL'
                self.phase = 'Stopped: ' + parts[3].replace('_',' ').lower()


    def tick(self):
        if not self.serial: return
        try:
            now = self.clock()
            if now - self.last_browser > BROWSER_TIMEOUT:
                self.disconnect('Browser stopped responding — reconnect.'); return
            self.buffer.extend(self.serial.read(min(self.serial.in_waiting, 2048)))
            if len(self.buffer) > 4096:
                self.disconnect('Invalid serial stream — reconnect.'); return
            for _ in range(40):
                if b'\n' not in self.buffer: break
                raw, _, self.buffer = self.buffer.partition(b'\n')
                if len(raw) > 256:
                    self.disconnect('Invalid serial line — reconnect.'); return
                self._receive(raw.decode('ascii', errors='replace').strip())
                if not self.serial: return
            if len(self.buffer) > 256 and b'\n' not in self.buffer:
                self.disconnect('Invalid serial line — reconnect.'); return
            if self.phase == 'Waiting for firmware 6':
                if now - self.started > 10:
                    self.disconnect('No firmware 6 response — upload the sketch and reconnect.'); return
                if now - self.started >= 2 and now - self.last_hello >= .5:
                    self._send('HELLO'); self.last_hello = now
                return
            if now - self.last_rx > 1:
                self.disconnect('Serial connection lost — reconnect.'); return
            if now - self.last_tx < .1: return
            self.last_tx = now
            self._send('HEARTBEAT')
            if not self.ready: return
            if self.mode != self.sent_mode:
                self._send('MODE ' + self.mode)
                self.sent_mode = self.mode; self.sent_motion = (0, 0)
            if self.mode == 'MANUAL':
                motion = wheel_command(self.keys, self.settings['power'])
                if motion != (0, 0): self._send('DRIVE %d %d' % motion)
                elif self.sent_motion != (0, 0): self._send('STOP')
                self.sent_motion = motion
        except (OSError, ValueError) as error:
            self.disconnect('Serial error: ' + str(error))

    def trace_csv(self):
        header='ms,sequence,tapeMask,xMm,yMm,headingRad,leftMm,rightMm,frontMm,rightMmRange,frontAgeMs,rightAgeMs,leftPWM,rightPWM,odometryConfigured'
        return header+'\n'+'\n'.join(self.trace_rows)+'\n'

    def snapshot(self):
        return dict(connected=self.serial is not None, ready=self.ready, status=self.phase,
                    mode=self.mode, settings=self.settings, telemetry=self.telemetry, navigation=self.navigation, mazeMap=self.maze_map, mapExporting=self.map_exporting,
                    traceRows=len(self.trace_rows),traceRecording=self.trace_recording,log=list(self.history))


class DemoSerial:
    """Protocol simulation for UI checks. Never opens or controls hardware."""
    def __init__(self, _port):
        self.pending = bytearray(); self.mode = 'MANUAL'; self.left = self.right = 0
        self.last_log = 0.
    def write(self, data):
        cmd = data.decode().strip()
        if cmd == 'HELLO': self.pending.extend(b'ROBOT_READY 6\n')
        elif cmd == 'MAP DUMP': self.pending.extend(b'MAP_BEGIN,0,0\nMAP_END\n')
        elif cmd.startswith(('ENC ','ODOM ','GEOMETRY ','DYNAMICS ','MODEL ','TRACE ','MAP ')):
            prefix=' '.join(cmd.split()[:2]) if cmd.startswith('MAP ') else cmd.split()[0]
            self.pending.extend(('OK '+prefix+'\n').encode())
        elif cmd.startswith('CONFIG '): self.pending.extend(b'OK CONFIG\n')
        elif cmd == 'HEARTBEAT':
            self.pending.extend(b'ALIVE\n')
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
