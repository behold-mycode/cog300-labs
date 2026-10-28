"""One serial connection and one browser owner. Caller holds a lock around calls."""
from collections import deque
import math
import secrets
import time

DEFAULTS = dict(power=255, minimum=255, invertA=True, invertB=False,
                followCm=20, wallCm=20)
ARROWS = {'Up', 'Down', 'Left', 'Right'}
MODES = {'MANUAL', 'FOLLOW', 'WALL'}
BROWSER_TIMEOUT = .6


def validate_settings(values):
    if not isinstance(values, dict) or set(values) != set(DEFAULTS):
        raise ValueError('Settings must include power, minimum, motor A/B directions and both distances.')
    for name, low, high in [('power', 1, 255), ('minimum', 0, 255),
                            ('followCm', 5, 100), ('wallCm', 5, 80)]:
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
        left = power // 2 if 'Left' in keys else power
        right = power // 2 if 'Right' in keys else power
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
        self.phase = 'Waiting for firmware 4'
        self.keys, self.mode = [], 'MANUAL'
        self.buffer.clear(); self.telemetry = {}
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

    def _send(self, command):
        data = (command + '\n').encode('ascii')
        if self.serial.write(data) != len(data):
            raise OSError('Incomplete serial write')
        if command != 'HEARTBEAT' and not command.startswith('DRIVE '):
            self.log('> ' + command)

    def _config(self):
        s = self.settings
        self._send('CONFIG %d %d %d %d %d %d' % (
            s['power'], s['minimum'], s['invertA'], s['invertB'], s['followCm'], s['wallCm']))

    def configure(self, owner, seq, settings):
        settings = validate_settings(settings)
        self._authorize(owner, seq)
        if not self.ready: raise ValueError('Wait for the connection to finish.')
        self._send('STOP')
        self.settings = settings
        self.keys, self.mode = [], 'MANUAL'
        self.ready = False; self.phase = 'Applying settings'
        self.last_browser = self.last_rx = self.clock()
        self._config()

    def disconnect(self, reason='Disconnected'):
        if self.serial:
            try: self._send('STOP')
            except (OSError, ValueError): pass
            try: self.serial.close()
            except OSError: pass
        self.serial = self.owner = None
        self.ready = False; self.keys, self.mode = [], 'MANUAL'
        self.phase = reason
        self.log(reason)

    def _receive(self, line):
        if line != 'ALIVE': self.log('< ' + line)
        if line == 'ROBOT_READY 4' and self.phase == 'Waiting for firmware 4':
            self.phase = 'Applying settings'; self.last_rx = self.clock()
            self._config()
        elif line.startswith('ROBOT_READY ') and line != 'ROBOT_READY 4':
            self.disconnect('Upload Robot firmware 4, then reconnect.')
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
        elif line.startswith('DATA,'):
            parts = line.split(',')
            if len(parts) != 8 or parts[2] not in MODES: return
            try:
                values = [float(p) for p in parts[4:]]
                if not all(math.isfinite(v) for v in values): return
                self.telemetry = dict(ms=int(parts[1]), mode=parts[2], state=parts[3],
                                      front=values[0], right=values[1], left=values[2], motorRight=values[3])
            except ValueError: return
            if parts[3] == 'TURN_TIMEOUT':
                self.disconnect('Wall turn timed out — check the path, then reconnect.')

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
            if self.phase == 'Waiting for firmware 4':
                if now - self.started > 10:
                    self.disconnect('No firmware 4 response — upload the sketch and reconnect.'); return
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

    def snapshot(self):
        return dict(connected=self.serial is not None, ready=self.ready, status=self.phase,
                    mode=self.mode, settings=self.settings, telemetry=self.telemetry,
                    log=list(self.history))


class DemoSerial:
    """Protocol simulation for UI checks. Never opens or controls hardware."""
    def __init__(self, _port):
        self.pending = bytearray(); self.mode = 'MANUAL'; self.left = self.right = 0
        self.last_log = 0.
    def write(self, data):
        cmd = data.decode().strip()
        if cmd == 'HELLO': self.pending.extend(b'ROBOT_READY 4\n')
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
