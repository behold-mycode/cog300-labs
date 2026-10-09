"""Robot control state machine. No serial or GUI dependency, and no hardware auto-connect."""

def wheel_command(keys, speed):
    if {'Up', 'Down'} <= keys or {'Left', 'Right'} <= keys:
        return 0, 0
    direction = 1 if 'Up' in keys else -1 if 'Down' in keys else 0
    if direction:
        left = right = direction * speed
        if 'Left' in keys: left //= 2
        if 'Right' in keys: right //= 2
        return left, right
    if 'Left' in keys: return -speed, speed
    if 'Right' in keys: return speed, -speed
    return 0, 0


class Controller:
    HOTKEYS = {'F1': 'Wheels', 'F2': 'Straight', 'F3': 'Turns', 'F4': 'Spins', 'F5': 'Full lab'}
    ARROWS = {'Up', 'Down', 'Left', 'Right'}

    def __init__(self, send, clock, log=lambda message: None):
        self.send, self.clock, self.log = send, clock, log
        self.connected = self.ready = self.enabled = False
        self.phase = 'Disconnected'
        self.keys, self.blocked = set(), set()
        self.speed, self.duration = 255, 500
        self.trim = (100, 100, False, True)
        self.plan = None
        self.step = 0
        self.test_id = 0
        self.waiting_test = False
        self.gap_until = None
        self.last_frame = self.last_heartbeat = -100.0
        self.last_rx = 0.0
        self.motion = (0, 0)

    def connect(self):
        self.connected = True
        self.ready = self.enabled = False
        self.keys.clear(); self.blocked.clear()
        self.plan = None; self.waiting_test = False; self.gap_until = None
        self.phase = 'Waiting for firmware'
        self.connect_at = self.clock()
        self.last_hello = -100.0
        self.last_rx = self.clock()

    def receive(self, line):
        if line != 'ALIVE': self.log('< ' + line)
        now = self.clock()
        if line == 'ROBOT_READY 2' and self.connected and self.phase == 'Waiting for firmware':
            self.last_rx = now
            self.phase = 'Applying settings'
            self.last_heartbeat = now
            self.send('CONFIG %d %d %d %d' % self.trim)
        elif line == 'CONFIG_OK' and self.phase == 'Applying settings':
            self.ready = True; self.phase = 'Connected — controls disabled'; self.last_rx = now
        elif line == 'ALIVE':
            self.last_rx = now
        elif line.startswith('CONTROL_TIMEOUT') or (line.startswith('ERROR') and self.connected):
            self.fail('Firmware stopped — reconnect and check log')
        elif line.startswith('TEST_DONE ') and self.plan is not None and self.waiting_test:
            try: request = int(line.split()[1])
            except (ValueError, IndexError): return
            if request == self.test_id:
                self.waiting_test = False
                self.gap_until = now + 1.0
                self.phase = 'Test step complete — stopped pause'
        elif line.startswith('Lab03Robot READY') and self.ready:
            self.fail('Board reset — reconnect')

    def enable(self):
        if self.ready:
            self.enabled = True; self.phase = 'Ready — hold arrows to drive'

    def stop(self):
        self.blocked.update(self.keys)
        self.keys.clear()
        self.plan = None; self.waiting_test = False; self.gap_until = None
        self.motion = (0, 0)
        if self.connected: self.send('STOP')
        if self.ready: self.phase = 'Stopped'

    def disable(self):
        self.stop(); self.enabled = False
        if self.ready: self.phase = 'Controls disabled — enable to resume'

    def fail(self, reason):
        self.stop(); self.ready = self.enabled = False; self.phase = reason

    def disconnect(self):
        self.disable(); self.connected = self.ready = False; self.phase = 'Disconnected'

    def configure(self, left, right, invert_left, invert_right):
        if not (1 <= left <= 200 and 1 <= right <= 200): raise ValueError('Trim must be 1–200%.')
        self.trim = (left, right, bool(invert_left), bool(invert_right))
        self.disable()
        if self.ready:
            self.ready = False; self.phase = 'Applying settings'; self.last_rx = self.clock()
            self.send('CONFIG %d %d %d %d' % self.trim)

    def press(self, key):
        if not self.enabled or key in self.blocked or key in self.keys: return
        if key in self.HOTKEYS:
            self.keys.add(key)
            self.start_test(self.HOTKEYS[key]); return
        if key not in self.ARROWS: return
        if self.plan is not None: self.stop()
        self.keys.add(key)
        self.last_frame = -100.0

    def release(self, key):
        self.blocked.discard(key)
        self.keys.discard(key)
        if key in self.ARROWS and self.plan is None:
            self.last_frame = -100.0
            if not (self.keys & self.ARROWS):
                self.motion = (0, 0)
                if self.ready: self.send('STOP')

    def start_test(self, name):
        if not self.enabled: return
        speed = self.speed
        groups = {
            'Wheels': [('Left wheel', speed, 0), ('Right wheel', 0, speed)],
            'Straight': [('Forward', speed, speed), ('Reverse', -speed, -speed)],
            'Turns': [('Left curve', speed//2, speed), ('Right curve', speed, speed//2)],
            'Spins': [('Left turn in place', -speed, speed), ('Right turn in place', speed, -speed)],
        }
        steps = sum(groups.values(), []) if name == 'Full lab' else groups[name]
        self.stop()
        self.plan = steps; self.step = 0; self.gap_until = self.clock()
        self.phase = 'Starting ' + name
        self.log('PRESET ' + name + ' — observer must record actual results')

    def tick(self):
        if not self.connected: return
        now = self.clock()
        if self.phase == 'Waiting for firmware':
            if now - self.connect_at > 10:
                self.fail('No protocol-2 firmware — upload the updated sketch'); return
            if now - self.connect_at >= 2 and now - self.last_hello >= .5:
                self.send('HELLO'); self.last_hello = now
            return
        if not self.ready and self.phase != 'Applying settings': return
        if now - self.last_rx > 1.0:
            self.fail('Connection lost — reconnect'); return
        if now - self.last_heartbeat >= .1:
            self.send('HEARTBEAT'); self.last_heartbeat = now
        if not self.ready or not self.enabled: return
        if self.plan is not None:
            if self.waiting_test and now > self.test_deadline:
                self.fail('Test did not acknowledge completion — stopped'); return
            if self.gap_until is not None and now >= self.gap_until:
                if self.step >= len(self.plan):
                    self.plan = None; self.gap_until = None
                    self.phase = 'Preset finished — record observed results'; return
                label, left, right = self.plan[self.step]
                self.step += 1; self.test_id += 1
                self.send(f'TEST {self.test_id} {left} {right} {self.duration}')
                self.test_deadline = now + self.duration/1000 + 2
                self.waiting_test = True; self.gap_until = None
                self.phase = 'Testing: ' + label
            return
        motion = wheel_command(self.keys, self.speed)
        if motion != (0, 0) and now - self.last_frame >= .1:
            self.send('DRIVE %d %d' % motion); self.last_frame = now
            self.motion = motion; self.phase = 'Driving — release arrows to stop'
        elif motion == (0, 0) and self.motion != (0, 0):
            self.send('STOP'); self.motion = (0, 0); self.phase = 'Stopped'
