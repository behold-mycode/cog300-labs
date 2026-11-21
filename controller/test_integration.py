"""Run RobotLink against host-compiled production firmware via a serial adapter."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from robot_link import RobotLink


class FirmwareSerial:
    def __init__(self, binary):
        self.proc = subprocess.Popen([binary], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.pending = bytearray(); self.state = (0, 0)
    def exchange(self, command):
        self.proc.stdin.write(command + '\n'); self.proc.stdin.flush()
        while True:
            line = self.proc.stdout.readline().strip()
            if line == '@end': return
            if not line: raise RuntimeError('Firmware bridge exited unexpectedly')
            if line.startswith('@state '): self.state = tuple(map(int, line.split()[1:]))
            else: self.pending.extend((line + '\n').encode())
    def write(self, data): self.exchange(data.decode().strip()); return len(data)
    @property
    def in_waiting(self): return len(self.pending)
    def read(self, count):
        result = bytes(self.pending[:count]); del self.pending[:count]; return result
    def close(self): pass  # Keep test process alive for assertions after disconnect.
    def finish(self):
        self.proc.stdin.close(); self.proc.wait(timeout=3); self.proc.stdout.close()


class IntegrationTests(unittest.TestCase):
    def test_controller_to_real_firmware(self):
        lab = Path(__file__).resolve().parent.parent
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / 'bridge')
            subprocess.run(['c++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-I', str(lab/'tests'),
                            str(lab/'tests/firmware_bridge.cpp'), '-o', binary], check=True)
            wire = FirmwareSerial(binary)
            try:
                now = 0.; seq = 0
                link = RobotLink(clock=lambda: now, serial_factory=lambda port: wire)
                owner = link.connect('test')
                def steps(count=1, keys=None, mode='MANUAL'):
                    nonlocal now, seq
                    observed = []
                    for _ in range(count):
                        now += .05; seq += 1
                        link.frame(owner, seq, keys or [], mode)
                        wire.exchange('@tick 50'); link.tick()
                        observed.append(wire.state)
                    return observed
                steps(50); self.assertTrue(link.ready)
                steps(10, ['Up']); self.assertEqual(wire.state, (255,255))
                wire.exchange('@distance -1 -1')
                steps(5, ['Up']); self.assertEqual(wire.state, (255,255))
                steps(5); self.assertEqual(wire.state, (0,0))
                wire.exchange('@distance 35 20')
                steps(20, mode='FOLLOW'); self.assertEqual(wire.state,(255,255))
                wire.exchange('@distance 15 20')
                steps(20, mode='FOLLOW'); self.assertEqual(wire.state,(-255,-255))
                wire.exchange('@distance -1 20')
                steps(3, mode='FOLLOW'); self.assertEqual(wire.state,(0,0))
                wire.exchange('@distance 50 20')
                steps(20, mode='WALL'); self.assertEqual(wire.state,(255,0))
                steps(3); self.assertEqual(wire.state,(0,0))
                # Synthetic physical profile, then legacy protocol commissioning fixtures.
                wire.exchange("@profile")
                seq+=1;link.configure(owner,seq,dict(link.settings,lineTapeLevel=1));steps(3)
                def action(name, values):
                    nonlocal seq
                    seq+=1;link.action(owner,seq,name,values);steps(3)
                    self.assertTrue(link.ready, link.phase)
                action('sonar',dict(enabled=1,interval=70))
                action('encoders',dict(kind=1,invertL=0,invertR=0))
                action('odometry',dict(left=1,right=1,track=120))
                action('body',dict(front=90,rear=70,halfWidth=60,frontSensor=80,sideX=30,sideY=50))
                action('wall_config',dict(cruise=60,turnSpeed=40,margin=15,deceleration=200,reaction=.15,matchRadius=60,corridorWidth=400))
                action('speed_config',dict(fullLeft=200,fullRight=200,kp=.6,ki=.3,acceleration=150))
                action('race_config',dict(speed=50,offset=100,angle=1.8))
                action('save_calibration',{})
                seq+=1;link.configure(owner,seq,dict(link.settings,power=200,minimum=200));steps(3)
                action('load_calibration',{})
                self.assertEqual(link.settings['power'],255)
                self.assertEqual(link.telemetry['race']['speed'],50)
                wire.exchange('@ir 0 1 0')
                steps(6,mode='RACE');self.assertEqual(link.telemetry['mode'],'RACE')
                self.assertGreater(wire.state[0],0);self.assertGreater(wire.state[1],0)
                # Browser sleeps while the server continues servicing real firmware.
                for _ in range(2400):
                    now += .05
                    wire.exchange('@tick 50'); link.tick()
                self.assertTrue(link.snapshot()['connected'])
                self.assertFalse(link.snapshot()['ready'])
                self.assertEqual(wire.state, (0,0))
                steps(3)  # Neutral browser acknowledgement restores controls, not motion.
                self.assertTrue(link.snapshot()['ready'])
                self.assertEqual(wire.state, (0,0))
                wire.exchange('@tick 501')  # Entire host freezes, including heartbeat worker.
                self.assertEqual(wire.state,(0,0))
                link.tick(); self.assertFalse(link.ready)
                self.assertTrue(link.snapshot()['connected'])
                steps(3, mode='RACE')  # Delayed browser commands cannot resume motion.
                self.assertEqual(wire.state,(0,0))
                self.assertFalse(link.snapshot()['ready'])
                steps(3)
                self.assertTrue(link.snapshot()['ready'])
                self.assertEqual(wire.state,(0,0))
                self.assertTrue(link.snapshot()['radar']['capable'])
                action('radar_profile',dict(minMm=50,maxMm=100))
                action('radar_clearance',dict(radius=0))
                action('radar_servo',dict(rightUs=900,centerUs=1500,leftUs=2100))
                action('radar_step',dict(step=2))
                self.assertTrue(all(motors==(0,0) for motors in steps(60,mode='RADAR_SCAN')))
                radar = link.snapshot()['radar']
                self.assertTrue(radar['samples'])
                self.assertEqual(radar['settings']['step'],2)
                self.assertEqual(link.telemetry['mode'],'RADAR_SCAN')
                steps(5)
                self.assertEqual(link.telemetry['mode'],'MANUAL')
            finally: wire.finish()

if __name__ == '__main__': unittest.main()
