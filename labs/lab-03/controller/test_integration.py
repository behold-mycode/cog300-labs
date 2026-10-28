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
            subprocess.run(['c++', '-std=c++11', '-Wall', '-Wextra', '-Werror', '-I', str(lab/'tests'),
                            str(lab/'tests/firmware_bridge.cpp'), '-o', binary], check=True)
            wire = FirmwareSerial(binary)
            try:
                now = 0.; seq = 0
                link = RobotLink(clock=lambda: now, serial_factory=lambda port: wire)
                owner = link.connect('test')
                def steps(count=1, keys=None, mode='MANUAL'):
                    nonlocal now, seq
                    for _ in range(count):
                        now += .05; seq += 1
                        link.frame(owner, seq, keys or [], mode)
                        wire.exchange('@tick 50'); link.tick()
                steps(50); self.assertTrue(link.ready)
                steps(10, ['Up']); self.assertEqual(wire.state, (255,255))
                wire.exchange('@distance -1 -1')
                steps(5, ['Up']); self.assertEqual(wire.state, (255,255))
                steps(5); self.assertEqual(wire.state, (0,0))
                wire.exchange('@distance 35 20')
                steps(10, mode='FOLLOW'); self.assertEqual(wire.state,(255,255))
                wire.exchange('@distance 15 20')
                steps(10, mode='FOLLOW'); self.assertEqual(wire.state,(-255,-255))
                wire.exchange('@distance -1 20')
                steps(3, mode='FOLLOW'); self.assertEqual(wire.state,(0,0))
                wire.exchange('@distance 50 30')
                steps(10, mode='WALL'); self.assertGreater(wire.state[0],wire.state[1])
                steps(3); self.assertEqual(wire.state,(0,0))
                steps(10, ['Up']); self.assertEqual(wire.state,(255,255))
                wire.exchange('@tick 501')  # Entire host freezes, including heartbeat worker.
                self.assertEqual(wire.state,(0,0))
                link.tick(); self.assertFalse(link.ready)
                wire.exchange('DRIVE 255 255'); self.assertEqual(wire.state,(0,0))
            finally: wire.finish()

if __name__ == '__main__': unittest.main()
