"""Run against the real sketch compiled for host I/O, not a duplicate firmware model."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from controller_core import Controller

class IntegrationTests(unittest.TestCase):
    def test_full_controller_to_firmware_protocol(self):
        lab = Path(__file__).resolve().parent.parent
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory)/'bridge')
            subprocess.run(['c++','-std=c++11','-I',str(lab/'tests'),str(lab/'tests/firmware_bridge.cpp'),'-o',binary],check=True)
            proc = subprocess.Popen([binary],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
            try:
                now = 0.0
                tx,rx=[],[]
                state=(0,0)
                def exchange(command):
                    nonlocal state
                    proc.stdin.write(command+'\n');proc.stdin.flush()
                    while True:
                        line=proc.stdout.readline().strip()
                        if line=='@end':return
                        if line.startswith('@state '):state=tuple(map(int,line.split()[1:]))
                        else:rx.append(line)
                c=Controller(tx.append,lambda:now)
                c.connect()
                def step(milliseconds=25):
                    nonlocal now
                    now+=milliseconds/1000
                    exchange(f'@tick {milliseconds}')
                    while rx:c.receive(rx.pop(0))
                    c.tick()
                    while tx:exchange(tx.pop(0))
                    while rx:c.receive(rx.pop(0))
                for _ in range(100):step()
                self.assertTrue(c.ready)
                c.enable();c.press('Up')
                for _ in range(60):step()
                self.assertEqual(state,(255,255))
                c.release('Up');step();self.assertEqual(state,(0,0))
                c.start_test('Full lab')
                for _ in range(650):step()
                self.assertIsNone(c.plan)
                self.assertEqual(c.step,8)
                self.assertEqual(state,(0,0))
                c.press('Up')
                for _ in range(20):step()
                self.assertEqual(state,(255,255))
                # Close and missing readings must not release held keys or stop motors.
                for distance in (10, -1, 50):
                    exchange(f'@distance {distance}')
                    for _ in range(20): step()
                    self.assertEqual(state, (255, 255))
                    self.assertIn('Up', c.keys)
                self.assertIn('logging only', c.sensor_status)
                c.release('Up'); step(); self.assertEqual(state, (0, 0))
                c.start_test('Full lab')
                for _ in range(12): step()
                exchange('@distance -1'); step()
                self.assertIsNotNone(c.plan)
                c.stop(); step()
                c.press('Up')
                for _ in range(20): step()
                self.assertEqual(state, (255, 255))
                exchange('@tick 501') # Simulate frozen application: no heartbeat or tick on host.
                self.assertEqual(state,(0,0))
                while rx:c.receive(rx.pop(0))
                self.assertFalse(c.ready)
            finally:
                proc.stdin.close();proc.wait(timeout=3);proc.stdout.close()

if __name__=='__main__':unittest.main()
