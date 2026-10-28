import json
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from controller import make_server
from robot_link import RobotLink, DemoSerial


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.link = RobotLink(serial_factory=DemoSerial)
        self.server, self.lock, url = make_server(self.link, demo=True)
        self.base, self.token = url.split('/#token=')
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval':.01})
        self.thread.start()
    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join()
        self.link.disconnect()
    def request(self, path, body=None, **headers):
        all_headers = {'X-Robot-Token':self.token, 'Content-Type':'application/json'}
        all_headers.update(headers)
        data = None if body is None else json.dumps(body).encode()
        with urlopen(Request(self.base + path, data=data, headers=all_headers), timeout=2) as response:
            return json.load(response)
    def test_api_auth_and_single_owner(self):
        with self.assertRaises(HTTPError) as error:
            self.request('/api/connect', {'port':'DEMO'}, **{'X-Robot-Token':'wrong'})
        self.assertEqual(error.exception.code,403); self.assertIsNone(self.link.serial)
        owner = self.request('/api/connect', {'port':'DEMO'})['owner']
        with self.assertRaises(HTTPError): self.request('/api/connect', {'port':'DEMO'})
        self.request('/api/frame', dict(owner=owner,seq=1,keys=[],mode='MANUAL'))
        with self.assertRaises(HTTPError):
            self.request('/api/frame', dict(owner=owner,seq=0,keys=['Up'],mode='MANUAL'))
        self.request('/api/disconnect', dict(owner=owner,seq=2))
        self.assertIsNone(self.link.serial)
    def test_cross_origin_and_rebinding_rejected(self):
        for headers in [{'Origin':'https://example.com'}, {'Host':'evil.example'}]:
            with self.assertRaises(HTTPError) as error: self.request('/api/state', **headers)
            self.assertEqual(error.exception.code,403)
    def test_malformed_body_does_not_open_port(self):
        with self.assertRaises(HTTPError): self.request('/api/connect', [])
        self.assertIsNone(self.link.serial)
        self.assertEqual(self.request('/api/ports')[0]['device'], 'DEMO')

if __name__ == '__main__': unittest.main()
