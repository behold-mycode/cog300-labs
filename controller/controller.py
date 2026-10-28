#!/usr/bin/env python3
"""Local browser UI. No Tk or global keyboard permission required."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlsplit
import webbrowser

from robot_link import RobotLink, DemoSerial

ROOT = Path(__file__).resolve().parent


def make_server(link, demo=False, port=0):
    lock = threading.RLock()
    token = secrets.token_urlsafe(32)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass

        def respond(self, status, data, content_type='application/json'):
            if not isinstance(data, bytes): data = json.dumps(data).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            try: self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError): pass

        def allowed(self):
            # Exact Host/Origin checks also prevent DNS rebinding and cross-site control.
            host = f'127.0.0.1:{self.server.server_port}'
            return (self.headers.get('Host') == host and
                    self.headers.get('Origin', 'http://' + host) == 'http://' + host)

        def authenticated(self):
            return self.allowed() and secrets.compare_digest(self.headers.get('X-Robot-Token', ''), token)

        def do_GET(self):
            path = urlsplit(self.path).path
            if not self.allowed(): self.respond(403, {'error': 'Invalid origin'}); return
            assets = {'/': ('index.html', 'text/html; charset=utf-8'),
                      '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                      '/style.css': ('style.css', 'text/css; charset=utf-8')}
            if path in assets:
                name, mime = assets[path]
                self.respond(200, (ROOT / name).read_bytes(), mime); return
            if not self.authenticated(): self.respond(403, {'error': 'Open the launcher URL.'}); return
            if path == '/api/state':
                with lock: state = link.snapshot()
                state['demo'] = demo
                self.respond(200, state)
            elif path == '/api/ports':
                if demo: ports = [{'device': 'DEMO', 'description': 'Demo — no robot'}]
                else:
                    from serial.tools import list_ports
                    ports = [dict(device=p.device, description=p.description) for p in list_ports.comports()]
                    ports.sort(key=lambda p: ('usb' not in p['device'].lower(), p['device']))
                self.respond(200, ports)
            else: self.respond(404, {'error': 'Not found'})

        def do_POST(self):
            if not self.authenticated(): self.respond(403, {'error': 'Invalid controller token or origin'}); return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 2048: raise ValueError('Invalid request size')
                if self.headers.get('Content-Type') != 'application/json': raise ValueError('JSON required')
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict): raise ValueError('JSON object required')
                with lock:
                    path = urlsplit(self.path).path
                    if path == '/api/connect':
                        device = body['port']
                        if not demo:
                            from serial.tools import list_ports
                            if device not in [p.device for p in list_ports.comports()]:
                                raise ValueError('Port is unavailable. Refresh the port list.')
                        result = {'owner': link.connect(device)}
                    elif path == '/api/frame':
                        link.frame(body['owner'], body['seq'], body['keys'], body['mode'])
                        result = link.snapshot()
                    elif path == '/api/settings':
                        link.configure(body['owner'], body['seq'], body['settings'])
                        result = link.snapshot()
                    elif path == '/api/disconnect':
                        link._authorize(body['owner'], body['seq'])
                        link.disconnect(); result = link.snapshot()
                    else:
                        self.respond(404, {'error': 'Not found'}); return
                self.respond(200, result)
            except (ValueError, KeyError, TypeError) as error:
                self.respond(400, {'error': str(error)})
            except OSError as error:
                with lock: link.disconnect('Serial error: ' + str(error))
                self.respond(503, {'error': str(error)})

    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    return server, lock, f'http://127.0.0.1:{server.server_port}/#token={token}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Simulate firmware; never open hardware')
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--port', type=int, default=0, help='HTTP port; default selects an available port')
    args = parser.parse_args()
    if not args.demo:
        try: import serial  # noqa: F401
        except ImportError:
            parser.exit(1, 'Missing pyserial. Run the launcher or: python3 -m pip install -r requirements.txt\n')
    link = RobotLink(serial_factory=DemoSerial) if args.demo else RobotLink()
    server, lock, url = make_server(link, args.demo, args.port)
    done = threading.Event()

    def worker():
        while not done.wait(.02):
            with lock: link.tick()

    threading.Thread(target=worker, daemon=True).start()
    print(('DEMO — no robot\n' if args.demo else '') + 'Open ' + url, flush=True)
    print('Keep this terminal open. Press Ctrl+C to stop the server.', flush=True)
    if not args.no_browser: webbrowser.open(url)
    try: server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt: pass
    finally:
        done.set()
        with lock: link.disconnect('Controller closed')
        server.server_close()


if __name__ == '__main__': main()
