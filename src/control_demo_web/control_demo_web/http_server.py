import json
import math
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

from .ros_bridge import COMMAND_CODES

STATIC_FILES = {
    '/': ('index.html', 'text/html; charset=utf-8'),
    '/index.html': ('index.html', 'text/html; charset=utf-8'),
    '/style.css': ('style.css', 'text/css; charset=utf-8'),
    '/app.js': ('app.js', 'application/javascript; charset=utf-8'),
    '/joystick.js': ('joystick.js', 'application/javascript; charset=utf-8'),
}

MAX_JOY_BODY = 1024
MAX_COMMAND_BODY = 2048


class TokenBucket:
    def __init__(self, rate=60.0, burst=120.0):
        self.rate = rate
        self.burst = burst
        self.tokens = burst
        self.updated = time.monotonic()

    def allow(self):
        now = time.monotonic()
        self.tokens = min(self.burst, self.tokens + (now - self.updated) * self.rate)
        self.updated = now
        if self.tokens >= 1.0:
            self.tokens -= 1.0
            return True
        return False


class ControlHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, server_address, handler_class, bridge, web_root):
        super().__init__(server_address, handler_class)
        self.bridge = bridge
        self.web_root = Path(web_root)
        self._buckets = {}
        self._bucket_lock = threading.Lock()

    def allow_joy_request(self, client_ip):
        with self._bucket_lock:
            bucket = self._buckets.get(client_ip)
            if bucket is None:
                bucket = TokenBucket()
                self._buckets[client_ip] = bucket
            return bucket.allow()


class ControlRequestHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'ControlDemo/0.1'

    def log_request(self, code='-', size='-'):
        # Suppress the 20 Hz joy POSTs; keep 4xx/5xx visible.
        if isinstance(code, int) and code < 400:
            return
        super().log_request(code, size)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == '/api/status':
            self._send_json(HTTPStatus.OK, self.server.bridge.get_status())
            return
        if path == '/api/health':
            status = self.server.bridge.get_status()
            if status['stale']:
                self._send_json(HTTPStatus.SERVICE_UNAVAILABLE, {
                    'ok': False,
                    'driver_status_age_ms': status['driver_status_age_ms'],
                    'reason': 'stale_status',
                })
            else:
                self._send_json(HTTPStatus.OK, {'ok': True})
            return
        if path == '/favicon.ico':
            self.send_response(HTTPStatus.NO_CONTENT)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            return

        entry = STATIC_FILES.get(path)
        if entry is None:
            self._send_json(HTTPStatus.NOT_FOUND, {'ok': False, 'error': 'not_found'})
            return

        filename, content_type = entry
        # filename comes from the fixed allowlist above, never from the URL,
        # so no traversal is possible. Do not resolve symlinks here: with
        # colcon --symlink-install the installed files point into the source
        # tree, and a resolved-path containment check would reject them.
        candidate = self.server.web_root / filename
        if not candidate.is_file():
            self._send_json(HTTPStatus.NOT_FOUND, {'ok': False, 'error': 'not_found'})
            return
        self._send_bytes(HTTPStatus.OK, candidate.read_bytes(), content_type)

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in ('/api/joy', '/api/command', '/api/estop', '/api/estop/clear'):
            self._reject(HTTPStatus.NOT_FOUND, 'not_found')
            return
        if not self._origin_allowed():
            self._reject(HTTPStatus.FORBIDDEN, 'bad_origin')
            return

        limit = MAX_JOY_BODY if path == '/api/joy' else MAX_COMMAND_BODY
        body = self._read_body(limit)
        if body is None:
            return

        if path == '/api/estop':
            if not self.server.bridge.enqueue_command(
                    COMMAND_CODES['emergency_stop']):
                self._send_json(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {'ok': False, 'error': 'bridge_disconnected'})
                return
            self._send_json(HTTPStatus.OK, {'ok': True, 'latched': True})
            return
        if path == '/api/estop/clear':
            if not self.server.bridge.request_estop_clear():
                self._send_json(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {'ok': False, 'error': 'bridge_disconnected'})
                return
            self._send_json(HTTPStatus.ACCEPTED, {'ok': True, 'pending': True})
            return

        content_type = self.headers.get('Content-Type', '')
        if 'application/json' not in content_type.lower():
            self._send_json(
                HTTPStatus.UNSUPPORTED_MEDIA_TYPE,
                {'ok': False, 'error': 'content_type'})
            return
        try:
            payload = json.loads(body.decode('utf-8')) if body else {}
        except (UnicodeDecodeError, ValueError):
            self._send_json(HTTPStatus.BAD_REQUEST, {'ok': False, 'error': 'invalid_json'})
            return
        if not isinstance(payload, dict):
            self._send_json(HTTPStatus.BAD_REQUEST, {'ok': False, 'error': 'invalid_payload'})
            return

        if path == '/api/joy':
            self._handle_joy(payload)
        else:
            self._handle_command(payload)

    def _handle_joy(self, payload):
        if not self.server.allow_joy_request(self.client_address[0]):
            self._send_json(
                HTTPStatus.TOO_MANY_REQUESTS,
                {'ok': False, 'error': 'rate_limited'})
            return

        axes = []
        for name in ('forward', 'yaw', 'strafe', 'head'):
            value = payload.get(name, 0.0)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                self._send_json(
                    HTTPStatus.BAD_REQUEST,
                    {'ok': False, 'error': 'invalid_' + name})
                return
            value = float(value)
            if not math.isfinite(value):
                self._send_json(
                    HTTPStatus.BAD_REQUEST,
                    {'ok': False, 'error': 'invalid_' + name})
                return
            axes.append(max(-1.0, min(1.0, value)))

        seq = payload.get('seq')
        if seq is not None and (
                isinstance(seq, bool) or not isinstance(seq, int) or seq < 0):
            self._send_json(
                HTTPStatus.BAD_REQUEST, {'ok': False, 'error': 'invalid_seq'})
            return

        if not self.server.bridge.set_joy(tuple(axes), seq):
            self._send_json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                {'ok': False, 'error': 'bridge_disconnected'})
            return
        self._send_json(HTTPStatus.OK, {
            'ok': True,
            'seq': seq,
            'accepted': True,
        })

    def _handle_command(self, payload):
        name = payload.get('command')
        if not isinstance(name, str) or name not in COMMAND_CODES:
            self._send_json(
                HTTPStatus.BAD_REQUEST, {'ok': False, 'error': 'unknown_command'})
            return
        if not self.server.bridge.enqueue_command(COMMAND_CODES[name]):
            self._send_json(
                HTTPStatus.SERVICE_UNAVAILABLE,
                {'ok': False, 'error': 'bridge_disconnected'})
            return
        self._send_json(HTTPStatus.OK, {
            'ok': True, 'command': name, 'code': COMMAND_CODES[name]})

    def _read_body(self, limit):
        length_header = self.headers.get('Content-Length')
        if length_header is None:
            self._reject(HTTPStatus.LENGTH_REQUIRED, 'length_required')
            return None
        try:
            length = int(length_header)
        except ValueError:
            self._reject(HTTPStatus.BAD_REQUEST, 'invalid_length')
            return None
        if length < 0:
            self._reject(HTTPStatus.BAD_REQUEST, 'invalid_length')
            return None
        if length > limit:
            # The unread body must not be parsed as the next keep-alive request.
            self._reject(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, 'body_too_large')
            return None
        return self.rfile.read(length) if length else b''

    def _origin_allowed(self):
        origin = self.headers.get('Origin')
        if not origin:
            return True
        host = self.headers.get('Host')
        if not host:
            return False
        return origin in ('http://' + host, 'https://' + host)

    def _reject(self, status, error):
        self.close_connection = True
        self._send_json(status, {'ok': False, 'error': error}, close=True)

    def _send_json(self, status, payload, close=False):
        body = json.dumps(
            payload, separators=(',', ':'), ensure_ascii=False).encode('utf-8')
        self._send_bytes(status, body, 'application/json; charset=utf-8', close=close)

    def _send_bytes(self, status, body, content_type, close=False):
        if close:
            self.close_connection = True
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Connection', 'close' if close else 'keep-alive')
        self.end_headers()
        self.wfile.write(body)
