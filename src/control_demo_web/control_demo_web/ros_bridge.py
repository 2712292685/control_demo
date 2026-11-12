import json
import socket
import threading
import time

COMMAND_CODES = {
    'emergency_stop': 90,
    'stand_up': 122,
    'sit_down': 106,
    'move_mode': 138,
    'balance_stand': 154,
    'lock': 155,
    'slow': 174,
    'normal': 175,
    'fast': 176,
}


class BridgeClient:
    """Loopback TCP client for control_demo_driver.

    Protocol (newline-delimited, web -> driver):
      J:forward,yaw,strafe,head,seq
      C:<command code>
      EC
    Driver -> web:
      S:<status json>

    Using TCP on 127.0.0.1 instead of ROS2 topics for the web <-> driver link
    avoids Fast DDS discovery problems between two local launch-managed nodes
    in WSL2. The driver is still a normal ROS2 node.
    """

    def __init__(self, host='127.0.0.1', port=8765, status_timeout_sec=2.0):
        self._host = host
        self._port = int(port)
        self._status_timeout = float(status_timeout_sec)
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._sock = None
        self._status = None
        self._status_rx = None
        self._status_rx_count = 0
        self._connected = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            sock = None
            try:
                sock = socket.create_connection((self._host, self._port), timeout=1.0)
                sock.settimeout(0.5)
                with self._lock:
                    self._sock = sock
                    self._connected = True
                buffer = b''
                while not self._stop.is_set():
                    try:
                        data = sock.recv(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not data:
                        break
                    buffer += data
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        self._handle_line(line)
            except OSError:
                pass
            finally:
                with self._lock:
                    if self._sock is sock:
                        self._sock = None
                        self._connected = False
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            self._stop.wait(1.0)

    def _handle_line(self, line):
        text = line.decode('utf-8', 'replace').strip()
        if not text.startswith('S:'):
            return
        try:
            status = json.loads(text[2:])
        except ValueError:
            return
        with self._lock:
            self._status = status
            self._status_rx = time.monotonic()
            self._status_rx_count += 1

    def _send(self, line):
        with self._send_lock:
            with self._lock:
                sock = self._sock
            if sock is None:
                return False
            try:
                sock.sendall((line + '\n').encode('utf-8'))
                return True
            except OSError:
                with self._lock:
                    if self._sock is sock:
                        self._sock = None
                        self._connected = False
                return False

    def set_joy(self, axes, seq):
        values = ','.join('%.4f' % float(value) for value in axes)
        sequence = int(seq) if seq is not None else -1
        return self._send('J:%s,%d' % (values, sequence))

    def enqueue_command(self, code):
        return self._send('C:%d' % int(code))

    def request_estop_clear(self):
        return self._send('EC')

    def get_status(self):
        with self._lock:
            status = self._status
            rx = self._status_rx
            count = self._status_rx_count
            connected = self._connected
        now = time.monotonic()
        age = None if rx is None else int((now - rx) * 1000)
        stale = age is None or age > int(self._status_timeout * 1000)
        return {
            'ok': True,
            'server_time_ms': int(time.time() * 1000),
            'web_version': '0.1.0',
            'stale': stale,
            'driver_status_age_ms': age,
            'driver': status,
            'bridge_connected': connected,
            'web_status_rx_count': count,
        }

    def stop(self):
        self._stop.set()
        with self._lock:
            sock = self._sock
            self._sock = None
            self._connected = False
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        self._thread.join(timeout=1.0)
