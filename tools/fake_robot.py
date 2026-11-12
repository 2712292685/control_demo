#!/usr/bin/env python3
"""Minimal fake Zsibot robot for control_demo testing.

This is a test-only UDP JSON simulator. It never talks to a real robot and
refuses non-loopback addresses unless --allow-non-loopback is given.

The SDK sends heartbeats/remote/cmd to robot_ip:send_port (default 8081)
and receives on its fixed recv_port (default 8080). Replies MUST go to the
fixed recv port; replying to the heartbeat source port does not reach the
SDK's receive socket (verified empirically).
"""
import argparse
import json
import signal
import socket
import sys
import threading
import time

MODEL_NAMES = {'XG', 'XGW', 'XGWHSPD'}
SPEED_LEVELS = {'low', 'mid', 'high'}


class FakeRobot:
    def __init__(self, args):
        self.args = args
        self.listen = args.listen
        self.reply = args.reply
        self.model = args.model
        self.initial_function = args.initial_function
        self.function = self.initial_function
        self.control_mode = 'stay'
        self.motion_mode = 'rest'
        self.motion_type = 'unknown'
        self.speed_level = 'mid'
        self.grant_at = None
        self.control_grant_delay = args.control_grant_delay_ms / 1000.0
        self.stop_event = threading.Event()
        self.start = time.monotonic()

        self.counters = {'heartbeat': 0, 'remote': 0, 'cmd': 0}
        self.cmd_counts = {}
        self.last_joystick = [0.0, 0.0, 0.0, 0.0]
        self.last_rx_monotonic = None
        self.last_summary_time = time.monotonic()
        self.last_remote_count = 0
        self.lock = threading.Lock()
        self.log_file = None
        if args.log_file:
            self.log_file = open(args.log_file, 'w', encoding='utf-8')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(self.listen)
        self.sock.settimeout(0.2)

    # ---------------------------- packets ----------------------------
    def dog_state(self):
        return {
            'type': 'dog_state',
            'result': 0,
            'power': 86,
            'temp': 36.0,
            'sn': 'SIM-XG-0001',
            'speed': 0.0,
            'angle_speed': 0.0,
            'shift_speed': 0.0,
            'angle': 0.0,
            'dev_name': 'FakeDog',
            'ssid': 'Zsibot-SIM',
            'model': self.model,
        }

    def dev_info(self):
        return {
            'type': 'dev_info',
            'result': 0,
            'battery': {
                'current': 1230,
                'error': 0,
                'power': 86,
                'temp': 30500,
                'volt': 24100,
            },
        }

    def feedback(self):
        return {
            'type': 'feedback',
            'result': 0,
            'function': self.function,
            'control_mode': self.control_mode,
            'motion_mode': self.motion_mode,
            'motion_type': self.motion_type,
            'motor_temp': [30.0 + index * 0.5 for index in range(16)],
        }

    def imu_info(self):
        return {
            'type': 'imu_info',
            'result': 0,
            'acc': [0.0, 0.0, 9.81],
            'gyro': [0.0, 0.0, 0.0],
            'quat': [1.0, 0.0, 0.0, 0.0],
            'rpy': [0.0, 0.0, 0.0],
        }

    def odom_info(self):
        return {
            'type': 'odom_info',
            'result': 0,
            'v_world': [0.0, 0.0, 0.0],
            'position': [0.0, 0.0, 0.0],
            'omega_world': [0.0, 0.0, 0.0],
            'v_body': [0.0, 0.0, 0.0],
            'omega_body': [0.0, 0.0, 0.0],
        }

    def leg_joint_info(self):
        return {
            'type': 'leg_joint_info',
            'result': 0,
            'leg_joint_info': {
                'leg_abad_joint': [0.0, 0.0, 0.0, 0.0],
                'leg_hip_joint': [0.0, 0.0, 0.0, 0.0],
                'leg_knee_joint': [0.0, 0.0, 0.0, 0.0],
                'leg_foot_joint': [0.0, 0.0, 0.0, 0.0],
                'leg_abad_joint_vel': [0.0, 0.0, 0.0, 0.0],
                'leg_hip_joint_vel': [0.0, 0.0, 0.0, 0.0],
                'leg_knee_joint_vel': [0.0, 0.0, 0.0, 0.0],
                'leg_foot_joint_vel': [0.0, 0.0, 0.0, 0.0],
                'leg_abad_joint_torque': [0.0, 0.0, 0.0, 0.0],
                'leg_hip_joint_torque': [0.0, 0.0, 0.0, 0.0],
                'leg_knee_joint_torque': [0.0, 0.0, 0.0, 0.0],
                'leg_foot_joint_torque': [0.0, 0.0, 0.0, 0.0],
            },
        }

    def version(self):
        return {
            'type': 'version',
            'result': 0,
            'version': {'mc': 'SIM-MC-1.0', 'dog_task': 'SIM-TASK-1.0'},
        }

    def fault_info(self):
        return {'type': 'fault_info', 'result': 0, 'faults': []}

    def speed_set(self):
        return {
            'type': 'speed_set',
            'result': 0,
            'level': self.speed_level,
            'high': {'x': 1.0, 'y': 0.5, 'yaw': 1.0},
            'mid': {'x': 0.6, 'y': 0.3, 'yaw': 0.6},
            'low': {'x': 0.3, 'y': 0.2, 'yaw': 0.3},
        }

    # ---------------------------- threads ----------------------------
    def sender_loop(self):
        tick = 0
        fast_packets = [
            self.dog_state,
            self.dev_info,
            self.feedback,
            self.imu_info,
            self.odom_info,
            self.leg_joint_info,
        ]
        slow_packets = [self.version, self.fault_info, self.speed_set]
        while not self.stop_event.is_set():
            now = time.monotonic()
            with self.lock:
                if self.grant_at is not None and now >= self.grant_at:
                    self.function = 'sdk'
                    self.grant_at = None
                    print('[fake] control granted: function=sdk', flush=True)
                control_mode = self.control_mode

            packets = [builder() for builder in fast_packets]
            if tick % 10 == 0:
                packets.extend(builder() for builder in slow_packets)
            for packet in packets:
                self.sock.sendto(
                    json.dumps(packet, separators=(',', ':')).encode('utf-8'),
                    self.reply)
            tick += 1
            self.stop_event.wait(0.1)

    def receive_loop(self):
        last_summary = time.monotonic()
        while not self.stop_event.is_set():
            try:
                data, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                data = None
            except OSError:
                break

            if data:
                try:
                    packet = json.loads(data.decode('utf-8'))
                except (UnicodeDecodeError, ValueError):
                    packet = {'type': 'invalid', 'raw': data[:80].decode('utf-8', 'replace')}
                self._handle_packet(packet)
                if self.log_file:
                    self.log_file.write(json.dumps({
                        'ts': time.time(),
                        'packet': packet,
                    }, ensure_ascii=False) + '\n')
                    self.log_file.flush()

            now = time.monotonic()
            if not self.args.quiet and now - last_summary >= 1.0:
                last_summary = now
                self._print_summary()

    def _handle_packet(self, packet):
        packet_type = packet.get('type')
        now = time.monotonic()
        with self.lock:
            self.last_rx_monotonic = now
            if packet_type == 'heartbeat':
                self.counters['heartbeat'] += 1
            elif packet_type == 'remote':
                self.counters['remote'] += 1
                joystick = packet.get('joystick', [])
                if isinstance(joystick, list) and len(joystick) >= 4:
                    self.last_joystick = [float(value) for value in joystick[:4]]
            elif packet_type == 'cmd':
                code = int(packet.get('cmd', 0))
                self.counters['cmd'] += 1
                self.cmd_counts[code] = self.cmd_counts.get(code, 0) + 1
                self._apply_command(code, now)

    def _apply_command(self, code, now):
        if code == 179:  # CMD_SDK_CONTROL_RIGHT
            if self.function != 'sdk' and self.grant_at is None:
                self.grant_at = now + self.control_grant_delay
        elif code == 90:  # emergency stop
            self.control_mode = 'emergency'
            print('[fake] cmd 90: EMERGENCY STOP', flush=True)
        elif code == 122:  # stand up
            self.control_mode = 'stand_up'
            self._schedule_control('stay', 0.2)
            print('[fake] cmd 122: stand up', flush=True)
        elif code == 106:  # sit down
            self.control_mode = 'stand_down'
            print('[fake] cmd 106: sit down', flush=True)
        elif code == 138:  # move mode
            self.control_mode = 'stay'
            print('[fake] cmd 138: move mode', flush=True)
        elif code == 154:  # balance stand
            self.control_mode = 'stay'
            print('[fake] cmd 154: balance stand', flush=True)
        elif code == 155:  # lock
            self.control_mode = 'lock'
            print('[fake] cmd 155: lock', flush=True)
        elif code == 174:
            self.speed_level = 'low'
            print('[fake] cmd 174: slow', flush=True)
        elif code == 175:
            self.speed_level = 'mid'
            print('[fake] cmd 175: normal', flush=True)
        elif code == 176:
            self.speed_level = 'high'
            print('[fake] cmd 176: fast', flush=True)
        else:
            print('[fake] cmd %d (unhandled)' % code, flush=True)

    def _schedule_control(self, mode, delay):
        def apply():
            time.sleep(delay)
            if not self.stop_event.is_set():
                with self.lock:
                    self.control_mode = mode
        threading.Thread(target=apply, daemon=True).start()

    def _print_summary(self):
        now = time.monotonic()
        with self.lock:
            harness = dict(self.counters)
            joystick = list(self.last_joystick)
            function = self.function
            control = self.control_mode
            speed = self.speed_level
            commands = dict(self.cmd_counts)
            window = max(0.001, now - self.last_summary_time)
            rate = (harness['remote'] - self.last_remote_count) / window
            self.last_summary_time = now
            self.last_remote_count = harness['remote']
        print(
            '[fake] hb=%d remote=%d (%.1f/s) cmd=%d joy=(%+.2f,%+.2f,%+.2f,%+.2f) '
            'function=%s control=%s speed=%s commands=%s'
            % (harness['heartbeat'], harness['remote'], rate, harness['cmd'],
               joystick[0], joystick[1], joystick[2], joystick[3],
               function, control, speed, commands),
            flush=True)

    # ---------------------------- lifecycle ----------------------------
    def run(self):
        print(
            '[fake] listening on %s:%d, replying to %s:%d, model=%s, initial function=%s'
            % (self.listen[0], self.listen[1], self.reply[0], self.reply[1],
               self.model, self.initial_function),
            flush=True)
        sender = threading.Thread(target=self.sender_loop, daemon=True)
        receiver = threading.Thread(target=self.receive_loop, daemon=True)
        sender.start()
        receiver.start()

        deadline = None
        if self.args.duration_sec > 0:
            deadline = time.monotonic() + self.args.duration_sec
        try:
            while not self.stop_event.is_set():
                if deadline is not None and time.monotonic() >= deadline:
                    break
                self.stop_event.wait(0.2)
        except KeyboardInterrupt:
            pass
        self.stop_event.set()
        sender.join(timeout=1.0)
        receiver.join(timeout=1.0)
        self._print_summary()
        if self.log_file:
            self.log_file.close()
        self.sock.close()
        print('[fake] stopped', flush=True)


def parse_address(text):
    host, _, port = text.rpartition(':')
    if not host or not port:
        raise argparse.ArgumentTypeError('address must be HOST:PORT')
    return host, int(port)


def main():
    parser = argparse.ArgumentParser(description='Fake Zsibot robot for control_demo')
    parser.add_argument('--listen', type=parse_address, default=('127.0.0.1', 8081))
    parser.add_argument('--reply', type=parse_address, default=('127.0.0.1', 8080))
    parser.add_argument('--model', choices=sorted(MODEL_NAMES), default='XG')
    parser.add_argument('--initial-function', choices=['remote', 'sdk'], default='remote')
    parser.add_argument('--control-grant-delay-ms', type=int, default=200)
    parser.add_argument('--duration-sec', type=float, default=0.0)
    parser.add_argument('--log-file', default=None)
    parser.add_argument('--quiet', action='store_true')
    parser.add_argument('--allow-non-loopback', action='store_true')
    args = parser.parse_args()

    if not args.allow_non_loopback:
        for address in (args.listen, args.reply):
            if address[0] not in ('127.0.0.1', 'localhost'):
                parser.error('non-loopback address requires --allow-non-loopback')

    robot = FakeRobot(args)

    def stop(_signum, _frame):
        robot.stop_event.set()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    robot.run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
