import threading
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from .http_server import ControlHTTPServer, ControlRequestHandler
from .ros_bridge import BridgeClient
from .urls import list_ipv4, route_ip_for


class WebConfigNode(Node):
    """ROS2 node used only for launch parameters and signal handling.

    The actual web <-> driver control path is the loopback TCP bridge, not
    ROS2 topics; this node intentionally has no publishers or subscriptions.
    """

    def __init__(self):
        super().__init__('control_demo_web')
        self.declare_parameter('port', 8000)
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('robot_ip', '192.168.234.1')
        self.declare_parameter('status_timeout_sec', 2.0)
        self.declare_parameter('bridge_host', '127.0.0.1')
        self.declare_parameter('bridge_port', 8765)


def resolve_web_root():
    try:
        share = Path(get_package_share_directory('control_demo_web')) / 'web'
        if share.is_dir():
            return share
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / 'web'


def main(args=None):
    rclpy.init(args=args)
    config = WebConfigNode()

    port = int(config.get_parameter('port').value)
    bind_address = str(config.get_parameter('bind_address').value)
    robot_ip = str(config.get_parameter('robot_ip').value)
    status_timeout = float(config.get_parameter('status_timeout_sec').value)
    bridge_host = str(config.get_parameter('bridge_host').value)
    bridge_port = int(config.get_parameter('bridge_port').value)
    web_root = resolve_web_root()

    bridge = BridgeClient(bridge_host, bridge_port, status_timeout)
    try:
        server = ControlHTTPServer(
            (bind_address, port), ControlRequestHandler, bridge, web_root)
    except OSError as error:
        config.get_logger().error(
            '无法监听 %s:%d: %s' % (bind_address, port, error))
        bridge.stop()
        config.destroy_node()
        rclpy.shutdown()
        return 1

    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()

    recommended = route_ip_for(robot_ip, 8081)
    config.get_logger().info(
        'Web 服务已启动: http://%s:%d/' % (bind_address, port))
    config.get_logger().info(
        '控制桥: %s:%d（本机 TCP，驱动未启动时会自动重连）'
        % (bridge_host, bridge_port))
    if recommended and not recommended.startswith('127.'):
        config.get_logger().info(
            '推荐手机访问: http://%s:%d/ （与机器狗 %s 同网段）'
            % (recommended, port, robot_ip))
    else:
        config.get_logger().info(
            '本机测试地址: http://127.0.0.1:%d/' % port)
    for address in list_ipv4():
        if address != recommended:
            config.get_logger().info(
                '其他本机地址: http://%s:%d/' % (address, port))
    config.get_logger().info(
        '手机需连接机器狗热点；WSL2 建议 mirrored 网络模式，详见 README。')

    exit_code = 0
    try:
        rclpy.spin(config)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001 - report and shut down cleanly
        config.get_logger().error('Web 节点异常: %s' % error)
        exit_code = 1
    finally:
        server.shutdown()
        server.server_close()
        bridge.stop()
        config.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
