from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    driver = Node(
        package='control_demo_driver',
        executable='control_demo_driver',
        name='control_demo_driver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'send_port': ParameterValue(
                LaunchConfiguration('send_port'), value_type=int),
            'recv_port': ParameterValue(
                LaunchConfiguration('recv_port'), value_type=int),
            'role': LaunchConfiguration('role'),
            'control_rate_hz': ParameterValue(
                LaunchConfiguration('control_rate_hz'), value_type=float),
            'joy_timeout_sec': ParameterValue(
                LaunchConfiguration('joy_timeout_sec'), value_type=float),
            'status_rate_hz': ParameterValue(
                LaunchConfiguration('status_rate_hz'), value_type=float),
            'auto_request_control': ParameterValue(
                LaunchConfiguration('auto_request_control'), value_type=bool),
            'max_speed_scale': ParameterValue(
                LaunchConfiguration('max_speed_scale'), value_type=float),
            'bridge_port': ParameterValue(
                LaunchConfiguration('bridge_port'), value_type=int),
        }],
    )

    web = Node(
        package='control_demo_web',
        executable='web_server',
        name='control_demo_web',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'bind_address': LaunchConfiguration('bind_address'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'status_timeout_sec': ParameterValue(
                LaunchConfiguration('status_timeout_sec'), value_type=float),
            'bridge_host': LaunchConfiguration('bridge_host'),
            'bridge_port': ParameterValue(
                LaunchConfiguration('bridge_port'), value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.234.1',
            description='机器狗 IP 地址'),
        DeclareLaunchArgument(
            'send_port', default_value='8081',
            description='SDK UDP 发送端口'),
        DeclareLaunchArgument(
            'recv_port', default_value='8080',
            description='SDK UDP 接收端口'),
        DeclareLaunchArgument(
            'role', default_value='sdk',
            description='控制角色：sdk 或 remote'),
        DeclareLaunchArgument(
            'control_rate_hz', default_value='30.0',
            description='SetRemote 发送频率'),
        DeclareLaunchArgument(
            'joy_timeout_sec', default_value='0.35',
            description='摇杆输入超时（秒）'),
        DeclareLaunchArgument(
            'status_rate_hz', default_value='2.0',
            description='状态发布频率'),
        DeclareLaunchArgument(
            'auto_request_control', default_value='true',
            description='role=sdk 时自动请求控制权'),
        DeclareLaunchArgument(
            'max_speed_scale', default_value='0.5',
            description='摇杆安全限速比例'),
        DeclareLaunchArgument(
            'port', default_value='8000',
            description='网页 HTTP 端口'),
        DeclareLaunchArgument(
            'bind_address', default_value='0.0.0.0',
            description='网页监听地址'),
        DeclareLaunchArgument(
            'status_timeout_sec', default_value='2.0',
            description='网页判定驱动状态过期的超时'),
        DeclareLaunchArgument(
            'bridge_host', default_value='127.0.0.1',
            description='本机 TCP 控制桥地址'),
        DeclareLaunchArgument(
            'bridge_port', default_value='8765',
            description='本机 TCP 控制桥端口'),
        driver,
        web,
    ])
