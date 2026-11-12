#include "control_demo_driver/control_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "control_demo_driver/axis_mapping.hpp"

namespace control_demo_driver
{
namespace
{

constexpr int64_t kMinPort = 1;
constexpr int64_t kMaxPort = 65535;
constexpr double kMinControlRateHz = 5.0;
constexpr double kMaxControlRateHz = 100.0;
constexpr double kMinJoyTimeoutSec = 0.05;
constexpr double kMaxJoyTimeoutSec = 5.0;
constexpr double kMinStatusRateHz = 0.5;
constexpr double kMaxStatusRateHz = 20.0;
constexpr double kMinControlRequestPeriodSec = 0.2;
constexpr double kMaxControlRequestPeriodSec = 10.0;
constexpr double kMinSpeedScale = 0.1;
constexpr double kMaxSpeedScale = 1.0;
constexpr int64_t kMaxCommandDebounceMs = 5000;
constexpr double kMinWarnPeriodSec = 0.5;
constexpr double kMaxWarnPeriodSec = 60.0;

constexpr int kEstopResendCount = 3;
constexpr auto kEstopResendPeriod = std::chrono::milliseconds(200);
constexpr auto kRecoveryConfirmDelay = std::chrono::milliseconds(300);
constexpr auto kCommandHoldDuration = std::chrono::milliseconds(200);
constexpr auto kRecoveryTimeout = std::chrono::seconds(5);
constexpr int kShutdownZeroCount = 5;
constexpr auto kShutdownZeroPeriod = std::chrono::milliseconds(10);

constexpr uint8_t kCmdEmergencyStop = 90;
constexpr uint8_t kCmdStandUp = 122;
constexpr uint8_t kCmdSitDown = 106;
constexpr uint8_t kCmdMoveMode = 138;
constexpr uint8_t kCmdBalanceStand = 154;
constexpr uint8_t kCmdLock = 155;
constexpr uint8_t kCmdSlow = 174;
constexpr uint8_t kCmdNormal = 175;
constexpr uint8_t kCmdFast = 176;
constexpr uint8_t kCmdSdkControlRight = 179;

rcl_interfaces::msg::ParameterDescriptor make_descriptor(const std::string & description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

std::string to_lower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

bool is_posture_command(int64_t code)
{
    return code == kCmdStandUp || code == kCmdSitDown || code == kCmdMoveMode ||
           code == kCmdBalanceStand || code == kCmdLock;
}

template <typename T>
bool in_range(T value, T low, T high)
{
    return value >= low && value <= high;
}

}  // namespace

ControlDemoNode::ControlDemoNode()
: rclcpp::Node("control_demo_driver")
{
    robot_ip_ = declare_parameter<std::string>(
        "robot_ip", "192.168.234.1",
        make_descriptor("机器狗 IP 地址（默认 192.168.234.1）"));
    send_port_ = declare_parameter<int64_t>(
        "send_port", 8081, make_descriptor("UDP 发送端口（默认 8081）"));
    recv_port_ = declare_parameter<int64_t>(
        "recv_port", 8080, make_descriptor("UDP 接收端口（默认 8080）"));
    role_ = declare_parameter<std::string>(
        "role", "sdk", make_descriptor("控制角色：sdk 或 remote（默认 sdk）"));
    control_rate_hz_ = declare_parameter<double>(
        "control_rate_hz", 30.0,
        make_descriptor("SetRemote 发送频率（Hz），范围 5~100"));
    joy_timeout_sec_ = declare_parameter<double>(
        "joy_timeout_sec", 0.35,
        make_descriptor("摇杆输入超时（秒），超时后输出归零"));
    status_rate_hz_ = declare_parameter<double>(
        "status_rate_hz", 2.0, make_descriptor("状态发布频率（Hz），范围 0.5~20"));
    auto_request_control_ = declare_parameter<bool>(
        "auto_request_control", true,
        make_descriptor("role=sdk 时自动请求并保持控制权"));
    control_request_period_sec_ = declare_parameter<double>(
        "control_request_period_sec", 1.0,
        make_descriptor("控制权请求重发周期（秒），范围 0.2~10"));
    max_speed_scale_ = declare_parameter<double>(
        "max_speed_scale", 0.5,
        make_descriptor("摇杆输出的最大比例（安全限速），范围 0.1~1.0"));
    command_debounce_ms_ = declare_parameter<int64_t>(
        "command_debounce_ms", 300,
        make_descriptor("同一命令去抖时间（毫秒），范围 0~5000"));
    estop_recovery_cmd_ = declare_parameter<int64_t>(
        "estop_recovery_cmd", 122,
        make_descriptor("解除急停时发送的姿态命令码（默认 122 站立）"));
    bridge_port_ = declare_parameter<int64_t>(
        "bridge_port", 8765,
        make_descriptor("本机 TCP 控制桥端口（仅监听 127.0.0.1）"));
    warn_period_sec_ = declare_parameter<double>(
        "warn_period_sec", 5.0,
        make_descriptor("等待/错误日志限流周期（秒），范围 0.5~60"));

    validate_parameters();

    warn_period_ms_ = static_cast<int64_t>(warn_period_sec_ * 1000.0);
    status_period_sec_ = 1.0 / status_rate_hz_;
    control_period_sec_ = 1.0 / control_rate_hz_;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/control_demo/joy", rclcpp::QoS(1).best_effort(),
        std::bind(&ControlDemoNode::on_joy, this, std::placeholders::_1));
    command_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/control_demo/command", rclcpp::QoS(10).reliable(),
        std::bind(&ControlDemoNode::on_command, this, std::placeholders::_1));
    estop_clear_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/control_demo/estop_clear", rclcpp::QoS(10).reliable(),
        std::bind(&ControlDemoNode::on_estop_clear, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/control_demo/status", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(
        get_logger(),
        "启动配置: robot_ip=%s send_port=%lld recv_port=%lld role=%s "
        "rate=%.1fHz joy_timeout=%.2fs max_speed=%.2f auto_request=%s "
        "bridge_port=%lld",
        robot_ip_.c_str(), static_cast<long long>(send_port_),
        static_cast<long long>(recv_port_), role_.c_str(), control_rate_hz_,
        joy_timeout_sec_, max_speed_scale_, auto_request_control_ ? "true" : "false",
        static_cast<long long>(bridge_port_));

    holder_ = std::make_shared<ExecutorHolder>();
    holder_->ensure(
        parse_role(), robot_ip_, static_cast<uint32_t>(send_port_),
        static_cast<uint32_t>(recv_port_), std::chrono::steady_clock::now());

    bridge_thread_ = std::thread(&ControlDemoNode::bridge_server_loop, this);

    const auto period = std::chrono::milliseconds(
        std::max<int64_t>(1, static_cast<int64_t>(1000.0 / control_rate_hz_)));
    control_timer_ =
        create_wall_timer(period, std::bind(&ControlDemoNode::on_control_tick, this));
}

ControlDemoNode::~ControlDemoNode()
{
    bridge_stop_ = true;
    if (bridge_thread_.joinable()) {
        bridge_thread_.join();
    }
    if (control_timer_) {
        control_timer_->cancel();
    }
    shutdown_motion();
    holder_->shutdown();
}

void ControlDemoNode::validate_parameters() const
{
    if (robot_ip_.empty() ||
        std::any_of(robot_ip_.begin(), robot_ip_.end(), [](unsigned char ch) {
            return std::isspace(ch) != 0;
        }))
    {
        throw std::invalid_argument(
            "参数 robot_ip 不能为空或包含空白字符，当前值: '" + robot_ip_ + "'");
    }
    if (!in_range<int64_t>(send_port_, kMinPort, kMaxPort)) {
        throw std::invalid_argument(
            "参数 send_port 必须在 1~65535 之间，当前值: " + std::to_string(send_port_));
    }
    if (!in_range<int64_t>(recv_port_, kMinPort, kMaxPort)) {
        throw std::invalid_argument(
            "参数 recv_port 必须在 1~65535 之间，当前值: " + std::to_string(recv_port_));
    }
    if (send_port_ == recv_port_) {
        throw std::invalid_argument(
            "参数 send_port 和 recv_port 不能相同，当前均为: " + std::to_string(send_port_));
    }
    const std::string role = to_lower(role_);
    if (role != "sdk" && role != "remote") {
        throw std::invalid_argument(
            "参数 role 只能是 sdk 或 remote，当前值: '" + role_ + "'");
    }
    if (!in_range<double>(control_rate_hz_, kMinControlRateHz, kMaxControlRateHz)) {
        throw std::invalid_argument(
            "参数 control_rate_hz 必须在 5~100 之间，当前值: " +
            std::to_string(control_rate_hz_));
    }
    if (!in_range<double>(joy_timeout_sec_, kMinJoyTimeoutSec, kMaxJoyTimeoutSec)) {
        throw std::invalid_argument(
            "参数 joy_timeout_sec 必须在 0.05~5.0 之间，当前值: " +
            std::to_string(joy_timeout_sec_));
    }
    if (!in_range<double>(status_rate_hz_, kMinStatusRateHz, kMaxStatusRateHz)) {
        throw std::invalid_argument(
            "参数 status_rate_hz 必须在 0.5~20.0 之间，当前值: " +
            std::to_string(status_rate_hz_));
    }
    if (!in_range<double>(
            control_request_period_sec_, kMinControlRequestPeriodSec,
            kMaxControlRequestPeriodSec))
    {
        throw std::invalid_argument(
            "参数 control_request_period_sec 必须在 0.2~10.0 之间，当前值: " +
            std::to_string(control_request_period_sec_));
    }
    if (!in_range<double>(max_speed_scale_, kMinSpeedScale, kMaxSpeedScale)) {
        throw std::invalid_argument(
            "参数 max_speed_scale 必须在 0.1~1.0 之间，当前值: " +
            std::to_string(max_speed_scale_));
    }
    if (command_debounce_ms_ < 0 || command_debounce_ms_ > kMaxCommandDebounceMs) {
        throw std::invalid_argument(
            "参数 command_debounce_ms 必须在 0~5000 之间，当前值: " +
            std::to_string(command_debounce_ms_));
    }
    if (!is_posture_command(estop_recovery_cmd_)) {
        throw std::invalid_argument(
            "参数 estop_recovery_cmd 必须是姿态命令码（122/106/138/154/155），当前值: " +
            std::to_string(estop_recovery_cmd_));
    }
    if (!in_range<int64_t>(bridge_port_, kMinPort, kMaxPort)) {
        throw std::invalid_argument(
            "参数 bridge_port 必须在 1~65535 之间，当前值: " +
            std::to_string(bridge_port_));
    }
    if (!in_range<double>(warn_period_sec_, kMinWarnPeriodSec, kMaxWarnPeriodSec)) {
        throw std::invalid_argument(
            "参数 warn_period_sec 必须在 0.5~60.0 之间，当前值: " +
            std::to_string(warn_period_sec_));
    }
}

bool ControlDemoNode::role_is_sdk() const
{
    return to_lower(role_) == "sdk";
}

zsibot::Role ControlDemoNode::parse_role() const
{
    return role_is_sdk() ? zsibot::Role::ROLE_SDK : zsibot::Role::ROLE_REMOTE;
}

bool ControlDemoNode::is_allowed_command(uint8_t code)
{
    switch (code) {
        case kCmdEmergencyStop:
        case kCmdStandUp:
        case kCmdSitDown:
        case kCmdMoveMode:
        case kCmdBalanceStand:
        case kCmdLock:
        case kCmdSlow:
        case kCmdNormal:
        case kCmdFast:
            return true;
        default:
            return false;
    }
}

bool ControlDemoNode::is_joy_fresh(TimePoint now) const
{
    if (!joy_received_) {
        return false;
    }
    const double age =
        std::chrono::duration<double>(now - last_joy_).count();
    return age <= joy_timeout_sec_;
}

void ControlDemoNode::on_joy(const sensor_msgs::msg::Joy::SharedPtr message)
{
    std::array<float, 4> external{0.0f, 0.0f, 0.0f, 0.0f};
    for (std::size_t index = 0; index < external.size(); ++index) {
        if (index >= message->axes.size()) {
            break;
        }
        float value = message->axes[index];
        if (!std::isfinite(value)) {
            value = 0.0f;
        }
        external[index] = std::clamp(value, -1.0f, 1.0f);
    }
    // External topic order [forward, yaw, strafe, head] -> SDK order
    // [forward, strafe, yaw, head]. See axis_mapping.hpp.
    joy_axes_ = reorder_external_axes(
        external[0], external[1], external[2], external[3]);
    last_joy_ = std::chrono::steady_clock::now();
    joy_received_ = true;
    joy_stale_logged_ = false;
    ++joy_rx_count_;
}

void ControlDemoNode::on_command(const std_msgs::msg::UInt8::SharedPtr message)
{
    handle_command_code(message->data, std::chrono::steady_clock::now());
}

void ControlDemoNode::handle_command_code(uint8_t code, TimePoint now)
{
    if (!is_allowed_command(code)) {
        last_error_ = "未知命令码: " + std::to_string(code);
        RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
        return;
    }

    if (code == kCmdEmergencyStop) {
        estop_latched_ = true;
        estop_clear_pending_ = false;
        estop_recovery_sent_ = false;
        estop_started_ = now;
        in_control_ = false;
        pending_command_.reset();
        last_command_code_ = static_cast<int>(code);
        last_command_name_ = command_name(code);
        last_command_time_ = now;

        // Send immediately from the callback. A clear request arriving before
        // the next control tick must never suppress the E-stop itself.
        zsibot::ZsibotExecutor * executor = holder_->get();
        if (executor != nullptr) {
            send_cmd(*executor, kCmdEmergencyStop);
            last_estop_send_ = now;
            estop_resend_count_ = 1;
        } else {
            last_estop_send_ = TimePoint{};
            estop_resend_count_ = 0;
        }
        RCLCPP_WARN(get_logger(), "收到急停命令，锁定运动输出");
        return;
    }

    if (estop_latched_) {
        last_error_ = "急停已锁定，拒绝命令: " + command_name(code);
        return;
    }
    if (!connected_) {
        last_error_ = "未连接机器狗，拒绝命令: " + command_name(code);
        return;
    }
    if (!in_control_) {
        last_error_ = "未取得控制权，拒绝命令: " + command_name(code);
        return;
    }
    if (last_debounce_code_ == static_cast<int>(code) &&
        now - last_debounce_time_ < std::chrono::milliseconds(command_debounce_ms_))
    {
        return;
    }

    last_debounce_code_ = static_cast<int>(code);
    last_debounce_time_ = now;
    pending_command_ = code;
    pending_command_time_ = now;
    last_error_.clear();
}

void ControlDemoNode::on_estop_clear(const std_msgs::msg::Bool::SharedPtr message)
{
    handle_estop_clear(message->data);
}

void ControlDemoNode::handle_estop_clear(bool value)
{
    if (!value || !estop_latched_) {
        return;
    }
    estop_clear_pending_ = true;
    estop_clear_start_ = std::chrono::steady_clock::now();
    estop_recovery_sent_ = false;
    RCLCPP_INFO(get_logger(), "收到急停恢复请求");
}

void ControlDemoNode::bridge_server_loop()
{
    const int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        RCLCPP_ERROR(get_logger(), "无法创建 TCP 控制桥 socket");
        return;
    }

    int reuse = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<uint16_t>(bridge_port_));

    if (bind(listen_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
        listen(listen_fd, 1) < 0)
    {
        RCLCPP_ERROR(
            get_logger(), "TCP 控制桥无法监听 127.0.0.1:%lld（端口被占用？）",
            static_cast<long long>(bridge_port_));
        close(listen_fd);
        return;
    }
    RCLCPP_INFO(
        get_logger(), "TCP 控制桥监听 127.0.0.1:%lld",
        static_cast<long long>(bridge_port_));

    while (!bridge_stop_.load()) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(listen_fd, &read_fds);
        int max_fd = listen_fd;

        int client_fd = -1;
        {
            std::lock_guard<std::mutex> lock(bridge_send_mutex_);
            client_fd = bridge_client_fd_;
        }
        if (client_fd >= 0) {
            FD_SET(client_fd, &read_fds);
            max_fd = std::max(max_fd, client_fd);
        }

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;
        const int ready = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready <= 0) {
            continue;
        }

        if (FD_ISSET(listen_fd, &read_fds)) {
            const int new_fd = accept(listen_fd, nullptr, nullptr);
            if (new_fd >= 0) {
                std::lock_guard<std::mutex> lock(bridge_send_mutex_);
                if (bridge_client_fd_ >= 0) {
                    close(bridge_client_fd_);
                }
                bridge_client_fd_ = new_fd;
                bridge_read_buffer_.clear();
                RCLCPP_INFO(get_logger(), "Web 控制桥已连接");
            }
        }

        {
            std::lock_guard<std::mutex> lock(bridge_send_mutex_);
            client_fd = bridge_client_fd_;
        }
        if (client_fd >= 0 && FD_ISSET(client_fd, &read_fds)) {
            char buffer[512];
            const ssize_t received = recv(client_fd, buffer, sizeof(buffer), 0);
            if (received <= 0) {
                std::lock_guard<std::mutex> lock(bridge_send_mutex_);
                if (bridge_client_fd_ == client_fd) {
                    close(client_fd);
                    bridge_client_fd_ = -1;
                    bridge_read_buffer_.clear();
                    RCLCPP_INFO(get_logger(), "Web 控制桥已断开");
                }
            } else {
                bridge_read_buffer_.append(buffer, static_cast<std::size_t>(received));
                std::size_t newline = 0;
                while ((newline = bridge_read_buffer_.find('\n')) != std::string::npos) {
                    const std::string line = bridge_read_buffer_.substr(0, newline);
                    bridge_read_buffer_.erase(0, newline + 1);
                    bridge_handle_line(line);
                }
                if (bridge_read_buffer_.size() > 4096) {
                    bridge_read_buffer_.clear();
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(bridge_send_mutex_);
        if (bridge_client_fd_ >= 0) {
            close(bridge_client_fd_);
            bridge_client_fd_ = -1;
        }
    }
    close(listen_fd);
    RCLCPP_INFO(get_logger(), "TCP 控制桥已停止");
}

void ControlDemoNode::bridge_handle_line(const std::string & line)
{
    if (line.empty()) {
        return;
    }

    if (line.rfind("J:", 0) == 0) {
        float forward = 0.0f;
        float yaw = 0.0f;
        float strafe = 0.0f;
        float head = 0.0f;
        if (std::sscanf(
                line.c_str() + 2, "%f,%f,%f,%f", &forward, &yaw, &strafe, &head) != 4)
        {
            return;
        }
        BridgeInput input;
        input.type = BridgeInput::Type::JOY;
        // J: line order is forward,yaw,strafe,head; the SDK expects
        // forward,strafe,yaw,head.
        input.axes = reorder_external_axes(
            std::clamp(forward, -1.0f, 1.0f),
            std::clamp(yaw, -1.0f, 1.0f),
            std::clamp(strafe, -1.0f, 1.0f),
            std::clamp(head, -1.0f, 1.0f));
        std::lock_guard<std::mutex> lock(bridge_input_mutex_);
        if (bridge_inputs_.size() >= 256) {
            bridge_inputs_.pop_front();
        }
        bridge_inputs_.push_back(input);
        return;
    }

    if (line.rfind("C:", 0) == 0) {
        const int code = std::atoi(line.c_str() + 2);
        if (code < 0 || code > 255) {
            return;
        }
        BridgeInput input;
        input.type = BridgeInput::Type::COMMAND;
        input.code = static_cast<uint8_t>(code);
        std::lock_guard<std::mutex> lock(bridge_input_mutex_);
        if (bridge_inputs_.size() >= 256) {
            bridge_inputs_.pop_front();
        }
        bridge_inputs_.push_back(input);
        return;
    }

    if (line == "EC") {
        BridgeInput input;
        input.type = BridgeInput::Type::ESTOP_CLEAR;
        std::lock_guard<std::mutex> lock(bridge_input_mutex_);
        if (bridge_inputs_.size() >= 256) {
            bridge_inputs_.pop_front();
        }
        bridge_inputs_.push_back(input);
    }
}

void ControlDemoNode::drain_bridge_inputs(TimePoint now)
{
    std::deque<BridgeInput> inputs;
    {
        std::lock_guard<std::mutex> lock(bridge_input_mutex_);
        inputs.swap(bridge_inputs_);
    }
    for (const BridgeInput & input : inputs) {
        switch (input.type) {
            case BridgeInput::Type::JOY:
                joy_axes_ = input.axes;
                last_joy_ = now;
                joy_received_ = true;
                joy_stale_logged_ = false;
                ++joy_rx_count_;
                break;
            case BridgeInput::Type::COMMAND:
                handle_command_code(input.code, now);
                break;
            case BridgeInput::Type::ESTOP_CLEAR:
                handle_estop_clear(true);
                break;
        }
    }
}

void ControlDemoNode::send_bridge_status(const std::string & json)
{
    std::lock_guard<std::mutex> lock(bridge_send_mutex_);
    if (bridge_client_fd_ < 0) {
        return;
    }
    const std::string line = "S:" + json + "\n";
    std::size_t offset = 0;
    while (offset < line.size()) {
        const ssize_t sent = send(
            bridge_client_fd_, line.data() + offset, line.size() - offset,
            MSG_NOSIGNAL);
        if (sent <= 0) {
            close(bridge_client_fd_);
            bridge_client_fd_ = -1;
            return;
        }
        offset += static_cast<std::size_t>(sent);
    }
}

bool ControlDemoNode::refresh_feedback(zsibot::ZsibotExecutor & executor)
{
    try {
        connected_ = executor.IsConnected();
        model_ = model_name(static_cast<int>(executor.GetModel()));
        battery_ = executor.GetBatteryInfo();
        body_temp_c_ = executor.GetTemperature();
        speed_level_ = static_cast<int>(executor.GetSpeedLevel());
        function_mode_ = static_cast<int>(executor.GetFunctionMode());
        control_mode_ = static_cast<int>(executor.GetControlMode());
        motion_mode_ = static_cast<int>(executor.GetMotionMode());
        motion_type_ = static_cast<int>(executor.GetMotionType());
        faults_ = executor.GetFaultInfo();
    } catch (const std::exception & exception) {
        connected_ = false;
        in_control_ = false;
        last_error_ = std::string("读取状态失败: ") + exception.what();
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), warn_period_ms_, "%s", last_error_.c_str());
        return false;
    }

    if (connected_ && !estop_latched_ &&
        control_mode_ == static_cast<int>(zsibot::ControlMode::CM_EMERGENCY_STOP))
    {
        estop_latched_ = true;
        estop_clear_pending_ = false;
        estop_recovery_sent_ = false;
        estop_started_ = std::chrono::steady_clock::now();
        estop_resend_count_ = 0;
        last_estop_send_ = TimePoint{};
        in_control_ = false;
        pending_command_.reset();
        RCLCPP_WARN(get_logger(), "机器人反馈处于急停状态，锁定运动输出");
    }
    return true;
}

void ControlDemoNode::update_control_state(
    zsibot::ZsibotExecutor & executor, TimePoint now)
{
    const bool emergency =
        control_mode_ == static_cast<int>(zsibot::ControlMode::CM_EMERGENCY_STOP);

    if (!connected_) {
        if (in_control_) {
            RCLCPP_WARN(get_logger(), "与机器狗连接断开，停止运动");
        }
        in_control_ = false;
        state_ = ControlState::WAIT_CONNECTION;
        control_confirm_ticks_ = 0;
        return;
    }

    if (role_is_sdk()) {
        const bool sdk_ready =
            function_mode_ == static_cast<int>(zsibot::FunctionMode::FM_SDK) && !emergency;
        if (in_control_) {
            if (!sdk_ready) {
                in_control_ = false;
                state_ = ControlState::REQUESTING_CONTROL;
                control_confirm_ticks_ = 0;
                RCLCPP_WARN(get_logger(), "SDK 控制权丢失，停止运动");
            }
            return;
        }

        state_ = ControlState::REQUESTING_CONTROL;
        if (auto_request_control_ &&
            (last_control_request_ == TimePoint{} ||
             now - last_control_request_ >=
                 std::chrono::duration<double>(control_request_period_sec_)))
        {
            send_cmd(executor, kCmdSdkControlRight);
            last_control_request_ = now;
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), warn_period_ms_, "正在请求 SDK 控制权...");
        }

        if (sdk_ready) {
            ++control_confirm_ticks_;
            if (control_confirm_ticks_ >= 2) {
                in_control_ = true;
                state_ = ControlState::IN_CONTROL;
                control_confirm_ticks_ = 0;
                last_error_.clear();
                RCLCPP_INFO(get_logger(), "已取得 SDK 控制权");
            }
        } else {
            control_confirm_ticks_ = 0;
        }
        return;
    }

    const bool remote_ready =
        function_mode_ == static_cast<int>(zsibot::FunctionMode::FM_REMOTE) && !emergency;
    if (remote_ready != in_control_) {
        in_control_ = remote_ready;
        state_ = remote_ready ? ControlState::IN_CONTROL
                              : ControlState::WAIT_CONNECTION;
        if (remote_ready) {
            RCLCPP_INFO(get_logger(), "已取得遥控模式的运动输出");
        } else {
            RCLCPP_WARN(
                get_logger(),
                "机器人当前功能模式不是 remote，运动指令可能被丢弃");
        }
    }
}

void ControlDemoNode::process_pending_command(
    zsibot::ZsibotExecutor & executor, TimePoint now)
{
    if (!pending_command_.has_value()) {
        return;
    }
    const uint8_t code = pending_command_.value();
    pending_command_.reset();

    if (!connected_ || !in_control_) {
        last_error_ = "控制权丢失，命令未发送: " + command_name(code);
        return;
    }

    // 姿态切换前先发送零运动，避免摇杆保持导致切换期间机器人移动。
    set_remote(executor, std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f});
    send_cmd(executor, code);

    last_command_code_ = static_cast<int>(code);
    last_command_name_ = command_name(code);
    last_command_time_ = now;
    if (code != kCmdSlow && code != kCmdNormal && code != kCmdFast) {
        command_hold_until_ = now + kCommandHoldDuration;
    }
    RCLCPP_INFO(
        get_logger(), "发送命令: %s (%u)", last_command_name_.c_str(),
        static_cast<unsigned int>(code));
}

void ControlDemoNode::handle_estop_tick(
    zsibot::ZsibotExecutor & executor, TimePoint now)
{
    in_control_ = false;
    set_remote(executor, std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f});

    // Always finish the E-stop resend cycle before considering recovery, even
    // if a clear request is already pending.
    if (estop_resend_count_ < kEstopResendCount &&
        (estop_resend_count_ == 0 || now - last_estop_send_ >= kEstopResendPeriod))
    {
        send_cmd(executor, kCmdEmergencyStop);
        last_estop_send_ = now;
        ++estop_resend_count_;
        last_command_code_ = static_cast<int>(kCmdEmergencyStop);
        last_command_name_ = command_name(kCmdEmergencyStop);
        last_command_time_ = now;
        return;
    }

    if (!estop_clear_pending_) {
        return;
    }

    const bool control_ready =
        connected_ &&
        (role_is_sdk()
             ? function_mode_ == static_cast<int>(zsibot::FunctionMode::FM_SDK)
             : function_mode_ == static_cast<int>(zsibot::FunctionMode::FM_REMOTE));

    if (!control_ready) {
        if (role_is_sdk() && auto_request_control_ && connected_ &&
            (last_control_request_ == TimePoint{} ||
             now - last_control_request_ >=
                 std::chrono::duration<double>(control_request_period_sec_)))
        {
            send_cmd(executor, kCmdSdkControlRight);
            last_control_request_ = now;
        }
        if (now - estop_clear_start_ >= kRecoveryTimeout) {
            estop_clear_pending_ = false;
            estop_recovery_sent_ = false;
            last_error_ = "急停恢复超时：未能重新取得控制权";
            RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
        }
        return;
    }

    if (!estop_recovery_sent_) {
        send_cmd(executor, static_cast<uint8_t>(estop_recovery_cmd_));
        estop_recovery_sent_ = true;
        estop_recovery_time_ = now;
        last_command_code_ = static_cast<int>(estop_recovery_cmd_);
        last_command_name_ = command_name(last_command_code_.value());
        last_command_time_ = now;
        RCLCPP_INFO(
            get_logger(), "急停恢复：发送姿态命令 %s", last_command_name_.c_str());
        return;
    }

    if (now - estop_recovery_time_ >= kRecoveryConfirmDelay &&
        control_mode_ != static_cast<int>(zsibot::ControlMode::CM_EMERGENCY_STOP))
    {
        estop_latched_ = false;
        estop_clear_pending_ = false;
        estop_recovery_sent_ = false;
        estop_resend_count_ = 0;
        in_control_ = true;
        state_ = ControlState::IN_CONTROL;
        last_error_.clear();
        RCLCPP_INFO(get_logger(), "急停已解除，恢复运动输出");
    } else if (now - estop_clear_start_ >= kRecoveryTimeout) {
        estop_clear_pending_ = false;
        estop_recovery_sent_ = false;
        last_error_ = "急停恢复超时：机器人未离开急停状态";
        RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
    }
}

void ControlDemoNode::set_remote(
    zsibot::ZsibotExecutor & executor, const std::array<float, 4> & axes)
{
    try {
        executor.SetRemote(axes, std::array<float, 14>{});
        ++tx_count_;
    } catch (const std::exception & exception) {
        last_error_ = std::string("SetRemote 失败: ") + exception.what();
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), warn_period_ms_, "%s", last_error_.c_str());
    }
}

void ControlDemoNode::send_cmd(zsibot::ZsibotExecutor & executor, uint8_t code)
{
    try {
        executor.SetCmd(static_cast<zsibot::CmdCode>(code));
    } catch (const std::exception & exception) {
        last_error_ = std::string("SetCmd 失败: ") + exception.what();
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), warn_period_ms_, "%s", last_error_.c_str());
    }
}

void ControlDemoNode::update_rates(TimePoint now)
{
    if (rate_window_start_ == TimePoint{}) {
        rate_window_start_ = now;
        tx_count_ = 0;
        joy_rx_count_ = 0;
        return;
    }
    const double elapsed =
        std::chrono::duration<double>(now - rate_window_start_).count();
    if (elapsed >= 1.0) {
        tx_hz_ = static_cast<double>(tx_count_) / elapsed;
        joy_hz_ = static_cast<double>(joy_rx_count_) / elapsed;
        tx_count_ = 0;
        joy_rx_count_ = 0;
        rate_window_start_ = now;
    }
}

void ControlDemoNode::publish_status(TimePoint now)
{
    if (last_status_publish_ != TimePoint{} &&
        now - last_status_publish_ <
            std::chrono::duration<double>(status_period_sec_))
    {
        return;
    }
    last_status_publish_ = now;

    StatusSnapshot snapshot;
    snapshot.executor_ready = holder_->get() != nullptr;
    snapshot.connected = connected_;
    snapshot.in_control = in_control_;
    snapshot.role = role_;
    snapshot.robot_ip = robot_ip_;
    snapshot.model = model_;
    snapshot.battery.power = battery_.power;
    snapshot.battery.volt = battery_.volt;
    snapshot.battery.current = battery_.current;
    snapshot.battery.temp = battery_.temp;
    snapshot.battery.error = battery_.error;
    snapshot.body_temp_c = body_temp_c_;
    snapshot.speed_level = speed_level_;
    snapshot.speed_level_name = speed_level_name(speed_level_);
    snapshot.function_mode = function_mode_;
    snapshot.function_mode_name = function_mode_name(function_mode_);
    snapshot.control_mode = control_mode_;
    snapshot.control_mode_name = control_mode_name(control_mode_);
    snapshot.motion_mode = motion_mode_;
    snapshot.motion_mode_name = motion_mode_name(motion_mode_);
    snapshot.motion_type = motion_type_;
    snapshot.motion_type_name = motion_type_name(motion_type_);
    snapshot.fault_count = static_cast<int>(faults_.size());
    for (std::size_t index = 0; index < faults_.size() && index < 5; ++index) {
        const zsibot::FaultInfo & fault = faults_[index];
        snapshot.faults.push_back(FaultSnapshot{
            fault.module, fault.submodule, fault.level, fault.info, fault.error_code});
    }
    snapshot.estop_latched = estop_latched_;
    snapshot.estop_clear_pending = estop_clear_pending_;
    if (joy_received_) {
        snapshot.last_joy_age_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_joy_)
                .count();
    }
    if (last_command_code_.has_value()) {
        snapshot.last_command = CommandSnapshot{
            last_command_code_.value(), last_command_name_,
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_time_)
                .count()};
    }
    snapshot.last_error = last_error_;
    snapshot.control_tx_hz = tx_hz_;
    snapshot.joy_rx_hz = joy_hz_;
    snapshot.status_seq = ++status_seq_;

    std_msgs::msg::String message;
    message.data = build_status_json(snapshot);
    send_bridge_status(message.data);
    status_pub_->publish(message);
}

void ControlDemoNode::shutdown_motion()
{
    zsibot::ZsibotExecutor * executor = holder_->get();
    if (executor == nullptr) {
        return;
    }
    try {
        for (int count = 0; count < kShutdownZeroCount; ++count) {
            executor->SetRemote(
                std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f},
                std::array<float, 14>{});
            std::this_thread::sleep_for(kShutdownZeroPeriod);
        }
    } catch (const std::exception & exception) {
        RCLCPP_WARN(
            get_logger(), "退出时发送归零指令失败: %s", exception.what());
    }
}

void ControlDemoNode::on_control_tick()
{
    const auto now = std::chrono::steady_clock::now();
    drain_bridge_inputs(now);
    update_rates(now);

    if (!holder_->ensure(
            parse_role(), robot_ip_, static_cast<uint32_t>(send_port_),
            static_cast<uint32_t>(recv_port_), now))
    {
        std::string error;
        if (holder_->take_failure(error)) {
            last_error_ = error;
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), warn_period_ms_,
                "创建 ZsibotExecutor 失败: %s（约 5 秒后重试）", error.c_str());
        } else if (holder_->in_progress()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), warn_period_ms_,
                "正在等待机器狗连接（SDK 初始化中）...");
        }
        state_ = ControlState::WAIT_EXECUTOR;
        publish_status(now);
        return;
    }

    zsibot::ZsibotExecutor * executor = holder_->get();
    if (executor == nullptr) {
        publish_status(now);
        return;
    }

    refresh_feedback(*executor);

    if (estop_latched_) {
        state_ = ControlState::ESTOP_LATCHED;
        handle_estop_tick(*executor, now);
        publish_status(now);
        return;
    }

    update_control_state(*executor, now);

    if (pending_command_.has_value()) {
        process_pending_command(*executor, now);
    }

    if (joy_received_ && !is_joy_fresh(now) && !joy_stale_logged_) {
        RCLCPP_WARN(
            get_logger(), "摇杆输入超时（>%.2f s），输出归零", joy_timeout_sec_);
        joy_stale_logged_ = true;
    }

    std::array<float, 4> axes{0.0f, 0.0f, 0.0f, 0.0f};
    if (connected_ && in_control_ && is_joy_fresh(now) &&
        now >= command_hold_until_)
    {
        for (std::size_t index = 0; index < axes.size(); ++index) {
            const float scaled =
                joy_axes_[index] * static_cast<float>(max_speed_scale_);
            axes[index] = std::clamp(scaled, -1.0f, 1.0f);
        }
    }
    set_remote(*executor, axes);
    publish_status(now);
}

}  // namespace control_demo_driver

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<control_demo_driver::ControlDemoNode>();
        rclcpp::spin(node);
    } catch (const std::exception & exception) {
        RCLCPP_FATAL(
            rclcpp::get_logger("control_demo_driver"), "节点启动失败: %s",
            exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
