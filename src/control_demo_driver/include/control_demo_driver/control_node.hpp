#ifndef CONTROL_DEMO_DRIVER__CONTROL_NODE_HPP_
#define CONTROL_DEMO_DRIVER__CONTROL_NODE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "zsibot_sdk/zsibot_api.h"

#include "control_demo_driver/executor_holder.hpp"
#include "control_demo_driver/status_builder.hpp"

namespace control_demo_driver
{

class ControlDemoNode final : public rclcpp::Node
{
   public:
    ControlDemoNode();
    ~ControlDemoNode() override;

   private:
    using TimePoint = std::chrono::steady_clock::time_point;

    struct BridgeInput
    {
        enum class Type
        {
            JOY,
            COMMAND,
            ESTOP_CLEAR,
        };
        Type type{Type::JOY};
        std::array<float, 4> axes{0.0f, 0.0f, 0.0f, 0.0f};
        uint8_t code{0};
    };

    enum class ControlState
    {
        WAIT_EXECUTOR,
        WAIT_CONNECTION,
        REQUESTING_CONTROL,
        IN_CONTROL,
        ESTOP_LATCHED,
    };

    void on_control_tick();
    void on_joy(const sensor_msgs::msg::Joy::SharedPtr message);
    void on_command(const std_msgs::msg::UInt8::SharedPtr message);
    void on_estop_clear(const std_msgs::msg::Bool::SharedPtr message);

    void bridge_server_loop();
    void bridge_handle_line(const std::string & line);
    void drain_bridge_inputs(TimePoint now);
    void send_bridge_status(const std::string & json);
    void handle_command_code(uint8_t code, TimePoint now);
    void handle_estop_clear(bool value);

    bool refresh_feedback(zsibot::ZsibotExecutor & executor);
    void update_control_state(zsibot::ZsibotExecutor & executor, TimePoint now);
    void process_pending_command(zsibot::ZsibotExecutor & executor, TimePoint now);
    void handle_estop_tick(zsibot::ZsibotExecutor & executor, TimePoint now);
    void set_remote(zsibot::ZsibotExecutor & executor, const std::array<float, 4> & axes);
    void send_cmd(zsibot::ZsibotExecutor & executor, uint8_t code);
    void publish_status(TimePoint now);
    void update_rates(TimePoint now);
    void shutdown_motion();

    bool is_joy_fresh(TimePoint now) const;
    bool role_is_sdk() const;
    zsibot::Role parse_role() const;
    void validate_parameters() const;
    static bool is_allowed_command(uint8_t code);

    std::shared_ptr<ExecutorHolder> holder_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_clear_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    std::string robot_ip_{"192.168.234.1"};
    int64_t send_port_{8081};
    int64_t recv_port_{8080};
    std::string role_{"sdk"};
    double control_rate_hz_{30.0};
    double joy_timeout_sec_{0.35};
    double status_rate_hz_{2.0};
    bool auto_request_control_{true};
    double control_request_period_sec_{1.0};
    double max_speed_scale_{0.5};
    int64_t command_debounce_ms_{300};
    int64_t estop_recovery_cmd_{122};
    int64_t bridge_port_{8765};
    double warn_period_sec_{5.0};

    int64_t warn_period_ms_{5000};
    double status_period_sec_{0.5};
    double control_period_sec_{0.033};

    ControlState state_{ControlState::WAIT_EXECUTOR};
    bool connected_{false};
    bool in_control_{false};
    std::string model_;
    zsibot::BatteryInfo battery_{};
    float body_temp_c_{0.0f};
    int speed_level_{0};
    int function_mode_{0};
    int control_mode_{0};
    int motion_mode_{0};
    int motion_type_{0};
    std::vector<zsibot::FaultInfo> faults_;

    std::array<float, 4> joy_axes_{0.0f, 0.0f, 0.0f, 0.0f};
    TimePoint last_joy_{};
    bool joy_received_{false};
    bool joy_stale_logged_{false};

    bool estop_latched_{false};
    bool estop_clear_pending_{false};
    bool estop_recovery_sent_{false};
    TimePoint estop_started_{};
    TimePoint estop_clear_start_{};
    TimePoint estop_recovery_time_{};
    TimePoint last_estop_send_{};
    int estop_resend_count_{0};

    TimePoint last_control_request_{};
    int control_confirm_ticks_{0};

    std::optional<uint8_t> pending_command_;
    TimePoint pending_command_time_{};
    TimePoint command_hold_until_{};
    int last_debounce_code_{-1};
    TimePoint last_debounce_time_{};

    std::optional<int> last_command_code_;
    std::string last_command_name_;
    TimePoint last_command_time_{};

    std::string last_error_;
    uint64_t status_seq_{0};
    TimePoint last_status_publish_{};

    TimePoint rate_window_start_{};
    uint64_t tx_count_{0};
    uint64_t joy_rx_count_{0};
    double tx_hz_{0.0};
    double joy_hz_{0.0};

    // 本机 TCP 控制桥：仅监听 127.0.0.1，供 Python Web 节点使用。
    // 这样 Web ↔ 驱动不依赖 DDS，避免 WSL2 下两个本地节点偶发无法匹配。
    std::thread bridge_thread_;
    std::atomic<bool> bridge_stop_{false};
    int bridge_client_fd_{-1};
    std::mutex bridge_send_mutex_;
    std::mutex bridge_input_mutex_;
    std::deque<BridgeInput> bridge_inputs_;
    std::string bridge_read_buffer_;
};

}  // namespace control_demo_driver

#endif  // CONTROL_DEMO_DRIVER__CONTROL_NODE_HPP_
