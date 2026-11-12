#ifndef CONTROL_DEMO_DRIVER__STATUS_BUILDER_HPP_
#define CONTROL_DEMO_DRIVER__STATUS_BUILDER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace control_demo_driver
{

struct BatterySnapshot
{
    int power{0};
    float volt{0.0f};
    float current{0.0f};
    float temp{0.0f};
    int error{0};
};

struct FaultSnapshot
{
    std::string module;
    std::string submodule;
    std::string level;
    std::string info;
    uint32_t code{0};
};

struct CommandSnapshot
{
    int code{0};
    std::string name;
    int64_t age_ms{0};
};

struct StatusSnapshot
{
    bool executor_ready{false};
    bool connected{false};
    bool in_control{false};
    std::string role{"sdk"};
    std::string robot_ip;
    std::string model;
    BatterySnapshot battery;
    float body_temp_c{0.0f};
    int speed_level{0};
    std::string speed_level_name{"null"};
    int function_mode{0};
    std::string function_mode_name{"null"};
    int control_mode{0};
    std::string control_mode_name{"null"};
    int motion_mode{0};
    std::string motion_mode_name{"null"};
    int motion_type{0};
    std::string motion_type_name{"null"};
    int fault_count{0};
    std::vector<FaultSnapshot> faults;
    bool estop_latched{false};
    bool estop_clear_pending{false};
    std::optional<int64_t> last_joy_age_ms;
    std::optional<CommandSnapshot> last_command;
    std::string last_error;
    double control_tx_hz{0.0};
    double joy_rx_hz{0.0};
    uint64_t status_seq{0};
};

std::string build_status_json(const StatusSnapshot & snapshot);

std::string model_name(int value);
std::string speed_level_name(int value);
std::string function_mode_name(int value);
std::string control_mode_name(int value);
std::string motion_mode_name(int value);
std::string motion_type_name(int value);
std::string command_name(int code);

}  // namespace control_demo_driver

#endif  // CONTROL_DEMO_DRIVER__STATUS_BUILDER_HPP_
