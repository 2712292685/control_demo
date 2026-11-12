#include "control_demo_driver/status_builder.hpp"

#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

#include "zsibot_sdk/zsibot_define.h"

namespace control_demo_driver
{
namespace
{

std::string escape_json(const std::string & value)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    for (const unsigned char ch : value) {
        switch (ch) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\b':
                out << "\\b";
                break;
            case '\f':
                out << "\\f";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                if (ch < 0x20) {
                    out << "\\u" << std::hex << std::uppercase << std::setw(4)
                        << std::setfill('0') << static_cast<int>(ch) << std::dec;
                } else {
                    out << static_cast<char>(ch);
                }
                break;
        }
    }
    return out.str();
}

std::string number(double value, int precision)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

std::string boolean(bool value)
{
    return value ? "true" : "false";
}

template <typename EnumT>
std::string unknown_name(int value)
{
    (void)sizeof(EnumT);
    return "unknown(" + std::to_string(value) + ")";
}

}  // namespace

std::string model_name(int value)
{
    switch (static_cast<zsibot::Model>(value)) {
        case zsibot::Model::MODEL_XG:
            return "XG";
        case zsibot::Model::MODEL_XGW:
            return "XGW";
        case zsibot::Model::MODEL_XGWHSPD:
            return "XGWHSPD";
    }
    return unknown_name<zsibot::Model>(value);
}

std::string speed_level_name(int value)
{
    switch (static_cast<zsibot::SpeedLevel>(value)) {
        case zsibot::SpeedLevel::SL_NULL:
            return "null";
        case zsibot::SpeedLevel::SL_SLOW:
            return "slow";
        case zsibot::SpeedLevel::SL_NORMAL:
            return "normal";
        case zsibot::SpeedLevel::SL_FAST:
            return "fast";
    }
    return unknown_name<zsibot::SpeedLevel>(value);
}

std::string function_mode_name(int value)
{
    switch (static_cast<zsibot::FunctionMode>(value)) {
        case zsibot::FunctionMode::FM_NULL:
            return "null";
        case zsibot::FunctionMode::FM_REMOTE:
            return "remote";
        case zsibot::FunctionMode::FM_TRACE:
            return "trace";
        case zsibot::FunctionMode::FM_SDK:
            return "sdk";
        case zsibot::FunctionMode::FM_GENERAL_SDK:
            return "general_sdk";
        case zsibot::FunctionMode::FM_ROAMER:
            return "roamer";
    }
    return unknown_name<zsibot::FunctionMode>(value);
}

std::string control_mode_name(int value)
{
    switch (static_cast<zsibot::ControlMode>(value)) {
        case zsibot::ControlMode::CM_NULL:
            return "null";
        case zsibot::ControlMode::CM_STAND_UP:
            return "stand_up";
        case zsibot::ControlMode::CM_SIT_DOWN:
            return "stand_down";
        case zsibot::ControlMode::CM_LOCK_MODE:
            return "lock";
        case zsibot::ControlMode::CM_EMERGENCY_STOP:
            return "emergency";
        case zsibot::ControlMode::CM_MOVE_MODE:
            return "move";
        case zsibot::ControlMode::CM_BALANCE_STAND_MODE:
            return "stay";
        case zsibot::ControlMode::CM_INIT:
            return "init";
    }
    return unknown_name<zsibot::ControlMode>(value);
}

std::string motion_mode_name(int value)
{
    switch (static_cast<zsibot::MotionMode>(value)) {
        case zsibot::MotionMode::MM_NULL:
            return "null";
        case zsibot::MotionMode::MM_RUNNING:
            return "running";
        case zsibot::MotionMode::MM_REST:
            return "rest";
        case zsibot::MotionMode::MM_FORBID:
            return "forbid";
    }
    return unknown_name<zsibot::MotionMode>(value);
}

std::string motion_type_name(int value)
{
    switch (static_cast<zsibot::MotionType>(value)) {
        case zsibot::MotionType::MT_NULL:
            return "null";
        case zsibot::MotionType::MT_JUMP:
            return "jump";
        case zsibot::MotionType::MT_FORWARD_JUMP:
            return "forward_jump";
        case zsibot::MotionType::MT_BACK_FLIP:
            return "back_flip";
        case zsibot::MotionType::MT_GREET:
            return "greet";
        case zsibot::MotionType::MT_TWO_LEG_STAND:
            return "two_leg_stand";
        case zsibot::MotionType::MT_FORBID:
            return "forbid";
        case zsibot::MotionType::MT_UNKNOWN:
            return "unknown";
        case zsibot::MotionType::MT_UNLOADING_SQUAT:
            return "unloading_squat";
    }
    return unknown_name<zsibot::MotionType>(value);
}

std::string command_name(int code)
{
    switch (code) {
        case 90:
            return "emergency_stop";
        case 122:
            return "stand_up";
        case 106:
            return "sit_down";
        case 138:
            return "move_mode";
        case 154:
            return "balance_stand";
        case 155:
            return "lock";
        case 174:
            return "slow";
        case 175:
            return "normal";
        case 176:
            return "fast";
        default:
            return "unknown(" + std::to_string(code) + ")";
    }
}

std::string build_status_json(const StatusSnapshot & snapshot)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());

    out << "{";
    out << "\"schema_version\":1";
    out << ",\"status_seq\":" << snapshot.status_seq;
    out << ",\"executor_ready\":" << boolean(snapshot.executor_ready);
    out << ",\"connected\":" << boolean(snapshot.connected);
    out << ",\"in_control\":" << boolean(snapshot.in_control);
    out << ",\"role\":\"" << escape_json(snapshot.role) << "\"";
    out << ",\"robot_ip\":\"" << escape_json(snapshot.robot_ip) << "\"";
    out << ",\"model\":\"" << escape_json(snapshot.model) << "\"";
    out << ",\"battery\":{"
        << "\"power\":" << snapshot.battery.power
        << ",\"volt\":" << number(snapshot.battery.volt, 2)
        << ",\"current\":" << number(snapshot.battery.current, 2)
        << ",\"temp\":" << number(snapshot.battery.temp, 1)
        << ",\"error\":" << snapshot.battery.error << "}";
    out << ",\"body_temp_c\":" << number(snapshot.body_temp_c, 1);
    out << ",\"speed_level\":" << snapshot.speed_level
        << ",\"speed_level_name\":\"" << escape_json(snapshot.speed_level_name) << "\"";
    out << ",\"function_mode\":" << snapshot.function_mode
        << ",\"function_mode_name\":\"" << escape_json(snapshot.function_mode_name) << "\"";
    out << ",\"control_mode\":" << snapshot.control_mode
        << ",\"control_mode_name\":\"" << escape_json(snapshot.control_mode_name) << "\"";
    out << ",\"motion_mode\":" << snapshot.motion_mode
        << ",\"motion_mode_name\":\"" << escape_json(snapshot.motion_mode_name) << "\"";
    out << ",\"motion_type\":" << snapshot.motion_type
        << ",\"motion_type_name\":\"" << escape_json(snapshot.motion_type_name) << "\"";
    out << ",\"fault_count\":" << snapshot.fault_count;
    out << ",\"faults\":[";
    for (std::size_t i = 0; i < snapshot.faults.size(); ++i) {
        if (i > 0) {
            out << ",";
        }
        const FaultSnapshot & fault = snapshot.faults[i];
        out << "{\"module\":\"" << escape_json(fault.module)
            << "\",\"submodule\":\"" << escape_json(fault.submodule)
            << "\",\"code\":" << fault.code << ",\"level\":\"" << escape_json(fault.level)
            << "\",\"info\":\"" << escape_json(fault.info) << "\"}";
    }
    out << "]";
    out << ",\"estop_latched\":" << boolean(snapshot.estop_latched);
    out << ",\"estop_clear_pending\":" << boolean(snapshot.estop_clear_pending);
    out << ",\"last_joy_age_ms\":";
    if (snapshot.last_joy_age_ms.has_value()) {
        out << snapshot.last_joy_age_ms.value();
    } else {
        out << "null";
    }
    out << ",\"last_command\":";
    if (snapshot.last_command.has_value()) {
        const CommandSnapshot & command = snapshot.last_command.value();
        out << "{\"code\":" << command.code << ",\"name\":\"" << escape_json(command.name)
            << "\",\"age_ms\":" << command.age_ms << "}";
    } else {
        out << "null";
    }
    out << ",\"last_error\":\"" << escape_json(snapshot.last_error) << "\"";
    out << ",\"control_tx_hz\":" << number(snapshot.control_tx_hz, 1);
    out << ",\"joy_rx_hz\":" << number(snapshot.joy_rx_hz, 1);
    out << "}";
    return out.str();
}

}  // namespace control_demo_driver
