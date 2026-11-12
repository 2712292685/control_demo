#ifndef CONTROL_DEMO_DRIVER__AXIS_MAPPING_HPP_
#define CONTROL_DEMO_DRIVER__AXIS_MAPPING_HPP_

#include <array>

namespace control_demo_driver
{

/// The web UI and the public ROS Joy topic keep the historical order
/// [forward, yaw, strafe, head]. The real SDK/robot order, confirmed by the
/// user's hardware test, is [forward, strafe, yaw, head]: a/d physically turn
/// and q/e physically strafe, opposite to the example comments. Translate only
/// at the SDK boundary.
inline std::array<float, 4> reorder_external_axes(
    float forward, float yaw, float strafe, float head)
{
    return {forward, strafe, yaw, head};
}

}  // namespace control_demo_driver

#endif  // CONTROL_DEMO_DRIVER__AXIS_MAPPING_HPP_
