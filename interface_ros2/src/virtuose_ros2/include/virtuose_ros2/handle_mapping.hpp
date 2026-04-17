#pragma once

#include <cstdint>

namespace virtuose_ros2
{
namespace handle_mapping
{

// Topic names for handle-related outputs.
static constexpr const char * kHandleAnalogTopic = "out_handle_analog";
static constexpr const char * kButtonATopic = "out_fsm_trigger/button_a";
static constexpr const char * kButtonBTopic = "out_fsm_trigger/button_b";
static constexpr const char * kTriggerATopic = "out_fsm_trigger/trigger_a";
static constexpr const char * kTriggerBTopic = "out_fsm_trigger/trigger_b";

// Bit masks for virtGetTorInputs() value.
// Adjust these values per device model if wiring differs.
static constexpr uint32_t kButtonAMask = 0x000020U;
static constexpr uint32_t kButtonBMask = 0x000040U;
static constexpr uint32_t kTriggerAMask = 0x000004U;
static constexpr uint32_t kTriggerBMask = 0x000008U;

}  // namespace handle_mapping
}  // namespace virtuose_ros2
