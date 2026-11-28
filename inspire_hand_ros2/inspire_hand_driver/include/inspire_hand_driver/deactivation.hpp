#pragma once

#include <chrono>

namespace inspire_hand_driver::deactivation
{
// Nominal unloaded full-stroke time, not confirmation of fingertip motion.
constexpr auto open_dwell = std::chrono::milliseconds(800);
} // namespace inspire_hand_driver::deactivation
