/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "pilot_settings.h"

namespace pilot_settings {

namespace {

constexpr double kRadToDeg = 57.29577951308232;
constexpr double kMsToKn = 3600.0 / 1852.0;

}  // namespace

const std::vector<Definition>& All() {
  // Group 18 general, 19 profile Low, 1A profile High, 1C sailing, 09 on the
  // rudder feedback node. Confirmed by the signalk-navico-autopilot-bridge
  // author by changing them on a display and listening.
  static const std::vector<Definition> list = {
      {{0x18, 0x0A}, Node::kPilot, "Boat", "Boat type", "", 1, false, 1,
       {"Sail", "Outboard", "Displacement", "Planing"}},
      {{0x18, 0x0C}, Node::kPilot, "Boat", "Transition speed", "kn", 2, false, 0.01 * kMsToKn, {}},
      {{0x18, 0x0B}, Node::kPilot, "Sailing", "Tack angle", "deg", 2, false, 360.0 / 65536.0, {}},
      {{0x1C, 0x08}, Node::kPilot, "Sailing", "Tack time", "s", 2, false, 0.1, {}},
      {{0x19, 0x0D}, Node::kPilot, "Profile Low", "Rudder gain", "", 2, false, 0.01, {}},
      {{0x19, 0x0F}, Node::kPilot, "Profile Low", "Counter rudder", "", 2, false, 0.01, {}},
      {{0x19, 0x0E}, Node::kPilot, "Profile Low", "Autotrim", "s", 2, false, 0.1, {}},
      {{0x19, 0x10}, Node::kPilot, "Profile Low", "Turn rate", "deg/min", 4, false,
       3.125e-8 * kRadToDeg * 60.0, {}},
      {{0x1A, 0x0D}, Node::kPilot, "Profile High", "Rudder gain", "", 2, false, 0.01, {}},
      {{0x1A, 0x0F}, Node::kPilot, "Profile High", "Counter rudder", "", 2, false, 0.01, {}},
      {{0x1A, 0x0E}, Node::kPilot, "Profile High", "Autotrim", "s", 2, false, 0.1, {}},
      {{0x1A, 0x10}, Node::kPilot, "Profile High", "Turn rate", "deg/min", 4, false,
       3.125e-8 * kRadToDeg * 60.0, {}},
      // Recorded on Penny Lane (13 Nov 2026, Triton2 set and NAC-3 reply).
      {{0x14, 0x06}, Node::kPilot, "Drive", "Rudder deadband", "deg", 2, false,
       0.0001 * kRadToDeg, {}, "Auto"},
      {{0x09, 0x01}, Node::kRudder, "Rudder", "Calibration, starboard", "deg", 2, true,
       0.0001 * kRadToDeg, {}},
      {{0x09, 0x02}, Node::kRudder, "Rudder", "Calibration, port", "deg", 2, true,
       0.0001 * kRadToDeg, {}},
  };
  return list;
}

std::optional<double> Decode(const Definition& d, const simnet::KeyValue& kv) {
  auto raw = kv.Unsigned(static_cast<size_t>(d.bytes));
  if (!raw) return std::nullopt;
  double v = *raw;
  if (d.is_signed) {
    if (d.bytes == 1) v = static_cast<int8_t>(*raw);
    if (d.bytes == 2) v = static_cast<int16_t>(*raw);
    if (d.bytes == 4) v = static_cast<int32_t>(*raw);
  }
  if (!d.choices.empty()) return v;
  return v * d.scale;
}

}  // namespace pilot_settings
