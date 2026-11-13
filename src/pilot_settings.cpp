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
  // Group 18 general, 1A profile High, 1C sailing, 14 drive, 09 on the
  // rudder feedback node. Confirmed by the signalk-navico-autopilot-bridge
  // author by changing them on a display and listening.
  static const std::vector<Definition> list = {
      {{0x18, 0x0A}, Node::kPilot, "Boat", "Boat type", "", 1, false, 1,
       {"Sail", "Outboard", "Displacement", "Planing"}},
      // Measured on Penny Lane (13 Nov 2026), checked against the Triton2 menu.
      {{0x18, 0x56}, Node::kPilot, "Boat", "Boat length", "m", 2, false, 0.1, {}},
      {{0x18, 0x7F}, Node::kPilot, "Boat", "Cruising speed", "kn", 2, false, 0.01 * kMsToKn, {}},
      {{0x18, 0x0C}, Node::kPilot, "Boat", "Transition speed", "kn", 2, false, 0.01 * kMsToKn, {}},
      {{0x18, 0x57}, Node::kPilot, "Boat", "Low boat speed alarm below", "kn", 2, false,
       0.01 * kMsToKn, {}},
      {{0x18, 0x0B}, Node::kPilot, "Sailing", "Tack angle", "deg", 2, false, 360.0 / 65536.0, {}, nullptr, 0},
      {{0x1C, 0x08}, Node::kPilot, "Sailing", "Tack time", "s", 2, false, 0.1, {}},
      // Per steering profile (instance); the "Low/High" of other pilots is
      // not group 19 on this NAC-3 (no answer). Autotune changed gain and
      // counter rudder of the active profile only (15 Oct 2026).
      {{0x1A, 0x0D}, Node::kPilot, "Steering profiles", "Rudder gain", "", 2, false, 0.01, {},
       nullptr, 2, true},
      {{0x1A, 0x0F}, Node::kPilot, "Steering profiles", "Counter rudder", "", 2, false, 0.01, {},
       nullptr, 2, true},
      {{0x1A, 0x0E}, Node::kPilot, "Steering profiles", "Autotrim", "s", 2, false, 0.1, {},
       nullptr, 1, true},
      {{0x1A, 0x10}, Node::kPilot, "Steering profiles", "Turn rate", "deg/min", 4, false,
       3.125e-8 * kRadToDeg * 60.0, {}, nullptr, 0, true},
      // Recorded on Penny Lane (13 Nov 2026, Triton2 set and NAC-3 reply).
      {{0x14, 0x06}, Node::kPilot, "Drive", "Rudder deadband", "deg", 2, false,
       0.0001 * kRadToDeg, {}, "Auto"},
      {{0x14, 0x1D}, Node::kPilot, "Drive", "Drive voltage", "V", 2, false, 0.01, {}},
      {{0x14, 0x02}, Node::kPilot, "Drive", "Minimum power", "%", 2, false, 1, {}, nullptr, 0},
      {{0x14, 0x01}, Node::kPilot, "Drive", "Maximum power", "%", 2, false, 1, {}, nullptr, 0},
      {{0x18, 0x7A}, Node::kPilot, "Drive", "Rudder limit", "deg", 2, false, 0.0001 * kRadToDeg, {}, nullptr, 0},
      {{0x09, 0x01}, Node::kRudder, "Rudder", "Calibration, starboard", "deg", 2, true,
       0.0001 * kRadToDeg, {}},
      {{0x09, 0x02}, Node::kRudder, "Rudder", "Calibration, port", "deg", 2, true,
       0.0001 * kRadToDeg, {}},
  };
  return list;
}

std::optional<Limits> Editable(simnet::SettingKey key) {
  // Wide guard rails of the plugin itself; the pilot applies its own limits
  // and rounds (the deadband 0.1 deg came back as 0.0016 rad).
  struct Entry {
    simnet::SettingKey key;
    Limits limits;
  };
  static const Entry list[] = {
      {{0x18, 0x56}, {3, 60}},      // boat length, m
      {{0x18, 0x7F}, {1, 40}},      // cruising speed, kn
      {{0x18, 0x0C}, {1, 40}},      // transition speed, kn
      {{0x18, 0x57}, {0, 10}},      // low boat speed alarm, kn
      {{0x18, 0x0B}, {50, 150}},    // tack angle, deg
      {{0x1C, 0x08}, {2, 60}},      // tack time, s
      {{0x1A, 0x0D}, {0.05, 4}},    // rudder gain
      {{0x1A, 0x0F}, {0, 4}},       // counter rudder
      {{0x1A, 0x0E}, {5, 400}},     // autotrim, s
      {{0x1A, 0x10}, {30, 720}},    // turn rate, deg/min
      {{0x14, 0x06}, {0, 5}},       // rudder deadband, deg (0 = Auto)
      {{0x14, 0x02}, {0, 50}},      // minimum power, %
      {{0x14, 0x01}, {50, 100}},    // maximum power, %
      {{0x18, 0x7A}, {10, 45}},     // rudder limit, deg
  };
  for (const auto& e : list)
    if (e.key == key) return e.limits;
  return std::nullopt;
}

std::string Text(const simnet::KeyValue& kv) {
  std::string t;
  for (uint8_t c : kv.value) {
    if (c == 0) break;
    t += static_cast<char>(c);
  }
  while (!t.empty() && (t.back() == ' ' || t.back() == '@')) t.pop_back();
  return t;
}

uint32_t Encode(const Definition& d, double shown) {
  double raw = d.choices.empty() ? shown / d.scale : shown;
  if (raw < 0) {
    int32_t s = static_cast<int32_t>(raw - 0.5);
    return static_cast<uint32_t>(s) & (d.bytes == 4 ? 0xFFFFFFFFu : d.bytes == 2 ? 0xFFFFu : 0xFFu);
  }
  return static_cast<uint32_t>(raw + 0.5);
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
