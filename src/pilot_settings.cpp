/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "pilot_settings.h"

#include <cstdio>
#include <sstream>

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
      // Changed on a Triton2 while listening (13 Nov 2026): Apparent 0x19,
      // True 0x1A, Auto 0x1C; before that the pilot had 0x04.
      {{0x1C, 0x09}, Node::kPilot, "Sailing", "Wind mode", "", 1, false, 1,
       {"", "Apparent", "True", "", "Auto"}, nullptr, 0, false, 0x07, 0x18},
      {{0x1C, 0x08}, Node::kPilot, "Sailing", "Tack time", "s", 2, false, 0.1, {}},
      {{0x18, 0x0B}, Node::kPilot, "Sailing", "Tack angle", "deg", 2, false, 360.0 / 65536.0, {}, nullptr, 0},
      // Triton2 "Manual speed" (13 Nov 2026: 5 kn sent as 257).
      {{0x18, 0x91}, Node::kPilot, "Sailing", "Manual speed", "kn", 2, false, 0.01 * kMsToKn, {}},
      // Per steering profile (instance), in the order of the Triton2 menu
      // (13 Nov 2026). The "Low" of other sources in group 19 does not
      // answer on this NAC-3. Autotune changed gain and counter rudder of
      // profile 0 only (15 Oct 2026).
      {{0x1A, 0x10}, Node::kPilot, "Steering profiles", "Turn rate", "deg/min", 4, false,
       3.125e-8 * kRadToDeg * 60.0, {}, nullptr, 0, true},
      {{0x1A, 0x0D}, Node::kPilot, "Steering profiles", "Rudder gain", "", 2, false, 0.01, {},
       nullptr, 2, true},
      {{0x1A, 0x0F}, Node::kPilot, "Steering profiles", "Counter rudder", "", 2, false, 0.01, {},
       nullptr, 2, true},
      {{0x1A, 0x0E}, Node::kPilot, "Steering profiles", "Autotrim", "s", 2, false, 0.1, {},
       nullptr, 1, true},
      {{0x18, 0x79}, Node::kPilot, "Steering profiles", "Init rudder", "", 1, false, 1,
       {"Actual", "Midships"}, nullptr, 0, true},
      {{0x18, 0x7A}, Node::kPilot, "Steering profiles", "Rudder limit", "deg", 2, false,
       0.0001 * kRadToDeg, {}, nullptr, 0, true},
      // A tick box on the Triton2: off sends 0, on sends the lowest value.
      {{0x18, 0x78}, Node::kPilot, "Steering profiles", "Off heading limit", "deg", 2, false,
       0.0001 * kRadToDeg, {}, "Off", 0, true},
      {{0x1B, 0x11}, Node::kPilot, "Steering profiles", "Track response", "", 1, false, 1, {},
       nullptr, 0, true},
      {{0x1B, 0x81}, Node::kPilot, "Steering profiles", "Track approach angle", "deg", 2, false,
       0.0001 * kRadToDeg, {}, nullptr, 0, true},
      {{0x1B, 0x0C}, Node::kPilot, "Steering profiles", "Course change confirm angle", "deg", 2,
       false, 0.0001 * kRadToDeg, {}, "Off", 0, true},
      // Recorded on Penny Lane (13 Nov 2026, Triton2 set and NAC-3 reply).
      {{0x14, 0x06}, Node::kPilot, "Drive", "Rudder deadband", "deg", 2, false,
       0.0001 * kRadToDeg, {}, "Auto"},
      {{0x14, 0x1D}, Node::kPilot, "Drive", "Drive voltage", "V", 2, false, 0.01, {}},
      // Changed on a Triton2 while listening (13 Nov 2026): Auto sets 14/0061
      // to 1 and bits 5/6 of 14/0009; Clutch sets both back.
      {{0x14, 0x61}, Node::kPilot, "Drive", "Drive engage", "", 1, false, 1, {"Clutch", "Auto"}},
      {{0x14, 0x63}, Node::kPilot, "Drive", "Minimum rudder", "deg", 2, false, 0.0001 * kRadToDeg, {}},
      {{0x14, 0x02}, Node::kPilot, "Drive", "Minimum power", "%", 2, false, 1, {}, nullptr, 0},
      {{0x14, 0x01}, Node::kPilot, "Drive", "Maximum power", "%", 2, false, 1, {}, nullptr, 0},
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
      {{0x18, 0x91}, {0, 99}},      // manual speed, kn (two digits on a Triton2)
      {{0x1C, 0x08}, {2, 60}},      // tack time, s
      {{0x1A, 0x0D}, {0.05, 4}},    // rudder gain
      {{0x1A, 0x0F}, {0, 4}},       // counter rudder
      {{0x1A, 0x0E}, {5, 400}},     // autotrim, s
      {{0x1A, 0x10}, {30, 720}},    // turn rate, deg/min
      {{0x14, 0x06}, {0, 5}},       // rudder deadband, deg (0 = Auto)
      {{0x14, 0x63}, {0, 5}},       // minimum rudder, deg
      {{0x14, 0x02}, {0, 50}},      // minimum power, %
      {{0x14, 0x01}, {50, 100}},    // maximum power, %
      {{0x18, 0x7A}, {10, 45}},     // rudder limit, deg
      {{0x18, 0x78}, {0, 90}},      // off heading limit, deg (0 = off)
      {{0x1B, 0x81}, {5, 90}},      // track approach angle, deg
      {{0x1B, 0x0C}, {0, 90}},      // course change confirm angle, deg (0 = off)
      {{0x1C, 0x09}, {1, 4}},       // wind mode (a choice)
      {{0x18, 0x79}, {0, 1}},       // init rudder (a choice)
  };
  for (const auto& e : list)
    if (e.key == key) return e.limits;
  return std::nullopt;
}

const std::vector<Profile>& Profiles() {
  static const std::vector<Profile> list = {{0, "Low speed"}, {1, "High speed"}};
  return list;
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

const Definition* Find(simnet::SettingKey key) {
  for (const auto& d : All())
    if (d.key == key) return &d;
  return nullptr;
}

std::string Serialize(const Snapshot& s) {
  std::string out = "# navico_autopilot_pi snapshot of the pilot settings\n";
  out += "time=" + s.time + "\n";
  out += "reason=" + s.reason + "\n";
  char line[64];
  for (const auto& e : s.entries) {
    std::snprintf(line, sizeof line, "%s %02X/%04X %02X %u\n",
                  e.node == Node::kPilot ? "pilot" : "rudder", e.key.group, e.key.param, e.instance,
                  e.raw);
    out += line;
  }
  return out;
}

std::optional<Snapshot> ParseSnapshot(const std::string& text) {
  Snapshot s;
  std::istringstream in(text);
  std::string line;
  bool header = false;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.rfind("# navico_autopilot_pi snapshot", 0) == 0) {
      header = true;
      continue;
    }
    if (line.rfind("time=", 0) == 0) {
      s.time = line.substr(5);
      continue;
    }
    if (line.rfind("reason=", 0) == 0) {
      s.reason = line.substr(7);
      continue;
    }
    char node[16];
    unsigned group, param, instance, raw;
    if (std::sscanf(line.c_str(), "%15s %x/%x %x %u", node, &group, &param, &instance, &raw) != 5)
      continue;
    std::string n(node);
    if ((n != "pilot" && n != "rudder") || group > 0xFF || param > 0xFFFF || instance > 0xFF)
      continue;
    s.entries.push_back({n == "pilot" ? Node::kPilot : Node::kRudder,
                         {static_cast<uint8_t>(group), static_cast<uint16_t>(param)},
                         static_cast<uint8_t>(instance), static_cast<uint32_t>(raw)});
  }
  if (!header) return std::nullopt;
  return s;
}

uint32_t Encode(const Definition& d, double shown) {
  if (d.mask != 0xFFFFFFFF)
    return (static_cast<uint32_t>(shown + 0.5) & d.mask) | d.set_bits;
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
  *raw &= d.mask;
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
