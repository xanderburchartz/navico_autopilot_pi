/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "sk_bridge.h"

#include <cctype>
#include <cmath>
#include <cstdlib>

namespace sk_bridge {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string Lower(std::string s) {
  for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

/** Unsigned angle 0..2pi in 0.0001 rad, as Simnet and 127237 carry it. */
uint16_t Angle(double rad) {
  double a = std::fmod(rad, 2 * kPi);
  if (a < 0) a += 2 * kPi;
  long raw = std::lround(a * 10000);
  if (raw >= 0xFFFD) raw = 0;
  return static_cast<uint16_t>(raw);
}

simnet::Message Msg(int pgn, uint8_t source, std::vector<uint8_t> data) {
  simnet::Message m;
  m.pgn = pgn;
  m.source = source;
  m.destination = 255;
  m.data = std::move(data);
  return m;
}

}  // namespace

simnet::Mode ModeOf(const std::string& state, bool auto_is_nodrift) {
  std::string s = Lower(state);
  if (s == "standby") return simnet::Mode::kStandby;
  if (s == "heading") return simnet::Mode::kAuto;
  if (s == "nodrift") return simnet::Mode::kNoDrift;
  if (s == "auto") return auto_is_nodrift ? simnet::Mode::kNoDrift : simnet::Mode::kAuto;
  if (s == "wind") return simnet::Mode::kWind;
  if (s == "route" || s == "track" || s == "nav") return simnet::Mode::kNav;
  return simnet::Mode::kUnknown;
}

std::optional<uint8_t> SourceAddress(const std::string& source) {
  size_t dot = source.rfind('.');
  std::string tail = dot == std::string::npos ? source : source.substr(dot + 1);
  if (tail.empty() || tail.size() > 3) return std::nullopt;
  for (char c : tail)
    if (!std::isdigit(static_cast<unsigned char>(c))) return std::nullopt;
  long v = std::strtol(tail.c_str(), nullptr, 10);
  if (v < 0 || v > 251) return std::nullopt;
  return static_cast<uint8_t>(v);
}

std::vector<simnet::Message> ToMessages(const Status& s, uint8_t address, bool auto_is_nodrift) {
  std::vector<simnet::Message> out;
  simnet::Mode mode = simnet::Mode::kUnknown;
  if (s.state) {
    mode = ModeOf(*s.state, auto_is_nodrift);
    uint16_t bits = 0;
    switch (mode) {
      case simnet::Mode::kStandby: bits = 1 << 3; break;
      case simnet::Mode::kAuto: bits = 1 << 4; break;
      case simnet::Mode::kNav: bits = 1 << 6; break;
      case simnet::Mode::kNoDrift: bits = 1 << 8; break;
      case simnet::Mode::kWind: bits = 1 << 10; break;
      default: break;
    }
    if (bits) {
      // 65305, report 0x0A "Mode", model 0x64 (NAC), as a NAC-3 sends it.
      out.push_back(Msg(simnet::kPgnDeviceStatus, address,
                        {0x41, 0x9F, 0x64, 0x0A, static_cast<uint8_t>(bits & 0xFF),
                         static_cast<uint8_t>(bits >> 8), 0x00, 0x00}));
    }
  }
  // 127237: heading to steer at bytes 5-6, magnetic (bits 6-7 of byte 1 = 1).
  // In standby there is none.
  {
    bool none = mode == simnet::Mode::kStandby || !s.target_heading;
    uint16_t raw = none ? 0xFFFF : Angle(*s.target_heading);
    if (s.state || s.target_heading) {
      std::vector<uint8_t> d(21, 0xFF);
      d[1] = 0x7F;  // bits 6-7 = 01: magnetic
      d[5] = static_cast<uint8_t>(raw & 0xFF);
      d[6] = static_cast<uint8_t>(raw >> 8);
      out.push_back(Msg(simnet::kPgnHeadingControl, address, d));
    }
  }
  // 65341 with mode 3 (wind): the wind angle to hold.
  if (mode == simnet::Mode::kWind && s.target_wind) {
    uint16_t raw = Angle(*s.target_wind);
    out.push_back(Msg(simnet::kPgnApAngle, address,
                      {0x41, 0x9F, 0xFF, 0xFF, 0x03, 0xFF, static_cast<uint8_t>(raw & 0xFF),
                       static_cast<uint8_t>(raw >> 8)}));
  }
  // 127245: rudder position (signed, 0.0001 rad) at bytes 4-5.
  if (s.rudder) {
    long raw = std::lround(*s.rudder * 10000);
    if (raw > 32000) raw = 32000;
    if (raw < -32000) raw = -32000;
    auto r = static_cast<uint16_t>(static_cast<int16_t>(raw));
    out.push_back(Msg(simnet::kPgnRudder, address,
                      {0x00, 0xFF, 0xFF, 0xFF, static_cast<uint8_t>(r & 0xFF),
                       static_cast<uint8_t>(r >> 8), 0xFF, 0xFF}));
  }
  return out;
}

std::optional<Put> SimnetPut(simnet::Command c) {
  const std::string path = "steering/autopilot/actions/command";
  switch (c) {
    case simnet::Command::kStandby: return Put{path, "\"standby\""};
    case simnet::Command::kAuto: return Put{path, "\"heading\""};
    case simnet::Command::kWind: return Put{path, "\"wind\""};
    case simnet::Command::kNav: return Put{path, "\"nav\""};
    case simnet::Command::kTack: return Put{path, "\"tack\""};
    case simnet::Command::kPort1: return Put{path, "\"-1\""};
    case simnet::Command::kPort10: return Put{path, "\"-10\""};
    case simnet::Command::kStarboard1: return Put{path, "\"+1\""};
    case simnet::Command::kStarboard10: return Put{path, "\"+10\""};
    default: return std::nullopt;  // No Drift, Confirm
  }
}

std::optional<Put> AutopilotPut(simnet::Command c) {
  const std::string state = "steering/autopilot/state";
  const std::string adjust = "steering/autopilot/actions/adjustHeading";
  switch (c) {
    case simnet::Command::kStandby: return Put{state, "\"standby\""};
    case simnet::Command::kAuto: return Put{state, "\"auto\""};
    case simnet::Command::kWind: return Put{state, "\"wind\""};
    case simnet::Command::kNav: return Put{state, "\"route\""};
    case simnet::Command::kNoDrift: return Put{state, "\"noDrift\""};
    case simnet::Command::kTack: return Put{"steering/autopilot/actions/tack", "\"port\""};
    case simnet::Command::kPort1: return Put{adjust, "-1"};
    case simnet::Command::kPort10: return Put{adjust, "-10"};
    case simnet::Command::kStarboard1: return Put{adjust, "1"};
    case simnet::Command::kStarboard10: return Put{adjust, "10"};
    default: return std::nullopt;  // Confirm
  }
}

Put NfuPut(simnet::NfuSide side) {
  const std::string path = "steering/autopilot/actions/nfu";
  switch (side) {
    case simnet::NfuSide::kPort: return Put{path, "\"port\""};
    case simnet::NfuSide::kStarboard: return Put{path, "\"starboard\""};
    case simnet::NfuSide::kStop:
    default: return Put{path, "\"stop\""};
  }
}

}  // namespace sk_bridge
