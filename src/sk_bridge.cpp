/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "sk_bridge.h"

#include <cctype>
#include <cstdlib>

namespace sk_bridge {

namespace {

std::string Lower(std::string s) {
  for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
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

std::optional<Put> AutopilotPut(simnet::Command c, std::optional<double> wind) {
  const std::string state = "steering/autopilot/state";
  const std::string adjust = "steering/autopilot/actions/adjustHeading";
  switch (c) {
    case simnet::Command::kStandby: return Put{state, "\"standby\""};
    case simnet::Command::kAuto: return Put{state, "\"auto\""};
    case simnet::Command::kWind: return Put{state, "\"wind\""};
    case simnet::Command::kNav: return Put{state, "\"route\""};
    case simnet::Command::kNoDrift: return Put{state, "\"noDrift\""};
    case simnet::Command::kTack:
      // The Simrad provider lets the pilot choose the side; others need it.
      if (!wind) return std::nullopt;
      return Put{"steering/autopilot/actions/tack", *wind < 0 ? "\"port\"" : "\"starboard\""};
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
