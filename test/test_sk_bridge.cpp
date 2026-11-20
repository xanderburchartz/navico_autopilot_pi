/***************************************************************************
 *   navico_autopilot_pi: unit tests for the Signal K way
 *   SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   What a Signal K server reports goes into the same PilotState as the
 *   messages from the bus (PilotState::HandleSignalK).
 ***************************************************************************/

#include <cmath>
#include <cstdio>
#include <string>

#include "pilot_state.h"
#include "simnet.h"
#include "sk_bridge.h"

using namespace sk_bridge;

static int failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      failures++;                                                  \
    }                                                              \
  } while (0)

static bool Near(std::optional<double> a, double b) { return a && std::fabs(*a - b) < 0.0002; }

static void TestModes() {
  CHECK(ModeOf("standby", false) == simnet::Mode::kStandby);
  CHECK(ModeOf("heading", true) == simnet::Mode::kAuto);
  CHECK(ModeOf("auto", false) == simnet::Mode::kAuto);
  CHECK(ModeOf("auto", true) == simnet::Mode::kNoDrift);
  CHECK(ModeOf("noDrift", false) == simnet::Mode::kNoDrift);
  CHECK(ModeOf("wind", false) == simnet::Mode::kWind);
  CHECK(ModeOf("route", false) == simnet::Mode::kNav);
  CHECK(ModeOf("dodge", false) == simnet::Mode::kUnknown);
  CHECK(SourceAddress("can0.4") == uint8_t(4));
  CHECK(SourceAddress("simnet-pilot-state.12") == uint8_t(12));
  CHECK(!SourceAddress("simnet-pilot-state"));
  CHECK(!SourceAddress("x.300"));
}

static void TestPilotState() {
  PilotState s;
  double t = 100;
  s.HandleSignalK(4, simnet::Mode::kStandby, std::nullopt, std::nullopt, std::nullopt, t);
  CHECK(s.Alive(t));
  CHECK(s.Address(t) == uint8_t(4));
  CHECK(s.Mode() == simnet::Mode::kStandby);
  CHECK(!s.TargetHeading());

  // AUTO with a heading to steer (magnetic) and a rudder angle.
  s.HandleSignalK(4, ModeOf("heading", true), 1.5708, std::nullopt, -0.05, t + 1);
  CHECK(s.Mode() == simnet::Mode::kAuto);
  CHECK(Near(s.TargetHeading(), 1.5708));
  CHECK(s.TargetMagnetic());
  CHECK(Near(s.Rudder(t + 1), -0.05));

  // A heading reported in standby is not a heading to steer.
  s.HandleSignalK(4, simnet::Mode::kStandby, 1.5708, std::nullopt, std::nullopt, t + 2);
  CHECK(!s.TargetHeading());

  // WIND: the wind angle to hold, negative = port.
  s.HandleSignalK(4, ModeOf("wind", true), 1.5708, -0.7, std::nullopt, t + 3);
  CHECK(s.Mode() == simnet::Mode::kWind);
  CHECK(Near(s.TargetWindAngle(), -0.7));

  // A command is confirmed by the mode the server reports next.
  auto stby = simnet::CommandData(simnet::Command::kStandby, 4);
  s.CommandSent(simnet::Command::kStandby, stby, t + 4);
  s.HandleSignalK(4, simnet::Mode::kStandby, std::nullopt, std::nullopt, std::nullopt, t + 4.5);
  CHECK(!s.Unconfirmed(t + 6));
  CHECK(!s.TargetWindAngle());

  // No report for longer than the timeout: no longer alive.
  CHECK(!s.Alive(t + 4.5 + PilotState::kAliveTimeout + 1));
}

static void TestPuts() {
  auto a = SimnetPut(simnet::Command::kAuto);
  CHECK(a && a->path == "steering/autopilot/actions/command" && a->value == "\"heading\"");
  auto p10 = SimnetPut(simnet::Command::kPort10);
  CHECK(p10 && p10->value == "\"-10\"");
  CHECK(!SimnetPut(simnet::Command::kNoDrift));
  CHECK(!SimnetPut(simnet::Command::kConfirm));
  auto s1 = AutopilotPut(simnet::Command::kStarboard1, std::nullopt);
  CHECK(s1 && s1->path == "steering/autopilot/actions/adjustHeading" && s1->value == "1");
  auto nav = AutopilotPut(simnet::Command::kNav, std::nullopt);
  CHECK(nav && nav->path == "steering/autopilot/state" && nav->value == "\"route\"");
  // Tack: the side from the wind angle held; none without one.
  auto port = AutopilotPut(simnet::Command::kTack, -0.7);
  CHECK(port && port->path == "steering/autopilot/actions/tack" && port->value == "\"port\"");
  auto stb = AutopilotPut(simnet::Command::kTack, 0.7);
  CHECK(stb && stb->value == "\"starboard\"");
  CHECK(!AutopilotPut(simnet::Command::kTack, std::nullopt));
  // A gybe (wind abaft the beam) is not sent: the side would be wrong.
  CHECK(!AutopilotPut(simnet::Command::kTack, -2.6));
  CHECK(!AutopilotPut(simnet::Command::kTack, 2.6));
  CHECK(NfuPut(simnet::NfuSide::kStarboard).value == "\"starboard\"");
  CHECK(NfuPut(simnet::NfuSide::kStop).path == "steering/autopilot/actions/nfu");
}

int main() {
  TestModes();
  TestPilotState();
  TestPuts();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("all tests passed\n");
  return 0;
}
