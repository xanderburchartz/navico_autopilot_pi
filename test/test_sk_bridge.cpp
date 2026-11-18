/***************************************************************************
 *   navico_autopilot_pi: unit tests for the Signal K way
 *   SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   What a Signal K server reports goes through the same PilotState as the
 *   messages from the bus; these tests feed it the converted messages.
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

static void Feed(PilotState& s, const Status& st, double now, bool auto_is_nodrift = false) {
  for (const auto& m : ToMessages(st, st.source.value_or(0), auto_is_nodrift)) s.Handle(m, now);
}

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
  Status st;
  st.state = "standby";
  st.source = 4;
  Feed(s, st, t);
  CHECK(s.Alive(t));
  CHECK(s.Address(t) == uint8_t(4));
  CHECK(s.Mode() == simnet::Mode::kStandby);
  CHECK(!s.TargetHeading());

  // AUTO with a heading to steer (magnetic) and a rudder angle.
  st.state = "heading";
  st.target_heading = 1.5708;  // 90 degrees
  st.rudder = -0.05;           // port
  Feed(s, st, t + 1);
  CHECK(s.Mode() == simnet::Mode::kAuto);
  CHECK(Near(s.TargetHeading(), 1.5708));
  CHECK(s.TargetMagnetic());
  CHECK(Near(s.Rudder(t + 1), -0.05));

  // WIND: the wind angle to hold, negative = port.
  st.state = "wind";
  st.target_wind = -0.7;
  Feed(s, st, t + 2);
  CHECK(s.Mode() == simnet::Mode::kWind);
  CHECK(Near(s.TargetWindAngle(), -0.7));

  // "auto" from signalk-simnet-pilot-state is No Drift.
  st.state = "auto";
  Feed(s, st, t + 3, true);
  CHECK(s.Mode() == simnet::Mode::kNoDrift);

  // No report for longer than the timeout: no longer alive.
  CHECK(!s.Alive(t + 3 + PilotState::kAliveTimeout + 1));
}

static void TestPuts() {
  auto a = SimnetPut(simnet::Command::kAuto);
  CHECK(a && a->path == "steering/autopilot/actions/command" && a->value == "\"heading\"");
  auto p10 = SimnetPut(simnet::Command::kPort10);
  CHECK(p10 && p10->value == "\"-10\"");
  CHECK(!SimnetPut(simnet::Command::kNoDrift));
  CHECK(!SimnetPut(simnet::Command::kConfirm));
  auto s1 = AutopilotPut(simnet::Command::kStarboard1);
  CHECK(s1 && s1->path == "steering/autopilot/actions/adjustHeading" && s1->value == "1");
  auto nav = AutopilotPut(simnet::Command::kNav);
  CHECK(nav && nav->path == "steering/autopilot/state" && nav->value == "\"route\"");
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
