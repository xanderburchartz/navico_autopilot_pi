/***************************************************************************
 *   navico_autopilot_pi: unit tests for simnet and pilot_state
 *   SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   Expected bytes come from recordings of a real bus (B&G Triton2 keypad,
 *   NAC-3 at address 4, commands via a Yacht Devices gateway that the NAC-3
 *   followed). Build and run: see test/CMakeLists.txt or README.md.
 ***************************************************************************/

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "pilot_settings.h"
#include "pilot_state.h"
#include "simnet.h"

using namespace simnet;

static int failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      failures++;                                                  \
    }                                                              \
  } while (0)

static std::vector<uint8_t> Bytes(const std::string& hex) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < hex.size();) {
    if (hex[i] == ' ') { i++; continue; }
    out.push_back(static_cast<uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    i += 2;
  }
  return out;
}

static void CheckEq(const std::vector<uint8_t>& got, const std::string& want,
                    const char* what) {
  if (got != Bytes(want)) {
    std::printf("FAIL %s: got %s, want %s\n", what, Hex(got).c_str(), want.c_str());
    failures++;
  }
}

// A received message as OpenCPN hands it to a plugin (0x93 layout).
static Message Rx(int pgn, uint8_t src, const std::string& hex) {
  std::vector<uint8_t> d = Bytes(hex);
  std::vector<uint8_t> v = {0x93, static_cast<uint8_t>(d.size() + 11), 2,
                            static_cast<uint8_t>(pgn & 0xFF),
                            static_cast<uint8_t>((pgn >> 8) & 0xFF),
                            static_cast<uint8_t>((pgn >> 16) & 0xFF),
                            255, src, 0xFF, 0xFF, 0xFF, 0xFF,
                            static_cast<uint8_t>(d.size())};
  v.insert(v.end(), d.begin(), d.end());
  v.push_back(0x55);
  auto m = ParseOcpnPayload(v);
  CHECK(m.has_value());
  return *m;
}

static void TestCommands() {
  // Recording 16 Oct 2026, 07:05 (opname-signalk-130850-20261016.log): fast
  // packets 09FF2264 "40 0B 41 9F 04 FF FF 0A" + "41 09 00 FF FF FF FF FF".
  CheckEq(CommandData(Command::kAuto, 4), "41 9F 04 FF FF 0A 09 00 FF FF FF", "auto");
  CheckEq(CommandData(Command::kStandby, 4), "41 9F 04 FF FF 0A 06 00 FF FF FF", "standby");
  CheckEq(CommandData(Command::kWind, 4), "41 9F 04 FF FF 0A 0F 00 FF FF FF", "wind");
  // "40 0C 41 9F 04 FF FF 0A" + "41 1A 00 03 D1 06 FF FF": +10
  CheckEq(CommandData(Command::kStarboard10, 4), "41 9F 04 FF FF 0A 1A 00 03 D1 06 FF", "+10");
  CheckEq(CommandData(Command::kPort10, 4), "41 9F 04 FF FF 0A 1A 00 02 D1 06 FF", "-10");
  // "41 1A 00 03 AF 00 FF FF": +1
  CheckEq(CommandData(Command::kStarboard1, 4), "41 9F 04 FF FF 0A 1A 00 03 AF 00 FF", "+1");
  CheckEq(CommandData(Command::kPort1, 4), "41 9F 04 FF FF 0A 1A 00 02 AF 00 FF", "-1");
  // Not recorded; canboat events 10 (nav) and 12 (no drift).
  CheckEq(CommandData(Command::kNav, 4), "41 9F 04 FF FF 0A 0A 00 FF FF FF", "nav");
  CheckEq(CommandData(Command::kNoDrift, 7), "41 9F 07 FF FF 0A 0C 00 FF FF FF", "nodrift");
  // Byte 7 must be 0x00 for every command.
  for (int c = 0; c <= static_cast<int>(Command::kStarboard10); c++)
    CHECK(CommandData(static_cast<Command>(c), 4)[7] == 0x00);
}

static void TestParse() {
  // 65305 from the NAC-3 (04FF1904), recording 16 Oct 2026.
  auto standby = ParsePilotMode(Bytes("41 9F 64 0A 08 00 00 00"));
  CHECK(standby && standby->mode == Mode::kStandby && standby->model == 0x64);
  auto heading = ParsePilotMode(Bytes("41 9F 64 0A 10 00 00 00"));
  CHECK(heading && heading->mode == Mode::kAuto);
  auto wind = ParsePilotMode(Bytes("41 9F 64 0A 00 04 00 00"));
  CHECK(wind && wind->mode == Mode::kWind);
  // From the TwoCan notes and canboat, not recorded here.
  CHECK(ParsePilotMode(Bytes("41 9F 64 0A 40 00 00 40"))->mode == Mode::kNav);
  CHECK(ParsePilotMode(Bytes("41 9F 64 0A 00 01 00 00"))->mode == Mode::kNoDrift);
  // The keypad (0CFF1902) sends requests with report 0x0B and 0x03: not a pilot.
  CHECK(!ParsePilotMode(Bytes("41 9F 64 0B 00 00 00 00")));
  CHECK(!ParsePilotMode(Bytes("41 9F 64 03 04 00 00 00")));
  // Status report.
  CHECK(ParsePilotEngaged(Bytes("41 9F 64 02 02 00 00 00")) == false);
  CHECK(ParsePilotEngaged(Bytes("41 9F 64 02 10 00 00 00")) == true);
  CHECK(!ParsePilotEngaged(Bytes("41 9F 64 02 06 00 00 00")).has_value());

  // 127237: true reference, target 0x5BB5 = 2.3477 rad.
  auto hc = ParseHeadingControl(Bytes("00 3F FF FF 7F B5 5B FF FF FF FF FF FF FF FF FF FF FF FF FF FF"));
  CHECK(hc && hc->target && std::fabs(*hc->target - 2.3477) < 1e-6 && !hc->magnetic);
  auto hc_standby = ParseHeadingControl(Bytes("00 7F FF FF 7F FF FF FF"));
  CHECK(hc_standby && !hc_standby->target && hc_standby->magnetic);

  // 65341, wind mode, 0x2710 = 1.0 rad.
  auto aa = ParseApAngle(Bytes("41 9F FF FF 03 FF 10 27"));
  CHECK(aa && aa->mode == 3 && aa->angle && std::fabs(*aa->angle - 1.0) < 1e-9);

  // 60928 of the NAC-3 autopilot: manufacturer 275, function 150.
  uint64_t raw = 1907005ull | (275ull << 21) | (150ull << 40) | (40ull << 49) | (4ull << 60) | (1ull << 63);
  std::vector<uint8_t> claim;
  for (int i = 0; i < 8; i++) claim.push_back((raw >> (8 * i)) & 0xFF);
  auto n = ParseAddressClaim(claim);
  CHECK(n && n->manufacturer == 275 && n->function == 150 && n->unique == 1907005 && n->IsAutopilot());

  // Own transmission looped back (0x94 layout).
  std::vector<uint8_t> own = {0x94, 0x13, 2, 0x22, 0xFF, 0x01, 255, 255, 0x41, 0x9F, 0x55};
  auto m = ParseOcpnPayload(own);
  CHECK(m && m->own && m->pgn == 130850 && m->data.size() == 2);
}

static void TestPilotState() {
  PilotState s;
  double t = 100;
  CHECK(!s.Address(t));
  // Keypad at 2 sends requests, NAC-3 at 4 reports.
  s.Handle(Rx(65305, 2, "41 9F 64 0B 00 00 00 00"), t);
  CHECK(!s.Address(t));
  s.Handle(Rx(65305, 4, "41 9F 64 0A 08 00 00 00"), t);
  CHECK(s.Address(t) == 4 && s.Alive(t) && s.Mode() == Mode::kStandby);

  // AUTO confirmed by the next report.
  s.CommandSent(Command::kAuto, CommandData(Command::kAuto, 4), t);
  s.Handle(Rx(65305, 4, "41 9F 64 0A 10 00 00 00"), t + 0.03);
  CHECK(s.Mode() == Mode::kAuto && !s.Unconfirmed(t + 2));

  // +10 confirmed by a new target in 127237.
  s.Handle(Rx(127237, 4, "00 3F FF FF 7F 00 50 FF FF FF FF FF FF FF FF FF FF FF FF FF FF"), t + 0.1);
  s.CommandSent(Command::kStarboard10, CommandData(Command::kStarboard10, 4), t + 0.2);
  CHECK(!s.Unconfirmed(t + 0.5));
  s.Handle(Rx(127237, 4, "00 3F FF FF 7F D1 56 FF FF FF FF FF FF FF FF FF FF FF FF FF FF"), t + 0.3);
  CHECK(!s.Unconfirmed(t + 3));

  // No answer: unconfirmed after the timeout.
  s.CommandSent(Command::kWind, CommandData(Command::kWind, 4), t + 4);
  CHECK(!s.Unconfirmed(t + 4.5));
  CHECK(s.Unconfirmed(t + 6));

  // Silence: not alive after kAliveTimeout (10 s).
  CHECK(s.Alive(t + 9) && !s.Alive(t + 11));

  // Second pilot (e.g. an emulated AC12): the most likely one is used (NAC
  // before AC); the user can choose another.
  double u = 200;
  s.Handle(Rx(65305, 4, "41 9F 64 0A 08 00 00 00"), u);
  s.Handle(Rx(65305, 6, "41 9F 00 0A 08 00 00 00"), u);
  CHECK(s.Address(u) == 4);
  s.Handle(Rx(65305, 6, "41 9F 00 0A 10 00 00 00"), u + 0.1);
  CHECK(s.Mode() == Mode::kStandby);  // reports of 6 do not count
  s.SetFixedAddress(6);
  s.Handle(Rx(65305, 6, "41 9F 00 0A 10 00 00 00"), u + 0.2);
  CHECK(s.Address(u) == 6 && s.Mode() == Mode::kAuto);
  s.SetFixedAddress(4);
  s.Handle(Rx(65305, 4, "41 9F 64 0A 10 00 00 00"), u + 0.2);
  CHECK(s.Address(u) == 4 && s.Alive(u + 0.2) && s.Mode() == Mode::kAuto);

  // Pilot moves to address 9: followed by NAME.
  uint64_t raw = 1907005ull | (275ull << 21) | (150ull << 40) | (40ull << 49) | (4ull << 60) | (1ull << 63);
  std::string hex;
  char b[4];
  for (int i = 0; i < 8; i++) { std::snprintf(b, sizeof b, "%02X ", static_cast<unsigned>((raw >> (8 * i)) & 0xFF)); hex += b; }
  s.Handle(Rx(60928, 4, hex), u + 1);
  s.SetPilotName(raw);
  s.Handle(Rx(60928, 9, hex), u + 2);
  CHECK(s.FixedAddress() == 9);

  // Own messages are ignored.
  PilotState o;
  std::vector<uint8_t> own = {0x94, 0x13, 7, 0x19, 0xFF, 0x00, 255, 255, 0x41, 0x9F, 0x64, 0x0A, 0x08, 0, 0, 0, 0x55};
  o.Handle(*ParseOcpnPayload(own), 1);
  CHECK(!o.Address(1));
}

// Frames recorded on Penny Lane, 13 Nov 2026 (NAC-3 at 4, Triton2 at 18).
static void TestNavAndSensors() {
  CheckEq(CommandData(Command::kConfirm, 4), "41 9F 04 FF FF 0A 1C 00 FF FF FF", "confirm");
  CheckEq(CommandData(Command::kTack, 4), "41 9F 04 FF FF 0A 11 00 00 FF FF FF", "tack");
  CheckEq(NfuData(NfuSide::kPort, 4), "41 9F 04 FF FF 02 0D 00 04 00 00 00", "nfu port");
  CheckEq(NfuData(NfuSide::kStarboard, 4), "41 9F 04 FF FF 02 0D 00 05 00 00 00", "nfu stbd");
  CheckEq(NfuData(NfuSide::kStop, 4), "41 9F 04 FF FF 02 0D 00 FF 00 00 00", "nfu stop");
  auto stw = ParseSpeedThroughWater(Bytes("01 F4 01 FF FF 00 FF FF"));  // 5.00 m/s
  CHECK(stw && std::fabs(*stw - 5.0) < 1e-9);
  CHECK(!ParseSpeedThroughWater(Bytes("01 FF FF FF FF 00 FF FF")));

  // Echo: an 11-byte command comes back padded to 12 bytes.
  CHECK(EchoMatches(CommandData(Command::kNav, 4), Bytes("41 9f 04 ff ff 0a 0a 00 ff ff ff a3")));
  CHECK(EchoMatches(CommandData(Command::kNav, 4), Bytes("41 9f 04 ff ff 0a 0a 00 ff ff ff 00")));
  CHECK(!EchoMatches(CommandData(Command::kNav, 4), Bytes("41 9f 04 ff ff 0a 09 00 ff ff ff a3")));

  // Rudder from the NAC-3 rudder feedback; the pilot's own 127245 has none.
  auto r = ParseRudder(Bytes("FF 07 FF 7F EF FF FF FF"));
  CHECK(r && std::fabs(*r - -0.0017) < 1e-9);
  CHECK(!ParseRudder(Bytes("FF 00 10 00 FF 7F FF FF")));

  // Alarm and its text.
  auto a = ParseAlarm(Bytes("41 9f ff ff 01 ff 39 00 4d 00 04 00"));
  CHECK(a && a->id == 0x4D && a->state == 0x39 && a->f == 4);
  CHECK(!ParseAlarm(CommandData(Command::kNav, 4)));
  auto t = ParseAlarmText(Bytes("41 9f 4d 00 18 01 44 72 69 76 65 20 63 6f 6d 70 75 74 65 72 20 6d 69 73 73 69 6e 67"));
  CHECK(t && t->id == 0x4D && t->text == "Drive computer missing");

  PilotState s;
  double t0 = 1000;
  s.Handle(Rx(65305, 4, "41 9F 64 0A 08 00 00 00"), t0);
  CHECK(s.Mode() == Mode::kStandby && !s.AwaitsConfirmation(t0));

  // NAV: echo, then "waiting for confirmation" (0x88), not an error.
  s.CommandSent(Command::kNav, CommandData(Command::kNav, 4), t0 + 1);
  s.Handle(Rx(130851, 4, "41 9f 04 ff ff 0a 0a 00 ff ff ff a3"), t0 + 1.3);
  s.Handle(Rx(65305, 4, "41 9F 64 0A 88 00 00 00"), t0 + 1.4);
  CHECK(s.AwaitsConfirmation(t0 + 2) && s.Mode() == Mode::kStandby);
  CHECK(!s.Unconfirmed(t0 + 5) && !s.NotFollowed(t0 + 5));
  // Confirm + NAV, pilot goes to NAV (40 00 00 40).
  s.CommandSent(Command::kNav, CommandData(Command::kNav, 4), t0 + 4);
  s.Handle(Rx(65305, 4, "41 9F 64 0A 40 00 00 40"), t0 + 4.1);
  CHECK(s.Mode() == Mode::kNav && !s.AwaitsConfirmation(t0 + 4.2) && !s.Unconfirmed(t0 + 9));

  // Waiting expires: back to standby without bit 7.
  s.Handle(Rx(65305, 4, "41 9F 64 0A 88 00 00 00"), t0 + 20);
  CHECK(s.AwaitsConfirmation(t0 + 21));
  s.Handle(Rx(65305, 4, "41 9F 64 0A 08 00 00 40"), t0 + 30);
  CHECK(!s.AwaitsConfirmation(t0 + 30));

  // NAV without navigation data: echoed but not followed.
  s.CommandSent(Command::kNav, CommandData(Command::kNav, 4), t0 + 40);
  s.Handle(Rx(130851, 4, "41 9f 04 ff ff 0a 0a 00 ff ff ff a3"), t0 + 40.3);
  s.Handle(Rx(65305, 4, "41 9F 64 0A 08 00 00 40"), t0 + 41);
  CHECK(!s.NotFollowed(t0 + 42) && !s.Unconfirmed(t0 + 42));  // 3 s, as the app
  CHECK(s.NotFollowed(t0 + 44) && !s.Unconfirmed(t0 + 44));
  // No echo at all: unconfirmed.
  s.CommandSent(Command::kAuto, CommandData(Command::kAuto, 4), t0 + 50);
  CHECK(s.Unconfirmed(t0 + 52) && !s.NotFollowed(t0 + 52));

  // Alarms from the pilot only.
  s.Handle(Rx(130850, 4, "41 9f ff ff 01 ff 39 00 4d 00 04 00"), t0 + 60);
  s.Handle(Rx(130856, 4, "41 9f 4d 00 18 01 44 72 69 76 65 20 63 6f 6d 70 75 74 65 72 20 6d 69 73 73 69 6e 67"), t0 + 60);
  s.Handle(Rx(130850, 2, "41 9f ff ff 01 ff 39 00 22 00 01 00"), t0 + 60);
  auto al = s.Alarms(t0 + 61);
  CHECK(al.size() == 1 && al[0].id == 0x4D && al[0].text == "Drive computer missing");
  s.Handle(Rx(130850, 4, "41 9f ff ff 01 ff 3a 00 4d 00 04 00"), t0 + 62);
  CHECK(s.Alarms(t0 + 63).empty());

  // Rudder sources: automatic only with exactly one.
  s.Handle(Rx(127245, 7, "FF 07 FF 7F EF FF FF FF"), t0 + 70);
  CHECK(s.Rudder(t0 + 70) && std::fabs(*s.Rudder(t0 + 70) - -0.0017) < 1e-9);
  s.Handle(Rx(127245, 8, "FF 07 FF 7F 00 00 FF FF"), t0 + 70);
  s.Handle(Rx(127245, 4, "FF 00 10 00 FF 7F FF FF"), t0 + 70);
  CHECK(s.RudderSources(t0 + 70).size() == 2);
  auto name_hex = [](uint64_t raw) {
    std::string hex;
    char b[4];
    for (int i = 0; i < 8; i++) { std::snprintf(b, sizeof b, "%02X ", static_cast<unsigned>((raw >> (8 * i)) & 0xFF)); hex += b; }
    return hex;
  };
  uint64_t raw7 = 1087805ull | (275ull << 21) | (155ull << 40) | (40ull << 49);
  uint64_t raw8 = 1104189ull | (275ull << 21) | (155ull << 40) | (40ull << 49);
  s.Handle(Rx(60928, 7, name_hex(raw7)), t0 + 70);
  s.Handle(Rx(60928, 8, name_hex(raw8)), t0 + 70);
  // Same make and function: 126996 tells the virtual one apart.
  auto product = [](const std::string& model) {
    std::string hex = "35 08 B8 63 ";
    char b[4];
    for (size_t i = 0; i < 32; i++) { std::snprintf(b, sizeof b, "%02X ", i < model.size() ? static_cast<unsigned char>(model[i]) : 0xFF); hex += b; }
    for (int i = 0; i < 98; i++) hex += "FF ";
    return hex;
  };
  s.Handle(Rx(126996, 8, product("NAC-3 _Virtual rudder feedback")), t0 + 70);
  s.Handle(Rx(126996, 7, product("NAC-3 _Rudder feedback")), t0 + 70);
  CHECK(s.AutoRudder(t0 + 70) == 7);
  CHECK(s.Rudder(t0 + 71) && std::fabs(*s.Rudder(t0 + 71) - -0.0017) < 1e-9);
  s.SetRudderName(raw8);  // the user overrules
  CHECK(s.Rudder(t0 + 71) && *s.Rudder(t0 + 71) == 0);
  s.SetRudderName(raw7);
  CHECK(!s.Rudder(t0 + 80));  // stale

  // Apparent wind, port = negative; a second source is ignored while the first talks.
  s.Handle(Rx(130306, 13, "01 F4 01 9F B3 02 FF FF"), t0 + 90);   // 4.5983 rad = -96.5 deg
  s.Handle(Rx(130306, 40, "01 F4 01 10 27 02 FF FF"), t0 + 90.1); // 1.0 rad
  auto w = s.ApparentWind(t0 + 90.2);
  CHECK(w && *w < 0 && std::fabs(*w - (4.5983 - 2 * 3.14159265358979)) < 1e-3);
  s.Handle(Rx(130306, 13, "01 F4 01 10 27 03 FF FF"), t0 + 91);   // true wind: ignored
  CHECK(s.ApparentWind(t0 + 91) && *s.ApparentWind(t0 + 91) < 0);
}

static void TestKeyValue() {
  // canboat example: read 18/0079 at address 3.
  CheckEq(KeyValueRead(3, {0x18, 0x0079}), "41 9F 03 00 FF FF 18 79 00 00 FF FF FF FF", "kv read");
  // Recorded from a Triton2, rudder deadband manual 0.1 deg and back to Auto.
  CheckEq(KeyValueSet(4, {0x14, 0x0006}, 17), "41 9F 04 FF FF FF 14 06 00 01 11 00 FF FF", "kv set");
  CheckEq(KeyValueSet(4, {0x14, 0x0006}, 0), "41 9F 04 FF FF FF 14 06 00 01 00 00 FF FF", "kv set auto");
  auto kv = ParseKeyValue(Bytes("41 9F 03 00 FF FF 18 79 00 00 FF FF FF FF"), false);
  CHECK(kv && kv->address == 3 && kv->key.group == 0x18 && kv->key.param == 0x79 &&
        kv->op == KeyOp::kRead && !kv->Unsigned());
  auto r = ParseKeyValue(Bytes("41 9F 04 00 01 05 18 0A 00 02 00 FF FF FF"), false);
  CHECK(r && r->op == KeyOp::kReply && r->Unsigned(1) == 0u);
  // 130846 from a Triton2: "Alert bits" (80 15) set to the whole bus, 8 bytes.
  auto l = ParseKeyValue(Bytes("41 9F FF FF 01 FF FF 15 80 01 08 55 55 55 55 45 55 55 01"), true);
  CHECK(l && l->address == 0xFF && l->key.group == 0xFF && l->key.param == 0x8015 &&
        l->op == KeyOp::kSet && l->value.size() == 8);

  // Replies are kept per device; reads and sets are not.
  PilotState s;
  s.Handle(Rx(130845, 4, "41 9F 04 00 01 05 18 0A 00 02 00 FF FF FF"), 1);
  s.Handle(Rx(130845, 4, "41 9F 04 00 FF FF 18 0B 00 00 FF FF FF FF"), 1);
  auto v = s.Setting(4, {0x18, 0x0A});
  CHECK(v && v->Unsigned(1) == 0u);
  CHECK(!s.Setting(4, {0x18, 0x0B}) && !s.Setting(7, {0x18, 0x0A}));

  // Decoding: transition speed 617 (0.01 m/s) = 12.0 kn; rudder calibration -6109 = -35 deg.
  const pilot_settings::Definition* ts = nullptr;
  const pilot_settings::Definition* cal = nullptr;
  for (const auto& d : pilot_settings::All()) {
    if (d.key == SettingKey{0x18, 0x0C}) ts = &d;
    if (d.key == SettingKey{0x09, 0x02}) cal = &d;
  }
  CHECK(ts && cal);
  auto t = pilot_settings::Decode(*ts, *ParseKeyValue(Bytes("41 9F 04 00 01 05 18 0C 00 02 69 02 FF FF"), false));
  CHECK(t && std::fabs(*t - 11.99) < 0.01);
  auto c = pilot_settings::Decode(*cal, *ParseKeyValue(Bytes("41 9F 07 00 01 05 09 02 00 02 23 E8 FF FF"), false));
  CHECK(c && std::fabs(*c - -35.0) < 0.01);

  // Encoding for a change: deadband 0.1 deg = 17 (as the Triton2 sent), 0 = Auto.
  const pilot_settings::Definition* db = nullptr;
  for (const auto& d : pilot_settings::All())
    if (d.key == SettingKey{0x14, 0x06}) db = &d;
  CHECK(db && pilot_settings::Encode(*db, 0.1) == 17 && pilot_settings::Encode(*db, 0) == 0);
  CHECK(pilot_settings::Editable({0x14, 0x06}) && !pilot_settings::Editable({0x14, 0x1D}) &&
        !pilot_settings::Editable({0x18, 0x0A}) && !pilot_settings::Editable({0x09, 0x01}));

  // Steering profiles (NAC-3, 13 Nov 2026): rudder gain per instance. A
  // broadcast of profile 1 must not replace profile 0; a set reply (FF)
  // counts for the plain lookup.
  PilotState p;
  p.Handle(Rx(130845, 4, "41 9F FF 00 FF FF 1A 0D 00 02 34 00 00 00"), 1);
  p.Handle(Rx(130845, 4, "41 9F FF 01 FF FF 1A 0D 00 02 32 00 00 00"), 2);
  CHECK(p.Setting(4, {0x1A, 0x0D})->Unsigned(2) == 0x34u);
  CHECK(p.Setting(4, {0x1A, 0x0D}, 1)->Unsigned(2) == 0x32u);
  CHECK(!p.Setting(4, {0x1A, 0x0D}, 2));
  p.Handle(Rx(130845, 4, "41 9F FF FF FF FF 14 06 00 02 10 00 00 00"), 3);
  p.Handle(Rx(130845, 4, "41 9F FF 00 FF FF 14 06 00 02 00 00 00 00"), 2);
  CHECK(p.Setting(4, {0x14, 0x06})->Unsigned(2) == 0x10u && p.SettingTime(4, {0x14, 0x06}) == 3);
  // Profile name in 130846.
  p.Handle(Rx(130846, 4, "41 9F FF 03 FF 00 18 08 80 02 0A 42 45 41 54 00 00 00 00 00 00"), 4);
  auto name = p.Setting(4, pilot_settings::kProfileName, 3);
  CHECK(name && pilot_settings::Text(*name) == "BEAT");
}

int main() {
  TestCommands();
  TestParse();
  TestPilotState();
  TestNavAndSensors();
  TestKeyValue();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("all tests passed\n");
  return 0;
}
