/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "simnet.h"

#include <cmath>
#include <algorithm>
#include <cstdio>

namespace simnet {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Simrad (1857) in 11 bits, reserved 2 bits set, industry Marine (4).
constexpr uint8_t kHeader0 = 0x41;
constexpr uint8_t kHeader1 = 0x9F;

bool HasSimnetHeader(const std::vector<uint8_t>& d) {
  return d.size() >= 2 && d[0] == kHeader0 && d[1] == kHeader1;
}

uint16_t U16(const std::vector<uint8_t>& d, size_t i) {
  return static_cast<uint16_t>(d[i] | (d[i + 1] << 8));
}

std::optional<double> Angle(uint16_t raw) {
  if (raw >= 0xFFFD) return std::nullopt;  // not available / out of range
  return raw * 0.0001;
}

// Course change in 0.0001 rad, rounded like the keypad: 1 deg -> 175,
// 10 deg -> 1745 (recorded on a real bus).
uint16_t DegreesToRaw(int degrees) {
  return static_cast<uint16_t>(std::lround(degrees * kPi / 180.0 * 10000.0));
}

}  // namespace

const char* ModeName(Mode m) {
  switch (m) {
    case Mode::kStandby: return "STBY";
    case Mode::kAuto: return "AUTO";
    case Mode::kWind: return "WIND";
    case Mode::kNav: return "NAV";
    case Mode::kNoDrift: return "NO DRIFT";
    case Mode::kUnknown: break;
  }
  return "?";
}

const char* CommandName(Command c) {
  switch (c) {
    case Command::kStandby: return "STBY";
    case Command::kAuto: return "AUTO";
    case Command::kWind: return "WIND";
    case Command::kNav: return "NAV";
    case Command::kNoDrift: return "NO DRIFT";
    case Command::kPort1: return "-1";
    case Command::kPort10: return "-10";
    case Command::kStarboard1: return "+1";
    case Command::kStarboard10: return "+10";
    case Command::kConfirm: return "CONFIRM";
    case Command::kTack: return "TACK";
  }
  return "?";
}

std::vector<uint8_t> CommandData(Command command, uint8_t address) {
  // Byte 2 address of the pilot, 3-4 reserved, 5 command type 0x0A
  // ("AP command"), 6 event, 7 must be 0x00: a NAC-3 ignores the command
  // when it is 0xFF (canboat calls it "spare").
  std::vector<uint8_t> d = {kHeader0, kHeader1, address, 0xFF, 0xFF, 0x0A};
  auto mode = [&d](uint8_t event) {
    d.insert(d.end(), {event, 0x00, 0xFF, 0xFF, 0xFF});
  };
  auto change = [&d](bool starboard, int degrees) {
    uint16_t raw = DegreesToRaw(degrees);
    // Event 0x1A change course; direction 2 port, 3 starboard.
    d.insert(d.end(), {0x1A, 0x00, static_cast<uint8_t>(starboard ? 0x03 : 0x02),
                       static_cast<uint8_t>(raw & 0xFF),
                       static_cast<uint8_t>(raw >> 8), 0xFF});
  };
  switch (command) {
    case Command::kStandby: mode(0x06); break;
    case Command::kAuto: mode(0x09); break;
    case Command::kNav: mode(0x0A); break;
    case Command::kNoDrift: mode(0x0C); break;
    case Command::kWind: mode(0x0F); break;
    case Command::kPort1: change(false, 1); break;
    case Command::kPort10: change(false, 10); break;
    case Command::kStarboard1: change(true, 1); break;
    case Command::kStarboard10: change(true, 10); break;
    // Recorded from a Triton2 keypad confirming NAV, followed by NAV (0x0A).
    case Command::kConfirm: mode(0x1C); break;
    // As the Triton2 app sends it: no side, the pilot tacks through the wind
    // (or gybes) to the mirrored wind angle.
    case Command::kTack: d.insert(d.end(), {0x11, 0x00, 0x00, 0xFF, 0xFF, 0xFF}); break;
  }
  return d;
}

std::vector<uint8_t> NfuData(NfuSide side, uint8_t address) {
  // Recorded from a Triton2 keypad (16 Oct 2026): command type 0x02, event
  // 0x0D (non-follow-up), direction 4 port rudder, 5 starboard, FF stop.
  uint8_t dir = side == NfuSide::kPort ? 0x04 : side == NfuSide::kStarboard ? 0x05 : 0xFF;
  return {kHeader0, kHeader1, address, 0xFF, 0xFF, 0x02, 0x0D, 0x00, dir, 0x00, 0x00, 0x00};
}

std::vector<uint8_t> IsoRequestData(int pgn) {
  return {static_cast<uint8_t>(pgn & 0xFF), static_cast<uint8_t>((pgn >> 8) & 0xFF),
          static_cast<uint8_t>((pgn >> 16) & 0xFF)};
}

std::string Hex(const std::vector<uint8_t>& data) {
  std::string s;
  char b[4];
  for (size_t i = 0; i < data.size(); i++) {
    std::snprintf(b, sizeof b, i ? " %02X" : "%02X", data[i]);
    s += b;
  }
  return s;
}

std::optional<Message> ParseOcpnPayload(const std::vector<uint8_t>& v) {
  Message m;
  if (v.size() >= 13 && v[0] == 0x93) {
    // [0x93, len, prio, pgn0, pgn1, pgn2, dst, src, time x4, count, data.., crc]
    size_t count = v[12];
    if (v.size() < 13 + count) return std::nullopt;
    m.data.assign(v.begin() + 13, v.begin() + 13 + count);
    m.source = v[7];
  } else if (v.size() >= 9 && v[0] == 0x94) {
    // Own transmission: [0x94, 0x13, prio, pgn0, pgn1, pgn2, addr, addr, data.., 0x55]
    m.data.assign(v.begin() + 8, v.end() - 1);
    m.source = 255;
    m.own = true;
  } else {
    return std::nullopt;
  }
  m.priority = v[2];
  m.pgn = v[3] | (v[4] << 8) | (v[5] << 16);
  m.destination = v[6];
  return m;
}

std::optional<PilotMode> ParsePilotMode(const std::vector<uint8_t>& d) {
  if (d.size() < 6 || !HasSimnetHeader(d) || d[3] != 0x0A) return std::nullopt;
  if (d[2] != 0x00 && d[2] != 0x64) return std::nullopt;  // AC or NAC only
  PilotMode p;
  p.model = d[2];
  p.bits = U16(d, 4);
  // Bits (canboat SIMNET_AP_MODE_BITFIELD): 3 standby, 4 heading, 6 nav,
  // 8 no drift, 10 wind. If more than one is set, the last match below wins
  // (same order as n2k-signalk).
  if (p.bits & (1 << 3)) p.mode = Mode::kStandby;
  if (p.bits & (1 << 8)) p.mode = Mode::kNoDrift;
  if (p.bits & (1 << 4)) p.mode = Mode::kAuto;
  if (p.bits & (1 << 10)) p.mode = Mode::kWind;
  if (p.bits & (1 << 6)) p.mode = Mode::kNav;
  return p;
}

std::optional<bool> ParsePilotEngaged(const std::vector<uint8_t>& d) {
  if (d.size() < 5 || !HasSimnetHeader(d) || d[3] != 0x02) return std::nullopt;
  if (d[2] != 0x00 && d[2] != 0x64) return std::nullopt;
  if (d[4] == 0x10) return true;
  if (d[4] == 0x02) return false;
  return std::nullopt;  // other values seen on a NAC-3, meaning unknown
}

std::optional<HeadingControl> ParseHeadingControl(const std::vector<uint8_t>& d) {
  if (d.size() < 7) return std::nullopt;
  HeadingControl h;
  h.magnetic = ((d[1] >> 6) & 0x03) == 1;
  h.target = Angle(U16(d, 5));
  return h;
}

std::optional<ApAngle> ParseApAngle(const std::vector<uint8_t>& d) {
  if (d.size() < 8 || !HasSimnetHeader(d)) return std::nullopt;
  ApAngle a;
  a.mode = d[4];
  a.angle = Angle(U16(d, 6));
  return a;
}

bool EchoMatches(const std::vector<uint8_t>& sent, const std::vector<uint8_t>& reply) {
  if (sent.size() < 7 || reply.size() < sent.size()) return false;
  // The pilot pads an 11-byte command to 12 bytes; compare what was sent.
  for (size_t i = 0; i < sent.size(); i++)
    if (sent[i] != reply[i]) return false;
  return true;
}

std::optional<double> ParseRudder(const std::vector<uint8_t>& d) {
  if (d.size() < 6) return std::nullopt;
  int16_t raw = static_cast<int16_t>(U16(d, 4));
  if (raw == 0x7FFF || raw == -0x8000) return std::nullopt;
  return raw * 0.0001;
}

std::optional<Wind> ParseWind(const std::vector<uint8_t>& d) {
  if (d.size() < 6) return std::nullopt;
  Wind w;
  w.angle = Angle(U16(d, 3));
  w.reference = d[5] & 0x07;
  return w;
}

std::optional<Alarm> ParseAlarm(const std::vector<uint8_t>& d) {
  if (d.size() < 11 || !HasSimnetHeader(d) || d[4] != 0x01) return std::nullopt;
  Alarm a;
  a.state = d[6];
  a.id = U16(d, 8);
  a.f = d[10];
  return a;
}

std::optional<AlarmText> ParseAlarmText(const std::vector<uint8_t>& d) {
  if (d.size() < 6 || !HasSimnetHeader(d) || d[5] != 0x01) return std::nullopt;
  AlarmText t;
  t.id = U16(d, 2);
  size_t len = d[4] >= 2 ? d[4] - 2 : 0;
  len = std::min(len, d.size() - 6);
  t.text.assign(d.begin() + 6, d.begin() + 6 + len);
  while (!t.text.empty() && (t.text.back() == '\0' || t.text.back() == ' '))
    t.text.pop_back();
  return t;
}

std::vector<uint8_t> KeyValueRead(uint8_t address, SettingKey key, uint8_t instance) {
  // Layout from canboat (130845 Simnet Key Value); example there:
  // 41 9F 03 00 FF FF 18 79 00 00 FF FF FF FF = read 18/0079 at address 3.
  return {kHeader0, kHeader1, address, instance, 0xFF, 0xFF, key.group,
          static_cast<uint8_t>(key.param & 0xFF), static_cast<uint8_t>(key.param >> 8),
          static_cast<uint8_t>(KeyOp::kRead), 0xFF, 0xFF, 0xFF, 0xFF};
}

std::vector<uint8_t> KeyValueSet(uint8_t address, SettingKey key, uint32_t value, int bytes,
                                 uint8_t instance) {
  std::vector<uint8_t> d = {kHeader0, kHeader1, address, instance, 0xFF, 0xFF, key.group,
                            static_cast<uint8_t>(key.param & 0xFF),
                            static_cast<uint8_t>(key.param >> 8),
                            static_cast<uint8_t>(KeyOp::kSet)};
  for (int i = 0; i < 4; i++)
    d.push_back(i < bytes ? static_cast<uint8_t>((value >> (8 * i)) & 0xFF) : 0xFF);
  return d;
}

std::optional<uint32_t> KeyValue::Unsigned(size_t bytes) const {
  if (value.size() < bytes || bytes == 0 || bytes > 4) return std::nullopt;
  uint32_t v = 0;
  bool all_ff = true;
  for (size_t i = 0; i < bytes; i++) {
    v |= static_cast<uint32_t>(value[i]) << (8 * i);
    if (value[i] != 0xFF) all_ff = false;
  }
  if (all_ff) return std::nullopt;
  return v;
}

std::optional<KeyValue> ParseKeyValue(const std::vector<uint8_t>& d, bool long_form) {
  if (d.size() < 10 || !HasSimnetHeader(d)) return std::nullopt;
  KeyValue kv;
  kv.address = d[2];
  kv.instance = d[3];
  kv.key.group = d[6];
  kv.key.param = U16(d, 7);
  if (d[9] > 2) return std::nullopt;
  kv.op = static_cast<KeyOp>(d[9]);
  if (long_form) {
    // 130846: a length byte, then the value.
    if (d.size() < 11) return std::nullopt;
    size_t len = std::min<size_t>(d[10], d.size() - 11);
    kv.value.assign(d.begin() + 11, d.begin() + 11 + len);
  } else {
    kv.value.assign(d.begin() + 10, d.end());
  }
  return kv;
}

std::optional<double> ParseSpeedThroughWater(const std::vector<uint8_t>& d) {
  if (d.size() < 3) return std::nullopt;
  uint16_t raw = U16(d, 1);
  if (raw >= 0xFFFD) return std::nullopt;
  return raw * 0.01;
}

std::optional<NavData> ParseNavData(const std::vector<uint8_t>& d) {
  if (d.size() < 16) return std::nullopt;
  NavData n;
  uint32_t dist = d[1] | (d[2] << 8) | (d[3] << 16) | (static_cast<uint32_t>(d[4]) << 24);
  if (dist < 0xFFFFFFFD) n.distance = dist * 0.01;
  n.bearing = Angle(U16(d, 14));  // bearing, position to destination
  if (d.size() >= 24) {
    uint32_t dest = d[20] | (d[21] << 8) | (d[22] << 16) | (static_cast<uint32_t>(d[23]) << 24);
    if (dest < 0xFFFFFFFD) n.destination = dest;
  }
  return n;
}

namespace {
// A STRING_LAU at \p pos: length (with these two bytes), encoding (1 ASCII,
// 0 UTF-16), then the text. Moves \p pos past it.
std::optional<std::string> Lau(const std::vector<uint8_t>& d, size_t& pos) {
  if (pos + 2 > d.size()) return std::nullopt;
  size_t len = d[pos];
  uint8_t enc = d[pos + 1];
  if (len < 2 || pos + len > d.size()) return std::nullopt;
  std::string s;
  for (size_t i = pos + 2; i < pos + len; i += (enc == 0 ? 2 : 1)) {
    uint8_t c = d[i];
    if (c == 0) break;
    s += static_cast<char>(c);
  }
  pos += len;
  while (!s.empty() && (s.back() == ' ' || s.back() == '@')) s.pop_back();
  return s;
}
}  // namespace

std::optional<RouteInfo> ParseRouteInfo(const std::vector<uint8_t>& d) {
  if (d.size() < 13) return std::nullopt;
  RouteInfo r;
  uint16_t items = U16(d, 2);
  size_t pos = 9;
  auto name = Lau(d, pos);
  if (!name) return std::nullopt;
  r.route_name = *name;
  pos += 1;  // reserved
  for (uint16_t i = 0; i < items; i++) {
    if (pos + 2 > d.size()) return std::nullopt;
    RouteWaypoint w;
    w.id = U16(d, pos);
    pos += 2;
    auto n = Lau(d, pos);
    if (!n || pos + 8 > d.size()) return std::nullopt;
    w.name = *n;
    pos += 8;  // latitude, longitude
    r.waypoints.push_back(w);
  }
  return r;
}

std::optional<std::string> ParseModelId(const std::vector<uint8_t>& d) {
  if (d.size() < 36) return std::nullopt;
  std::string m;
  for (size_t i = 4; i < 36; i++) {
    if (d[i] == 0x00 || d[i] == 0xFF || d[i] == '@') break;
    m += static_cast<char>(d[i]);
  }
  while (!m.empty() && m.back() == ' ') m.pop_back();
  return m;
}

std::string Name::ManufacturerName() const {
  switch (manufacturer) {
    case 275: return "Navico";
    case 381: return "B&G";
    case 1857: return "Simrad";
    case 1855: return "Furuno";
    case 717: return "Yacht Devices";
    case 135: return "Airmar";
    case 358: return "Victron";
    case 1851: return "Raymarine";
    default: return std::to_string(manufacturer);
  }
}

bool Name::IsAutopilot() const {
  return function == kFunctionAutopilot &&
         (manufacturer == kManufacturerNavico ||
          manufacturer == kManufacturerSimrad);
}

std::optional<Name> ParseAddressClaim(const std::vector<uint8_t>& d) {
  if (d.size() < 8) return std::nullopt;
  Name n;
  for (int i = 7; i >= 0; i--) n.raw = (n.raw << 8) | d[i];
  n.unique = n.raw & 0x1FFFFF;
  n.manufacturer = (n.raw >> 21) & 0x7FF;
  n.function = (n.raw >> 40) & 0xFF;
  n.device_class = (n.raw >> 49) & 0x7F;
  return n;
}

}  // namespace simnet
