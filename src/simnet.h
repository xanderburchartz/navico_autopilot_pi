/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

/**
 * \file simnet.h
 * Building and parsing the Simnet (Navico proprietary NMEA 2000) messages used
 * to control a Navico autopilot computer (NAC-1/2/3, AC12/42).
 *
 * No wxWidgets and no OpenCPN dependencies, so this can be unit tested.
 * Byte layouts: canboat (docs/canboat.json) and recordings of a B&G Triton2
 * pilot keypad and a NAC-3 on a real bus. See README.md, "Protocol".
 */
#ifndef NAVICO_AP_SIMNET_H
#define NAVICO_AP_SIMNET_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace simnet {

constexpr int kPgnCommand = 130850;       ///< Simnet: AP command (sent)
constexpr int kPgnDeviceStatus = 65305;   ///< Simnet: device status / pilot mode
constexpr int kPgnApAngle = 65341;        ///< Simnet: autopilot angle
constexpr int kPgnHeadingControl = 127237;  ///< Heading/Track control
constexpr int kPgnAddressClaim = 60928;   ///< ISO address claim
constexpr int kPgnIsoRequest = 59904;     ///< ISO request
constexpr int kPgnProductInfo = 126996;   ///< Product information (model ID)
constexpr uint8_t kFunctionRudder = 155;
constexpr int kPgnCommandReply = 130851;  ///< Simnet: echo of an executed command
constexpr int kPgnAlarmText = 130856;     ///< Simnet: alarm text
constexpr int kPgnRudder = 127245;        ///< Rudder
constexpr int kPgnWind = 130306;          ///< Wind data
constexpr int kPgnNavData = 129284;       ///< Navigation data (to the waypoint)
constexpr int kPgnSpeed = 128259;         ///< Speed (through the water)
constexpr int kPgnKeyValue = 130845;      ///< Simnet: key value (settings)
constexpr int kPgnKeyValueLong = 130846;  ///< Simnet: key value, longer values
constexpr int kCommandPriority = 2;       ///< as the keypad and the B&G app
constexpr uint16_t kManufacturerSimrad = 1857;
constexpr uint16_t kManufacturerNavico = 275;
constexpr uint8_t kFunctionAutopilot = 150;

/** Commands the plugin can send. */
enum class Command {
  kStandby,
  kAuto,     ///< heading hold
  kWind,
  kNav,
  kNoDrift,
  kPort1,
  kPort10,
  kStarboard1,
  kStarboard10,
  kConfirm,  ///< confirm a pending mode change (NAV: course to waypoint)
  kTack      ///< tack or gybe (in WIND); the pilot chooses the side
};

/** Non-follow-up: rudder to port, to starboard, or stop. */
enum class NfuSide { kStop, kPort, kStarboard };

/** Data of PGN 130850 for non-follow-up, as a Triton2 keypad sends it. */
std::vector<uint8_t> NfuData(NfuSide side, uint8_t address);

/** Pilot mode as reported in PGN 65305, report "Mode". */
enum class Mode { kUnknown, kStandby, kAuto, kWind, kNav, kNoDrift };

const char* ModeName(Mode m);    ///< "STBY", "AUTO", ...
const char* CommandName(Command c);

/** Data bytes of PGN 130850 for \p command to the pilot at \p address. */
std::vector<uint8_t> CommandData(Command command, uint8_t address);

/** Data of an ISO request (59904) for \p pgn. */
std::vector<uint8_t> IsoRequestData(int pgn);

/** Hex dump like "41 9F 04 FF". */
std::string Hex(const std::vector<uint8_t>& data);

/** A message as handed to plugins by OpenCPN's GetN2000Payload(). */
struct Message {
  int pgn = 0;
  uint8_t priority = 0;
  uint8_t source = 255;
  uint8_t destination = 255;
  bool own = false;  ///< sent by this OpenCPN instance (looped back)
  std::vector<uint8_t> data;
};

/**
 * Parse the vector returned by GetN2000Payload(). Received messages start
 * with 0x93 (with time stamp and length fields); messages this OpenCPN sent
 * itself come back with 0x94 and without those fields.
 */
std::optional<Message> ParseOcpnPayload(const std::vector<uint8_t>& v);

/** Model byte in 65305: 0x64 NAC, 0x00 AC. */
struct PilotMode {
  uint8_t model = 0;
  Mode mode = Mode::kUnknown;
  uint16_t bits = 0;
  /** Bit 7: the pilot waits for a confirmation (seen after NAV: 0x88 in
   *  standby, back to 0x08 after 10 s without one). */
  bool AwaitsConfirmation() const { return bits & 0x80; }
};

/** 65305 with report 0x0A ("Mode") from an AC or NAC. Nothing for requests. */
std::optional<PilotMode> ParsePilotMode(const std::vector<uint8_t>& d);

/** 65305 with report 0x02 ("Status"): true = automatic, false = manual. */
std::optional<bool> ParsePilotEngaged(const std::vector<uint8_t>& d);

/** 127237: heading to steer (radians) and whether it is magnetic. */
struct HeadingControl {
  std::optional<double> target;  ///< none in standby (0xFFFF)
  bool magnetic = false;
};
std::optional<HeadingControl> ParseHeadingControl(const std::vector<uint8_t>& d);

/** 65341: mode (2 heading, 3 wind, 10 nav, 11 no drift) and angle (rad). */
struct ApAngle {
  uint8_t mode = 0;
  std::optional<double> angle;  ///< 0..2pi
};
std::optional<ApAngle> ParseApAngle(const std::vector<uint8_t>& d);

/**
 * 130851 from the pilot is a literal copy of the 130850 it executed (padded
 * to 12 bytes), 25 to 55 ms later. True if \p reply echoes \p sent.
 */
bool EchoMatches(const std::vector<uint8_t>& sent, const std::vector<uint8_t>& reply);

/** 127245: rudder position (rad, positive = starboard), none if not available. */
std::optional<double> ParseRudder(const std::vector<uint8_t>& d);

/** 130306: wind angle (rad, 0..2pi) and reference (2 = apparent). */
struct Wind {
  std::optional<double> angle;
  uint8_t reference = 0xFF;
};
std::optional<Wind> ParseWind(const std::vector<uint8_t>& d);

/** Alarm variant of 130850 (byte 4 = 0x01), sent by the pilot. */
struct Alarm {
  uint16_t id = 0;
  uint8_t state = 0;  ///< 0x39 active, 0x3A cleared, 0x38 acknowledged
  uint8_t f = 0;      ///< meaning unknown, 1..4 seen
};
std::optional<Alarm> ParseAlarm(const std::vector<uint8_t>& d);

/** 130856: text of an alarm. */
struct AlarmText {
  uint16_t id = 0;
  std::string text;
};
std::optional<AlarmText> ParseAlarmText(const std::vector<uint8_t>& d);

/** 128259: speed through the water (m/s), none if not available. */
std::optional<double> ParseSpeedThroughWater(const std::vector<uint8_t>& d);

/** 129284: bearing (rad, 0..2pi) and distance (m) from position to waypoint. */
struct NavData {
  std::optional<double> bearing;
  std::optional<double> distance;
};
std::optional<NavData> ParseNavData(const std::vector<uint8_t>& d);

/** 126996: model ID, e.g. "NAC-3 _Rudder feedback" (trimmed). */
std::optional<std::string> ParseModelId(const std::vector<uint8_t>& d);

/**
 * A setting key of the Simnet key value channel (130845): byte 6 is the
 * group of the setting, bytes 7-8 the parameter. Written as "18/0A".
 */
struct SettingKey {
  uint8_t group = 0;
  uint16_t param = 0;
  bool operator<(const SettingKey& o) const {
    return group != o.group ? group < o.group : param < o.param;
  }
  bool operator==(const SettingKey& o) const { return group == o.group && param == o.param; }
};

enum class KeyOp : uint8_t { kRead = 0, kSet = 1, kReply = 2 };

/** 130845 asking the device at \p address for the value of \p key. */
std::vector<uint8_t> KeyValueRead(uint8_t address, SettingKey key, uint8_t instance = 0);
/**
 * 130845 setting \p key at \p address to \p value: \p bytes bytes
 * little-endian, padded with FF, instance FF, as a Triton2 sends it
 * (recorded 13 Nov 2026: 41 9F 04 FF FF FF 14 06 00 01 11 00 FF FF). A
 * setting kept per steering profile goes with that profile as instance
 * (Triton2, 13 Nov 2026: 41 9F 04 01 FF FF 1A 0E 00 01 54 01 FF FF).
 */
std::vector<uint8_t> KeyValueSet(uint8_t address, SettingKey key, uint32_t value,
                                 int bytes = 2, uint8_t instance = 0xFF);

/** A received 130845 or 130846. */
struct KeyValue {
  uint8_t address = 0;   ///< byte 2: the device with the setting, FF = whole bus
  uint8_t instance = 0;  ///< byte 3
  SettingKey key;
  KeyOp op = KeyOp::kRead;
  std::vector<uint8_t> value;  ///< raw value bytes (4 in 130845)
  /** Value as unsigned little-endian of \p bytes bytes; none if all FF. */
  std::optional<uint32_t> Unsigned(size_t bytes = 4) const;
};
std::optional<KeyValue> ParseKeyValue(const std::vector<uint8_t>& d, bool long_form);

/** The parts of an ISO NAME (PGN 60928) used to recognise a pilot. */
struct Name {
  uint64_t raw = 0;
  uint32_t unique = 0;
  uint16_t manufacturer = 0;
  uint8_t function = 0;
  uint8_t device_class = 0;
  bool IsAutopilot() const;
  /** "Navico", "B&G", ... or the manufacturer code. */
  std::string ManufacturerName() const;
};
std::optional<Name> ParseAddressClaim(const std::vector<uint8_t>& d);

}  // namespace simnet

#endif  // NAVICO_AP_SIMNET_H
