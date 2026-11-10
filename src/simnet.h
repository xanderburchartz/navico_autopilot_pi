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
  kStarboard10
};

/** Pilot mode as reported in PGN 65305, report "Mode". */
enum class Mode { kUnknown, kStandby, kAuto, kWind, kNav, kNoDrift };

const char* ModeName(Mode m);    ///< "STBY", "AUTO", ...
const char* CommandName(Command c);

/** Data bytes of PGN 130850 for \p command to the pilot at \p address. */
std::vector<uint8_t> CommandData(Command command, uint8_t address);

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

/** The parts of an ISO NAME (PGN 60928) used to recognise a pilot. */
struct Name {
  uint64_t raw = 0;
  uint32_t unique = 0;
  uint16_t manufacturer = 0;
  uint8_t function = 0;
  uint8_t device_class = 0;
  bool IsAutopilot() const;
};
std::optional<Name> ParseAddressClaim(const std::vector<uint8_t>& d);

}  // namespace simnet

#endif  // NAVICO_AP_SIMNET_H
