/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

/**
 * \file sk_bridge.h
 * The Signal K way: what a Signal K server reports about the pilot, turned
 * into the NMEA 2000 messages the pilot itself sends, so PilotState and the
 * panel work the same as on the bus; and the commands as Signal K PUT
 * requests.
 *
 * Two kinds of server plugins are understood:
 *  - signalk-simnet-pilot-state: PUT steering.autopilot.actions.command
 *    ("standby", "heading", "wind", "nav", "tack", "+1", "-1", "+10", "-10")
 *    and steering.autopilot.actions.nfu ("port", "starboard", "stop").
 *    It reports Simnet's No Drift as "auto" and Heading as "heading".
 *  - signalk-autopilot (Simrad provider): PUT steering.autopilot.state,
 *    actions.adjustHeading (degrees) and actions.tack. Newer n2k-signalk
 *    reports Heading as "auto" and No Drift as "noDrift".
 *
 * No wxWidgets and no OpenCPN dependencies, so this can be unit tested.
 */
#ifndef NAVICO_AP_SK_BRIDGE_H
#define NAVICO_AP_SK_BRIDGE_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "simnet.h"

namespace sk_bridge {

/** What the server says about the pilot (values in SI units: radians). */
struct Status {
  std::optional<std::string> state;          ///< steering.autopilot.state
  std::optional<double> target_heading;      ///< steering.autopilot.target.headingMagnetic
  std::optional<double> target_wind;         ///< steering.autopilot.target.windAngleApparent
  std::optional<double> rudder;              ///< steering.rudderAngle
  std::optional<uint8_t> source;             ///< NMEA 2000 address in the state's $source
};

/**
 * The mode for a Signal K state. \p auto_is_nodrift: the server reports
 * Simnet's No Drift as "auto" (signalk-simnet-pilot-state, older n2k-signalk).
 */
simnet::Mode ModeOf(const std::string& state, bool auto_is_nodrift);

/** The address at the end of a $source such as "can0.4" or "simnet-pilot-state.4". */
std::optional<uint8_t> SourceAddress(const std::string& source);

/**
 * Messages as the pilot at \p address would send them: 65305 (mode),
 * 127237 (heading to steer, magnetic), 65341 (wind angle, in WIND only) and
 * 127245 (rudder). Nothing for a value the server did not report.
 */
std::vector<simnet::Message> ToMessages(const Status& s, uint8_t address, bool auto_is_nodrift);

/** One PUT request: the path below /signalk/v1/api/vessels/self/ and its JSON value. */
struct Put {
  std::string path;
  std::string value;  ///< JSON, e.g. "\"heading\"" or "-10"
};

/** The request for \p c with signalk-simnet-pilot-state; none if it has no such command. */
std::optional<Put> SimnetPut(simnet::Command c);
/** The request for \p c with signalk-autopilot; none if it has no such command. */
std::optional<Put> AutopilotPut(simnet::Command c);
/** Non-follow-up (signalk-simnet-pilot-state only). */
Put NfuPut(simnet::NfuSide side);

}  // namespace sk_bridge

#endif  // NAVICO_AP_SK_BRIDGE_H
