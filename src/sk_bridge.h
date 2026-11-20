/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

/**
 * \file sk_bridge.h
 * The Signal K way: the pilot's state as a Signal K server reports it, and
 * the commands as Signal K PUT requests.
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

#include "simnet.h"

namespace sk_bridge {

/**
 * The mode for a Signal K state. \p auto_is_nodrift: the value comes from
 * signalk-simnet-pilot-state, which reports Simnet's No Drift as "auto".
 */
simnet::Mode ModeOf(const std::string& state, bool auto_is_nodrift);

/** The address at the end of a $source such as "can0.4" or "simnet-pilot-state.4". */
std::optional<uint8_t> SourceAddress(const std::string& source);

/** One PUT request: the path below /signalk/v1/api/vessels/self/ and its JSON value. */
struct Put {
  std::string path;
  std::string value;  ///< JSON, e.g. "\"heading\"" or "-10"
};

/** The request for \p c with signalk-simnet-pilot-state; none if it has no such command. */
std::optional<Put> SimnetPut(simnet::Command c);
/**
 * The request for \p c with signalk-autopilot; none if it has no such
 * command. A tack needs the side: from the wind angle held (\p wind, rad,
 * negative = wind from port, so the bow turns to port). No gybe: none with
 * the wind abaft the beam.
 */
std::optional<Put> AutopilotPut(simnet::Command c, std::optional<double> wind);
/** Non-follow-up (signalk-simnet-pilot-state only). */
Put NfuPut(simnet::NfuSide side);

/** The heading to steer and whether it is magnetic. */
struct Heading {
  std::optional<double> value;  ///< rad
  bool magnetic = true;
};
/**
 * Of steering.autopilot.target.headingMagnetic and .headingTrue, the one the
 * server got last (ISO 8601 timestamps): the REST API keeps a value forever,
 * so an old magnetic one must not win over a new true one. Equal or unknown
 * times: magnetic.
 */
Heading ChooseHeading(std::optional<double> magnetic, const std::string& magnetic_time,
                      std::optional<double> true_heading, const std::string& true_time);

}  // namespace sk_bridge

#endif  // NAVICO_AP_SK_BRIDGE_H
