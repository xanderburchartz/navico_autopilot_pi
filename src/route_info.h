/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

/**
 * \file route_info.h
 * OpenCPN's route messages to plugins (OCPN_RTE_ACTIVATED, OCPN_WPT_ARRIVED
 * ...), sent as small JSON objects with string values. No wxWidgets, so it
 * can be unit tested.
 */
#ifndef NAVICO_AP_ROUTE_INFO_H
#define NAVICO_AP_ROUTE_INFO_H

#include <optional>
#include <string>

namespace route_info {

/** The string value of \p key in a flat JSON object, if there is one. */
std::optional<std::string> JsonString(const std::string& json, const std::string& key);

/** What OpenCPN's own navigation is doing, from its messages. */
struct OpenCpnRoute {
  std::string route;        ///< name of the active route, empty if none
  std::string waypoint;     ///< waypoint it steers to
  std::string arrived;      ///< waypoint last arrived at
  std::string ended;        ///< route that ended at its last waypoint
  double event_time = -1e9;  ///< when arrived or ended was set
};

/** Update \p r with message \p id / \p body received at \p now; false if
 *  the message is not a route message. */
bool Apply(OpenCpnRoute& r, const std::string& id, const std::string& body, double now);

}  // namespace route_info

#endif  // NAVICO_AP_ROUTE_INFO_H
