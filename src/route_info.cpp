/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "route_info.h"

namespace route_info {

std::optional<std::string> JsonString(const std::string& json, const std::string& key) {
  std::string quoted = "\"" + key + "\"";
  size_t pos = 0;
  while ((pos = json.find(quoted, pos)) != std::string::npos) {
    size_t p = pos + quoted.size();
    while (p < json.size() && (json[p] == ' ' || json[p] == '\t' || json[p] == '\n' ||
                               json[p] == '\r'))
      p++;
    if (p >= json.size() || json[p] != ':') {
      pos = p;
      continue;
    }
    p++;
    while (p < json.size() && (json[p] == ' ' || json[p] == '\t' || json[p] == '\n' ||
                               json[p] == '\r'))
      p++;
    if (p >= json.size() || json[p] != '"') return std::nullopt;
    std::string out;
    for (p++; p < json.size(); p++) {
      char c = json[p];
      if (c == '"') return out;
      if (c == '\\' && p + 1 < json.size()) {
        char e = json[++p];
        switch (e) {
          case 'n': out += '\n'; break;
          case 't': out += '\t'; break;
          case 'u':  // keep \uXXXX as is; names are mostly ASCII
            out += "\\u";
            break;
          default: out += e;
        }
        continue;
      }
      out += c;
    }
    return std::nullopt;
  }
  return std::nullopt;
}

bool Apply(OpenCpnRoute& r, const std::string& id, const std::string& body, double now) {
  auto get = [&](const char* key) { return JsonString(body, key).value_or(""); };
  if (id == "OCPN_RTE_ACTIVATED") {
    r.route = get("Route_activated");
    r.arrived.clear();
    r.ended.clear();
  } else if (id == "OCPN_WPT_ACTIVATED") {
    r.waypoint = get("WP_activated");
  } else if (id == "OCPN_WPT_ARRIVED") {
    r.arrived = get("WP_arrived");
    r.waypoint = get("Next_WP");
    r.event_time = now;
  } else if (id == "OCPN_RTE_DEACTIVATED") {
    r = OpenCpnRoute{};
  } else if (id == "OCPN_RTE_ENDED") {
    std::string name = get("Route_ended");
    r = OpenCpnRoute{};
    r.ended = name;
    r.event_time = now;
  } else {
    return false;
  }
  return true;
}

}  // namespace route_info
