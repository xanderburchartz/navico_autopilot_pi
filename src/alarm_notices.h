/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#ifndef NAVICO_AP_ALARM_NOTICES_H
#define NAVICO_AP_ALARM_NOTICES_H

#include <cstdint>
#include <map>
#include <set>
#include <vector>

/** Which pilot alarms get an OpenCPN notification. An alarm that goes
 *  away keeps its notification for \p hold seconds, so an alarm that comes
 *  and goes every few seconds stays one notification instead of a new one
 *  each time. */
class AlarmNotices {
public:
  explicit AlarmNotices(double hold = 30.0) : m_hold(hold) {}

  struct Changes {
    std::vector<uint16_t> raise;  ///< new: raise a notification
    std::vector<uint16_t> clear;  ///< gone for \p hold s: acknowledge it
  };

  /** Feed the active alarms; returns what to raise and to take back. */
  Changes Update(const std::set<uint16_t>& active, double now) {
    Changes c;
    for (uint16_t id : active) {
      if (!m_last_seen.count(id)) c.raise.push_back(id);
      m_last_seen[id] = now;
    }
    for (auto it = m_last_seen.begin(); it != m_last_seen.end();) {
      if (!active.count(it->first) && now - it->second >= m_hold) {
        c.clear.push_back(it->first);
        it = m_last_seen.erase(it);
      } else {
        ++it;
      }
    }
    return c;
  }

private:
  double m_hold;
  std::map<uint16_t, double> m_last_seen;  // alarms with a notification
};

#endif  // NAVICO_AP_ALARM_NOTICES_H
