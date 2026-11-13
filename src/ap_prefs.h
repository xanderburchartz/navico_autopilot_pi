/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#ifndef NAVICO_AP_PREFS_H
#define NAVICO_AP_PREFS_H

#include <cstdint>
#include <string>
#include <vector>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif
#include <wx/spinctrl.h>
#include <wx/timer.h>

class NavicoAutopilotPi;

/** Settings: N2K connection, which autopilot, which rudder sensor. */
class ApPrefs : public wxDialog {
public:
  ApPrefs(wxWindow* parent, NavicoAutopilotPi* plugin);

private:
  void OnPilotChoice(wxCommandEvent&);
  void OnOk(wxCommandEvent&);
  void FillPilotList();
  void FillRudderList();

  NavicoAutopilotPi* m_plugin;
  wxChoice* m_connection;
  std::vector<std::string> m_handles;
  wxChoice* m_pilot;
  std::vector<int> m_pilot_addresses;  // -1 automatic, -2 fixed (spin)
  int m_pilot_choice = -1;
  wxSpinCtrl* m_address;
  wxChoice* m_rudder;
  wxChoice* m_speed;
  std::vector<uint64_t> m_rudder_names;  // 0 = automatic
  uint64_t m_rudder_choice = 0;
  wxTimer m_refresh;
  int m_refreshes = 0;
};

#endif  // NAVICO_AP_PREFS_H
