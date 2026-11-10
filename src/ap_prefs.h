/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#ifndef NAVICO_AP_PREFS_H
#define NAVICO_AP_PREFS_H

#include <string>
#include <vector>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif
#include <wx/spinctrl.h>

class NavicoAutopilotPi;

/** Settings: N2K connection, which autopilot, control on/off. */
class ApPrefs : public wxDialog {
public:
  ApPrefs(wxWindow* parent, NavicoAutopilotPi* plugin);

private:
  void OnPilotChoice(wxCommandEvent&);
  void OnOk(wxCommandEvent&);

  NavicoAutopilotPi* m_plugin;
  wxChoice* m_connection;
  std::vector<std::string> m_handles;
  wxChoice* m_pilot;
  std::vector<int> m_pilot_addresses;  // -1 automatic, -2 fixed (spin)
  wxSpinCtrl* m_address;
};

#endif  // NAVICO_AP_PREFS_H
