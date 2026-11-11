/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#ifndef NAVICO_AP_PANEL_H
#define NAVICO_AP_PANEL_H

#include <cstdint>  // before ocpn_plugin.h, see navico_autopilot_pi.h
#include <map>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include "ocpn_plugin.h"
#include "simnet.h"

class NavicoAutopilotPi;

/** The control panel: pilot mode, headings and the buttons. */
class ApPanel : public wxPanel {
public:
  ApPanel(wxWindow* parent, NavicoAutopilotPi* plugin);
  /** Update texts and which buttons can be pressed. */
  void UpdateState(double now);
  /** Colours for OpenCPN's day, dusk or night scheme. */
  void ApplyColours(PI_ColorScheme cs);

private:
  wxWindow* AddButton(wxSizer* sizer, const wxString& label, simnet::Command c);

  NavicoAutopilotPi* m_plugin;
  wxStaticText* m_mode;
  wxStaticText* m_pilot;
  wxStaticText* m_heading;
  wxStaticText* m_target;
  wxStaticText* m_message;
  std::map<simnet::Command, wxWindow*> m_buttons;
};

#endif  // NAVICO_AP_PANEL_H
