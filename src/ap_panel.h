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
class ApDisplay;
class NavConfirmDialog;

/**
 * The autopilot panel: a display drawn in the style of a Navico pilot
 * controller (heading arc, mode box, values, rudder bar), the mode and course
 * buttons, and a line for messages and alarms.
 */
class ApPanel : public wxPanel {
public:
  ApPanel(wxWindow* parent, NavicoAutopilotPi* plugin);
  ~ApPanel() override;
  /** Update display, buttons, messages and the NAV confirmation window. */
  void UpdateState(double now);
  /** Colours for OpenCPN's day, dusk or night scheme. */
  void ApplyColours(PI_ColorScheme cs);

private:
  wxWindow* AddButton(wxSizer* sizer, const wxString& label, simnet::Command c);
  void UpdateConfirmation(double now);
  void SetNfuButtons(bool nfu);
  void OpenTackDialog();

  NavicoAutopilotPi* m_plugin;
  ApDisplay* m_display;
  wxStaticText* m_message;
  std::map<simnet::Command, wxWindow*> m_buttons;
  NavConfirmDialog* m_confirm = nullptr;
  double m_confirm_handled = -1e9;  ///< AwaitingSince() of the last shown request
  bool m_nfu_buttons = false;
  wxWindow* m_settings_button = nullptr;
};

#endif  // NAVICO_AP_PANEL_H
