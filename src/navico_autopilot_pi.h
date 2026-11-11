/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#ifndef NAVICO_AUTOPILOT_PI_H
#define NAVICO_AUTOPILOT_PI_H

#include <cstdint>  // ocpn_plugin.h (api-18) uses uint8_t/uint64_t without it
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif
#include <wx/timer.h>

#include "ocpn_plugin.h"
#include "pilot_state.h"
#include "simnet.h"

class ApPanel;

/** One NMEA 2000 connection of OpenCPN, as offered in the settings. */
struct N2kDriver {
  DriverHandle handle;
  std::string label;  ///< address/port or device, plus user comment
};

class NavicoAutopilotPi : public wxEvtHandler, public opencpn_plugin_118 {
public:
  explicit NavicoAutopilotPi(void* ppimgr);
  ~NavicoAutopilotPi() override;

  int Init() override;
  bool DeInit() override;

  int GetAPIVersionMajor() override;
  int GetAPIVersionMinor() override;
  int GetPlugInVersionMajor() override;
  int GetPlugInVersionMinor() override;
  int GetPlugInVersionPatch() override;
  int GetPlugInVersionPost() override;
  const char* GetPlugInVersionPre() override;
  const char* GetPlugInVersionBuild() override;
  wxBitmap* GetPlugInBitmap() override;
  wxString GetCommonName() override;
  wxString GetShortDescription() override;
  wxString GetLongDescription() override;

  int GetToolbarToolCount() override { return 1; }
  void OnToolbarToolCallback(int id) override;
  void ShowPreferencesDialog(wxWindow* parent) override;
  void SetPositionFixEx(PlugIn_Position_Fix_Ex& pfix) override;
  void SetColorScheme(PI_ColorScheme cs) override;

  // Used by the panel and the settings dialog.
  void SendCommand(simnet::Command c);
  double Now() const;
  const PilotState& State() const { return m_state; }
  std::vector<N2kDriver> Drivers() const;
  /** The connection commands go to, if any. */
  std::optional<N2kDriver> Driver() const;
  /** Actual heading from OpenCPN (deg), magnetic or true, NaN if unknown. */
  double Heading(bool magnetic) const;
  wxString LastMessage() const { return m_last_message; }
  void ApplySettings(const std::string& driver, std::optional<uint8_t> address);
  void PanelClosed();

private:
  void OnN2k(int pgn, ObservedEvt& ev);
  void OnTimer(wxTimerEvent&);
  void LoadConfig();
  void SaveConfig();
  void ShowPanel(bool show);
  void RememberPilotName();

  PilotState m_state;
  std::vector<std::shared_ptr<ObservableListener>> m_listeners;
  std::set<DriverHandle> m_registered;  // RegisterTXPGNs done
  std::string m_driver;                 // from the settings, may be empty
  wxString m_last_message;

  double m_hdt = NAN, m_hdm = NAN, m_var = NAN;
  double m_heading_time = -1e9;

  wxBitmap m_bitmap;
  int m_tool_id = -1;
  ApPanel* m_panel = nullptr;
  bool m_panel_shown = false;
  PI_ColorScheme m_scheme = PI_GLOBAL_COLOR_SCHEME_DAY;
  wxTimer m_timer;
};

#endif  // NAVICO_AUTOPILOT_PI_H
