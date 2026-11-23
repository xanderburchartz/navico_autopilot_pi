/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#ifndef NAVICO_AUTOPILOT_PI_H
#define NAVICO_AUTOPILOT_PI_H

#include <cstdint>  // ocpn_plugin.h uses uint8_t/uint64_t without it
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <string>
#include <vector>

#include <wx/wxprec.h>
#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif
#include <wx/timer.h>

#include "ocpn_plugin.h"
#include "alarm_notices.h"
#include "pilot_state.h"
#include "pilot_settings.h"
#include "route_info.h"
#include "simnet.h"
#include "translate.h"
#include "wrap0183.h"
#include "signalk_client.h"
#include "sk_bridge.h"

class ApPanel;

/**
 * One way to the pilot, as offered in the settings: an NMEA 2000 connection
 * of OpenCPN, an NMEA 0183 connection that carries NMEA 2000 wrapped in
 * $PCDIN or $MXPGN sentences, or the Signal K server from the plugin's own
 * settings (no OpenCPN connection, so no handle).
 */
struct N2kDriver {
  enum class Link { kN2k, kPcdin, kMxpgn, kSignalK };
  DriverHandle handle;
  std::string label;  ///< address/port or device, plus user comment
  /** Not a direct NMEA 2000 bus: a TCP/UDP gateway (OpenCPN keeps 126996 to
   *  itself) or wrapped sentences (no directed requests). */
  bool network = false;
  Link link = Link::kN2k;
  /** What the settings store: the handle, with "pcdin:" or "mxpgn:" before
   *  it for the wrapped ways, or "signalk". */
  std::string key;
  /** An NMEA 2000 network connection over UDP: OpenCPN receives on it but
   *  does not transmit (OpenCPN#5564), so it can only read the pilot. */
  bool udp = false;
};

// API 1.20 (OpenCPN 5.12) has notifications; the Android build stays on 1.18.
#if API_VERSION_MINOR >= 20
#define NAVICO_AP_NOTIFICATIONS 1
using NavicoPluginBase = opencpn_plugin_120;
#else
using NavicoPluginBase = opencpn_plugin_118;
#endif

class NavicoAutopilotPi : public wxEvtHandler, public NavicoPluginBase {
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
  void SetActiveLegInfo(Plugin_Active_Leg_Info& leg_info) override;
  /** OpenCPN's route messages (OCPN_RTE_ACTIVATED, OCPN_WPT_ARRIVED ...). */
  void SetPluginMessage(wxString& message_id, wxString& message_body) override;

  // Used by the panel and the settings dialog.
  bool SendCommand(simnet::Command c);
  /** Active pilot alarms not yet acknowledged; none through Signal K. */
  std::vector<PilotState::ActiveAlarm> AlarmsToAcknowledge() const;
  /** Acknowledge each of them on the pilot, as a Triton2 keypad does. Only
   *  on a user's action. */
  bool AcknowledgeAlarms();
  /** Confirm a pending NAV, as a Triton2 keypad does: 0x1C, then NAV. */
  void ConfirmNav();
  /** Ask all devices once for their NAME (ISO request for 60928). */
  void RequestAddressClaims();
  /** Non-follow-up: rudder moves while a button is held (repeated every
   *  0.5 s); StopNfu sends stop three times. */
  void StartNfu(simnet::NfuSide side);
  void StopNfu();
  bool NfuActive() const { return m_nfu != simnet::NfuSide::kStop; }
  /** Read all known pilot settings (130845), one at a time. */
  void ReadPilotSettings();
  /** True while settings requests are queued or waiting for an answer. */
  bool ReadingSettings() const { return !m_kv_queue.empty() || m_kv_waiting.has_value(); }
  /** Change one setting (130845 set, as a Triton2 sends it); the pilot
   *  answers with the value it took. */
  bool WritePilotSetting(uint8_t address, simnet::SettingKey key, uint32_t raw, int bytes,
                         uint8_t instance = 0xFF);
  /** One change of a restore: the pilot takes them one at a time. */
  struct SettingWrite {
    uint8_t address;
    simnet::SettingKey key;
    uint32_t raw;
    int bytes;
    uint8_t instance;
  };
  /** Send the changes one after the other (next after the answer or 1.5 s),
   *  then read the settings again. */
  void WritePilotSettings(const std::vector<SettingWrite>& writes);
  bool WritingSettings() const { return !m_kv_writes.empty() || m_kv_write_waiting.has_value(); }
  /** The settings as last answered, for a snapshot; \p missing counts the
   *  known settings without an answer. */
  pilot_settings::Snapshot TakeSnapshot(const std::string& reason, int* missing) const;
  /** Store a snapshot in OpenCPN's data folder; false if that failed. */
  bool SaveSnapshot(const pilot_settings::Snapshot& s);
  /** Stored snapshots, newest first. */
  std::vector<pilot_settings::Snapshot> Snapshots() const;
  /** Address holding the rudder calibration (the chosen rudder sensor). */
  std::optional<uint8_t> RudderNode() const;
  /** Show speed through the water (128259, the default) or OpenCPN's SOG. */
  bool UseStw() const { return m_use_stw; }
  /** Waypoint name of OpenCPN's own active leg, if OpenCPN navigates. */
  wxString ActiveWaypoint() const;
  /** Name of the waypoint the pilot steers to: from the plotter (129285)
   *  if it sends one, else OpenCPN's when OpenCPN navigates. */
  wxString DestinationName() const;
  /** "Arrived at ..." or "Route ... ended" for a short while; empty else. */
  wxString RouteNote() const;
  PI_ColorScheme ColorScheme() const { return m_scheme; }
  double Now() const;
  const PilotState& State() const { return m_state; }
  std::vector<N2kDriver> Drivers() const;
  /** The connection commands go to, if any. */
  std::optional<N2kDriver> Driver() const;
  /** A connection was chosen in the settings but OpenCPN no longer has it. */
  bool ChosenMissing() const {
    if (m_driver.empty()) return false;
    for (const auto& d : Drivers())
      if (d.key == m_driver) return false;
    return true;
  }
  /** Actual heading from OpenCPN (deg), magnetic or true, NaN if unknown. */
  double Heading(bool magnetic) const;
  /** Speed over ground (kn) from OpenCPN, NaN if unknown. */
  double Sog() const;
  wxString LastMessage() const { return m_last_message; }
  void ApplySettings(const std::string& driver, std::optional<uint8_t> address,
                     uint64_t rudder_name, bool use_stw);
  // ---- Signal K ----
  /** Server as "host:port" (empty if not set). */
  std::string SignalKServer() const;
  void SetSignalKServer(const std::string& host_port);
  /** Ask the server for a token; the server's administrator approves it. */
  void RequestSignalKAccess();
  /** Short state of the access for the settings ("token present", ...). */
  wxString SignalKAccessText() const;
  /** True when the chosen way is the Signal K server. */
  bool UsingSignalK() const;
  /** The pilot as the Signal K server names it ("Simnet", "simrad", ...),
   *  else "pilot": the server does not say the model. */
  wxString SignalKPilotLabel() const {
    return m_sk_provider.empty() ? wxString("pilot") : wxString::FromUTF8(m_sk_provider.c_str());
  }
  /** When the current way to the pilot was chosen or came up (Now() time). */
  double LinkSince() const { return m_link_since; }
  void PanelClosed();

private:
  void OnN2k(int pgn, ObservedEvt& ev);
  /** A $PCDIN or $MXPGN sentence from an NMEA 0183 connection. */
  void On0183(const char* id, ObservedEvt& ev);
  /** A received message, whichever way it came. */
  void HandleMessage(const simnet::Message& msg);
  /** Log a new target heading or wind angle while the pilot steers. */
  void LogTargetChanges(std::optional<double> target_before, std::optional<double> wind_before);
  /** Send one NMEA 2000 message the chosen way. */
  CommDriverResult SendN2k(const N2kDriver& driver, int pgn, uint8_t destination,
                           int priority, const std::vector<uint8_t>& data);
  wrap0183::Assembler m_mxpgn_in;
  std::string m_link_key;     ///< key of the way seen at the last timer tick
  N2kDriver::Link m_link = N2kDriver::Link::kN2k;  ///< its kind
  double m_link_since = 0;
  // Signal K way.
  std::unique_ptr<SignalKClient> m_sk;
  std::string m_sk_host;
  int m_sk_port = 3000;
  std::string m_sk_token;
  std::string m_sk_client_id;
  /** The server has signalk-simnet-pilot-state (actions.command and .nfu),
   *  seen in the state's sources; else commands go to signalk-autopilot. */
  bool m_sk_simnet = false;
  std::string m_sk_provider;  ///< "Simnet", or signalk-autopilot's defaultPilot
  std::string m_sk_state_time;  ///< timestamp of the last state seen
  std::string m_sk_request;     ///< href of a pending access request
  wxString m_sk_access;         ///< last access state, for the settings
  bool m_sk_polling = false;
  bool m_sk_access_polling = false;
  int m_sk_nfu_pending = 0;  ///< NFU requests sent, not yet answered
  double m_sk_access_last = 0;
  void PollSignalK();
  void OnSignalKStatus(const std::string& body);
  void PollAccessRequest();
  void SkPut(const sk_bridge::Put& put, simnet::Command c);
  uint8_t m_mxpgn_sequence = 0;
  void OnTimer(wxTimerEvent&);
  void LoadConfig();
  void SaveConfig();
  void ShowPanel(bool show);
  void Register(const std::string& handle);
  void SendIsoRequest(int pgn, uint8_t destination);
  void RememberPilotName();

  PilotState m_state;
  std::vector<std::shared_ptr<ObservableListener>> m_listeners;
  std::set<DriverHandle> m_registered;  // RegisterTXPGNs done
  std::string m_driver;                 // from the settings, may be empty
  wxString m_last_message;

  double m_hdt = NAN, m_hdm = NAN, m_var = NAN, m_sog = NAN;
  double m_heading_time = -1e9;
  double m_hdm_time = -1e9;

  bool SendNfu(simnet::NfuSide side, bool log);
  void OnKvTimer(wxTimerEvent&);
  /** Address, key, instance of a settings read. */
  using KvRead = std::tuple<uint8_t, simnet::SettingKey, uint8_t>;
  std::vector<KvRead> m_kv_queue;
  std::optional<KvRead> m_kv_waiting;
  std::map<KvRead, int> m_kv_retried;  // times asked again
  std::vector<SettingWrite> m_kv_writes;
  std::optional<SettingWrite> m_kv_write_waiting;
  double m_kv_write_sent = 0;
  double m_kv_sent = 0;
  wxTimer m_kv_timer;
  void OnNfuTimer(wxTimerEvent&);
  void OnAppActivate(wxActivateEvent& e);

  double m_discover_until = 0;
  std::string m_claims_driver;  // connection the address claims were requested on
  bool m_use_stw = true;
  simnet::NfuSide m_nfu = simnet::NfuSide::kStop;
  int m_nfu_stops_left = 0;
  wxTimer m_nfu_timer;
  std::set<uint8_t> m_product_asked;
  wxString m_leg_name;
  double m_leg_time = -1e9;
  route_info::OpenCpnRoute m_ocpn_route;
  void OnNavData(ObservedEvt& ev);
  void NotifyAlarms();
  std::map<uint16_t, std::string> m_notifications;  // alarm id -> notification guid
  AlarmNotices m_alarm_notices;

  wxBitmap m_bitmap;
  int m_tool_id = -1;
  ApPanel* m_panel = nullptr;
  bool m_panel_shown = false;
  PI_ColorScheme m_scheme = PI_GLOBAL_COLOR_SCHEME_DAY;
  wxTimer m_timer;
};

#endif  // NAVICO_AUTOPILOT_PI_H
