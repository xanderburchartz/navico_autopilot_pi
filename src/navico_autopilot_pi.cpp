/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   Talks to the pilot computer through OpenCPN's own connections: an
 *   NMEA 2000 connection (GetActiveDrivers, RegisterTXPGNs, WriteCommDriverN2K
 *   and listeners), or an NMEA 0183 connection to a gateway that passes
 *   NMEA 2000 wrapped in $PCDIN or $MXPGN sentences (wrap0183.h). Or through
 *   a Signal K server, with its own HTTP requests (signalk_client.h).
 *
 *   Commands and setting changes only on a user's action (one command per
 *   press; NFU repeats while held). Without one it sends only requests: once
 *   per connection, at start, an ISO request for the devices' NAMEs (on
 *   serial/SocketCAN also for the model ID of pilots and rudder sensors);
 *   never repeated by itself. The pilot settings are read while that window
 *   is open.
 *   It listens on all NMEA 2000 connections, but sends only on the chosen one.
 ***************************************************************************/

#include "navico_autopilot_pi.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include <wx/aui/aui.h>
#include <wx/fileconf.h>
#include <wx/dir.h>
#include <wx/file.h>
#include <wx/filename.h>
#include <wx/jsonreader.h>
#include <wx/jsonval.h>

#include "ap_panel.h"
#include "ap_prefs.h"
#include "config.h"
#include "translate.h"  // after config.h: PKG_SUMMARY uses _()
#include "pilot_settings.h"

namespace {

const char* const kConfigPath = "/PlugIns/NavicoAutopilot";
const char* const kPaneName = "NavicoAutopilot";
constexpr int kToolPosition = -1;
constexpr double kRadToDeg = 57.29577951308232;

wxString PluginIcon(const char* base) {
  wxFileName f(GetPluginDataDir(PKG_NAME), "");
  f.AppendDir("data");
  f.SetName(base);
  f.SetExt("svg");
  return f.GetFullPath();
}

std::optional<double> JsonNumber(const wxJSONValue& v) {
  if (v.IsDouble()) return v.AsDouble();
  if (v.IsInt()) return static_cast<double>(v.AsInt());
  if (v.IsLong()) return static_cast<double>(v.AsLong());
  return std::nullopt;
}

}  // namespace

wxDEFINE_EVENT(EVT_NAVICO_AP_65305, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_65341, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_127237, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_60928, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_130850, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_130851, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_130856, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_127245, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_130306, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_129284, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_126996, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_128259, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_129285, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_NAVDATA, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_130845, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_130846, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_PCDIN, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_MXPGN, ObservedEvt);

extern "C" DECL_EXP opencpn_plugin* create_pi(void* ppimgr) {
  return new NavicoAutopilotPi(ppimgr);
}

extern "C" DECL_EXP void destroy_pi(opencpn_plugin* p) { delete p; }

NavicoAutopilotPi::NavicoAutopilotPi(void* ppimgr)
    : NavicoPluginBase(ppimgr), m_kv_timer(this), m_nfu_timer(this), m_timer(this) {}

NavicoAutopilotPi::~NavicoAutopilotPi() = default;

int NavicoAutopilotPi::Init() {
  AddLocaleCatalog(NAVICO_AP_CATALOG);
  LoadConfig();
  m_sk = std::make_unique<SignalKClient>(this);
  m_sk_polling = false;  // the callbacks of an earlier client never ran
  m_sk_access_polling = false;
  m_sk_nfu_pending = 0;
  m_sk->SetServer(m_sk_host, m_sk_port, m_sk_token);

  wxString icon = PluginIcon("navico_autopilot_pi");
  wxString toggled = PluginIcon("navico_autopilot_pi_toggled");
  m_tool_id = InsertPlugInToolSVG(_("Autopilot"), icon, icon, toggled,
                                  wxITEM_CHECK, _("Navico autopilot"), "",
                                  nullptr, kToolPosition, 0, this);
  m_bitmap = GetBitmapFromSVGFile(PluginIcon("navico_autopilot_panel_icon"), 32, 32);

  auto listen = [this](int pgn, const wxEventTypeTag<ObservedEvt>& type) {
    m_listeners.push_back(GetListener(NMEA2000Id(pgn), type, this));
    Bind(type, [this, pgn](ObservedEvt& ev) { OnN2k(pgn, ev); });
  };
  listen(simnet::kPgnDeviceStatus, EVT_NAVICO_AP_65305);
  listen(simnet::kPgnApAngle, EVT_NAVICO_AP_65341);
  listen(simnet::kPgnHeadingControl, EVT_NAVICO_AP_127237);
  listen(simnet::kPgnAddressClaim, EVT_NAVICO_AP_60928);
  listen(simnet::kPgnCommand, EVT_NAVICO_AP_130850);       // alarms from the pilot
  listen(simnet::kPgnCommandReply, EVT_NAVICO_AP_130851);  // echo of a command
  listen(simnet::kPgnAlarmText, EVT_NAVICO_AP_130856);
  listen(simnet::kPgnRudder, EVT_NAVICO_AP_127245);
  listen(simnet::kPgnWind, EVT_NAVICO_AP_130306);
  listen(simnet::kPgnNavData, EVT_NAVICO_AP_129284);
  listen(simnet::kPgnProductInfo, EVT_NAVICO_AP_126996);
  listen(simnet::kPgnSpeed, EVT_NAVICO_AP_128259);
  listen(simnet::kPgnRouteInfo, EVT_NAVICO_AP_129285);
  // Heading, variation and SOG as OpenCPN decoded them (BasicNavDataMsg).
  m_listeners.push_back(GetListener(NavDataId(), EVT_NAVICO_AP_NAVDATA, this));
  Bind(EVT_NAVICO_AP_NAVDATA, &NavicoAutopilotPi::OnNavData, this);
  listen(simnet::kPgnKeyValue, EVT_NAVICO_AP_130845);
  listen(simnet::kPgnKeyValueLong, EVT_NAVICO_AP_130846);
  // NMEA 2000 wrapped in NMEA 0183, for gateways that pass it on.
  m_listeners.push_back(GetListener(NMEA0183Id("PCDIN"), EVT_NAVICO_AP_PCDIN, this));
  Bind(EVT_NAVICO_AP_PCDIN, [this](ObservedEvt& ev) { On0183("PCDIN", ev); });
  m_listeners.push_back(GetListener(NMEA0183Id("MXPGN"), EVT_NAVICO_AP_MXPGN, this));
  Bind(EVT_NAVICO_AP_MXPGN, [this](ObservedEvt& ev) { On0183("MXPGN", ev); });

  Bind(wxEVT_TIMER, &NavicoAutopilotPi::OnTimer, this, m_timer.GetId());
  Bind(wxEVT_TIMER, &NavicoAutopilotPi::OnNfuTimer, this, m_nfu_timer.GetId());
  Bind(wxEVT_TIMER, &NavicoAutopilotPi::OnKvTimer, this, m_kv_timer.GetId());
  m_timer.Start(500);
  // Safety net for NFU: stop when OpenCPN loses focus.
  if (wxTheApp) wxTheApp->Bind(wxEVT_ACTIVATE_APP, &NavicoAutopilotPi::OnAppActivate, this);

  wxLogMessage("navico_autopilot_pi: started");
  return WANTS_TOOLBAR_CALLBACK | INSTALLS_TOOLBAR_TOOL | WANTS_PREFERENCES |
         WANTS_CONFIG | WANTS_NMEA_EVENTS | WANTS_PLUGIN_MESSAGING;
}

bool NavicoAutopilotPi::DeInit() {
  StopNfu();
  if (wxTheApp) wxTheApp->Unbind(wxEVT_ACTIVATE_APP, &NavicoAutopilotPi::OnAppActivate, this);
  m_timer.Stop();
  m_nfu_timer.Stop();
  m_kv_timer.Stop();
  m_sk.reset();  // stops its thread; pending results are dropped
  m_listeners.clear();
  SaveConfig();
  if (m_panel) {
    wxAuiManager* aui = GetFrameAuiManager();
    if (aui) {
      aui->DetachPane(m_panel);
      aui->Update();
    }
    m_panel->Destroy();
    m_panel = nullptr;
  }
  return true;
}

int NavicoAutopilotPi::GetAPIVersionMajor() { return atoi(API_VERSION); }
int NavicoAutopilotPi::GetAPIVersionMinor() {
  std::string v(API_VERSION);
  size_t dot = v.find('.');
  return dot == std::string::npos ? 0 : atoi(v.substr(dot + 1).c_str());
}
int NavicoAutopilotPi::GetPlugInVersionMajor() { return PLUGIN_VERSION_MAJOR; }
int NavicoAutopilotPi::GetPlugInVersionMinor() { return PLUGIN_VERSION_MINOR; }
int NavicoAutopilotPi::GetPlugInVersionPatch() { return PLUGIN_VERSION_PATCH; }
int NavicoAutopilotPi::GetPlugInVersionPost() { return PLUGIN_VERSION_TWEAK; }
const char* NavicoAutopilotPi::GetPlugInVersionPre() { return PKG_PRERELEASE; }
const char* NavicoAutopilotPi::GetPlugInVersionBuild() { return PKG_BUILD_INFO; }
wxBitmap* NavicoAutopilotPi::GetPlugInBitmap() { return &m_bitmap; }
wxString NavicoAutopilotPi::GetCommonName() { return PLUGIN_API_NAME; }
// OpenCPN shows these in labels where "&" marks a keyboard shortcut, so
// "B&G" would appear as "BG"; double it there.
static wxString LabelText(wxString s) {
  s.Replace("&", "&&");
  return s;
}
wxString NavicoAutopilotPi::GetShortDescription() { return LabelText(PKG_SUMMARY); }
wxString NavicoAutopilotPi::GetLongDescription() { return LabelText(PKG_DESCRIPTION); }

double NavicoAutopilotPi::Now() const {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ---- bus ------------------------------------------------------------------

void NavicoAutopilotPi::OnN2k(int pgn, ObservedEvt& ev) {
  auto msg = simnet::ParseOcpnPayload(GetN2000Payload(NMEA2000Id(pgn), ev));
  if (!msg || msg->pgn != pgn) return;
  HandleMessage(*msg);
}

void NavicoAutopilotPi::On0183(const char* id, ObservedEvt& ev) {
  // The chosen way as the timer last saw it: a gateway passing every PGN sends
  // hundreds of sentences a second, too many to look up the connections for.
  bool mxpgn = std::string(id) == "MXPGN";
  if (m_link != (mxpgn ? N2kDriver::Link::kMxpgn : N2kDriver::Link::kPcdin)) return;
  std::string sentence = GetN0183Payload(NMEA0183Id(id), ev);
  std::optional<simnet::Message> msg;
  if (mxpgn) {
    auto frame = wrap0183::DecodeMxpgn(sentence);
    // The send bit marks a sentence meant for the bus: our own, echoed back.
    if (!frame || frame->send) return;
    msg = m_mxpgn_in.Add(*frame);
  } else {
    msg = wrap0183::DecodePcdin(sentence);
  }
  if (msg) HandleMessage(*msg);
}

void NavicoAutopilotPi::HandleMessage(const simnet::Message& msg) {
  int pgn = msg.pgn;
  simnet::Mode before = m_state.Mode();
  auto target_before = m_state.TargetHeading();
  auto wind_before = m_state.TargetWindAngle();
  double now = Now();
  m_state.Handle(msg, now);
  // Once per connection, ask the devices for their NAME, as the Triton app
  // does when it connects: one request, one frame back from each device.
  if (!m_link_key.empty() && m_link_key != m_claims_driver) {
    m_claims_driver = m_link_key;
    RequestAddressClaims();
  }
  // Shortly after the address claim request: ask pilots and rudder sensors
  // (only those) for their model ID, to tell a NAC-3's rudder feedback from
  // its virtual one. Not on network connections: OpenCPN's network driver
  // keeps 126996 to itself (HandleMgntMsg in comm_drv_n2k_net.cpp), so the
  // answers would never reach the plugin.
  if (pgn == simnet::kPgnAddressClaim && now < m_discover_until) {
    auto drv = Driver();
    auto name = simnet::ParseAddressClaim(msg.data);
    if (drv && !drv->network && name &&
        (name->function == simnet::kFunctionAutopilot ||
         name->function == simnet::kFunctionRudder) &&
        !m_product_asked.count(msg.source)) {
      m_product_asked.insert(msg.source);
      SendIsoRequest(simnet::kPgnProductInfo, msg.source);
    }
  }
  if (m_state.Mode() != before)
    wxLogMessage("navico_autopilot_pi: pilot mode %s -> %s",
                 simnet::ModeName(before), simnet::ModeName(m_state.Mode()));
  LogTargetChanges(target_before, wind_before);
}

void NavicoAutopilotPi::LogTargetChanges(std::optional<double> target_before,
                                         std::optional<double> wind_before) {
  // Log target changes, so a course change can be checked afterwards.
  auto deg = [](std::optional<double> r) { return r ? *r * kRadToDeg : NAN; };
  // Only while the pilot steers, and only when the whole degrees differ.
  bool steering = m_state.Mode() != simnet::Mode::kStandby &&
                  m_state.Mode() != simnet::Mode::kUnknown;
  auto changed = [steering](double a, double b) {
    if (!steering || std::isnan(b)) return false;
    return std::isnan(a) || std::lround(a) != std::lround(b);
  };
  auto text = [](double d) {
    return std::isnan(d) ? wxString("---") : wxString::Format("%ld", std::lround(d));
  };
  double t0 = deg(target_before), t1 = deg(m_state.TargetHeading());
  if (changed(t0, t1))
    wxLogMessage("navico_autopilot_pi: target heading %s -> %s %s", text(t0),
                 text(t1), m_state.TargetMagnetic() ? "M" : "T");
  double w0 = deg(wind_before), w1 = deg(m_state.TargetWindAngle());
  if (changed(w0, w1))
    wxLogMessage("navico_autopilot_pi: target wind angle %s -> %s", text(w0), text(w1));
}

std::vector<N2kDriver> NavicoAutopilotPi::Drivers() const {
  std::vector<N2kDriver> direct, wrapped;
  for (const auto& handle : GetActiveDrivers()) {
    auto attr = GetAttributes(handle);
    auto p = attr.find("protocol");
    if (p == attr.end() || (p->second != "nmea2000" && p->second != "nmea0183")) continue;
    std::string label;
    if (attr.count("netAddress"))
      label = attr["netAddress"] + ":" + attr["netPort"];
    else if (attr.count("commPort"))
      label = attr["commPort"];
    else {
      size_t sep = handle.find("!@!");
      label = sep == std::string::npos ? handle : handle.substr(sep + 3);
    }
    if (attr.count("userComment") && !attr["userComment"].empty())
      label += " (" + attr["userComment"] + ")";
    if (p->second == "nmea2000") {
      direct.push_back({handle, label, attr.count("netAddress") > 0, N2kDriver::Link::kN2k, handle});
      continue;
    }
    // An NMEA 0183 connection works only with a gateway that passes NMEA 2000
    // wrapped in these sentences on to the bus, in both directions.
    if (attr.count("ioDirection") && attr["ioDirection"] == "IN") continue;
    wrapped.push_back({handle, label + ", NMEA 0183 $PCDIN (SeaSmart)", true,
                       N2kDriver::Link::kPcdin, "pcdin:" + handle});
    wrapped.push_back({handle, label + ", NMEA 0183 $MXPGN (MiniPlex)", true,
                       N2kDriver::Link::kMxpgn, "mxpgn:" + handle});
  }
  direct.insert(direct.end(), wrapped.begin(), wrapped.end());
  // The Signal K server from the plugin's own settings (OpenCPN's Signal K
  // connection only reads, and plugins cannot send through it).
  std::string server = SignalKServer();
  direct.push_back({"", server.empty() ? std::string("Signal K server") : "Signal K server " + server,
                    true, N2kDriver::Link::kSignalK, "signalk"});
  return direct;
}

std::optional<N2kDriver> NavicoAutopilotPi::Driver() const {
  auto list = Drivers();
  for (const auto& d : list)
    if (d.key == m_driver) return d;
  // A chosen connection that is gone is not replaced by another one: that
  // could be a way to another bus. The panel says so (ChosenMissing).
  if (!m_driver.empty()) return std::nullopt;
  // Not chosen: only when there is exactly one NMEA 2000 connection, so a
  // command never goes out twice over two connections to the same bus. A
  // wrapped way is never chosen on its own.
  std::optional<N2kDriver> only;
  for (const auto& d : list) {
    if (d.link != N2kDriver::Link::kN2k) continue;
    if (only) return std::nullopt;
    only = d;
  }
  return only;
}

CommDriverResult NavicoAutopilotPi::SendN2k(const N2kDriver& driver, int pgn,
                                            uint8_t destination, int priority,
                                            const std::vector<uint8_t>& data) {
  auto write = [&driver](const std::string& sentence) {
    auto bytes = std::make_shared<std::vector<uint8_t>>(sentence.begin(), sentence.end());
    return WriteCommDriver(driver.handle, bytes);
  };
  switch (driver.link) {
    case N2kDriver::Link::kPcdin:
      // SeaSmart has no destination field: for PDU1 PGNs it is the low byte of the PGN.
      return write(wrap0183::EncodePcdin(pgn, 0, data, destination));
    case N2kDriver::Link::kMxpgn: {
      CommDriverResult r = RESULT_COMM_NO_ERROR;
      for (const auto& s : wrap0183::EncodeMxpgn(pgn, static_cast<uint8_t>(priority),
                                                destination, data, m_mxpgn_sequence)) {
        r = write(s);
        if (r != RESULT_COMM_NO_ERROR) break;
      }
      m_mxpgn_sequence = (m_mxpgn_sequence + 1) & 7;
      return r;
    }
    case N2kDriver::Link::kSignalK:
      // Raw NMEA 2000 does not go through a Signal K server.
      return RESULT_COMM_INVALID_PARMS;
    case N2kDriver::Link::kN2k:
    default: {
      Register(driver.handle);
      auto payload = std::make_shared<std::vector<uint8_t>>(data);
      return WriteCommDriverN2K(driver.handle, pgn, destination, priority, payload);
    }
  }
}

bool NavicoAutopilotPi::SendCommand(simnet::Command c) {
  double now = Now();
  auto driver = Driver();
  if (!driver) {
    m_last_message = _("No connection");
    return false;
  }
  auto address = m_state.Address(now);
  if (!address) {
    m_last_message = _("No autopilot");
    return false;
  }
  // A tack window stays open for 10 s: check again when it is confirmed, as
  // a pilot in Auto would take 0x11 as a course change by the tack angle.
  if (c == simnet::Command::kTack &&
      (!m_state.Alive(now) || m_state.Mode() != simnet::Mode::kWind)) {
    m_last_message = _("Not in Wind");
    return false;
  }
  if (driver->link == N2kDriver::Link::kSignalK) {
    std::optional<sk_bridge::Put> put;
    if (m_sk_simnet) put = sk_bridge::SimnetPut(c);
    // No Drift is not in signalk-simnet-pilot-state; signalk-autopilot has it.
    if (!put) put = sk_bridge::AutopilotPut(c, m_state.TargetWindAngle());
    if (!put) {
      m_last_message = c == simnet::Command::kConfirm
                           ? _("Confirm on pilot")
                           : _("Not via Signal K");
      return false;
    }
    SkPut(*put, c);
    m_state.CommandSent(c, simnet::CommandData(c, *address), now);
    m_last_message.clear();
    return true;
  }
  auto data = simnet::CommandData(c, *address);
  CommDriverResult r = SendN2k(*driver, simnet::kPgnCommand, 255, simnet::kCommandPriority, data);
  wxLogMessage("navico_autopilot_pi: %s to pilot %d via %s: 130850 %s (result %d)",
               simnet::CommandName(c), *address, driver->label.c_str(),
               simnet::Hex(data).c_str(), r);
  if (r != RESULT_COMM_NO_ERROR) {
    m_last_message = _("Send error");
    return false;
  }
  m_state.CommandSent(c, data, now);
  m_last_message.clear();
  return true;
}

std::vector<PilotState::ActiveAlarm> NavicoAutopilotPi::AlarmsToAcknowledge() const {
  std::vector<PilotState::ActiveAlarm> out;
  auto driver = Driver();
  if (!driver || driver->link == N2kDriver::Link::kSignalK) return out;
  for (const auto& a : m_state.Alarms(Now()))
    // Not before its text (130856): the panel shows only alarms with a text.
    if (a.state == 0x39 && !a.quiet && !a.text.empty()) out.push_back(a);
  return out;
}

bool NavicoAutopilotPi::AcknowledgeAlarms() {
  auto driver = Driver();
  auto alarms = AlarmsToAcknowledge();
  if (!driver || alarms.empty()) return false;
  bool ok = true;
  for (const auto& a : alarms) {
    auto data = simnet::AlarmAckData(a.id, a.f);
    CommDriverResult r =
        SendN2k(*driver, simnet::kPgnCommand, 255, simnet::kCommandPriority, data);
    wxLogMessage("navico_autopilot_pi: acknowledge alarm %d (%s) via %s: 130850 %s (result %d)",
                 a.id, a.text.c_str(), driver->label.c_str(), simnet::Hex(data).c_str(), r);
    if (r != RESULT_COMM_NO_ERROR) {
      m_last_message = _("Send error");
      ok = false;
    }
  }
  if (ok) m_last_message.clear();
  return ok;
}

void NavicoAutopilotPi::Register(const std::string& handle) {
  if (m_registered.count(handle)) return;
  std::vector<int> pgns = {simnet::kPgnCommand, simnet::kPgnIsoRequest, simnet::kPgnKeyValue};
  CommDriverResult r = RegisterTXPGNs(handle, pgns);
  if (r != RESULT_COMM_NO_ERROR)
    wxLogWarning("navico_autopilot_pi: RegisterTXPGNs failed (%d)", r);
  m_registered.insert(handle);
}

void NavicoAutopilotPi::SendIsoRequest(int pgn, uint8_t destination) {
  auto driver = Driver();
  if (!driver || driver->link == N2kDriver::Link::kSignalK) return;
  auto data = simnet::IsoRequestData(pgn);
  CommDriverResult r = SendN2k(*driver, simnet::kPgnIsoRequest, destination, 6, data);
  wxLogMessage("navico_autopilot_pi: request %d to %d via %s: 59904 %s (result %d)", pgn,
               destination, driver->label.c_str(), simnet::Hex(data).c_str(), r);
}

void NavicoAutopilotPi::RequestAddressClaims() {
  // Devices send their NAME (60928) only at start-up or when asked. The
  // settings need them to tell devices apart, so opening the settings asks
  // once: one request, one frame back from each device. Pilots and rudder
  // sensors that answer are then asked for their model ID (see OnN2k).
  m_discover_until = Now() + 3;
  m_product_asked.clear();
  SendIsoRequest(simnet::kPgnAddressClaim, 255);
}

bool NavicoAutopilotPi::SendNfu(simnet::NfuSide side, bool log) {
  auto driver = Driver();
  auto address = m_state.Address(Now());
  if (!driver || !address) return false;
  if (driver->link == N2kDriver::Link::kSignalK) {
    if (!m_sk_simnet) return false;  // signalk-autopilot has no NFU
    auto put = sk_bridge::NfuPut(side);
    m_sk_nfu_pending++;
    std::string path = "/signalk/v1/api/vessels/self/" + put.path;
    std::string body = "{\"value\":" + put.value + "}";
    auto done = [this](const SignalKClient::Response& r) {
      m_sk_nfu_pending--;
      if (r.status == 405 || r.status == 404)
        m_last_message = _("NFU not available");
      else if (r.status == 401 || r.status == 403)
        m_last_message = _("No access");
    };
    // A stop goes ahead of polls and of the movement it stops; one queued
    // direction is dropped, so the rudder does not move on after release.
    if (side == simnet::NfuSide::kStop)
      m_sk->RequestFirst("PUT", path, body, done);
    else
      m_sk->Request("PUT", path, body, done);
    if (log)
      wxLogMessage("navico_autopilot_pi: NFU via Signal K: PUT %s %s", put.path.c_str(),
                   put.value.c_str());
    return true;
  }
  auto data = simnet::NfuData(side, *address);
  CommDriverResult r = SendN2k(*driver, simnet::kPgnCommand, 255, simnet::kCommandPriority, data);
  if (log)  // not every repeat, to keep the log readable
    wxLogMessage("navico_autopilot_pi: NFU %s to pilot %d: 130850 %s (result %d)",
                 side == simnet::NfuSide::kStop ? "stop"
                 : side == simnet::NfuSide::kPort ? "port" : "starboard",
                 *address, simnet::Hex(data).c_str(), r);
  return r == RESULT_COMM_NO_ERROR;
}

void NavicoAutopilotPi::StartNfu(simnet::NfuSide side) {
  if (side == simnet::NfuSide::kStop) return StopNfu();
  // As the Triton2 keypad: first a stop, then the direction. The NAC-3
  // stops on its own about 2.5 s after the last command, so the direction is
  // repeated every 0.5 s while the button is held.
  SendNfu(simnet::NfuSide::kStop, false);
  if (!SendNfu(side, true)) {
    m_last_message = _("NFU failed");
    return;
  }
  m_nfu = side;
  m_nfu_stops_left = 0;
  m_nfu_timer.Start(500);
}

void NavicoAutopilotPi::StopNfu() {
  if (m_nfu == simnet::NfuSide::kStop && m_nfu_stops_left == 0) return;
  m_nfu = simnet::NfuSide::kStop;
  // Stop three times, 0.1 s apart: one lost message must not let the rudder run.
  SendNfu(simnet::NfuSide::kStop, true);
  m_nfu_stops_left = 2;
  m_nfu_timer.Start(100);
}

void NavicoAutopilotPi::OnNfuTimer(wxTimerEvent&) {
  if (m_nfu != simnet::NfuSide::kStop) {
    // Signal K: no repeat while one is unanswered, so a stop never waits
    // behind a queue of repeats on a slow server.
    if (m_sk_nfu_pending > 0) return;
    if (!SendNfu(m_nfu, false)) StopNfu();
    return;
  }
  if (m_nfu_stops_left > 0) {
    SendNfu(simnet::NfuSide::kStop, false);
    m_nfu_stops_left--;
  }
  if (m_nfu_stops_left == 0) m_nfu_timer.Stop();
}

void NavicoAutopilotPi::OnAppActivate(wxActivateEvent& e) {
  if (!e.GetActive()) StopNfu();
  e.Skip();
}

std::optional<uint8_t> NavicoAutopilotPi::RudderNode() const {
  double now = Now();
  if (m_state.RudderName()) {
    for (const auto& r : m_state.RudderSources(now))
      if (r.name && r.name->raw == m_state.RudderName()) return r.address;
  }
  return m_state.AutoRudder(now);
}

void NavicoAutopilotPi::ReadPilotSettings() {
  if (UsingSignalK()) {
    // The settings are Simnet key values (130845); no Signal K plugin passes them.
    m_last_message = _("Not via Signal K");
    return;
  }
  // Reading changes nothing on the pilot. Spread the requests so the bus and
  // the pilot get one at a time, as a display does when it opens a page.
  auto pilot = m_state.Address(Now());
  auto rudder = RudderNode();
  m_kv_retried.clear();
  auto add = [this](uint8_t address, simnet::SettingKey key, uint8_t instance) {
    KvRead item{address, key, instance};
    if (m_kv_waiting == item ||
        std::find(m_kv_queue.begin(), m_kv_queue.end(), item) != m_kv_queue.end())
      return;
    m_kv_queue.push_back(item);
  };
  std::vector<const pilot_settings::Definition*> profiles;
  for (const auto& d : pilot_settings::All()) {
    auto a = d.node == pilot_settings::Node::kPilot ? pilot : rudder;
    if (!a) continue;
    if (d.per_profile)
      profiles.push_back(&d);
    else
      add(*a, d.key, 0);
  }
  // Then the steering profiles, per instance.
  if (pilot) {
    for (const auto& prof : pilot_settings::Profiles())
      for (const auto* d : profiles) add(*pilot, d->key, prof.instance);
  }
  // One request at a time: the NAC-3 answers late and bundled when asked
  // quickly (13 Nov 2026), so the next one goes out after the answer or 1.5 s.
  if (!m_kv_queue.empty() && !m_kv_timer.IsRunning()) m_kv_timer.Start(100);
  wxLogMessage("navico_autopilot_pi: reading %zu pilot settings", m_kv_queue.size());
}

bool NavicoAutopilotPi::WritePilotSetting(uint8_t address, simnet::SettingKey key,
                                          uint32_t raw, int bytes, uint8_t instance) {
  auto driver = Driver();
  if (!driver) return false;
  auto data = simnet::KeyValueSet(address, key, raw, bytes, instance);
  CommDriverResult r = SendN2k(*driver, simnet::kPgnKeyValue, 255, 3, data);
  wxLogMessage("navico_autopilot_pi: set %02X/%04X instance %02X at %d to %u: 130845 %s (result %d)",
               key.group, key.param, instance, address, raw, simnet::Hex(data).c_str(), r);
  return r == RESULT_COMM_NO_ERROR;
}

void NavicoAutopilotPi::WritePilotSettings(const std::vector<SettingWrite>& writes) {
  if (UsingSignalK()) return;
  m_kv_writes.insert(m_kv_writes.end(), writes.begin(), writes.end());
  if (!m_kv_writes.empty() && !m_kv_timer.IsRunning()) m_kv_timer.Start(100);
  wxLogMessage("navico_autopilot_pi: restoring %zu pilot settings", writes.size());
}

pilot_settings::Snapshot NavicoAutopilotPi::TakeSnapshot(const std::string& reason,
                                                         int* missing) const {
  pilot_settings::Snapshot snap;
  snap.time = wxDateTime::Now().Format("%Y-%m-%d %H:%M:%S").ToStdString();
  snap.reason = reason;
  int lacking = 0;
  auto pilot = m_state.Address(Now());
  auto rudder = RudderNode();
  if (pilot)
    if (auto name = m_state.NameOf(*pilot)) snap.pilot = name->raw;
  for (const auto& d : pilot_settings::All()) {
    auto a = d.node == pilot_settings::Node::kPilot ? pilot : rudder;
    std::vector<uint8_t> instances;
    if (d.per_profile)
      for (const auto& p : pilot_settings::Profiles()) instances.push_back(p.instance);
    else
      instances.push_back(0);
    for (uint8_t inst : instances) {
      std::optional<simnet::KeyValue> kv;
      if (a) kv = d.per_profile ? m_state.Setting(*a, d.key, inst) : m_state.Setting(*a, d.key);
      std::optional<uint32_t> raw;
      if (kv) raw = kv->Unsigned(static_cast<size_t>(d.bytes));
      if (!raw) {
        lacking++;
        continue;
      }
      snap.entries.push_back({d.node, d.key, inst, *raw});
    }
  }
  if (missing) *missing = lacking;
  return snap;
}

static wxString SnapshotDir() {
  wxString dir = *GetpPrivateApplicationDataLocation();
  wxFileName fn(dir, "");
  fn.AppendDir("plugins");
  fn.AppendDir("navico_autopilot_pi");
  fn.AppendDir("snapshots");
  return fn.GetPath();
}

bool NavicoAutopilotPi::SaveSnapshot(const pilot_settings::Snapshot& s) {
  wxString dir = SnapshotDir();
  if (!wxFileName::DirExists(dir) && !wxFileName::Mkdir(dir, wxS_DIR_DEFAULT, wxPATH_MKDIR_FULL))
    return false;
  wxString name = wxString(s.time).Clone();
  name.Replace("-", "");
  name.Replace(":", "");
  name.Replace(" ", "-");
  wxFileName fn(dir, name, "txt");
  wxFile f;
  if (!f.Create(fn.GetFullPath(), true)) return false;
  bool ok = f.Write(wxString::FromUTF8(pilot_settings::Serialize(s).c_str()), wxConvUTF8);
  wxLogMessage("navico_autopilot_pi: snapshot %s (%zu values) %s", fn.GetFullPath(),
               s.entries.size(), ok ? "saved" : "not saved");
  return ok;
}

std::vector<pilot_settings::Snapshot> NavicoAutopilotPi::Snapshots() const {
  std::vector<pilot_settings::Snapshot> list;
  wxString dir = SnapshotDir();
  if (!wxFileName::DirExists(dir)) return list;
  wxArrayString files;
  wxDir::GetAllFiles(dir, &files, "*.txt", wxDIR_FILES);
  files.Sort(true);  // names are times: newest first
  for (const auto& path : files) {
    wxFile f(path);
    wxString text;
    if (!f.IsOpened() || !f.ReadAll(&text, wxConvUTF8)) continue;
    auto s = pilot_settings::ParseSnapshot(std::string(text.utf8_str()));
    if (s && !s->entries.empty()) list.push_back(*s);
  }
  return list;
}

void NavicoAutopilotPi::OnKvTimer(wxTimerEvent&) {
  auto driver = Driver();
  if ((m_kv_queue.empty() && !m_kv_waiting && !WritingSettings()) || !driver) {
    m_kv_queue.clear();
    m_kv_waiting.reset();
    m_kv_writes.clear();
    m_kv_write_waiting.reset();
    m_kv_timer.Stop();
    return;
  }
  double now = Now();
  // Changes first, one at a time; the pilot answers with the instance sent.
  if (m_kv_write_waiting) {
    const auto& w = *m_kv_write_waiting;
    bool answered = m_state.Setting(w.address, w.key, w.instance) &&
                    m_state.SettingTime(w.address, w.key, w.instance) >= m_kv_write_sent;
    if (!answered && now - m_kv_write_sent < 1.5) return;
    m_kv_write_waiting.reset();
  }
  if (!m_kv_writes.empty()) {
    SettingWrite w = m_kv_writes.front();
    m_kv_writes.erase(m_kv_writes.begin());
    m_kv_write_waiting = w;
    m_kv_write_sent = now;
    WritePilotSetting(w.address, w.key, w.raw, w.bytes, w.instance);
    if (m_kv_writes.empty()) ReadPilotSettings();  // to show what the pilot took
    return;
  }
  if (m_kv_queue.empty() && !m_kv_waiting) {
    m_kv_timer.Stop();
    return;
  }
  if (m_kv_waiting) {
    auto [wa, wk, wi] = *m_kv_waiting;
    bool answered = m_state.Setting(wa, wk, wi) && m_state.SettingTime(wa, wk, wi) >= m_kv_sent;
    if (!answered && now - m_kv_sent < 1.5) return;
    // The NAC-3 leaves some requests unanswered (more so while the bus is
    // busy); ask those again at the end, up to twice, as a display asks again.
    if (!answered && m_kv_retried[*m_kv_waiting]++ < 2) m_kv_queue.push_back(*m_kv_waiting);
    m_kv_waiting.reset();
  }
  if (m_kv_queue.empty()) {
    m_kv_timer.Stop();
    return;
  }
  auto [address, key, instance] = m_kv_queue.front();
  m_kv_queue.erase(m_kv_queue.begin());
  m_kv_waiting = KvRead{address, key, instance};
  m_kv_sent = now;
  auto data = simnet::KeyValueRead(address, key, instance);
  // 130845 is a PDU2 PGN: the device address is in the data (byte 2).
  SendN2k(*driver, simnet::kPgnKeyValue, 255, 3, data);
}

void NavicoAutopilotPi::ConfirmNav() {
  // Recorded from a Triton2 confirming the course to the waypoint: 0x1C,
  // directly followed by NAV; the NAC-3 echoed both and went to NAV.
  if (SendCommand(simnet::Command::kConfirm)) SendCommand(simnet::Command::kNav);
}

void NavicoAutopilotPi::SetActiveLegInfo(Plugin_Active_Leg_Info& leg_info) {
  m_leg_name = leg_info.wp_name;
  m_leg_time = Now();
}

wxString NavicoAutopilotPi::ActiveWaypoint() const {
  // OpenCPN calls SetActiveLegInfo about once a second while it navigates;
  // its route messages say at once which waypoint is next.
  if (Now() - m_leg_time < 5) return m_leg_name;
  if (!m_ocpn_route.route.empty()) return wxString::FromUTF8(m_ocpn_route.waypoint.c_str());
  return wxString();
}

// ---- heading from OpenCPN ---------------------------------------------------

void NavicoAutopilotPi::OnNavData(ObservedEvt& ev) {
  PluginNavdata nd = GetEventNavdata(ev);
  m_hdt = nd.hdt;
  m_var = nd.var;
  m_sog = nd.sog;
  m_heading_time = Now();
}

void NavicoAutopilotPi::SetPositionFixEx(PlugIn_Position_Fix_Ex& pfix) {
  // Only the magnetic heading as the sensor sent it; the rest comes with
  // the NavData event. Without it a pilot steering magnetic would depend on
  // OpenCPN knowing the variation.
  m_hdm = pfix.Hdm;
  m_hdm_time = Now();
}

void NavicoAutopilotPi::SetPluginMessage(wxString& message_id, wxString& message_body) {
  if (route_info::Apply(m_ocpn_route, message_id.ToStdString(),
                        std::string(message_body.utf8_str()), Now()))
    wxLogMessage("navico_autopilot_pi: %s %s", message_id, message_body);
}

wxString NavicoAutopilotPi::DestinationName() const {
  std::string plotter = m_state.DestinationName(Now());
  if (!plotter.empty()) return wxString::FromUTF8(plotter.c_str());
  return ActiveWaypoint();
}

wxString NavicoAutopilotPi::RouteNote() const {
  const auto& r = m_ocpn_route;
  if (Now() - r.event_time > 30) return "";
  if (!r.ended.empty())
    return wxString::Format(_("Route %s ended"), wxString::FromUTF8(r.ended.c_str()));
  if (!r.arrived.empty())
    return r.waypoint.empty()
               ? wxString::Format(_("%s reached"), wxString::FromUTF8(r.arrived.c_str()))
               : wxString::Format(_("%s reached, next %s"),
                                  wxString::FromUTF8(r.arrived.c_str()),
                                  wxString::FromUTF8(r.waypoint.c_str()));
  return "";
}

double NavicoAutopilotPi::Sog() const {
  return Now() - m_heading_time > 5 ? NAN : m_sog;
}

double NavicoAutopilotPi::Heading(bool magnetic) const {
  if (Now() - m_heading_time > 5) return NAN;
  double hdm = Now() - m_hdm_time < 5 ? m_hdm : NAN;
  double h = magnetic ? hdm : m_hdt;
  if (std::isnan(h) && !std::isnan(m_var)) {
    // Derive one from the other with OpenCPN's variation (east positive).
    h = magnetic ? m_hdt - m_var : hdm + m_var;
    if (!std::isnan(h)) h = std::fmod(h + 360.0, 360.0);
  }
  return h;
}

// ---- panel, settings ----------------------------------------------------------

void NavicoAutopilotPi::ShowPanel(bool show) {
  wxAuiManager* aui = GetFrameAuiManager();
  if (!aui) {
    wxLogWarning("navico_autopilot_pi: no AUI manager, cannot show the panel");
    return;
  }
  if (!m_panel) {
    // Parent and pane setup as the Dashboard plugin does it.
    m_panel = new ApPanel(GetOCPNCanvasWindow(), this);
    m_panel->ApplyColours(m_scheme);
    wxSize size = m_panel->GetBestSize();
    wxAuiPaneInfo info;
    info.Name(kPaneName)
        .Caption(_("Autopilot"))
        .CaptionVisible(true)
        .MinSize(size)
        .BestSize(size)
        .FloatingSize(size + wxSize(0, 2 * m_panel->GetCharHeight()))
        .FloatingPosition(100, 100)
        .Float()
        // Always floating: docked, wxAUI stretches it over the full width or
        // height of the chart window, possibly over the toolbar that would
        // show it again. A click on its caption with a small drag towards an
        // edge, or (wxWidgets 3.3, Windows) a double click, docks a dockable
        // pane.
        .Dockable(false)
        .CloseButton(true)
        .Hide();
    aui->AddPane(m_panel, info);
  }
  if (!show) StopNfu();
  aui->GetPane(m_panel).Show(show);
  aui->Update();
  m_panel_shown = show;
  SetToolbarItemState(m_tool_id, show);
}

void NavicoAutopilotPi::SetColorScheme(PI_ColorScheme cs) {
  m_scheme = cs;
  if (m_panel) m_panel->ApplyColours(cs);
}

void NavicoAutopilotPi::PanelClosed() {
  StopNfu();
  m_panel_shown = false;
  SetToolbarItemState(m_tool_id, false);
}

void NavicoAutopilotPi::OnToolbarToolCallback(int) { ShowPanel(!m_panel_shown); }

void NavicoAutopilotPi::OnTimer(wxTimerEvent&) {
  if (m_panel) {
    wxAuiManager* aui = GetFrameAuiManager();
    if (aui && m_panel_shown && !aui->GetPane(m_panel).IsShown()) PanelClosed();
    if (m_panel_shown) m_panel->UpdateState(Now());
  }
  RememberPilotName();
  NotifyAlarms();
  auto link = Driver();
  m_link = link ? link->link : N2kDriver::Link::kN2k;
  std::string key = link ? link->key : std::string();
  if (key != m_link_key) {
    m_link_key = key;
    m_link_since = Now();
  }
  if (m_link == N2kDriver::Link::kSignalK) PollSignalK();
  if (!m_sk_request.empty()) PollAccessRequest();
}

void NavicoAutopilotPi::NotifyAlarms() {
#ifdef NAVICO_AP_NOTIFICATIONS
  // Each alarm of the pilot once as an OpenCPN notification (the bell on the
  // chart), taken back when the pilot has not sent it for 30 s (an alarm that
  // comes and goes stays one notification). Acknowledging it in OpenCPN does
  // not acknowledge it on the pilot; the panel's button does.
  std::set<uint16_t> active;
  std::map<uint16_t, std::string> texts;
  for (const auto& a : m_state.Alarms(Now())) {
    if (a.text.empty() || a.quiet) continue;  // wait for the text (130856)
    active.insert(a.id);
    texts[a.id] = a.text;
  }
  AlarmNotices::Changes c = m_alarm_notices.Update(active, Now());
  for (uint16_t id : c.raise) {
    wxString text = wxString::Format(_("Autopilot: %s"), wxString::FromUTF8(texts[id].c_str()));
    m_notifications[id] =
        RaiseNotification(PI_NotificationSeverity::PI_kWarning, std::string(text.utf8_str()));
    wxLogMessage("navico_autopilot_pi: notification for alarm %d: %s", id, texts[id].c_str());
  }
  for (uint16_t id : c.clear) {
    auto it = m_notifications.find(id);
    if (it == m_notifications.end()) continue;
    AcknowledgeNotification(it->second);
    m_notifications.erase(it);
    wxLogMessage("navico_autopilot_pi: notification for alarm %d taken back", id);
  }
#endif
}

void NavicoAutopilotPi::RememberPilotName() {
  // Keep the NAME of a fixed pilot, so it is followed when its address
  // changes (also after a restart).
  if (!m_state.FixedAddress()) return;
  for (const auto& c : m_state.Candidates(Now())) {
    if (c.address == *m_state.FixedAddress() && c.name &&
        c.name->raw != m_state.PilotName()) {
      m_state.SetPilotName(c.name->raw);
      SaveConfig();
    }
  }
}

void NavicoAutopilotPi::ShowPreferencesDialog(wxWindow* parent) {
  ApPrefs dlg(parent, this);
  dlg.ShowModal();
}

void NavicoAutopilotPi::ApplySettings(const std::string& driver,
                                      std::optional<uint8_t> address,
                                      uint64_t rudder_name, bool use_stw) {
  m_use_stw = use_stw;
  if (address != m_state.FixedAddress()) m_state.SetPilotName(0);
  m_driver = driver;
  m_state.SetFixedAddress(address);
  m_state.SetRudderName(rudder_name);
  m_last_message.clear();
  SaveConfig();
  wxLogMessage("navico_autopilot_pi: settings: connection '%s', pilot %s",
               driver.c_str(),
               address ? std::to_string(*address).c_str() : "automatic");
}

void NavicoAutopilotPi::LoadConfig() {
  wxFileConfig* conf = GetOCPNConfigObject();
  if (!conf) return;
  conf->SetPath(kConfigPath);
  wxString driver;
  conf->Read("Connection", &driver, "");
  m_driver = driver.ToStdString();
  long address = -1;
  conf->Read("PilotAddress", &address, -1);
  if (address >= 0 && address <= 251)
    m_state.SetFixedAddress(static_cast<uint8_t>(address));
  wxString name;
  conf->Read("PilotName", &name, "");
  unsigned long long raw = 0;
  if (name.ToULongLong(&raw, 16)) m_state.SetPilotName(raw);
  wxString speed;
  conf->Read("SpeedSource", &speed, "SOG");
  m_use_stw = speed == "STW";
  wxString rudder;
  conf->Read("RudderName", &rudder, "");
  raw = 0;
  if (rudder.ToULongLong(&raw, 16)) m_state.SetRudderName(raw);
  wxString sk;
  conf->Read("SignalKServer", &sk, "");
  SetSignalKServer(sk.ToStdString());
  wxString token;
  conf->Read("SignalKToken", &token, "");
  m_sk_token = token.ToStdString();
  wxString id;
  conf->Read("SignalKClientId", &id, "");
  m_sk_client_id = id.ToStdString();
}

void NavicoAutopilotPi::SaveConfig() {
  wxFileConfig* conf = GetOCPNConfigObject();
  if (!conf) return;
  conf->SetPath(kConfigPath);
  conf->Write("Connection", wxString(m_driver));
  auto fixed = m_state.FixedAddress();
  conf->Write("PilotAddress", fixed ? static_cast<long>(*fixed) : -1L);
  conf->Write("PilotName", m_state.PilotName()
                               ? wxString::Format("%llx", static_cast<unsigned long long>(m_state.PilotName()))
                               : wxString());
  conf->Write("RudderName", m_state.RudderName()
                               ? wxString::Format("%llx", static_cast<unsigned long long>(m_state.RudderName()))
                               : wxString());
  conf->Write("SpeedSource", m_use_stw ? "STW" : "SOG");
  conf->Write("SignalKServer", wxString(SignalKServer()));
  conf->Write("SignalKToken", wxString(m_sk_token));
  conf->Write("SignalKClientId", wxString(m_sk_client_id));
  conf->DeleteEntry("ControlEnabled", false);  // setting of 0.1.0 alpha, gone
  conf->Flush();
}

// ---- Signal K -----------------------------------------------------------------

std::string NavicoAutopilotPi::SignalKServer() const {
  if (m_sk_host.empty()) return "";
  return m_sk_host + ":" + std::to_string(m_sk_port);
}

void NavicoAutopilotPi::SetSignalKServer(const std::string& host_port) {
  std::string hp = host_port;
  while (!hp.empty() && hp.back() == ' ') hp.pop_back();
  while (!hp.empty() && hp.front() == ' ') hp.erase(0, 1);
  // Accept "http://host:port/" as well.
  if (hp.rfind("http://", 0) == 0) hp = hp.substr(7);
  if (!hp.empty() && hp.back() == '/') hp.pop_back();
  std::string host = hp;
  int port = 3000;
  size_t colon = hp.rfind(':');
  if (colon != std::string::npos) {
    host = hp.substr(0, colon);
    long p = std::strtol(hp.substr(colon + 1).c_str(), nullptr, 10);
    if (p > 0 && p < 65536) port = static_cast<int>(p);
  }
  if (host != m_sk_host || port != m_sk_port) {
    m_sk_state_time.clear();
    m_sk_simnet = false;
  }
  m_sk_host = host;
  m_sk_port = port;
  if (m_sk) m_sk->SetServer(m_sk_host, m_sk_port, m_sk_token);
}

bool NavicoAutopilotPi::UsingSignalK() const {
  auto d = Driver();
  return d && d->link == N2kDriver::Link::kSignalK;
}

wxString NavicoAutopilotPi::SignalKAccessText() const {
  if (!m_sk_access.empty()) return m_sk_access;
  return m_sk_token.empty() ? _("No token (needed only with security)")
                            : _("Token present.");
}

void NavicoAutopilotPi::PollSignalK() {
  if (!m_sk || m_sk_host.empty() || m_sk_polling) return;
  m_sk_polling = true;
  m_sk->Request("GET", "/signalk/v1/api/vessels/self/steering", "",
                [this](const SignalKClient::Response& r) {
                  m_sk_polling = false;
                  if (r.status == 200) {
                    OnSignalKStatus(r.body);
                  } else if (r.status == 401 || r.status == 403) {
                    m_sk_access = _("Token needed: request access");
                  }
                });
}

void NavicoAutopilotPi::OnSignalKStatus(const std::string& body) {
  wxJSONReader reader;
  wxJSONValue root;
  if (reader.Parse(wxString::FromUTF8(body.c_str()), &root) > 0) return;
  wxJSONValue ap = root["autopilot"];
  wxJSONValue state = ap["state"];
  // signalk-simnet-pilot-state reports the state as source
  // "simnet-pilot-state.<address>". If it is there, take its value (it names
  // No Drift "auto") and send the commands to it; else to signalk-autopilot.
  wxString source = state["$source"].IsString() ? state["$source"].AsString() : wxString();
  wxJSONValue values = state["values"];
  wxArrayString names = values.GetMemberNames();
  for (const auto& name : names)
    if (name.StartsWith("simnet-pilot-state.")) source = name;
  m_sk_simnet = source.StartsWith("simnet-pilot-state.");
  wxJSONValue chosen = values.HasMember(source) ? values[source] : state;
  if (!chosen["value"].IsString()) return;
  // Only a state the server got again since the last poll: the REST API keeps
  // the last value forever, also when the pilot is gone.
  std::string time = std::string(chosen["timestamp"].AsString().utf8_str());
  if (time == m_sk_state_time) return;
  m_sk_state_time = time;
  auto address = sk_bridge::SourceAddress(std::string(source.utf8_str()));
  // signalk-autopilot reports as "simrad", without an address: take it from
  // the bus source of the same state, e.g. "can0.4".
  for (size_t i = 0; !address && i < names.size(); i++)
    address = sk_bridge::SourceAddress(std::string(names[i].utf8_str()));
  simnet::Mode mode =
      sk_bridge::ModeOf(std::string(chosen["value"].AsString().utf8_str()), m_sk_simnet);
  if (mode == simnet::Mode::kUnknown) return;  // e.g. "dodge"
  simnet::Mode before = m_state.Mode();
  auto target_before = m_state.TargetHeading();
  auto wind_before = m_state.TargetWindAngle();
  // A NAC-3 sends its heading to steer (127237) as true or magnetic, as the
  // compass is set up; the server has it under headingTrue or headingMagnetic.
  auto stamp = [](wxJSONValue v) {  // operator[] is not const
    return v["timestamp"].IsString() ? std::string(v["timestamp"].AsString().utf8_str())
                                     : std::string();
  };
  wxJSONValue mag = ap["target"]["headingMagnetic"], tru = ap["target"]["headingTrue"];
  auto target = sk_bridge::ChooseHeading(JsonNumber(mag["value"]), stamp(mag),
                                         JsonNumber(tru["value"]), stamp(tru));
  // Which pilot: signalk-simnet-pilot-state speaks Simnet; signalk-autopilot
  // names its provider (simrad, raymarine, pypilot, ...).
  if (m_sk_simnet)
    m_sk_provider = "Simnet";
  else if (ap["defaultPilot"]["value"].IsString())
    m_sk_provider = std::string(ap["defaultPilot"]["value"].AsString().utf8_str());
  else
    m_sk_provider.clear();
  m_state.HandleSignalK(address.value_or(0), mode, target.value, target.magnetic,
                        JsonNumber(ap["target"]["windAngleApparent"]["value"]),
                        JsonNumber(root["rudderAngle"]["value"]), Now());
  if (m_state.Mode() != before)
    wxLogMessage("navico_autopilot_pi: pilot mode %s -> %s (Signal K)", simnet::ModeName(before),
                 simnet::ModeName(m_state.Mode()));
  LogTargetChanges(target_before, wind_before);
}

void NavicoAutopilotPi::SkPut(const sk_bridge::Put& put, simnet::Command c) {
  std::string body = "{\"value\":" + put.value + "}";
  wxLogMessage("navico_autopilot_pi: %s via Signal K %s: PUT %s %s", simnet::CommandName(c),
               SignalKServer().c_str(), put.path.c_str(), put.value.c_str());
  m_sk->Request("PUT", "/signalk/v1/api/vessels/self/" + put.path, body,
                [this, put](const SignalKClient::Response& r) {
                  if (r.status >= 200 && r.status < 300) return;
                  if (r.status == 401 || r.status == 403)
                    m_last_message = _("No access");
                  else if (r.status == 0)
                    m_last_message = _("Server not reachable");
                  else if (r.status == SignalKClient::kNoAnswer)
                    // Sent, but no answer in time: pressing again could give
                    // the command twice (+10 becoming +20).
                    m_last_message = _("Unconfirmed");
                  else
                    m_last_message = wxString::Format(_("Refused (%d)"), r.status);
                  wxLogMessage("navico_autopilot_pi: Signal K answered %d to PUT %s: %s", r.status,
                               put.path.c_str(), r.body.substr(0, 200).c_str());
                });
}

void NavicoAutopilotPi::RequestSignalKAccess() {
  if (!m_sk || m_sk_host.empty()) {
    m_sk_access = _("Enter server first");
    return;
  }
  if (m_sk_client_id.empty()) {
    // A random id in UUID form, so the server knows it is the same client.
    std::random_device rd;
    m_sk_client_id = std::string(wxString::Format("%08x-%04x-4%03x-a%03x-%04x%08x", rd(),
                                                  rd() & 0xFFFF, rd() & 0xFFF, rd() & 0xFFF,
                                                  rd() & 0xFFFF, rd())
                                     .utf8_str());
    SaveConfig();
  }
  std::string body = "{\"clientId\":\"" + m_sk_client_id +
                     "\",\"description\":\"OpenCPN Navico autopilot plugin\","
                     "\"permissions\":\"readwrite\"}";
  m_sk_access = _("Asking the server...");
  m_sk->Request("POST", "/signalk/v1/access/requests", body,
                [this](const SignalKClient::Response& r) {
                  wxJSONReader reader;
                  wxJSONValue v;
                  reader.Parse(wxString::FromUTF8(r.body.c_str()), &v);
                  if ((r.status == 202 || r.status == 200) && v["href"].IsString()) {
                    m_sk_request = std::string(v["href"].AsString().utf8_str());
                    m_sk_access = _("Approve in server: Security, Access Requests");
                  } else if (r.status <= 0) {
                    m_sk_access = _("Server not reachable");
                  } else {
                    m_sk_access = wxString::Format(_("Refused (%d)"), r.status);
                  }
                  wxLogMessage("navico_autopilot_pi: Signal K access request: %d %s", r.status,
                               r.body.substr(0, 200).c_str());
                });
}

void NavicoAutopilotPi::PollAccessRequest() {
  // Every 2 s, one request at a time (the status polls keep the client busy).
  if (!m_sk || m_sk_access_polling) return;
  double now = Now();
  if (now - m_sk_access_last < 2) return;
  m_sk_access_last = now;
  m_sk_access_polling = true;
  std::string href = m_sk_request;
  m_sk->Request("GET", href, "", [this, href](const SignalKClient::Response& r) {
    m_sk_access_polling = false;
    if (href != m_sk_request) return;
    wxJSONReader reader;
    wxJSONValue v;
    reader.Parse(wxString::FromUTF8(r.body.c_str()), &v);
    if (r.status == 404) {
      m_sk_request.clear();
      m_sk_access = _("Expired, ask again");
      return;
    }
    if (!v["state"].IsString() || v["state"].AsString() != "COMPLETED") return;
    m_sk_request.clear();
    wxJSONValue ar = v["accessRequest"];
    if (ar["permission"].IsString() && ar["permission"].AsString() == "APPROVED" &&
        ar["token"].IsString()) {
      m_sk_token = std::string(ar["token"].AsString().utf8_str());
      m_sk->SetServer(m_sk_host, m_sk_port, m_sk_token);
      m_sk_access = _("Access granted");
      SaveConfig();
      wxLogMessage("navico_autopilot_pi: Signal K access granted");
    } else {
      m_sk_access = _("Access denied");
      wxLogMessage("navico_autopilot_pi: Signal K access denied");
    }
  });
}
