/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   Talks to the pilot computer only through OpenCPN's own NMEA 2000 API
 *   (GetActiveDrivers, RegisterTXPGNs, WriteCommDriverN2K and listeners), so
 *   it works with any N2K connection OpenCPN supports. It sends one PGN
 *   130850 per button press and nothing else.
 ***************************************************************************/

#include "navico_autopilot_pi.h"

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>

#include <wx/aui/aui.h>
#include <wx/fileconf.h>
#include <wx/dir.h>
#include <wx/file.h>
#include <wx/filename.h>

#include "ap_panel.h"
#include "ap_prefs.h"
#include "config.h"
#include "pilot_settings.h"

namespace {

const char* const kConfigPath = "/PlugIns/NavicoAutopilot";
const char* const kPaneName = "NavicoAutopilot";
constexpr int kToolPosition = -1;

wxString PluginIcon(const char* base) {
  wxFileName f(GetPluginDataDir(PKG_NAME), "");
  f.AppendDir("data");
  f.SetName(base);
  f.SetExt("svg");
  return f.GetFullPath();
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
wxDEFINE_EVENT(EVT_NAVICO_AP_130845, ObservedEvt);
wxDEFINE_EVENT(EVT_NAVICO_AP_130846, ObservedEvt);

extern "C" DECL_EXP opencpn_plugin* create_pi(void* ppimgr) {
  return new NavicoAutopilotPi(ppimgr);
}

extern "C" DECL_EXP void destroy_pi(opencpn_plugin* p) { delete p; }

NavicoAutopilotPi::NavicoAutopilotPi(void* ppimgr)
    : opencpn_plugin_118(ppimgr), m_kv_timer(this), m_nfu_timer(this), m_timer(this) {}

NavicoAutopilotPi::~NavicoAutopilotPi() = default;

int NavicoAutopilotPi::Init() {
  AddLocaleCatalog("opencpn-" PKG_NAME);
  LoadConfig();

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
  listen(simnet::kPgnKeyValue, EVT_NAVICO_AP_130845);
  listen(simnet::kPgnKeyValueLong, EVT_NAVICO_AP_130846);

  Bind(wxEVT_TIMER, &NavicoAutopilotPi::OnTimer, this, m_timer.GetId());
  Bind(wxEVT_TIMER, &NavicoAutopilotPi::OnNfuTimer, this, m_nfu_timer.GetId());
  Bind(wxEVT_TIMER, &NavicoAutopilotPi::OnKvTimer, this, m_kv_timer.GetId());
  m_timer.Start(500);
  // Safety net for NFU: stop when OpenCPN loses focus.
  if (wxTheApp) wxTheApp->Bind(wxEVT_ACTIVATE_APP, &NavicoAutopilotPi::OnAppActivate, this);

  wxLogMessage("navico_autopilot_pi: started");
  return WANTS_TOOLBAR_CALLBACK | INSTALLS_TOOLBAR_TOOL | WANTS_PREFERENCES |
         WANTS_CONFIG | WANTS_NMEA_EVENTS;
}

bool NavicoAutopilotPi::DeInit() {
  StopNfu();
  if (wxTheApp) wxTheApp->Unbind(wxEVT_ACTIVATE_APP, &NavicoAutopilotPi::OnAppActivate, this);
  m_timer.Stop();
  m_nfu_timer.Stop();
  m_kv_timer.Stop();
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
  simnet::Mode before = m_state.Mode();
  auto target_before = m_state.TargetHeading();
  auto wind_before = m_state.TargetWindAngle();
  double now = Now();
  m_state.Handle(*msg, now);
  // Once per connection, ask the devices for their NAME, as the Triton app
  // does when it connects: one request, one frame back from each device.
  if (auto d = Driver(); d && d->handle != m_claims_driver) {
    m_claims_driver = d->handle;
    RequestAddressClaims();
  }
  // Shortly after the address claim request: ask pilots and rudder sensors
  // (only those) for their model ID, to tell a NAC-3's rudder feedback from
  // its virtual one. Not on network connections: OpenCPN's network driver
  // keeps 126996 to itself (HandleMgntMsg in comm_drv_n2k_net.cpp), so the
  // answers would never reach the plugin.
  auto drv = Driver();
  if (pgn == simnet::kPgnAddressClaim && now < m_discover_until && drv && !drv->network) {
    auto name = simnet::ParseAddressClaim(msg->data);
    if (name && (name->function == simnet::kFunctionAutopilot ||
                 name->function == simnet::kFunctionRudder) &&
        !m_product_asked.count(msg->source)) {
      m_product_asked.insert(msg->source);
      SendIsoRequest(simnet::kPgnProductInfo, msg->source);
    }
  }
  if (m_state.Mode() != before)
    wxLogMessage("navico_autopilot_pi: pilot mode %s -> %s",
                 simnet::ModeName(before), simnet::ModeName(m_state.Mode()));
  // Log target changes, so a course change can be checked afterwards.
  auto deg = [](std::optional<double> r) { return r ? *r * 57.29577951308232 : NAN; };
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
  std::vector<N2kDriver> out;
  for (const auto& handle : GetActiveDrivers()) {
    auto attr = GetAttributes(handle);
    auto p = attr.find("protocol");
    if (p == attr.end() || p->second != "nmea2000") continue;
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
    out.push_back({handle, label, attr.count("netAddress") > 0});
  }
  return out;
}

std::optional<N2kDriver> NavicoAutopilotPi::Driver() const {
  auto list = Drivers();
  for (const auto& d : list)
    if (d.handle == m_driver) return d;
  // Not chosen (or no longer there): only when there is exactly one, so a
  // command never goes out twice over two connections to the same bus.
  if (list.size() == 1) return list.front();
  return std::nullopt;
}

bool NavicoAutopilotPi::SendCommand(simnet::Command c) {
  double now = Now();
  auto driver = Driver();
  if (!driver) {
    m_last_message = _("No NMEA 2000 connection chosen.");
    return false;
  }
  auto address = m_state.Address(now);
  if (!address) {
    m_last_message = _("No autopilot selected.");
    return false;
  }
  Register(driver->handle);
  auto data = simnet::CommandData(c, *address);
  auto payload = std::make_shared<std::vector<uint8_t>>(data);
  CommDriverResult r = WriteCommDriverN2K(driver->handle, simnet::kPgnCommand,
                                          255, simnet::kCommandPriority, payload);
  wxLogMessage("navico_autopilot_pi: %s to pilot %d via %s: 130850 %s (result %d)",
               simnet::CommandName(c), *address, driver->label.c_str(),
               simnet::Hex(data).c_str(), r);
  if (r != RESULT_COMM_NO_ERROR) {
    m_last_message = wxString::Format(_("Sending failed (error %d)."), r);
    return false;
  }
  m_state.CommandSent(c, data, now);
  m_last_message.clear();
  return true;
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
  if (!driver) return;
  Register(driver->handle);
  auto data = simnet::IsoRequestData(pgn);
  auto payload = std::make_shared<std::vector<uint8_t>>(data);
  CommDriverResult r =
      WriteCommDriverN2K(driver->handle, simnet::kPgnIsoRequest, destination, 6, payload);
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
  Register(driver->handle);
  auto data = simnet::NfuData(side, *address);
  auto payload = std::make_shared<std::vector<uint8_t>>(data);
  CommDriverResult r = WriteCommDriverN2K(driver->handle, simnet::kPgnCommand, 255,
                                          simnet::kCommandPriority, payload);
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
    m_last_message = _("NFU could not be sent.");
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
  Register(driver->handle);
  auto data = simnet::KeyValueSet(address, key, raw, bytes, instance);
  auto payload = std::make_shared<std::vector<uint8_t>>(data);
  CommDriverResult r = WriteCommDriverN2K(driver->handle, simnet::kPgnKeyValue, 255, 3, payload);
  wxLogMessage("navico_autopilot_pi: set %02X/%04X instance %02X at %d to %u: 130845 %s (result %d)",
               key.group, key.param, instance, address, raw, simnet::Hex(data).c_str(), r);
  return r == RESULT_COMM_NO_ERROR;
}

void NavicoAutopilotPi::WritePilotSettings(const std::vector<SettingWrite>& writes) {
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
    auto s = pilot_settings::ParseSnapshot(text.ToStdString(wxConvUTF8));
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
    // busy); ask those once more at the end, as a display asks again.
    if (!answered && m_kv_retried.insert(*m_kv_waiting).second) m_kv_queue.push_back(*m_kv_waiting);
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
  Register(driver->handle);
  auto data = simnet::KeyValueRead(address, key, instance);
  auto payload = std::make_shared<std::vector<uint8_t>>(data);
  // 130845 is a PDU2 PGN: the device address is in the data (byte 2).
  WriteCommDriverN2K(driver->handle, simnet::kPgnKeyValue, 255, 3, payload);
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
  // OpenCPN calls SetActiveLegInfo about once a second while it navigates.
  return Now() - m_leg_time < 5 ? m_leg_name : wxString();
}

// ---- heading from OpenCPN ---------------------------------------------------

void NavicoAutopilotPi::SetPositionFixEx(PlugIn_Position_Fix_Ex& pfix) {
  m_hdt = pfix.Hdt;
  m_hdm = pfix.Hdm;
  m_var = pfix.Var;
  m_sog = pfix.Sog;
  m_heading_time = Now();
}

double NavicoAutopilotPi::Sog() const {
  return Now() - m_heading_time > 5 ? NAN : m_sog;
}

double NavicoAutopilotPi::Heading(bool magnetic) const {
  if (Now() - m_heading_time > 5) return NAN;
  double h = magnetic ? m_hdm : m_hdt;
  if (std::isnan(h) && !std::isnan(m_var)) {
    // Derive one from the other with OpenCPN's variation (east positive).
    h = magnetic ? m_hdt - m_var : m_hdm + m_var;
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
  conf->DeleteEntry("ControlEnabled", false);  // setting of 0.1.0 alpha, gone
  conf->Flush();
}
