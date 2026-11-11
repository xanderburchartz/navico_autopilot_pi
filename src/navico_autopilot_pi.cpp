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
#include <cmath>
#include <cstdio>

#include <wx/aui/aui.h>
#include <wx/fileconf.h>
#include <wx/filename.h>

#include "ap_panel.h"
#include "ap_prefs.h"
#include "config.h"

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

extern "C" DECL_EXP opencpn_plugin* create_pi(void* ppimgr) {
  return new NavicoAutopilotPi(ppimgr);
}

extern "C" DECL_EXP void destroy_pi(opencpn_plugin* p) { delete p; }

NavicoAutopilotPi::NavicoAutopilotPi(void* ppimgr)
    : opencpn_plugin_118(ppimgr), m_timer(this) {}

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

  Bind(wxEVT_TIMER, &NavicoAutopilotPi::OnTimer, this);
  m_timer.Start(500);

  wxLogMessage("navico_autopilot_pi: started");
  return WANTS_TOOLBAR_CALLBACK | INSTALLS_TOOLBAR_TOOL | WANTS_PREFERENCES |
         WANTS_CONFIG | WANTS_NMEA_EVENTS;
}

bool NavicoAutopilotPi::DeInit() {
  m_timer.Stop();
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
  m_state.Handle(*msg, Now());
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
    out.push_back({handle, label});
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

void NavicoAutopilotPi::SendCommand(simnet::Command c) {
  double now = Now();
  auto driver = Driver();
  if (!driver) {
    m_last_message = _("No NMEA 2000 connection chosen.");
    return;
  }
  auto address = m_state.Address(now);
  if (!address) {
    m_last_message = _("No autopilot selected.");
    return;
  }
  if (!m_registered.count(driver->handle)) {
    std::vector<int> pgns = {simnet::kPgnCommand};
    CommDriverResult r = RegisterTXPGNs(driver->handle, pgns);
    if (r != RESULT_COMM_NO_ERROR)
      wxLogWarning("navico_autopilot_pi: RegisterTXPGNs failed (%d)", r);
    m_registered.insert(driver->handle);
  }
  auto data = simnet::CommandData(c, *address);
  auto payload = std::make_shared<std::vector<uint8_t>>(data);
  CommDriverResult r = WriteCommDriverN2K(driver->handle, simnet::kPgnCommand,
                                          255, simnet::kCommandPriority, payload);
  wxLogMessage("navico_autopilot_pi: %s to pilot %d via %s: 130850 %s (result %d)",
               simnet::CommandName(c), *address, driver->label.c_str(),
               simnet::Hex(data).c_str(), r);
  if (r != RESULT_COMM_NO_ERROR) {
    m_last_message = wxString::Format(_("Sending failed (error %d)."), r);
    return;
  }
  m_state.CommandSent(c, now);
  m_last_message.clear();
}

// ---- heading from OpenCPN ---------------------------------------------------

void NavicoAutopilotPi::SetPositionFixEx(PlugIn_Position_Fix_Ex& pfix) {
  m_hdt = pfix.Hdt;
  m_hdm = pfix.Hdm;
  m_var = pfix.Var;
  m_heading_time = Now();
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
                                      std::optional<uint8_t> address) {
  if (address != m_state.FixedAddress()) m_state.SetPilotName(0);
  m_driver = driver;
  m_state.SetFixedAddress(address);
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
  conf->DeleteEntry("ControlEnabled", false);  // setting of 0.1.0 alpha, gone
  conf->Flush();
}
