/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "ap_prefs.h"

#include "navico_autopilot_pi.h"

namespace {
constexpr int kAutomatic = -1;
constexpr int kFixed = -2;
}  // namespace

ApPrefs::ApPrefs(wxWindow* parent, NavicoAutopilotPi* plugin)
    : wxDialog(parent, wxID_ANY, _("Navico autopilot settings"), wxDefaultPosition,
               wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER),
      m_plugin(plugin) {
  const PilotState& state = plugin->State();
  auto* top = new wxBoxSizer(wxVERTICAL);
  auto* grid = new wxFlexGridSizer(2, 6, 8);
  grid->AddGrowableCol(1);

  // Connection.
  grid->Add(new wxStaticText(this, wxID_ANY, _("NMEA 2000 connection")), 0,
            wxALIGN_CENTRE_VERTICAL);
  m_connection = new wxChoice(this, wxID_ANY);
  auto current = plugin->Driver();
  for (const auto& d : plugin->Drivers()) {
    m_connection->Append(wxString::FromUTF8(d.label.c_str()));
    m_handles.push_back(d.handle);
    if (current && current->handle == d.handle)
      m_connection->SetSelection(m_connection->GetCount() - 1);
  }
  if (m_handles.empty()) {
    m_connection->Append(_("(no NMEA 2000 connection in OpenCPN)"));
    m_connection->SetSelection(0);
    m_connection->Disable();
  }
  grid->Add(m_connection, 1, wxEXPAND);

  // Pilot and rudder sensor: the most likely device is used automatically and
  // named in the first entry; choosing another entry overrules it. The
  // lists are filled again once the devices have answered the requests below.
  grid->Add(new wxStaticText(this, wxID_ANY, _("Autopilot")), 0, wxALIGN_CENTRE_VERTICAL);
  m_pilot = new wxChoice(this, wxID_ANY);
  auto fixed = state.FixedAddress();
  m_pilot_choice = fixed ? *fixed : kAutomatic;
  m_pilot->Bind(wxEVT_CHOICE, &ApPrefs::OnPilotChoice, this);
  grid->Add(m_pilot, 1, wxEXPAND);

  grid->Add(new wxStaticText(this, wxID_ANY, _("Address")), 0, wxALIGN_CENTRE_VERTICAL);
  m_address = new wxSpinCtrl(this, wxID_ANY, "", wxDefaultPosition, wxDefaultSize,
                             wxSP_ARROW_KEYS, 0, 251, fixed ? *fixed : 0);
  grid->Add(m_address, 0);

  // Several devices may send 127245 (a pilot's real and virtual rudder
  // feedback, an uncalibrated sensor) with the same manufacturer and
  // function; a choice is stored by NAME.
  grid->Add(new wxStaticText(this, wxID_ANY, _("Rudder angle from")), 0,
            wxALIGN_CENTRE_VERTICAL);
  m_rudder = new wxChoice(this, wxID_ANY);
  m_rudder_choice = state.RudderName();
  m_rudder->Bind(wxEVT_CHOICE, [this](wxCommandEvent&) {
    int r = m_rudder->GetSelection();
    m_rudder_choice = r >= 0 ? m_rudder_names[r] : 0;
  });
  grid->Add(m_rudder, 1, wxEXPAND);

  grid->Add(new wxStaticText(this, wxID_ANY, _("Boat speed")), 0, wxALIGN_CENTRE_VERTICAL);
  m_speed = new wxChoice(this, wxID_ANY);
  m_speed->Append(_("Speed over ground (from OpenCPN)"));
  m_speed->Append(_("Speed through water (log)"));
  m_speed->SetSelection(plugin->UseStw() ? 1 : 0);
  grid->Add(m_speed, 1, wxEXPAND);
  top->Add(grid, 0, wxEXPAND | wxALL, 10);
  FillPilotList();
  FillRudderList();

  // Ask the devices for their NAME (and pilots and rudder sensors for their
  // model ID), then fill the lists again.
  plugin->RequestAddressClaims();
  m_refresh.SetOwner(this);
  Bind(wxEVT_TIMER, [this](wxTimerEvent&) {
    FillPilotList();
    FillRudderList();
    if (++m_refreshes < 3) m_refresh.StartOnce(1200);
  });
  m_refresh.StartOnce(1200);

  top->Add(CreateButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, 10);
  Bind(wxEVT_BUTTON, &ApPrefs::OnOk, this, wxID_OK);
  SetSizerAndFit(top);
  CentreOnParent();
}

void ApPrefs::OnPilotChoice(wxCommandEvent&) {
  int sel = m_pilot->GetSelection();
  int a = sel >= 0 ? m_pilot_addresses[sel] : kAutomatic;
  m_pilot_choice = a;
  if (a >= 0) m_address->SetValue(a);
  m_address->Enable(a == kFixed);
}

namespace {

wxString PilotLabel(const PilotState::Candidate& c) {
  wxString model = c.model_id.empty() ? wxString(c.model == 0x64 ? "NAC" : "AC")
                                      : wxString::FromUTF8(c.model_id.c_str());
  if (c.name)
    model += " (" + wxString::FromUTF8(c.name->ManufacturerName().c_str()) + ")";
  return wxString::Format(_("%s, address %d"), model, c.address);
}

wxString RudderLabel(const PilotState::RudderSource& r) {
  wxString what = !r.model_id.empty() ? wxString::FromUTF8(r.model_id.c_str())
                  : r.name ? wxString::Format(_("%s, unique number %u"),
                                              wxString::FromUTF8(r.name->ManufacturerName().c_str()),
                                              r.name->unique)
                           : wxString(_("Unknown device"));
  return wxString::Format(_("%s, address %d: %.1f deg"), what, r.address,
                          r.angle * 57.29577951308232);
}

}  // namespace

void ApPrefs::FillPilotList() {
  const PilotState& state = m_plugin->State();
  double now = m_plugin->Now();
  auto list = state.Candidates(now);
  auto best = state.AutoPilot(now);
  m_pilot->Clear();
  m_pilot_addresses.clear();
  wxString automatic = _("Automatic: no autopilot found yet");
  for (const auto& c : list)
    if (best && c.address == *best)
      automatic = wxString::Format(_("%s (automatic)"), PilotLabel(c));
  m_pilot->Append(automatic);
  m_pilot_addresses.push_back(kAutomatic);
  int sel = 0;
  for (const auto& c : list) {
    m_pilot->Append(PilotLabel(c));
    m_pilot_addresses.push_back(c.address);
    if (m_pilot_choice == c.address) sel = m_pilot->GetCount() - 1;
  }
  m_pilot->Append(_("Fixed address"));
  m_pilot_addresses.push_back(kFixed);
  if (m_pilot_choice == kFixed || (m_pilot_choice >= 0 && sel == 0))
    sel = m_pilot->GetCount() - 1;
  m_pilot->SetSelection(sel);
  m_address->Enable(m_pilot_addresses[sel] == kFixed);
  Layout();
  Fit();
}

void ApPrefs::OnOk(wxCommandEvent& ev) {
  std::string handle;
  int c = m_connection->GetSelection();
  if (c >= 0 && c < static_cast<int>(m_handles.size())) handle = m_handles[c];
  std::optional<uint8_t> address;
  int a = m_pilot_choice;
  if (a >= 0)
    address = static_cast<uint8_t>(a);
  else if (a == kFixed)
    address = static_cast<uint8_t>(m_address->GetValue());
  m_plugin->ApplySettings(handle, address, m_rudder_choice, m_speed->GetSelection() == 1);
  ev.Skip();  // closes the dialog
}

void ApPrefs::FillRudderList() {
  const PilotState& state = m_plugin->State();
  double now = m_plugin->Now();
  auto list = state.RudderSources(now);
  auto best = state.AutoRudder(now);
  m_rudder->Clear();
  m_rudder_names.clear();
  wxString automatic = _("Automatic: no rudder sensor found yet");
  for (const auto& r : list)
    if (best && r.address == *best)
      automatic = wxString::Format(_("%s (automatic)"), RudderLabel(r));
  m_rudder->Append(automatic);
  m_rudder_names.push_back(0);
  int sel = 0;
  for (const auto& r : list) {
    if (!r.name) continue;  // no address claim seen yet: cannot be stored
    m_rudder->Append(RudderLabel(r));
    m_rudder_names.push_back(r.name->raw);
    if (r.name->raw == m_rudder_choice) sel = m_rudder->GetCount() - 1;
  }
  if (m_rudder_choice && sel == 0) {
    // Chosen sensor not on the bus right now: keep the choice.
    m_rudder->Append(_("The chosen sensor (not seen now)"));
    m_rudder_names.push_back(m_rudder_choice);
    sel = m_rudder->GetCount() - 1;
  }
  m_rudder->SetSelection(sel);
  Layout();
  Fit();
}
