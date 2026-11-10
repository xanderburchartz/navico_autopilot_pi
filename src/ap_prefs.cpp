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
  double now = plugin->Now();
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

  // Pilot.
  grid->Add(new wxStaticText(this, wxID_ANY, _("Autopilot")), 0, wxALIGN_CENTRE_VERTICAL);
  m_pilot = new wxChoice(this, wxID_ANY);
  m_pilot->Append(_("Automatic (only if there is exactly one)"));
  m_pilot_addresses.push_back(kAutomatic);
  auto fixed = state.FixedAddress();
  int select = 0;
  for (const auto& c : state.Candidates(now)) {
    wxString label = wxString::Format(_("%s at address %d"),
                                      c.model == 0x64 ? "NAC" : "AC", c.address);
    m_pilot->Append(label);
    m_pilot_addresses.push_back(c.address);
    if (fixed && *fixed == c.address) select = m_pilot->GetCount() - 1;
  }
  m_pilot->Append(_("Fixed address"));
  m_pilot_addresses.push_back(kFixed);
  if (fixed && select == 0) select = m_pilot->GetCount() - 1;
  m_pilot->SetSelection(select);
  m_pilot->Bind(wxEVT_CHOICE, &ApPrefs::OnPilotChoice, this);
  grid->Add(m_pilot, 1, wxEXPAND);

  grid->Add(new wxStaticText(this, wxID_ANY, _("Address")), 0, wxALIGN_CENTRE_VERTICAL);
  m_address = new wxSpinCtrl(this, wxID_ANY, "", wxDefaultPosition, wxDefaultSize,
                             wxSP_ARROW_KEYS, 0, 251, fixed ? *fixed : 0);
  m_address->Enable(m_pilot_addresses[select] == kFixed);
  grid->Add(m_address, 0);
  top->Add(grid, 0, wxEXPAND | wxALL, 10);

  top->Add(CreateButtonSizer(wxOK | wxCANCEL), 0, wxEXPAND | wxALL, 10);
  Bind(wxEVT_BUTTON, &ApPrefs::OnOk, this, wxID_OK);
  SetSizerAndFit(top);
  CentreOnParent();
}

void ApPrefs::OnPilotChoice(wxCommandEvent&) {
  int sel = m_pilot->GetSelection();
  int a = sel >= 0 ? m_pilot_addresses[sel] : kAutomatic;
  if (a >= 0) m_address->SetValue(a);
  m_address->Enable(a == kFixed);
}

void ApPrefs::OnOk(wxCommandEvent& ev) {
  std::string handle;
  int c = m_connection->GetSelection();
  if (c >= 0 && c < static_cast<int>(m_handles.size())) handle = m_handles[c];
  std::optional<uint8_t> address;
  int sel = m_pilot->GetSelection();
  int a = sel >= 0 ? m_pilot_addresses[sel] : kAutomatic;
  if (a >= 0)
    address = static_cast<uint8_t>(a);
  else if (a == kFixed)
    address = static_cast<uint8_t>(m_address->GetValue());
  m_plugin->ApplySettings(handle, address);
  ev.Skip();  // closes the dialog
}
