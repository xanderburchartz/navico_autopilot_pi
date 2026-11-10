/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "ap_panel.h"

#include <cmath>

#include "navico_autopilot_pi.h"

using simnet::Command;
using simnet::Mode;

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

wxString Degrees(double deg, bool magnetic) {
  if (std::isnan(deg)) return "---";
  int d = static_cast<int>(std::lround(deg)) % 360;
  if (d < 0) d += 360;
  return wxString::Format(L"%03d\u00B0%s", d, magnetic ? "M" : "T");
}

}  // namespace

ApPanel::ApPanel(wxWindow* parent, NavicoAutopilotPi* plugin)
    : wxPanel(parent, wxID_ANY), m_plugin(plugin) {
  // The chart canvas parent has a black background; set our own colours
  // before creating the children so they inherit them.
  SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
  SetForegroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT));
  auto* top = new wxBoxSizer(wxVERTICAL);

  m_mode = new wxStaticText(this, wxID_ANY, "----", wxDefaultPosition,
                            wxDefaultSize, wxALIGN_CENTRE_HORIZONTAL | wxST_NO_AUTORESIZE);
  wxFont big = m_mode->GetFont();
  big.SetPointSize(big.GetPointSize() * 2);
  big.SetWeight(wxFONTWEIGHT_BOLD);
  m_mode->SetFont(big);
  top->Add(m_mode, 0, wxEXPAND | wxALL, 6);

  m_pilot = new wxStaticText(this, wxID_ANY, "", wxDefaultPosition,
                             wxDefaultSize, wxALIGN_CENTRE_HORIZONTAL | wxST_NO_AUTORESIZE);
  top->Add(m_pilot, 0, wxEXPAND | wxLEFT | wxRIGHT, 6);

  auto* headings = new wxFlexGridSizer(2, 2, 2, 12);
  headings->Add(new wxStaticText(this, wxID_ANY, _("Heading")));
  m_heading = new wxStaticText(this, wxID_ANY, "---");
  headings->Add(m_heading);
  headings->Add(new wxStaticText(this, wxID_ANY, _("Target")));
  m_target = new wxStaticText(this, wxID_ANY, "---");
  headings->Add(m_target);
  wxFont mid = m_heading->GetFont();
  mid.SetPointSize(mid.GetPointSize() * 3 / 2);
  m_heading->SetFont(mid);
  m_target->SetFont(mid);
  top->Add(headings, 0, wxALIGN_CENTRE_HORIZONTAL | wxALL, 6);

  auto* modes = new wxGridSizer(2, 3, 4, 4);
  AddButton(modes, _("STBY"), Command::kStandby);
  AddButton(modes, _("AUTO"), Command::kAuto);
  AddButton(modes, _("WIND"), Command::kWind);
  AddButton(modes, _("NAV"), Command::kNav);
  AddButton(modes, _("NO DRIFT"), Command::kNoDrift);
  modes->AddStretchSpacer();
  top->Add(modes, 0, wxEXPAND | wxALL, 6);

  auto* course = new wxGridSizer(1, 4, 4, 4);
  AddButton(course, "-10", Command::kPort10);
  AddButton(course, "-1", Command::kPort1);
  AddButton(course, "+1", Command::kStarboard1);
  AddButton(course, "+10", Command::kStarboard10);
  top->Add(course, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  m_message = new wxStaticText(this, wxID_ANY, "", wxDefaultPosition,
                               wxSize(-1, -1), wxST_NO_AUTORESIZE);
  m_message->SetMinSize(wxSize(GetCharWidth() * 34, GetCharHeight() * 2));
  top->Add(m_message, 0, wxEXPAND | wxALL, 6);

  SetSizerAndFit(top);
  UpdateState(m_plugin->Now());
}

wxButton* ApPanel::AddButton(wxSizer* sizer, const wxString& label, Command c) {
  auto* b = new wxButton(this, wxID_ANY, label);
  // Large enough to hit on a touch screen.
  b->SetMinSize(wxSize(GetCharWidth() * 9, GetCharHeight() * 3));
  b->Bind(wxEVT_BUTTON, [this, c](wxCommandEvent&) {
    m_plugin->SendCommand(c);
    UpdateState(m_plugin->Now());
  });
  sizer->Add(b, 1, wxEXPAND);
  m_buttons[c] = b;
  return b;
}

void ApPanel::UpdateState(double now) {
  const PilotState& s = m_plugin->State();
  auto address = s.Address(now);
  bool alive = s.Alive(now);
  bool control = m_plugin->Driver().has_value();

  // Mode and pilot.
  wxString mode = alive ? wxString(simnet::ModeName(s.Mode())) : wxString("----");
  if (m_mode->GetLabel() != mode) m_mode->SetLabel(mode);
  wxString pilot;
  if (s.NeedsChoice(now))
    pilot = _("More than one autopilot found");
  else if (!address)
    pilot = _("No autopilot found");
  else {
    wxString model;
    for (const auto& c : s.Candidates(now))
      if (c.address == *address) model = c.model == 0x64 ? "NAC" : "AC";
    pilot = wxString::Format(_("%s at address %d"), model.empty() ? wxString(_("Pilot")) : model,
                             *address);
    if (!alive) pilot += " - " + wxString(_("no data"));
  }
  if (m_pilot->GetLabel() != pilot) m_pilot->SetLabel(pilot);

  // Headings, in the reference the pilot uses.
  bool magnetic = s.TargetMagnetic();
  m_heading->SetLabel(Degrees(m_plugin->Heading(magnetic), magnetic));
  wxString target = "---";
  if (alive && s.Mode() == Mode::kWind && s.TargetWindAngle()) {
    double w = *s.TargetWindAngle() * kRadToDeg;
    target = wxString::Format(L"%d\u00B0 %s", static_cast<int>(std::lround(std::fabs(w))),
                              w < 0 ? _("port") : _("stbd"));
  } else if (alive && s.TargetHeading()) {
    target = Degrees(*s.TargetHeading() * kRadToDeg, magnetic);
  }
  m_target->SetLabel(target);

  // Buttons. STBY whenever a pilot is selected; the rest only with fresh data.
  Mode m = s.Mode();
  bool steering = alive && (m == Mode::kAuto || m == Mode::kWind || m == Mode::kNoDrift);
  for (auto& [c, b] : m_buttons) {
    bool on = control && address.has_value();
    switch (c) {
      case Command::kStandby: break;
      case Command::kAuto:
      case Command::kWind:
      case Command::kNav:
      case Command::kNoDrift: on = on && alive; break;
      default: on = on && steering; break;
    }
    if (b->IsEnabled() != on) b->Enable(on);
  }

  // One line of explanation, most important first.
  wxString msg = m_plugin->LastMessage();
  if (msg.empty()) {
    if (!m_plugin->Driver())
      msg = _("Choose the NMEA 2000 connection in the plugin settings.");
    else if (s.NeedsChoice(now))
      msg = _("Choose the autopilot in the plugin settings.");
    else if (s.Unconfirmed(now))
      msg = _("No confirmation from the autopilot.");
  }
  if (m_message->GetLabel() != msg) {
    m_message->SetLabel(msg);
    m_message->Wrap(m_message->GetSize().GetWidth());
  }
}
