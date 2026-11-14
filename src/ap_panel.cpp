/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   The display is drawn in the style of a Navico pilot controller: heading
 *   arc at the top with the set heading as a triangle, a mode box on the
 *   left, two values on the right and the rudder angle at the bottom. Layout
 *   proportions follow the Triton Pilot app (photos of a Triton2 display).
 ***************************************************************************/

#include "ap_panel.h"

#include <cmath>
#include <functional>
#include <optional>
#include <set>

#include <wx/dcbuffer.h>
#include <wx/choicdlg.h>
#include <wx/textdlg.h>
#if wxUSE_GRAPHICS_CONTEXT
#include <wx/dcgraph.h>
#endif

#include "navico_autopilot_pi.h"
#include "pilot_settings.h"

using simnet::Command;
using simnet::Mode;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

/** Mix of two colours, f = 0 gives a, f = 1 gives b. */
wxColour Mix(const wxColour& a, const wxColour& b, double f) {
  auto m = [f](int x, int y) { return static_cast<unsigned char>(std::lround(x + (y - x) * f)); };
  return wxColour(m(a.Red(), b.Red()), m(a.Green(), b.Green()), m(a.Blue(), b.Blue()));
}

/** Colours of display and buttons for one colour scheme. */
struct Theme {
  wxColour back, text, label, band, gap, edge;
  wxColour mode_fill, mode_caption, mode_text;
  wxColour port, starboard, box_fill, box_text;
  wxColour button_face, button_border, button_text;
};

Theme ThemeFor(PI_ColorScheme cs) {
  Theme t;
  switch (cs) {
    case PI_GLOBAL_COLOR_SCHEME_DUSK:
      t = {wxColour(12, 12, 12),   wxColour(160, 160, 160), wxColour(110, 110, 110),
           wxColour(42, 42, 42),   wxColour(120, 120, 120), wxColour(40, 95, 82),
           wxColour(30, 76, 73),   wxColour(45, 102, 98),   wxColour(175, 175, 175),
           wxColour(160, 40, 36),  wxColour(40, 130, 55),   wxColour(36, 36, 36),
           wxColour(180, 180, 180), wxColour(34, 34, 34),   wxColour(72, 72, 72),
           wxColour(160, 160, 160)};
      break;
    case PI_GLOBAL_COLOR_SCHEME_NIGHT:
      // Red only, dim: keeps night vision, like a pilot display at night.
      t = {wxColour(0, 0, 0),      wxColour(150, 20, 14),   wxColour(100, 13, 9),
           wxColour(32, 4, 3),     wxColour(115, 15, 10),   wxColour(75, 9, 6),
           wxColour(62, 8, 5),     wxColour(85, 11, 7),     wxColour(170, 24, 16),
           wxColour(150, 20, 14),  wxColour(150, 20, 14),   wxColour(48, 6, 4),
           wxColour(170, 24, 16),  wxColour(26, 3, 2),      wxColour(64, 8, 6),
           wxColour(150, 20, 14)};
      break;
    default:  // day and RGB
      t = {wxColour(247, 247, 247), wxColour(20, 20, 20),   wxColour(77, 77, 77),
           wxColour(15, 15, 15),    wxColour(255, 255, 255), wxColour(51, 140, 115),
           wxColour(61, 153, 148),  wxColour(107, 186, 181), wxColour(255, 255, 255),
           wxColour(219, 31, 31),   wxColour(41, 158, 61),  wxColour(20, 20, 20),
           wxColour(255, 255, 255), wxColour(232, 232, 232), wxColour(170, 170, 170),
           wxColour(20, 20, 20)};
      break;
  }
  return t;
}

wxFont PixelFont(int px, bool bold = false) {
  wxFontInfo info(wxSize(0, std::max(px, 6)));
  info.Family(wxFONTFAMILY_SWISS);
  if (bold) info.Bold();
  return wxFont(info);
}

std::optional<double> Deg(std::optional<double> rad) {
  if (!rad) return std::nullopt;
  return *rad * kRadToDeg;
}

/** -180..180 */
double Signed(double deg) {
  deg = std::fmod(deg, 360.0);
  if (deg > 180) deg -= 360;
  if (deg < -180) deg += 360;
  return deg;
}

wxString Three(std::optional<double> deg) {
  if (!deg || std::isnan(*deg)) return "---";
  int d = static_cast<int>(std::lround(*deg)) % 360;
  if (d < 0) d += 360;
  return wxString::Format("%03d", d);
}

}  // namespace

// ---- display ----------------------------------------------------------------

/** What the display shows; filled by ApPanel::UpdateState(). */
struct DisplayData {
  bool alive = false;
  Mode mode = Mode::kUnknown;
  bool magnetic = false;
  std::optional<double> heading, target, wpt_bearing, dtw_nm, sog;
  std::optional<double> awa, set_wind, rudder;  // signed, negative = port
  bool stw = false;  // speed is through the water, not over ground
  bool nfu = false;  // non-follow-up active
  wxString badge;    // "NAC-3 4"
  wxString waypoint;  // name of the destination waypoint, if known
};

class ApDisplay : public wxWindow {
public:
  explicit ApDisplay(wxWindow* parent)
      : wxWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE) {
    SetBackgroundStyle(wxBG_STYLE_PAINT);
    SetMinSize(wxSize(300, 210));
    Bind(wxEVT_PAINT, &ApDisplay::OnPaint, this);
    Bind(wxEVT_SIZE, [this](wxSizeEvent& e) {
      Refresh();
      e.Skip();
    });
    // The gear opens the pilot settings, as in the Triton Pilot app.
    Bind(wxEVT_LEFT_UP, [this](wxMouseEvent& e) {
      if (m_on_settings && m_gear.Contains(e.GetPosition())) m_on_settings();
    });
    Bind(wxEVT_MOTION, [this](wxMouseEvent& e) {
      SetCursor(m_on_settings && m_gear.Contains(e.GetPosition()) ? wxCursor(wxCURSOR_HAND)
                                                                    : wxNullCursor);
    });
  }

  void SetOnSettings(std::function<void()> f) {
    m_on_settings = std::move(f);
    Refresh(false);
  }

  void Set(const DisplayData& d) {
    m_data = d;
    Refresh(false);
  }
  void SetTheme(const Theme& t) {
    m_theme = t;
    Refresh(false);
  }

private:
  static constexpr double kHalfSpan = 40;  // degrees of the arc each side

  // Arc geometry, as in the Triton Pilot app: a large circle of which only
  // the top shows, so 80 degrees of heading fit the width.
  struct Arc {
    double w, h, radius, cx, cy, band, k;  // k: radians on screen per degree
    wxPoint At(double offset_deg, double r) const {
      double a = offset_deg * k;
      return wxPoint(static_cast<int>(std::lround(cx + r * std::sin(a))),
                     static_cast<int>(std::lround(cy - r * std::cos(a))));
    }
  };

  void OnPaint(wxPaintEvent&) {
    wxAutoBufferedPaintDC pdc(this);
    pdc.SetBackground(wxBrush(m_theme.back));
    pdc.Clear();
#if wxUSE_GRAPHICS_CONTEXT
    wxGCDC gdc(pdc);
    wxDC& dc = gdc;
#else
    wxDC& dc = pdc;  // Android (wxQt) has no graphics context
#endif
    wxSize sz = GetClientSize();
    double w = sz.x, h = sz.y;
    DrawHeadingArc(dc, w, h * 0.36);
    DrawModeBox(dc, wxRect(wxPoint(w * 0.05, h * 0.42), wxSize(w * 0.27, h * 0.40)));
    DrawReadout(dc, w, h);
    DrawRudder(dc, w, h);
    DrawBadge(dc, w, h);
    DrawGear(dc, w, h);
  }

  void DrawGear(wxDC& dc, double w, double h) {
    if (!m_on_settings) return;
    double r = std::max(7.0, h * 0.045);
    double cx = w - r * 2.0, cy = r * 2.0;
    // Touch target larger than the drawing.
    m_gear = wxRect(static_cast<int>(cx - r * 2), 0, static_cast<int>(r * 4),
                    static_cast<int>(cy + r * 2));
    std::vector<wxPoint> pts;
    const int teeth = 8;
    for (int i = 0; i < teeth * 4; i++) {
      double a = i * kPi * 2 / (teeth * 4);
      double rr = (i % 4 < 2) ? r : r * 0.72;
      pts.push_back(wxPoint(static_cast<int>(std::lround(cx + rr * std::cos(a))),
                            static_cast<int>(std::lround(cy + rr * std::sin(a)))));
    }
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(m_theme.label));
    dc.DrawPolygon(static_cast<int>(pts.size()), pts.data());
    dc.SetBrush(wxBrush(m_theme.back));
    dc.DrawCircle(static_cast<int>(cx), static_cast<int>(cy), static_cast<int>(r * 0.35));
  }

  void DrawHeadingArc(wxDC& dc, double w, double h) {
    Arc g;
    g.w = w;
    g.h = h;
    g.radius = w * 1.05;
    g.cx = w / 2;
    g.cy = h * 0.30 + g.radius;
    g.band = h * 0.17;
    double phys_half = std::asin(std::min(1.0, (w / 2) / g.radius));
    g.k = phys_half / kHalfSpan;

    // Band and edge as polylines: works with and without a graphics context.
    auto arc_points = [&](double r) {
      std::vector<wxPoint> pts;
      for (double off = -kHalfSpan * 1.15; off <= kHalfSpan * 1.15; off += 1.0)
        pts.push_back(g.At(off, r));
      return pts;
    };
    auto band = arc_points(g.radius);
    dc.SetPen(wxPen(m_theme.band, std::max(2, static_cast<int>(g.band))));
    dc.DrawLines(static_cast<int>(band.size()), band.data());
    auto edge = arc_points(g.radius + g.band * 0.62);
    dc.SetPen(wxPen(m_theme.edge, 2));
    dc.DrawLines(static_cast<int>(edge.size()), edge.data());

    auto heading = m_data.heading;
    if (heading) {
      // A gap every 10 degrees, a number every 30.
      dc.SetFont(PixelFont(static_cast<int>(h * 0.13), true));
      dc.SetTextForeground(m_theme.text);
      int first = static_cast<int>(std::floor((*heading - kHalfSpan - 10) / 10)) * 10;
      for (int deg = first; deg <= *heading + kHalfSpan + 10; deg += 10) {
        double off = deg - *heading;
        if (std::fabs(off) > kHalfSpan * 1.15) continue;
        dc.SetPen(wxPen(m_theme.gap, 2));
        dc.DrawLine(g.At(off, g.radius + g.band / 2), g.At(off, g.radius - g.band / 2));
        if (deg % 30 != 0) continue;
        wxPoint at = g.At(off, g.radius - g.band * 1.35);
        if (at.x < w * 0.08 || at.x > w * 0.92) continue;
        wxString label = Three(static_cast<double>(deg));
        DrawRotatedCentred(dc, label, at, -off * g.k * kRadToDeg);
      }
      // Set heading (or bearing to the waypoint), and the wind in WIND mode.
      auto target = ArcTarget();
      if (target) {
        double off = std::max(-kHalfSpan, std::min(kHalfSpan, Signed(*target - *heading)));
        DrawTriangle(dc, g, off, off < 0 ? m_theme.port : m_theme.starboard);
      }
      if (m_data.mode == Mode::kWind && m_data.awa) {
        double off = std::max(-kHalfSpan, std::min(kHalfSpan, *m_data.awa));
        DrawTriangle(dc, g, off, *m_data.awa < 0 ? m_theme.port : m_theme.starboard);
      }
    }

    // Heading in a box at the top.
    wxString text = Three(heading);
    dc.SetFont(PixelFont(static_cast<int>(h * 0.24), true));
    wxSize t = dc.GetTextExtent(text);
    wxRect box(wxPoint(static_cast<int>(w / 2 - t.x / 2 - h * 0.05), static_cast<int>(h * 0.02)),
               wxSize(static_cast<int>(t.x + h * 0.1), static_cast<int>(t.y + h * 0.02)));
    dc.SetBrush(wxBrush(m_theme.box_fill));
    dc.SetPen(wxPen(m_theme.back));
    dc.DrawRoundedRectangle(box, 4);
    dc.SetTextForeground(m_theme.box_text);
    dc.DrawText(text, box.x + (box.width - t.x) / 2, box.y + (box.height - t.y) / 2);
  }

  std::optional<double> ArcTarget() const {
    switch (m_data.mode) {
      case Mode::kAuto:
      case Mode::kNoDrift: return m_data.target;
      case Mode::kNav: return m_data.wpt_bearing ? m_data.wpt_bearing : m_data.target;
      default: return std::nullopt;
    }
  }

  void DrawTriangle(wxDC& dc, const Arc& g, double off, const wxColour& colour) {
    double s = g.band * 0.9;
    double a = off * g.k;
    wxPoint c = g.At(off, g.radius - g.band / 2 - s * 0.6);
    // Pointing outwards (up on screen when off = 0), rotated with the arc.
    double pts[3][2] = {{0, -s * 0.5}, {s * 0.6, s * 0.5}, {-s * 0.6, s * 0.5}};
    wxPoint poly[3];
    for (int i = 0; i < 3; i++) {
      double x = pts[i][0], y = pts[i][1];
      poly[i] = wxPoint(static_cast<int>(std::lround(c.x + x * std::cos(a) - y * std::sin(a))),
                        static_cast<int>(std::lround(c.y + x * std::sin(a) + y * std::cos(a))));
    }
    dc.SetBrush(wxBrush(colour));
    dc.SetPen(wxPen(colour));
    dc.DrawPolygon(3, poly);
  }

  static void DrawRotatedCentred(wxDC& dc, const wxString& text, wxPoint centre, double angle_deg) {
    // DrawRotatedText turns around the top left corner; shift so the middle
    // of the text lands on centre.
    wxSize t = dc.GetTextExtent(text);
    double a = angle_deg / kRadToDeg;
    double dx = t.x / 2.0, dy = t.y / 2.0;
    double ox = dx * std::cos(a) + dy * std::sin(a);
    double oy = -dx * std::sin(a) + dy * std::cos(a);
    dc.DrawRotatedText(text, static_cast<int>(std::lround(centre.x - ox)),
                       static_cast<int>(std::lround(centre.y - oy)), angle_deg);
  }

  void DrawModeBox(wxDC& dc, const wxRect& r) {
    bool engaged = m_data.alive && ((m_data.mode != Mode::kStandby && m_data.mode != Mode::kUnknown) ||
                                    m_data.nfu);
    wxString caption, letter;
    switch (m_data.alive ? m_data.mode : Mode::kUnknown) {
      case Mode::kStandby:
        if (m_data.nfu) { caption = _("NON FOLLOW-UP"); letter = "NFU"; break; }
        caption = _("STANDBY"); letter = "S"; break;
      case Mode::kAuto: caption = _("HEADING HOLD"); letter = "A"; break;
      case Mode::kWind: caption = _("WIND"); letter = "W"; break;
      case Mode::kNav: caption = _("NAVIGATION"); letter = "N"; break;
      case Mode::kNoDrift: caption = _("NO DRIFT"); letter = "ND"; break;
      default: caption = _("NO PILOT"); letter = "--"; break;
    }
    int radius = std::max(3, r.width / 25);
    dc.SetPen(wxPen(engaged ? m_theme.mode_fill : m_theme.label, 2));
    dc.SetBrush(wxBrush(engaged ? m_theme.mode_fill : m_theme.back));
    dc.DrawRoundedRectangle(r, radius);
    wxRect strip(r.x, r.y, r.width, static_cast<int>(r.height * 0.27));
    if (engaged) {
      dc.SetPen(wxPen(m_theme.mode_caption));
      dc.SetBrush(wxBrush(m_theme.mode_caption));
      dc.DrawRoundedRectangle(strip, radius);
    }
    wxColour fg = engaged ? m_theme.mode_text : m_theme.label;
    dc.SetTextForeground(fg);
    // Caption: shrink until it fits the width.
    int px = static_cast<int>(strip.height * 0.42);
    dc.SetFont(PixelFont(px, true));
    while (px > 7 && dc.GetTextExtent(caption).x > r.width * 0.92) dc.SetFont(PixelFont(--px, true));
    wxSize t = dc.GetTextExtent(caption);
    dc.DrawText(caption, r.x + (r.width - t.x) / 2, strip.y + (strip.height - t.y) / 2);
    int lp = static_cast<int>(r.height * (letter.length() > 2 ? 0.30 : letter.length() > 1 ? 0.42 : 0.55));
    dc.SetFont(PixelFont(lp, true));
    t = dc.GetTextExtent(letter);
    int body_y = strip.GetBottom();
    dc.DrawText(letter, r.x + (r.width - t.x) / 2, body_y + (r.GetBottom() - body_y - t.y) / 2);
  }

  void DrawRow(wxDC& dc, const wxRect& r, const wxString& label, const wxString& value,
               const wxString& unit, std::optional<double> side) {
    dc.SetTextForeground(m_theme.label);
    dc.SetFont(PixelFont(static_cast<int>(r.height * 0.24)));
    dc.DrawText(label, r.x + r.width * 0.02, r.y);
    int vy = r.y + static_cast<int>(r.height * 0.24);
    dc.SetFont(PixelFont(static_cast<int>(r.height * 0.70), true));
    wxSize t = dc.GetTextExtent(value);
    int right = r.x + static_cast<int>(r.width * 0.74);
    dc.SetTextForeground(m_theme.text);
    dc.DrawText(value, right - t.x, vy);
    if (side) {
      // Small triangle before the value: port red pointing left, starboard green right.
      int s = static_cast<int>(r.height * 0.22);
      int cx = right - t.x - s, cy = vy + t.y / 2;
      bool port = *side < 0;
      wxPoint p[3] = {wxPoint(cx + (port ? -s / 2 : s / 2), cy), wxPoint(cx - (port ? -s / 2 : s / 2), cy - s / 2),
                      wxPoint(cx - (port ? -s / 2 : s / 2), cy + s / 2)};
      wxColour c = port ? m_theme.port : m_theme.starboard;
      dc.SetBrush(wxBrush(c));
      dc.SetPen(wxPen(c));
      dc.DrawPolygon(3, p);
    }
    dc.SetTextForeground(m_theme.label);
    dc.SetFont(PixelFont(static_cast<int>(r.height * 0.24)));
    dc.DrawText(unit, right + r.width * 0.03, vy + r.height * 0.08);
  }

  void DrawReadout(wxDC& dc, double w, double h) {
    int x = static_cast<int>(w * 0.35);
    int rw = static_cast<int>(w * 0.63);
    int rh = static_cast<int>(h * 0.21);
    wxRect r1(x, static_cast<int>(h * 0.39), rw, rh);
    wxRect r2(x, static_cast<int>(h * 0.62), rw, rh);
    wxString unit = m_data.magnetic ? L"°M" : L"°T";
    auto abs_deg = [](std::optional<double> v) {
      return v ? wxString::Format("%.0f", std::fabs(*v)) : wxString("---");
    };
    auto sog = [&]() {
      return m_data.sog ? wxString::Format("%.1f", *m_data.sog) : wxString("-.-");
    };
    wxString speed_label = m_data.stw ? _("Speed through water") : _("Speed over ground");
    switch (m_data.alive ? m_data.mode : Mode::kUnknown) {
      case Mode::kWind:
        DrawRow(dc, r1, _("Apparent wind"), abs_deg(m_data.awa), L"°", m_data.awa);
        DrawRow(dc, r2, _("Set angle"), abs_deg(m_data.set_wind), L"°", m_data.set_wind);
        break;
      case Mode::kNav:
        DrawRow(dc, r1,
                m_data.waypoint.empty() ? wxString(_("Bearing WPT"))
                                        : wxString::Format(_("Bearing %s"), m_data.waypoint), Three(m_data.wpt_bearing ? m_data.wpt_bearing : m_data.target),
                unit, std::nullopt);
        DrawRow(dc, r2, _("Distance WPT"),
                m_data.dtw_nm ? wxString::Format("%.2f", *m_data.dtw_nm) : wxString("-.--"), "NM",
                std::nullopt);
        break;
      case Mode::kAuto:
      case Mode::kNoDrift:
        DrawRow(dc, r1, _("Set heading"), Three(m_data.target), unit, std::nullopt);
        DrawRow(dc, r2, speed_label, sog(), "kn", std::nullopt);
        break;
      default:
        if (m_data.nfu) {
          DrawRow(dc, r1, _("Rudder angle"),
                  m_data.rudder ? wxString::Format("%.0f", std::fabs(*m_data.rudder)) : wxString("--"),
                  L"\u00B0", m_data.rudder);
        } else {
          DrawRow(dc, r1, _("Heading"), Three(m_data.heading), unit, std::nullopt);
        }
        DrawRow(dc, r2, speed_label, sog(), "kn", std::nullopt);
        break;
    }
  }

  void DrawRudder(wxDC& dc, double w, double h) {
    double y = h * 0.92;
    double box_w = w * 0.10;
    double max_len = w * 0.38 - box_w / 2;
    dc.SetPen(wxPen(m_theme.edge, 2));
    dc.DrawLine(static_cast<int>(w * 0.12), static_cast<int>(y), static_cast<int>(w * 0.88),
                static_cast<int>(y));
    auto rudder = m_data.rudder;
    if (rudder) {
      constexpr double kFullScale = 35;
      double len = std::min(std::fabs(*rudder), kFullScale) / kFullScale * max_len;
      if (std::fabs(*rudder) >= 0.5) len = std::max(len, 3.0);
      wxColour c = *rudder < 0 ? m_theme.port : m_theme.starboard;
      dc.SetBrush(wxBrush(c));
      dc.SetPen(wxPen(c));
      int bh = std::max(3, static_cast<int>(h * 0.035));
      int x0 = *rudder < 0 ? static_cast<int>(w / 2 - box_w / 2 - 2 - len)
                           : static_cast<int>(w / 2 + box_w / 2 + 2);
      dc.DrawRectangle(x0, static_cast<int>(y - bh / 2), static_cast<int>(len), bh);
    }
    wxString text = rudder ? wxString::Format("%02.0f", std::fabs(std::round(*rudder))) : wxString("--");
    dc.SetFont(PixelFont(static_cast<int>(h * 0.07), true));
    wxSize t = dc.GetTextExtent(text);
    wxRect box(static_cast<int>(w / 2 - box_w / 2), static_cast<int>(y - t.y / 2 - 1),
               static_cast<int>(box_w), t.y + 2);
    dc.SetBrush(wxBrush(m_theme.box_fill));
    dc.SetPen(wxPen(m_theme.box_fill));
    dc.DrawRoundedRectangle(box, 3);
    dc.SetTextForeground(m_theme.box_text);
    dc.DrawText(text, box.x + (box.width - t.x) / 2, box.y + 1);
  }

  void DrawBadge(wxDC& dc, double w, double h) {
    if (m_data.badge.empty()) return;
    dc.SetFont(PixelFont(static_cast<int>(h * 0.045), true));
    dc.SetTextForeground(m_data.alive ? m_theme.label : m_theme.port);
    dc.DrawText(m_data.badge, static_cast<int>(w * 0.015), static_cast<int>(h * 0.01));
  }

  DisplayData m_data;
  Theme m_theme = ThemeFor(PI_GLOBAL_COLOR_SCHEME_DAY);
  std::function<void()> m_on_settings;
  wxRect m_gear;
};

// ---- buttons ------------------------------------------------------------------

/**
 * A push button drawn by the plugin itself. Native buttons (macOS above all)
 * cannot be coloured, so in OpenCPN's dusk and night schemes they stayed
 * bright. Sends wxEVT_BUTTON like wxButton.
 */
class ApButton : public wxWindow {
public:
  ApButton(wxWindow* parent, const wxString& label)
      : wxWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE),
        m_label(label) {
    SetBackgroundStyle(wxBG_STYLE_PAINT);
    Bind(wxEVT_PAINT, &ApButton::OnPaint, this);
    Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent&) {
      if (!IsEnabled()) return;
      m_pressed = true;
      if (!HasCapture()) CaptureMouse();
      Refresh();
      if (m_on_press) m_on_press();
    });
    Bind(wxEVT_LEFT_UP, [this](wxMouseEvent& e) {
      if (HasCapture()) ReleaseMouse();
      bool click = m_pressed && GetClientRect().Contains(e.GetPosition());
      bool was_pressed = m_pressed;
      m_pressed = false;
      Refresh();
      if (was_pressed && m_on_release) m_on_release();
      if (click && IsEnabled() && !m_on_press) {
        wxCommandEvent ev(wxEVT_BUTTON, GetId());
        ev.SetEventObject(this);
        ProcessWindowEvent(ev);
      }
    });
    Bind(wxEVT_MOUSE_CAPTURE_LOST, [this](wxMouseCaptureLostEvent&) {
      bool was_pressed = m_pressed;
      m_pressed = false;
      Refresh();
      if (was_pressed && m_on_release) m_on_release();
    });
  }

  /** Press-and-hold mode (NFU): called on press and on release instead of
   *  sending wxEVT_BUTTON; release also on capture loss. */
  void SetHold(std::function<void()> press, std::function<void()> release) {
    m_on_press = std::move(press);
    m_on_release = std::move(release);
  }
  bool IsHold() const { return static_cast<bool>(m_on_press); }

  void SetText(const wxString& label) {
    if (label == m_label) return;
    m_label = label;
    Refresh();
  }

  void SetTheme(const Theme& t) {
    m_theme = t;
    Refresh();
  }

  bool Enable(bool enable = true) override {
    bool changed = wxWindow::Enable(enable);
    if (changed) Refresh();
    return changed;
  }

protected:
  wxSize DoGetBestSize() const override {
    wxSize t = GetTextExtent(m_label);
    return wxSize(t.x + 2 * GetCharWidth(), t.y + GetCharHeight());
  }

private:
  void OnPaint(wxPaintEvent&) {
    wxAutoBufferedPaintDC pdc(this);
    pdc.SetBackground(wxBrush(m_theme.back));
    pdc.Clear();
#if wxUSE_GRAPHICS_CONTEXT
    wxGCDC gdc(pdc);  // anti-aliased rounded corners
    wxDC& dc = gdc;
#else
    wxDC& dc = pdc;  // Android (wxQt) has no graphics context
#endif
    wxRect r = GetClientRect().Deflate(1);
    wxColour face = m_pressed ? Mix(m_theme.button_face, m_theme.button_text, 0.25)
                              : m_theme.button_face;
    if (!IsEnabled()) face = Mix(m_theme.button_face, m_theme.back, 0.5);
    dc.SetBrush(wxBrush(face));
    dc.SetPen(wxPen(IsEnabled() ? m_theme.button_border : face));
    dc.DrawRoundedRectangle(r, 5);
    dc.SetFont(GetFont());
    dc.SetTextForeground(IsEnabled() ? m_theme.button_text
                                     : Mix(face, m_theme.button_text, 0.4));
    wxSize t = dc.GetTextExtent(m_label);
    dc.DrawText(m_label, r.x + (r.width - t.x) / 2, r.y + (r.height - t.y) / 2);
  }

  wxString m_label;
  bool m_pressed = false;
  std::function<void()> m_on_press, m_on_release;
  Theme m_theme = ThemeFor(PI_GLOBAL_COLOR_SCHEME_DAY);
};

// ---- tack / gybe ----------------------------------------------------------------

/**
 * Tack or gybe in WIND, as on a Triton2: a window with the heading now and
 * the expected new heading and wind angle; Confirm sends the command. The
 * estimate assumes the pilot mirrors the set wind angle (heading turns by
 * twice that angle), ignoring the loss of speed.
 */
/**
 * Base of the plugin's own windows. They hang on OpenCPN's frame (or on the
 * floating AUI frame) and would otherwise be deleted after OpenCPN unloaded
 * the plugin library, crashing in their destructor (seen on macOS when
 * quitting with the settings window open). The panel deletes them all when
 * it goes, which happens in DeInit.
 */
class PluginDialog : public wxDialog {
public:
  PluginDialog(wxWindow* parent, const wxString& title, long style)
      : wxDialog(parent, wxID_ANY, title, wxDefaultPosition, wxDefaultSize, style) {
    Open().insert(this);
  }
  ~PluginDialog() override { Open().erase(this); }

  static void DeleteAll() {
    std::set<PluginDialog*> list = Open();
    for (PluginDialog* d : list) {
      // A dialog that closed itself waits in wxWidgets' pending delete list;
      // take it out so it is not deleted a second time.
      if (wxTheApp && wxTheApp->IsScheduledForDestruction(d)) wxPendingDelete.DeleteObject(d);
      delete d;
    }
  }

private:
  static std::set<PluginDialog*>& Open() {
    static std::set<PluginDialog*> open;
    return open;
  }
};

class TackDialog : public PluginDialog {
public:
  TackDialog(wxWindow* parent, NavicoAutopilotPi* plugin, const Theme& theme,
             std::optional<double> heading, std::optional<double> wind, bool magnetic)
      : PluginDialog(parent, _("Tack or gybe"), wxCAPTION | wxSTAY_ON_TOP),
        m_plugin(plugin) {
    SetBackgroundColour(theme.back);
    bool gybe = wind && std::fabs(*wind) > 90;
    wxString unit = magnetic ? L"\u00B0M" : L"\u00B0T";
    auto* top = new wxBoxSizer(wxVERTICAL);
    auto text = [&](const wxString& t, bool bold) {
      auto* st = new wxStaticText(this, wxID_ANY, t);
      st->SetForegroundColour(bold ? theme.text : theme.label);
      if (bold) {
        wxFont f = st->GetFont();
        f.SetPointSize(f.GetPointSize() * 5 / 4);
        f.SetWeight(wxFONTWEIGHT_BOLD);
        st->SetFont(f);
      }
      top->Add(st, 0, wxLEFT | wxRIGHT | wxTOP, 12);
    };
    text(gybe ? _("Gybe?") : _("Tack?"), true);
    std::optional<double> next;
    if (heading && wind) next = std::fmod(*heading + 2 * *wind + 720.0, 360.0);
    text(wxString::Format(_("Heading now %s%s, after the maneuver about %s%s."), Three(heading), unit,
                          Three(next), unit), false);
    if (wind)
      text(wxString::Format(_("Wind angle %s %s becomes %s %s."),
                            wxString::Format(L"%.0f\u00B0", std::fabs(*wind)),
                            *wind < 0 ? _("port") : _("starboard"),
                            wxString::Format(L"%.0f\u00B0", std::fabs(*wind)),
                            *wind < 0 ? _("starboard") : _("port")),
           false);
    auto* buttons = new wxBoxSizer(wxHORIZONTAL);
    auto* cancel = new ApButton(this, _("Cancel"));
    auto* confirm = new ApButton(this, gybe ? _("Gybe") : _("Tack"));
    for (ApButton* b : {cancel, confirm}) {
      b->SetTheme(theme);
      b->SetMinSize(wxSize(GetCharWidth() * 14, GetCharHeight() * 3));
      buttons->Add(b, 0, wxLEFT, 8);
    }
    top->Add(buttons, 0, wxALIGN_RIGHT | wxALL, 12);
    confirm->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
      m_plugin->SendCommand(Command::kTack);
      Close();
    });
    cancel->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { Close(); });
    Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent&) { Destroy(); });
    // Closes by itself after 10 s, as on a Triton2.
    m_timeout.SetOwner(this);
    Bind(wxEVT_TIMER, [this](wxTimerEvent&) { Close(); });
    m_timeout.StartOnce(10000);
    SetSizerAndFit(top);
  }

private:
  NavicoAutopilotPi* m_plugin;
  wxTimer m_timeout;
};

// ---- pilot settings -------------------------------------------------------------

/**
 * The pilot's own settings, read over the key value channel (130845) when
 * the window opens. Change sends a new value after a confirmation and shows
 * the value the pilot answers with.
 */
class SettingsDialog : public PluginDialog {
public:
  SettingsDialog(wxWindow* parent, NavicoAutopilotPi* plugin, const Theme& theme)
      : PluginDialog(parent, _("Autopilot settings"),
                     wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER | wxSTAY_ON_TOP),
        m_plugin(plugin) {
    SetBackgroundColour(theme.back);
    auto* top = new wxBoxSizer(wxVERTICAL);
    auto* grid = new wxFlexGridSizer(4, 4, 16);
    grid->AddGrowableCol(0);
    const char* section = "";
    for (const auto& d : pilot_settings::All()) {
      if (d.per_profile) {
        m_profile_defs.push_back(&d);
        continue;
      }
      // Boat, drive and rudder are the commissioning settings (Installation
      // on a Triton2); they stay hidden until asked for.
      bool installation = std::string(d.section) != "Sailing";
      size_t first = m_all_rows.size();
      if (std::string(section) != d.section) {
        section = d.section;
        auto* h = new wxStaticText(this, wxID_ANY, Tr(section));
        m_all_rows.push_back(h);
        wxFont f = h->GetFont();
        f.SetWeight(wxFONTWEIGHT_BOLD);
        h->SetFont(f);
        h->SetForegroundColour(theme.text);
        grid->Add(h, 0, wxTOP, 8);
        grid->AddSpacer(0);
        grid->AddSpacer(0);
        grid->AddSpacer(0);
      }
      auto* name = new wxStaticText(this, wxID_ANY, Tr(d.name));
      name->SetForegroundColour(theme.label);
      auto* value = new wxStaticText(this, wxID_ANY, "...", wxDefaultPosition, wxDefaultSize,
                                     wxALIGN_RIGHT | wxST_NO_AUTORESIZE);
      value->SetMinSize(wxSize(GetCharWidth() * 12, -1));
      value->SetForegroundColour(theme.text);
      auto* unit = new wxStaticText(this, wxID_ANY, UnitText(d.unit));
      unit->SetForegroundColour(theme.label);
      grid->Add(name, 0, wxALIGN_CENTRE_VERTICAL);
      grid->Add(value, 0, wxALIGN_CENTRE_VERTICAL | wxEXPAND);
      grid->Add(unit, 0, wxALIGN_CENTRE_VERTICAL);
      m_all_rows.insert(m_all_rows.end(), {name, value, unit});
      m_values.push_back(value);
      m_units.push_back(unit);
      if (pilot_settings::Editable(d.key)) {
        auto* change = new ApButton(this, _("Change"));
        change->SetTheme(theme);
        change->SetMinSize(wxSize(GetCharWidth() * 10, GetCharHeight() + 10));
        const pilot_settings::Definition* def = &d;
        change->Bind(wxEVT_BUTTON, [this, def](wxCommandEvent&) { Change(*def); });
        grid->Add(change, 0, wxALIGN_CENTRE_VERTICAL);
        m_all_rows.push_back(change);
      } else {
        grid->AddSpacer(0);
      }
      if (installation)
        m_installation.insert(m_installation.end(), m_all_rows.begin() + first, m_all_rows.end());
    }
    // Two columns, so the window fits on a laptop screen: the settings left,
    // the steering profiles right.
    auto* body = new wxBoxSizer(wxHORIZONTAL);
    body->Add(grid, 0, wxLEFT | wxRIGHT | wxTOP, 12);
    auto* right = new wxBoxSizer(wxVERTICAL);
    body->Add(right, 0, wxLEFT | wxRIGHT, 12);
    top->Add(body, 0, wxEXPAND);

    // Steering profiles: one column per profile, as the Triton2 shows them.
    // An editable value is a button: press it to change that profile's value.
    if (!m_profile_defs.empty()) {
      const auto& profiles = pilot_settings::Profiles();
      auto* h = new wxStaticText(this, wxID_ANY, _("Steering profiles"));
      wxFont f = h->GetFont();
      f.SetWeight(wxFONTWEIGHT_BOLD);
      h->SetFont(f);
      h->SetForegroundColour(theme.text);
      right->Add(h, 0, wxTOP, 20);
      auto* pg = new wxFlexGridSizer(static_cast<int>(profiles.size()) + 2, 4, 12);
      pg->AddGrowableCol(0);
      pg->AddSpacer(0);
      for (const auto& prof : profiles) {
        auto* n = new wxStaticText(this, wxID_ANY, Tr(prof.name), wxDefaultPosition,
                                   wxDefaultSize, wxALIGN_CENTRE_HORIZONTAL | wxST_NO_AUTORESIZE);
        n->SetMinSize(wxSize(GetCharWidth() * 12, -1));
        n->SetForegroundColour(theme.label);
        pg->Add(n, 0, wxEXPAND);
      }
      pg->AddSpacer(0);
      for (const auto* d : m_profile_defs) {
        auto* name = new wxStaticText(this, wxID_ANY, Tr(d->name));
        name->SetForegroundColour(theme.label);
        pg->Add(name, 0, wxALIGN_CENTRE_VERTICAL);
        std::vector<std::function<void(const wxString&)>> row;
        bool editable = pilot_settings::Editable(d->key).has_value();
        for (const auto& prof : profiles) {
          if (editable) {
            auto* b = new ApButton(this, "...");
            b->SetTheme(theme);
            b->SetMinSize(wxSize(GetCharWidth() * 12, GetCharHeight() + 10));
            uint8_t inst = prof.instance;
            b->Bind(wxEVT_BUTTON, [this, d, inst](wxCommandEvent&) { Change(*d, inst); });
            pg->Add(b, 0, wxEXPAND);
            row.push_back([b](const wxString& t) { b->SetText(t); });
          } else {
            auto* v = new wxStaticText(this, wxID_ANY, "...", wxDefaultPosition, wxDefaultSize,
                                       wxALIGN_CENTRE_HORIZONTAL | wxST_NO_AUTORESIZE);
            v->SetForegroundColour(theme.text);
            pg->Add(v, 0, wxALIGN_CENTRE_VERTICAL | wxEXPAND);
            row.push_back([v](const wxString& t) {
              if (v->GetLabel() != t) v->SetLabel(t);
            });
          }
        }
        m_profile_cells.push_back(row);
        auto* unit = new wxStaticText(this, wxID_ANY, UnitText(d->unit));
        unit->SetForegroundColour(theme.label);
        pg->Add(unit, 0, wxALIGN_CENTRE_VERTICAL);
      }
      right->Add(pg, 0, wxEXPAND | wxTOP, 8);
      auto* about = new wxStaticText(this, wxID_ANY, _("Press a value to change it."));
      about->SetForegroundColour(theme.label);
      right->Add(about, 0, wxTOP, 12);
    }
    top->AddSpacer(12);
    m_note = new wxStaticText(this, wxID_ANY, "");
    m_note->SetForegroundColour(theme.label);
    top->Add(m_note, 0, wxLEFT | wxRIGHT, 12);

    auto* buttons = new wxBoxSizer(wxHORIZONTAL);
    // Installation settings behind three quick presses, as the superuser
    // level of a Victron GX: hard to reach by accident, no password.
    m_install_button = new ApButton(this, _("Installation"));
    m_install_button->SetTheme(theme);
    m_install_button->SetMinSize(wxSize(GetCharWidth() * 14, GetCharHeight() * 2 + 8));
    m_install_button->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { InstallationPressed(); });
    buttons->Add(m_install_button, 0);
    buttons->AddStretchSpacer();
    auto* snapshots = new ApButton(this, _("Snapshots..."));
    snapshots->SetTheme(theme);
    snapshots->SetMinSize(wxSize(GetCharWidth() * 14, GetCharHeight() * 2 + 8));
    snapshots->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { ShowSnapshots(); });
    buttons->Add(snapshots, 0, wxRIGHT, 0);
    auto* again = new ApButton(this, _("Read again"));
    auto* close = new ApButton(this, _("Close"));
    for (ApButton* b : {again, close}) {
      b->SetTheme(theme);
      b->SetMinSize(wxSize(GetCharWidth() * 14, GetCharHeight() * 2 + 8));
      buttons->Add(b, 0, wxLEFT, 8);
    }
    top->Add(buttons, 0, wxEXPAND | wxALL, 12);
    again->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
      m_asked = m_plugin->Now();
      m_plugin->ReadPilotSettings();
    });
    close->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) { Close(); });
    Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent&) { Destroy(); });
    m_refresh.SetOwner(this);
    Bind(wxEVT_TIMER, [this](wxTimerEvent&) { ShowValues(); });
    m_refresh.Start(500);
    for (wxWindow* w : m_installation) w->Hide();
    SetSizerAndFit(top);
    m_asked = m_plugin->Now();
    m_plugin->ReadPilotSettings();
  }

private:
  static wxString UnitText(const char* unit) {
    std::string u(unit);
    if (u == "deg") return L"\u00B0";
    if (u == "deg/min") return L"\u00B0/min";
    return wxString::FromUTF8(unit);
  }

  static wxString Shown(const pilot_settings::Definition& d, double v) {
    if (!d.choices.empty()) {
      auto i = static_cast<size_t>(v);
      return v >= 0 && i < d.choices.size() && d.choices[i][0] ? Tr(d.choices[i])
                                                                : wxString::Format("%.0f", v);
    }
    if (d.zero_label && v == 0) return Tr(d.zero_label);
    return wxString::Format("%.*f", d.decimals, v);
  }

  /** Value with its unit; no unit after a label such as "Auto". */
  static wxString WithUnit(const pilot_settings::Definition& d, double v) {
    wxString unit = UnitText(d.unit);
    if ((d.zero_label && v == 0) || unit.empty()) return Shown(d, v);
    return unit.StartsWith(L"\u00B0") ? Shown(d, v) + unit : Shown(d, v) + " " + unit;
  }

  /** Value of a raw setting as text, with unit and profile, for a list. */
  static wxString Describe(const pilot_settings::Definition& d, uint32_t raw) {
    simnet::KeyValue kv;
    for (int i = 0; i < 4; i++) kv.value.push_back(static_cast<uint8_t>((raw >> (8 * i)) & 0xFF));
    auto v = pilot_settings::Decode(d, kv);
    if (!v) return "NA";
    return WithUnit(d, *v);
  }

  /** Take a snapshot now, or go back to an earlier one. */
  void ShowSnapshots() {
    auto list = m_plugin->Snapshots();
    wxArrayString choices;
    choices.Add(_("Take a snapshot now"));
    for (const auto& snap : list)
      choices.Add(wxString::Format(_("%s, %s (%zu values)"), wxString::FromUTF8(snap.time.c_str()),
                                   wxString::FromUTF8(snap.reason.c_str()),
                                   snap.entries.size()));
    wxSingleChoiceDialog pick(this, _("Take a snapshot of the settings now, or choose one to go back to."),
                              _("Snapshots"), choices);
    if (pick.ShowModal() != wxID_OK) return;
    int missing = 0;
    if (pick.GetSelection() == 0) {
      auto snap = m_plugin->TakeSnapshot(std::string(wxString(_("Taken by hand")).utf8_str()), &missing);
      bool ok = m_plugin->SaveSnapshot(snap);
      SetNote(!ok ? _("The snapshot could not be saved.")
                  : missing ? wxString::Format(_("Snapshot saved: %zu values, %d not read. Read again "
                                                 "and take another for a complete one."),
                                               snap.entries.size(), missing)
                            : wxString::Format(_("Snapshot saved: %zu values."), snap.entries.size()));
      return;
    }
    const auto& snap = list[static_cast<size_t>(pick.GetSelection() - 1)];
    // What differs from now, among the settings the plugin may change.
    const PilotState& s = m_plugin->State();
    auto pilot = s.Address(m_plugin->Now());
    auto rudder = m_plugin->RudderNode();
    std::vector<NavicoAutopilotPi::SettingWrite> writes;
    wxString lines;
    for (const auto& e : snap.entries) {
      const auto* d = pilot_settings::Find(e.key);
      if (!d || !pilot_settings::Editable(e.key)) continue;
      auto a = e.node == pilot_settings::Node::kPilot ? pilot : rudder;
      if (!a) continue;
      auto kv = d->per_profile ? s.Setting(*a, e.key, e.instance) : s.Setting(*a, e.key);
      std::optional<uint32_t> now;
      if (kv) now = kv->Unsigned(static_cast<size_t>(d->bytes));
      if (now && *now == e.raw) continue;
      wxString name = Tr(d->name);
      for (const auto& prof : pilot_settings::Profiles())
        if (d->per_profile && prof.instance == e.instance)
          name += " (" + Tr(prof.name) + ")";
      lines += wxString::Format("%s: %s -> %s\n", name, now ? Describe(*d, *now) : wxString("?"),
                                Describe(*d, e.raw));
      writes.push_back({*a, e.key, e.raw, d->bytes, d->per_profile ? e.instance : uint8_t{0xFF}});
    }
    if (writes.empty()) {
      SetNote(_("The settings are the same as in that snapshot."));
      return;
    }
    wxString question = wxString::Format(_("Go back to the snapshot of %s?\n\n%s"),
                                         wxString::FromUTF8(snap.time.c_str()), lines);
    if (wxMessageBox(question, _("Snapshots"), wxYES_NO | wxICON_QUESTION, this) != wxYES) return;
    auto before = m_plugin->TakeSnapshot(
        std::string(wxString::Format(_("Before going back to %s"),
                                     wxString::FromUTF8(snap.time.c_str()))
                        .utf8_str()),
        &missing);
    if (!m_plugin->SaveSnapshot(before) &&
        wxMessageBox(_("The snapshot of the current settings could not be saved. Change anyway?"),
                     _("Snapshots"), wxYES_NO | wxICON_WARNING, this) != wxYES)
      return;
    m_snapshot_taken = true;
    m_plugin->WritePilotSettings(writes);
    SetNote(wxString::Format(_("Values sent: %zu. The window shows what the autopilot answers."),
                             writes.size()));
  }

  void InstallationPressed() {
    double now = m_plugin->Now();
    if (now - m_install_first > 3) {
      m_install_first = now;
      m_install_presses = 0;
    }
    if (++m_install_presses < 3) {
      SetNote(wxString::Format(_("Press %d more times to show the installation settings."),
                               3 - m_install_presses));
      return;
    }
    for (wxWindow* w : m_installation) w->Show();
    m_install_button->Hide();
    SetNote(_("Installation settings: boat, drive and rudder. Change them only if you know "
              "what they do."));
    GetSizer()->SetSizeHints(this);
    Fit();
  }

  /** Ask for a new value, confirm, send, and check the pilot's answer. */
  void Change(const pilot_settings::Definition& d, uint8_t instance = 0xFF) {
    const PilotState& s = m_plugin->State();
    auto address = d.node == pilot_settings::Node::kPilot ? s.Address(m_plugin->Now())
                                                          : m_plugin->RudderNode();
    auto limits = pilot_settings::Editable(d.key);
    if (!address || !limits) return;
    std::optional<double> current;
    auto kv_now = instance == 0xFF ? s.Setting(*address, d.key) : s.Setting(*address, d.key, instance);
    if (kv_now) current = pilot_settings::Decode(d, *kv_now);
    wxString title = Tr(d.name);
    for (const auto& prof : pilot_settings::Profiles())
      if (d.per_profile && prof.instance == instance)
        title += " (" + Tr(prof.name) + ")";
    if (!d.choices.empty()) {
      // A choice, e.g. wind mode: pick from the names.
      wxArrayString names;
      std::vector<int> values;
      for (size_t i = 0; i < d.choices.size(); i++) {
        if (!d.choices[i][0] || i < limits->min || i > limits->max) continue;
        names.Add(Tr(d.choices[i]));
        values.push_back(static_cast<int>(i));
      }
      wxSingleChoiceDialog pick(this, title, title, names);
      if (current)
        for (size_t i = 0; i < values.size(); i++)
          if (values[i] == static_cast<int>(*current)) pick.SetSelection(static_cast<int>(i));
      if (pick.ShowModal() != wxID_OK) return;
      Send(d, instance, title, current, values[static_cast<size_t>(pick.GetSelection())]);
      return;
    }
    wxString range = wxString::Format(_("Between %g and %g %s."), limits->min, limits->max,
                                      UnitText(d.unit));
    if (d.zero_label) range += " " + wxString::Format(_("0 = %s."), Tr(d.zero_label));
    wxTextEntryDialog ask(this, range, title,
                          !current ? wxString()
                          : d.zero_label && *current == 0 ? wxString("0")
                                                          : Shown(d, *current));
    if (ask.ShowModal() != wxID_OK) return;
    wxString text = ask.GetValue();
    text.Replace(",", ".");
    double v;
    if (!text.ToCDouble(&v) || v < limits->min || v > limits->max) {
      wxMessageBox(range, title, wxOK | wxICON_WARNING, this);
      return;
    }
    Send(d, instance, title, current, v);
  }

  /** Confirm, take a snapshot the first time, send and wait for the answer. */
  void Send(const pilot_settings::Definition& d, uint8_t instance, const wxString& title,
            std::optional<double> current, double v) {
    const PilotState& s = m_plugin->State();
    auto address = d.node == pilot_settings::Node::kPilot ? s.Address(m_plugin->Now())
                                                          : m_plugin->RudderNode();
    if (!address) return;
    if (current && std::fabs(*current - v) < 1e-9) return;
    wxString question = wxString::Format(_("Change %s from %s to %s?"), title,
                                         current ? WithUnit(d, *current) : wxString("?"),
                                         WithUnit(d, v));
    if (wxMessageBox(question, _("Change autopilot setting"), wxYES_NO | wxICON_QUESTION, this) !=
        wxYES)
      return;
    // A way back: the settings as they are now, once per window, before the
    // first change.
    if (!m_snapshot_taken) {
      int missing = 0;
      auto snap = m_plugin->TakeSnapshot(
          std::string(wxString::Format(_("Before changing %s"), title).utf8_str()), &missing);
      if (!m_plugin->SaveSnapshot(snap) &&
          wxMessageBox(_("The snapshot of the current settings could not be saved. Change anyway?"),
                       _("Change autopilot setting"), wxYES_NO | wxICON_WARNING, this) != wxYES)
        return;
      m_snapshot_taken = true;
    }
    uint32_t raw = pilot_settings::Encode(d, v);
    m_write_sent = m_plugin->Now();
    m_write = WriteState{*address, &d, v, instance, title};
    if (!m_plugin->WritePilotSetting(*address, d.key, raw, d.bytes, instance)) {
      m_write.reset();
      SetNote(_("Sending failed."));
    }
  }

  void SetNote(const wxString& note) {
    if (m_note->GetLabel() == note) return;
    m_note->SetLabel(note);
    m_note->Wrap(GetClientSize().GetWidth() - 24);
    Layout();
  }

  void ShowValues() {
    const PilotState& s = m_plugin->State();
    auto pilot = s.Address(m_plugin->Now());
    auto rudder = m_plugin->RudderNode();
    int answered = 0;
    size_t i = 0;
    for (const auto& d : pilot_settings::All()) {
      if (d.per_profile) continue;
      auto a = d.node == pilot_settings::Node::kPilot ? pilot : rudder;
      wxString text = "...";
      bool show_unit = true;
      if (a) {
        if (auto kv = s.Setting(*a, d.key)) {
          answered++;
          auto v = pilot_settings::Decode(d, *kv);
          if (!v)
            text = "NA";
          else if (d.zero_label && *v == 0) {
            text = Tr(d.zero_label);
            show_unit = false;
          }
          else
            text = Shown(d, *v);
        }
      }
      if (m_values[i]->GetLabel() != text) m_values[i]->SetLabel(text);
      wxString unit = show_unit ? UnitText(d.unit) : wxString();
      if (m_units[i]->GetLabel() != unit) m_units[i]->SetLabel(unit);
      i++;
    }
    const auto& profiles = pilot_settings::Profiles();
    for (size_t r = 0; pilot && r < m_profile_defs.size(); r++) {
      const auto& d = *m_profile_defs[r];
      for (size_t p = 0; p < profiles.size(); p++) {
        wxString text = "...";
        if (auto kv = s.Setting(*pilot, d.key, profiles[p].instance)) {
          answered++;
          auto v = pilot_settings::Decode(d, *kv);
          text = v ? Shown(d, *v) : wxString("NA");
        }
        m_profile_cells[r][p](text);
      }
    }
    if (m_write) {
      // Answer to a change: the pilot sends back the value it took.
      double now = m_plugin->Now();
      const auto& w = *m_write;
      // A set with instance FF is answered with FF, one for a profile with
      // that profile.
      auto kv = w.instance == 0xFF ? s.Setting(w.address, w.def->key)
                                   : s.Setting(w.address, w.def->key, w.instance);
      double when = w.instance == 0xFF ? s.SettingTime(w.address, w.def->key)
                                       : s.SettingTime(w.address, w.def->key, w.instance);
      bool answered_now = kv && when >= m_write_sent;
      if (answered_now) {
        auto v = pilot_settings::Decode(*w.def, *kv);
        wxString name = w.title;
        wxString got = v ? WithUnit(*w.def, *v) : wxString("NA");
        double step = std::fabs(w.def->scale) * 1.5 + 1e-9;
        if (v && std::fabs(*v - w.value) <= std::max(step, std::fabs(w.value) * 0.02))
          SetNote(wxString::Format(_("%s set to %s."), name, got));
        else
          SetNote(wxString::Format(_("%s: the autopilot answered %s."), name, got));
        m_write.reset();
      } else if (now - m_write_sent > 3) {
        SetNote(_("No answer from the autopilot: the setting may not have changed. Read again to check."));
        m_write.reset();
      } else {
        SetNote(_("Waiting for the autopilot..."));
      }
      return;
    }
    if (!pilot)
      SetNote(_("No autopilot found."));
    else if (answered == 0 && !m_plugin->ReadingSettings() && m_plugin->Now() - m_asked > 3)
      SetNote(_("The autopilot did not answer. It may accept settings requests only from its own displays."));
  }

  struct WriteState {
    uint8_t address;
    const pilot_settings::Definition* def;
    double value;
    uint8_t instance;
    wxString title;
  };

  NavicoAutopilotPi* m_plugin;
  std::vector<wxStaticText*> m_values;
  std::vector<wxStaticText*> m_units;
  std::vector<wxWindow*> m_all_rows;
  std::vector<wxWindow*> m_installation;
  ApButton* m_install_button = nullptr;
  double m_install_first = -1e9;
  int m_install_presses = 0;
  bool m_snapshot_taken = false;
  std::vector<const pilot_settings::Definition*> m_profile_defs;
  std::vector<std::vector<std::function<void(const wxString&)>>> m_profile_cells;
  wxStaticText* m_note;
  wxTimer m_refresh;
  double m_asked = 0;
  std::optional<WriteState> m_write;
  double m_write_sent = 0;
};

// ---- NAV confirmation ---------------------------------------------------------

/**
 * After NAV the pilot waits 10 s for a confirmation (as a Triton2 or an MFD
 * asks for one). This window asks it inside OpenCPN; Confirm sends what a
 * Triton2 sends.
 */
class NavConfirmDialog : public PluginDialog {
public:
  NavConfirmDialog(wxWindow* parent, NavicoAutopilotPi* plugin, const Theme& theme)
      : PluginDialog(parent, _("Start navigation"), wxCAPTION | wxSTAY_ON_TOP),
        m_plugin(plugin) {
    SetBackgroundColour(theme.back);
    SetForegroundColour(theme.text);
    auto* top = new wxBoxSizer(wxVERTICAL);
    m_question = new wxStaticText(this, wxID_ANY, "");
    wxFont f = m_question->GetFont();
    f.SetPointSize(f.GetPointSize() * 5 / 4);
    f.SetWeight(wxFONTWEIGHT_BOLD);
    m_question->SetFont(f);
    m_question->SetForegroundColour(theme.text);
    top->Add(m_question, 0, wxALL, 12);
    m_detail = new wxStaticText(this, wxID_ANY, "");
    m_detail->SetForegroundColour(theme.text);
    top->Add(m_detail, 0, wxLEFT | wxRIGHT, 12);
    m_countdown = new wxStaticText(this, wxID_ANY, "");
    m_countdown->SetForegroundColour(theme.label);
    top->Add(m_countdown, 0, wxALL, 12);

    auto* buttons = new wxBoxSizer(wxHORIZONTAL);
    auto* cancel = new ApButton(this, _("Cancel"));
    auto* confirm = new ApButton(this, _("Confirm"));
    for (ApButton* b : {cancel, confirm}) {
      b->SetTheme(theme);
      b->SetMinSize(wxSize(GetCharWidth() * 14, GetCharHeight() * 3));
      buttons->Add(b, 0, wxLEFT, 8);
    }
    top->Add(buttons, 0, wxALIGN_RIGHT | wxALL, 12);
    confirm->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
      m_plugin->ConfirmNav();
      m_done = true;
      Hide();
    });
    cancel->Bind(wxEVT_BUTTON, [this](wxCommandEvent&) {
      m_done = true;
      Hide();
    });
    SetSizerAndFit(top);
  }

  /** Update texts; seconds_left counts down the pilot's 10 s. */
  void SetInfo(const wxString& waypoint, std::optional<double> bearing, bool magnetic,
              int seconds_left) {
    wxString q = waypoint.empty() ? wxString(_("Steer to the waypoint?"))
                                  : wxString::Format(_("Steer to waypoint %s?"), waypoint);
    wxString d = bearing ? wxString::Format(_("The autopilot will turn to %s%s."), Three(bearing),
                                            magnetic ? L"°M" : L"°T")
                         : wxString(_("The autopilot will turn to the waypoint."));
    wxString c = wxString::Format(_("%d s left to confirm"), std::max(0, seconds_left));
    bool resize = false;
    if (m_question->GetLabel() != q) { m_question->SetLabel(q); resize = true; }
    if (m_detail->GetLabel() != d) { m_detail->SetLabel(d); resize = true; }
    if (m_countdown->GetLabel() != c) m_countdown->SetLabel(c);
    if (resize) Fit();
  }

  bool Done() const { return m_done; }

private:
  NavicoAutopilotPi* m_plugin;
  wxStaticText* m_question;
  wxStaticText* m_detail;
  wxStaticText* m_countdown;
  bool m_done = false;
};

// ---- panel ----------------------------------------------------------------------

ApPanel::ApPanel(wxWindow* parent, NavicoAutopilotPi* plugin)
    : wxPanel(parent, wxID_ANY), m_plugin(plugin) {
  auto* top = new wxBoxSizer(wxVERTICAL);
  m_display = new ApDisplay(this);
  top->Add(m_display, 1, wxEXPAND | wxALL, 4);

  auto* modes = new wxGridSizer(2, 3, 4, 4);
  AddButton(modes, _("STBY"), Command::kStandby);
  AddButton(modes, _("AUTO"), Command::kAuto);
  AddButton(modes, _("WIND"), Command::kWind);
  AddButton(modes, _("NAV"), Command::kNav);
  AddButton(modes, _("NO DRIFT"), Command::kNoDrift);
  AddButton(modes, _("TACK"), Command::kTack);
  top->Add(modes, 0, wxEXPAND | wxLEFT | wxRIGHT | wxTOP, 6);

  auto* course = new wxGridSizer(1, 4, 4, 4);
  AddButton(course, "-10", Command::kPort10);
  AddButton(course, "-1", Command::kPort1);
  AddButton(course, "+1", Command::kStarboard1);
  AddButton(course, "+10", Command::kStarboard10);
  top->Add(course, 0, wxEXPAND | wxALL, 6);

  m_display->SetOnSettings([this]() {
    auto* dlg = new SettingsDialog(wxGetTopLevelParent(this), m_plugin,
                                   ThemeFor(m_plugin->ColorScheme()));
    // Next to the panel, which OpenCPN keeps above other windows.
    wxRect r = GetScreenRect();
    dlg->Move(r.GetRight() + 8, r.GetTop());
    dlg->Show();
  });

  m_message = new wxStaticText(this, wxID_ANY, "", wxDefaultPosition, wxSize(-1, -1),
                               wxST_NO_AUTORESIZE);
  m_message->SetMinSize(wxSize(GetCharWidth() * 34, GetCharHeight() * 2));
  top->Add(m_message, 0, wxEXPAND | wxLEFT | wxRIGHT | wxBOTTOM, 6);

  SetSizerAndFit(top);
  ApplyColours(m_plugin->ColorScheme());
  UpdateState(m_plugin->Now());
}

ApPanel::~ApPanel() {
  m_confirm = nullptr;
  PluginDialog::DeleteAll();
}

wxWindow* ApPanel::AddButton(wxSizer* sizer, const wxString& label, Command c) {
  auto* b = new ApButton(this, label);
  // Large enough to hit on a touch screen.
  b->SetMinSize(wxSize(GetCharWidth() * 9, GetCharHeight() * 3));
  b->Bind(wxEVT_BUTTON, [this, c](wxCommandEvent&) {
    if (c == Command::kTack) {
      OpenTackDialog();
      return;
    }
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

  DisplayData d;
  d.alive = alive;
  d.mode = s.Mode();
  d.magnetic = s.TargetMagnetic();
  double hdg = m_plugin->Heading(d.magnetic);
  if (!std::isnan(hdg)) d.heading = hdg;
  if (alive) d.target = Deg(s.TargetHeading());
  if (auto nav = s.NavData(now)) {
    d.wpt_bearing = Deg(nav->bearing);
    d.waypoint = m_plugin->DestinationName();
    if (nav->distance) d.dtw_nm = *nav->distance / 1852.0;
  }
  d.stw = m_plugin->UseStw();
  if (d.stw) {
    if (auto v = s.SpeedThroughWater(now)) d.sog = *v * 3600.0 / 1852.0;
  } else {
    double sog = m_plugin->Sog();
    if (!std::isnan(sog)) d.sog = sog;
  }
  d.nfu = m_plugin->NfuActive();
  d.awa = Deg(s.ApparentWind(now));
  if (alive) d.set_wind = Deg(s.TargetWindAngle());
  d.rudder = Deg(s.Rudder(now));
  if (address) {
    wxString model = "AP";
    for (const auto& c : s.Candidates(now)) {
      if (c.address != *address) continue;
      model = c.model == 0x64 ? "NAC" : "AC";
      if (!c.model_id.empty()) {
        // "NAC-3 _Autopilot" -> "NAC-3"
        std::string m = c.model_id.substr(0, c.model_id.find(" _"));
        while (!m.empty() && m.back() == ' ') m.pop_back();
        model = wxString::FromUTF8(m.c_str());
      }
    }
    d.badge = wxString::Format("%s %d", model, *address);
  }
  m_display->Set(d);

  // Buttons. STBY whenever a pilot is selected; the rest only with fresh data.
  // In standby the course buttons are non-follow-up (rudder while held).
  Mode m = s.Mode();
  bool steering = alive && (m == Mode::kAuto || m == Mode::kWind || m == Mode::kNoDrift);
  bool nfu_buttons = alive && m == Mode::kStandby;
  if (nfu_buttons != m_nfu_buttons) SetNfuButtons(nfu_buttons);
  for (auto& [c, b] : m_buttons) {
    bool on = control && address.has_value();
    switch (c) {
      case Command::kStandby: break;
      case Command::kAuto:
      case Command::kWind:
      case Command::kNav:
      case Command::kNoDrift: on = on && alive; break;
      case Command::kTack: on = on && alive && m == Mode::kWind; break;
      default: on = on && (steering || nfu_buttons); break;  // course buttons, NFU in standby
    }
    if (b->IsEnabled() != on) b->Enable(on);
  }
  // Safety net: no NFU outside standby or without data.
  if (m_plugin->NfuActive() && !nfu_buttons) m_plugin->StopNfu();

  // One line of explanation, most important first; then the pilot's alarms.
  wxString msg = m_plugin->LastMessage();
  if (msg.empty()) {
    if (!m_plugin->Driver())
      msg = _("Choose the NMEA 2000 connection in the plugin settings.");
    else if (!address)
      msg = _("No autopilot found");
    else if (!alive)
      msg = _("No data from the autopilot.");
    else if (s.Unconfirmed(now))
      msg = _("No reply from the autopilot. Is transmitting enabled on the gateway?");
    else if (s.NotFollowed(now))
      msg = _("The autopilot did not change mode.");
  }
  wxString note = m_plugin->RouteNote();
  if (!note.empty()) {
    if (!msg.empty()) msg += "\n";
    msg += note;
  }
  for (const auto& a : s.Alarms(now)) {
    if (a.text.empty()) continue;
    if (!msg.empty()) msg += "\n";
    msg += wxString::Format(_("Alarm: %s"), wxString::FromUTF8(a.text.c_str()));
  }
  if (m_message->GetLabel() != msg) {
    m_message->SetLabel(msg);
    m_message->Wrap(m_message->GetSize().GetWidth());
  }

  UpdateConfirmation(now);
}

void ApPanel::UpdateConfirmation(double now) {
  const PilotState& s = m_plugin->State();
  bool awaiting = s.AwaitsConfirmation(now);
  if (!awaiting || (m_confirm && m_confirm->Done())) {
    if (m_confirm) {
      m_confirm->Destroy();
      m_confirm = nullptr;
    }
    if (!awaiting) return;
  }
  if (!m_confirm) {
    if (m_confirm_handled == s.AwaitingSince()) return;  // confirmed or cancelled
    m_confirm_handled = s.AwaitingSince();
    m_confirm = new NavConfirmDialog(wxGetTopLevelParent(this), m_plugin,
                                     ThemeFor(m_plugin->ColorScheme()));
    m_confirm->CentreOnParent();
    m_confirm->Show();
    m_confirm->Raise();
  }
  auto nav = s.NavData(now);
  std::optional<double> bearing = nav ? Deg(nav->bearing) : std::nullopt;
  int left = static_cast<int>(std::ceil(PilotState::kNavConfirmWindow - (now - s.AwaitingSince())));
  m_confirm->SetInfo(m_plugin->DestinationName(), bearing, s.TargetMagnetic(), left);
}

void ApPanel::SetNfuButtons(bool nfu) {
  // As on a Triton2: in standby the course buttons are non-follow-up, the
  // rudder moves while one is held (-1/-10 port, +1/+10 starboard).
  m_nfu_buttons = nfu;
  if (!nfu) m_plugin->StopNfu();
  for (Command c : {Command::kPort1, Command::kPort10, Command::kStarboard1, Command::kStarboard10}) {
    auto* b = static_cast<ApButton*>(m_buttons[c]);
    if (!nfu) {
      b->SetHold(nullptr, nullptr);
      continue;
    }
    auto side = (c == Command::kPort1 || c == Command::kPort10) ? simnet::NfuSide::kPort
                                                                : simnet::NfuSide::kStarboard;
    b->SetHold([this, side] { m_plugin->StartNfu(side); }, [this] { m_plugin->StopNfu(); });
  }
}

void ApPanel::OpenTackDialog() {
  const PilotState& s = m_plugin->State();
  double now = m_plugin->Now();
  bool magnetic = s.TargetMagnetic();
  std::optional<double> heading;
  double h = m_plugin->Heading(magnetic);
  if (!std::isnan(h)) heading = h;
  auto wind = Deg(s.TargetWindAngle());
  if (!wind) wind = Deg(s.ApparentWind(now));
  auto* dlg = new TackDialog(wxGetTopLevelParent(this), m_plugin, ThemeFor(m_plugin->ColorScheme()),
                             heading, wind, magnetic);
  dlg->CentreOnParent();
  dlg->Show();
}

void ApPanel::ApplyColours(PI_ColorScheme cs) {
  Theme t = ThemeFor(cs);
  SetBackgroundColour(t.back);
  SetForegroundColour(t.text);
  m_message->SetBackgroundColour(t.back);
  m_message->SetForegroundColour(t.text);
  m_display->SetTheme(t);
  for (auto& [c, b] : m_buttons) static_cast<ApButton*>(b)->SetTheme(t);
  Refresh();
}
