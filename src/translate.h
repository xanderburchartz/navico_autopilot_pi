/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#ifndef NAVICO_AP_TRANSLATE_H
#define NAVICO_AP_TRANSLATE_H

// Translations of the plugin come from its own catalog only. wxWidgets' _()
// searches every loaded catalog, so a plain _("Cancel") or _("True") could
// take OpenCPN's or another plugin's text, and the plugin's catalog could
// change theirs. Include this after the wxWidgets headers.

#include <wx/string.h>
#include <wx/translation.h>

#define NAVICO_AP_CATALOG "opencpn-navico_autopilot_pi"

#undef _
#define _(s) wxGetTranslation(wxS(s), wxS(NAVICO_AP_CATALOG))

/** Translation of a text kept in a table (marked with N_ there). */
inline wxString Tr(const char* s) {
  return wxGetTranslation(wxString::FromUTF8(s), NAVICO_AP_CATALOG);
}

#endif  // NAVICO_AP_TRANSLATE_H
