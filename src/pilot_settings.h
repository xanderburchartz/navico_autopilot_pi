/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

/**
 * \file pilot_settings.h
 * The pilot settings the plugin knows, read over the Simnet key value
 * channel (130845). Keys and scaling from the signalk-navico-autopilot-bridge
 * author (changed on a display while listening) and from recordings; only
 * confirmed keys are listed. No wxWidgets, so it can be unit tested.
 */
#ifndef NAVICO_AP_PILOT_SETTINGS_H
#define NAVICO_AP_PILOT_SETTINGS_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "simnet.h"

namespace pilot_settings {

/** Which device holds the setting. */
enum class Node { kPilot, kRudder };

struct Definition {
  simnet::SettingKey key;
  Node node;
  const char* section;  ///< untranslated, e.g. "Boat"
  const char* name;     ///< untranslated, e.g. "Boat type"
  const char* unit;     ///< "kn", "deg", "s", "" ...
  /** Number of value bytes and whether signed. */
  int bytes;
  bool is_signed;
  double scale;  ///< shown value = raw * scale
  /** For a list: names of 0, 1, 2 ...; empty for a number. */
  std::vector<const char*> choices;
  /** If set, the raw value 0 means this (e.g. "Auto") instead of a number. */
  const char* zero_label = nullptr;
  /** Decimals to show, matching the pilot's resolution and the displays. */
  int decimals = 1;
};

const std::vector<Definition>& All();

/** Range in shown units within which the plugin lets a setting be changed;
 *  none if the plugin does not change it (boat type: may reset the steering
 *  profiles; drive voltage and rudder calibration: wrong values can harm the
 *  drive or belong to the calibration procedure). */
struct Limits {
  double min, max;
};
std::optional<Limits> Editable(simnet::SettingKey key);

/** Raw value to send for a shown value (rounded to the setting's resolution). */
uint32_t Encode(const Definition& d, double shown);

/** Shown value of \p kv for \p d: a number (scaled) or a choice index. */
std::optional<double> Decode(const Definition& d, const simnet::KeyValue& kv);

}  // namespace pilot_settings

#endif  // NAVICO_AP_PILOT_SETTINGS_H
