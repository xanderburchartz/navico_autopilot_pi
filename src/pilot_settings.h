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
  /** One value per steering profile (see Profiles()). */
  bool per_profile = false;
  /** Bits of the raw value that hold the setting, and bits a display sets
   *  as well when it writes (wind mode: mode in bits 0-2, Triton2 adds 0x18). */
  uint32_t mask = 0xFFFFFFFF;
  uint32_t set_bits = 0;
};

/** Steering profiles of a NAC-3 set to Sail, by instance (byte 3): 0 ENGINE,
 *  1 REACH, 2 RUN, 3 BEAT, 4 WORK4, 5 WORK5 (names in 130846 under 18/8008);
 *  instance FF answers with the active one. A Triton2 shows 0 as "Low
 *  speed" and 1 as "High speed" and not the others (13 Nov 2026). */
struct Profile {
  uint8_t instance;
  const char* name;  ///< untranslated
};
const std::vector<Profile>& Profiles();
/** Profile name, answered in 130846 per instance ("ENGINE"). */
constexpr simnet::SettingKey kProfileName{0x18, 0x8008};

/**
 * The pilot's settings at one moment, to go back to after a change. Raw
 * values per node (pilot or rudder feedback), key and instance; addresses
 * are not stored, they can change.
 */
struct Snapshot {
  struct Entry {
    Node node;
    simnet::SettingKey key;
    uint8_t instance;  ///< 0, or the profile; FF is not stored
    uint32_t raw;
  };
  std::string time;    ///< "2026-11-13 16:30:05", local time
  std::string reason;  ///< e.g. "Before changing Rudder deadband"
  std::vector<Entry> entries;
};
std::string Serialize(const Snapshot& s);
std::optional<Snapshot> ParseSnapshot(const std::string& text);
/** The definition of \p key, if the plugin knows it. */
const Definition* Find(simnet::SettingKey key);

/** Text of a 130846 value, up to the first NUL. */
std::string Text(const simnet::KeyValue& kv);

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
