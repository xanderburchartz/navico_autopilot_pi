/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

/**
 * \file pilot_state.h
 * Which device on the bus is the pilot computer, and what it reports.
 *
 * No wxWidgets and no OpenCPN dependencies. Time is passed in as seconds
 * from any monotonic clock, so the logic can be tested without waiting.
 */
#ifndef NAVICO_AP_PILOT_STATE_H
#define NAVICO_AP_PILOT_STATE_H

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "simnet.h"

class PilotState {
public:
  /** A device that reports a pilot mode (65305, report "Mode"). */
  struct Candidate {
    uint8_t address = 0;
    uint8_t model = 0;  ///< 0x64 NAC, 0x00 AC
    double last_seen = 0;
    std::optional<simnet::Name> name;
    std::string model_id;  ///< from 126996, empty if not seen
  };

  /** Pilot mode older than this: "no data". Normally a NAC-3 reports its mode
   *  about twice a second, but the rate can be set lower to reduce bus load; on
   *  one boat it was every 2 to 4 s, with longer gaps. 10 s covers both. */
  static constexpr double kAliveTimeout = 10.0;
  /** Candidates not heard from for this long are forgotten. */
  static constexpr double kCandidateTimeout = 30.0;
  /** A command without a matching report within this time: "not confirmed". */
  static constexpr double kConfirmTimeout = 1.5;
  /** Echoed command without a matching report after this: "not followed". */
  static constexpr double kFollowTimeout = 3.0;
  /** How long the pilot waits for a NAV confirmation (measured on a NAC-3). */
  static constexpr double kNavConfirmWindow = 10.0;
  /** Rudder and wind values older than this are not shown. */
  static constexpr double kSensorTimeout = 5.0;
  /** An active alarm is repeated every ~8.8 s; forget it after this. */
  static constexpr double kAlarmTimeout = 30.0;
  /** A wind source must be quiet this long before another one is taken. */
  static constexpr double kWindSourceHold = 5.0;

  /** A device that sends a rudder angle (127245). */
  struct RudderSource {
    uint8_t address = 0;
    double angle = 0;  ///< rad, positive = starboard
    double last_seen = 0;
    std::optional<simnet::Name> name;
    std::string model_id;  ///< e.g. "NAC-3 _Rudder feedback"
  };

  /** An alarm the pilot reports as active. */
  struct ActiveAlarm {
    uint16_t id = 0;
    std::string text;  ///< empty until 130856 arrived
  };

  /** Fixed pilot address from the settings, or none for automatic. */
  void SetFixedAddress(std::optional<uint8_t> address);
  std::optional<uint8_t> FixedAddress() const { return m_fixed; }

  /** NAME of the chosen pilot, used to follow it when its address changes. */
  void SetPilotName(uint64_t name) { m_pilot_name = name; }
  uint64_t PilotName() const { return m_pilot_name; }

  void Handle(const simnet::Message& m, double now);

  /** The pilot to talk to: the fixed one, else the most likely one found. */
  std::optional<uint8_t> Address(double now) const;
  std::vector<Candidate> Candidates(double now) const;
  /** Most likely pilot among the candidates (automatic choice). */
  std::optional<uint8_t> AutoPilot(double now) const;

  /** A recent mode report from the chosen pilot. */
  bool Alive(double now) const;
  simnet::Mode Mode() const { return m_mode; }
  std::optional<bool> Engaged() const { return m_engaged; }
  /** Heading to steer (rad) from 127237, none in standby. */
  std::optional<double> TargetHeading() const { return m_target; }
  bool TargetMagnetic() const { return m_target_magnetic; }
  /** Wind angle to hold (rad, -pi..pi, negative = port) from 65341. */
  std::optional<double> TargetWindAngle() const { return m_wind; }

  /** Remember a command (and its bytes) so the next reports can confirm it. */
  void CommandSent(simnet::Command c, const std::vector<uint8_t>& data, double now);
  /** The last command got no echo (130851) and no matching report in time. */
  bool Unconfirmed(double now) const;
  /** The last command was echoed (executed) but the mode or target did not
   *  change, for example NAV without navigation data. */
  bool NotFollowed(double now) const;

  /** The pilot waits for a confirmation (after NAV); since when. */
  bool AwaitsConfirmation(double now) const;
  double AwaitingSince() const { return m_awaiting_since; }

  /** Rudder: NAME of the chosen source, 0 for automatic (only if there is
   *  exactly one source). */
  void SetRudderName(uint64_t name) { m_rudder_name = name; }
  uint64_t RudderName() const { return m_rudder_name; }
  std::vector<RudderSource> RudderSources(double now) const;
  /** Most likely rudder sensor (automatic choice), by address. */
  std::optional<uint8_t> AutoRudder(double now) const;
  std::optional<double> Rudder(double now) const;

  /** Apparent wind angle (rad, -pi..pi, negative = port) from 130306. */
  std::optional<double> ApparentWind(double now) const;

  std::vector<ActiveAlarm> Alarms(double now) const;

  /** Latest value of a setting from a 130845/130846 reply by \p address. */
  std::optional<simnet::KeyValue> Setting(uint8_t address, simnet::SettingKey key) const;

  /** Speed through the water (m/s) from 128259. */
  std::optional<double> SpeedThroughWater(double now) const;

  /** Bearing (rad) and distance (m) to the waypoint from 129284, if valid. */
  std::optional<simnet::NavData> NavData(double now) const;

private:
  bool IsPilot(uint8_t source, double now) const;

  std::optional<uint8_t> m_fixed;
  uint64_t m_pilot_name = 0;
  std::map<uint8_t, Candidate> m_candidates;
  std::map<uint8_t, simnet::Name> m_names;  // address -> NAME from 60928
  std::map<uint8_t, std::string> m_models;  // address -> model ID from 126996
  std::string ModelId(uint8_t address) const;

  simnet::Mode m_mode = simnet::Mode::kUnknown;
  double m_mode_time = -1e9;
  std::optional<bool> m_engaged;
  std::optional<double> m_target;
  bool m_target_magnetic = false;
  std::optional<double> m_wind;

  uint16_t m_mode_bits = 0;
  double m_awaiting_since = -1e9;

  struct AlarmEntry {
    uint8_t state = 0;
    double last_seen = 0;
    std::string text;
  };
  std::map<uint16_t, AlarmEntry> m_alarms;

  uint64_t m_rudder_name = 0;
  std::map<uint8_t, RudderSource> m_rudder;
  std::map<std::pair<uint8_t, simnet::SettingKey>, simnet::KeyValue> m_settings;
  std::optional<double> m_stw;
  uint8_t m_stw_source = 255;
  double m_stw_time = -1e9;
  simnet::NavData m_nav;
  double m_nav_time = -1e9;
  std::optional<double> m_awa;
  uint8_t m_awa_source = 255;
  double m_awa_time = -1e9;

  std::optional<simnet::Command> m_pending;
  std::vector<uint8_t> m_pending_data;
  bool m_echoed = false;
  double m_pending_time = 0;
  std::optional<double> m_pending_target;  // target when a +/- was sent
  std::optional<double> m_pending_wind;
};

#endif  // NAVICO_AP_PILOT_STATE_H
