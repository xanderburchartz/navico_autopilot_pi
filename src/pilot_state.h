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
  };

  /** Pilot mode older than this: "no data". Normally a NAC-3 reports its mode
   *  about twice a second, but the rate can be set lower to reduce bus load; on
   *  one boat it was every 2 to 4 s, with longer gaps. 10 s covers both. */
  static constexpr double kAliveTimeout = 10.0;
  /** Candidates not heard from for this long are forgotten. */
  static constexpr double kCandidateTimeout = 30.0;
  /** A command without a matching report within this time: "not confirmed". */
  static constexpr double kConfirmTimeout = 1.5;

  /** Fixed pilot address from the settings, or none for automatic. */
  void SetFixedAddress(std::optional<uint8_t> address);
  std::optional<uint8_t> FixedAddress() const { return m_fixed; }

  /** NAME of the chosen pilot, used to follow it when its address changes. */
  void SetPilotName(uint64_t name) { m_pilot_name = name; }
  uint64_t PilotName() const { return m_pilot_name; }

  void Handle(const simnet::Message& m, double now);

  /** The pilot to talk to, or none (not found, or more than one found). */
  std::optional<uint8_t> Address(double now) const;
  std::vector<Candidate> Candidates(double now) const;
  /** Automatic mode and more than one pilot found: the user must choose. */
  bool NeedsChoice(double now) const;

  /** A recent mode report from the chosen pilot. */
  bool Alive(double now) const;
  simnet::Mode Mode() const { return m_mode; }
  std::optional<bool> Engaged() const { return m_engaged; }
  /** Heading to steer (rad) from 127237, none in standby. */
  std::optional<double> TargetHeading() const { return m_target; }
  bool TargetMagnetic() const { return m_target_magnetic; }
  /** Wind angle to hold (rad, -pi..pi, negative = port) from 65341. */
  std::optional<double> TargetWindAngle() const { return m_wind; }

  /** Remember a command so the next reports can confirm it. */
  void CommandSent(simnet::Command c, double now);
  /** The last command got no matching report in time. */
  bool Unconfirmed(double now) const;

private:
  bool IsPilot(uint8_t source, double now) const;

  std::optional<uint8_t> m_fixed;
  uint64_t m_pilot_name = 0;
  std::map<uint8_t, Candidate> m_candidates;
  std::map<uint8_t, simnet::Name> m_names;  // address -> NAME from 60928

  simnet::Mode m_mode = simnet::Mode::kUnknown;
  double m_mode_time = -1e9;
  std::optional<bool> m_engaged;
  std::optional<double> m_target;
  bool m_target_magnetic = false;
  std::optional<double> m_wind;

  std::optional<simnet::Command> m_pending;
  double m_pending_time = 0;
  std::optional<double> m_pending_target;  // target when a +/- was sent
  std::optional<double> m_pending_wind;
};

#endif  // NAVICO_AP_PILOT_STATE_H
