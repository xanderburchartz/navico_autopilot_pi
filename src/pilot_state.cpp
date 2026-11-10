/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "pilot_state.h"

#include <cmath>

using simnet::Command;
using simnet::Mode;

namespace {

constexpr double kPi = 3.14159265358979323846;

std::optional<Mode> ExpectedMode(Command c) {
  switch (c) {
    case Command::kStandby: return Mode::kStandby;
    case Command::kAuto: return Mode::kAuto;
    case Command::kWind: return Mode::kWind;
    case Command::kNav: return Mode::kNav;
    case Command::kNoDrift: return Mode::kNoDrift;
    default: return std::nullopt;  // course change: confirmed by a new target
  }
}

bool Differs(std::optional<double> a, std::optional<double> b) {
  if (!a || !b) return a.has_value() != b.has_value();
  return std::fabs(*a - *b) > 0.001;  // about 0.06 degrees
}

}  // namespace

void PilotState::SetFixedAddress(std::optional<uint8_t> address) {
  if (address == m_fixed) return;
  m_fixed = address;
  m_mode = Mode::kUnknown;
  m_mode_time = -1e9;
  m_engaged.reset();
  m_target.reset();
  m_wind.reset();
}

void PilotState::Handle(const simnet::Message& m, double now) {
  if (m.own) return;
  switch (m.pgn) {
    case simnet::kPgnAddressClaim: {
      auto name = simnet::ParseAddressClaim(m.data);
      if (!name) return;
      // A device that moved: forget its old address.
      for (auto it = m_names.begin(); it != m_names.end();) {
        if (it->second.raw == name->raw && it->first != m.source)
          it = m_names.erase(it);
        else
          ++it;
      }
      m_names[m.source] = *name;
      // Follow the chosen pilot when it claims another address.
      if (m_pilot_name && name->raw == m_pilot_name && m_fixed &&
          *m_fixed != m.source) {
        m_fixed = m.source;
      }
      return;
    }
    case simnet::kPgnDeviceStatus: {
      if (auto pm = simnet::ParsePilotMode(m.data)) {
        Candidate& c = m_candidates[m.source];
        c.address = m.source;
        c.model = pm->model;
        c.last_seen = now;
        if (IsPilot(m.source, now)) {
          m_mode = pm->mode;
          m_mode_time = now;
          if (m_pending && ExpectedMode(*m_pending) == pm->mode) m_pending.reset();
        }
      } else if (auto engaged = simnet::ParsePilotEngaged(m.data)) {
        if (IsPilot(m.source, now)) m_engaged = engaged;
      }
      return;
    }
    case simnet::kPgnHeadingControl: {
      if (!IsPilot(m.source, now)) return;
      auto h = simnet::ParseHeadingControl(m.data);
      if (!h) return;
      m_target = h->target;
      m_target_magnetic = h->magnetic;
      if (m_pending && !ExpectedMode(*m_pending) && m_mode != Mode::kWind &&
          Differs(m_target, m_pending_target))
        m_pending.reset();
      return;
    }
    case simnet::kPgnApAngle: {
      if (!IsPilot(m.source, now)) return;
      auto a = simnet::ParseApAngle(m.data);
      if (!a) return;
      if (a->mode == 3 && a->angle) {
        double w = *a->angle;
        m_wind = w > kPi ? w - 2 * kPi : w;
        if (m_pending && !ExpectedMode(*m_pending) && m_mode == Mode::kWind &&
            Differs(m_wind, m_pending_wind))
          m_pending.reset();
      } else if (a->mode != 3) {
        m_wind.reset();
      }
      return;
    }
    default:
      return;
  }
}

std::vector<PilotState::Candidate> PilotState::Candidates(double now) const {
  std::vector<Candidate> out;
  for (const auto& [addr, c] : m_candidates) {
    if (now - c.last_seen > kCandidateTimeout) continue;
    Candidate copy = c;
    auto n = m_names.find(addr);
    if (n != m_names.end()) copy.name = n->second;
    out.push_back(copy);
  }
  return out;
}

std::optional<uint8_t> PilotState::Address(double now) const {
  if (m_fixed) return m_fixed;
  auto list = Candidates(now);
  if (list.size() == 1) return list.front().address;
  return std::nullopt;
}

bool PilotState::NeedsChoice(double now) const {
  return !m_fixed && Candidates(now).size() > 1;
}

bool PilotState::IsPilot(uint8_t source, double now) const {
  auto a = Address(now);
  return a && *a == source;
}

bool PilotState::Alive(double now) const {
  return Address(now) && now - m_mode_time <= kAliveTimeout;
}

void PilotState::CommandSent(Command c, double now) {
  m_pending = c;
  m_pending_time = now;
  m_pending_target = m_target;
  m_pending_wind = m_wind;
}

bool PilotState::Unconfirmed(double now) const {
  return m_pending && now - m_pending_time > kConfirmTimeout;
}
