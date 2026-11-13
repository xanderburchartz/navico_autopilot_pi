/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "pilot_state.h"

#include <cctype>
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

bool Contains(std::string text, const char* part) {
  for (auto& ch : text) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  return text.find(part) != std::string::npos;
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
      if (m_names.count(m.source) && m_names[m.source].raw != name->raw)
        m_models.erase(m.source);  // another device at this address
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
          bool awaiting = pm->AwaitsConfirmation();
          if (awaiting && !(m_mode_bits & 0x80)) m_awaiting_since = now;
          m_mode_bits = pm->bits;
          if (m_pending && ExpectedMode(*m_pending) == pm->mode) m_pending.reset();
          // NAV accepted, now waiting for the user's confirmation.
          if (m_pending && *m_pending == Command::kNav && awaiting) m_pending.reset();
          if (m_pending && *m_pending == Command::kConfirm && !awaiting) m_pending.reset();
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
    case simnet::kPgnProductInfo: {
      if (auto model = simnet::ParseModelId(m.data)) m_models[m.source] = *model;
      return;
    }
    case simnet::kPgnCommandReply: {
      if (!IsPilot(m.source, now)) return;
      if (m_pending && simnet::EchoMatches(m_pending_data, m.data)) m_echoed = true;
      return;
    }
    case simnet::kPgnCommand: {
      if (!IsPilot(m.source, now)) return;
      auto a = simnet::ParseAlarm(m.data);
      if (!a) return;
      AlarmEntry& e = m_alarms[a->id];
      e.state = a->state;
      e.last_seen = now;
      return;
    }
    case simnet::kPgnAlarmText: {
      if (!IsPilot(m.source, now)) return;
      auto t = simnet::ParseAlarmText(m.data);
      if (t) m_alarms[t->id].text = t->text;
      return;
    }
    case simnet::kPgnRudder: {
      auto r = simnet::ParseRudder(m.data);
      if (!r) return;  // the pilot itself sends 127245 without a position
      RudderSource& src = m_rudder[m.source];
      src.address = m.source;
      src.angle = *r;
      src.last_seen = now;
      return;
    }
    case simnet::kPgnSpeed: {
      auto v = simnet::ParseSpeedThroughWater(m.data);
      if (!v) return;
      if (m_stw_source != m.source && now - m_stw_time < kWindSourceHold) return;
      m_stw = v;
      m_stw_source = m.source;
      m_stw_time = now;
      return;
    }
    case simnet::kPgnNavData: {
      auto n = simnet::ParseNavData(m.data);
      if (!n || !n->bearing) return;  // all "no value" when navigation stops
      m_nav = *n;
      m_nav_time = now;
      return;
    }
    case simnet::kPgnWind: {
      auto w = simnet::ParseWind(m.data);
      if (!w || !w->angle || w->reference != 2) return;  // apparent only
      // Several sources may send it; stay with one until it goes quiet.
      if (m_awa_source != m.source && now - m_awa_time < kWindSourceHold) return;
      double a = *w->angle;
      m_awa = a > kPi ? a - 2 * kPi : a;
      m_awa_source = m.source;
      m_awa_time = now;
      return;
    }
    default:
      return;
  }
}

std::optional<double> PilotState::SpeedThroughWater(double now) const {
  if (now - m_stw_time > kSensorTimeout) return std::nullopt;
  return m_stw;
}

std::optional<simnet::NavData> PilotState::NavData(double now) const {
  if (now - m_nav_time > kSensorTimeout) return std::nullopt;
  return m_nav;
}

bool PilotState::AwaitsConfirmation(double now) const {
  return Alive(now) && (m_mode_bits & 0x80) &&
         now - m_awaiting_since <= kNavConfirmWindow + 2;
}

std::vector<PilotState::RudderSource> PilotState::RudderSources(double now) const {
  std::vector<RudderSource> out;
  for (const auto& [addr, r] : m_rudder) {
    if (now - r.last_seen > kSensorTimeout) continue;
    RudderSource copy = r;
    auto n = m_names.find(addr);
    if (n != m_names.end()) copy.name = n->second;
    copy.model_id = ModelId(addr);
    out.push_back(copy);
  }
  return out;
}

std::optional<uint8_t> PilotState::AutoRudder(double now) const {
  // Prefer a rudder feedback unit (function 155, steering class 40) of the
  // pilot's make, and not a "virtual" one (a NAC-3 has both; only the model
  // ID from 126996 tells them apart, which OpenCPN does not pass on from
  // network gateways). Equal: the lowest address.
  std::optional<simnet::Name> pilot;
  if (auto a = Address(now)) {
    auto n = m_names.find(*a);
    if (n != m_names.end()) pilot = n->second;
  }
  std::optional<uint8_t> best;
  int best_score = -1000;
  for (const auto& r : RudderSources(now)) {
    int score = 0;
    if (r.name && r.name->function == simnet::kFunctionRudder) score += 4;
    if (r.name && r.name->device_class == 40) score += 1;
    if (r.name && pilot && r.name->manufacturer == pilot->manufacturer) score += 1;
    if (Contains(r.model_id, "virtual")) score -= 6;
    if (score > best_score) {
      best_score = score;
      best = r.address;
    }
  }
  return best;
}

std::optional<double> PilotState::Rudder(double now) const {
  auto list = RudderSources(now);
  if (m_rudder_name) {
    for (const auto& r : list)
      if (r.name && r.name->raw == m_rudder_name) return r.angle;
    return std::nullopt;
  }
  auto a = AutoRudder(now);
  for (const auto& r : list)
    if (a && r.address == *a) return r.angle;
  return std::nullopt;
}

std::optional<double> PilotState::ApparentWind(double now) const {
  if (now - m_awa_time > kSensorTimeout) return std::nullopt;
  return m_awa;
}

std::vector<PilotState::ActiveAlarm> PilotState::Alarms(double now) const {
  std::vector<ActiveAlarm> out;
  for (const auto& [id, e] : m_alarms) {
    if (e.state == 0x3A || now - e.last_seen > kAlarmTimeout) continue;
    if (e.state != 0x39 && e.state != 0x38) continue;
    out.push_back({id, e.text});
  }
  return out;
}

std::vector<PilotState::Candidate> PilotState::Candidates(double now) const {
  std::vector<Candidate> out;
  for (const auto& [addr, c] : m_candidates) {
    if (now - c.last_seen > kCandidateTimeout) continue;
    Candidate copy = c;
    auto n = m_names.find(addr);
    if (n != m_names.end()) copy.name = n->second;
    copy.model_id = ModelId(addr);
    out.push_back(copy);
  }
  return out;
}

std::optional<uint8_t> PilotState::Address(double now) const {
  if (m_fixed) return m_fixed;
  return AutoPilot(now);
}

std::string PilotState::ModelId(uint8_t address) const {
  auto it = m_models.find(address);
  return it == m_models.end() ? std::string() : it->second;
}

std::optional<uint8_t> PilotState::AutoPilot(double now) const {
  // A real Navico pilot computer claims manufacturer Navico (275); an
  // emulation (e.g. a Yacht Devices pilot posing as a Simrad AC) does not.
  std::optional<uint8_t> best;
  int best_score = -1000;
  for (const auto& c : Candidates(now)) {
    int score = 0;
    if (c.name && c.name->manufacturer == simnet::kManufacturerNavico) score += 4;
    if (c.model == 0x64) score += 1;  // NAC
    if (Contains(c.model_id, "autopilot")) score += 1;
    if (score > best_score) {  // equal: the lowest address, map order
      best_score = score;
      best = c.address;
    }
  }
  return best;
}

bool PilotState::IsPilot(uint8_t source, double now) const {
  auto a = Address(now);
  return a && *a == source;
}

bool PilotState::Alive(double now) const {
  return Address(now) && now - m_mode_time <= kAliveTimeout;
}

void PilotState::CommandSent(Command c, const std::vector<uint8_t>& data, double now) {
  m_pending = c;
  m_pending_data = data;
  m_echoed = false;
  m_pending_time = now;
  m_pending_target = m_target;
  m_pending_wind = m_wind;
}

bool PilotState::Unconfirmed(double now) const {
  return m_pending && !m_echoed && now - m_pending_time > kConfirmTimeout;
}

bool PilotState::NotFollowed(double now) const {
  return m_pending && m_echoed && now - m_pending_time > kFollowTimeout;
}
