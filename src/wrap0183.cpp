/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

#include "wrap0183.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>

namespace wrap0183 {

namespace {

std::vector<std::string> Fields(const std::string& body) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : body) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur += c;
    }
  }
  out.push_back(cur);
  return out;
}

bool HexValue(const std::string& s, uint32_t& value) {
  if (s.empty() || s.size() > 8) return false;
  for (char c : s)
    if (!std::isxdigit(static_cast<unsigned char>(c))) return false;
  value = static_cast<uint32_t>(std::strtoul(s.c_str(), nullptr, 16));
  return true;
}

bool HexBytes(const std::string& s, std::vector<uint8_t>& out) {
  if (s.size() % 2) return false;
  out.clear();
  for (size_t i = 0; i < s.size(); i += 2) {
    uint32_t v;
    if (!HexValue(s.substr(i, 2), v)) return false;
    out.push_back(static_cast<uint8_t>(v));
  }
  return true;
}

/**
 * The part between '$' and '*' when the sentence starts with \p tag and its
 * checksum (if any) is right; nothing else.
 */
std::optional<std::string> Body(const std::string& sentence, const char* tag) {
  size_t start = sentence.find('$');
  if (start == std::string::npos) return std::nullopt;
  std::string s = sentence.substr(start);
  while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ')) s.pop_back();
  if (s.compare(1, 5, tag) != 0) return std::nullopt;
  size_t star = s.find('*');
  std::string body = s.substr(1, star == std::string::npos ? std::string::npos : star - 1);
  if (star != std::string::npos) {
    uint32_t sum;
    if (!HexValue(s.substr(star + 1, 2), sum) || sum != Checksum(s)) return std::nullopt;
  }
  return body;
}

std::string Finish(const std::string& without_dollar) {
  std::string s = "$" + without_dollar;
  char tail[8];
  std::snprintf(tail, sizeof tail, "*%02X\r\n", Checksum(s));
  return s + tail;
}

std::string Hex(const std::vector<uint8_t>& data, bool reversed) {
  std::string out;
  char b[3];
  for (size_t i = 0; i < data.size(); i++) {
    std::snprintf(b, sizeof b, "%02X", data[reversed ? data.size() - 1 - i : i]);
    out += b;
  }
  return out;
}

/** PDU1 PGNs carry the destination in the low byte. */
void SplitDestination(int& pgn, uint8_t& destination) {
  if (((pgn >> 8) & 0xFF) < 240) {
    destination = static_cast<uint8_t>(pgn & 0xFF);
    pgn &= ~0xFF;
  } else {
    destination = 255;
  }
}

}  // namespace

uint8_t Checksum(const std::string& sentence) {
  uint8_t sum = 0;
  size_t i = sentence.find('$');
  i = i == std::string::npos ? 0 : i + 1;
  for (; i < sentence.size() && sentence[i] != '*'; i++) sum ^= static_cast<uint8_t>(sentence[i]);
  return sum;
}

std::string EncodePcdin(int pgn, uint8_t source, const std::vector<uint8_t>& data,
                        uint8_t destination, uint32_t time) {
  if (((pgn >> 8) & 0xFF) < 240) pgn = (pgn & ~0xFF) | destination;  // PDU1
  char head[40];
  std::snprintf(head, sizeof head, "PCDIN,%06X,%08X,%02X,", pgn & 0xFFFFFF, time, source);
  return Finish(head + Hex(data, false));
}

std::optional<simnet::Message> DecodePcdin(const std::string& sentence) {
  auto body = Body(sentence, "PCDIN");
  if (!body) return std::nullopt;
  auto f = Fields(*body);
  if (f.size() != 5) return std::nullopt;
  uint32_t pgn, time, source;
  std::vector<uint8_t> data;
  if (!HexValue(f[1], pgn) || !HexValue(f[2], time) || !HexValue(f[3], source) ||
      source > 255 || !HexBytes(f[4], data))
    return std::nullopt;
  simnet::Message m;
  int p = static_cast<int>(pgn);
  SplitDestination(p, m.destination);
  m.pgn = p;
  m.source = static_cast<uint8_t>(source);
  m.data = data;
  return m;
}

bool IsFastPacket(int pgn) {
  switch (pgn) {
    case simnet::kPgnCommand:
    case simnet::kPgnCommandReply:
    case simnet::kPgnAlarmText:
    case simnet::kPgnKeyValue:
    case simnet::kPgnKeyValueLong:
    case simnet::kPgnProductInfo:
    case simnet::kPgnHeadingControl:
    case simnet::kPgnNavData:
    case simnet::kPgnRouteInfo:
      return true;
    default:
      return false;
  }
}

std::vector<std::string> EncodeMxpgn(int pgn, uint8_t priority, uint8_t destination,
                                     const std::vector<uint8_t>& data, uint8_t sequence) {
  auto sentence = [&](const std::vector<uint8_t>& frame) {
    unsigned attr = 0x8000u | ((priority & 7u) << 12) | ((frame.size() & 0xFu) << 8) | destination;
    char head[32];
    std::snprintf(head, sizeof head, "MXPGN,%06X,%04X,", pgn & 0xFFFFFF, attr);
    return Finish(head + Hex(frame, true));
  };
  std::vector<std::string> out;
  if (!IsFastPacket(pgn) && data.size() <= 8) {
    out.push_back(sentence(data));
    return out;
  }
  uint8_t seq = static_cast<uint8_t>((sequence & 7) << 5);
  std::vector<uint8_t> frame = {seq, static_cast<uint8_t>(data.size())};
  size_t i = 0;
  for (; i < data.size() && frame.size() < 8; i++) frame.push_back(data[i]);
  while (frame.size() < 8) frame.push_back(0xFF);
  out.push_back(sentence(frame));
  for (uint8_t n = 1; i < data.size(); n++) {
    frame = {static_cast<uint8_t>(seq | (n & 0x1F))};
    for (; i < data.size() && frame.size() < 8; i++) frame.push_back(data[i]);
    while (frame.size() < 8) frame.push_back(0xFF);
    out.push_back(sentence(frame));
  }
  return out;
}

std::optional<Frame> DecodeMxpgn(const std::string& sentence) {
  auto body = Body(sentence, "MXPGN");
  if (!body) return std::nullopt;
  auto f = Fields(*body);
  if (f.size() != 4) return std::nullopt;
  uint32_t pgn, attr;
  std::vector<uint8_t> reversed;
  if (!HexValue(f[1], pgn) || !HexValue(f[2], attr) || !HexBytes(f[3], reversed) ||
      reversed.empty() || reversed.size() > 8)
    return std::nullopt;
  Frame fr;
  fr.send = (attr & 0x8000u) != 0;
  fr.priority = static_cast<uint8_t>((attr >> 12) & 7);
  uint8_t address = static_cast<uint8_t>(attr & 0xFF);
  int p = static_cast<int>(pgn);
  uint8_t dest = 255;
  SplitDestination(p, dest);
  fr.pgn = p;
  if (fr.send) {
    fr.destination = address;
  } else {
    fr.source = address;
    fr.destination = dest;
  }
  fr.data.assign(reversed.rbegin(), reversed.rend());
  return fr;
}

std::optional<simnet::Message> Assembler::Add(const Frame& f) {
  simnet::Message m;
  m.pgn = f.pgn;
  m.priority = f.priority;
  m.source = f.source;
  m.destination = f.destination;
  if (!IsFastPacket(f.pgn)) {
    m.data = f.data;
    return m;
  }
  if (f.data.empty()) return std::nullopt;
  uint8_t seq = f.data[0] >> 5;
  uint8_t counter = f.data[0] & 0x1F;
  auto key = std::make_tuple(f.pgn, f.source);
  if (counter == 0) {
    if (f.data.size() < 2) return std::nullopt;
    Partial p;
    p.sequence = seq;
    p.length = f.data[1];
    p.data.assign(f.data.begin() + 2, f.data.end());
    if (p.data.size() >= p.length) {
      m_partial.erase(key);
      p.data.resize(p.length);
      m.data = p.data;
      return m;
    }
    m_partial[key] = p;
    return std::nullopt;
  }
  auto it = m_partial.find(key);
  if (it == m_partial.end()) return std::nullopt;
  Partial& p = it->second;
  if (seq != p.sequence || counter != p.next) {
    m_partial.erase(it);  // a frame got lost: drop the message
    return std::nullopt;
  }
  p.data.insert(p.data.end(), f.data.begin() + 1, f.data.end());
  p.next++;
  if (p.data.size() >= p.length) {
    p.data.resize(p.length);
    m.data = p.data;
    m_partial.erase(it);
    return m;
  }
  return std::nullopt;
}

}  // namespace wrap0183
