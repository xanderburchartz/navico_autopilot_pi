/***************************************************************************
 *   navico_autopilot_pi: Navico (B&G, Simrad) autopilot control for OpenCPN
 *   SPDX-License-Identifier: GPL-3.0-or-later
 ***************************************************************************/

/**
 * \file wrap0183.h
 * NMEA 2000 messages wrapped in NMEA 0183 sentences, for gateways that pass
 * them on to the bus (Yacht Devices YDWG-02/YDNR-02/YDEN-02, ShipModul
 * MiniPlex-3):
 *
 *  - $PCDIN (SeaSmart): one whole message per sentence,
 *    `$PCDIN,<PGN 6 hex>,<time 8 hex>,<source 2 hex>,<data hex>*hh`.
 *    There is no destination field.
 *  - $MXPGN (MiniPlex): one CAN frame per sentence,
 *    `$MXPGN,<PGN 6 hex>,<attributes 4 hex>,<data hex>*hh`. Attributes:
 *    bit 15 send, bits 14-12 priority, bits 11-8 length, bits 7-0 the
 *    destination (sending) or source (received). The data is written as one
 *    number, last byte first. Messages longer than 8 bytes travel as NMEA 2000
 *    fast packets, one frame per sentence.
 *
 * No wxWidgets and no OpenCPN dependencies, so this can be unit tested.
 */
#ifndef NAVICO_AP_WRAP0183_H
#define NAVICO_AP_WRAP0183_H

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "simnet.h"

namespace wrap0183 {

/** "*hh" checksum of a sentence between '$' and '*' (or the end). */
uint8_t Checksum(const std::string& sentence);

/** The whole $PCDIN sentence, with checksum and CR LF. For a PDU1 PGN the
 *  destination goes in the low byte of the PGN field. */
std::string EncodePcdin(int pgn, uint8_t source, const std::vector<uint8_t>& data,
                        uint8_t destination = 255, uint32_t time = 0);

/** A received $PCDIN; nothing if it is not one, or the checksum is wrong. */
std::optional<simnet::Message> DecodePcdin(const std::string& sentence);

/** True for the PGNs this plugin handles that are sent as fast packets. */
bool IsFastPacket(int pgn);

/**
 * The $MXPGN sentences for one message: one for 8 bytes or less (when the
 * PGN is not a fast-packet PGN), else a fast packet with sequence \p sequence
 * (0-7), one sentence per frame.
 */
std::vector<std::string> EncodeMxpgn(int pgn, uint8_t priority, uint8_t destination,
                                     const std::vector<uint8_t>& data, uint8_t sequence);

/** One received $MXPGN frame. */
struct Frame {
  int pgn = 0;
  uint8_t priority = 0;
  uint8_t source = 255;
  uint8_t destination = 255;
  bool send = false;  ///< the send bit: a sentence meant for the bus, not from it
  std::vector<uint8_t> data;
};

/** A received $MXPGN frame; nothing if it is not one, or malformed. */
std::optional<Frame> DecodeMxpgn(const std::string& sentence);

/**
 * Puts $MXPGN frames back together into messages. Single-frame PGNs come out
 * at once; fast packets when their last frame arrived.
 */
class Assembler {
public:
  std::optional<simnet::Message> Add(const Frame& f);

private:
  struct Partial {
    uint8_t sequence = 0;
    size_t length = 0;
    uint8_t next = 1;
    std::vector<uint8_t> data;
  };
  std::map<std::tuple<int, uint8_t>, Partial> m_partial;  // (pgn, source)
};

}  // namespace wrap0183

#endif  // NAVICO_AP_WRAP0183_H
