/***************************************************************************
 *   navico_autopilot_pi: unit tests for NMEA 2000 wrapped in NMEA 0183
 *   SPDX-License-Identifier: GPL-3.0-or-later
 *
 *   The $MXPGN example is the one quoted from the MiniPlex-3 manual in
 *   OpenCPN's comm_drv_n2k_net.cpp. The 130850 bytes are the AUTO command
 *   recorded from a B&G Triton2 keypad (see test_simnet.cpp).
 ***************************************************************************/

#include <cstdio>
#include <string>
#include <vector>

#include "simnet.h"
#include "wrap0183.h"

using namespace wrap0183;

static int failures = 0;

#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      failures++;                                                  \
    }                                                              \
  } while (0)

static std::vector<uint8_t> Bytes(const std::string& hex) {
  std::vector<uint8_t> out;
  for (size_t i = 0; i + 1 < hex.size();) {
    if (hex[i] == ' ') { i++; continue; }
    out.push_back(static_cast<uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    i += 2;
  }
  return out;
}

static void TestChecksum() {
  // From the MiniPlex-3 manual, as quoted in OpenCPN.
  CHECK(Checksum("$MXPGN,01F119,3816,FFFAAF01A3FDE301*14") == 0x14);
  CHECK(Checksum("$MXPGN,01F201,2816,C50E0A19A0001A40*66") == 0x66);
}

static void TestPcdin() {
  auto auto_cmd = simnet::CommandData(simnet::Command::kAuto, 4);
  std::string s = EncodePcdin(simnet::kPgnCommand, 0, auto_cmd);
  CHECK(s.rfind("$PCDIN,01FF22,00000000,00,419F04FFFF0A09", 0) == 0);
  CHECK(s.size() > 4 && s.substr(s.size() - 2) == "\r\n");
  auto m = DecodePcdin(s);
  CHECK(m && m->pgn == simnet::kPgnCommand && m->source == 0 && m->data == auto_cmd);
  CHECK(m && m->destination == 255);
  // A wrong checksum is refused.
  std::string bad = s;
  bad[bad.size() - 3] = bad[bad.size() - 3] == '0' ? '1' : '0';
  CHECK(!DecodePcdin(bad));
  // Not a PCDIN.
  CHECK(!DecodePcdin("$GPRMC,1,2,3*00"));
  // A PDU1 PGN carries the destination in the low byte.
  auto req = DecodePcdin(EncodePcdin(simnet::kPgnIsoRequest, 0x10, Bytes("00EE00"), 4));
  CHECK(req && req->pgn == simnet::kPgnIsoRequest && req->destination == 4 && req->source == 0x10);
}

static void TestMxpgnDecode() {
  auto f = DecodeMxpgn("$MXPGN,01F119,3816,FFFAAF01A3FDE301*14");
  CHECK(f && f->pgn == 127257 && f->priority == 3 && f->source == 0x16 && !f->send);
  CHECK(f && f->data == Bytes("01 E3 FD A3 01 AF FA FF"));
  CHECK(!DecodeMxpgn("$MXPGN,01F119,3816,FFFAAF01A3FDE301*15"));
}

static void TestMxpgnRoundTrip() {
  // 130850 is more than 8 bytes: a fast packet of two frames.
  auto data = simnet::CommandData(simnet::Command::kAuto, 4);
  auto sentences = EncodeMxpgn(simnet::kPgnCommand, 2, 255, data, 3);
  CHECK(sentences.size() == 2);
  Assembler a;
  std::optional<simnet::Message> got;
  for (const auto& s : sentences) {
    auto f = DecodeMxpgn(s);
    CHECK(f && f->send && f->destination == 255 && f->priority == 2);
    if (!f) continue;
    // Pretend the gateway sent it from address 0x22.
    f->send = false;
    f->source = 0x22;
    got = a.Add(*f);
  }
  CHECK(got && got->pgn == simnet::kPgnCommand && got->source == 0x22 && got->data == data);
  // First frame: sequence 3 in the top bits, then the length (11 bytes).
  auto first = DecodeMxpgn(sentences[0]);
  CHECK(data.size() == 11);
  CHECK(first && first->data[0] == 0x60 && first->data[1] == data.size());

  // A short single-frame PGN goes as one sentence and comes out at once.
  auto iso = simnet::IsoRequestData(simnet::kPgnAddressClaim);
  auto one = EncodeMxpgn(simnet::kPgnIsoRequest, 6, 255, iso, 0);
  CHECK(one.size() == 1);
  auto f = DecodeMxpgn(one[0]);
  CHECK(f && f->pgn == simnet::kPgnIsoRequest && f->data == iso && f->destination == 255);

  // A lost middle frame drops the message instead of returning rubbish.
  std::vector<uint8_t> long_data(20, 0xAB);
  auto three = EncodeMxpgn(simnet::kPgnKeyValue, 3, 255, long_data, 1);
  CHECK(three.size() == 3);
  Assembler b;
  auto f0 = DecodeMxpgn(three[0]);
  auto f2 = DecodeMxpgn(three[2]);
  CHECK(f0 && f2);
  if (f0 && f2) {
    CHECK(!b.Add(*f0));
    CHECK(!b.Add(*f2));
  }
}

// Vectors agreed with the Triton Remote app (wrap0183-afspraken-18nov.md), byte
// for byte the same in both.
static void TestSharedVectors() {
  auto auto_cmd = simnet::CommandData(simnet::Command::kAuto, 4);
  // 4: AUTO via PCDIN.
  CHECK(EncodePcdin(simnet::kPgnCommand, 0, auto_cmd) ==
        "$PCDIN,01FF22,00000000,00,419F04FFFF0A0900FFFFFF*57\r\n");
  // 5: AUTO via MXPGN, priority 2, destination FF, sequence 1.
  auto five = EncodeMxpgn(simnet::kPgnCommand, 2, 255, auto_cmd, 1);
  CHECK(five.size() == 2 && five[0] == "$MXPGN,01FF22,A8FF,0AFFFF049F410B20*67\r\n" &&
        five[1] == "$MXPGN,01FF22,A8FF,FFFFFFFFFF000921*12\r\n");
  // 6 and 7: ISO request for 60928 to 255, priority 6.
  auto iso = simnet::IsoRequestData(simnet::kPgnAddressClaim);
  auto six = EncodeMxpgn(simnet::kPgnIsoRequest, 6, 255, iso, 0);
  CHECK(six.size() == 1 && six[0] == "$MXPGN,00EA00,E3FF,00EE00*12\r\n");
  CHECK(EncodePcdin(simnet::kPgnIsoRequest, 0, iso, 255) == "$PCDIN,00EAFF,00000000,00,00EE00*54\r\n");
  // 8: PCDIN PDU1 without checksum.
  auto eight = DecodePcdin("$PCDIN,00EA04,00000000,10,00EE00");
  CHECK(eight && eight->pgn == simnet::kPgnIsoRequest && eight->destination == 4 &&
        eight->source == 0x10 && eight->data == Bytes("00EE00"));
  // 9 and 10: 130845 read of 1A/0D, steering profile 1, from pilot 4.
  auto kv = simnet::KeyValueRead(4, simnet::SettingKey{0x1A, 0x0D}, 1);
  auto nine = EncodeMxpgn(simnet::kPgnKeyValue, 3, 255, kv, 2);
  CHECK(nine.size() == 3 && nine[0] == "$MXPGN,01FF1D,B8FF,FFFF01049F410E40*60\r\n" &&
        nine[1] == "$MXPGN,01FF1D,B8FF,FFFFFF00000D1A41*6F\r\n" &&
        nine[2] == "$MXPGN,01FF1D,B8FF,FFFFFFFFFFFFFF42*68\r\n");
  CHECK(EncodePcdin(simnet::kPgnKeyValue, 0, kv) ==
        "$PCDIN,01FF1D,00000000,00,419F0401FFFF1A0D0000FFFFFFFF*5F\r\n");
  // 11: ISO request for 126996 to address 7 (PDU1 to one address), source 0.
  auto prod = simnet::IsoRequestData(simnet::kPgnProductInfo);
  CHECK(EncodePcdin(simnet::kPgnIsoRequest, 0, prod, 7) == "$PCDIN,00EA07,00000000,00,14F001*21\r\n");
  auto eleven = DecodePcdin("$PCDIN,00EA07,00000000,00,14F001*21");
  CHECK(eleven && eleven->pgn == simnet::kPgnIsoRequest && eleven->destination == 7 &&
        eleven->source == 0 && eleven->data == Bytes("14F001"));
}

int main() {
  TestSharedVectors();
  TestChecksum();
  TestPcdin();
  TestMxpgnDecode();
  TestMxpgnRoundTrip();
  if (failures) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("all tests passed\n");
  return 0;
}
