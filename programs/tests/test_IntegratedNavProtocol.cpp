//***************************************************************************
// Copyright 2007-2026 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Test program for IntegratedNav placeholder protocol.                    *
//***************************************************************************

// ISO C++ headers
#include <cstring>
#include <vector>

// Local headers
#include "Test.hpp"
#include "../src/Sensors/IntegratedNav/Protocol.hpp"

using namespace Sensors::IntegratedNav::Protocol;

//! Feed a byte stream and count completed frames.
static unsigned
feedCount(Parser& p, const std::vector<uint8_t>& bytes)
{
  unsigned n = 0;
  for (unsigned i = 0; i < bytes.size(); ++i)
  {
    if (p.feed(bytes[i]))
      ++n;
  }

  return n;
}

int
main(int argc, char** argv)
{
  (void)argc;
  (void)argv;
  Test test("Sensors::IntegratedNav::Protocol");

  // 1. ATT encode/decode round-trip
  {
    AttData in;
    in.roll = 1.5f;
    in.pitch = -2.25f;
    in.yaw = 123.5f;

    std::vector<uint8_t> raw = Encoder::encode(in);
    Parser p;
    unsigned frames = feedCount(p, raw);
    AttData out;
    bool got = (frames == 1) ? p.get(out) : false;
    test.boolean("ATT round-trip", frames == 1 && got &&
                 out.roll == in.roll && out.pitch == in.pitch && out.yaw == in.yaw);
  }

  // 2-4. IMU / DVL / DEPTH round-trip
  {
    ImuData imu;
    imu.gx = 0.1f; imu.gy = 0.2f; imu.gz = 0.3f;
    imu.ax = 0.0f; imu.ay = 0.0f; imu.az = 9.81f;
    Parser p;
    unsigned frames = feedCount(p, Encoder::encode(imu));
    ImuData imu2;
    test.boolean("IMU round-trip", frames == 1 && p.get(imu2) &&
                 imu2.gz == imu.gz && imu2.az == imu.az);
  }

  {
    DvlData dvl;
    dvl.vx = 1.25f; dvl.vy = -0.5f; dvl.vz = 0.1f;
    dvl.altitude = 8.0f;
    dvl.status = 1;
    Parser p;
    unsigned frames = feedCount(p, Encoder::encode(dvl));
    DvlData dvl2;
    test.boolean("DVL round-trip", frames == 1 && p.get(dvl2) &&
                 dvl2.vx == dvl.vx && dvl2.altitude == dvl.altitude &&
                 dvl2.status == dvl.status);
  }

  {
    DepthData depth;
    depth.pressure = 201325.0f;
    depth.temperature = 15.5f;
    Parser p;
    unsigned frames = feedCount(p, Encoder::encode(depth));
    DepthData depth2;
    test.boolean("DEPTH round-trip", frames == 1 && p.get(depth2) &&
                 depth2.pressure == depth.pressure &&
                 depth2.temperature == depth.temperature);
  }

  // 5. Byte-fragmented feed
  {
    AttData in;
    in.roll = 3.0f; in.pitch = 4.0f; in.yaw = 5.0f;
    std::vector<uint8_t> raw = Encoder::encode(in);
    Parser p;
    unsigned frames = feedCount(p, raw);
    AttData out;
    test.boolean("fragmented byte feed", frames == 1 && p.get(out) && out.yaw == 5.0f);
  }

  // 6. CRC error
  {
    AttData in;
    in.roll = 1.0f; in.pitch = 2.0f; in.yaw = 3.0f;
    std::vector<uint8_t> raw = Encoder::encode(in);
    raw[raw.size() - 1] ^= 0xff;
    Parser p;
    unsigned frames = feedCount(p, raw);
    test.boolean("CRC error rejected", frames == 0 && p.crcErrors() == 1);
  }

  // 7. Garbage then valid frame (resync)
  {
    AttData in;
    in.roll = 0.25f; in.pitch = 0.5f; in.yaw = 0.75f;
    std::vector<uint8_t> raw = Encoder::encode(in);
    std::vector<uint8_t> stream;
    stream.push_back(0x11);
    stream.push_back(0xAA);
    stream.push_back(0x00);
    stream.insert(stream.end(), raw.begin(), raw.end());

    Parser p;
    unsigned frames = feedCount(p, stream);
    AttData out;
    test.boolean("garbage resync", frames == 1 && p.get(out) && out.roll == 0.25f);
  }

  // 8. len overflow triggers resync
  {
    std::vector<uint8_t> bad;
    bad.push_back(0xAA);
    bad.push_back(0x55);
    bad.push_back(0x01);
    bad.push_back(100); // len > 64
    Parser p;
    unsigned frames = feedCount(p, bad);
    test.boolean("len>64 resync", frames == 0 && p.resyncs() == 1);
  }

  // 9. Multiple frames back-to-back
  {
    AttData att;
    att.roll = 1; att.pitch = 2; att.yaw = 3;
    DvlData dvl;
    dvl.vx = 1; dvl.vy = 0; dvl.vz = 0;
    dvl.altitude = 5; dvl.status = 1;
    DepthData depth;
    depth.pressure = 101325; depth.temperature = 20;

    std::vector<uint8_t> stream = Encoder::encode(att);
    std::vector<uint8_t> b = Encoder::encode(dvl);
    std::vector<uint8_t> c = Encoder::encode(depth);
    stream.insert(stream.end(), b.begin(), b.end());
    stream.insert(stream.end(), c.begin(), c.end());

    Parser p;
    unsigned frames = feedCount(p, stream);
    test.boolean("three frames in a row", frames == 3 && p.crcErrors() == 0);
  }

  // 10. CRC-16/CCITT-FALSE known vector ("123456789" -> 0x29B1)
  {
    const uint8_t vec[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint16_t crc = Crc16Ccitt::compute(vec, 9);
    test.boolean("CRC-16/CCITT-FALSE vector", crc == 0x29B1);
  }

  // 11. UART drop 1 byte from f1, then good f2 + f3 -> must decode 2 frames
  {
    AttData att;
    att.roll = 10; att.pitch = 20; att.yaw = 30;
    std::vector<uint8_t> f1 = Encoder::encode(att);
    att.roll = 40; att.pitch = 50; att.yaw = 60;
    std::vector<uint8_t> f2 = Encoder::encode(att);
    att.roll = 70; att.pitch = 80; att.yaw = 90;
    std::vector<uint8_t> f3 = Encoder::encode(att);

    // Drop one payload byte from f1 (UART overrun).
    std::vector<uint8_t> f1_dropped(f1.begin(), f1.begin() + 8);
    f1_dropped.insert(f1_dropped.end(), f1.begin() + 9, f1.end());

    std::vector<uint8_t> stream = f1_dropped;
    stream.insert(stream.end(), f2.begin(), f2.end());
    stream.insert(stream.end(), f3.begin(), f3.end());

    Parser p;
    unsigned frames = feedCount(p, stream);
    test.boolean("drop-byte f1 keeps f2+f3", frames == 2);
  }

  // 12. Mid-stream join: partial header + good f2 + f3 -> 2 frames
  {
    AttData att;
    att.roll = 11; att.pitch = 22; att.yaw = 33;
    std::vector<uint8_t> f2 = Encoder::encode(att);
    att.roll = 44; att.pitch = 55; att.yaw = 66;
    std::vector<uint8_t> f3 = Encoder::encode(att);

    std::vector<uint8_t> stream;
    stream.push_back(0xAA);
    stream.push_back(0x55);
    stream.push_back(0x01);
    stream.push_back(0x0C);
    stream.insert(stream.end(), f2.begin(), f2.end());
    stream.insert(stream.end(), f3.begin(), f3.end());

    Parser p;
    unsigned frames = feedCount(p, stream);
    test.boolean("partial header keeps f2+f3", frames == 2);
  }

  // 13. CRC error mid-stream then two good frames
  {
    AttData att;
    att.roll = 1; att.pitch = 1; att.yaw = 1;
    std::vector<uint8_t> f1 = Encoder::encode(att);
    f1[f1.size() / 2] ^= 0x55; // corrupt payload
    att.roll = 2; att.pitch = 2; att.yaw = 2;
    std::vector<uint8_t> f2 = Encoder::encode(att);
    att.roll = 3; att.pitch = 3; att.yaw = 3;
    std::vector<uint8_t> f3 = Encoder::encode(att);

    std::vector<uint8_t> stream = f1;
    stream.insert(stream.end(), f2.begin(), f2.end());
    stream.insert(stream.end(), f3.begin(), f3.end());

    Parser p;
    unsigned frames = feedCount(p, stream);
    test.boolean("bad CRC keeps f2+f3", frames == 2 && p.crcErrors() >= 1);
  }

  // 14. Unknown type is skipped but does not break the stream
  {
    AttData att;
    att.roll = 9; att.pitch = 8; att.yaw = 7;
    std::vector<uint8_t> good = Encoder::encode(att);
    std::vector<uint8_t> unknown = Encoder::encode(0x7A, good.data() + 4, 4);

    std::vector<uint8_t> stream = unknown;
    stream.insert(stream.end(), good.begin(), good.end());

    Parser p;
    unsigned frames = feedCount(p, stream);
    AttData out;
    test.boolean("unknown type skipped", frames == 1 && p.get(out) && out.roll == 9.0f);
    test.boolean("unknown type counted", p.unknownTypes() == 1);
  }

  // 15. next() drains frames buffered after a feed burst
  {
    AttData att;
    att.roll = 1; att.pitch = 2; att.yaw = 3;
    std::vector<uint8_t> f1 = Encoder::encode(att);
    att.roll = 4; att.pitch = 5; att.yaw = 6;
    std::vector<uint8_t> f2 = Encoder::encode(att);

    std::vector<uint8_t> stream = f1;
    stream.insert(stream.end(), f2.begin(), f2.end());

    Parser p;
    unsigned frames = 0;
    for (unsigned i = 0; i < stream.size(); ++i)
    {
      if (p.feed(stream[i]))
      {
        ++frames;
        while (p.next())
          ++frames;
      }
    }
    test.boolean("next() drains burst", frames == 2);
  }

  // 16. Fixed-seed fuzz: 1% bit flips, 0.5% drops, no false frames
  {
    // Simple LCG for a deterministic seed.
    unsigned lcg = 0xC0FFEEu;
    for (unsigned i = 0; i < 200; ++i)
      lcg = lcg * 1664525u + 1013904223u;

    AttData att;
    att.roll = 1; att.pitch = 2; att.yaw = 3;
    std::vector<uint8_t> clean;
    for (unsigned i = 0; i < 50; ++i)
    {
      std::vector<uint8_t> f = Encoder::encode(att);
      clean.insert(clean.end(), f.begin(), f.end());
    }

    std::vector<uint8_t> dirty;
    for (unsigned i = 0; i < clean.size(); ++i)
    {
      lcg = lcg * 1664525u + 1013904223u;
      double r = (double)((lcg >> 16) & 0xffff) / 65535.0;
      if (r < 0.005)
        continue; // drop
      uint8_t b = clean[i];
      if (r > 0.99)
      {
        lcg = lcg * 1664525u + 1013904223u;
        b ^= (uint8_t)(1u << (lcg & 7));
      }
      dirty.push_back(b);
    }

    Parser p;
    unsigned frames = 0;
    bool false_frame = false;
    for (unsigned i = 0; i < dirty.size(); ++i)
    {
      bool ok = p.feed(dirty[i]);
      while (ok)
      {
        ++frames;
        AttData out;
        if (!(p.get(out) && out.roll == 1.0f))
          false_frame = true;
        ok = p.next();
      }
    }
    test.boolean("fixed-seed fuzz no false frames", frames <= 50 && !false_frame);
  }

  return test.getReturnValue();
}
