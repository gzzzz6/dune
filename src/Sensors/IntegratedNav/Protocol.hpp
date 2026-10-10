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

#ifndef SENSORS_INTEGRATEDNAV_PROTOCOL_HPP_INCLUDED_
#define SENSORS_INTEGRATEDNAV_PROTOCOL_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <stdint.h>
#include <string.h>
#include <vector>

namespace Sensors
{
  namespace IntegratedNav
  {
    //! Wire protocol v0 (placeholder; replace when the real manual arrives).
    //! Frame: 0xAA 0x55 | type u8 | len u8 | payload[len] | crc u16 LE
    //! CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type..payload.
    //!
    //! DUNE Algorithms::CRC16 is CRC-16-IBM (poly 0x8005), so it cannot be
    //! reused here; CCITT-FALSE is implemented below instead.
    //!
    //! Nested under Sensors::IntegratedNav::Protocol so that drivers written
    //! in namespace Sensors::IntegratedNav can say Protocol::Parser without
    //! name-lookup clashes.
    namespace Protocol
    {
      //! Frame sync bytes.
      static const uint8_t c_sync0 = 0xAA;
      static const uint8_t c_sync1 = 0x55;

      //! Maximum payload length (larger frames are treated as desync).
      static const unsigned c_max_payload = 64;

      //! Max candidate frame size: sync(2)+type+len+payload+crc(2).
      static const unsigned c_max_frame = 2 + 2 + c_max_payload + 2;

      //! Message type identifiers.
      enum MessageType
      {
        MSG_ATT = 0x01,
        MSG_IMU = 0x02,
        MSG_DVL = 0x03,
        MSG_DEPTH = 0x04
      };

      //! Attitude sample (degrees on the wire).
      struct AttData
      {
        float roll;
        float pitch;
        float yaw;
      };

      //! IMU sample (gyro deg/s, accel m/s^2).
      struct ImuData
      {
        float gx;
        float gy;
        float gz;
        float ax;
        float ay;
        float az;
      };

      //! DVL sample (body velocities m/s, altitude m, status bit0 = bottom lock).
      struct DvlData
      {
        float vx;
        float vy;
        float vz;
        float altitude;
        uint8_t status;
      };

      //! Depth/pressure sample (absolute pressure Pa, temperature C).
      struct DepthData
      {
        float pressure;
        float temperature;
      };

      //! CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, xorout 0.
      class Crc16Ccitt
      {
      public:
        static inline uint16_t
        compute(const uint8_t* data, unsigned len, uint16_t crc = 0xFFFF)
        {
          while (len--)
          {
            crc ^= (uint16_t)(*data++) << 8;
            for (unsigned i = 0; i < 8; ++i)
            {
              if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
              else
                crc = (uint16_t)(crc << 1);
            }
          }

          return crc;
        }
      };

      //! Little-endian primitive writers.
      //! putF32/readF32 assume IEEE-754 little-endian hosts (x86/ARM).
      inline void
      putU8(std::vector<uint8_t>& out, uint8_t v)
      {
        out.push_back(v);
      }

      inline void
      putU16(std::vector<uint8_t>& out, uint16_t v)
      {
        out.push_back((uint8_t)(v & 0xff));
        out.push_back((uint8_t)((v >> 8) & 0xff));
      }

      inline void
      putF32(std::vector<uint8_t>& out, float v)
      {
        uint8_t raw[4];
        memcpy(raw, &v, 4);
        for (unsigned i = 0; i < 4; ++i)
          out.push_back(raw[i]);
      }

      //! Frame encoder (device simulator and tests).
      class Encoder
      {
      public:
        static std::vector<uint8_t>
        encode(uint8_t type, const uint8_t* payload, uint8_t len)
        {
          std::vector<uint8_t> out;
          out.reserve(6 + len);
          putU8(out, c_sync0);
          putU8(out, c_sync1);
          putU8(out, type);
          putU8(out, len);

          for (uint8_t i = 0; i < len; ++i)
            out.push_back(payload[i]);

          uint16_t crc = Crc16Ccitt::compute(&out[2], 2 + len);
          putU16(out, crc);
          return out;
        }

        static std::vector<uint8_t>
        encode(const AttData& d)
        {
          std::vector<uint8_t> p;
          putF32(p, d.roll);
          putF32(p, d.pitch);
          putF32(p, d.yaw);
          return encode(MSG_ATT, &p[0], (uint8_t)p.size());
        }

        static std::vector<uint8_t>
        encode(const ImuData& d)
        {
          std::vector<uint8_t> p;
          putF32(p, d.gx);
          putF32(p, d.gy);
          putF32(p, d.gz);
          putF32(p, d.ax);
          putF32(p, d.ay);
          putF32(p, d.az);
          return encode(MSG_IMU, &p[0], (uint8_t)p.size());
        }

        static std::vector<uint8_t>
        encode(const DvlData& d)
        {
          std::vector<uint8_t> p;
          putF32(p, d.vx);
          putF32(p, d.vy);
          putF32(p, d.vz);
          putF32(p, d.altitude);
          putU8(p, d.status);
          return encode(MSG_DVL, &p[0], (uint8_t)p.size());
        }

        static std::vector<uint8_t>
        encode(const DepthData& d)
        {
          std::vector<uint8_t> p;
          putF32(p, d.pressure);
          putF32(p, d.temperature);
          return encode(MSG_DEPTH, &p[0], (uint8_t)p.size());
        }
      };

      //! Byte-wise frame parser.
      //! On CRC/length failure the candidate bytes are re-scanned from the
      //! second byte so a following good frame is not swallowed.
      //!
      //! get() is valid only immediately after feed() returns true (and only
      //! until the next feed() call).
      class Parser
      {
      public:
        Parser(void):
          m_state(ST_SYNC0),
          m_type(0),
          m_len(0),
          m_idx(0),
          m_crc(0),
          m_have(false),
          m_crc_errors(0),
          m_resyncs(0),
          m_unknown(0)
        {
          memset(m_payload, 0, sizeof(m_payload));
        }

        //! Feed one byte. Returns true when a complete valid frame is ready.
        bool
        feed(uint8_t b)
        {
          // Last frame is only valid until the next feed().
          m_have = false;

          // FIFO so a resync (loop, not recursion) can push bytes back.
          m_pending.push_back(b);

          while (!m_pending.empty())
          {
            uint8_t x = m_pending[0];
            m_pending.erase(m_pending.begin());

            StepResult r = step(x);
            if (r == STEP_FRAME)
            {
              m_have = true;
              return true;
            }

            if (r == STEP_RESYNC)
              resync();
          }

          m_have = false;
          return false;
        }

        uint8_t
        type(void) const
        {
          return m_type;
        }

        unsigned
        crcErrors(void) const
        {
          return m_crc_errors;
        }

        unsigned
        resyncs(void) const
        {
          return m_resyncs;
        }

        unsigned
        unknownTypes(void) const
        {
          return m_unknown;
        }

        //! Pop the next already-buffered valid frame (no new byte).
        //! Call after a read burst: while (parser.next()) { ... get() ... }
        bool
        next(void)
        {
          m_have = false;

          while (!m_pending.empty())
          {
            uint8_t x = m_pending[0];
            m_pending.erase(m_pending.begin());

            StepResult r = step(x);
            if (r == STEP_FRAME)
            {
              m_have = true;
              return true;
            }

            if (r == STEP_RESYNC)
              resync();
          }

          return false;
        }

        bool
        get(AttData& out) const
        {
          if (!m_have || m_type != MSG_ATT || m_len != 12)
            return false;
          readF32(0, out.roll);
          readF32(4, out.pitch);
          readF32(8, out.yaw);
          return true;
        }

        bool
        get(ImuData& out) const
        {
          if (!m_have || m_type != MSG_IMU || m_len != 24)
            return false;
          readF32(0, out.gx);
          readF32(4, out.gy);
          readF32(8, out.gz);
          readF32(12, out.ax);
          readF32(16, out.ay);
          readF32(20, out.az);
          return true;
        }

        bool
        get(DvlData& out) const
        {
          if (!m_have || m_type != MSG_DVL || m_len != 17)
            return false;
          readF32(0, out.vx);
          readF32(4, out.vy);
          readF32(8, out.vz);
          readF32(12, out.altitude);
          out.status = m_payload[16];
          return true;
        }

        bool
        get(DepthData& out) const
        {
          if (!m_have || m_type != MSG_DEPTH || m_len != 8)
            return false;
          readF32(0, out.pressure);
          readF32(4, out.temperature);
          return true;
        }

      private:
        enum State
        {
          ST_SYNC0,
          ST_SYNC1,
          ST_TYPE,
          ST_LEN,
          ST_PAYLOAD,
          ST_CRC_LO,
          ST_CRC_HI
        };

        enum StepResult
        {
          STEP_NEED,
          STEP_FRAME,
          STEP_RESYNC
        };

        //! Process one byte against the state machine.
        StepResult
        step(uint8_t b)
        {
          // Keep raw candidate (max one frame) for failure rescan.
          if (m_state != ST_SYNC0 || b == c_sync0)
          {
            if (m_raw.size() < c_max_frame)
              m_raw.push_back(b);
          }

          switch (m_state)
          {
            case ST_SYNC0:
              if (b == c_sync0)
                m_state = ST_SYNC1;
              break;

            case ST_SYNC1:
              if (b == c_sync1)
                m_state = ST_TYPE;
              else if (b == c_sync0)
                m_state = ST_SYNC1;
              else
              {
                m_state = ST_SYNC0;
                m_raw.clear();
              }
              break;

            case ST_TYPE:
              m_type = b;
              m_state = ST_LEN;
              break;

            case ST_LEN:
              if (b > c_max_payload)
                return STEP_RESYNC;
              m_len = b;
              m_idx = 0;
              m_state = (m_len == 0) ? ST_CRC_LO : ST_PAYLOAD;
              break;

            case ST_PAYLOAD:
              m_payload[m_idx++] = b;
              if (m_idx >= m_len)
                m_state = ST_CRC_LO;
              break;

            case ST_CRC_LO:
              m_crc = b;
              m_state = ST_CRC_HI;
              break;

            case ST_CRC_HI:
              m_crc |= (uint16_t)b << 8;
              {
                uint8_t hdr[2];
                hdr[0] = m_type;
                hdr[1] = m_len;
                uint16_t calc = Crc16Ccitt::compute(hdr, 2);
                if (m_len)
                  calc = Crc16Ccitt::compute(m_payload, m_len, calc);

                if (calc != m_crc)
                {
                  ++m_crc_errors;
                  return STEP_RESYNC;
                }

                m_state = ST_SYNC0;
                m_raw.clear();

                if (m_type == MSG_ATT || m_type == MSG_IMU ||
                    m_type == MSG_DVL || m_type == MSG_DEPTH)
                  return STEP_FRAME;

                ++m_unknown;
              }
              break;

            default:
              m_state = ST_SYNC0;
              break;
          }

          return STEP_NEED;
        }

        //! Drop first candidate byte and replay the rest (loop, not recursion).
        void
        resync(void)
        {
          ++m_resyncs;
          m_state = ST_SYNC0;

          if (m_raw.empty())
            return;

          std::vector<uint8_t> replay(m_raw.begin() + 1, m_raw.end());
          m_raw.clear();

          // Prepend replayed bytes so they are consumed before the rest.
          m_pending.insert(m_pending.begin(), replay.begin(), replay.end());
        }

        void
        readF32(unsigned off, float& out) const
        {
          memcpy(&out, &m_payload[off], 4);
        }

        State m_state;
        uint8_t m_type;
        uint8_t m_len;
        uint8_t m_idx;
        uint8_t m_payload[c_max_payload];
        uint16_t m_crc;
        bool m_have;
        unsigned m_crc_errors;
        unsigned m_resyncs;
        unsigned m_unknown;
        std::vector<uint8_t> m_raw;
        std::vector<uint8_t> m_pending;
      };
    }
  }
}

#endif
