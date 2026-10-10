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
// IntegratedNav serial driver: translate placeholder protocol frames into *
// IMC measurements (AHRS / DVL / Depth) for Navigation.AUV EKF.          *
//***************************************************************************

// ISO C++ headers.
#include <cmath>
#include <vector>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "Protocol.hpp"

namespace Sensors
{
  namespace IntegratedNav
  {
    using DUNE_NAMESPACES;

    //! Standard gravity (m/s^2) for pressure-to-depth.
    static const double c_gravity = 9.80665;

    //! Read timeout per message class [s].
    static const double c_default_timeout = 2.0;

    //! %Task arguments.
    struct Arguments
    {
      //! IO device URI (uart://...).
      std::string io_dev;
      //! Entity labels for the four measurement channels.
      std::string elabel_ahrs;
      std::string elabel_dvl;
      std::string elabel_depth;
      std::string elabel_alt;
      //! Mounting rotation matrices (row-major, 3x3).
      std::vector<double> imu_rot;
      std::vector<double> dvl_rot;
      //! Fluid density [kg/m^3].
      double water_density;
      //! Atmospheric pressure [Pa].
      double atm_pressure;
      //! Depth offset [m].
      double depth_offset;
      //! Per-stream input timeout [s].
      double input_timeout;
      //! Timestamp latency compensation [s].
      double latency_compensation;
    };

    //! IntegratedNav device driver (placeholder protocol v0).
    struct Task: public Hardware::BasicDeviceDriver
    {
      //! Task arguments.
      Arguments m_args;
      //! Byte-wise parser.
      Protocol::Parser m_parser;
      //! IO handle.
      IO::Handle* m_io;
      //! Entity identifiers.
      unsigned m_ent_ahrs;
      unsigned m_ent_dvl;
      unsigned m_ent_depth;
      unsigned m_ent_alt;
      //! Outgoing messages.
      IMC::EulerAngles m_euler;
      IMC::AngularVelocity m_angvel;
      IMC::Acceleration m_accel;
      IMC::GroundVelocity m_gvel;
      IMC::Distance m_distance;
      IMC::Depth m_depth;
      IMC::Pressure m_pressure;
      IMC::Temperature m_temperature;
      IMC::DataSanity m_sanity;
      //! Per-stream watchdogs.
      Counter<double> m_wdog_att;
      Counter<double> m_wdog_imu;
      Counter<double> m_wdog_dvl;
      Counter<double> m_wdog_depth;
      //! Frequency accounting.
      unsigned m_cnt_att;
      unsigned m_cnt_imu;
      unsigned m_cnt_dvl;
      unsigned m_cnt_depth;
      double m_stat_time;
      //! Last DVL bottom-lock state (DataSanity edge).
      bool m_last_lock;

      Task(const std::string& name, Tasks::Context& ctx):
        Hardware::BasicDeviceDriver(name, ctx),
        m_io(NULL),
        m_ent_ahrs(0),
        m_ent_dvl(0),
        m_ent_depth(0),
        m_ent_alt(0),
        m_cnt_att(0),
        m_cnt_imu(0),
        m_cnt_dvl(0),
        m_cnt_depth(0),
        m_stat_time(0),
        m_last_lock(true)
      {
        param("IO Port - Device", m_args.io_dev)
        .defaultValue("")
        .description("IO device URI, e.g. \"uart:///tmp/ttyINAV:115200\"");

        param("Entity Label - AHRS", m_args.elabel_ahrs)
        .defaultValue("AHRS")
        .description("Entity label for attitude / IMU messages.");

        param("Entity Label - DVL", m_args.elabel_dvl)
        .defaultValue("DVL")
        .description("Entity label for DVL velocity / sanity.");

        param("Entity Label - Depth", m_args.elabel_depth)
        .defaultValue("Depth Sensor")
        .description("Entity label for depth / pressure.");

        param("Entity Label - Altitude", m_args.elabel_alt)
        .defaultValue("DVL Filtered")
        .description("Entity label for DVL altitude (Distance).");

        param("IMU Rotation Matrix", m_args.imu_rot)
        .defaultValue("1, 0, 0, 0, 1, 0, 0, 0, 1")
        .size(9)
        .description("Row-major 3x3 rotation, IMU/device to body.");

        param("DVL Rotation Matrix", m_args.dvl_rot)
        .defaultValue("1, 0, 0, 0, 1, 0, 0, 0, 1")
        .size(9)
        .description("Row-major 3x3 rotation, DVL/device to body.");

        param("Water Density", m_args.water_density)
        .defaultValue("1025.0")
        .units(Units::KilogramPerCubicMeter)
        .description("Fluid density for pressure-to-depth.");

        param("Atmospheric Pressure", m_args.atm_pressure)
        .defaultValue("101325.0")
        .units(Units::Pascal)
        .description("Atmospheric pressure reference.");

        param("Depth Offset", m_args.depth_offset)
        .defaultValue("0.0")
        .units(Units::Meter)
        .description("Constant depth offset after conversion.");

        param("Input Timeout", m_args.input_timeout)
        .defaultValue("2.0")
        .minimumValue("0.5")
        .units(Units::Second)
        .description("Per-message-class input timeout.");

        param("Latency Compensation", m_args.latency_compensation)
        .defaultValue("0.0")
        .units(Units::Second)
        .description("Subtract from message timestamps (device latency).");
      }

      void
      onUpdateParameters(void)
      {
        BasicDeviceDriver::onUpdateParameters();

        if (m_args.input_timeout < 0.5)
          m_args.input_timeout = c_default_timeout;

        m_wdog_att.setTop(m_args.input_timeout);
        m_wdog_imu.setTop(m_args.input_timeout);
        m_wdog_dvl.setTop(m_args.input_timeout);
        m_wdog_depth.setTop(m_args.input_timeout);
      }

      void
      onEntityReservation(void)
      {
        m_ent_ahrs = reserveEntity(m_args.elabel_ahrs);
        m_ent_dvl = reserveEntity(m_args.elabel_dvl);
        m_ent_depth = reserveEntity(m_args.elabel_depth);
        m_ent_alt = reserveEntity(m_args.elabel_alt);

        m_euler.setSourceEntity(m_ent_ahrs);
        m_angvel.setSourceEntity(m_ent_ahrs);
        m_accel.setSourceEntity(m_ent_ahrs);
        m_gvel.setSourceEntity(m_ent_dvl);
        m_sanity.setSourceEntity(m_ent_dvl);
        m_distance.setSourceEntity(m_ent_alt);
        m_depth.setSourceEntity(m_ent_depth);
        m_pressure.setSourceEntity(m_ent_depth);
        m_temperature.setSourceEntity(m_ent_depth);
      }

      bool
      onConnect(void)
      {
        m_io = openUART(m_args.io_dev);
        return m_io != NULL;
      }

      void
      onDisconnect(void)
      {
        if (m_io != NULL)
        {
          Memory::clear(m_io);
          m_io = NULL;
        }
      }

      void
      onInitializeDevice(void)
      {
        m_parser = Protocol::Parser();
        m_stat_time = Clock::get();
        m_wdog_att.reset();
        m_wdog_imu.reset();
        m_wdog_dvl.reset();
        m_wdog_depth.reset();
        inf("IntegratedNav driver ready on %s", m_args.io_dev.c_str());
      }

      //! Apply row-major 3x3 matrix to a 3-vector.
      void
      rotate(const std::vector<double>& r, double x, double y, double z,
             double& xo, double& yo, double& zo) const
      {
        if (r.size() < 9)
        {
          xo = x;
          yo = y;
          zo = z;
          return;
        }

        xo = r[0] * x + r[1] * y + r[2] * z;
        yo = r[3] * x + r[4] * y + r[5] * z;
        zo = r[6] * x + r[7] * y + r[8] * z;
      }

      //! Normalize angle to (-pi, pi].
      double
      normalizePi(double a) const
      {
        return Angles::normalizeRadian(a);
      }

      void
      handleAtt(const Protocol::AttData& d)
      {
        // Device frame == body frame after mount matrix; identity by default.
        double roll = Angles::radians(d.roll);
        double pitch = Angles::radians(d.pitch);
        double yaw = Angles::radians(d.yaw);
        double cp, sp, cy, sy;
        cp = std::cos(pitch); sp = std::sin(pitch);
        cy = std::cos(yaw); sy = std::sin(yaw);

        // R_dev (ZYX) then optional mount rotation -> body Euler.
        // For the placeholder protocol R_mount is identity unless configured.
        double x, y, z;
        rotate(m_args.imu_rot, cy * cp, sy * cp, -sp, x, y, z);
        // With identity mount, recover ZYX Euler; otherwise best-effort yaw/pitch.
        double yaw_b = std::atan2(y, x);
        double pitch_b = std::atan2(-z, std::sqrt(x * x + y * y));
        double roll_b = roll;
        if (m_args.imu_rot.size() >= 9 &&
            (m_args.imu_rot[0] != 1.0 || m_args.imu_rot[4] != 1.0 || m_args.imu_rot[8] != 1.0))
        {
          // Keep calibrated body roll/pitch from device axes when mounting is non-trivial.
          rotate(m_args.imu_rot, 0, 0, 1, x, y, z);
          roll_b = std::atan2(y, z);
        }

        m_euler.phi = normalizePi(roll_b);
        m_euler.theta = normalizePi(pitch_b);
        m_euler.psi = normalizePi(yaw_b);
        m_euler.psi_magnetic = m_euler.psi;

        sendMeasurement(m_euler);
        m_wdog_att.reset();
        ++m_cnt_att;
      }

      void
      handleImu(const Protocol::ImuData& d)
      {
        double gx = Angles::radians(d.gx);
        double gy = Angles::radians(d.gy);
        double gz = Angles::radians(d.gz);
        double ax = d.ax, ay = d.ay, az = d.az;
        double xo, yo, zo;

        rotate(m_args.imu_rot, gx, gy, gz, xo, yo, zo);
        m_angvel.x = xo;
        m_angvel.y = yo;
        m_angvel.z = zo;
        sendMeasurement(m_angvel);

        rotate(m_args.imu_rot, ax, ay, az, xo, yo, zo);
        m_accel.x = xo;
        m_accel.y = yo;
        m_accel.z = zo;
        sendMeasurement(m_accel);

        m_wdog_imu.reset();
        ++m_cnt_imu;
      }

      void
      handleDvl(const Protocol::DvlData& d)
      {
        bool lock = (d.status & 0x01) != 0;
        double xo, yo, zo;
        rotate(m_args.dvl_rot, d.vx, d.vy, d.vz, xo, yo, zo);

        if (lock)
        {
          m_gvel.x = xo;
          m_gvel.y = yo;
          m_gvel.z = zo;
          m_gvel.validity = IMC::GroundVelocity::VAL_VEL_X |
                            IMC::GroundVelocity::VAL_VEL_Y |
                            IMC::GroundVelocity::VAL_VEL_Z;
          m_distance.value = d.altitude;
          m_distance.validity = IMC::Distance::DV_VALID;
        }
        else
        {
          m_gvel.validity = 0;
          m_distance.validity = IMC::Distance::DV_INVALID;
        }

        sendMeasurement(m_gvel);
        sendMeasurement(m_distance);

        if (lock != m_last_lock)
        {
          m_sanity.sane = lock ? IMC::DataSanity::DS_SANE : IMC::DataSanity::DS_NOT_SANE;
          sendMeasurement(m_sanity);
          m_last_lock = lock;
        }

        m_wdog_dvl.reset();
        ++m_cnt_dvl;
      }

      void
      handleDepth(const Protocol::DepthData& d)
      {
        double p = d.pressure;
        m_pressure.value = p;
        m_temperature.value = d.temperature;
        sendMeasurement(m_pressure);
        sendMeasurement(m_temperature);

        double z = (p - m_args.atm_pressure) / (m_args.water_density * c_gravity);
        m_depth.value = z + m_args.depth_offset;
        sendMeasurement(m_depth);

        m_wdog_depth.reset();
        ++m_cnt_depth;
      }

      //! Dispatch a measurement (optional latency compensation).
      void
      sendMeasurement(IMC::Message& msg)
      {
        if (m_args.latency_compensation > 0.0)
          msg.setTimeStamp(Time::Clock::getSinceEpoch() - m_args.latency_compensation);

        dispatch(msg, DF_KEEP_TIME);
      }

      void
      handleFrame(void)
      {
        switch (m_parser.type())
        {
          case Protocol::MSG_ATT:
            {
              Protocol::AttData d;
              if (m_parser.get(d))
                handleAtt(d);
            }
            break;

          case Protocol::MSG_IMU:
            {
              Protocol::ImuData d;
              if (m_parser.get(d))
                handleImu(d);
            }
            break;

          case Protocol::MSG_DVL:
            {
              Protocol::DvlData d;
              if (m_parser.get(d))
                handleDvl(d);
            }
            break;

          case Protocol::MSG_DEPTH:
            {
              Protocol::DepthData d;
              if (m_parser.get(d))
                handleDepth(d);
            }
            break;

          default:
            break;
        }
      }

      void
      updateWatchdogs(void)
      {
        std::string missing;
        if (m_wdog_att.overflow())
          missing += "ATT ";
        if (m_wdog_imu.overflow())
          missing += "IMU ";
        if (m_wdog_dvl.overflow())
          missing += "DVL ";
        if (m_wdog_depth.overflow())
          missing += "DEPTH ";

        if (!missing.empty())
        {
          std::string msg = String::str("missing: %s", missing.c_str());
          if (m_wdog_att.overflow() && m_wdog_imu.overflow() &&
              m_wdog_dvl.overflow() && m_wdog_depth.overflow())
          {
            throw RestartNeeded(msg, 5.0);
          }

          if (getEntityState() != IMC::EntityState::ESTA_ERROR)
          {
            err("%s", msg.c_str());
            setEntityState(IMC::EntityState::ESTA_ERROR, msg);
          }
        }
      }

      void
      updateStatus(void)
      {
        double now = Clock::get();
        double dt = now - m_stat_time;
        if (dt < 2.0)
          return;

        double hz_att = m_cnt_att / dt;
        double hz_imu = m_cnt_imu / dt;
        double hz_dvl = m_cnt_dvl / dt;
        double hz_depth = m_cnt_depth / dt;
        m_cnt_att = m_cnt_imu = m_cnt_dvl = m_cnt_depth = 0;
        m_stat_time = now;

        std::string desc = String::str(
            "ATT %.1f Hz | IMU %.1f Hz | DVL %.1f Hz | DEPTH %.1f Hz | crc=%u resync=%u unk=%u",
            hz_att, hz_imu, hz_dvl, hz_depth,
            m_parser.crcErrors(), m_parser.resyncs(), m_parser.unknownTypes());

        if (getEntityState() == IMC::EntityState::ESTA_NORMAL ||
            getEntityState() == IMC::EntityState::ESTA_ERROR)
          setEntityState(getEntityState(), desc);
      }

      bool
      onReadData(void)
      {
        if (m_io == NULL)
          return false;

        uint8_t buf[256];
        size_t rv = 0;
        try
        {
          if (!Poll::poll(*m_io, 0.05))
          {
            updateWatchdogs();
            updateStatus();
            return false;
          }

          rv = m_io->read(buf, sizeof(buf));
        }
        catch (std::runtime_error& e)
        {
          throw RestartNeeded(e.what(), 5);
        }

        bool got = false;
        for (size_t i = 0; i < rv; ++i)
        {
          if (m_parser.feed(buf[i]))
          {
            handleFrame();
            got = true;
            // Drain frames already buffered while a read burst is active.
            while (m_parser.next())
              handleFrame();
          }
        }

        // Any leftover complete frame after the last feed().
        while (m_parser.next())
        {
          handleFrame();
          got = true;
        }

        updateWatchdogs();
        updateStatus();
        return got;
      }
    };
  }
}

DUNE_TASK
