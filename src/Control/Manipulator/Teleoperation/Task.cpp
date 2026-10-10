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
// Modified European Union Public Licence - EUPL v1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://www.ec.europa.eu/idabc/eupl.html.                                 *
//***************************************************************************

// =====================================================================
// Control.Manipulator.Teleoperation
// RemoteActions (Grip0/Grip1/Enable/Stop) -> SetServoPosition 0/1.
// Simulation virtual manipulator only; no hardware, no AUV attitude/thrust.
// =====================================================================

// ISO C++ 98 headers.
#include <cmath>
#include <cstdlib>
#include <cerrno>
#include <climits>
#include <cstring>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Manipulator
  {
    namespace Teleoperation
    {
      using DUNE_NAMESPACES;

      static const float c_axis_max = 127.0f;
      static const unsigned c_axis_count = 2;

      struct Arguments
      {
        float max_angle;
        float input_timeout;
        int servo_id[c_axis_count];
        bool trace_cmds;
      };

      struct Task: public Tasks::Task
      {
        Arguments m_args;
        float m_axis[c_axis_count];
        bool m_enabled;
        bool m_stopped;
        double m_last_input;
        IMC::SetServoPosition m_cmd[c_axis_count];

        Task(const std::string& name, Tasks::Context& ctx):
          Tasks::Task(name, ctx),
          m_enabled(false),
          m_stopped(false),
          m_last_input(-1.0)
        {
          for (unsigned i = 0; i < c_axis_count; ++i)
          {
            m_axis[i] = 0.0f;
            m_cmd[i].id = (uint8_t)i;
            m_cmd[i].value = 0.0f;
          }

          param("Maximum Command Angle", m_args.max_angle)
          .defaultValue("30.0")
          .description("Maximum commanded servo angle in DEGREES from a full-scale axis");

          param("Input Timeout", m_args.input_timeout)
          .defaultValue("0.5")
          .units(Units::Second)
          .description("Zero outputs when no valid RemoteActions for this period");

          param("Servo Id 0", m_args.servo_id[0])
          .defaultValue("0")
          .description("SetServoPosition id for Grip0");

          param("Servo Id 1", m_args.servo_id[1])
          .defaultValue("1")
          .description("SetServoPosition id for Grip1");

          param("Trace Commands", m_args.trace_cmds)
          .defaultValue("false")
          .description("Log dispatched SetServoPosition values");

          bind<IMC::RemoteActions>(this);
        }

        void
        onUpdateParameters(void)
        {
          if (m_args.max_angle < 0.0f)
            m_args.max_angle = 0.0f;
          if (m_args.input_timeout < 0.0f)
            m_args.input_timeout = 0.0f;

          for (unsigned i = 0; i < c_axis_count; ++i)
          {
            if (m_args.servo_id[i] < 0)
              m_args.servo_id[i] = 0;
            else if (m_args.servo_id[i] > 255)
              m_args.servo_id[i] = 255;
          }

          m_cmd[0].id = (uint8_t)m_args.servo_id[0];
          m_cmd[1].id = (uint8_t)m_args.servo_id[1];
        }

        void
        onResourceInitialization(void)
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          applySafeOutput(true);
        }

        void
        onResourceRelease(void)
        {
          applySafeOutput(true);
        }

        void
        applySafeOutput(bool clear_enable)
        {
          for (unsigned i = 0; i < c_axis_count; ++i)
          {
            m_axis[i] = 0.0f;
            m_cmd[i].value = 0.0f;
            dispatch(m_cmd[i]);
          }

          if (clear_enable)
          {
            m_enabled = false;
            m_stopped = true;
          }
        }

        void
        parseInteger(TupleList& tuples, const char* key, int& value,
                     bool& valid)
        {
          std::string text = tuples.get(key);
          if (text.empty())
          {
            valid = false;
            value = 0;
            return;
          }

          char* end = NULL;
          errno = 0;
          long parsed = std::strtol(text.c_str(), &end, 10);
          if (errno == ERANGE || end == text.c_str() || *end != '\0' ||
              parsed < INT_MIN || parsed > INT_MAX)
          {
            valid = false;
            value = 0;
            return;
          }

          value = (int)parsed;
        }

        void
        consume(const IMC::RemoteActions* msg)
        {
          spew("RemoteActions: %s", msg->actions.c_str());

          TupleList tuples(msg->actions);
          bool valid = true;
          int stop = 0;
          int enable = 0;
          parseInteger(tuples, "Stop", stop, valid);
          parseInteger(tuples, "Enable", enable, valid);
          m_stopped = (stop != 0);
          m_enabled = (enable != 0) && !m_stopped;

          const char* keys[c_axis_count] = { "Grip0", "Grip1" };
          for (unsigned i = 0; i < c_axis_count; ++i)
          {
            int raw = 0;
            parseInteger(tuples, keys[i], raw, valid);
            if (raw > 127 || raw < -127)
              valid = false;
            float n = (float)raw / c_axis_max;
            if (!m_enabled)
              n = 0.0f;
            m_axis[i] = Math::trimValue(n, -1.0f, 1.0f);
          }

          if (valid)
          {
            m_last_input = Clock::get();
            driveServos();
          }
          else
          {
            war("invalid RemoteActions input - ignored");
          }
        }

        void
        driveServos(void)
        {
          for (unsigned i = 0; i < c_axis_count; ++i)
          {
            float deg = m_axis[i] * m_args.max_angle;
            deg = Math::trimValue(deg, -m_args.max_angle, m_args.max_angle);
            m_cmd[i].value = (float)Angles::radians((fp64_t)deg);
            dispatch(m_cmd[i]);

            if (m_args.trace_cmds)
            {
              spew("servo %u axis=%.3f cmd=%.4f rad (%.2f deg)",
                   (unsigned)m_cmd[i].id, m_axis[i], m_cmd[i].value, (double)deg);
            }
          }
        }

        void
        onMain(void)
        {
          double last_out = -1.0;

          while (!stopping())
          {
            waitForMessages(0.05);

            double now = Clock::get();

            if (m_last_input >= 0.0)
            {
              double elapsed = now - m_last_input;
              if (elapsed > (double)m_args.input_timeout)
              {
                if (m_enabled || m_axis[0] != 0.0f || m_axis[1] != 0.0f)
                {
                  applySafeOutput(true);
                  war("input timeout (%.2f s) - outputs zeroed", (double)m_args.input_timeout);
                }
              }
            }

            if (last_out < 0.0 || (now - last_out) >= 0.1)
            {
              driveServos();
              last_out = now;
            }
          }
        }
      };
    }
  }
}

DUNE_TASK
