//***************************************************************************
// Copyright 2007-2026 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//***************************************************************************

// ISO C++ 98 headers.
#include <cmath>
#include <sstream>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
  namespace Manipulator
  {
    namespace Haptics
    {
      //! Map synthetic / virtual contact events to abstract haptic commands.
      //! Does not emulate WEART SDK internals. Output is IMC::DevDataText
      //! lines of the form HAPTIC|axis=|level=|active=|reason=
      namespace HapticsCmd
      {
      }

      using DUNE_NAMESPACES;

      static const unsigned c_axis_count = 2;
      static const unsigned c_servo_count = 4;

      //! Contact input source.
      enum ContactSource
      {
        SRC_SERVO = 0,
        SRC_SYNTHETIC = 1
      };

      struct Arguments
      {
        float maximum_angle;
        float input_timeout;
        float level_scale;
        bool trace_events;
        bool synthetic;
        float synth_period;
      };

      struct Task: public Tasks::Task
      {
        Arguments m_args;
        IMC::ServoPosition m_position[c_axis_count];
        bool m_seen[c_axis_count];
        bool m_active;
        bool m_stopped;
        double m_last_input;
        double m_last_emit;
        double m_synth_t0;
        std::string m_last_text;
        int m_last_axis;
        int m_last_level;

        Task(const std::string& name, Tasks::Context& ctx):
          Tasks::Task(name, ctx),
          m_active(false),
          m_stopped(false),
          m_last_input(-1.0),
          m_last_emit(-1.0),
          m_synth_t0(-1.0),
          m_last_axis(-1),
          m_last_level(-1)
        {
          param("Maximum Angle", m_args.maximum_angle)
          .defaultValue("45.0")
          .description("Expected virtual servo full-scale in DEGREES");

          param("Input Timeout", m_args.input_timeout)
          .defaultValue("0.5")
          .units(Units::Second)
          .description("Clear haptic output after this time without contact updates");

          param("Level Scale", m_args.level_scale)
          .defaultValue("100.0")
          .description("Map normalized contact magnitude to an integer level");

          param("Trace Events", m_args.trace_events)
          .defaultValue("false")
          .description("Log generated abstract haptic events");

          param("Synthetic Contact", m_args.synthetic)
          .defaultValue("true")
          .description("true=repeatable synthetic contact generator; false=use ServoPosition as virtual contact");

          param("Synthetic Period", m_args.synth_period)
          .defaultValue("2.0")
          .units(Units::Second)
          .description("Period of the synthetic contact sweep (triangle intensity)");

          for (unsigned i = 0; i < c_axis_count; ++i)
          {
            m_position[i].id = (uint8_t)i;
            m_position[i].value = 0.0f;
            m_seen[i] = false;
          }

          bind<IMC::ServoPosition>(this);
        }

        void
        onUpdateParameters(void)
        {
          if (m_args.maximum_angle <= 0.0f)
            m_args.maximum_angle = 45.0f;
          if (m_args.input_timeout < 0.0f)
            m_args.input_timeout = 0.0f;
          if (m_args.level_scale < 1.0f)
            m_args.level_scale = 1.0f;
          if (m_args.synth_period < 0.2f)
            m_args.synth_period = 0.2f;
        }

        void
        onResourceInitialization(void)
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          m_synth_t0 = Clock::get();
          emitEvent(false, "startup");
        }

        void
        onResourceRelease(void)
        {
          emitEvent(false, "shutdown");
        }

        void
        consume(const IMC::ServoPosition* msg)
        {
          if (m_args.synthetic)
            return;
          if (msg->id >= c_servo_count)
            return;
          if (msg->id >= c_axis_count)
            return;
          if (!std::isfinite((double)msg->value))
          {
            emitEvent(false, "invalid-feedback");
            return;
          }

          m_position[msg->id] = *msg;
          m_seen[msg->id] = true;
          m_last_input = Clock::get();
          emitFromServo();
        }

        //! Virtual contact magnitude from |servo angle| / max_angle.
        void
        emitFromServo(void)
        {
          if (!m_seen[0] && !m_seen[1])
            return;

          bool active = false;
          unsigned axis = 0;
          float level = 0.0f;
          float max_rad = (float)Angles::radians((fp64_t)m_args.maximum_angle);

          for (unsigned i = 0; i < c_axis_count; ++i)
          {
            if (!m_seen[i])
              continue;
            float normalized = std::fabs(m_position[i].value) / max_rad;
            normalized = Math::trimValue(normalized, 0.0f, 1.0f);
            if (normalized > level)
            {
              level = normalized;
              axis = i;
            }
          }

          active = level > 0.001f && !m_stopped;
          emitEvent(active, active ? "contact" : "clear", axis, level);
        }

        //! Repeatable synthetic contact: triangle sweep on axis (t/period)%2.
        void
        tickSynthetic(void)
        {
          double now = Clock::get();
          if (m_synth_t0 < 0.0)
            m_synth_t0 = now;

          double t = now - m_synth_t0;
          double period = (double)m_args.synth_period;
          double phase = std::fmod(t / period, 2.0);
          if (phase < 0.0)
            phase += 2.0;

          float level;
          if (phase < 1.0)
            level = (float)phase;
          else
            level = (float)(2.0 - phase);

          unsigned axis = (unsigned)(std::fmod(std::floor(t / period), 2.0));
          if (axis >= c_axis_count)
            axis = 0;

          bool active = level > 0.001f && !m_stopped;
          m_last_input = now;
          emitEvent(active, active ? "synthetic-contact" : "clear", axis, level);
        }

        void
        emitEvent(bool active, const char* reason, unsigned axis = 0,
                  float level = 0.0f)
        {
          double now = Clock::get();
          int level_int = Math::roundToInteger(
            Math::trimValue(level, 0.0f, 1.0f) * m_args.level_scale);

          // 100 ms coalesce only when axis+level+active+reason are unchanged
          // (so intensity/axis changes are never dropped).
          if (m_last_emit >= 0.0 && (now - m_last_emit) < 0.1 &&
              active == m_active && m_last_text == reason &&
              (int)axis == m_last_axis && level_int == m_last_level)
            return;
          std::ostringstream os;
          os << "HAPTIC|axis=" << axis
             << "|level=" << level_int
             << "|active=" << (active ? 1 : 0)
             << "|reason=" << reason;

          IMC::DevDataText msg;
          msg.value = os.str();
          dispatch(msg);
          m_active = active;
          m_last_emit = now;
          m_last_text = reason;
          m_last_axis = (int)axis;
          m_last_level = level_int;

          if (m_args.trace_events)
            spew("%s", msg.value.c_str());
        }

        void
        onMain(void)
        {
          double last_tick = -1.0;

          while (!stopping())
          {
            waitForMessages(0.05);
            double now = Clock::get();

            if (m_args.synthetic)
            {
              if (last_tick < 0.0 || (now - last_tick) >= 0.1)
              {
                tickSynthetic();
                last_tick = now;
              }
            }

            if (m_last_input >= 0.0)
            {
              double elapsed = now - m_last_input;
              if (elapsed > (double)m_args.input_timeout && m_active)
              {
                m_active = false;
                emitEvent(false, "feedback-timeout");
              }
            }
          }
        }
      };
    }
  }
}

DUNE_TASK
