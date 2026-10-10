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
// Author: Ricardo Martins                                                  *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

// 使用 DUNE 命名空间宏（IMC、Tasks 等短名）
using DUNE_NAMESPACES;

namespace Simulators
{
  //! Depth sensor simulator for DUNE.
  //! Simulators::DepthSensor is responsible to gather data from the
  //! vehicle simulator by consuming DUNE::IMC::SimulatedState
  //! messages and then add a configurable gaussian noise
  //! component before sending DUNE::IMC::Depth information to the bus.
  //!
  //! 【作用】仿真深度计：
  //!   1) 订阅 SimulatedState（仿真船真实深度）
  //!   2) 加高斯噪声
  //!   3) 发 IMC::Depth 到总线（给导航/控制用）
  //!
  //! @author Ricardo Martins
  namespace DepthSensor
  {
    //! %Task arguments.
    // 配置参数结构体
    struct Arguments
    {
      //! Standard deviation of depth sensor measurements.
      double std_dev;          // 测量噪声标准差（米）
      //! PRNG type.
      std::string prng_type;   // 随机数发生器类型
      //! PRNG seed.
      int prng_seed;           // 随机种子；-1 表示不固定
    };

    //! %DepthSensor simulator task.
    // 周期任务：按频率发布 Depth
    struct Task: public Tasks::Periodic
    {
      //! Last received simulated state.
      IMC::SimulatedState m_sstate;  // 最近一次仿真船状态（含真实 z）
      //! Current depth.
      IMC::Depth m_depth;            // 要发出的深度消息
      //! PRNG handle.
      Random::Generator* m_prng;     // 噪声随机数
      //! Task arguments.
      Arguments m_args;              // 配置

      // 构造：注册参数、绑定消息
      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Periodic(name, ctx),
        m_prng(NULL)                 // 随机数稍后在资源申请时创建
      {
        // Retrieve configuration values.
        // 深度噪声标准差，默认 5 cm
        param("Standard Deviation", m_args.std_dev)
        .units(Units::Meter)
        .defaultValue("0.05");

        // 随机数类型
        param("PRNG Type", m_args.prng_type)
        .defaultValue(Random::Factory::c_default);

        // 随机种子
        param("PRNG Seed", m_args.prng_seed)
        .defaultValue("-1");

        // Register consumers.
        // 收仿真船状态
        bind<IMC::SimulatedState>(this);
      }

      //! Acquire resources. Initializes random number generator.
      // 申请资源：按类型/种子创建 PRNG（伪随机数发生器）
      void
      onResourceAcquisition(void)
      {
        m_prng = Random::Factory::create(m_args.prng_type, m_args.prng_seed);
      }

      //! Release resources.
      // 释放资源：删掉 PRNG
      void
      onResourceRelease(void)
      {
        Memory::clear(m_prng);
      }

      //! Requests activation if not already active and updates #m_sstate with
      //! the value of #msg.
      // 收到 SimulatedState：记录船状态；若未激活则请求激活
      void
      consume(const IMC::SimulatedState* msg)
      {
        // 第一次收到数据时自动激活本任务
        if (!isActive())
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
          requestActivation();
        }

        // 保存最新仿真状态（m_sstate.z 为真实深度）
        m_sstate = *msg;
      }

      //! Dispatches a @publish DUNE::IMC::Depth message with the depth
      //! stored in #m_sstate plus some gaussian noise (according to parametrized
      //! standard deviation). If task is not active, returns immediately.
      // 周期任务主函数：真深度 + 噪声 → IMC::Depth
      void
      task(void)
      {
        // Return if task is not active.
        if (!isActive())
          return;

        //! Compute positive depth value adding a gaussian noise component.
        // z + 高斯噪声×标准差，且深度不小于 0
        //m_depth.value = max( 真实z + 噪声 , 0 )
        m_depth.value = std::max(m_sstate.z + m_prng->gaussian() * m_args.std_dev, 0.0); 
        // Set timestamp and dispatch message to bus.
        // 时间戳用仿真状态的，发到总线时不要覆盖时间
        m_depth.setTimeStamp(m_sstate.getTimeStamp());
        dispatch(m_depth, DF_KEEP_TIME);
      }
    };
  }
}

DUNE_TASK
