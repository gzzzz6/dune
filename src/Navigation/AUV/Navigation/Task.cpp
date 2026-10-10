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
// Author: Aníbal Matos (original LBL filter circa 1999)                    *
// Author: Ricardo Martins (enhancements)                                   *
// Author: Pedro Vaz Teixeira (state machine)                               *
// Author: Jorge Estrela da Silva, (port and theory enhancements)           *
// Author: José Braga (new Extended Kalman Filter)                          *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

namespace Navigation
{
  namespace AUV
  {  //! Second Generation Navigation
    namespace Navigation
    {  //! Navigation states.
      enum StateIndexes
      {  //! Vehicle north (m).北向位置 (m)
        STATE_X = 0,  //! Vehicle east (m).东向位置 (m)
        STATE_Y = 1,  //! Heading Angle (rad).船头朝哪，艏向（psi）以正北为 0，顺时针增大
        STATE_PSI = 2,  //! Heading Rate (rad/s)转头多快
        STATE_R = 3,  //! Ground Velocity x (m/s).相对海底/大地，沿船头方向往前的速度
        STATE_U = 4,  //! Ground Velocity y (m/s).相对大地，垂直于船头方向的侧向速度
        STATE_V = 5,  //! Heading Rate Bias (rad/s).角速度零偏
        STATE_R_BIAS = 6,  //! Heading Angle Bias (rad).艏向零偏
        STATE_PSI_BIAS = 7,  //! Revolutions to speed factor.螺旋桨每转一圈，大约能推进多少米
        STATE_K = 8,  //! Number of States.
        NUM_STATE = 9
      };

      //! Navigation Output states.
      //! EKF 观测向量 z 的分量索引。
      //!
      //! 【关键概念】EKF 有两套索引，别混：
      //!   STATE_*  = 状态向量 x 的分量（我要估计的东西，9 个）
      //!   OUT_*    = 观测向量 z 的分量（传感器测到的东西，6 个）
      //! 一份观测可以同时修正多个状态分量（见 resetKalman 里 OUT_PSI
      //! 同时修正 STATE_PSI 和 STATE_PSI_BIAS）。
      enum OutputIndexes
      {  //! DVL Forward Speed (m/s).
        OUT_U = 0,  //! DVL Transversal Speed (m/s).
        OUT_V = 1,  //! Heading Angle (rad).
        OUT_PSI = 2,  //! Heading Rate (rad/s).
        OUT_R = 3,  //! GPS x (m).
        OUT_GPS_X = 4,  //! GPS y (m).
        OUT_GPS_Y = 5,  //! Number of Outputs (without LBL).
        NUM_OUT = 6
      };

      //! Process Noise parameters.
      enum ProcessNoiseIndexes
      {
        PN_POSITION = 0,
        PN_PSI = 1,
        PN_YAWRATE = 2,
        PN_SPEED = 3,
        PN_YRATE_BIAS = 4,
        PN_PSI_BIAS = 5
      };

      //! Measure Noise parameters.测量噪声参数
      enum MeasureNoiseIndexes
      {
        MN_U = 0,
        MN_V = 1,
        MN_PSI = 2,
        MN_YAWRATE = 3,
        MN_LBL = 4
      };

      //! State Covariance parameters.状态协方差参数
      enum StateCovarianceIndexes
      {
        SC_POSITION = 0,
        SC_ANGLES = 1,
        SC_SPEED = 2,
        SC_BIASES = 3,
        SC_RPM = 4
      };

      //! GPS accuracy indexes.GPS 精度指标
      enum GpsAccuracyIndexes
      {
        GPS_BAD_IX = 0,
        GPS_AVG_IX = 1,
        GPS_GOOD_IX = 2,
        GPS_PURE_IX = 3
      };

      //! GPS Horizontal Accuracy (HACC) threshold values.GPS 水平精度（HACC）阈值
      enum GpsAccuracyValues
      {
        GPS_BAD = 10,
        GPS_AVG = 8,
        GPS_GOOD = 4
      };

      //! %Task arguments.任务参数
      struct Arguments
      {  //! Position process noise when tactical grade IMU is enabled.
        float pos_noise;  //! LBL measurement noise when tactical grade IMU enabled.
        float lbl_noise;  //! GPS measurement noise covariance values.
        std::vector<double> gps_noise;  //! USBL measurement noise covariance value.
        double usbl_noise;  //! IMU entity label.
        std::string elabel_imu;  //! Number of samples to average forward speed.
        unsigned navg_speed;  //! Revolutions to speed factor.
        float rpm_ini;  //! Maximum revolutions to speed variation.
        float rpm_max;  //! Heading bias uncertainty alignment threshold.
        double alignment_index;  //! Heading alignment sensor diff threshold
        double alignment_diff;  //! Diff threshold - buffer of values for threshold validation
        double heading_buffer_value;  //! Abort if navigation exceeds maximum uncertainty.
        bool abort;  //! Activate RPM to m/s estimation
        bool rpm_estimation;  //!  Activate RPM to m/s % limit on estimation
        bool speed_relation_Limit;  //!  Value RPM to m/s limit on estimation
        double speed_relation_limit_value;  //!  Distance of Depth sensor to the veicle pitch rotation axis
        float distance_depth_sensor;

      };

      struct Task: public DUNE::Navigation::BasicNavigation
      {  //! Periodic GPS fix reading check.周期性GPS定位数据读取检查
        bool m_gps_reading;  //! USBL fix reading check.USBL定位数据读取检查
        bool m_usbl_reading;  //! Moving average for vehicle forward speed.载体前向航速的移动平均计算器
        MovingAverage<double>* m_avg_speed;  //! Task arguments.任务参数
        Arguments m_args;  //! Heading alignment buffer航向对准缓存计数
        int m_heading_buffer;  //! Pointer to speed model for speed conversions用于航速换算的速度模型指针
        const Plans::SpeedModel* m_speed_model;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Navigation::BasicNavigation(name, ctx),
          m_avg_speed(nullptr),
          m_heading_buffer(0),
          m_speed_model(nullptr)
        {  // Declare configuration parameters.
          // 有 IMU 时的位置过程噪声：位置模型误差（越小越信推算）
          param("Position Noise Covariance with IMU", m_args.pos_noise)
          .defaultValue("0.0")
          .minimumValue("0.0")
          .description("Position process noise covariance value when IMU is available");  // 有 IMU 时的 LBL 量测噪声：水声基线定位可信度（越大越少信 LBL）
          param("LBL Noise Covariance with IMU", m_args.lbl_noise)
          .defaultValue("0.1")
          .minimumValue("0.0")
          .description("LBL measurement noise covariance value when IMU is available");

          // EKF 过程噪声 Q（6 项）：位置/艏向/角速度/速度/零偏等模型不确定度
          param("Process Noise Covariance", m_process_noise)
          .defaultValue("")
          .size(6)
          .description("Kalman Filter process noise covariance values");

          // GPS 量测噪声，分 4 档（按 HACC 质量切换，越好噪声越小）
          param("GPS Noise Covariance", m_args.gps_noise)
          .defaultValue("")
          .size(4)
          .description("Kalman Filter GPS noise covariance values");

          // USBL 量测噪声：水声定位可信度（通常大于好 GPS）
          param("USBL Noise Covariance", m_args.usbl_noise)
          .defaultValue("100.0")
          .minimumValue("0.0")
          .description("Kalman Filter USBL noise covariance value");

          // EKF 量测噪声 R（5 项）：U/V/艏向/角速度/LBL 等传感器噪声
          param("Measure Noise Covariance", m_measure_noise)
          .defaultValue("")
          .size(5)
          .description("Kalman Filter measurement noise covariance values");

          // 状态协方差 P 初值（5 项）：开局对位置/角度/速度/零偏/K 的不确定度
          param("State Covariance Initial State", m_state_cov)
          .defaultValue("")
          .size(5)
          .description("Kalman Filter State Covariance initial values");

          // 前向速度滑动平均点数：平滑 EstimatedState.u，减少抖动
          param("Speed Moving Average Samples", m_args.navg_speed)
          .defaultValue("10")
          .minimumValue("5")
          .maximumValue("20")
          .description("Number of moving average samples to smooth forward speed");

          // 转速→速度系数 K 初值：u≈RPM×K（无 DVL/GPS 时估船速）
          param("Revolutions to speed factor", m_args.rpm_ini)
          .defaultValue("1.2e-3")
          .minimumValue("0.8e-3")
          .maximumValue("2.0e-3")
          .description("Kalman Filter initial revolutions to speed factor value");

          // K 允许变化幅度：限制在 rpm_ini±rpm_max，防止滤波跑飞
          param("Revolutions to speed variation", m_args.rpm_max)
          .defaultValue("6e-4")
          .minimumValue("4e-4")
          .maximumValue("8e-4")
          .description("Kalman Filter revolutions to speed maximum allowed variation");

          // 位置不确定度超限时是否中止/报警
          param("Abort if Uncertainty Exceeded", m_args.abort)
          .defaultValue("true")
          .description("Abort if position uncertainty is exceeded");

          // 航向零偏对齐阈值：PSI_BIAS 协方差小于该值视为零偏已收敛
          param("Heading Bias Alignment Index", m_args.alignment_index)
          .defaultValue("1e-5")
          .minimumValue("1e-6")
          .maximumValue("1e-4")
          .description("Heading bias uncertainty alignment threshold");

          // 对齐允许的航向差：滤波艏向与 AHRS/IMU 艏向之差上限（度）
          param("Heading alignment sensor diff", m_args.alignment_diff)
          .defaultValue("15")
          .minimumValue("1")
          .maximumValue("180")
          .description("Heading alignment sensor diff threshold");

          // 失对齐连续计数上限：超过则可自动关 IMU 等
          param("Heading buffer value", m_args.heading_buffer_value)
          .defaultValue("200")
          .minimumValue("1")
          .description("Heading buffer value - how many repetitions to aligned");

          // IMU 实体标签（IMC 实体名）：用于解析实体 ID、判断 IMU 是否激活
          param("Entity Label - IMU", m_args.elabel_imu)
          .description("Entity label of the IMU");

          // 深度传感器到俯仰转轴的水平距离：用俯仰角修正深度读数
          param("Depth sensor localization in x axis", m_args.distance_depth_sensor)
          .defaultValue("0.0")
          .minimumValue("0.0")
          .maximumValue("2.0")
          .description("Depth sensor localization in x axis in meters- used for depth correction due to pitch");

          // 是否用转速×K 估前向速度（无测速手段时）
          param("Rpm to speed estimation", m_args.rpm_estimation)
          .defaultValue("true")
          .description("");

          // 是否限制 RPM 估速：与速度模型偏差过大时丢弃 RPM 估计
          param("Activate speed to rpm estimation limit", m_args.speed_relation_Limit)
          .defaultValue("false")
          .description("");

          // RPM 估速相对速度模型的最大允许偏差百分比
          param("speed to rpm estimation percentage limit", m_args.speed_relation_limit_value)
          .defaultValue("15")
          .minimumValue("1")
          .maximumValue("100")
          .description("speed to rpm maximum diference between estimation and speed model");

          // Extended Kalman Filter initialization.扩展卡尔曼滤波器初始化
          m_kal.reset(NUM_STATE, NUM_OUT);
          resetKalman();

          // Register callbacks
          bind<IMC::EntityActivationState>(this);
        }

        void
        onUpdateParameters(void) override
        {
          BasicNavigation::onUpdateParameters();

          // Initialize Process and Measure Covariances matrices
          m_kal.setProcessNoise(STATE_X, m_process_noise[PN_POSITION]);
          m_kal.setProcessNoise(STATE_Y, m_process_noise[PN_POSITION]);
          m_kal.setProcessNoise(STATE_PSI, m_process_noise[PN_PSI]);
          m_kal.setProcessNoise(STATE_R, m_process_noise[PN_YAWRATE]);
          m_kal.setProcessNoise(STATE_U, m_process_noise[PN_SPEED]);
          m_kal.setProcessNoise(STATE_V, m_process_noise[PN_SPEED]);
          m_kal.setProcessNoise(STATE_R_BIAS, m_process_noise[PN_YRATE_BIAS]);
          m_kal.setProcessNoise(STATE_PSI_BIAS, 0.0);

          m_kal.setMeasurementNoise(OUT_U, m_measure_noise[MN_U]);
          m_kal.setMeasurementNoise(OUT_V, m_measure_noise[MN_V]);
          m_kal.setMeasurementNoise(OUT_PSI, m_measure_noise[MN_PSI]);
          m_kal.setMeasurementNoise(OUT_R, m_measure_noise[MN_YAWRATE]);
          m_kal.setMeasurementNoise(OUT_GPS_X, m_args.gps_noise[GPS_GOOD_IX]);
          m_kal.setMeasurementNoise(OUT_GPS_Y, m_args.gps_noise[GPS_GOOD_IX]);

          for (unsigned i = 0; i < m_ranging.getSize(); i++)
            m_kal.setMeasurementNoise(NUM_OUT + i, m_measure_noise[MN_LBL]);

          if (m_dead_reckoning)
          {  // Position process noise covariance value if IMU is available.
            m_kal.setProcessNoise(STATE_X, m_args.pos_noise);
            m_kal.setProcessNoise(STATE_Y, m_args.pos_noise);
            m_kal.setProcessNoise(STATE_PSI_BIAS, m_process_noise[PN_PSI_BIAS]);

            // LBL noise.
            for (unsigned i = 0; i < m_ranging.getSize(); i++)
              m_kal.setMeasurementNoise(NUM_OUT + i, m_args.lbl_noise);
          }
        }  //资源初始化回调函数
        void
        onResourceInitialization(void) override
        {
          BasicNavigation::onResourceInitialization();
          m_avg_speed = new MovingAverage<double>(m_args.navg_speed);
          startSpeedModel(&m_ctx.config);
        }  //资源释放回调函数
        void
        onResourceRelease(void) override
        {
          BasicNavigation::onResourceRelease();
          Memory::clear(m_avg_speed);
          Memory::clear(m_speed_model);
        }  //实体解析回调函数
        void
        onEntityResolution(void) override
        {
          BasicNavigation::onEntityResolution();
          try
          {
            m_imu_eid = resolveEntity(m_args.elabel_imu);
          }
          catch (...)
          {
            m_imu_eid = std::numeric_limits<unsigned>::max();
          }
        }  //上报实体状态回调函数
        void
        onReportEntityState(void)
        {
          IMC::AlignmentState as;

          as.state = m_aligned ? IMC::AlignmentState::AS_ALIGNED : IMC::AlignmentState::AS_NOT_ALIGNED;

          // No IMU unit available.
          if (m_imu_eid == std::numeric_limits<unsigned>::max())
            as.state = IMC::AlignmentState::AS_NOT_SUPPORTED;

          dispatch(as);
        }  //它在 IMU 激活/关闭时，切换航位推算模式，并改写航向零偏、位置/零偏过程噪声和 LBL 量测噪声等 EKF 配置
        void
        consume(const IMC::EntityActivationState* msg)
        {
          if (msg->getSource() != getSystemId())
            return;

          if (msg->getSourceEntity() != m_imu_eid)
            return;

          if ((msg->state == IMC::EntityActivationState::EAS_ACTIVE ||
               msg->state == IMC::EntityActivationState::EAS_ACT_DONE))
          {  // IMU already activated.
            if (m_dead_reckoning)
              return;

            // Dead reckoning mode.
            m_dead_reckoning = true;
            debug("start navigation alignment");//启动导航对准

            // Reinitialize state covariance matrix value.
            m_kal.resetCovariance(STATE_PSI_BIAS);
            m_kal.setCovariance(STATE_PSI_BIAS, m_state_cov[SC_BIASES]);

            // Position process noise covariance value if IMU is available.
            m_kal.setProcessNoise(STATE_X, m_args.pos_noise);
            m_kal.setProcessNoise(STATE_Y, m_args.pos_noise);
            m_kal.setProcessNoise(STATE_PSI_BIAS, m_process_noise[PN_PSI_BIAS]);

            // LBL noise.
            for (unsigned i = 0; i < m_ranging.getSize(); i++)
              m_kal.setMeasurementNoise(NUM_OUT + i, m_args.lbl_noise);
          }
          else  // DVL 或 GPS 至少一个正常：用速度 u,v 推算位置
          {
            if (!m_dead_reckoning)
              return;

            // Stop integrate heading rates and use AHRS data.
            m_dead_reckoning = false;
            m_aligned = false;
            debug("navigation not aligned");

            m_kal.setState(STATE_PSI_BIAS, 0.0);

            // No heading offset estimation without IMU.
            m_kal.resetCovariance(STATE_PSI_BIAS);

            // Reinitialize EKF variances.
            m_kal.setProcessNoise(STATE_X, m_process_noise[PN_POSITION]);
            m_kal.setProcessNoise(STATE_Y, m_process_noise[PN_POSITION]);
            m_kal.setProcessNoise(STATE_PSI_BIAS, 0.0);

            for (unsigned i = 0; i < m_ranging.getSize(); i++)
              m_kal.setMeasurementNoise(NUM_OUT + i, m_measure_noise[MN_LBL]);
          }
        }  //EKF 启动时的初始化：设定状态初值和协方差 P 的对角初值
        bool
        setup(void)
        {
          BasicNavigation::setup();

          m_kal.setState(STATE_K, m_args.rpm_ini);

          // Initial position covariances (GPS and LBL)
          m_kal.setCovariance(STATE_X, m_state_cov[SC_POSITION]);
          m_kal.setCovariance(STATE_Y, m_state_cov[SC_POSITION]);
          m_kal.setCovariance(STATE_PSI, m_state_cov[SC_ANGLES]);
          m_kal.setCovariance(STATE_R, m_state_cov[SC_ANGLES]);
          m_kal.setCovariance(STATE_U, m_state_cov[SC_SPEED]);
          m_kal.setCovariance(STATE_V, m_state_cov[SC_SPEED]);
          m_kal.setCovariance(STATE_R_BIAS, m_state_cov[SC_BIASES]);
          m_kal.setCovariance(STATE_K, m_state_cov[SC_RPM]);

          // No heading bias estimation without IMU.
          m_kal.setCovariance(STATE_PSI_BIAS, 0.0);

          return true;
        }

        void
        reset(void)
        {
          BasicNavigation::reset();
          m_gps_reading = false;
          m_usbl_reading = false;
        }

        double
        getBiasedHeading(void) const
        {
          return m_kal.getState(STATE_PSI) + m_kal.getState(STATE_PSI_BIAS);
        }

        void
        onConsumeLblConfig(void)
        {
          if (m_kal.resize(NUM_OUT + m_ranging.getSize()))
            Task::onUpdateParameters();
        }  // 按 GPS 精度实时切换 EKF 对 GPS 的信任程度
        void
        updateKalmanGpsParameters(double hacc)
        {
          if (hacc > GPS_BAD)
          {
            m_kal.setMeasurementNoise(OUT_GPS_X, m_args.gps_noise[GPS_BAD_IX]);
            m_kal.setMeasurementNoise(OUT_GPS_Y, m_args.gps_noise[GPS_BAD_IX]);
          }
          if (hacc > GPS_AVG && hacc <= GPS_BAD)
          {
            m_kal.setMeasurementNoise(OUT_GPS_X, m_args.gps_noise[GPS_AVG_IX]);
            m_kal.setMeasurementNoise(OUT_GPS_Y, m_args.gps_noise[GPS_AVG_IX]);
          }
          if (hacc > GPS_GOOD && hacc <= GPS_AVG)
          {
            m_kal.setMeasurementNoise(OUT_GPS_X, m_args.gps_noise[GPS_GOOD_IX]);
            m_kal.setMeasurementNoise(OUT_GPS_Y, m_args.gps_noise[GPS_GOOD_IX]);
          }
          if (hacc <= GPS_GOOD)
          {
            m_kal.setMeasurementNoise(OUT_GPS_X, m_args.gps_noise[GPS_PURE_IX]);
            m_kal.setMeasurementNoise(OUT_GPS_Y, m_args.gps_noise[GPS_PURE_IX]);
          }
        }  // 把 GPS 定位结果装进观测向量 z 的 x/y 分量。
        // 【注意】这里只填"观测值"，不直接改状态——状态更新由 m_kal.update() 完成。
        // 配套的两步在 step() 里：
        //   setOutput(OUT_GPS_X, x)                 ← 这里：观测值
        //   setInnovation(OUT_GPS_X, 观测 - 状态)   ← step()：新息(innovation)
        void
        runKalmanGPS(double x, double y)
        {
          m_gps_reading = true;

          // Define Measurements matrix - GPS
          m_kal.setOutput(OUT_GPS_X, x);
          m_kal.setOutput(OUT_GPS_Y, y);
        }

        void
        runKalmanUSBL(double x, double y)
        {  // if there's gps, ignore usbl.
          if (!m_time_without_gps.overflow())
            return;

          m_usbl_reading = true;

          // set kalman.
          m_kal.setMeasurementNoise(OUT_GPS_X, m_args.usbl_noise);
          m_kal.setMeasurementNoise(OUT_GPS_Y, m_args.usbl_noise);

          m_kal.setOutput(OUT_GPS_X, x);
          m_kal.setOutput(OUT_GPS_Y, y);
        }

        void
        getSpeedOutputStates(unsigned* u, unsigned* v)
        {
          *u = OUT_U;
          *v = OUT_V;
        }  //告诉框架：标准量测输出一共有几路
        unsigned
        getNumberOutputs(void)
        {
          return NUM_OUT;
        }  //AUV 导航每拍的 EKF 主循环
        void
        task(void)
        {
          if(!BasicNavigation::isActive())
            return;

          // Compute time delta.
          double tstep = getTimeStep();  // Check if we have a valid time delta.
          if (tstep < 0)
            return;

          // Kalman Filter
          // Reset and Discretize A matrix
          Matrix a(NUM_STATE, NUM_STATE, 0.0);
          setTransition(a);

          Matrix x(NUM_STATE, 1, 0.0);
          x = m_kal.getState();

          m_kal.setStateTransition((a * tstep).expmts());

          // Modify covariance state transition matrix.
          double yaw = m_kal.getState(STATE_PSI);  // 当前艏向角，用于构造旋转矩阵

          a(STATE_X, STATE_PSI) = (- x(STATE_U) * std::sin(yaw)
                                   - x(STATE_V) * std::cos(yaw));
          a(STATE_Y, STATE_PSI) = (x(STATE_U) * std::cos(yaw)
                                   - x(STATE_V) * std::sin(yaw));

          m_kal.setCovarianceTransition((a * tstep).expmts());

          // Kalman Prediction.
          m_kal.predict();

          // Euler Angles update modes.
          double hrate = getHeadingRate();
          m_kal.setOutput(OUT_R, hrate);

          if (m_dead_reckoning)
            m_heading += tstep * hrate;
          else  // DVL 或 GPS 至少一个正常：用速度 u,v 推算位置
            m_heading += Angles::minSignedAngle(m_heading, Angles::normalizeRadian(getEuler(AXIS_Z)));

          // Update heading in Kalman filter.
          m_kal.setOutput(OUT_PSI, m_heading);
          m_kal.setInnovation(OUT_PSI, m_kal.getOutput(OUT_PSI) - getBiasedHeading());

          double r = m_kal.getState(STATE_R) + m_kal.getState(STATE_R_BIAS);
          m_kal.setInnovation(OUT_R,  m_kal.getOutput(OUT_R) - r);

          // GPS innovation matrix.
          if (m_gps_reading || m_usbl_reading)
          {  // Reset GPS counter.
            if (m_gps_reading)
              m_time_without_gps.reset();

            m_kal.setInnovation(OUT_GPS_X, m_kal.getOutput(OUT_GPS_X) - m_kal.getState(STATE_X));
            m_kal.setInnovation(OUT_GPS_Y, m_kal.getOutput(OUT_GPS_Y) - m_kal.getState(STATE_Y));
          }
          else  // DVL 或 GPS 至少一个正常：用速度 u,v 推算位置
          {
            m_kal.setObservation(OUT_GPS_X, STATE_X, 0.0);
            m_kal.setObservation(OUT_GPS_Y, STATE_Y, 0.0);
          }

          // Speed innovation matrix.
          if (m_valid_gv || m_valid_wv)
          {
            runKalmanDVL();
            m_kal.setInnovation(OUT_U, m_kal.getOutput(OUT_U) - m_kal.getState(STATE_U));
            m_kal.setInnovation(OUT_V, m_kal.getOutput(OUT_V) - m_kal.getState(STATE_V));
          }
          else if (m_time_without_gps.overflow() && m_time_without_dvl.overflow())  // DVL 或 GPS 至少一个正常：用速度 u,v 推算位置
          {
            double u = 0.0;
            double speed_m  = getRpmToMs(m_rpm);
            if(m_args.rpm_estimation)
            {
              u = m_rpm * m_kal.getState(STATE_K) * std::cos(getEuler(AXIS_Y));
              if(m_args.speed_relation_Limit)
              {
                double speedR = (std::abs(u) - std::abs(speed_m))/ std::abs(speed_m) * 100;
                if (speedR > m_args.speed_relation_limit_value)
                  u = speed_m;
              }
            }
            else  // DVL 或 GPS 至少一个正常：用速度 u,v 推算位置
            {
              u = speed_m;
            }
            m_kal.setInnovation(OUT_U, u - m_kal.getState(STATE_U));
            m_kal.setInnovation(OUT_V, 0 - m_kal.getState(STATE_V));
          }
          else  // DVL 或 GPS 至少一个正常：用速度 u,v 推算位置
          {  // Use GPS speed over ground.
            if (m_gps_reading && m_time_without_dvl.overflow())
            {
              m_kal.setInnovation(OUT_U, m_gps_sog - m_kal.getState(STATE_U));
              m_kal.setInnovation(OUT_V, 0 - m_kal.getState(STATE_V));
            }
            else  // DVL 或 GPS 至少一个正常：用速度 u,v 推算位置
            {
              m_kal.setObservation(OUT_U, STATE_U, 0.0);
              m_kal.setObservation(OUT_V, STATE_V, 0.0);
            }
          }


          // Do not estimate bias using LBL fixes.
          double bias = m_kal.getCovariance(STATE_PSI_BIAS);
          if (m_lbl_reading)
            m_kal.resetCovariance(STATE_PSI_BIAS);

          // Extended Kalman Filter update with no threshold defined.
          m_kal.update(0.0);

          // Restore bias estimation.
          if (m_lbl_reading)
          {
            m_kal.setCovariance(STATE_PSI_BIAS, bias + m_process_noise[PN_PSI_BIAS]);
            m_lbl_reading = false;
          }

          // Limit revolutions to speed factor.
          float k_lim = trimValue(m_kal.getState(STATE_K),
                                  m_args.rpm_ini - m_args.rpm_max,
                                  m_args.rpm_ini + m_args.rpm_max);

          m_kal.setState(STATE_K, k_lim);

          // Check alignment threshold index.
          double diff_psi = std::abs(Angles::normalizeRadian(Angles::normalizeRadian(m_kal.getState(STATE_PSI))
                                                             - Angles::normalizeRadian(getEuler(AXIS_Z)) ) );


          if (m_dead_reckoning)
          {
            if (m_kal.getCovariance(STATE_PSI_BIAS) < m_args.alignment_index &&
                diff_psi < Angles::normalizeRadian(Angles::radians(m_args.alignment_diff)) )
            {
              m_aligned = true;
              m_heading_buffer = 0;
            }
            else  // DVL 或 GPS 至少一个正常：用速度 u,v 推算位置
            {
              if (m_aligned)
              {
                m_heading_buffer++;
                if (m_heading_buffer > m_args.heading_buffer_value)
                {
                  sendDeActiveIMU();
                  war(DTR("navigation not aligned - Automatic IMU poweroff"));
                  m_aligned  = false;
                  m_heading_buffer=0;
                }
              }
            }
          }

          checkUncertainty(m_args.abort);

          logData();
          reportToBus();

          // Reset variables.
          updateBuffers(c_wma_filter);
          m_gps_reading = false;
          m_usbl_reading = false;
          m_valid_gv = false;
          m_valid_wv = false;
          resetKalman();
        }

        void
        sendDeActiveIMU(void)
        {
          IMC::EntityParameter p;
          p.name = "Active";
          p.value = "false";
          IMC::SetEntityParameters msg;
          msg.name = m_args.elabel_imu ;
          msg.params.push_back(p);
          dispatch(msg);
        }

        void
        startSpeedModel(Parsers::Config* config)
        {
          try
          {
            m_speed_model = new Plans::SpeedModel(config);
            m_speed_model->validate();
          }
          catch (...)
          {
            Memory::clear(m_speed_model);
            inf(DTR("Nav: speed model invalid"));
          }
        }

        double
        getRpmToMs(double rpm) const
        {
          if (m_speed_model != nullptr)
            return m_speed_model->toMPS(rpm, IMC::SUNITS_RPM);

          return m_args.rpm_ini * rpm;
        }

        // Reinitialize Extended Kalman Filter transition matrix function.
        void
        setTransition(Matrix& A)
        {
          A.fill(0.0);  // 清空矩阵：所有元素置零，准备重新填充

          double phi = Angles::normalizeRadian(getEuler(AXIS_X));  // 横滚角（roll），用于机体系→导航系的坐标变换
          double theta = Angles::normalizeRadian(getEuler(AXIS_Y));  // 纵倾角（pitch），用于机体系→导航系的坐标变换

          A(STATE_PSI, STATE_R) = 1.0;  // 艏向角 ψ ← 角速度 r：ψ_new = ψ + r·dt，这里填的是 r 的系数 1

          double yaw = m_kal.getState(STATE_PSI);  // 当前艏向角，用于构造旋转矩阵

          if (m_time_without_dvl.overflow() && m_time_without_gps.overflow())  // DVL 和 GPS 都超时：没有速度观测，只能用 RPM 因子 K 推算位置
          {
            A(STATE_X, STATE_K) = m_rpm * std::cos(yaw) * std::cos(theta);  // 位置x ← K：x_new = x + K·rpm·cos(ψ)·cos(θ)·dt
            A(STATE_Y, STATE_K) = m_rpm * std::sin(yaw) * std::cos(theta);  // 位置y ← K：y_new = y + K·rpm·sin(ψ)·cos(θ)·dt
          }
          else  // DVL 或 GPS 至少一个正常：用速度 u,v 推算位置
          {
            A(STATE_X, STATE_U) = std::cos(yaw) * std::cos(theta);  // 位置x ← u：x_new = x + u·cos(ψ)·cos(θ)·dt
            A(STATE_X, STATE_V) = (std::cos(yaw) * std::sin(theta) * std::sin(phi)  // 位置x ← v：含姿态耦合项（横滚+纵倾对横向速度的影响）
                                   - std::sin(yaw) * std::cos(phi));
            A(STATE_Y, STATE_U) = std::sin(yaw) * std::cos(theta);  // 位置y ← u：y_new = y + u·sin(ψ)·cos(θ)·dt
            A(STATE_Y, STATE_V) = (std::sin(yaw) * std::sin(theta) * std::sin(phi)  // 位置y ← v：含姿态耦合项（横滚+纵倾对横向速度的影响）
                                   + std::cos(yaw) * std::cos(phi));
          }
        }

        // Reinitialize Extended Kalman Filter output matrix function.
        //
        // 【这就是"每种传感器修正哪个分量"的答案】
        // setObservation(观测分量, 状态分量, 系数)  —— 观测矩阵 H 的一行。
        // 系数 1.0 = 该传感器直接修正这个状态量；0.0 = 本周期不修正。
        //
        //   传感器      观测量   →  修正的状态分量
        //   ─────────────────────────────────────────────
        //   DVL/WaterVel 速度 u →  STATE_U  (纵向速度)
        //   DVL/WaterVel 速度 v →  STATE_V  (横向速度)
        //   AHRS/Euler   艏向 ψ →  STATE_PSI      (艏向)
        //                          STATE_PSI_BIAS (艏向零偏) ← 关键
        //   IMU/AHRS     角速度 r → STATE_R       (转头角速度)
        //                          STATE_R_BIAS   (角速度零偏)
        //   GPS/USBL/LBL x,y   →  STATE_X / STATE_Y (北向/东向位置)
        //
        // 【两个要点】
        // 1. OUT_R 对 STATE_R_BIAS 的系数是 0.0 —— 角速度零偏不由角速度观测
        //    直接修正，而是靠 STATE_R 估计出来后减去零偏得到残差再反推
        //    （见 step() 里 setInnovation(OUT_R, 观测 - (STATE_R + STATE_R_BIAS))）。
        // 2. 深度不在这个表里。深度不进 EKF 的 x 向量，而是直接算进 EstimatedState
        //    的 depth 字段（见 logData()），因为深度变化慢、无需滤波估计。
        //
        // 缺观测时怎么处理：把对应 H 系数置 0（setObservation(..., 0.0)），
        // 该传感器本周期就完全不参与更新，而不是喂一个假值。
        void
        resetKalman(void)
        {
          m_kal.resetOutputs();

          // —— 速度观测（DVL / Water Velocity）——
          m_kal.setObservation(OUT_U, STATE_U, 1.0);
          m_kal.setObservation(OUT_V, STATE_V, 1.0);

          // —— 艏向观测（AHRS/EulerAngles）——
          // 同时修正艏向本身和艏向零偏：磁罗盘长期有偏，把偏差也估出来
          m_kal.setObservation(OUT_PSI, STATE_PSI, 1.0);
          m_kal.setObservation(OUT_PSI, STATE_PSI_BIAS, 1.0);

          // —— 角速度观测（IMU/AngularVelocity）——
          // 注意 STATE_R_BIAS 系数是 0.0：零偏走间接修正路径
          m_kal.setObservation(OUT_R, STATE_R, 1.0);
          m_kal.setObservation(OUT_R, STATE_R_BIAS, 0.0);

          // —— 位置观测（GPS / USBL / LBL）——
          m_kal.setObservation(OUT_GPS_X, STATE_X, 1.0);
          m_kal.setObservation(OUT_GPS_Y, STATE_Y, 1.0);
        }

        // 把 EKF 状态打包成对外发布的 EstimatedState。
        //
        // 【深度为什么不在 EKF 状态向量里】
        // m_kal 的 x 向量只有 9 个 STATE_*，没有深度。但 logData 里直接写了：
        //   m_estate.depth = getDepth() - m_estate.theta * distance_depth_sensor;
        // 原因是深度变化极缓（船上下沉是慢过程），不需要 Kalman 滤波去"估计"，
        // 直接用滤波后的深度测量值 + 一个俯仰角补偿即可。
        // distance_depth_sensor 是"深度计安装位置到船体原点的距离"——
        // 深度计装在船头和重心不在同一处时，船一俯仰两者读数就差一个 sinθ。
        void
        logData(void)
        {
          m_estate.psi = Angles::normalizeRadian(m_kal.getState(STATE_PSI));
          m_estate.r = m_kal.getState(STATE_R);
          onDispatchNavigation();

          m_estate.u = m_avg_speed->update(m_kal.getState(STATE_U));
          m_estate.v = m_kal.getState(STATE_V);
          m_estate.depth = getDepth() - m_estate.theta * m_args.distance_depth_sensor;
          m_estate.z = m_last_z + m_estate.depth;

          // Log Navigation Uncertainty.
          m_uncertainty.psi = m_kal.getCovariance(STATE_PSI);
          m_uncertainty.bias_psi = m_kal.getCovariance(STATE_PSI_BIAS);
          m_uncertainty.r = m_kal.getCovariance(STATE_R);
          m_uncertainty.u = m_kal.getCovariance(STATE_U);
          m_uncertainty.v = m_kal.getCovariance(STATE_V);

          // Log Navigation Data.
          m_navdata.cog = (std::abs(m_kal.getState(STATE_U)) > 0.2 ?
                           std::atan2(m_estate.vy, m_estate.vx) : 0);
          m_navdata.bias_psi = m_kal.getState(STATE_PSI_BIAS);
          m_navdata.bias_r = m_kal.getState(STATE_R_BIAS);

          // Navigation error.
          m_navdata.custom_x = Math::norm(m_kal.getInnovation(OUT_GPS_X),
                                          m_kal.getInnovation(OUT_GPS_Y));
          m_navdata.custom_y = m_kal.getState(STATE_K);

          double ang = m_estate.psi - Angles::normalizeRadian(getEuler(AXIS_Z));
          m_navdata.custom_z = Angles::degrees(Angles::normalizeRadian(ang));
        }
      };
    }
  }
}

DUNE_TASK
