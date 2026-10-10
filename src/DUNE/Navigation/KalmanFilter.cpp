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
// Author: Pedro Vaz Teixeira                                               *
// Author: José Braga                                                       *
//***************************************************************************

// DUNE headers.
#include <DUNE/Navigation/KalmanFilter.hpp>

namespace DUNE
{
  namespace Navigation
  {
    KalmanFilter::KalmanFilter(void)
    {
      m_state_count = 1;
      Math::Matrix I(1);
      I(0) = 0;
      m_x = m_y = m_ax = m_ap = m_c = m_p = m_q = m_r = m_innov = I;
    }

    KalmanFilter::KalmanFilter(Math::Matrix& A, Math::Matrix& C, Math::Matrix& P, Math::Matrix& Q)
    {
      m_ax = A;
      m_ap = A;
      m_c = C;
      m_p = P;
      m_q = Q;
      m_state_count = m_ax.rows();
      m_x.resizeAndFill(m_state_count, 1, 0.0);
    }

    void
    KalmanFilter::reset(short num_states, short num_outputs)
    {
      m_x.resizeAndFill(num_states, 1, 0.0);
      m_y.resizeAndFill(num_outputs, 1, 0.0);
      m_ax.resizeAndFill(num_states, num_states, 0.0);
      m_ap.resizeAndFill(num_states, num_states, 0.0);
      m_c.resizeAndFill(num_outputs, num_states, 0.0);
      m_p.resizeAndFill(num_states, num_states, 0.0);
      m_q.resizeAndFill(num_states, num_states, 0.0);
      m_r.resizeAndFill(num_outputs, num_outputs, 0.0);
      m_innov.resizeAndFill(num_outputs, 1, 0.0);
      m_state_count = m_ax.rows();

      m_ax.identity();
      m_ap.identity();
    }

    bool
    KalmanFilter::resize(short num_outputs)
    {
      if (num_outputs != m_y.rows())
      {
        short num_states = m_state_count;
        m_y.resizeAndKeep(num_outputs, 1);
        m_c.resizeAndKeep(num_outputs, num_states);
        m_r.resizeAndKeep(num_outputs, num_outputs);
        m_innov.resizeAndKeep(num_outputs, 1);
        return true;
      }
      else
        return false;
    }

    void
    KalmanFilter::initialize(Math::Matrix& x0, Math::Matrix& P0)
    {
      if ((size_t)P0.rows() != m_state_count || (size_t)P0.columns() != m_state_count)
        throw std::runtime_error(DTR("invalid dimensions"));

      if ((size_t)x0.rows() != m_state_count || (size_t)x0.columns() != 1)
        throw std::runtime_error(DTR("invalid dimensions"));

      m_x = x0;
      m_p = P0;
    }
    //带控制输入版本
    void
    KalmanFilter::normalize(void)
    {
      m_p = 0.5 * (m_p + transpose(m_p));
    }

    void
    KalmanFilter::predict(Math::Matrix& b, Math::Matrix& u)//b=控制矩阵，u=控制输入（如 RPM）
    {
      if (u.rows() != b.columns() || u.columns() != 1)
        throw std::runtime_error(DTR("invalid dimensions"));

      m_x = m_ax * m_x + b * u;//状态预测：x = F·x + B·u
      m_p = m_ap * m_p * transpose(m_ap) + m_q;//协方差预测：P = F·P·F^T + Q
    }
    //无控制输入
    void
    KalmanFilter::predict(void)
    {
      m_x = m_ax * m_x;//状态预测：x = F·x
      m_p = m_ap * m_p * transpose(m_ap) + m_q;//协方差预测：P = F·P·F^T + Q
    }

    int
    KalmanFilter::update(float threshold)
    {
      if (m_c.rows() != m_innov.rows())  // 维度检查：观测矩阵行数 == 新息向量行数
        throw std::runtime_error(DTR("invalid dimensions"));

      if (m_c.columns() != m_x.rows())  // 维度检查：观测矩阵列数 == 状态向量行数
        throw std::runtime_error(DTR("invalid dimensions"));

      if (m_innov.columns() != 1)  // 维度检查：新息是列向量
        throw std::runtime_error(DTR("invalid dimensions"));

      if (m_r.rows() != m_r.columns() || m_r.rows() != m_innov.rows())  // 维度检查：测量噪声是方阵且匹配新息维度
        throw std::runtime_error(DTR("invalid dimensions"));

      // Measurement prediction covariance.
      Math::Matrix S = (m_c * m_p * transpose(m_c)) + m_r;  // S = H·P·H^T + R：测量预测协方差（预测观测的不确定性 + 测量噪声）
      Math::Matrix S_1;

      // Inverse of the measurement prediction covariance.
      try
      {
        S_1 = inverse(S);  // S 的逆矩阵，用于后续增益计算
      }
      catch (...)
      {
        throw std::runtime_error(DTR("matrix inversion error"));
      }

      // Check if innovation is above a threshold value.
      // Set threshold to 0 to accept everything.
      if (threshold != 0)
      {
        double level = (transpose(m_innov) * S_1 * m_innov)(0);  // 马氏距离：新息^(-1)·S^(-1)·新息，衡量新息是否异常

        if (level >= threshold)  // 超过阈值说明新息异常（可能是野值），拒绝本次更新
          return -1;
      }

      // Kalman Gain.
      Math::Matrix K = m_p * transpose(m_c) * S_1;  // 卡尔曼增益 K = P·H^T·S^(-1)：决定信测量多少

      // State update.
      m_x = m_x + K * m_innov;  // 状态修正：x_new = x_pred + K·(实测 - 预测)

      // State Covariance update.
      m_p = m_p - K * m_c * m_p;  // 协方差修正：P_new = P - K·H·P（修正后不确定性降低）

      return 0;
    }

    void
    KalmanFilter::setState(short pos, double value)
    {
      if ((size_t)pos >= m_state_count)
        throw std::runtime_error(DTR("invalid index"));

      m_x(pos) = value;
    }

    void
    KalmanFilter::resetState(void)
    {
      m_x.fill(0.0);
    }

    void
    KalmanFilter::resetOutputs(void)
    {
      m_y.fill(0.0);
      m_innov.fill(0.0);
      m_c.fill(0.0);
    }

    void
    KalmanFilter::setTransitions(Math::Matrix a)
    {
      setStateTransition(a);
      setCovarianceTransition(a);
    }

    void
    KalmanFilter::setStateTransition(Math::Matrix a)
    {
      if (a.rows() != a.columns())
        throw std::runtime_error(DTR("invalid dimensions"));

      if ((size_t)a.rows() != m_state_count)
        throw std::runtime_error(DTR("invalid dimensions"));

      m_ax = a;
    }

    void
    KalmanFilter::setCovarianceTransition(Math::Matrix a)
    {
      if (a.rows() != a.columns())
        throw std::runtime_error(DTR("invalid dimensions"));

      if ((size_t)a.rows() != m_state_count)
        throw std::runtime_error(DTR("invalid dimensions"));

      m_ap = a;
    }

    void
    KalmanFilter::setOutput(short pos, double value)
    {
      if (pos >= m_y.rows())
        throw std::runtime_error(DTR("invalid index"));

      m_y(pos) = value;
    }

    void
    KalmanFilter::setInnovation(short pos, double value)
    {
      if (pos >= m_innov.rows())
        throw std::runtime_error(DTR("invalid index"));

      m_innov(pos) = value;
    }

    void
    KalmanFilter::setObservation(short ln, short cl, double value)//ln=行号（观测分量），cl=列号（状态分量），value=填入的系数
    {
      if (ln >= m_c.rows() || cl >= m_c.columns())
        throw std::runtime_error(DTR("invalid index"));

      m_c(ln, cl) = value;
    }

    void
    KalmanFilter::setProcessNoise(short ln, short cl, double value)
    {
      if (ln >= m_q.rows() || cl >= m_q.columns())
        throw std::runtime_error(DTR("invalid index"));

      m_q(ln, cl) = value;//往观测矩阵 H 的指定位置填值
    }

    void
    KalmanFilter::setProcessNoise(short in, double value)
    {
      if ((size_t)in >= m_state_count)
        throw std::runtime_error(DTR("invalid index"));

      m_q(in, in) = value;
    }

    void
    KalmanFilter::setProcessNoise(double value)
    {
      for (size_t i = 0; i < m_state_count; ++i)
        m_q(i, i) = value;
    }

    void
    KalmanFilter::setMeasurementNoise(short ln, short cl, double value)
    {
      if (ln >= m_r.rows() || cl >= m_r.columns())
        throw std::runtime_error(DTR("invalid index"));

      m_r(ln, cl) = value;
    }

    void
    KalmanFilter::setMeasurementNoise(short in, double value)
    {
      if (in >= m_r.rows())
        throw std::runtime_error(DTR("invalid index"));

      m_r(in, in) = value;
    }

    void
    KalmanFilter::setMeasurementNoise(double value)
    {
      for (int i = 0; i < m_r.rows(); ++i)
        m_r(i, i) = value;
    }

    void
    KalmanFilter::setCovariance(short ln, short cl, double value)
    {
      if (ln >= m_p.rows() || cl >= m_p.columns())
        throw std::runtime_error(DTR("invalid index"));

      m_p(ln, cl) = value;
    }

    void
    KalmanFilter::setCovariance(short in, double value)
    {
      if ((size_t)in >= m_state_count)
        throw std::runtime_error(DTR("invalid index"));

      m_p(in, in) = value;
    }

    void
    KalmanFilter::setCovariance(double value)
    {
      for (size_t i = 0; i < m_state_count; ++i)
        m_p(i, i) = value;
    }

    void
    KalmanFilter::resetCovariance(short in)
    {
      for (size_t i = 0; i < m_state_count; ++i)
      {
        m_p(i, in) = 0.0;
        m_p(in, i) = 0.0;
      }
    }
  }
}
