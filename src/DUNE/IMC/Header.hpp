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
// Automatically generated.                                                 *
//***************************************************************************
// IMC XML MD5: 431862f600b4d042b3eff3664dedc132                            *
//***************************************************************************

#ifndef DUNE_IMC_HEADER_HPP_INCLUDED_
#define DUNE_IMC_HEADER_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/Config.hpp>

namespace DUNE
{
  namespace IMC
  {
    //! Header format.
    struct Header
    {
      //! Synchronization Number.同步编号，标记「一帧消息从哪里开始」。
      //数据在线缆/串口上是一长串字节，接收方要从中间切出「完整的一条消息」，IMC 解析第一步：找 sync，再读后面的内容
      uint16_t sync;
      //! Message Identification Number.消息类型编号。告诉你：这条消息“是什么”
      uint16_t mgid;
      //! Message size.业务区长度：头之后还有多少字节，知道读到哪为止
      uint16_t size;
      //! Time stamp.时间戳，这条消息产生的时间
      fp64_t timestamp;
      //! Source Address.发送方地址，标识这条 IMC 消息是由哪个设备 / 模块发出
      uint16_t src;
      //! Source Entity.发送方实体
      uint8_t src_ent;
      //! Destination Address.接收方系统的地址
      uint16_t dst;
      //! Destination Entity.	接收方系统里的哪个实体/模块
      uint8_t dst_ent;
    };
  }
}

#endif
