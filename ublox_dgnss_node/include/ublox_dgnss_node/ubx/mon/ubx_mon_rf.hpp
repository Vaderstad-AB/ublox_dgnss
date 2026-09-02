// Copyright 2026 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UBLOX_DGNSS_NODE__UBX__MON__UBX_MON_RF_HPP_
#define UBLOX_DGNSS_NODE__UBX__MON__UBX_MON_RF_HPP_

#include "ublox_dgnss_node/ubx/ubx.hpp"
#include <cstring>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace ubx::mon::rf
{

// one entry per RF block; a block corresponds to a GNSS band, not to an antenna port
struct RFBlock
{
  u1_t blockId;
  u1_t jammingState;      // 0 unknown, 1 ok, 2 warning, 3 critical
  u1_t antStatus;         // 0 INIT, 1 DONTKNOW, 2 OK, 3 SHORT, 4 OPEN
  u1_t antPower;          // 0 OFF, 1 ON, 2 DONTKNOW
  u4_t postStatus;
  u2_t noisePerMS;
  u2_t agcCnt;            // 0..8191 == 100% of maximum gain
  u1_t cwSuppression;
  i1_t ofsI;
  u1_t magI;
  i1_t ofsQ;
  u1_t magQ;
  u1_t rfBlockGnssBand;   // 0 unknown, 1 L1, 2 L2, 3 L3, 4 L5
};

class MonRFPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_MON;
  static const msg_id_t MSG_ID = UBX_MON_RF;

  u1_t version;
  u1_t nBlocks;
  u1_t msgSource;
  std::vector<RFBlock> blocks;

public:
  MonRFPayload()
  : UBXPayload(MSG_CLASS, MSG_ID) {}

  MonRFPayload(u1_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    version = 0;
    nBlocks = 0;
    msgSource = 0;

    if (size < 4) {
      return;
    }

    auto ptr = payload_.data();
    memcpy(&version, ptr, 1); ptr += 1;
    memcpy(&nBlocks, ptr, 1); ptr += 1;
    x1_t recInf = 0;
    memcpy(&recInf, ptr, 1); ptr += 1;
    msgSource = recInf & 0x03;
    ptr += 1;   // reserved0

    // a truncated frame must never be walked past the end of the payload
    size_t available = (size - 4) / 24;
    size_t count = (nBlocks < available) ? nBlocks : available;

    for (size_t i = 0; i < count; ++i) {
      RFBlock b;
      memcpy(&b.blockId, ptr, 1); ptr += 1;
      x1_t flags = 0;
      memcpy(&flags, ptr, 1); ptr += 1;
      b.jammingState = flags & 0x03;
      memcpy(&b.antStatus, ptr, 1); ptr += 1;
      memcpy(&b.antPower, ptr, 1); ptr += 1;
      memcpy(&b.postStatus, ptr, 4); ptr += 4;
      ptr += 4;   // reserved1[4]
      memcpy(&b.noisePerMS, ptr, 2); ptr += 2;
      memcpy(&b.agcCnt, ptr, 2); ptr += 2;
      memcpy(&b.cwSuppression, ptr, 1); ptr += 1;
      memcpy(&b.ofsI, ptr, 1); ptr += 1;
      memcpy(&b.magI, ptr, 1); ptr += 1;
      memcpy(&b.ofsQ, ptr, 1); ptr += 1;
      memcpy(&b.magQ, ptr, 1); ptr += 1;
      memcpy(&b.rfBlockGnssBand, ptr, 1); ptr += 1;
      ptr += 2;   // reserved2[2]
      blocks.push_back(b);
    }
  }

  std::tuple<u1_t *, size_t> make_poll_payload() override
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }

  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << static_cast<int>(version);
    oss << " nBlocks: " << static_cast<int>(nBlocks);
    for (auto & b : blocks) {
      oss << " [block " << static_cast<int>(b.blockId);
      oss << " band: " << static_cast<int>(b.rfBlockGnssBand);
      oss << " antStatus: " << static_cast<int>(b.antStatus);
      oss << " antPower: " << static_cast<int>(b.antPower);
      oss << " jamming: " << static_cast<int>(b.jammingState);
      oss << " agcCnt: " << b.agcCnt;
      oss << " noisePerMS: " << b.noisePerMS << "]";
    }
    return oss.str();
  }
};

}  // namespace ubx::mon::rf

#endif  // UBLOX_DGNSS_NODE__UBX__MON__UBX_MON_RF_HPP_
