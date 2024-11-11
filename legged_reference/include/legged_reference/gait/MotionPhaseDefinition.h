/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "legged_reference/common/Types.h"

namespace ocs2
{
  namespace hexapod_robot
  {
    using namespace legged_robot;
    
    enum ModeNumber
    { // leg index 543210(LB LM LF RB RM RF) - 0b000000
      FLY = 0b000000,
      TRIPOD1 = 0b010101,
      TRIPOD2 = 0b101010,
      STANCE = 0b111111,
    };

    inline contact_flag_t modeNumber2StanceLeg(const size_t &modeNumber)
    {
      contact_flag_t stanceLegs;
      for (size_t i = 0; i < 6; i++)
        stanceLegs[i] = (modeNumber >> i) & 1;
      return stanceLegs;
    }

    inline size_t stanceLeg2ModeNumber(const contact_flag_t &stanceLegs)
    {
      size_t modeNumber = 0;
      for (size_t i = 0; i < 6; i++)
        modeNumber |= stanceLegs[i] << i;
      return modeNumber;
    }

    inline std::string modeNumber2String(const size_t &modeNumber)
    {
      // build the map from mode number to name
      std::map<size_t, std::string> modeToName;
      modeToName[FLY] = "FLY";
      modeToName[TRIPOD1] = "TRIPOD1";
      modeToName[TRIPOD2] = "TRIPOD2";
      modeToName[STANCE] = "STANCE";
      return modeToName[modeNumber];
    }

    inline size_t string2ModeNumber(const std::string &modeString)
    {
      // build the map from name to mode number
      std::map<std::string, size_t> nameToMode;
      nameToMode["FLY"] = FLY;
      nameToMode["TRIPOD1"] = TRIPOD1;
      nameToMode["TRIPOD2"] = TRIPOD2;
      nameToMode["STANCE"] = STANCE;
      return nameToMode[modeString];
    }

  } // namespace hexapod_robot
} // end of namespace ocs2
