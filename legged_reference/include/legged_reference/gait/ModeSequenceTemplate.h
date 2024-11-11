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
#include <vector>

#include <ocs2_core/reference/ModeSchedule.h>
#include "ocs2_legged_robot/gait/ModeSequenceTemplate.h"
#include "ocs2_legged_robot/gait/Gait.h"
#include "MotionPhaseDefinition.h"

namespace ocs2
{
  namespace hexapod_robot
  {
    using namespace legged_robot;

    /** Print the modesequence template */
    std::ostream &operator<<(std::ostream &stream, const ModeSequenceTemplate &modeSequenceTemplate);

    /** Converts a mode sequence template to a gait */
    Gait toGait(const ModeSequenceTemplate &modeSequenceTemplate);

    /**
     * Load a modesequence template from file.  The template needs to be declared as:
     *
     * topicName
     * {
     *   modeSequence
     *   {
     *     [0]     mode0
     *     [1]     mode1
     *   }
     *   switchingTimes
     *   {
     *     [0]     0.0
     *     [1]     t1
     *     [2]     T
     *   }
     * }
     */
    ModeSequenceTemplate loadModeSequenceTemplate(const std::string &filename, const std::string &topicName, bool verbose = true);

    /**
     * Load a mode schedule template from file.  The schedule needs to be declared as:
     *
     * topicName
     * {
     *   modeSequence
     *   {
     *     [0]     mode0
     *     [1]     mode1
     *     [2]     mode2
     *   }
     *   eventTimes
     *   {
     *     [0]     t0
     *     [1]     t1
     *   }
     * }
     */
    ModeSchedule loadModeSchedule(const std::string &filename, const std::string &topicName, bool verbose);

  } // namespace hexapod_robot
} // namespace ocs2
