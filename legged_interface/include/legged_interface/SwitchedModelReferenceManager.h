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

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

// #include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include "legged_reference/gait/MotionPhaseDefinition.h"
#include "legged_reference/gait/GaitSchedule.h"

#include "legged_interface/constraint/SwingTrajectoryPlanner.h"
#include <ocs2_legged_robot/common/utils.h>

namespace ocs2
{
  namespace legged_robot
  {

    inline size_t numberOfClosedContacts(const std::vector<bool> &contactFlags)
    {
      size_t numStanceLegs = 0;
      for (auto legInContact : contactFlags)
      {
        if (legInContact)
        {
          ++numStanceLegs;
        }
      }
      return numStanceLegs;
    }

    inline vector_t weightCompensatingInput(const CentroidalModelInfoTpl<scalar_t> &info, const std::vector<bool> &contactFlags)
    {
      const auto numStanceLegs = numberOfClosedContacts(contactFlags);
      vector_t input = vector_t::Zero(info.inputDim);
      if (numStanceLegs > 0)
      {
        const scalar_t totalWeight = info.robotMass * 9.81;
        const vector3_t forceInInertialFrame(0.0, 0.0, totalWeight / numStanceLegs);
        for (size_t i = 0; i < contactFlags.size(); i++)
        {
          if (contactFlags[i])
          {
            centroidal_model::getContactForces(input, i, info) = forceInInertialFrame;
          }
        } // end of i loop
      }
      return input;
    }
    /**
     * Manages the ModeSchedule and the TargetTrajectories for switched model.
     */
    class SwitchedModelReferenceManager : public ReferenceManager
    {
    public:
      SwitchedModelReferenceManager(std::shared_ptr<hexapod_robot::GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

      ~SwitchedModelReferenceManager() override = default;

      void setModeSchedule(const ModeSchedule &modeSchedule) override;

      virtual std::vector<bool> getContactFlags(scalar_t time) const;

      const std::shared_ptr<hexapod_robot::GaitSchedule> &getGaitSchedule() { return gaitSchedulePtr_; }

      const std::shared_ptr<SwingTrajectoryPlanner> &getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

    protected:
      void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t &initState, TargetTrajectories &targetTrajectories,
                            ModeSchedule &modeSchedule) override;

      std::shared_ptr<hexapod_robot::GaitSchedule> gaitSchedulePtr_;
      std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
    };

    class HexSwitchedModelReferenceManager : public SwitchedModelReferenceManager
    {
    public:
      HexSwitchedModelReferenceManager(std::shared_ptr<hexapod_robot::GaitSchedule> gaitSchedulePtr, std::shared_ptr<HexSwingTrajectoryPlanner> swingTrajectoryPtr)
          : SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)) {}

      ~HexSwitchedModelReferenceManager() override = default;

    protected:
      virtual std::vector<bool> getContactFlags(scalar_t time) const override;
    };

  } // namespace legged_robot
} // namespace ocs2
