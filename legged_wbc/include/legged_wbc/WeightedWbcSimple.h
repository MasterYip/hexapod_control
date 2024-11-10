/**
 * @file WeightedWbc_Simple.h
 * @author Master Yip (2205929492@qq.com)
 * @brief Remove desired ground react force input (\lambda)
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */

//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WbcBase.h"

namespace legged
{

  class WeightedWbcSimple : public WbcBase
  {
  public:
    using WbcBase::WbcBase;

    vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode,
                    scalar_t period) override;

    void loadTasksSetting(const std::string &taskFile, bool verbose) override;

  protected:
    virtual Task formulateConstraints();
    virtual Task formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);
    virtual Task formulateNoContactMotionTask();
    virtual Task formulateSwingLegTask();
    virtual Task formulateFrictionConeTask();

  private:
    scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
    hex_contact_flag_t contactFlagHex_{}; // Hexapod
  };

} // namespace legged