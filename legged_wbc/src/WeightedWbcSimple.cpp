/**
 * @file WeightedWbcSimple.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-03-20
 *
 * @copyright Copyright (c) 2024
 *
 */

//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedWbcSimple.h"

#include <qpOASES.hpp>

namespace legged
{

    inline hex_contact_flag_t modeNumber2StanceLegHex(const size_t &modeNumber)
    {
        hex_contact_flag_t stanceLegs; // {LF, RF, LM, RM, LH, RH}
        size_t mode = modeNumber;
        // leg index xx543210b
        for (size_t i = 0; i < 6; i++)
        {
            stanceLegs[i] = mode & 1;
            mode >>= 1;
        }
        return stanceLegs;
    }

    vector_t WeightedWbcSimple::update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode,
                                       scalar_t period)
    {
        // We reimplement the update function here (replace contactFlag_ calculation)
        // WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

        contactFlagHex_ = modeNumber2StanceLegHex(mode);
        numContacts_ = 0;
        for (bool flag : contactFlagHex_)
        {
            if (flag)
            {
                numContacts_++;
            }
        }
        updateMeasured(rbdStateMeasured);
        updateDesired(stateDesired, inputDesired);

        std::cerr << "WeightedWbcSimple::update" << std::endl;
        // Constraints
        Task constraints = formulateConstraints();
        size_t numConstraints = constraints.b_.size() + constraints.f_.size();
        std::cerr << "Formulate constraints" << std::endl;
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
        vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
        A << constraints.a_,
            constraints.d_;

        lbA << constraints.b_, // Equality constraints
            -qpOASES::INFTY * vector_t::Ones(constraints.f_.size()); // Inequality constraints
        ubA << constraints.b_,
            constraints.f_; // clang-format on
        std::cerr << "Load constraints" << std::endl;

        // Cost
        Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
        vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;
        std::cerr << "Load cost" << std::endl;
        
        // Solve
        auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_LOW;
        options.enableEqualities = qpOASES::BT_TRUE;
        qpProblem.setOptions(options);
        int nWsr = 20;
        std::cerr << "Init QP" << std::endl;
        qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
        vector_t qpSol(getNumDecisionVars());

        qpProblem.getPrimalSolution(qpSol.data());
        return qpSol;
    }

    Task WeightedWbcSimple::formulateConstraints()
    {
        std::cerr << "WeightedWbcSimple::formulateConstraints" << std::endl;
        return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
    }

    Task WeightedWbcSimple::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
    {
        return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_;
    }

    Task WeightedWbcSimple::formulateNoContactMotionTask()
    {
        matrix_t a(3 * numContacts_, numDecisionVars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; i++)
        {
            if (contactFlagHex_[i])
            {
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
                b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
                j++;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }

    void WeightedWbcSimple::loadTasksSetting(const std::string &taskFile, bool verbose)
    {
        WbcBase::loadTasksSetting(taskFile, verbose);

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::string prefix = "weight.";
        if (verbose)
        {
            std::cerr << "\n #### WBC weight:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
        loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
        loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
    }

} // namespace legged
