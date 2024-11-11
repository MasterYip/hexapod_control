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
#include "legged_reference/gait/MotionPhaseDefinition.h"
#include <qpOASES.hpp>

namespace legged
{


    vector_t WeightedWbcSimple::update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode,
                                       scalar_t period)
    {
        // We reimplement the update function here (replace contactFlag_ calculation)
        // WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

        contactFlagHex_ = hexapod_robot::modeNumber2StanceLeg(mode);
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
        std::cerr << "Stance legs: " << contactFlagHex_[0] << contactFlagHex_[1] << contactFlagHex_[2] << contactFlagHex_[3] << contactFlagHex_[4] << contactFlagHex_[5] << std::endl;
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
        // return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateNoContactMotionTask();
    }

    Task WeightedWbcSimple::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
    {
        return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_;
    }

    Task WeightedWbcSimple::formulateFrictionConeTask()
    {
        matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
        a.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (!contactFlagHex_[i])
            {
                // Non-contact, let lambda (contact force) be 0
                a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
            }
        }
        vector_t b(a.rows());
        b.setZero();

        matrix_t frictionPyramic(5, 3); // clang-format off
        // H-rep of the friction cone
        frictionPyramic << 0, 0, -1,
                        1, 0, -frictionCoeff_,
                        -1, 0, -frictionCoeff_,
                        0, 1, -frictionCoeff_,
                        0,-1, -frictionCoeff_; // clang-format on

        matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
        d.setZero();
        j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (contactFlagHex_[i])
            {
                d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
            }
        }
        vector_t f = Eigen::VectorXd::Zero(d.rows());

        return {a, b, d, f};
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

    Task WeightedWbcSimple::formulateSwingLegTask()
    {
        eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
        std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
        std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
        eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
        std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
        std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

        matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (!contactFlagHex_[i])
            {
                vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
                b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
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
