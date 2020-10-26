#include "btGenPDController.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include <iostream>
btGenPDController::btGenPDController(const std::string &config)
{
    Json::Value root;
    btJsonUtil::LoadJson(config, root);
    mEnableSPD = btJsonUtil::ParseAsBool("enable_stable_pd", root);
    mTorqueLim = btJsonUtil::ParseAsDouble("torque_lim", root);
    mKpConstant = btJsonUtil::ParseAsDouble("kp", root);
    mKdConstant = btJsonUtil::ParseAsDouble("kd", root);
    mTargetq.resize(0);
    mTargetqdot.resize(0);
    mKp.resize(0);
    mKd.resize(0);
}

btGenPDController::~btGenPDController() {}

bool btGenPDController::Init(cRobotModelDynamics *model)
{
    mModel = model;
    int dofs = model->GetNumOfFreedom();
    mKp.resize(dofs);
    mKp.fill(mKpConstant);
    mKd.resize(dofs);
    mKd.fill(mKdConstant);

    return true;
}

/**
 * \brief               Set the PD target for the q
 * \param q
 */
void btGenPDController::SetPDTargetq(const tVectorXd &q) { mTargetq = q; }
void btGenPDController::SetPDTargetqdot(const tVectorXd &qdot)
{
    mTargetqdot = qdot;
}

/**
 * \brief               Calculate the generalized control torque and apply it to
 * the robot model
 */
void btGenPDController::ApplyGeneralizedTau(double timestep)
{
    int dof = mModel->GetNumOfFreedom();
    if (dof != mTargetq.size() || dof != mTargetqdot.size())
    {
        std::cout << "[error] btGenPDController::ApplyGeneralizedTau target q "
                     "and qdot hasn't been set\n";
        exit(0);
    }
    tVectorXd tau = tVectorXd::Zero(dof);

    tVectorXd q = mModel->Getq(), qdot = mModel->Getqdot(),
              qddot = mModel->Getqddot();
    std::cout << "qdot = " << qdot.transpose() << std::endl;
    std::cout << "qddot = " << qddot.transpose() << std::endl;
    for (int i = 0; i < dof; i++)
    {
        // std::cout << "qdot " << i << " = " << qdot[i] << ", max vel = " <<
        // mModel->GetMaxVel() << std::endl; std::cout << "res = " <<
        // (std::fabs(qdot[i]) - 10) << std::endl;
        if ((std::fabs(qdot[i])) >= mModel->GetMaxVel() - 5)
        {
            std::cout << "qdot " << i << " = " << qdot[i]
                      << " exceed the maxvel, belongs to joint "
                      << mModel->GetJointByDofId(i)->GetName() << std::endl;
        }
    }
    if (mEnableSPD == false)
    {
        tau = mKp.cwiseProduct(mTargetq - q) +
              mKd.cwiseProduct(mTargetqdot - qdot);
    }
    else
    {
        tau = -mKp.cwiseProduct(q + timestep * qdot - mTargetq) -
              mKd.cwiseProduct(qdot + timestep * qddot - mTargetqdot);
    }

    double max_tau = tau.cwiseAbs().maxCoeff();
    if (max_tau > mTorqueLim)
    {
        std::cout << "the computed control torque " << max_tau << " > "
                  << mTorqueLim << ", clamped!";
        std::cout << "raw tau = " << tau.transpose() << std::endl;
        tau = tau.cwiseMax(-mTorqueLim);
        tau = tau.cwiseMin(mTorqueLim);
    }

    // underactuated: the control torque on root joint is prohibeted
    tau.segment(0, 6).setZero();

    for (int i = 0; i < dof; i++)
        mModel->ApplyGeneralizedForce(i, tau[i]);

    std::cout << "PD controller apply tau = " << tau.transpose() << std::endl;
}