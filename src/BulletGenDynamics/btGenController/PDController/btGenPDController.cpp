#include "btGenPDController.h"
#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include <iostream>

btGenPDController::btGenPDController(btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::PDController, world)
{
    mTargetq.resize(0);
    mTargetqdot.resize(0);
    mExpJointPDControllers.clear();
}

btGenPDController::~btGenPDController()
{
    for (auto &x : mExpJointPDControllers)
        delete x;
    mExpJointPDControllers.clear();
}

void btGenPDController::Init(cRobotModelDynamics *model,
                             const std::string &config)
{
    btGenControllerBase::Init(model, config);

    // 1. parse config
    ParseConfig(config);
}

/**
 * \brief               Set the PD target theta (gen coord)
 * \param q
 */
void btGenPDController::SetPDTargetq(const tVectorXd &q)
{
    mTargetq = q;

    if (mEnableSPD == false)
    {
        // if SPD is disabled, dispatch the target q to each joint
        int num_of_freedom = mModel->GetNumOfFreedom();
        BTGEN_ASSERT(mTargetq.size() == num_of_freedom);

        for (auto pd_ctrl : mExpJointPDControllers)
        {
            auto joint = pd_ctrl->GetJoint();
            pd_ctrl->SetTargetTheta(
                q.segment(joint->GetOffset(), joint->GetNumOfFreedom()));
        }
    }
}

/**
 * \brief               Set the PD target vel (gen vel)
*/
void btGenPDController::SetPDTargetqdot(const tVectorXd &qdot)
{
    mTargetqdot = qdot;

    if (mEnableSPD == false)
    {
        // if SPD is disabled, dispatch the target qdot to each joint's PD Controller
        int num_of_freedom = mModel->GetNumOfFreedom();
        BTGEN_ASSERT(mTargetqdot.size() == num_of_freedom);
        for (auto &pd_ctrl : mExpJointPDControllers)
        {
            auto joint = pd_ctrl->GetJoint();
            pd_ctrl->SetTargetVel(
                qdot.segment(joint->GetOffset(), joint->GetNumOfFreedom()));
        }
    }
}

// /**
//  * \brief               Calculate the generalized control torque and apply it to the robot model
//  */
// void btGenPDController::ApplyGeneralizedTau(double timestep)
// {
//     int dof = mModel->GetNumOfFreedom();
//     if (dof != mTargetq.size() || dof != mTargetqdot.size())
//     {
//         std::cout << "[error] btGenPDController::ApplyGeneralizedTau target q "
//                      "and qdot hasn't been set\n";
//         exit(0);
//     }
//     tVectorXd tau = tVectorXd::Zero(dof);

//     tVectorXd q = mModel->Getq(), qdot = mModel->Getqdot(),
//               qddot = mModel->Getqddot();
//     std::cout << "qdot = " << qdot.transpose() << std::endl;
//     std::cout << "qddot = " << qddot.transpose() << std::endl;
//     for (int i = 0; i < dof; i++)
//     {
//         // std::cout << "qdot " << i << " = " << qdot[i] << ", max vel = " <<
//         // mModel->GetMaxVel() << std::endl; std::cout << "res = " <<
//         // (std::fabs(qdot[i]) - 10) << std::endl;
//         if ((std::fabs(qdot[i])) >= mModel->GetMaxVel() - 5)
//         {
//             std::cout << "qdot " << i << " = " << qdot[i]
//                       << " exceed the maxvel, belongs to joint "
//                       << mModel->GetJointByDofId(i)->GetName() << std::endl;
//         }
//     }
//     if (mEnableSPD == false)
//     {
//         tau = mKp.cwiseProduct(mTargetq - q) +
//               mKd.cwiseProduct(mTargetqdot - qdot);
//     }
//     else
//     {
//         tau = -mKp.cwiseProduct(q + timestep * qdot - mTargetq) -
//               mKd.cwiseProduct(qdot + timestep * qddot - mTargetqdot);
//     }

//     double max_tau = tau.cwiseAbs().maxCoeff();
//     if (max_tau > mTorqueLim)
//     {
//         std::cout << "the computed control torque " << max_tau << " > "
//                   << mTorqueLim << ", clamped!";
//         std::cout << "raw tau = " << tau.transpose() << std::endl;
//         tau = tau.cwiseMax(-mTorqueLim);
//         tau = tau.cwiseMin(mTorqueLim);
//     }

//     // underactuated: the control torque on root joint is prohibeted
//     tau.segment(0, 6).setZero();

//     for (int i = 0; i < dof; i++)
//         mModel->ApplyGeneralizedForce(i, tau[i]);

//     std::cout << "PD controller apply tau = " << tau.transpose() << std::endl;
// }

void btGenPDController::Update(double dt)
{
    printf("pd controller update dt %.5f\n", dt);
    // set target q and target qdot
    // int num_of_freedom = mModel->GetNumOfFreedom();
    // tVectorXd q_tar = tVectorXd::Zero(num_of_freedom),
    //           qdot_tar = tVectorXd::Zero(num_of_freedom);
    // SetPDTargetq(q_tar);
    // SetPDTargetqdot(qdot_tar);

    // calculate the control force
    tEigenArr<btGenPDForce> forces;
    CalculateControlForces(dt, forces);

    // apply the control force
    BTGEN_ASSERT(forces.size() == mModel->GetNumOfJoint());
    for (int i = 0; i < forces.size(); i++)
    {
        auto joint = forces[i].mJoint;
        std::cout << "[pd] apply joint " << joint->GetId()
                  << " force = " << forces[i].mForce.transpose() << std::endl;
        mModel->ApplyJointTorque(joint->GetId(), forces[i].mForce);
    }
}
void btGenPDController::Reset() {}

/**
 * \brief       Parse the config of Gen PD controller
*/
void btGenPDController::ParseConfig(const std::string &string)
{
    Json::Value root;
    btJsonUtil::LoadJson(string, root);
    mEnableSPD = btJsonUtil::ParseAsBool("enable_stable_pd", root);

    // load the PD controlelrs, and check the completeness
    Json::Value joint_pd_controllers =
        btJsonUtil::ParseAsValue("PDControllers", root);
    BTGEN_ASSERT(mModel != nullptr);
    int num_of_joint = this->mModel->GetNumOfJoint();
    BTGEN_ASSERT(joint_pd_controllers.size() == num_of_joint);

    for (auto &single : joint_pd_controllers)
    {
        int joint_id = btJsonUtil::ParseAsInt("ID", single);
        std::string joint_name = btJsonUtil::ParseAsString("Name", single);
        double kp = btJsonUtil::ParseAsDouble("Kp", single),
               kd = btJsonUtil::ParseAsDouble("Kd", single),
               target_theta_init =
                   btJsonUtil::ParseAsDouble("TargetTheta0", single);
        bool use_world_coord = btJsonUtil::ParseAsBool("UseWorldCoord", single);
        Joint *cur_joint =
            dynamic_cast<Joint *>(mModel->GetJointById(joint_id));

        // keep the input order is ascending
        BTGEN_ASSERT(joint_id == mExpJointPDControllers.size());
        auto new_ctrl = new btGenJointPDCtrl(
            cur_joint, kp, kd, cur_joint->GetTorqueLim(), use_world_coord);
        new_ctrl->SetTargetTheta(target_theta_init *
                                 tVectorXd::Ones(new_ctrl->GetCtrlDims()));
        new_ctrl->SetTargetVel(tVectorXd::Zero(new_ctrl->GetCtrlDims()));

        mExpJointPDControllers.push_back(new_ctrl);
        printf("[log] joint %d kp %.1f kd %.1f, torque lim %.1f, world_coord "
               "%s \n",
               joint_id, kp, kd, cur_joint->GetTorqueLim(),
               use_world_coord ? "true" : "false");
    }
}

/**
 * \brief           Calculate the control force by PD control
 * \param dt        time step
 * \param pd_forces forces
*/
void btGenPDController::CalculateControlForces(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    if (mEnableSPD)
        CalculateControlForcesSPD(dt, pd_forces);
    else
        CalculateControlForcesExp(pd_forces);
}

/**
 * \brief           Calcualte control forces by SPD (stable PD)
*/
void btGenPDController::CalculateControlForcesSPD(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    BTGEN_ASSERT(false && "Hasn't been finished");
}

/**
 * \brief           Calculate control forces by normal PD
 * \param 
*/
void btGenPDController::CalculateControlForcesExp(
    tEigenArr<btGenPDForce> &pd_force_array)
{
    pd_force_array.clear();
    btGenPDForce pd_force;
    for (int i = 0; i < mExpJointPDControllers.size(); i++)
    {
        const auto &pd_ctrl = mExpJointPDControllers[i];
        pd_force.mForce = pd_ctrl->CalcControlForce();
        pd_force.mJoint = pd_ctrl->GetJoint();
        pd_force_array.push_back(pd_force);
    }
}