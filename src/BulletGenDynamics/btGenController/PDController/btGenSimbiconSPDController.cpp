#include "btGenSimbiconSPDController.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include <iostream>

btGenSimbiconSPDController::btGenSimbiconSPDController(btGeneralizeWorld *world)
    : btGenPDController(world)
{
    mCtrlType = ebtGenControllerType::SimbiconSPDController;
}

btGenSimbiconSPDController::~btGenSimbiconSPDController() {}
void btGenSimbiconSPDController::Init(cRobotModelDynamics *model,
                                      const std::string &config)

{
    btGenPDController::Init(model, config);
    std::cout << "simbicon SPD inited\n";
}
void btGenSimbiconSPDController::Update(double dt)
{
    BTGEN_ASSERT(false &&
                 "btGenSimbiconSPDController Update should not be called\n");
}
void btGenSimbiconSPDController::Reset() {}

#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
void btGenSimbiconSPDController::CalculateControlForces(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    // 1. add the virtual PD control the the body (torso), calculate the control torque (only the torque)
    int num_of_joint = mModel->GetNumOfJoint();
    BTGEN_ASSERT(num_of_joint == mExpJointPDControllers.size());
    auto root_ctrl = mExpJointPDControllers[0];
    Joint *root_joint = root_ctrl->GetJoint();
    BTGEN_ASSERT(root_joint->GetId() == 0);

    mTargetqCur = mTargetqSet;
    mTargetqdotCur = mTargetqdotSet;
    BuildTargetPose(mTargetqCur);
    BuildTargetVel(mTargetqdotCur);
    std::cout << "[log] target q and qdot should be builded before, changed in "
                 "each frame q = "
              << mTargetqCur.transpose() << std::endl;
    tVector root_force =
        root_ctrl->CalcControlForce(mTargetqCur, mTargetqdotCur);
    std::cout << "root force = " << root_force.transpose() << std::endl;

    // 2. make sure the torso's true applied torque is this value, which means \tau_torso = \tau_swing + \tau_stance

    /*
        
    */
}