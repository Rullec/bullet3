#pragma once
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
class btGenSimbiconState;
namespace Json
{
class Value;
}
/**
 * \brief       2007 SIGGRAPH paper SIMBICON reimplemention, Kangkang Yin. et al
*/
class btGenSimbiconControllerBase : public btGenControllerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenSimbiconControllerBase(btGeneralizeWorld *world);
    virtual ~btGenSimbiconControllerBase();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &conf) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;

protected:
    btGenPDController *mPDController;   // PD controller
    cRobotModelDynamics *mRefTrajModel; // ref traj model
    std::vector<btGenSimbiconState *> mStates;
    int mStateIndex; // current state index
    int mStance; // left stance or right stance? works with the macro BTGEN_LEFT_STANCE and BTGEN_RIGHT_STANCE
    int mRootId; // the id of root joint (usually zero I guess)
    Joint *mStanceFoot,
        *mSwingFoot; // pts to stance foot joint, and swing foot joint
    int mSwingHipIdx, mStanceHipIdx; // the joint id of swing / stance hip

    Joint *mLeftFoot, *mRightFoot; // ptrs to the left & right feet
    int mLeftHipIdx, mRightHipIdx; // the index of left and right hip joints
    bool mEnableDrawHeadingFrame;  // draw heading frame or not
    tVectorXd mInitPose;           // the init character pose
    double mPhi;                   // phi

    tVector3d mD, mV; // d vector and v vector
    void InitStates(const Json::Value &conf);
    void InitStartPose(const Json::Value &conf);
    void InitJointInfo();
    void TransiteToState(int state);
    void UpdateDandV();
    void GetTargetPose();
    void ComputeTorques(double dt, tEigenArr<btGenPDForce> &forces);
    void ApplyTorques(const tEigenArr<btGenPDForce> &forces);
    double GetStanceFootWeightRatio();
    void ComputeHipTorques(tEigenArr<btGenPDForce> &forces);
    void AdvanceInTime();
    void SetFSMStateTo(int state_idx);
    void SetStance(int new_stance);
};