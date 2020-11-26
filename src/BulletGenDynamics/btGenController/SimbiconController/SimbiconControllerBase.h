#pragma once
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
class btGenFSM;
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
    btGenSimbiconControllerBase(btGeneralizeWorld *world);
    virtual ~btGenSimbiconControllerBase();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &conf) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;

protected:
    btGenFSM *mFSM; // finite state machine
    btGenPDController *mPDController;
    cRobotModelDynamics *mRefTrajModel;

    int mStance; // left stance or right stance? works with the macro BTGEN_LEFT_STANCE and BTGEN_RIGHT_STANCE
    int mRootId;
    typedef std::pair<int, std::string> JointInfo; // joint id - name pair
    JointInfo mSwingHipInfo, mStanceHipInfo, mSwingFootInfo,
        mStanceFootInfo; // swing/stance hip info

    JointInfo mLeftHipInfo, mRightHipInfo, mLeftFootInfo,
        mRightFootInfo; // left and right hip/foot info

    bool
        mIgnoreBalanceControlInState02; // ignore balance control in state 0 and state 2 (stable state)
    bool
        mEnableStanceControlRatio; // calculate the stance control force by stance/swing foot vertical contact force ratio

    double mCd_forward, mCv_forward; // balance control param, forward
    double mCd_tangent, mCv_tangent; // balance control param, tangent

    void BuildFSM(const Json::Value &conf);
    void BuildPDCtrl(const std::string &pd_path);
    void BuildBalanceCtrl(const Json::Value &conf);
    void BuildJointInfo();
    void CalcTargetPose(tVectorXd &target_pose) const;
    void CalcTargetPoseRevoluteHips(tVectorXd &target_pose) const;
    void CalcTargetPoseSphericalHips(tVectorXd &target_pose) const;
    void CalcControlForce(double dt, tEigenArr<btGenPDForce> &forces);
    double CalcStanceSwingRatio() const;

    void UpdateStance();
    void UpdatePDController(const tVectorXd &tar_pose);
    void UpdateRefModel(const tVectorXd &tar_pose);
    int GetEndeffector(int id) const;
    bool IsFallDown() const;
    int GetJointByPartialName(const std::string &name);
};