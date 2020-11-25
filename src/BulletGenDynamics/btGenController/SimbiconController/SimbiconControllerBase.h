#pragma once
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
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
    btGenSimbiconControllerBase(btGeneralizeWorld *world,
                                ebtGenControllerType type);
    virtual ~btGenSimbiconControllerBase();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &conf) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;

protected:
    btGenFSM *mFSM; // finite state machine
    btGenPDController *mPDController;
    cRobotModelDynamics *mRefTrajModel;
    int mRootId, mSwingHip,
        mStanceHip; // the link/joint id of root, swing hip and stance hip
    std::string mStanceName, mSwingName;
    bool
        mIgnoreBalanceControlInState02; // ignore balance control in state 0 and state 2 (stable state)
    bool
        mEnableStanceControlRatio; // calculate the stance control force by stance/swing foot vertical contact force ratio
    void BuildFSM(const Json::Value &conf);
    void BuildPDCtrl(const std::string &pd_path);
    virtual void BuildBalanceCtrl(const Json::Value &conf) = 0;
    virtual void BalanceUpdateTargetPose(tVectorXd &target_pose) const = 0;
    void UpdateSwingStance();
    virtual void UpdatePDController(const tVectorXd &tar_pose);
    void CalculateControlForce(double dt, tEigenArr<btGenPDForce> &forces);
    double ComputeStanceSwingRatio() const;
    int GetEndeffector(int id) const;
    bool IsFallDown() const;
    void UpdateRefModel(const tVectorXd &tar_pose);
    int GetJointByPartialName(const std::string &name);
};