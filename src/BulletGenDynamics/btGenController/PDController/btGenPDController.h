#pragma once
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class cRobotModelDynamics;
class btGeneralizeWorld;
class Joint;
class btGenJointPDCtrl;
struct btGenPDForce
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    tVector mForce; // cartesian force (torque) on this joint
    Joint *mJoint;  // joint pointer
};

class btGenPDController : public btGenControllerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenPDController(btGeneralizeWorld *world);
    virtual ~btGenPDController();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &config) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;
    void SetPDTargetq(const tVectorXd &q);
    void SetPDTargetqdot(const tVectorXd &qdot);
    virtual void CalculateControlForces(double dt,
                                        tEigenArr<btGenPDForce> &pd_forces);
    std::vector<btGenJointPDCtrl *> &GetJointPDCtrls();

protected:
    tVectorXd mTargetq,
        mTargetqdot; // target q and target qdot, set from outside

    bool mEnableSPD;         // enable stable pd control or not
    bool mEnablePDForceTest; // enable debug verification
    std::vector<btGenJointPDCtrl *> mExpJointPDControllers;
    void ParseConfig(const std::string &string);
    tVectorXd CalcTargetPose(const tVectorXd &pose) const;
    tVectorXd CalcTargetVel(const tVectorXd &vel) const;
    void CalculateControlForcesSPD(double dt, const tVectorXd &target_q,
                                   const tVectorXd &target_qdot,
                                   tEigenArr<btGenPDForce> &pd_forces);
    void CalculateControlForcesExp(const tVectorXd &target_q,
                                   const tVectorXd target_qdot,
                                   tEigenArr<btGenPDForce> &pd_forces);
    void TestPDController(double dt);
    void TestPDControllerBuildPose(double dt);
    void TestPDControllerKpForce(double dt);
    void TestPDControllerKdForce(double dt);
};