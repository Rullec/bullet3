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
    void CalculateControlForces(double dt, tEigenArr<btGenPDForce> &pd_forces);

protected:
    tVectorXd mTargetqCur,
        mTargetqdotCur; // current target q and target qdot (gen coordinate and gen vel), time variant
    tVectorXd mTargetqSet,
        mTargetqdotSet; // setted target q and target qdot (gen coordinate and gen vel), fixed if not being set again

    bool mEnableSPD; // enable stable pd control or not
    std::vector<btGenJointPDCtrl *> mExpJointPDControllers;
    void ParseConfig(const std::string &string);
    void BuildTargetPose(tVectorXd &pose);
    void BuildTargetVel(tVectorXd &vel);
    void CalculateControlForcesSPD(double dt,
                                   tEigenArr<btGenPDForce> &pd_forces);
    void CalculateControlForcesExp(tEigenArr<btGenPDForce> &pd_forces);
};