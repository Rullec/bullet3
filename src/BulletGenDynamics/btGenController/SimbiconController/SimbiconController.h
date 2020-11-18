#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class btGenFSM;
class btGenPDController;
namespace Json
{
class Value;
}
/**
 * \brief       2007 SIGGRAPH paper SIMBICON reimplemention, Kangkang Yin. et al
*/
class btGenSimbiconController : public btGenControllerBase
{
public:
    btGenSimbiconController(btGeneralizeWorld *world);
    ~btGenSimbiconController();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &conf) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;

protected:
    btGenFSM *mFSM; // finite state machine
    btGenPDController *mPDController;
    cRobotModelDynamics *mRefTrajModel;
    int mRootId, mSwingHip,
        mStanceHip;  // the link/joint id of root, swing hip and stance hip
    double mCd, mCv; // balance control param
    void BuildFSM(const Json::Value &conf);
    void BuildPDCtrl(const std::string &pd_path);
    void BuildBalanceCtrl(const Json::Value &conf);
    void BalanceUpdateTargetPose(tVectorXd &target_pose) const;
    void UpdateSwingStance();
    void UpdatePDController(const tVectorXd &tar_pose);
    int GetEndeffector(int id) const;
    void RestrictRootPose();
    bool IsFallDown() const;
    void UpdateRefModel();
};