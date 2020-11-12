#include "BulletGenDynamics/btGenUtil/MathUtil.h"
class btGeneralizeWorld;
class cRobotModelDynamics;
class tState;
namespace Json
{
class Value;
}
class btTraj;

/**
 * \brief       Finite state machine used in SIMBICON controller
*/
class btGenFSM
{
public:
    btGenFSM(btGeneralizeWorld *world, cRobotModelDynamics *model,
             const Json::Value &config);
    ~btGenFSM();
    void InitPose();
    void Update(double dt, tVectorXd &target_pose);
    tVectorXd GetTargetPose();
    tState *GetCurrentState();

protected:
    btGeneralizeWorld *mWorld;
    cRobotModelDynamics *mModel;
    tState *mCurState;
    btTraj *mStateTraj;
    std::vector<tState *> mStateGraph;
};