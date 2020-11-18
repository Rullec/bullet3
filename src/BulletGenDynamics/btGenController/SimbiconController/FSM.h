#include "BulletGenDynamics/btGenUtil/MathUtil.h"
class btGeneralizeWorld;
class cRobotModelDynamics;
class tState;
namespace Json
{
class Value;
}
class btTraj;

#define BTGEN_LEFT_STANCE 0
#define BTGEN_RIGHT_STANCE 1
/**
 * \brief       Finite state machine used in SIMBICON controller
*/
class btGenFSM
{
public:
    btGenFSM(btGeneralizeWorld *world, cRobotModelDynamics *model,
             const Json::Value &config);
    ~btGenFSM();
    void Init(int &default_stance);
    void Update(double dt, tVectorXd &target_pose, int &stance);
    tVectorXd GetTargetPose();
    tState *GetCurrentState();

protected:
    btGeneralizeWorld *mWorld;
    cRobotModelDynamics *mModel;
    tState *mCurState;
    btTraj *mStateTraj;
    std::vector<tState *> mStateGraph;
};