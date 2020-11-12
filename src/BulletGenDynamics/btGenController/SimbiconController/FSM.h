#include "BulletGenDynamics/btGenUtil/MathUtil.h"
class btGeneralizeWorld;
class cRobotModelDynamics;
class tState;
namespace Json
{
class Value;
}

/**
 * \brief       Finite state machine used in SIMBICON controller
*/
class btGenFSM
{
public:
    btGenFSM(btGeneralizeWorld *world, cRobotModelDynamics *model,
             const Json::Value &config);
    ~btGenFSM();
    void Update(double dt, tVectorXd &target_pose);

protected:
    btGeneralizeWorld *mWorld;
    cRobotModelDynamics *mModel;
    tState *mCurState;
    std::vector<tState *> mStateGraph;
};