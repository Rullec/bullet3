#define LEFT_STANCE 0
#define RIGHT_STANCE 1

#define LEFT_STANCE_STR ("left_stance")
#define RIGHT_STANCE_STR ("right_stance")
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

namespace Json
{
class Value;
};

class cRobotModelDynamics;

class Joint;
class btGenSimbiconControllerBase;

struct BaseTrajectory
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    BaseTrajectory(const Json::Value &conf);
    ~BaseTrajectory();

    struct tFeedBack
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        tFeedBack(const tVector3d &proj_axis, double cd, double cv);
        tVector3d mFeedbackProjAxis;
        double Cd, Cv;
    };
    tVector3d mRotationAxis;
    tVectorXd mTimeKnots, mValueKnots;
    tFeedBack *mBalanceFeedback;
    int mLeftStanceIndex, mRightStanceIndex;
};

/**
 * \brief           Simbicon joint trajectory in a state
*/
class btGenSimbiconTraj
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenSimbiconTraj(const Json::Value &conf, cRobotModelDynamics *model);
    virtual ~btGenSimbiconTraj();
    int getJointIndex(int stance);
    tQuaternion evaluateTrajectory(btGenSimbiconControllerBase *ctrl,
                                   Joint *joint, int stance, double phi,
                                   const tVector3d &d,
                                   const tVector3d &v) const;
protected:
    std::string mJointName;

    std::vector<BaseTrajectory *> mBaseTrajs;
};
