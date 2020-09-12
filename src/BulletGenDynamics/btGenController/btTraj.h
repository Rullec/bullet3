#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class btGenContactForce;
class cRobotModelDynamics;
struct btTraj
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btTraj();
    ~btTraj();
    bool LoadTraj(const std::string &path, cRobotModelDynamics *model,
                  int max_frame = -1);
    bool SaveTraj(const std::string &path, cRobotModelDynamics *model);
    void Reshape(int num_of_frame_new);
    int mNumOfFrames;
    tEigenArr<tVectorXd> mq, mqdot, mqddot;
    tEigenArr<tVectorXd> mActiveForce;
    std::vector<std::vector<btGenContactForce *>> mContactForce;
    double mTimestep;

protected:
    std::string mTrajPath;
};