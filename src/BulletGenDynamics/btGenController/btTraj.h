#include "BulletGenDynamics/btGenSolver/ContactSolver.h"
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
    double GetTimeLength() const;
    tVectorXd GetGenContactForce(int frame_id, cRobotModelDynamics *model);
    tVectorXd GetGenContactForceNoSet(int frame_id, cRobotModelDynamics *model);
    tMatrixXd GetGenContactJacobianNoSet(int frame_id,
                                         cRobotModelDynamics *model);
    void GetGen_dContactJacobian_dq_NoSet(int frame_id,
                                          cRobotModelDynamics *model,
                                          tEigenArr<tMatrixXd> &dJacdq);
    tVectorXd GetGenControlForce(int frame_id, cRobotModelDynamics *model);
    void CalculateLocalContactPos(cRobotModelDynamics *model);

    int mNumOfFrames;
    tEigenArr<tVectorXd> mq, mqdot, mqddot;
    tEigenArr<tVectorXd> mTruthJointForceVec;
    tEigenArr<tVectorXd> mAction;
    std::vector<tEigenArr<tVector>> mTruthJointForce;
    std::vector<std::vector<btGenContactForce *>> mContactForce;
    double mTimestep;

protected:
    tEigenArr<tEigenArr<tVector>> mContactLocalPos;
    std::string mTrajPath;
    void CheckFrameId(int frame_id, std::string prefix);
    void CheckModelState(int frame_id, cRobotModelDynamics *model,
                         std::string prefix);
};