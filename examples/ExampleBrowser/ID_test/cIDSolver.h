#ifndef ID_SOLVER_H_
#define ID_SOLVER_H_
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include "BulletInverseDynamics/IDConfig.hpp"
#include "BulletInverseDynamics/MultiBodyTree.hpp"
#include <map>

#define MAX_FRAME_NUM 10000
class btMultiBody;
class btMultiBodyDynamicsWorld;

struct tForceInfo
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    int mId;
    tVector mPos, mForce;
    tForceInfo()
    {
        mId = -1;
        mPos = mForce = tVector::Zero();
    }
};

class cIDSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    enum eSolvingMode
    {
        VEL = 0,
        POS,
    };
    cIDSolver(btMultiBody *body, btMultiBodyDynamicsWorld *world);
    void ClearID();
    void SetTimestep(double deltaTime);
    void PreSim();
    void PostSim();
    tVector CalcAngMomentum(int frame_id);
    tVector CalcCOM(int frame_id);

private:
    // ID vars
    btMultiBody *mMultibody;
    btMultiBodyDynamicsWorld *mWorld;
    btInverseDynamics::MultiBodyTree *mInverseModel;
    bool mEnableExternalForce;
    bool mEnableExternalTorque;
    bool mEnableAppliedJointTorque;
    bool mEnableSolveID;
    bool mEnableVerifyVel, mEnableVerifyMomentum;
    bool mFloatingBase;
    int mDof;
    int mNumLinks; // including root
    double mCurTimestep;
    std::map<int, int>
        mWorldId2InverseId; // map the world array index to id in inverse dynamics
    std::map<int, int> mInverseId2WorldId; // reverse map above
    int mFrameId;
    eSolvingMode mSolvingMode;

    btInverseDynamicsBullet3::vecx solve_joint_force_bt;

    // ID buffer vars
    std::vector<tForceInfo> mContactForces;
    tEigenArr<tVector> mJointForces; // joint torque(except root)
    tEigenArr<tVector>
        mExternalForces; // for each link, external forces in COM
    tEigenArr<tVector> mExternalTorques; // for each link, external torques
    tEigenArr<tMatrix>
        mLinkRot[MAX_FRAME_NUM]; // local to world rotation mats
    tEigenArr<tVector> mLinkPos[MAX_FRAME_NUM], mLinkVel[MAX_FRAME_NUM],
        mLinkOmega[MAX_FRAME_NUM]; // link COM pos in world frame
    btVector3 *omega_buffer, *vel_buffer;
    //// permanent memory
    tVectorXd mBuffer_q
        [MAX_FRAME_NUM]; // generalized coordinate "q" buffer, storaged for each frame
    tVectorXd
        mBuffer_u[MAX_FRAME_NUM]; // q_dot = u buffer, storaged for each frame
    tVectorXd mBuffer_u_dot[MAX_FRAME_NUM]; //
    std::vector<double> mLinkMass;
    double mTotalMass;

    // tools
    void AddJointForces();
    void AddExternalForces();
    void GetContactForces();
    void RecordMultibodyInfo(tEigenArr<tMatrix> &local_to_world_rot,
                             tEigenArr<tVector> &link_pos_world,
                             tEigenArr<tVector> &link_vel_world,
                             tEigenArr<tVector> &link_omega_world) const;
    void RecordGeneralizedInfo(btInverseDynamicsBullet3::vecx &q,
                               btInverseDynamicsBullet3::vecx &q_dot) const;
    void RecordGeneralizedInfo(tVectorXd &q, tVectorXd &q_dot) const;
    void SolveID();
    void ApplyContactForcesToID();
    void ApplyExternalForcesToID();
    tVectorXd CalculateGeneralizedVel(const tVectorXd &q_before,
                                      const tVectorXd &q_after,
                                      double timestep) const;

    // the conservation of momentums
    void VerifyLinearMomentum();
    void VerifyAngMomentum();
    void VerifyMomentum();
    void VerifyLinkVel();
    void VerifyLinkOmega();
};
#endif