#include "BulletGenDynamics/btGenSolver/ContactSolver.h"
#include "btGenFrameByFrameCalculator.h"
struct btCharContactPt : public btGenContactPointData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btCharContactPt(int c_id);
    void Init(double dt, btPersistentManifold *manifold,
              int contact_id_in_manifold);
    void Init(const tVector &world_pos, btGenRobotCollider *collider);
    bool IsMultibodyInvolved(cRobotModelDynamics *model);
    void CalcCharacterInfo();

    btGenRobotCollider *mCollider;
    tVector mLocalPos;
    tVector mWorldPos;
    tMatrixXd mJac;
    eContactStatus mStatus;
};
