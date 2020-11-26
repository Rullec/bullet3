#ifndef ROBOT_FIXED_ROOTJOINT_H
#define ROBOT_FIXED_ROOTJOINT_H
#define INF 0x3f3f3f3f
#include "EulerAngelRotationMatrix.h"
#include "Joint.h"
#include "Printer.h"

/**
 * \brief           FIXED root joint, has no freedom
 * DOF: 0
*/
class FixedRootJoint : public Joint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    FixedRootJoint(BaseObjectParams &param, int n_f);
    void InitTerms() override final;
    void GetRotations(tMatrix3d &m) override final;
    void Tell() override final;
    void ComputeLocalTransformFirstDerive() override final;
    void ComputeLocalTransformSecondDerive() override final;
    void ComputeLocalTransformThirdDerive() override final;

    void SetFixedRootJointWorldTrans(const tMatrix &trans);
    tMatrix GetFixedRootJointWorldTrans() const;

protected:
    tMatrix
        mFixedRootJointWorldTrans; // the trasform from fixed root joint frame, to world frame
};

#endif // ROBOT_FIXED_ROOTJOINT_H