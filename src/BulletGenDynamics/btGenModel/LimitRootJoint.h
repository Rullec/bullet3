#ifndef ROBOT_LIMITROOTJOINT_H
#define ROBOT_LIMITROOTJOINT_H
#define INF 0x3f3f3f3f
#include "EulerAngleRotationMatrix.h"
#include "Joint.h"
#include "Printer.h"

/**
 * \brief       Limit Root joint, has only one X translation freedom, with no rotation freedom
 * DOF: 1
 * Rotation: None
 * Translation: X
*/
class LimitRootJoint : public Joint
{

public:
    LimitRootJoint(BaseObjectParams &param, int n_f);

    void InitTerms() override;

    void GetRotations(tMatrix3d &m) override;

    void Tell() override;

    void ComputeLocalTransformFirstDerive() override;

    void ComputeLocalTransformSecondDerive() override;
};

#endif // ROBOT_LIMITROOTJOINT_H
