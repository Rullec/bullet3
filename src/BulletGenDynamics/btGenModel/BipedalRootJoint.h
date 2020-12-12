#ifndef ROBOT_BIPEDAL_ROOTJOINT_H
#define ROBOT_BIPEDAL_ROOTJOINT_H
#define INF 0x3f3f3f3f
#include "EulerAngleRotationMatrix.h"
#include "Joint.h"
#include "Printer.h"

/**
 * \brief           Bipedal root joint, has only X rotation and YOZ translation, 3dof
 * DOF: 3
 * Rotation: X
 * Tranlsation: Y, Z
*/
class BipedalRootJoint : public Joint
{
public:
    BipedalRootJoint(BaseObjectParams &param, int n_f);
    void InitTerms() override final;
    void GetRotations(tMatrix3d &m) override final;
    void Tell() override final;
    void ComputeLocalTransformFirstDerive() override final;
    void ComputeLocalTransformSecondDerive() override final;
    void ComputeLocalTransformThirdDerive() override final;
};

#endif // ROBOT_BIPEDAL_ROOTJOINT_H