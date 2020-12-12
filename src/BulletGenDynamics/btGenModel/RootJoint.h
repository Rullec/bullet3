//
// Created by Hanke on 2019-02-13.
//

#ifndef ROBOT_ROOTJOINT_H
#define ROBOT_ROOTJOINT_H
#define INF 0x3f3f3f3f
#include "EulerAngleRotationMatrix.h"
#include "Joint.h"
#include "Printer.h"

class RootJoint : public Joint
{

public:
    RootJoint(BaseObjectParams &param, int n_f);

    void InitTerms() override;

    void GetRotations(tMatrix3d &m) override;

    void Tell() override;

    void ComputeLocalTransformFirstDerive() override;

    void ComputeLocalTransformSecondDerive() override;

    void ComputeLocalTransformThirdDerive() override;
};

#endif // ROBOT_ROOTJOINT_H
