#ifndef ROBOT_LIMITROOTJOINT_H
#define ROBOT_LIMITROOTJOINT_H
#define INF 0x3f3f3f3f
#include "EulerAngelRotationMatrix.h"
#include "Joint.h"
#include "Printer.h"

class LimitRootJoint : public Joint
{

public:
    LimitRootJoint(BaseObjectParams &param, int n_f);

    void InitTerms() override;

    void GetRotations(tMatrix3d &m) override;

    void Tell() override;

    void ComputeTransformFirstDerive() override;

    void ComputeLocalTransformSecondDerive() override;
};

#endif // ROBOT_LIMITROOTJOINT_H
