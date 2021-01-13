#pragma once
#include "Joint.h"

/**
 * \brief           Ball In Socket implemention (swing-twist decomposition)
 * swing part: exponential coordinate (2DOFs)
 * twist part: euler angle
 * 
 * R_local = R_twist * R_swing. first do swing, then do twist
*/
class btGenExpMapRotation;
class BallInSocketJoint : public Joint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    explicit BallInSocketJoint(const BaseObjectJsonParam &param);
    ~BallInSocketJoint();

    // // 1. GetRotations, calculate the current local rotation
    // virtual void GetRotations(tMatrix3d &m) override final;
    // 2. UpdateMatrix, calculate the indepent rotation matrix
    virtual void UpdateMatrix() override final;
    virtual void ComputeLocalTransformFirstDerive() override final;
    virtual void ComputeLocalFirstDeriveMatrix() override;
    virtual void ComputeLocalSecondDeriveMatrix() override;
    virtual void ComputeLocalTransformSecondDerive() override;
    virtual void ComputeLocalThirdDeriveMatrix() override final;
    virtual void ComputeLocalTransformThirdDerive() override final;
    virtual void SetupTwistAxis();
    tMatrix CalcTargetLocalTransform(const tVector3d &tar_q) const;
    tVectorXd
    CalcTargetPoseByTargetLocalTransform(const tQuaternion &orient) const;

protected:
    btGenExpMapRotation *mExpMap;
    enum eTwistAxis
    {
        INVALID_TWIST,
        TWIST_X = 0,
        TWIST_Y,
        TWIST_Z,
    };
    eTwistAxis mTwistAxis;

    int mMapFromDofIdToAxisAngleId[2];
    tMatrix GetTwistRot(double v) const;
    tVector GetTwistAxis() const;
};