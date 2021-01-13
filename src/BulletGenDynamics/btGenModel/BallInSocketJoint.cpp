#include "BallInSocketJoint.h"
#include "EulerAngleRotationMatrix.h"
#include "ExpMapRotMat.h"
#include <iostream>
BallInSocketJoint::BallInSocketJoint(const BaseObjectJsonParam &param)
    : Joint(param)
{
    mExpMap = new btGenExpMapRotation();
    mTwistAxis = INVALID_TWIST;
    mMapFromDofIdToAxisAngleId[0] = 0;
    mMapFromDofIdToAxisAngleId[1] = 0;
}

BallInSocketJoint::~BallInSocketJoint() { delete mExpMap; }
// 2. UpdateMatrix, calculate the indepent rotation matrix
/**
 * \brief       Calculate the component of local rotation matrix 
 * 
 * r_m: size = 3
 * r_m[0] = Rotation of the first 2 dof  (exp map)
 * r_m[1] = Identity
 * r_m[2] = rotation of the last 1 dof twist (euler)
*/
void BallInSocketJoint::UpdateMatrix()
{
    // printf("[log] ball-in-socket joint %s update matrix\n", GetName().c_str());
    BTGEN_ASSERT(r_m.size() == 3);

    BTGEN_ASSERT(freedoms[0].type == eFreedomType::EXPONENTIAL_MAP);
    BTGEN_ASSERT(freedoms[1].type == eFreedomType::EXPONENTIAL_MAP);
    BTGEN_ASSERT(freedoms[2].type == eFreedomType::REVOLUTE);

    // 1. swing: exp map
    {
        tVector3d exp_map_theta =
            freedoms[0].axis * freedoms[0].v + freedoms[1].axis * freedoms[1].v;
        mExpMap->SetAxis(btMathUtil::Expand(exp_map_theta, 0));
        r_m[0] = mExpMap->GetRotation();
        r_m[1].setIdentity();
        // std::cout << "[debug] ball joint update matr, exp map theta = "
        //           << exp_map_theta.transpose() << std::endl;
    }

    // 2. twist : euler angle
    r_m[2] = GetTwistRot(freedoms[2].v);

    auto IsOne = [](double v) -> bool { return (std::fabs(v - 1) < 1e-10); };
    // 3. confirm that each axis must have a 1 component, otherwise the definiation of 2nd & 3rd
    for (int i = 0; i < 3; i++)
    {
        // std::cout << i << " " << freedoms[i].axis.transpose() << std::endl;
        BTGEN_ASSERT(IsOne(freedoms[i].axis.maxCoeff()));
        BTGEN_ASSERT(IsOne(freedoms[i].axis.norm()));
        if (i <= 1)
        {
            int j = 0;
            for (j = 0; j < 3; j++)
            {
                if (IsOne(freedoms[i].axis[j]))
                {
                    mMapFromDofIdToAxisAngleId[i] = j;
                    break;
                }
            }
            BTGEN_ASSERT(j != 3);
        }
    }
}

/**
 * \brief       Calculate the d(L)/dqi, L is the local transform matrix
 * 
 * L = mTwist(q2) * mSwing(q0, q1)
 * dLdq0 = mTwist * dSwing/dq0
 * dLdq1 = mTwist * dSwing/dq1
 * dLdq2 = d(mTwist)/dq2 * Swing
*/
void BallInSocketJoint::ComputeLocalTransformFirstDerive()
{
    ComputeLocalFirstDeriveMatrix();

    // for freedom 0, 1
    for (int i = 0; i < 2; i++)
    {
        int axis_id = btGenExpMapRotation::GetFreedomAxisId(freedoms[i].axis);
        mTq[i] = r_m[2] * mExpMap->GetFirstDeriv(axis_id);
        // std::cout << "[debug] ball in socket mTq" << i << " = " << mTq[i]
        //           << std::endl;
    }

    // for freedom 2
    mTq[2] = r_m_first_deriv[2] * mExpMap->GetRotation();
    // std::cout << "[debug] ball in socket mTq2 = \n" << mTq[2] << std::endl;

    // std::cout << "[debug] ball in socket rot = \n"
    //           << mExpMap->GetRotation() << std::endl;
}

/**
 * \brief       Compute local first derivation matrix (indvidually)
*/
void BallInSocketJoint::ComputeLocalFirstDeriveMatrix()
{
    // the first two freedom's derivative cannot be calculated individually
    r_m_first_deriv[0].setZero();

    r_m_first_deriv[1].setZero();

    // twist freedom
    {
        auto twist_freedom = freedoms[2];

        switch (mTwistAxis)
        {
        case eTwistAxis::TWIST_X:
            xconventionRotation_dx(r_m_first_deriv[2], twist_freedom.v);
            break;
        case eTwistAxis::TWIST_Y:
            yconventionRotation_dy(r_m_first_deriv[2], twist_freedom.v);
            break;
        case eTwistAxis::TWIST_Z:
            zconventionRotation_dz(r_m_first_deriv[2], twist_freedom.v);
            break;
        default:
            BTGEN_ASSERT(false);
            break;
        }
    }
}

/**
 * \brief           Compute local second derivative matrix 
 * 
 * for the first two r_m_second_deriv: all zero, because they are all about the exp map
 * for the last, dzz
*/
void BallInSocketJoint::ComputeLocalSecondDeriveMatrix()
{
    // the first two freedom's derivative cannot be calculated individually
    r_m_second_deriv[0].setZero();
    r_m_second_deriv[1].setZero();

    // the last one is twist revolute
    {
        auto twist_freedom = freedoms[2];

        switch (mTwistAxis)
        {
        case eTwistAxis::TWIST_X:
            xconventionRotation_dxdx(r_m_second_deriv[2], twist_freedom.v);
            break;
        case eTwistAxis::TWIST_Y:
            yconventionRotation_dydy(r_m_second_deriv[2], twist_freedom.v);
            break;
        case eTwistAxis::TWIST_Z:
            zconventionRotation_dzdz(r_m_second_deriv[2], twist_freedom.v);
            break;
        default:
            BTGEN_ASSERT(false);
            break;
        }
    }
}

/**
 * \brief           compute local second derivative matrix
 * 
 * T = twist * swing
 * mT/qiqj
*/
void BallInSocketJoint::ComputeLocalTransformSecondDerive()
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            if (i == 2)
            {
                if (j == 2)
                {
                    // i =j = 2
                    mTqq[i][j] = r_m_second_deriv[2] * mExpMap->GetRotation();
                }
                else
                {
                    // i = 2, j<2
                    // the jth dof is not the ith component in axis angle!
                    mTqq[i][j] =
                        r_m_first_deriv[2] *
                        mExpMap->GetFirstDeriv(mMapFromDofIdToAxisAngleId[j]);
                }
            }
            else
            {
                // i, j < 2
                mTqq[i][j] = r_m[2] * mExpMap->GetSecondDeriv(
                                          mMapFromDofIdToAxisAngleId[i],
                                          mMapFromDofIdToAxisAngleId[j]);
            }
        }
    }
}

/**
 * \brief           Calculate mTqiii 
 * for i=0,1, it's invalid
*/
void BallInSocketJoint::ComputeLocalThirdDeriveMatrix()
{
    r_m_third_deriv[0].setZero();
    r_m_third_deriv[1].setZero();

    // the last one is twist revolute
    {
        auto twist_freedom = freedoms[2];

        switch (mTwistAxis)
        {
        case eTwistAxis::TWIST_X:
            xconventionRotation_dxdxdx(r_m_third_deriv[2], twist_freedom.v);
            break;
        case eTwistAxis::TWIST_Y:
            yconventionRotation_dydydy(r_m_third_deriv[2], twist_freedom.v);
            break;
        case eTwistAxis::TWIST_Z:
            zconventionRotation_dzdzdz(r_m_third_deriv[2], twist_freedom.v);
            break;
        default:
            BTGEN_ASSERT(false);
            break;
        }
    }
}

/**
 * \brief       Calcuate mTqqq
*/
void BallInSocketJoint::ComputeLocalTransformThirdDerive()
{
    for (int i = 0; i < GetNumOfFreedom(); i++)
        for (int j = 0; j <= i; j++)
            for (int k = 0; k <= j; k++)
            {
                // i >= j >= k
                auto &t_dq_dq_dq = mTqqq[i][j][k];
                if (i < 2)
                {
                    // i, j, k are all swing freedoms
                    t_dq_dq_dq.noalias() =
                        r_m[2] *
                        mExpMap->GetThirdDeriv(mMapFromDofIdToAxisAngleId[i],
                                               mMapFromDofIdToAxisAngleId[j],
                                               mMapFromDofIdToAxisAngleId[k]);
                }
                else
                {
                    // i = 2
                    if (j == 2)
                    {
                        // i = j = 2
                        if (k == 2)
                        {
                            // i = j = k = 2
                            t_dq_dq_dq.noalias() =
                                r_m_third_deriv[2] * mExpMap->GetRotation();
                        }
                        else
                        {
                            // i = j = 2, k < 2
                            t_dq_dq_dq.noalias() =
                                r_m_second_deriv[2] *
                                mExpMap->GetFirstDeriv(
                                    mMapFromDofIdToAxisAngleId[k]);
                        }
                    }
                    else
                    {
                        // i=2, j,k!=2
                        t_dq_dq_dq.noalias() =
                            r_m_first_deriv[2] *
                            mExpMap->GetSecondDeriv(
                                mMapFromDofIdToAxisAngleId[j],
                                mMapFromDofIdToAxisAngleId[k]);
                    }
                }
            }
}

void BallInSocketJoint::SetupTwistAxis()
{
    BTGEN_ASSERT(freedoms.size() == 3);
    BTGEN_ASSERT(mTwistAxis == eTwistAxis::INVALID_TWIST);
    for (int i = 0; i < 3; i++)
    {
        if (std::fabs(freedoms[2].axis[i] - 1) < 1e-10)
        {
            mTwistAxis = static_cast<eTwistAxis>(i);
            break;
        }
    }
    printf("[log] joint %s setup twist axis %d\n", GetName().c_str(),
           mTwistAxis);
}
/**
 * \brief               Given the target q (gen coords), calculate the (local) target orientation
 * \param tar_q         generalize coords target q
 * \return              local target orientation of this joint
*/
tMatrix
BallInSocketJoint::CalcTargetLocalTransform(const tVector3d &tar_q) const
{

    // 2. build & set the new axis, get orientaion
    tMatrix swing_rot = btMathUtil::AxisAngleToRotmat(btMathUtil::Expand(
        freedoms[0].axis * tar_q[0] + freedoms[1].axis * tar_q[1], 0));

    tMatrix twist_rot = GetTwistRot(tar_q[2]);

    return twist_rot * swing_rot;
}

/**
 * \brief               Given the local orientation, do the "swing-twist decomposition", 
 *                      and get the rotate vector "q" = [euler_z, aa_y, aa_x]
*/
tVectorXd BallInSocketJoint::CalcTargetPoseByTargetLocalTransform(
    const tQuaternion &orient) const
{
    // from http://allenchou.net/2018/05/game-math-swing-twist-interpolation-sterp/
    tVector3d r = tVector3d(orient.x(), orient.y(), orient.z());
    tVector3d twist_axis = GetTwistAxis().segment(0, 3);

    tVector3d p = r.dot(twist_axis) * twist_axis;
    tQuaternion twist_rot(orient.w(), p.x(), p.y(), p.z());
    twist_rot.normalize();

    // orient = twist * swing
    // swing = orient * twist.conj
    tQuaternion swing_rot = orient * twist_rot.conjugate();

    tVector swing_aa = btMathUtil::QuaternionToAxisAngle(swing_rot);
    tVector twist_aa = btMathUtil::QuaternionToAxisAngle(twist_rot);
    // std::cout << "swing aa = " << swing_aa.transpose() << std::endl;
    // std::cout << "twist aa = " << twist_aa.transpose() << std::endl;

    // confirm the swing aa can be fully represented by
    tMatrixXd swing_basis(3, 2);
    swing_basis.col(0) = this->freedoms[0].axis;
    swing_basis.col(1) = this->freedoms[1].axis;
    // std::cout << "swing basis = \n" << swing_basis << std::endl;
    tMatrixXd gen_inv = (swing_basis.transpose() * swing_basis).inverse() *
                        swing_basis.transpose(); // 2 * 3
    tVectorXd swing_theta = gen_inv * swing_aa.segment(0, 3);

    double twist_theta = twist_aa.segment(0, 3).dot(twist_axis);

    tVectorXd q = tVectorXd::Zero(GetNumOfFreedom());
    q[0] = swing_theta[0];
    q[1] = swing_theta[1];
    q[2] = twist_theta;
    return q;
    // std::cout << "swing theta = " << swing_theta.transpose() << std::endl;
    // std::cout << "twist theta = " << twist_theta << std::endl;
    // exit(0);
}

tMatrix BallInSocketJoint::GetTwistRot(double v) const
{
    tMatrix rot;
    switch (mTwistAxis)
    {
    case eTwistAxis::TWIST_X:
        xconventionTransform(rot, v);
        break;
    case eTwistAxis::TWIST_Y:
        yconventionTransform(rot, v);
        break;
    case eTwistAxis::TWIST_Z:
        zconventionTransform(rot, v);
        break;
    default:
        BTGEN_ASSERT(false);
        break;
    }

    return rot;
}
tVector BallInSocketJoint::GetTwistAxis() const
{
    tVector axis = tVector::Zero();
    axis[mTwistAxis] = 1;
    return axis;
}