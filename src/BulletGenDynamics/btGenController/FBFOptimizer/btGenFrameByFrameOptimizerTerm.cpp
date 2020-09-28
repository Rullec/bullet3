#include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenWorld.h"
#include "btCharContactPoint.h"
#include "btGenFBFConstraint.h"
#include "btGenFBFEnergyTerm.h"
#include "btGenFrameByFrameOptimizer.h"
extern int mNumOfFrictionDirs;
extern double mu;
/**
 * \brief					Construct the energy terms in
 * this opt problem
 *
 *  Energy terms:
 * 	1. dynamics terms
 *  2. min control force
 *  3. min contact forces norm
 */
void btGenFrameByFrameOptimizer::CalcEnergyTerms()
{
    if (mEnergyTerm == nullptr)
        std::cout << "[error] energy term hasn't been inited\n", exit(0);
    mEnergyTerm->Reset(mTotalSolutionSize);

    AddDynamicEnergyTerm();

    if (mControlForceCoef > 0)
        AddMinTauEnergyTerm();
    if (mContactForceCoef > 0)
        AddMinContactForceEnergyTerm();
    // if (mControlForceCloseToOriginCoef > 0)
    //     AddTauCloseToOriginEnergyTerm();
    // if (mContactForceCloseToOriginCoef > 0)
    //     AddContactForceCloseToOriginEnergyTerm();

    if (mEndEffectorPosCoef > 0)
        AddEndEffectorPosEnergyTerm();
    if (mEndEffectorOrientationCoef > 0)
        AddEndEffectorOrientationEnergyTerm();
    if (mEndEffectorVelCoef > 0)
        AddEndEffectorVelEnergyTerm();
    if (mRootPosCoef > 0)
        AddRootPosEnergyTerm();
    if (mRootVelCoef > 0)
        AddRootVelEnergyTerm();
    if (mRootOrientationCoef > 0)
        AddRootOrientationEnergyTerm();
    if (mEnableTrackRefContact == true)
        AddTrackRefContactEnergyTerm();
}

/**
 * \brief					Construct the constraints
 */
void btGenFrameByFrameOptimizer::CalcConstraints()
{
    if (nullptr != mConstraint)
        delete mConstraint;
    mConstraint = new btGenFrameByFrameConstraint(mTotalSolutionSize);

    // if (mEnableHardConstraintForDynamics == true)
    // AddDynamicConstraint();
    // for (int i = 0; i < mContactPoints.size(); i++)
    for (auto &pt : mContactPoints)
    {
        switch (pt->mStatus)
        {
        case eContactStatus::SLIDING:
            AddSlidingConstraint(pt);
            break;
        case eContactStatus::STATIC:
            AddStaticConstraint(pt);
            break;
        case eContactStatus::BREAKAGE:
            AddBreakageConstraint(pt);
            break;
        default:
            std::cout << "[error] illegal contact status " << pt->mStatus
                      << std::endl;

            exit(0);
            break;
        }
    }

    // add hard constraint for static contact point

    // maybe we still needs to add constraint on the contact position.
    // do we need to fix the contact point at somewhere when it is judge static?

    if (mEnableFixStaticContactPoint == true)
        AddFixStaticContactPointConstraint();

    // add the limitattion for contact forces
    if (mEnableContactForceLimit == true)
        AddContactForceLimitConstraint();

    // add the non-penetration hard constraint on each contact pointI
    if (mEnableContactNonPenetrationConstraint == true)
        AddNonPenetrationContactConstraint();
}

/**
 * \brief				Add sliding contact point constraint
 */
void btGenFrameByFrameOptimizer::AddSlidingConstraint(btCharContactPt *pt)
{
    int contact_id = pt->contact_id;
    int offset = mContactSolOffset[contact_id];
    int size = mContactSolSize[contact_id];
    // 1. ineqaulty make fn, ft >=0
    {
        tMatrixXd jac = tMatrixXd::Identity(size, size);
        tVectorXd res = tVectorXd::Zero(size);
        mConstraint->AddIneqCon(jac, res, offset,
                                "sliding_force_point_" +
                                    std::to_string(pt->contact_id));
    }
    // 2. equalities make ft = mu * fn
    {
        tMatrixXd jac = tMatrixXd::Zero(1, size);
        jac(0, 0) = mu;
        jac.block(0, 1, 1, size - 1).fill(-1);
        mConstraint->AddEqCon(jac, tVectorXd::Zero(1), offset,
                              "sliding_force_point_" +
                                  std::to_string(pt->contact_id));
    }
}

/**
 * \brief               Given one contact point info, add the "static" contact
 * constraint into the opt problem
 */
void btGenFrameByFrameOptimizer::AddStaticConstraint(btCharContactPt *pt)
{
    // add static constraint on the contact force
    int static_size = mContactSolSize[pt->contact_id];
    int offset = mContactSolOffset[pt->contact_id];
    tMatrixXd jac = tMatrixXd::Zero(static_size + 1, static_size);
    tVectorXd residual = tVectorXd::Zero(static_size + 1);

    jac.block(0, 0, static_size, static_size).setIdentity();

    jac(static_size, 0) = mu;
    jac.block(static_size, 1, 1, mNumOfFrictionDirs).fill(-1);
    mConstraint->AddIneqCon(jac, residual, offset,
                            "static_force_point_" +
                                std::to_string(pt->contact_id));
}

/**
 * \brief               Given one contact point info, add the "breakage" contact
 * constraint into the opt problem
 */
void btGenFrameByFrameOptimizer::AddBreakageConstraint(btCharContactPt *pt)
{
    int offset = mContactSolOffset[pt->contact_id];
    tMatrixXd jac = tMatrix3d::Identity();
    mConstraint->AddEqCon(jac, tVector3d::Zero(), offset,
                          "breakage_force_point_" +
                              std::to_string(pt->contact_id));
}

/**
 * \brief				Close to Origin energy term in this
 * frame by frame control q, qdot, qddot
 */
void btGenFrameByFrameOptimizer::AddDynamicEnergyTerm()
{
    if (mDynamicPosEnergyCoeff > 0)
        AddDynamicEnergyTermPos();
    if (mDynamicVelEnergyCoeff > 0)
        AddDynamicEnergyTermVel();
    if (mDynamicAccelEnergyCoeff > 0)
        AddDynamicEnergyTermAccel();
    if (mDynamicMinAccelEnergyCoeff > 0)
        AddDynamicEnergyTermMinAccel();
}

/**
 * \brief               Add the hard constraint of lagragian equation in order
 * to keep the physical-feasible property
 */
void btGenFrameByFrameOptimizer::AddDynamicConstraint()
{
    // int num_of_freedom = mModel->GetNumOfFreedom();
    // int num_of_underactuated_freedom = num_of_freedom - 6;
    tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    tVectorXd b = tVectorXd::Zero(num_of_freedom);

    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &C_d = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &q = mModel->Getq();
    const tVectorXd &q_next_ref = mTraj->mq[mRefFrameId + 1];
    double dt2 = mdt * mdt;
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    b = dt2 * Minv * (QG - C_d * qdot) + q + mdt * qdot - q_next_ref;
    tMatrixXd A1 = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize),
              A2 =
                  tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    {
        auto pt = mContactPoints[c_id];
        int size = mContactSolSize[pt->contact_id];
        int offset = mContactSolOffset[pt->contact_id];

        // (N * 3) * (3 * size) = N * size
        A1.block(0, offset, num_of_freedom, size).noalias() =
            dt2 * Minv * pt->mJac.transpose() * pt->mS;
    }

    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    A2.noalias() = dt2 * Minv * N;
    A.block(0, 0, num_of_freedom, mContactSolutionSize).noalias() = A1;
    A.block(0, mContactSolutionSize, num_of_freedom,
            num_of_underactuated_freedom)
        .noalias() = A2;
    mConstraint->AddEquivalentEqCon(A, b, 0, 1e-12, "dynamic_hard_constraint");
}
void btGenFrameByFrameOptimizer::AddMinTauEnergyTerm()
{
    // int num_of_underactuated_freedom = mModel->GetNumOfFreedom() - 6;
    tMatrixXd A = tMatrixXd::Identity(num_of_underactuated_freedom,
                                      num_of_underactuated_freedom);
    tVectorXd b = tVectorXd::Zero(num_of_underactuated_freedom);
    mEnergyTerm->AddEnergy(A, b, mControlForceCoef, mContactSolutionSize,
                           "control_force");
}

/**
 * \brief           Minimize the contact force value (in solved contact forces)
 */
void btGenFrameByFrameOptimizer::AddMinContactForceEnergyTerm()
{
    if (mContactSolutionSize == 0)
        return;
    tMatrixXd A =
        tMatrixXd::Identity(mContactSolutionSize, mContactSolutionSize);
    tVectorXd b = tVectorXd::Zero(mContactSolutionSize);
    mEnergyTerm->AddEnergy(A, b, mContactForceCoef, 0, "contact_force");
}

/**
 * \brief               Add the dynamic energy term which reduce the error of
 * gen pos w.r.t ref traj in next frame the optimizaiton vec is contact force
 * and control forces
 */
void btGenFrameByFrameOptimizer::AddDynamicEnergyTermPos()
{
    tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    tVectorXd b = tVectorXd::Zero(num_of_freedom);

    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &C_d = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &q = mModel->Getq();
    const tVectorXd &q_next_ref = mTraj->mq[mRefFrameId + 1];
    double dt2 = mdt * mdt;
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    b = dt2 * Minv * (QG - C_d * qdot) + q + mdt * qdot - q_next_ref;
    {
        tMatrixXd A1 = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize),
                  A2 = tMatrixXd::Zero(num_of_freedom,
                                       num_of_underactuated_freedom);

        // A1
        for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
        {
            auto pt = mContactPoints[c_id];
            int size = mContactSolSize[pt->contact_id];
            int offset = mContactSolOffset[pt->contact_id];

            // (N * 3) * (3 * size) = N * size
            A1.block(0, offset, num_of_freedom, size).noalias() =
                dt2 * Minv * pt->mJac.transpose() * pt->mS;
        }

        // A2
        tMatrixXd N =
            tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
        N.block(6, 0, num_of_underactuated_freedom,
                num_of_underactuated_freedom)
            .setIdentity();
        A2.noalias() = dt2 * Minv * N;

        // put A1 A2 in it
        A.block(0, 0, num_of_freedom, mContactSolutionSize).noalias() = A1;
        A.block(0, mContactSolutionSize, num_of_freedom,
                num_of_underactuated_freedom)
            .noalias() = A2;
        // std::cout << "[pos] A1 norm = " << A1.norm() << std::endl;
        // std::cout << "[pos] A2 norm = " << A2.norm() << std::endl;
        // std::cout << "[pos] Apre norm = " << A.norm() << std::endl;
    }

    // if (mIgnoreRootPosInDynamicEnergy == true)
    // {
    //     A = A.block(6, 0, num_of_underactuated_freedom, mTotalSolutionSize)
    //             .eval();
    //     b = b.segment(6, num_of_underactuated_freedom).eval();
    // }
    // energy term Ax + b
    double coef = mDynamicPosEnergyCoeff / dt2;
    // std::cout << "[pos] A norm = " << A.norm() << std::endl;
    // std::cout << "[pos] b norm = " << b.norm() << std::endl;
    // std::cout << "[pos] q norm = " << q.norm() << std::endl;
    // std::cout << "[pos] qdot norm = " << qdot.norm() << std::endl;
    // std::cout << "[pos] q_ref norm = " << q_next_ref.norm() << std::endl;
    mEnergyTerm->AddEnergy(A, b, coef, 0, "pos");
}

/**
 * \brief               Add the dynamic energy term which reduce the error of
 * gen vel w.r.t ref traj in next frame the optimizaiton vec is contact force
 * and control forces
 */
void btGenFrameByFrameOptimizer::AddDynamicEnergyTermVel()
{
    tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    tVectorXd b = tVectorXd::Zero(num_of_freedom);

    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &C_d = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &q = mModel->Getq();
    const tVectorXd &qdot_next_ref = mTraj->mqdot[mRefFrameId + 1];
    // double dt2 = mdt * mdt;
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    b = mdt * Minv * (QG - C_d * qdot) + qdot - qdot_next_ref;
    tMatrixXd A1 = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize),
              A2 =
                  tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    {
        auto pt = mContactPoints[c_id];
        int size = mContactSolSize[pt->contact_id];
        int offset = mContactSolOffset[pt->contact_id];

        // (N * 3) * (3 * size) = N * size
        A1.block(0, offset, num_of_freedom, size).noalias() =
            mdt * Minv * pt->mJac.transpose() * pt->mS;
    }

    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    A2.noalias() = mdt * Minv * N;
    A.block(0, 0, num_of_freedom, mContactSolutionSize).noalias() = A1;
    A.block(0, mContactSolutionSize, num_of_freedom,
            num_of_underactuated_freedom)
        .noalias() = A2;
    // A /= mdt;
    // b /= mdt;
    // if (mIgnoreRootPosInDynamicEnergy == true)
    // {
    //     A = A.block(6, 0, num_of_underactuated_freedom, mTotalSolutionSize)
    //             .eval();
    //     b = b.segment(6, num_of_underactuated_freedom).eval();
    // }
    double coef = mDynamicVelEnergyCoeff / mdt;
    mEnergyTerm->AddEnergy(A, b, coef, 0, "vel");
}

/**
 * \brief               Add the dynamic energy term which reduce the error of
 * accel w.r.t ref traj in next frame the optimizaiton vec is contact force and
 * control forces
 */
void btGenFrameByFrameOptimizer::AddDynamicEnergyTermAccel()
{
    tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    tVectorXd b = tVectorXd::Zero(num_of_freedom);

    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &C_d = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &q = mModel->Getq();
    const tVectorXd &qddot_cur_ref = mTraj->mqddot[mRefFrameId];
    // double dt2 = mdt * mdt;
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    b = Minv * (QG - C_d * qdot) - qddot_cur_ref;
    // std::cout << "[accel] b norm " << b.norm() << std::endl;
    // std::cout << "[accel] Minv norm " << Minv.norm() << std::endl;
    // std::cout << "[accel] QG norm " << QG.norm() << std::endl;
    // std::cout << "[accel] Cd norm " << C_d.norm() << std::endl;
    // std::cout << "[accel] qdot norm " << qdot.norm() << std::endl;
    // std::cout << "[accel] qddot_ref norm " << qddot_cur_ref.norm() <<
    // std::endl;
    tMatrixXd A1 = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize),
              A2 =
                  tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    {
        auto pt = mContactPoints[c_id];
        int size = mContactSolSize[pt->contact_id];
        int offset = mContactSolOffset[pt->contact_id];

        // (N * 3) * (3 * size) = N * size
        A1.block(0, offset, num_of_freedom, size).noalias() =
            Minv * pt->mJac.transpose() * pt->mS;
    }

    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    A2.noalias() = Minv * N;
    A.block(0, 0, num_of_freedom, mContactSolutionSize).noalias() = A1;
    A.block(0, mContactSolutionSize, num_of_freedom,
            num_of_underactuated_freedom)
        .noalias() = A2;
    // A *= mDynamicAccelEnergyCoeff;
    // b *= mDynamicAccelEnergyCoeff;
    // if (mIgnoreRootPosInDynamicEnergy == true)
    // {
    //     A = A.block(6, 0, num_of_underactuated_freedom, mTotalSolutionSize)
    //             .eval();
    //     b = b.segment(6, num_of_underactuated_freedom).eval();
    // }
    mEnergyTerm->AddEnergy(A, b, mDynamicAccelEnergyCoeff, 0, "accel");
}

void btGenFrameByFrameOptimizer::AddDynamicEnergyTermMinAccel()
{
    tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    tVectorXd b = tVectorXd::Zero(num_of_freedom);

    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &C_d = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &q = mModel->Getq();
    const tVectorXd &qddot_cur_ref = mTraj->mqddot[mRefFrameId];
    // double dt2 = mdt * mdt;
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    b = Minv * (QG - C_d * qdot);
    // std::cout << "[accel] b norm " << b.norm() << std::endl;
    // std::cout << "[accel] Minv norm " << Minv.norm() << std::endl;
    // std::cout << "[accel] QG norm " << QG.norm() << std::endl;
    // std::cout << "[accel] Cd norm " << C_d.norm() << std::endl;
    // std::cout << "[accel] qdot norm " << qdot.norm() << std::endl;
    // std::cout << "[accel] qddot_ref norm " << qddot_cur_ref.norm() <<
    // std::endl;
    tMatrixXd A1 = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize),
              A2 =
                  tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    {
        auto pt = mContactPoints[c_id];
        int size = mContactSolSize[pt->contact_id];
        int offset = mContactSolOffset[pt->contact_id];

        // (N * 3) * (3 * size) = N * size
        A1.block(0, offset, num_of_freedom, size).noalias() =
            Minv * pt->mJac.transpose() * pt->mS;
    }

    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    A2.noalias() = Minv * N;
    A.block(0, 0, num_of_freedom, mContactSolutionSize).noalias() = A1;
    A.block(0, mContactSolutionSize, num_of_freedom,
            num_of_underactuated_freedom)
        .noalias() = A2;
    // A *= mDynamicAccelEnergyCoeff;
    // b *= mDynamicAccelEnergyCoeff;
    // if (mIgnoreRootPosInDynamicEnergy == true)
    // {
    //     A = A.block(6, 0, num_of_underactuated_freedom, mTotalSolutionSize)
    //             .eval();
    //     b = b.segment(6, num_of_underactuated_freedom).eval();
    // }
    mEnergyTerm->AddEnergy(A, b, mDynamicMinAccelEnergyCoeff, 0, "min_accel");
}
/**
 * \brief               Fix all static contact (add hard constraint)
 *
 * for contact point c, then the constraint jacobian A and residual b is defined
 * as:
 *
 *  A * f_total + b = 0, f_total \in R^K
 *
 * For i th static contact point:
 *  J_assemble = [Jv_0^T * K_0; Jv_1^T * K_1; \dots; Jv_c^T  * K_c; N]
 *  A_base = (dt * Minv * J_assemble)
 *  b_base = (dt * Minv * (Q_gravity - C * qdot) + qdot)
 *  A = Jv_i * A_base
 *  b = Jv_i * b_base
 *
 *  solution vector = [contact_foce_vector, active_ctrl_force]
 */
void btGenFrameByFrameOptimizer::AddFixStaticContactPointConstraint()
{
    // 1. calculate the assemble Jacobian
    tMatrixXd J_assemble = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    for (auto &pt : mContactPoints)
    {
        int id = pt->contact_id;
        int offset = mContactSolOffset[id];
        int sol_size = GetSolutionSizeByContactStatus(pt->mStatus);
        J_assemble.block(0, offset, num_of_freedom, sol_size) =
            pt->mJac.transpose() * pt->mS;
    }
    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    J_assemble.block(0, mContactSolutionSize, num_of_freedom,
                     mCtrlSolutionSize) = N;

    // 2. calculate the A base and b base
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    tMatrixXd A_base = mdt * Minv * J_assemble;

    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    tVectorXd b_base =
        (mdt * Minv * (QG - mModel->GetCoriolisMatrix() * mModel->Getqdot()) +
         mModel->Getqdot());

    // 3. iterate on each contact point, find all static ones, calculate A and
    // b, then apply these constraints
    for (auto &pt : mContactPoints)
    {
        if (eContactStatus::STATIC == pt->mStatus)
        {

            int id = pt->contact_id;
            std::cout << "[FBF] add static hard position constraint for "
                         "contact point "
                      << id << ", on link " << pt->mCollider->mLinkId
                      << std::endl;
            mConstraint->AddEquivalentEqCon(
                pt->mJac * A_base, pt->mJac * b_base, 0, 1e-12,
                "fix_static_contact_point_" + std::to_string(pt->contact_id));
        }
    }
}

/**
 * \brief                   Constraint the contact forces inside some range
 */
void btGenFrameByFrameOptimizer::AddContactForceLimitConstraint()
{
    // 1. fetch and judge the contact force limit
    double limit = mContactForceLimit;
    if (limit <= 0)
    {
        std::cout << "the contact force limit is <=0, illegal\n";
        exit(0);
    }
    tMatrixXd jac =
        tMatrixXd::Identity(mContactSolutionSize, mContactSolutionSize);
    tVectorXd upper_limit = tVectorXd::Ones(mContactSolutionSize) * limit,
              lower_limit = -tVectorXd::Ones(mContactSolutionSize) * limit;

    // force >= lower_limit
    // force - lower_limit >= 0
    mConstraint->AddIneqCon(jac, -lower_limit, 0, "lower_force_limit");

    // -1 * force >= -upper_limit
    // -1 * force + upper_limit  >= 0
    mConstraint->AddIneqCon(-jac, upper_limit, 0, "upper_force_limit");
    std::cout << "[limit] set the contact force limit " << limit << std::endl;
}
/**
 * \brief           the control force tau should be close to the calculated
 * value in the reference trajectory
 *
 *
 *      ||coef * (\tau - \tau_ref) ||^2 -> 0
 *
 *  A = I
 *  b = -\tau_ref
 *  Don't forget to add the coefficient
 */
void btGenFrameByFrameOptimizer::AddTauCloseToOriginEnergyTerm()
{
    // // std::cout << "active force = " <<
    // mTraj->mActiveForce[mCurFrameId].size()
    // //           << std::endl;

    // // std::cout << "underactuated dof = " << num_of_underactuated_freedom
    // //           << std::endl;

    // tMatrixXd A = tMatrixXd::Zero(num_of_freedom,
    // num_of_underactuated_freedom); int root_dof = num_of_freedom -
    // num_of_underactuated_freedom; A.block(root_dof, 0,
    // num_of_underactuated_freedom,
    //         num_of_underactuated_freedom)
    //     .setIdentity();
    // tVectorXd b = -mTraj->mActiveForce[mCurFrameId];
    // // std::cout << "A = \n" << A << std::endl;
    // // std::cout << "b = " << b.transpose() << std::endl;
    // mEnergyTerm->AddEnergy(A, b, mControlForceCloseToOriginCoef,
    //                        mContactSolutionSize,
    //                        "control_force_close_to_origin");
    // exit(1);
}

/**
 * \brief           Make the gen force of contact forces are close to the given
 * value in reference traj
 *  CartesianContact = ConvertMat * ContactSolution
 *  GenContact = JvT * CartesianContact
 *  min || coef * (GenContact - RefGenContact) ||^2
 *
 *  A = JvT * ConvertMat
 *  b = -RefGenContact
 *
 *  Don't forget to add the coefficient
 */
void btGenFrameByFrameOptimizer::AddContactForceCloseToOriginEnergyTerm()
{
    // if (mContactSolutionSize == 0)
    // {
    //     std::cout << "[debug] no contact solution size, the contact force "
    //                  "close to origin energy term should be closed\n";
    //     return;
    // }
    // tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize);
    // tVectorXd b = tVectorXd::Zero(num_of_freedom);

    // // 1. calculate A
    // for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    // {
    //     auto pt = mContactPoints[c_id];
    //     int size = mContactSolSize[pt->contact_id];
    //     int offset = mContactSolOffset[pt->contact_id];

    //     // (N * 3) * (3 * size) = N * size
    //     A.block(0, offset, num_of_freedom, size).noalias() =
    //         pt->mJac.transpose() * pt->mS;
    // }

    // // 2. calculate b, set the old state and calcualte the jacobian for old
    // // contact forces
    // mModel->PushState("add_contact_force_close_energy");
    // {
    //     tVectorXd old_q = mTraj->mq[mCurFrameId],
    //               old_qdot = mTraj->mqdot[mCurFrameId];
    //     mModel->SetqAndqdot(old_q, old_qdot);
    //     for (auto &f : mTraj->mContactForce[mCurFrameId])
    //     {
    //         int link_id = dynamic_cast<btGenRobotCollider
    //         *>(f->mObj)->mLinkId; tMatrixXd jac;
    //         mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
    //             link_id, f->mWorldPos.segment(0, 3), jac);
    //         b -= jac.transpose() * f->mForce.segment(0, 3);
    //     }
    // }
    // mModel->PopState("add_contact_force_close_energy");
    // // std::cout << "A = \n" << A << std::endl;
    // // std::cout << "b = " << b.transpose() << std::endl;
    // // 3. add it to the energy term structure
    // mEnergyTerm->AddEnergy(A, b, mContactForceCloseToOriginCoef, 0,
    //                        "contact_force_close_to_origin");
}

/**
 * \brief                   Add an energy term to control the world position of
 * end effector (enforcement)
 */
void btGenFrameByFrameOptimizer::AddEndEffectorPosEnergyTerm()
{

    // 1. get end effector id and their target pos
    mModel->PushState("end_effector_pos_energy");
    std::vector<int> link_id_lst(0);
    tEigenArr<tVector3d> link_target_pos_lst(0);
    tEigenArr<tVector3d> link_no_root_bias_pos_lst(0);
    mModel->SetqAndqdot(mTraj->mq[mRefFrameId + 1],
                        mTraj->mqdot[mRefFrameId + 1]);
    auto root_link = mModel->GetLinkById(0);
    tVector3d root_link_pos = root_link->GetWorldPos();
    tMatrix3d root_link_rot_inv = root_link->GetWorldOrientation().transpose();

    for (int i = 0; i < mModel->GetNumOfLinks(); i++)
    {
        auto link = mModel->GetLinkById(i);

        // if (-1 == link->GetParentId() || link->GetNumOfChildren() == 0)
        if (link->GetNumOfChildren() == 0)
        {
            // std::cout << "we want to control " << link->GetName() <<
            // std::endl;
            link_id_lst.push_back(i);

            link_target_pos_lst.push_back(link->GetWorldPos());
            link_no_root_bias_pos_lst.push_back(
                root_link_rot_inv * (link->GetWorldPos() - root_link_pos));
        }
    }
    mModel->PopState("end_effector_pos_energy");

    // 2. construct the energy term for each of them
    for (int idx = 0; idx < link_id_lst.size(); idx++)
    {
        if (mIgnoreRootBiasInEndEffectorControl == false)
        {
            AddLinkPosEnergyTerm(link_id_lst[idx], mEndEffectorPosCoef,
                                 link_target_pos_lst[idx]);
        }
        else
        {
            // add link pos energy term, but despite the rotation of root
            AddLinkPosEnergyTermIgnoreRoot(link_id_lst[idx],
                                           mEndEffectorPosCoef,
                                           link_no_root_bias_pos_lst[idx]);
        }
    }
}

/**
 * \brief                   Add an energt term to control the velocity of end
 * effectorsI
 */
void btGenFrameByFrameOptimizer::AddEndEffectorVelEnergyTerm()
{
    // 1. get all end effector and the target link vel in next frame
    mModel->PushState("end_effector_vel_energy");
    std::vector<int> link_id_lst(0);
    tEigenArr<tVector3d> link_target_vel_lst(0);
    mModel->SetqAndqdot(mTraj->mq[mRefFrameId + 1],
                        mTraj->mqdot[mRefFrameId + 1]);
    for (int i = 0; i < mModel->GetNumOfLinks(); i++)
    {
        auto link = mModel->GetLinkById(i);

        // if (-1 == link->GetParentId() || link->GetNumOfChildren() == 0)
        if (link->GetNumOfChildren() == 0)
        {
            // std::cout << "we want to control " << link->GetName() <<
            // std::endl;
            link_id_lst.push_back(i);

            link_target_vel_lst.push_back(link->GetJKv() * mModel->Getqdot());
        }
    }
    mModel->PopState("end_effector_vel_energy");

    // 2. control the link velocity
    for (int idx = 0; idx < link_id_lst.size(); idx++)
    {
        AddLinkVelEnergyTerm(link_id_lst[idx], mEndEffectorVelCoef,
                             link_target_vel_lst[idx]);
    }
}

/**
 * \brief                   Control the world orientation of end effector
 */
void btGenFrameByFrameOptimizer::AddEndEffectorOrientationEnergyTerm()
{
    // 1. get end effector id and their target pos
    mModel->PushState("end_effector_pos_energy");
    std::vector<int> link_id_lst(0);
    tEigenArr<tMatrix3d> link_target_rot_lst(0);
    mModel->SetqAndqdot(mTraj->mq[mRefFrameId + 1],
                        mTraj->mqdot[mRefFrameId + 1]);
    for (int i = 0; i < mModel->GetNumOfLinks(); i++)
    {
        auto link = mModel->GetLinkById(i);

        // if (-1 == link->GetParentId() || link->GetNumOfChildren() == 0)
        if (link->GetNumOfChildren() == 0)
        {
            // std::cout << "we want to control " << link->GetName() <<
            // std::endl;
            link_id_lst.push_back(i);

            link_target_rot_lst.push_back(link->GetWorldOrientation());
        }
    }
    mModel->PopState("end_effector_pos_energy");

    // 2. construct the energy term for each of them
    for (int idx = 0; idx < link_id_lst.size(); idx++)
    {
        AddLinkOrientationEnergyTerm(link_id_lst[idx],
                                     mEndEffectorOrientationCoef,
                                     link_target_rot_lst[idx]);
    }
}
/**
 * \brief                       Control the root position
 */
void btGenFrameByFrameOptimizer::AddRootPosEnergyTerm()
{
    mModel->PushState("root_pos_energy");

    mModel->SetqAndqdot(mTraj->mq[mRefFrameId + 1],
                        mTraj->mqdot[mRefFrameId + 1]);
    auto link = mModel->GetLinkById(0);
    tVector3d link_pos = link->GetWorldPos();
    mModel->PopState("root_pos_energy");
    AddLinkPosEnergyTerm(0, mRootPosCoef, link_pos);
}

/**
 * \brief                       Control the root vel
 */
void btGenFrameByFrameOptimizer::AddRootVelEnergyTerm()
{
    mModel->PushState("root_vel_energy");

    mModel->SetqAndqdot(mTraj->mq[mRefFrameId + 1],
                        mTraj->mqdot[mRefFrameId + 1]);
    auto link = mModel->GetLinkById(0);
    tVector3d vel = link->GetJKv() * mModel->Getqdot();
    mModel->PopState("root_vel_energy");
    AddLinkVelEnergyTerm(0, mRootVelCoef, vel);
}

/**
 * \brief                       Control the orientation of root link
 */
void btGenFrameByFrameOptimizer::AddRootOrientationEnergyTerm()
{
    mModel->PushState("root_ori_energy");

    mModel->SetqAndqdot(mTraj->mq[mRefFrameId + 1],
                        mTraj->mqdot[mRefFrameId + 1]);
    auto link = mModel->GetLinkById(0);
    tMatrix3d link_rot = link->GetWorldOrientation();
    mModel->PopState("root_ori_energy");

    AddLinkOrientationEnergyTerm(0, mRootOrientationCoef, link_rot);
}

/**
 * \brief                   Add an energy term to control the world pos of a
 * targeted link
 *
 *
 *          min || coef * (Jv_i * qdot_{t+1} + P_t - P_{t+1})||^2
 */
void btGenFrameByFrameOptimizer::AddLinkPosEnergyTerm(
    int link_id, double coef, const tVector3d &target_pos)
{
    if (link_id < 0 || link_id >= mModel->GetNumOfLinks())
    {
        std::cout << "[error] link id " << link_id
                  << " is illegal in AddLinkPosEnergyTerm\n";
        exit(1);
    }
    auto link = mModel->GetLinkById(link_id);
    tMatrixXd jac = link->GetJKv();
    tVector3d cur_pos = link->GetWorldPos();
    tMatrixXd A = tMatrixXd::Zero(3, mTotalSolutionSize);
    tVectorXd b = tVectorXd::Zero(3);
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &C_d = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &q = mModel->Getq();
    const tVectorXd &qdot_next_ref = mTraj->mqdot[mRefFrameId + 1];
    double dt2 = mdt * mdt;
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    b = dt2 * jac * Minv * (QG - C_d * qdot) + mdt * jac * qdot + cur_pos -
        target_pos;
    // for contact forces
    for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    {
        auto pt = mContactPoints[c_id];
        int size = mContactSolSize[pt->contact_id];
        int offset = mContactSolOffset[pt->contact_id];

        // (3 * N) * (N * 3) * (3 * size) = 3 * size
        A.block(0, offset, 3, size).noalias() =
            dt2 * jac * Minv * pt->mJac.transpose() * pt->mS;
    }

    // for control forces
    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    A.block(0, mContactSolutionSize, 3, mCtrlSolutionSize) =
        dt2 * jac * Minv * N;

    coef /= dt2;
    mEnergyTerm->AddEnergy(A, b, coef, 0,
                           "link_pos_" + std::string(link->GetName()));
}

/**
 * \brief                   Only Constraint the relative pos wrt the root link
 * for the end effector
 */
void btGenFrameByFrameOptimizer::AddLinkPosEnergyTermIgnoreRoot(
    int link_id, double coef, const tVector3d &target_link_local_pos_wrt_root)
{
    std::cout << "add AddLinkPosEnergyTermIgnoreRoot for link " << link_id
              << std::endl;
    if (link_id == 0)
    {
        std::cout << "[error] ignore root link pos energy term shouldn't be "
                     "added to root link\n";
        exit(1);
    }
    if (link_id < 0 || link_id >= mModel->GetNumOfLinks())
    {
        std::cout << "[error] link id " << link_id
                  << " is illegal in AddLinkPosEnergyTermIgnoreRoot\n";
        exit(1);
    }
    auto link = mModel->GetLinkById(link_id);
    tMatrix3d root_rot_inv =
        mModel->GetLinkById(0)->GetWorldOrientation().transpose();
    tMatrixXd jac = link->GetJKv();
    tVector3d cur_pos = link->GetWorldPos();
    tVector3d cur_root_pos = mModel->GetLinkById(0)->GetWorldPos();
    tMatrixXd A = tMatrixXd::Zero(3, mTotalSolutionSize);
    tVectorXd b = tVectorXd::Zero(3);
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &C_d = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &q = mModel->Getq();
    const tVectorXd &qdot_next_ref = mTraj->mqdot[mRefFrameId + 1];
    double dt2 = mdt * mdt;
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    b = dt2 * root_rot_inv * jac * Minv * (QG - C_d * qdot) +
        mdt * root_rot_inv * jac * qdot +
        root_rot_inv * (cur_pos - cur_root_pos) -
        target_link_local_pos_wrt_root;
    // for contact forces
    for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    {
        auto pt = mContactPoints[c_id];
        int size = mContactSolSize[pt->contact_id];
        int offset = mContactSolOffset[pt->contact_id];

        // (3 * N) * (N * 3) * (3 * size) = 3 * size
        A.block(0, offset, 3, size).noalias() =
            dt2 * root_rot_inv * jac * Minv * pt->mJac.transpose() * pt->mS;
    }

    // for control forces
    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    A.block(0, mContactSolutionSize, 3, mCtrlSolutionSize) =
        dt2 * root_rot_inv * jac * Minv * N;

    coef /= dt2;
    mEnergyTerm->AddEnergy(
        A, b, coef, 0, "link_ignore_root_pos_" + std::string(link->GetName()));
}

/**
 * \brief                   Add the target orientation energy term for a link
 *
 *  min | R_target - R_t+1^i | ^2
 *
 * For more details, please check the note
 *
 * F_err    = F_{t+1}^i - F_target^i
 *          = dt * dFdq * \dot{q}_{t+1} + F_t^i - F_target^i
 *          = dt2 * dFdq * Minv * Q_ext + dFdq * (dt2 * (Qg - Cqdot) + dt*qdot)
 *              + F_{t} - F_target
 */
void btGenFrameByFrameOptimizer::AddLinkOrientationEnergyTerm(
    int link_id, double coef, const tMatrix3d &target_orientation)
{
    auto Flatten = [](const tMatrix3d &mat) -> tVectorXd {
        tVectorXd res = tVectorXd::Zero(9);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                res[i * 3 + j] = mat(i, j);
        return res;
    };

    // 1. get dR/dq, flatten qcur, flatten qtarget
    auto link = mModel->GetLinkById(link_id);

    tMatrixXd dRdq = tMatrixXd::Zero(9, num_of_freedom);
    {
        auto prev_dof_map = link->GetPrevFreedomIds();

        int num_of_total_freedom_link = link->GetNumTotalFreedoms();
        if (num_of_total_freedom_link != prev_dof_map.size())
        {
            std::cout << "[error] the num of freedom doesn't match for link "
                      << link_id << " prev dof = " << prev_dof_map.size()
                      << " total dof = " << num_of_total_freedom_link
                      << std::endl;
            exit(0);
        }

        for (int local_freedom_id = 0;
             local_freedom_id < num_of_total_freedom_link; local_freedom_id++)
        {
            int global_freedom_id = prev_dof_map[local_freedom_id];
            // std::cout << "link " << link_id << " local dof " <<
            // local_freedom_id
            //           << " global freedom " << global_freedom_id <<
            //           std::endl;
            dRdq.col(global_freedom_id) =
                Flatten(link->GetMWQ(local_freedom_id).block(0, 0, 3, 3));
        }
    }
    // exit(0);
    tVectorXd target_orient_flatten = Flatten(target_orientation);
    tVectorXd cur_orient_flatten = Flatten(link->GetWorldOrientation());

    // 2. shape the convert mat from opt solution to num_of_freedom
    // for contact forces
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    tMatrixXd convert_mat = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    {
        auto pt = mContactPoints[c_id];
        int size = mContactSolSize[pt->contact_id];
        int offset = mContactSolOffset[pt->contact_id];

        // (3 * N) * (N * 3) * (3 * size) = 3 * size
        convert_mat.block(0, offset, num_of_freedom, size).noalias() =
            pt->mJac.transpose() * pt->mS;
    }

    // for control forces
    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    convert_mat.block(0, mContactSolutionSize, num_of_freedom,
                      mCtrlSolutionSize) = N;

    // 2. shape the final A and b
    const tMatrixXd &C_d = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &q = mModel->Getq();
    const tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    double dt2 = mdt * mdt;

    tMatrixXd A = dt2 * dRdq * Minv * convert_mat;
    tVectorXd b = dRdq * (dt2 * Minv * (QG - C_d * qdot) + mdt * qdot) +
                  cur_orient_flatten - target_orient_flatten;

    // A /= dt2;
    // b /= dt2;
    coef /= dt2;
    // std::cout << "cur orientation " << cur_orient_flatten.transpose()
    //           << std::endl;
    // std::cout << "target orientation " << target_orient_flatten.transpose()
    //           << std::endl;
    // std::cout << "dFdq = \n" << dRdq << std::endl;
    // std::cout << "A = \n" << A << std::endl;
    // std::cout << "b = \n" << b.transpose() << std::endl;

    mEnergyTerm->AddEnergy(A, b, coef, 0,
                           "link_orientation_" + std::string(link->GetName()));
}

/**
 * \brief                   Add link vel energy term
 *
 *          A = dt * Jv * Minv * convert_mat
 *          b = Jv * (dt * Minv * (QG - Cqdot) + qdot) - v_target
 */
void btGenFrameByFrameOptimizer::AddLinkVelEnergyTerm(
    int link_id, double coef, const tVector3d &target_vel)
{
    // 1. shape the convert mat
    tMatrixXd convert_mat = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    {
        auto pt = mContactPoints[c_id];
        int size = mContactSolSize[pt->contact_id];
        int offset = mContactSolOffset[pt->contact_id];

        // (3 * N) * (N * 3) * (3 * size) = 3 * size
        convert_mat.block(0, offset, num_of_freedom, size).noalias() =
            pt->mJac.transpose() * pt->mS;
    }

    // for control forces
    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    convert_mat.block(0, mContactSolutionSize, num_of_freedom,
                      mCtrlSolutionSize) = N;

    // 2. shape other essential
    auto link = mModel->GetLinkById(link_id);
    const tMatrixXd &jac = link->GetJKv();
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tVectorXd &QG = mModel->CalcGenGravity(mWorld->GetGravity());
    const tVectorXd &qdot = mModel->Getqdot();
    tMatrixXd A = mdt * jac * Minv * convert_mat;
    tVectorXd b =
        jac * (mdt * Minv * (QG - mModel->GetCoriolisMatrix() * qdot) + qdot) -
        target_vel;

    coef /= mdt;
    mEnergyTerm->AddEnergy(A, b, coef, 0,
                           "link_vel_" + std::string(link->GetName()));
}

/**
 * \brief                   enforce that the contact point cannot be penetrated
 * in next frame
 */
void btGenFrameByFrameOptimizer::AddNonPenetrationContactConstraint()
{
    // std::cout << "added !\n ";
    // exit(1);
    if (mContactSolutionSize == 0)
        return;
    tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    tVectorXd b = tVectorXd::Zero(num_of_freedom);
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &C_d = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &q = mModel->Getq();
    const tVectorXd &qdot_next_ref = mTraj->mqdot[mRefFrameId + 1];
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    b = mdt * Minv * (QG - C_d * qdot) + qdot;
    // for contact forces
    for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
    {
        auto pt = mContactPoints[c_id];
        int size = mContactSolSize[pt->contact_id];
        int offset = mContactSolOffset[pt->contact_id];

        // (3 * N) * (N * 3) * (3 * size) = 3 * size
        A.block(0, offset, num_of_freedom, size).noalias() =
            mdt * Minv * pt->mJac.transpose() * pt->mS;
    }

    // for control forces
    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    A.block(0, mContactSolutionSize, num_of_freedom, mCtrlSolutionSize) =
        mdt * Minv * N;

    // std::cout << "--------------begin to add non penetration cons\n";
    for (int id = 0; id < mContactPoints.size(); id++)
    {

        auto &pt = mContactPoints[id];
        tVector3d normal = pt->mNormalPointToA.segment(
            0, 3); // the contact force is calculated to be
                   // applied on bodyA, so we take the normal "to
                   // A" but not "from A"
        tMatrixXd A_final = normal.transpose() * pt->mJac * A;
        tVectorXd b_final = normal.transpose() * pt->mJac * b;
        // std::cout << "contact " << id << " norm = " << normal.transpose()
        //           << std::endl;
        // std::cout << "A final = \n" << A_final << std::endl;
        // std::cout << "b final = \n" << b_final << std::endl;
        // A x + b >= eps
        // A x + b - eps >= 0
        // b_final -= eps * tVectorXd::Ones(b_final.size());
        mConstraint->AddIneqCon(A_final, b_final, 0,
                                "NonPenetration_point_" + std::to_string(id));
    }

    // exit(0);
}

/**
 * \brief                   track the contact points in ref trajectory
 * 
 * P_err = J_ci * A * X + J_ci * b_1 + b_2i
 * 
 * E_err = (coefi * P_err)^2
 *       = [coefi * J_ci * A * X + coefi * (J_ci * b_1 + b_2i)]^2
 *       =
 *          [ coef0 * J_c0 ]            [coef0 * (J_c0 * b1 + b_20)]
 *          [ coef1 * J_c1 ]            [coef1 * (J_c1 * b1 + b_21)]
 *          [      ...     ]            [           ...            ]
 *          [ coefn * J_cn ] * A * X +  [coefn * (J_cn * b1 + b_2n)]
 *       =   J_total * A * X + b_total
 * 
 * J_c is the jacobian of tracked contact point
 * A = dt2 * Minv * K 
 * b1 = dt2 * Minv * (QG - C*qdot) + dt * qdot
 * b2 = pt - ptar
 */
void btGenFrameByFrameOptimizer::AddTrackRefContactEnergyTerm()
{
    std::cout << "begin to add track ref contact energy term\n";
    // 1. get the points we want to control, calculate the coef weight by the
    // frame id
    tEigenArr<tMatrixXd> contact_jac_lst(0);
    std::vector<double> coeff_lst(0);
    tEigenArr<tVector3d> contact_point_cur_target_lst(0);
    tEigenArr<tVector3d> contact_point_next_target_lst(0);
    CalcTrackRefContactRef(contact_jac_lst, coeff_lst,
                           contact_point_next_target_lst,
                           contact_point_cur_target_lst);

    // 2. begin to calculate the energy term
    // 2.1 calculate the A part
    tMatrixXd K = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
    int num_of_track_pts = mContactPoints.size();
    {
        for (int c_id = 0; c_id < num_of_track_pts; c_id++)
        {
            auto pt = mContactPoints[c_id];
            int size = mContactSolSize[pt->contact_id];
            int offset = mContactSolOffset[pt->contact_id];

            // (3 * N) * (N * 3) * (3 * size) = 3 * size
            K.block(0, offset, num_of_freedom, size).noalias() =
                pt->mJac.transpose() * pt->mS;
        }

        // for control forces
        tMatrixXd N =
            tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
        N.block(6, 0, num_of_underactuated_freedom,
                num_of_underactuated_freedom)
            .setIdentity();
        K.block(0, mContactSolutionSize, num_of_freedom, mCtrlSolutionSize) = N;
    }
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    double dt2 = mdt * mdt;
    tMatrixXd A = dt2 * Minv * K;

    // 2.2 calculate the b1
    const tMatrixXd &C = mModel->GetCoriolisMatrix();
    const tVectorXd &qdot = mModel->Getqdot();
    const tVectorXd &QG = mModel->CalcGenGravity(mWorld->GetGravity());
    tVectorXd b1 = dt2 * Minv * (QG - C * qdot) + mdt * qdot;

    // 2.3 calculate the J_total and and b_total
    tMatrixXd J_total = tMatrixXd::Zero(3 * num_of_track_pts, num_of_freedom);
    tVectorXd b_total = tVectorXd::Zero(3 * num_of_track_pts);
    for (int id = 0; id < num_of_track_pts; id++)
    {
        // 2.3.1 blank in the J_total
        const double &coef = coeff_lst[id];
        const tMatrixXd &jac = contact_jac_lst[id];
        J_total.block(id * 3, 0, 3, num_of_freedom) = coef * jac;

        // 2.3.2 calc the b_total componenet
        tVectorXd b2i = contact_point_cur_target_lst[id] -
                        contact_point_next_target_lst[id];
        b_total.segment(3 * id, 3) = coef * (jac * b1 + b2i);
    }

    // 2.4 shape the final energy term
    mEnergyTerm->AddEnergy(J_total * A, b_total, 1, 0, "track_ref_contact");
}

/**
 * \brief                       Calculate the contact points in the ref
 * trajectory which we needs to track in "Track_Ref_Contact" Energy Term
 * \param jac_lst               the jacobian of these selected contact points in next frame
 * \param coeff_lst             the weight coefficient of this contact point
 * \param next_target_pos_lst   the target position of tracked contact point next frame in ref traj
 * \param cur_pos_lst           current position of tracked contact point
 * 
 */
double blend(double max_coef, double min_coef, int control_frame_range,
             int cur_frame)
{
    if (max_coef <= min_coef)
    {
        std::cout << "[error] max coef " << max_coef << " < " << min_coef
                  << std::endl;
        exit(1);
    }
    if (cur_frame > control_frame_range)
    {
        std::cout << "[error] cur frame " << cur_frame << " > "
                  << control_frame_range << std::endl;
        exit(1);
    }
    double cur_coef =
        (max_coef - min_coef) / control_frame_range * cur_frame + min_coef;
    return cur_coef;
}

void btGenFrameByFrameOptimizer::CalcTrackRefContactRef(
    tEigenArr<tMatrixXd> &jac_lst, std::vector<double> &coeff_lst,
    tEigenArr<tVector3d> &next_target_pos_lst,
    tEigenArr<tVector3d> &cur_pos_lst)
{
    jac_lst.clear();
    coeff_lst.clear();
    next_target_pos_lst.clear();
    cur_pos_lst.clear();

    std::vector<int> link_id_lst(0);
    tEigenArr<tVector3d> local_pos_lst(0);
    for (int inc = 0; inc < mTrackRefContactRange; inc++)
    {
        // 1. calc current frame & judge
        int frame_id = mRefFrameId + inc;
        if (mRefFrameId >= mTraj->mNumOfFrames)
            break;

        // 2. calc coeff in this frame
        double cur_coef =
            blend(mTrackRefContactMaxCoef, mTrackRefContactMinCoef,
                  mTrackRefContactRange, inc);
        // std::cout << "---------------frame " << frame_id << std::endl;
        // 3. fetch & save the local pos, calculate the current world pos, calculate the jacobian
        auto &local_contact_points = mRefContactLocalPos[frame_id];
        for (int i = 0; i < local_contact_points.size(); i++)
        {
            const tVector3d &local_contact_pt = local_contact_points[i];
            local_pos_lst.push_back(local_contact_pt);
            // 3.1 save the coef
            coeff_lst.push_back(cur_coef);

            // 3.2 calculate cur pos

            auto gen_collider = static_cast<btGenRobotCollider *>(
                mTraj->mContactForce[frame_id][i]->mObj);
            int link_id = gen_collider->mLinkId;
            link_id_lst.push_back(link_id);
            auto link = mModel->GetLinkById(link_id);

            tVector cur_world_pos = link->GetGlobalTransform() *
                                    btMathUtil::Expand(local_contact_pt, 1);
            cur_pos_lst.push_back(cur_world_pos.segment(0, 3));

            // 3.3 calcualte the jacobian
            tMatrixXd jac;
            mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                link_id, cur_world_pos.segment(0, 3), jac);
            jac_lst.push_back(jac);
            // std::cout << "link " << link_id << " local pos "
            //           << local_contact_pt.transpose() << " cur world pos "
            //           << cur_world_pos.transpose() << std::endl;
        }
    }

    // 4. calculate the target world pos for tracked ref contacts in next frame
    mModel->PushState("track_ref_contact");
    const tVectorXd &q_next = mTraj->mq[mRefFrameId + 1];
    const tVectorXd &qdot_next = mTraj->mq[mRefFrameId + 1];
    mModel->SetqAndqdot(q_next, qdot_next);

    for (int contact_id = 0; contact_id < local_pos_lst.size(); contact_id++)
    {

        const tVector3d &local_pos = local_pos_lst[contact_id];
        int link_id = link_id_lst[contact_id];
        auto link = mModel->GetLinkById(link_id);
        tVector next_world_pos =
            link->GetGlobalTransform() *
            btMathUtil::Expand(local_pos_lst[contact_id], 1);
        next_target_pos_lst.push_back(next_world_pos.segment(0, 3));
        // std::cout << "contact id " << contact_id << " local pos "
        //           << local_pos_lst[contact_id].transpose()
        //           << " target world pos " << next_world_pos.transpose()
        //           << std::endl;
    }
    mModel->PopState("track_ref_contact");
}