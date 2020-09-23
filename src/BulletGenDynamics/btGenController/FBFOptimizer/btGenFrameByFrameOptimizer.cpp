#include "btGenFrameByFrameOptimizer.h"
#include "BulletGenDynamics/btGenController/FBFOptimizer/btGenFBFConstraint.h"
#include "BulletGenDynamics/btGenController/FBFOptimizer/btGenFBFEnergyTerm.h"
#include "BulletGenDynamics/btGenController/QPSolver/MatlabQPSolver.h"
#include "BulletGenDynamics/btGenController/QPSolver/QuadProgQPSolver.h"
#include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenSolver/ConstraintData.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include "btCharContactPoint.h"
int mNumOfFrictionDirs = 4;

const std::string gContactStatusStr[] = {"INVALID_CONTACT_STATUS", "SLIDING",
                                         "STATIC", "BREAKAGE"};

eContactStatus JudgeContactStatus(const tVector &vel,
                                  double breakage_threshold = 2e-3,
                                  double sliding_threshold = 5e-3)
{
    double vel_n = vel[1];
    double vel_t = std::sqrt(std::pow(vel[0], 2) + std::pow(vel[2], 2));
    if (vel_n > breakage_threshold)
    {
        return eContactStatus::BREAKAGE;
        // return eContactStatus::STATIC;
    }
    else
    {
        if (vel_t > sliding_threshold)
        {
            return eContactStatus::SLIDING;
        }
        else
            return eContactStatus::STATIC;
    }
}

// static QuadProgQPSolver *matlabQPSolver = nullptr;
btGenFrameByFrameOptimizer::btGenFrameByFrameOptimizer()
{
    mCurFrameId = -1;
    mModel = nullptr;
    mTraj = nullptr;
    mWorld = nullptr;
    mQPSolver = nullptr;
    mContactPoints.clear();
    mContactSolOffset.clear();
    mContactSolSize.clear();

    mTotalSolutionSize = 0;
    mContactSolutionSize = 0;
    mCtrlSolutionSize = 0;

    A.resize(0, 0);
    Aeq.resize(0, 0);
    Aineq.resize(0, 0);

    b.resize(0);
    beq.resize(0);
    bineq.resize(0);
    mConstraint = nullptr;
    mEnergyTerm = nullptr;

    num_of_freedom = 0;
    num_of_underactuated_freedom = 0;
    mEnableFixStaticContactPoint = false;
}

btGenFrameByFrameOptimizer::~btGenFrameByFrameOptimizer()
{
    if (mConstraint)
        delete mConstraint;
    if (mQPSolver)
        delete mQPSolver;
}
// void btGenFrameByFrameOptimizer::Init(const Json::Value &conf, btTraj
// *ref_traj,
//                      btGeneralizeWorld *world)
void btGenFrameByFrameOptimizer::Init(btGeneralizeWorld *world,
                                      const Json::Value &conf)
{
    mWorld = world;
    mModel = mWorld->GetMultibody();

    mDynamicPosEnergyCoeff =
        btJsonUtil::ParseAsDouble("dynamic_pos_energy_coef", conf);
    mDynamicVelEnergyCoeff =
        btJsonUtil::ParseAsDouble("dynamic_vel_energy_coef", conf);
    mDynamicAccelEnergyCoeff =
        btJsonUtil::ParseAsDouble("dynamic_accel_energy_coef", conf);
    mControlForceCoef = btJsonUtil::ParseAsDouble("control_force_coef", conf);
    mContactForceCoef = btJsonUtil::ParseAsDouble("contact_force_coef", conf);
    mEnableFixStaticContactPoint =
        btJsonUtil::ParseAsBool("fix_the_static_contact_point", conf);
    mControlForceCloseToOriginCoef =
        btJsonUtil::ParseAsDouble("control_force_close_to_origin_coef", conf);
    mContactForceCloseToOriginCoef =
        btJsonUtil::ParseAsDouble("contact_force_close_to_origin_coef", conf);
    mEndEffectorPosCoef =
        btJsonUtil::ParseAsDouble("end_effector_pos_coef", conf);

    // ParseConfig(conf);
    InitModelInfo();
    InitQPSolver();
}

/**
 * \brief                   Calculate the reference qddot and the reference tau
 * by frame-by-frame control
 */
void btGenFrameByFrameOptimizer::CalcTarget(double dt, int target_frame_id,
                                            tVectorXd &tilde_qddot,
                                            tVectorXd &tilde_qdot,
                                            tVectorXd &tilde_q,
                                            tVectorXd &tilde_tau)
{
    if (mTraj == nullptr)
    {
        std::cout
            << "[error] the traj hasn't been set in the FBFOptimizer, exit";
        exit(0);
    }
    mCurFrameId = target_frame_id;
    mdt = dt;

    // if (mUseNativeRefTarget == true)
    // {
    //     std::cout << "[warn] use native ref target in FBF optimizer\n";
    //     tilde_qddot = mTraj->mqddot[mCurFrameId];
    //     tilde_tau = mTraj->mActiveForce[mCurFrameId].segment(
    //         6, num_of_underactuated_freedom);
    // }
    // else
    // {
    // }

    // 1. judge the contact status
    CalcContactStatus();

    // 2. construct the optimization problem: energy terms + hard
    // constraints for
    CalcSolutionVector();
    CalcEnergyTerms();
    CalcConstraints();

    // 3. solve this problem and get the ref contact force, ref control
    // force calculate the generalized coordinate accel qddot, and calculate
    // the ref tau
    Solve(tilde_qddot, tilde_qdot, tilde_q, tilde_tau);
}

/**
 * \brief                   Get the contact points, determine the contact
 * constraint
 */
void btGenFrameByFrameOptimizer::CalcContactStatus()
{
    ClearContactPoints();
    // 1. get contact points & link id, calc point positions in local frame
    std::vector<btPersistentManifold *> manifolds =
        mWorld->GetContactManifolds();

    tEigenArr<tVector> ContactLocalPos(0);
    std::vector<int> ContactLinkId(0);
    for (auto &mani : manifolds)
    {
        for (int i = 0; i < mani->getNumContacts(); i++)
        {
            auto data = new btCharContactPt(mContactPoints.size());
            data->Init(0, mani, i);

            // judge whether it belongs to the multibody
            if (data->IsMultibodyInvolved(mModel) == true &&
                data->mIsSelfCollision == false)
            {
                data->CalcCharacterInfo();
                mContactPoints.push_back(data);
            }

            else
                delete data;
        }
    }

    // 2. get the reference pos for these points in next frame, calculate their
    // velocity judge the contact status
    {
        tEigenArr<tVector> contact_pos_cur_ref(0),
            contact_pos_next_ref(
                0); // save the position of contact point in current frame and
                    // next frame of ref motion
        tEigenArr<tVector> contact_vel_cur_ref(0);
        {
            mModel->PushState("fbf ctrl");
            tVectorXd q_cur_ref = mTraj->mq[mCurFrameId];
            mModel->Apply(q_cur_ref, false);
            for (auto &pt : mContactPoints)
            {
                auto link = mModel->GetLinkById(pt->mCollider->mLinkId);
                contact_pos_cur_ref.push_back(link->GetGlobalTransform() *
                                              pt->mLocalPos);
            }
            mModel->PopState("fbf ctrl");
        }
        {
            mModel->PushState("fbf ctrl");
            tVectorXd q_next_ref = mTraj->mq[mCurFrameId + 1];
            mModel->Apply(q_next_ref, false);
            for (auto &pt : mContactPoints)
            {
                auto link = mModel->GetLinkById(pt->mCollider->mLinkId);
                tVector cur_global_pos =
                    link->GetGlobalTransform() * pt->mLocalPos;
                contact_pos_next_ref.push_back(cur_global_pos);

                {
                    tMatrixXd jac;
                    mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                        link->GetId(), cur_global_pos.segment(0, 3), jac);
                    tVectorXd qdot = mTraj->mqdot[mCurFrameId + 1];
                    tVector vel = tVector::Zero();
                    vel.segment(0, 3) = jac * qdot;
                    contact_vel_cur_ref.push_back(vel);
                }
            }
            mModel->PopState("fbf ctrl");
        }

        int num_of_contacts = mContactPoints.size();
        for (int id = 0; id < num_of_contacts; id++)
        {
            auto &pt = mContactPoints[id];

            // 1. calculate rel vel
            tVector vel =
                (contact_pos_next_ref[id] - contact_pos_cur_ref[id]) / mdt;
            // 2. judge contact status
            pt->mStatus = JudgeContactStatus(vel);
            // pt->mStatus = eContactStatus::STATIC;
            auto link = mModel->GetLinkById(pt->mCollider->mLinkId);

            std::cout << "[FBF] contact " << id << " on link "
                      << link->GetName()
                      << " cartesian vel in ref traj = " << vel.transpose()
                      << " status " << gContactStatusStr[pt->mStatus]
                      << std::endl;
            // std::cout << "contact " << id << " gen vel = " <<
            // contact_vel_cur_ref[id].transpose() << std::endl;
            // 3. calculate convert mat
            CalcContactConvertMat(pt, pt->mS);
        }
    }

    // std::cout
    // 	<< "calc contact status done " << mContactPoints.size() << std::endl;
}

/**
 * \brief                   Calculate the reference qddot and control forces
 */
void btGenFrameByFrameOptimizer::Solve(tVectorXd &tilde_qddot,
                                       tVectorXd &tilde_qdot,
                                       tVectorXd &tilde_q, tVectorXd &tilde_tau)
{
    // std::cout << "q, qdot solve hasn't been supported\n";
    // exit(1);
    // 1. solve the QP problem
    // std::cout << "[debug] begin to solve the QP problem\n";
    mEnergyTerm->GetEnergyTerm(A, b);
    mConstraint->GetEqJacobianAndResidual(Aeq, beq); // Aeq * x + beq = 0
    mConstraint->GetIneqJacobianAndResidual(Aineq,
                                            bineq); // Aineq * x + bineq >= 0
    tMatrixXd H = 2 * A.transpose() * A;
    tVectorXd f = 2 * b.transpose() * A;
    // bineq *= -1;
    // beq *= -1;
    tVectorXd solution = tVectorXd::Zero(mTotalSolutionSize);
    Aeq.transposeInPlace();
    Aineq.transposeInPlace();
    mQPSolver->Solve(mTotalSolutionSize, H, f, Aeq, beq, Aineq, bineq, 100,
                     solution);

    // try matlab solver
    // matlab solver gives the same solution as quadprog
    // {
    //     tVectorXd matlab_sol;
    //     tMatrixXd new_Aeq = Aeq.transpose(), new_Aineq = -Aineq.transpose();
    //     tVectorXd new_beq = -beq, new_bineq = bineq;
    //     matlabQPSolver->Solve(mTotalSolutionSize, H, f, new_Aeq, new_beq,
    //                           new_Aineq, new_bineq, 100, matlab_sol);
    //     tVectorXd diff = solution - matlab_sol;
    //     if (diff.norm() > 1e-6)
    //     {
    //         std::cout << "[FBF] quad prog sol = " << solution.transpose()
    //                   << std::endl;
    //         std::cout << "[FBF] matlab sol = " << matlab_sol.transpose()
    //                   << std::endl;
    //         std::cout << "[FBF] sol diff = " << diff.transpose() <<
    //         std::endl; std::cout << "[FBF] sol diff norm = " << diff.norm()
    //         << std::endl; solution = matlab_sol;
    //     }
    //     else
    //         std::cout << "[FBF] solved correctly\n";

    //     // exit(0);
    // }
    // double energy = 0.5 * (solution.transpose() * H).dot(solution) +
    //                 f.dot(solution) + b.dot(b);
    // std::cout << "[qp] energy = " << energy << std::endl;

    CalcTargetInternal(solution, tilde_qddot, tilde_qdot, tilde_q, tilde_tau);
    mEnergyTerm->CheckEnergyValue(solution);

    // check accel energy term
    // if (mDynamicAccelEnergyCoeff > 0 && mContactSolutionSize == 0)
    // {
    //     tMatrixXd N =
    //         tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    //     N.block(6, 0, num_of_underactuated_freedom,
    //             num_of_underactuated_freedom)
    //         .setIdentity();
    //     tMatrixXd A_manual = mModel->GetInvMassMatrix() * N;
    //     // std::cout << "N = \n" << N << std::endl;
    //     tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    //     tVectorXd b_manual =
    //         mModel->GetInvMassMatrix() *
    //             (QG - mModel->GetCoriolisMatrix() * mModel->Getqdot()) -
    //         mTraj->mqddot[mCurFrameId];
    //     tVectorXd lsq_sol = -(A_manual.transpose() * A_manual).inverse() *
    //                         A_manual.transpose() * b_manual;

    //     tVectorXd diff = solution - lsq_sol;
    //     if (diff.norm() > 1e-10)
    //     {
    //         std::cout << "sol diff = " << diff.transpose() << std::endl;
    //         std::cout << "sol diff norm = " << diff.norm() << std::endl;
    //         exit(1);
    //     }
    // }

    // // check vel energy term
    // if (mDynamicVelEnergyCoeff > 0 && mContactSolutionSize == 0)
    // {
    //     tMatrixXd N =
    //         tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    //     N.block(6, 0, num_of_underactuated_freedom,
    //             num_of_underactuated_freedom)
    //         .setIdentity();
    //     tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());

    //     tMatrixXd A_man = mdt * mModel->GetInvMassMatrix() * N;
    //     tVectorXd b_man =
    //         mdt * mModel->GetInvMassMatrix() *
    //             (QG - mModel->GetCoriolisMatrix() * mModel->Getqdot()) +
    //         mModel->Getqdot() - mTraj->mqdot[mCurFrameId + 1];
    //     tVectorXd lsq_sol =
    //         -(A_man.transpose() * A_man).inverse() * A_man.transpose() * b_man;

    //     tVectorXd diff = lsq_sol - solution;
    //     if (diff.norm() > 1e-10)
    //     {
    //         std::cout << "[vel] diff = " << diff.norm() << std::endl;
    //         exit(1);
    //     }
    // }
    // if (mDynamicPosEnergyCoeff > 0 && mContactSolutionSize == 0)
    // {
    //     tMatrixXd N =
    //         tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    //     N.block(6, 0, num_of_underactuated_freedom,
    //             num_of_underactuated_freedom)
    //         .setIdentity();
    //     tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());

    //     tMatrixXd A_man = mdt * mdt * mModel->GetInvMassMatrix() * N;
    //     tVectorXd b_man =
    //         mdt * mdt * mModel->GetInvMassMatrix() *
    //             (QG - mModel->GetCoriolisMatrix() * mModel->Getqdot()) +
    //         mdt * mModel->Getqdot() + mModel->Getq() -
    //         mTraj->mq[mCurFrameId + 1];
    //     tVectorXd lsq_sol =
    //         -(A_man.transpose() * A_man).inverse() * A_man.transpose() * b_man;

    //     tVectorXd diff = lsq_sol - solution;
    //     if (diff.norm() > 1e-10)
    //     {
    //         std::cout << "[pos] diff = " << diff.transpose() << std::endl;
    //         std::cout << "[pos] diff norm = " << diff.norm() << std::endl;
    //         std::cout << "[pos] opt sol = " << solution.transpose()
    //                   << std::endl;
    //         std::cout << "[pos] lsq sol = " << lsq_sol.transpose() << std::endl;
    //         exit(1);
    //     }
    // }
    // tVectorXd ref_tau = mTraj->mActiveForce[mCurFrameId].segment(
    //     6, num_of_underactuated_freedom);
    // std::cout << "[solved] tau = " << tilde_tau.transpose() << std::endl;
    // std::cout << "[ref] tau = " << ref_tau.transpose() << std::endl;
}

/**
 * \brief               Given the solution(solved contact forces + solved
 * control forces), calculate:
 * \param qddot         1. the gen accel of robot model at this moment (target)
 * \param qdot          2. the gen vel of model in next frame (target)
 * \param q             3. the gen pos of model in next frame (target)
 * \param tau           4. and the control force (tau) according to the dynamics
 * equation
 */
void btGenFrameByFrameOptimizer::CalcTargetInternal(const tVectorXd &solution,
                                                    tVectorXd &qddot,
                                                    tVectorXd &qdot,
                                                    tVectorXd &q,
                                                    tVectorXd &tau)
{
    if (solution.size() != mTotalSolutionSize)
    {
        std::cout << "[error] solution size doesn't match\n";
        exit(0);
    }
    tVectorXd contact_force = solution.segment(0, mContactSolutionSize);
    tVectorXd control_force =
        solution.segment(mContactSolutionSize, mCtrlSolutionSize);

    // 1. calculate the gen contact force and gen control force
    tVectorXd gen_contact_force = tVectorXd::Zero(num_of_freedom);
    tVectorXd gen_ctrl_force = tVectorXd::Zero(num_of_freedom);

    // std::cout << "contact point size = " << mContactPoints.size() <<
    // std::endl; std::cout << "[solved] raw contact solution = " <<
    // contact_force.transpose() << std::endl; std::cout << "[solved] raw tau
    // solution = " << control_force.transpose() << std::endl;
    for (int i = 0; i < mContactPoints.size(); i++)
    {
        auto &pt = mContactPoints[i];
        int offset = mContactSolOffset[i];
        int size = mContactSolSize[i];
        tVector3d solved_force = pt->mS * contact_force.segment(offset, size);
        std::cout << "[solved] FBF contact force " << i << " "
                  << solved_force.transpose() << std::endl;
        gen_contact_force += pt->mJac.transpose() * solved_force;
    }
    gen_ctrl_force.segment(6, num_of_underactuated_freedom) = control_force;
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    tVectorXd RHS = gen_contact_force + gen_ctrl_force + QG -
                    mModel->GetCoriolisMatrix() * mModel->Getqdot();
    // std::cout << gen_ctrl_force.size() << std::endl;
    // std::cout << gen_contact_force.size() << std::endl;
    // std::cout << RHS.size() << std::endl;
    // 2. calculate the generated accel
    qddot = mModel->GetInvMassMatrix() * RHS;
    tau = control_force;

    // 3. calculate the target vel and pos
    qdot = mModel->Getqdot() + qddot * mdt;
    q = mModel->Getq() + qdot * mdt;

    // output the contact force info in the ref traj
    for (auto &f : mTraj->mContactForce[mCurFrameId])
    {
        std::cout << "[ref] link "
                  << dynamic_cast<btGenRobotCollider *>(f->mObj)->mLinkId
                  << " force = " << f->mForce.transpose() << std::endl;
    }
    // calculate the static contact point vel
    {
        for (auto &pt : mContactPoints)
        {
            if (pt->mStatus == eContactStatus::STATIC)
            {
                tVector3d vel = pt->mJac * qdot;
                std::cout << "[vel] static contact point " << pt->contact_id
                          << " vel = " << vel.transpose() << std::endl;
            }
        }
    }
}

void btGenFrameByFrameOptimizer::ParseConfig(const Json::Value &conf)
{
    // mEnableHardConstraintForDynamics =
    //     btJsonUtil::ParseAsBool("enable_hard_dynamic_constraint", conf);
    // mUseNativeRefTarget =
    //     btJsonUtil::ParseAsBool("use_native_reference_target", conf);
    // mDynamicPosEnergyCoeff =
    //     btJsonUtil::ParseAsDouble("dynamic_pos_energy_coef", conf);
    // mDynamicVelEnergyCoeff =
    //     btJsonUtil::ParseAsDouble("dynamic_vel_energy_coef", conf);
    // mDynamicAccelEnergyCoeff =
    //     btJsonUtil::ParseAsDouble("dynamic_accel_energy_coef", conf);
}
void btGenFrameByFrameOptimizer::InitModelInfo()
{
    num_of_freedom = mModel->GetNumOfFreedom();
    num_of_underactuated_freedom = num_of_freedom - 6;
}
/**
 * \brief					Constrcut the Frame by frame
 * optimization problem
 */
void btGenFrameByFrameOptimizer::InitQPSolver()
{
    mQPSolver = new QuadProgQPSolver();
    // matlabQPSolver = new MatlabQPSolver();

    // std::cout << "init qp solve done\n";
    // exit(0);
}

void btGenFrameByFrameOptimizer::ClearContactPoints()
{
    for (auto &data : mContactPoints)
        delete data;
    mContactPoints.clear();
    mContactSolOffset.clear();
    mContactSolSize.clear();
}

/**
 * \brief				Calculate solution size, the offset
 * w.r.t each contact point
 *
 * 		solution vector = [contact_force_vector, active_ctrl_force]
 */
void btGenFrameByFrameOptimizer::CalcSolutionVector()
{
    mContactSolutionSize = 0;
    mCtrlSolutionSize = 0;
    mTotalSolutionSize = 0;
    int num_of_contacts = mContactPoints.size();
    // int num_of_freedom = mModel->GetNumOfFreedom();
    // int num_of_underactuated_freedom = num_of_freedom - 6;
    mContactSolOffset.resize(num_of_contacts, 0);
    mContactSolSize.resize(num_of_contacts, 0);
    // 1. init the result vector: contact forces + control forces (N-6)
    int offset = 0;

    for (int i = 0; i < num_of_contacts; i++)
    {
        auto pt = mContactPoints[i];

        int size = GetSolutionSizeByContactStatus(pt->mStatus);
        mContactSolSize[i] = size;
        mContactSolOffset[i] = offset;
        offset += size;
        mContactSolutionSize += size;
    }

    mCtrlSolutionSize += num_of_underactuated_freedom;

    mTotalSolutionSize = mCtrlSolutionSize + mContactSolutionSize;
    // std::cout << "[debug] contact num " << num_of_contacts
    //           << ", total solution size " << mTotalSolutionSize << std::endl;
}

/**
 * \brief               Given the num of friction direction, calculate the
 * friction cone which is perpendicular to the ground plance
 */
tMatrixXd CalcFrictionCone(int friction_num)
{
    assert(friction_num >= 3);
    tMatrixXd dir_mat = tMatrixXd::Zero(3, friction_num);
    double unit = 2 * M_PI / friction_num;
    for (int i = 0; i < friction_num; i++)
    {
        dir_mat(0, i) = std::cos(unit * i);
        dir_mat(2, i) = std::sin(unit * i);
    }

    // round to zero in order to get a clean output
    for (int i = 0; i < dir_mat.rows(); i++)
        for (int j = 0; j < dir_mat.cols(); j++)
        {
            if (std::fabs(dir_mat(i, j)) < 1e-10)
                dir_mat(i, j) = 0;
        }
    return dir_mat;
}

/**
 * \brief               Given a contact point, calculate the convert matrix from
 * solution to cartesian force
 */
void btGenFrameByFrameOptimizer::CalcContactConvertMat(btCharContactPt *contact,
                                                       tMatrixXd &convert_mat)
{
    int size = GetSolutionSizeByContactStatus(contact->mStatus);
    switch (contact->mStatus)
    {
    case eContactStatus::SLIDING:
    {
        convert_mat.resize(3, size);
        convert_mat.col(0) = tVector3d(0, 1, 0);
        convert_mat.block(0, 1, 3, mNumOfFrictionDirs) =
            CalcFrictionCone(mNumOfFrictionDirs);
    }
    break;
    case eContactStatus::STATIC:
    {
        convert_mat.resize(3, size);
        convert_mat.col(0) = tVector3d(0, 1, 0);
        convert_mat.block(0, 1, 3, mNumOfFrictionDirs) =
            CalcFrictionCone(mNumOfFrictionDirs);
    }
    break;
    case eContactStatus::BREAKAGE:
    {
        convert_mat.noalias() = tMatrix3d::Identity();
    }
    break;
    default:
        std::cout << "[error] contact status " << contact->mStatus << std::endl;
        break;
    }
}

/**
 * \brief               Given the contact status, return the solution size
 */
int btGenFrameByFrameOptimizer::GetSolutionSizeByContactStatus(
    eContactStatus status)
{
    if (status == eContactStatus::SLIDING)
    {
        return mNumOfFrictionDirs + 1;
    }
    else if (status == eContactStatus::BREAKAGE)
    {
        return 3;
    }
    else if (status == eContactStatus::STATIC)
    {
        return mNumOfFrictionDirs + 1;
    }
    std::cout << "[error] Unsupported contact status " << status << std::endl;
    exit(0);
    return 0;
}

/**
 * \brief               If the optimizer's guide trajectory has finished, we
 * need to reset it and give another traj
 */
void btGenFrameByFrameOptimizer::Reset()
{
    // std::cout << "FBF optimizer reset\n";
}

void btGenFrameByFrameOptimizer::SetTraj(btTraj *traj) { this->mTraj = traj; }
