#include "btGenFrameByFrameOptimizer.h"
#include "BulletGenDynamics/btGenController/FBFOptimizer/btGenFBFConstraint.h"
#include "BulletGenDynamics/btGenController/FBFOptimizer/btGenFBFEnergyTerm.h"
#include "BulletGenDynamics/btGenController/QPSolver/MatlabQPSolver.h"
#include "BulletGenDynamics/btGenController/QPSolver/QuadProgQPSolver.h"
#include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenSolver/ConstraintData.h"
#include "BulletGenDynamics/btGenSolver/ContactSolver.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
static int mNumOfFrictionDirs = 4;

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

struct btCharContactPt : public btGenContactPointData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btCharContactPt(int c_id) : btGenContactPointData(c_id)
    {
        mCollider = nullptr;
        mLocalPos = tVector::Zero();
        mWorldPos = tVector::Zero();
        mJac.resize(0, 0);
        mStatus = eContactStatus::INVALID_CONTACT_STATUS;
    }
    void Init(double dt, btPersistentManifold *manifold,
              int contact_id_in_manifold)
    {
        btGenContactPointData::Init(dt, manifold, contact_id_in_manifold);

        if (mIsSelfCollision == true)
        {
            std::cout << "[error] CalcContactStatus cannot handle self "
                         "collsion at this moment\n";
            // exit(0);
        }
    }
    bool IsMultibodyInvolved(cRobotModelDynamics *model)
    {
        bool involved = false;
        if (eColObjType::RobotCollder == mBodyA->GetType())
        {
            involved |=
                dynamic_cast<btGenRobotCollider *>(mBodyA)->mModel == model;
        }
        else if (eColObjType::RobotCollder == mBodyB->GetType())
        {
            involved |=
                dynamic_cast<btGenRobotCollider *>(mBodyB)->mModel == model;
        }
        return involved;
    }
    void CalcCharacterInfo()
    {
        if (eColObjType::RobotCollder == mBodyA->GetType())
        {
            mCollider = dynamic_cast<btGenRobotCollider *>(mBodyA);
            mWorldPos = mContactPtOnA;
        }

        else if (eColObjType::RobotCollder == mBodyB->GetType())
        {
            mCollider = dynamic_cast<btGenRobotCollider *>(mBodyB);
            mWorldPos = mContactPtOnB;
        }

        mWorldPos[3] = 1;
        auto link = mCollider->mModel->GetLinkById(mCollider->mLinkId);
        mLocalPos = btMathUtil::InverseTransform(link->GetGlobalTransform()) *
                    mWorldPos;

        mCollider->mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
            mCollider->mLinkId, mWorldPos.segment(0, 3), mJac);
        // std::cout << "world pos = " << mWorldPos.transpose() << std::endl;
        // std::cout << "local pos = " << mLocalPos.transpose() << std::endl;
    }

    btGenRobotCollider *mCollider;
    tVector mLocalPos;
    tVector mWorldPos;
    tMatrixXd mJac;
    eContactStatus mStatus;
};

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

    // {
    // 	std::cout << "H = \n"
    // 			  << H << std::endl;
    // 	std::cout << "f = " << f.transpose() << std::endl;
    // 	std::cout << "Aeq = \n"
    // 			  << Aeq << std::endl;
    // 	std::cout << "beq = " << beq.transpose() << std::endl;
    // 	std::cout << "Aieq = \n"
    // 			  << Aineq << std::endl;
    // 	std::cout << "bieq = " << bineq.transpose() << std::endl;
    // 	// exit(0);
    // }
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
    if (mEnergyTerm != nullptr)
        delete mEnergyTerm;
    mEnergyTerm = new btGenFrameByFrameEnergyTerm(mTotalSolutionSize);

    // if (mEnableHardConstraintForDynamics == false)
    // {

    // }
    AddDynamicEnergyTerm();
    AddMinTauEnergyTerm();
    AddMinContactForceEnergyTerm();
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
    {
        AddFixStaticContactPointConstraint();
        // std::cout << "fix static contact point is enabled\n";
        // exit(1);
    }
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
        mConstraint->AddIneqCon(jac, res, offset);
    }
    // 2. equalities make ft = mu * fn
    {
        tMatrixXd jac = tMatrixXd::Zero(1, size);
        jac(0, 0) = mu;
        jac.block(0, 1, 1, size - 1).fill(-1);
        mConstraint->AddEqCon(jac, tVectorXd::Zero(1), offset);
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
    mConstraint->AddIneqCon(jac, residual, offset);
}

/**
 * \brief               Given one contact point info, add the "breakage" contact
 * constraint into the opt problem
 */
void btGenFrameByFrameOptimizer::AddBreakageConstraint(btCharContactPt *pt)
{
    int offset = mContactSolOffset[pt->contact_id];
    tMatrixXd jac = tMatrix3d::Identity();
    mConstraint->AddEqCon(jac, tVector3d::Zero(), offset);
}

/**
 * \brief				Close to Origin energy term in this
 * frame by frame control q, qdot, qddot
 */
void btGenFrameByFrameOptimizer::AddDynamicEnergyTerm()
{
    AddDynamicEnergyTermPos();
    AddDynamicEnergyTermVel();
    AddDynamicEnergyTermAccel();
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
    const tVectorXd &q_next_ref = mTraj->mq[mCurFrameId + 1];
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
    mConstraint->AddEquivalentEqCon(A, b, 0, 1e-12);
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
    tMatrixXd A =
        tMatrixXd::Identity(mContactSolutionSize, mContactSolutionSize);
    tVectorXd b = tVectorXd::Zero(mContactSolutionSize);
    mEnergyTerm->AddEnergy(A, b, mContactForceCoef, 0, "contact_force");
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
    const tVectorXd &q_next_ref = mTraj->mq[mCurFrameId + 1];
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
    A /= dt2;
    b /= dt2;

    // energy term Ax + b
    mEnergyTerm->AddEnergy(A, b, mDynamicPosEnergyCoeff, 0, "pos");
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
    const tVectorXd &qdot_next_ref = mTraj->mq[mCurFrameId + 1];
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
    A /= mdt;
    b /= mdt;
    mEnergyTerm->AddEnergy(A, b, mDynamicVelEnergyCoeff, 0, "vel");
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
    const tVectorXd &qddot_next_ref = mTraj->mq[mCurFrameId];
    // double dt2 = mdt * mdt;
    tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
    b = Minv * (QG - C_d * qdot) + qdot - qddot_next_ref;
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
    A *= mDynamicAccelEnergyCoeff;
    b *= mDynamicAccelEnergyCoeff;
    mEnergyTerm->AddEnergy(A, b, mDynamicAccelEnergyCoeff, 0, "accel");
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
            mConstraint->AddEquivalentEqCon(pt->mJac * A_base,
                                            pt->mJac * b_base, 0, 1e-12);
        }
    }
}