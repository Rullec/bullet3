#include "ContactSolver.h"
// #include "QPSolver/QPSolver.h"
// #include "MatlabQPSolver/MatlabQPSolver.h"
#include "BulletGenDynamics/btGenController/ContactAwareController/btGenContactAwareController.h"
#include "BulletGenDynamics/btGenModel/RobotCollider.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenModel/SimObj.h"
#include "BulletGenDynamics/btGenSolver/LCPSolverBuilder.hpp"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenUtil/TimeUtil.hpp"
#include "btBulletDynamicsCommon.h"
#include <iostream>
#include <set>
// const std::string rigidbody_path = "rigidbody.txt";
// const std::string multibody_path = "multibody.txt";

// int contact_times = 0;
extern int global_frame_id;
btGenRigidBody *UpcastRigidBody(const btCollisionObject *col)
{
    return const_cast<btGenRigidBody *>(
        dynamic_cast<const btGenRigidBody *>(col));
}

btGenRobotCollider *UpcastRobotCollider(const btCollisionObject *col)
{
    return const_cast<btGenRobotCollider *>(
        dynamic_cast<const btGenRobotCollider *>(col));
}
btGenCollisionObject *UpcastColObj(const btCollisionObject *col)
{
    return const_cast<btGenCollisionObject *>(
        dynamic_cast<const btGenCollisionObject *>(col));
}

btGenContactForce::btGenContactForce(btGenCollisionObject *obj_,
                                     btGenCollisionObject *passive_obj,
                                     const tVector &f_, const tVector &p_,
                                     bool is_self_collision)
{
    mObj = obj_;
    mPassiveObj = passive_obj;
    mForce = f_;
    mWorldPos = p_;
    if (std::fabs(mWorldPos[3] - 1) > 1e-10)
    {
        std::cout << "[error] btGenContactForce::btGenContactForce the world "
                     "pos is not homogenous = "
                  << mWorldPos.transpose() << std::endl;
        exit(1);
    }
    mIsSelfCollision = is_self_collision;
}

btGenMBContactForce::btGenMBContactForce(btGenRobotCollider *collider,
                                         btGenCollisionObject *passive_obj,
                                         const tVector &f,
                                         const tVector &world_pos,
                                         const tVector &local_pos,
                                         bool is_self_collision)
    : btGenContactForce(collider, passive_obj, f, world_pos, is_self_collision)
{
    mLinkId = collider->mLinkId;
    mLocalPos = btMathUtil::Expand(local_pos, 1);
}
// const std::string log_path = "debug_solve_new.txt";
btGenContactSolver::btGenContactSolver(const std::string &config_path,
                                       btGeneralizeWorld *world)
{
    // std::ofstream fout(log_path);
    // fout << "";
    // fout.close();
    Json::Value config;
    btJsonUtil::LoadJson(config_path, config);

    mNumFrictionDirs = btJsonUtil::ParseAsInt("num_of_friction_cone", config);
    mMu = btJsonUtil::ParseAsFloat("mu", config);
    mEnableLCPProfiling =
        btJsonUtil::ParseAsBool("enable_lcp_profiling", config);
    cur_dt = 0;
    mEnableMultibodySelfCol =
        btJsonUtil::ParseAsBool("enable_multibody_self_collision", config);
    mEnableConvertMatTest =
        btJsonUtil::ParseAsBool("enable_convert_mat_test", config);
    mEnableContactPenetrationResolve =
        btJsonUtil::ParseAsBool("enable_contact_penetration_resolve", config);
    mEnableJointLimitPenetrationResolve = btJsonUtil::ParseAsBool(
        "enable_joint_limit_penetration_resolve", config);
    mContactErp =
        btJsonUtil::ParseAsDouble("contact_penetration_resolve_erp", config);
    mJointLimitErp =
        btJsonUtil::ParseAsDouble("contact_penetration_resolve_erp", config);

    // lcp related config
    mEnableLCPCalc = btJsonUtil::ParseAsBool("enable_lcp_calculation", config);
    mEnableContactLCP = btJsonUtil::ParseAsBool("enable_contact_lcp", config);

    mEnableFrictionalLCP =
        btJsonUtil::ParseAsBool("enable_frictional_lcp", config);
    mEnableJointLimitLCP =
        btJsonUtil::ParseAsBool("enable_joint_limit_lcp", config);
    mEnableGradientOverCtrlForce =
        btJsonUtil::ParseAsBool("enable_gradient_over_control_force", config);
    mEnableTestGradientOverCtrlForce = btJsonUtil::ParseAsBool(
        "enable_test_gradient_over_control_force", config);

    mUseLCPResult = btJsonUtil::ParseAsBool("use_lcp_result", config);

    // sequential impulse config
    mEnableSICalc = btJsonUtil::ParseAsBool(
        "enable_sequential_impulse_calculation", config);
    mEnableFrictionalSI =
        btJsonUtil::ParseAsBool("enable_frictional_sequential_impulse", config);
    mUseSIResult =
        btJsonUtil::ParseAsBool("use_sequential_impulse_result", config);
    mMaxItersSI =
        btJsonUtil::ParseAsInt("max_iters_sequential_impulse", config);
    mConvergeThresholdSI = btJsonUtil::ParseAsDouble(
        "converge_threshold_sequential_impulse", config);
    mEnableSILCPComparision =
        btJsonUtil::ParseAsBool("enable_si_lcp_comparision", config);
    mDiagonalEps = btJsonUtil::ParseAsDouble("diagonal_eps", config);

    // other stuff
    mEnableDebugOutput = btJsonUtil::ParseAsBool("enable_debug_output", config);
    mGenWorld = world;
    mWorld = world->GetInternalWorld();
    mLCPSolver =
        BuildLCPSolver(btJsonUtil::ParseAsString("lcp_solver_type", config));

    bool err = false;
    if (mMu < 0.1)
    {
        std::cout << "cContactSolver mMu should not be " << mMu << std::endl;
        err = true;
    }

    // make the "use_result_option" mutally exclusive
    if (mUseSIResult == mUseLCPResult && mUseLCPResult == true)
    {
        std::cout << "ContactSolver: Use LCP and SI config at the same time, "
                     "illegal\n";
        err = true;
    }

    // make sure that when we use a result, it must be calculated
    if ((mUseLCPResult == true && mEnableLCPCalc == false) ||
        (mUseSIResult == true && mEnableSICalc == false))
    {
        std::cout << "Use LCP Result but its calculation is disabled\n";
        std::cout << "Or use SI Result but its calculation is disabled\n";
        err = true;
    }

    // when we want to compare, we must compute
    if (mEnableSILCPComparision == true)
    {
        if (false == (mEnableLCPCalc && mEnableSICalc))
        {
            std::cout << "SI & LCP comparision is enabled but they are not "
                         "calculated both in advance\n";
            err = true;
        }
    }
    if (err)
        exit(0);

    contact_force_array.clear();
    map_colobjid_to_groupid.clear();
    mColGroupData.clear();
    mContactConstraintData.clear();
    mJointLimitConstraintData.clear();
    mMultibodyArray.clear();
    mNumContactPoints = 0;
    mNumJointLimitConstraints = 0;
}

btGenContactSolver::~btGenContactSolver()
{
    if (mLCPSolver)
        delete mLCPSolver;

    ClearConstraintForceTorqueArrays(contact_force_array, contact_torque_array);
    DeleteColObjData();
}

void btGenContactSolver::Reset()
{
    DGenConsForceDGenCtrlForce.resize(0, 0);
    ClearConstraintForceTorqueArrays(contact_force_array, contact_torque_array);
    DeleteColObjData();
    DeleteConstraintData();
}
/**
 * \brief	 return the contact forces. col_id->vector[pair<ColObj*, force>]
 */
void btGenContactSolver::ConstraintProcess(float dt_)
{
    // std::cout << "--------contact solver frame " << global_frame_id << "----------" << std::endl;
    // 1. reset the solver
    cur_dt = dt_;
    Reset();

    // btTimeUtil::Begin("constraint_setup");
    // 2. if there is no constraint return directly
    if (false == ConstraintSetup())
        return;

    // 3. else, begin to solve the constraint and calculate the reaction forces
    // btTimeUtil::End("constraint_setup");
    // btTimeUtil::Begin("constraint_solve");
    ConstraintSolve();
    // btTimeUtil::End("constraint_solve");
    // btTimeUtil::Begin("constraint_finished");
    ConstraintFinished();
    // btTimeUtil::End("constraint_finished");
}

std::vector<btGenContactForce *> btGenContactSolver::GetContactForces()
{
    return contact_force_array;
}
std::vector<btGenConstraintGeneralizedForce *>
btGenContactSolver::GetConstraintGeneralizedForces()
{
    return contact_torque_array;
}

/**
 * \brief           d(gen_constraint_force)/d(gen_control_force)
*/
tMatrixXd btGenContactSolver::GetDGenConsForceDCtrlForce() const
{
    BTGEN_ASSERT(mEnableGradientOverCtrlForce == true);
    return this->DGenConsForceDGenCtrlForce;
}

/**
 * \brief       finished the constraint solving
*/
void btGenContactSolver::ConstraintFinished()
{
    // 1. clear all contact force & torques array
    ClearConstraintForceTorqueArrays(contact_force_array, contact_torque_array);

    // 2. determine the valid contact solution vector (from sequential impulse or lcp?)
    tVectorXd contact_x;
    if (mUseSIResult)
        contact_x = x_si;
    else if (mUseLCPResult)
        contact_x = x_lcp;

    // 3. calculate the cartesian & generalized contact torque / forces
    contact_x = ConvertLCPResult(contact_x);
    CalcContactForceTorqueArrays(contact_force_array, contact_torque_array,
                                 contact_x);

    // 4. test some utilities
    if (mEnableGradientOverCtrlForce == true)
    {
        if (mEnableTestGradientOverCtrlForce)
        {
            TestDnDCtrlForce();
            TestDxDCtrlForce();
            TestDGenConsForceDx();
            TestDGenConsForceDCtrlForce();
            // exit(0);
        }
        // CalcDGenConsForceDx();
        DGenConsForceDGenCtrlForce = CalcDGenConsForceDCtrlForce();
        BTGEN_ASSERT(DGenConsForceDGenCtrlForce.hasNaN() == false);
    }

    // 5. clear the collision object and constraint info
    DeleteColObjData();
    DeleteConstraintData();
}

/**
 * \brief        Set up the LCP contact constraint
 *          Calculate the LCP constraint tMatrixXd A and vec b, so that:
 *                  0 <= x \cdot (Ax + b) >= 0
 */
bool btGenContactSolver::ConstraintSetup()
{
    // 1. build collision objects data
    RebuildColObjData();

    // 2. get all contact points' info
    if (mEnableContactLCP)
    {
        btDispatcher *dispatcher = mWorld->getDispatcher();
        int n_manifolds = dispatcher->getNumManifolds();
        for (int i = 0; i < n_manifolds; i++)
        {
            const auto &manifold = dispatcher->getManifoldByIndexInternal(i);
            AddManifold(manifold);
        }
    }

    // 3. add joint limit constraint if possible
    if (mEnableJointLimitLCP)
        AddJointLimit();

    mNumConstraints = mNumJointLimitConstraints + mNumContactPoints;
    // std::cout << "constraint numbers = " << mNumContactPoints << std::endl;
    // if here is a contact
    int self_contact_size = 0;
    for (int i = 0; i < mNumContactPoints; i++)
    {
        if (mContactConstraintData[i]->mIsSelfCollision)
            self_contact_size++;
    }

    if (mNumContactPoints + mNumJointLimitConstraints != 0)
    {
        // 3. calculate velocity convert matrix for each contact point (united
        // by collision groups) This convert matrix are shared by LCP and SI
        CalcAbsvelConvertMat();
        if (mEnableConvertMatTest)
            TestCartesianForceToCartesianVel();
        // exit(0);
        // the following calculation can only used in LCP solver, discard when
        // LCP is disabled
        if (mEnableLCPCalc)
        {
            // 4. calculate the relative velocity convert matrix
            CalcRelvelConvertMat();
            if (mEnableConvertMatTest)
                TestCartesianForceToCartesianRelVel(rel_vel_convert_mat,
                                                    rel_vel_convert_vec);

            // 5. calculate the relative normal/tangent convert matrix
            CalcDecomposedConvertMat();
            if (mEnableConvertMatTest)
                TestCartesianForceToNormalAndTangetRelVel(
                    normal_vel_convert_mat, normal_vel_convert_vec,
                    tangent_vel_convert_mat, tangent_vel_convert_vec);
            // if (mNumContactPoints > 0) exit(0);
            // 6. change base for decomposed convert matrix: the original base
            // is cartesian force, now it is [fn, f_4dirs, \lambda]
            CalcResultVectorBasedConvertMat();
            if (mEnableConvertMatTest)
                TestCartesianForceToNormalAndTangetResultBasedRelVel(
                    normal_vel_convert_result_based_mat,
                    normal_vel_convert_result_based_vec,
                    tangent_vel_convert_result_based_mat,
                    tangent_vel_convert_result_based_vec);
        }
        if (mEnableConvertMatTest == true)
        {
            printf("[log] LCP Convert Mat verified succ\n");
        }
    }

    return mNumConstraints != 0;
}
extern std::map<int, std::string> col_name;
void btGenContactSolver::ConstraintSolve()
{
    if (mNumConstraints == 0)
        return;

    // temporary disable the lcp
    if (mEnableLCPCalc)
    {
        SolveByLCP();
    }

    // begin to do SI test
    if (mEnableSICalc)
    {
        SolveBySI();
    }
}

// extern std::string gOutputLogPath;
void btGenContactSolver::SolveByLCP()
{
    // if (mNumFrictionDirs != 4)
    // {
    // 	std::cout << "error now friction dirs must be 4, otherwise the convert
    // tMatrixXd S is illegal\n";
    // 	// exit(0);
    // }

    // construct a LCP problem: (x * Mx+n) = 0, x>=0, Mx+n >=0
    // 1. shape the vecs and mats:
    int single_size = 1;

    // if fricional LCP is enabled
    if (mEnableFrictionalLCP)
        single_size += mNumFrictionDirs + 1;

    int final_shape =
        mNumContactPoints * single_size + mNumJointLimitConstraints;
    M.resize(final_shape, final_shape), M.setZero();
    x_lcp.resize(final_shape), x_lcp.setZero();
    n.resize(final_shape), n.setZero();

    int multibody_dof = 0;
    if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
    {
        multibody_dof = mMultibodyArray[0]->GetNumOfFreedom();
        n_convert_mat_final.resize(final_shape, multibody_dof);
    }
    // 2. fill in
    tMatrixXd unit_M(single_size, single_size);
    tVectorXd unit_n(single_size);
    tMatrixXd C_lambda(mNumFrictionDirs, single_size), C_mufn(1, single_size),
        C_c(1, single_size);
    CalcCMats(C_lambda, C_mufn, C_c);

    // 3. construct LCP problem
    tMatrixXd line_M(single_size, final_shape);
    tVectorXd line_n(single_size);
    tMatrixXd dline_n_dctrl_force(single_size, multibody_dof);

    for (int i = 0; i < mNumContactPoints; i++)
    {
        line_M.setZero();
        line_n.setZero();

        dline_n_dctrl_force.setZero();

        // 3.1 basic M and basic n
        line_M.block(0, 0, 1, final_shape) =
            normal_vel_convert_result_based_mat.block(i, 0, 1, final_shape);
        line_n.segment(0, 1) =
            normal_vel_convert_result_based_vec.block(i, 0, 1, 1);

        if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
        {
            dline_n_dctrl_force.block(0, 0, 1, multibody_dof) =
                normal_vel_convert_mat_result_based_of_residual.block(
                    i, 0, 1, multibody_dof);
        }

        // handle the constraint-drifting
        if (mEnableContactPenetrationResolve == true)
        {
            double penetration = mContactConstraintData[i]->distance;
            if (penetration < 0)
            {
                // std::cout << "[debug] normal contact point " << i
                //           << " penetration " << penetration << std::endl;
                line_n[0] += penetration / cur_dt * mContactErp;
            }
        }

        // 3.2 add C-related terms and frictional case
        if (mEnableFrictionalLCP)
        {
            line_M.block(1, 0, mNumFrictionDirs, final_shape) =
                tangent_vel_convert_result_based_mat.block(
                    i * mNumFrictionDirs, 0, mNumFrictionDirs, final_shape);
            line_n.segment(1, mNumFrictionDirs) =
                tangent_vel_convert_result_based_vec.segment(
                    i * mNumFrictionDirs, mNumFrictionDirs);
            line_M.block(1, i * single_size, mNumFrictionDirs, single_size) +=
                C_lambda;
            line_M.block(single_size - 1, i * single_size, 1, single_size) +=
                C_mufn - C_c;

            if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
            {
                dline_n_dctrl_force.block(1, 0, mNumFrictionDirs,
                                          multibody_dof) =
                    tangent_vel_convert_mat_result_based_of_residual.block(
                        i * mNumFrictionDirs, 0, mNumFrictionDirs,
                        multibody_dof);
            }
        }

        // 3.3 give line M and line n to total M and n
        M.block(i * single_size, 0, single_size, final_shape) = line_M;
        n.segment(i * single_size, single_size) = line_n;

        if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
        {
            n_convert_mat_final.block(i * single_size, 0, single_size,
                                      multibody_dof) = dline_n_dctrl_force;
        }
    }

    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        M.block(mNumContactPoints * single_size + i, 0, 1, final_shape) =
            normal_vel_convert_result_based_mat.block(mNumContactPoints + i, 0,
                                                      1, final_shape);
        n[mNumContactPoints * single_size + i] =
            normal_vel_convert_result_based_vec[mNumContactPoints + i];
        if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
        {
            n_convert_mat_final.row(mNumContactPoints * single_size) =
                normal_vel_convert_mat_result_based_of_residual.row(
                    mNumContactPoints + i);
        }

        // solve the joint limit constraint drifting
        if (mEnableJointLimitPenetrationResolve == true)
        {
            n[mNumContactPoints * single_size + i] +=
                mJointLimitConstraintData[i]->violate_value * mJointLimitErp;
            if (mEnableDebugOutput == true)
            {
                std::cout << "[debug] for joint limit cons " << i
                          << " add violate erp = "
                          << mJointLimitConstraintData[i]->violate_value *
                                 mJointLimitErp
                          << std::endl;
            }
        }
    }
    // std::cout << "[lcp] LCP M cond num = " <<
    // cMathUtil::CalcConditionNumber(M) << std::endl; add small values to
    // diagnal to keep it away from singular, similar to cfm varaible in ODE as
    // RTQL8 said std::cout << "[debug] add diagnoal eps " << mDiagonalEps << "
    // on LCP M\n";
    // for (int i = 0; i < M.rows(); i++)
    // {
    //     M(i, i) *= (1 + mDiagonalEps);
    // }
    M += tMatrixXd::Identity(M.rows(), M.cols()) * mDiagonalEps;
    if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
    {
        n_convert_mat_final *= cur_dt;
    }
    // std::cout << "[lcp] After M cond num = " <<
    // cMathUtil::CalcConditionNumber(M) << std::endl; std::cout << "begin to do
    // lcp solveing\n"; std::ofstream fout(log_path, std::ios::app); fout <<
    // "----------------frame " << global_frame_id << std::endl; fout << "q = "
    // << mMultibodyArray[0]->Getq().transpose() << std::endl; fout << "qdot = "
    // << mMultibodyArray[0]->Getqdot().transpose() << std::endl; fout << "M =
    // \n"
    // 	 << M << std::endl;
    // fout << "n = " << n.transpose() << std::endl;
    // btTimeUtil::Begin("MLCPSolver->Solver");
    if (mLCPSolver->GetType() == eLCPSolverType::ODEDantzig)
    {
        static_cast<cODEDantzigLCPSolver *>(mLCPSolver)
            ->SetInfo(mNumFrictionDirs, this->mMu, mNumContactPoints,
                      mNumJointLimitConstraints, mEnableFrictionalLCP);
    }
    if (mEnableLCPProfiling == true)
        btTimeUtil::Begin("lcp_solver");
    int ret = mLCPSolver->Solve(x_lcp.size(), M, n, x_lcp);
    if (mEnableLCPProfiling == true)
        btTimeUtil::End("lcp_solver");
    // std::cout << "[lcp] M norm = " << M.norm() << std::endl;
    // std::cout << "[lcp] n norm = " << n.norm() << std::endl;
    // std::cout << "[lcp] x = " << x_lcp.transpose() << std::endl;
    // btTimeUtil::End("MLCPSolver->Solver");
    // fout << "x = " << x_lcp.transpose() << std::endl;
    // std::cout << "x_lcp = " << x_lcp.transpose() << std::endl;
    // std::cout << "M*x+n = " << (M * x_lcp + n).transpose() << std::endl;
    // fout.close();
    // if (global_frame_id == 60) exit(0);
    // std::cout << "[lcp] x_lcp = " << x_lcp.transpose() << std::endl;
    // std::cout << "[lcp] M norm = " << M.norm() << std::endl;
    // std::cout << "[lcp] n norm = " << n.norm() << std::endl;
    // std::cout << "[lcp] q = " << mMultibodyArray[0]->Getq().transpose() <<
    // std::endl; std::cout << "[lcp] qdot = " <<
    // mMultibodyArray[0]->Getqdot().transpose() << std::endl; exit(0);
    // std::cout << "end to do lcp solveing\n";
    // std::ofstream fout(gOutputLogPath, std::ios::app);
    // fout << "rel vel convert mat = \n"
    // 	 << rel_vel_convert_mat << std::endl;
    // fout << "rel vel convert vec = "
    // 	 << rel_vel_convert_vec.transpose() << std::endl;
    // fout << "M = \n"
    // 	 << M << std::endl;
    // fout << "n = " << n.transpose() << std::endl;
    // std::cout << "[lcp] x_lcp = " << x_lcp.transpose() << std::endl;
    // fout.close();
    if (ret != 0)
    {
        std::cout << "[error] bt LCP solved failed, ret !=0 \n";
    }
    if (x_lcp.hasNaN() == true)
    {
        std::cout << "[lcp] constraint force has Nan = " << x_lcp.transpose()
                  << std::endl;
        std::cout << "M=\n" << M << std::endl;
        std::cout << "n=\n" << n.transpose() << std::endl;
    }
    // check whether our guess is true？
    // VerifySolution();
}

/**
 * \brief			convert the result vector of LCP problem into a readable format
 */
tVectorXd
btGenContactSolver::ConvertLCPResult(const tVectorXd &raw_lcp_result) const
{
    int single_contact_size = 1;
    if (mEnableFrictionalLCP)
        single_contact_size += mNumFrictionDirs + 1;

    tVectorXd new_lcp_result =
        tVectorXd::Zero(mNumContactPoints * 3 + mNumJointLimitConstraints);
    double total_contact_force_norm = 0;

    // 1. extract all contact force
    for (int i = 0; i < mNumContactPoints; i++)
    {
        if (0 == i && mEnableDebugOutput)
        {
            std::cout << "[lcp] num contact pt = " << mNumContactPoints
                      << std::endl;
            std::cout << "[lcp] x = " << raw_lcp_result.transpose()
                      << std::endl;
        }
        auto &data = mContactConstraintData[i];
        tVectorXd x_unit = raw_lcp_result.segment(i * single_contact_size,
                                                  single_contact_size);
        tVector contact_force = tVector::Zero();
        if (mEnableFrictionalLCP)
        {
            contact_force = btMathUtil::Expand((data->mS * x_unit), 0);
        }
        else
        {
            contact_force = data->mNormalPointToA * x_unit[0];
        }
        // std::cout << "[debug] contact " << i << " result vector = " <<
        // x_unit.transpose() << std::endl; std::cout << "[debug] contact " << i
        // << " cartesian force = " << contact_force.transpose() << std::endl;
        new_lcp_result.segment(i * 3, 3) = contact_force.segment(0, 3);
        if (mEnableDebugOutput)
            std::cout << "[lcp] contact " << i
                      << " force = " << contact_force.transpose() << std::endl;
        total_contact_force_norm += contact_force.norm();
    }

    // 2. extract all constraint (joint limit) constraint force
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        double generalized_force =
            raw_lcp_result[mNumContactPoints * single_contact_size + i];
        if (data->is_upper_bound)
            generalized_force *= -1;
        new_lcp_result[mNumContactPoints * 3 + i] = generalized_force;
    }
    return new_lcp_result;
}

void btGenContactSolver::CalcCMats(tMatrixXd &C_lambda, tMatrixXd &C_mufn,
                                   tMatrixXd &C_c)
{
    int single_size = (mNumFrictionDirs + 2);
    C_lambda.resize(mNumFrictionDirs, single_size),
        C_mufn.resize(1, single_size), C_c.resize(1, single_size);
    C_lambda.setZero(), C_mufn.setZero(), C_c.setZero();

    for (int i = 0; i < mNumFrictionDirs; i++)
    {
        C_lambda(i, single_size - 1) = 1;
    }

    C_mufn(0, 0) = mMu;
    for (int i = 0; i < mNumFrictionDirs; i++)
        C_c(0, 1 + i) = 1;

    C_mufn *= this->cur_dt;
    C_lambda *= this->cur_dt;
    C_c *= this->cur_dt;
}

/**
 * \brief			Get all collision object in this world and
 * create the collision group Collision group is a set of collision object. All
 * links of a single multibody belong to the same collision group A rigidbody
 * belong to a single collision group
 */
void btGenContactSolver::RebuildColObjData()
{
    int n_objs = mWorld->getNumCollisionObjects();
    mColGroupData.clear();
    map_colobjid_to_groupid.resize(n_objs);
    mMultibodyArray.clear();
    mNumJointLimitConstraints = 0;
    mNumContactPoints = 0;
    mNumConstraints = 0;

    // 1. build the collision group data
    int n_group = 0;
    std::map<cRobotModelDynamics *, int> model_set;
    std::vector<int> ignored_ids(0);
    for (int i = 0; i < n_objs; i++)
    {
        btGenCollisionObject *obj =
            UpcastColObj(mWorld->getCollisionObjectArray()[i]);
        if (obj == nullptr)
        {
            ignored_ids.push_back(i);
            //  std::cout << "[lcp] body " << i
            //                       << " cannot be recognized, ignore\n";
            continue;
        }
        bool add_entry = false;
        switch (obj->GetType())
        {
        case eColObjType::Rigidbody:
            add_entry = true;
            break;
        case eColObjType::RobotCollder:
        {
            cRobotModelDynamics *model = UpcastRobotCollider(obj)->mModel;
            std::map<cRobotModelDynamics *, int>::iterator it =
                model_set.find(model);
            if (it == model_set.end())
            {
                // cannot find the group, have a new group
                add_entry = true;
                model_set[model] = n_group;
            }
            else
            {
                map_colobjid_to_groupid[i] = it->second;
                continue;
            }
            break;
        }
        default:
            // std::cout << "[lcp] body " << i
            //           << " cannot be recognized, ignore\n";
            continue;
            break;
        }

        if (add_entry)
        {
            map_colobjid_to_groupid[i] = n_group;
            mColGroupData.push_back(
                new btGenCollisionObjData(obj, mEnableGradientOverCtrlForce));
            n_group++;
        }
    }

    if (ignored_ids.size())
    {
        printf("[lcp] %d bodies cannot be recognized, ignored\n",
               ignored_ids.size());
    }
    // 2. collect all multibodies
    for (auto &iter : model_set)
    {
        mMultibodyArray.push_back(iter.first);
    }
    // std::cout << "[debug] RebuildColObjData multibody size = " <<
    // mMultibodyArray.size() << std::endl;

    // 3. ODE dantzig should not be used with speical root joints
    if (eLCPSolverType::BulletDantzig == mLCPSolver->GetType() &&
        this->mMultibodyArray[0]->GetRoot()->GetJointType() !=
            JointType::NONE_JOINT)
    {
        printf("[error] Dantzig solver cannot be used in None Joint case, "
               "otherwise an NAN result would be solved\n");
        BTGEN_ASSERT(false);
    }
}

void btGenContactSolver::DeleteColObjData()
{
    for (auto &i : mColGroupData)
        delete i;
    mColGroupData.clear();
}

void btGenContactSolver::DeleteConstraintData()
{
    for (auto &i : mContactConstraintData)
        delete i;
    for (auto &i : mJointLimitConstraintData)
        delete i;
    mContactConstraintData.clear();
    mJointLimitConstraintData.clear();
    mNumContactPoints = 0;
    mNumJointLimitConstraints = 0;
    mNumConstraints = 0;
}

void btGenContactSolver::CalcAbsvelConvertMat()
{
    // for (auto& i : mColGroupData)
    int num_total_constraint =
        mContactConstraintData.size() + mJointLimitConstraintData.size();
    for (int i = 0; i < mColGroupData.size(); i++)
    {
        // std::cout << "[debug] set up data for obj " << i << std::endl;
        auto &data = mColGroupData[i];
        data->Setup(mContactConstraintData.size(),
                    mJointLimitConstraintData.size());
        // std::cout << "[abs convert] outside convert mat for group " << i <<
        // ": \n"
        // 		  << data->mConvertCartesianForceToVelocityMat <<
        // std::endl; std::cout << "[abs convert] outside convert vec for group
        // " << i << ": \n"
        // 		  <<
        // data->mConvertCartesianForceToVelocityVec.transpose()
        // << std::endl;
    }
}
void btGenContactSolver::CalcRelvelConvertMat()
{
    // int n_contact = mContactConstraintData.size();
    int final_shape = mNumContactPoints * 3 + mNumJointLimitConstraints;
    rel_vel_convert_mat.resize(final_shape, final_shape);
    rel_vel_convert_vec.resize(final_shape);

    // prepare the rel vel convert mat of residual
    int multibody_dof = 0;
    if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
    {
        multibody_dof = mMultibodyArray[0]->GetNumOfFreedom();
        rel_vel_convert_mat_of_residual.resize(final_shape, multibody_dof);
    }

    tEigenArr<tMatrixXd> buffer0, buffer1;
    for (int i = 0; i < mNumContactPoints; i++)
    {
        auto &data = mContactConstraintData[i];
        if (data->mIsSelfCollision == false)
        {
            // get body0 and body1's collision group
            int body0_groupid = map_colobjid_to_groupid[data->mBodyId0],
                body1_groupid = map_colobjid_to_groupid[data->mBodyId1];

            if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
            {
                mColGroupData[body0_groupid]
                    ->GetCharacterConvertMatOfResidualRecord(buffer0);
                mColGroupData[body1_groupid]
                    ->GetCharacterConvertMatOfResidualRecord(buffer1);
            }

            // u = v_0 - v_1 = A_0f_0 + a_0 - (A_1f_0 + a_1)
            //  = (A_0 - A_1)f_0 + (a_0 - a_1)
            // here is "mat0 + mat1", because the pair of force applied is oppo
            // to each other
            rel_vel_convert_mat.block(i * 3, 0, 3, final_shape) =
                mColGroupData[body0_groupid]
                    ->mConvertCartesianForceToVelocityMat.block(i * 3, 0, 3,
                                                                final_shape);
            // here vec is normally "minus", "vec0 - vec1"
            rel_vel_convert_vec.segment(3 * i, 3) =
                mColGroupData[body0_groupid]
                    ->mConvertCartesianForceToVelocityVec.segment(i * 3, 3);

            if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
            {
                rel_vel_convert_mat_of_residual.block(
                    i * 3, 0, 3, multibody_dof) = buffer0[i];
            }

            if (mColGroupData[body1_groupid]->mBody->IsStatic() == false)
            {
                tMatrixXd res =
                    rel_vel_convert_mat.block(i * 3, 0, 3, final_shape) -
                    mColGroupData[body1_groupid]
                        ->mConvertCartesianForceToVelocityMat.block(
                            i * 3, 0, 3, final_shape);
                rel_vel_convert_mat.block(i * 3, 0, 3, final_shape) = res;

                tVectorXd res_vec =
                    rel_vel_convert_vec.segment(3 * i, 3) -
                    mColGroupData[body1_groupid]
                        ->mConvertCartesianForceToVelocityVec.segment(i * 3, 3);
                rel_vel_convert_vec.segment(3 * i, 3) = res_vec;

                if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
                {
                    rel_vel_convert_mat_of_residual.block(
                        i * 3, 0, 3, multibody_dof) = -buffer1[i];
                }
            }
        }
        else
        {
            // get body0 and body1's collision group
            int single_groupid = map_colobjid_to_groupid[data->mBodyId0];

            // for self collision, the rel vel convert mat and vec has been
            // calculated well.
            rel_vel_convert_mat.block(i * 3, 0, 3, final_shape) =
                mColGroupData[single_groupid]
                    ->mConvertCartesianForceToVelocityMat.block(i * 3, 0, 3,
                                                                final_shape);
            rel_vel_convert_vec.segment(3 * i, 3) =
                mColGroupData[single_groupid]
                    ->mConvertCartesianForceToVelocityVec.segment(i * 3, 3);

            if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
            {
                rel_vel_convert_mat_of_residual.block(
                    i * 3, 0, 3, multibody_dof) = buffer0[i];
            }
        }
    }

    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        int row_st = mNumContactPoints * 3 + i;
        int group_id =
            map_colobjid_to_groupid[data->multibody->GetLinkCollider(0)
                                        ->getWorldArrayIndex()];

        rel_vel_convert_mat.block(row_st, 0, 1, final_shape) =
            mColGroupData[group_id]->mConvertCartesianForceToVelocityMat.block(
                row_st, 0, 1, final_shape);
        rel_vel_convert_vec.segment(row_st, 1) =
            mColGroupData[group_id]
                ->mConvertCartesianForceToVelocityVec.segment(row_st, 1);

        if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
        {
            mColGroupData[group_id]->GetCharacterConvertMatOfResidualRecord(
                buffer0);
            // std::cout << "buffer size = " << buffer0.size() << std::endl;
            rel_vel_convert_mat_of_residual.block(row_st, 0, 1, multibody_dof) =
                buffer0[mNumContactPoints + i];
        }
    }
    // std::cout << "rel vel convert mat = \n"
    // 		  << rel_vel_convert_mat << std::endl;
    // std::cout << "rel vel convert vec = " << rel_vel_convert_vec.transpose()
    // << std::endl;
}
void btGenContactSolver::CalcDecomposedConvertMat()
{
    // if (mNumFrictionDirs != 4)
    // {
    // 	std::cout << "unsupported friction dirs = " << mNumFrictionDirs <<
    // std::endl;
    // 	// exit(0);
    // }
    // int n_contact = mContactConstraintData.size();
    int x_size =
        3 * mNumContactPoints +
        mNumJointLimitConstraints; // x is the collection of contact force
    if (x_size == 0)
        return;

    int multibody_dof = 0;
    if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
    {
        multibody_dof = mMultibodyArray[0]->GetNumOfFreedom();
        normal_vel_convert_mat_of_residual.resize(
            mNumContactPoints + mNumJointLimitConstraints, multibody_dof);
    }

    // the normal vel of one contact point or one joint limit constraint occupy
    // a single line
    normal_vel_convert_mat.resize(mNumContactPoints + mNumJointLimitConstraints,
                                  x_size); // one contact point one line
    normal_vel_convert_vec.resize(
        mNumContactPoints +
        mNumJointLimitConstraints); // one contact point one real number

    if (mEnableFrictionalLCP)
    {
        tangent_vel_convert_mat.resize(
            mNumFrictionDirs * mNumContactPoints,
            x_size); // one contact point N lines, N = mNumFrictionDir
        tangent_vel_convert_vec.resize(
            mNumFrictionDirs *
            mNumContactPoints); // one contact point one real number

        if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
        {
            tangent_vel_convert_mat_of_residual.resize(
                mNumFrictionDirs * mNumContactPoints, multibody_dof);
        }
    }

    for (int i = 0; i < mNumContactPoints; i++)
    {
        tVector3d normal =
            mContactConstraintData[i]->mNormalPointToA.segment(0, 3);

        // normal_rel_vel_mat = n^T * rel_vel_mat
        normal_vel_convert_mat.block(i, 0, 1, x_size) =
            normal.transpose() * rel_vel_convert_mat.block(i * 3, 0, 3, x_size);
        normal_vel_convert_vec.segment(i, 1) =
            normal.transpose() * rel_vel_convert_vec.segment(i * 3, 3);

        if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
        {
            normal_vel_convert_mat_of_residual.block(i, 0, 1, multibody_dof) =
                normal.transpose() * rel_vel_convert_mat_of_residual.block(
                                         i * 3, 0, 3, multibody_dof);
        }

        if (mEnableFrictionalLCP)
        {
            // tangent_rel_vel_mat = D^T * (rel_vel_mat - normal_rel_vel_mat)
            tangent_vel_convert_mat.block(mNumFrictionDirs * i, 0,
                                          mNumFrictionDirs, x_size) =
                mContactConstraintData[i]->mD.transpose() *
                (rel_vel_convert_mat.block(3 * i, 0, 3, x_size) -
                 normal * normal_vel_convert_mat.block(i, 0, 1, x_size));

            tangent_vel_convert_vec.segment(mNumFrictionDirs * i,
                                            mNumFrictionDirs) =
                mContactConstraintData[i]->mD.transpose() *
                (rel_vel_convert_vec.segment(3 * i, 3) -
                 normal * normal_vel_convert_vec.segment(i, 1));

            if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
            {
                tangent_vel_convert_mat_of_residual.block(
                    mNumFrictionDirs * i, 0, mNumFrictionDirs, multibody_dof) =
                    mContactConstraintData[i]->mD.transpose() *
                    (rel_vel_convert_mat_of_residual.block(3 * i, 0, 3,
                                                           multibody_dof) -
                     normal * normal_vel_convert_mat_of_residual.block(
                                  i, 0, 1, multibody_dof));
            }
        }
    }

    // copy the normal velocity part for joint limit constraints from rel vel
    normal_vel_convert_mat.block(mNumContactPoints, 0,
                                 mNumJointLimitConstraints, x_size) =
        rel_vel_convert_mat.block(mNumContactPoints * 3, 0,
                                  mNumJointLimitConstraints, x_size);
    normal_vel_convert_vec.segment(mNumContactPoints,
                                   mNumJointLimitConstraints) =
        rel_vel_convert_vec.segment(mNumContactPoints * 3,
                                    mNumJointLimitConstraints);
    if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
    {
        normal_vel_convert_mat_of_residual.block(
            mNumContactPoints, 0, mNumJointLimitConstraints, multibody_dof) =
            rel_vel_convert_mat_of_residual.block(mNumContactPoints * 3, 0,
                                                  mNumJointLimitConstraints,
                                                  multibody_dof);
    }
}

/**
 * \brief				Add a contact "manifold" into the
 * constraint solver.
 *
 * 		1. create a new tContactPoint object
 * 		2. add contact point into the CollisionGroup
 */
void btGenContactSolver::AddManifold(btPersistentManifold *manifold)
{
    int n_contacts = manifold->getNumContacts();
    btGenContactPointData *data = nullptr;
    // determine collision object type

    // for each contact point
    for (int j = 0; j < n_contacts; j++)
    {
        data = new btGenContactPointData(mContactConstraintData.size());
        // std::cout << manifold->getContactProcessingThreshold() << std::endl;
        // std::cout << manifold->getContactBreakingThreshold() << std::endl;
        // 1. prepare contact point data
        {
            data->Init(cur_dt, manifold, j);
            data->Setup(mNumFrictionDirs);
            data->mbody0GroupId = map_colobjid_to_groupid[data->GetBody0Id()];
            data->mbody1GroupId = map_colobjid_to_groupid[data->GetBody1Id()];
            if (mEnableDebugOutput == true)
            {
                std::cout << "[debug] contact " << j
                          << " penetration = " << data->distance << std::endl;
            }
            // std::cout << "data0 vel = " << data->GetVelOnBody0().transpose()
            // << std::endl; std::cout << "data1 vel = " <<
            // data->GetVelOnBody1().transpose() << std::endl;
        }

        // add it to body info
        // std::cout << "col obj data size " << mColObjData.size() << std::endl;
        // std::cout << "boidy 0 id " << data->GetBody0Id() << std::endl;
        int col_group0_data_main_group =
            map_colobjid_to_groupid[mColGroupData[data->mbody0GroupId]
                                        ->mBody->getWorldArrayIndex()];
        int col_group1_data_main_group =
            map_colobjid_to_groupid[mColGroupData[data->mbody1GroupId]
                                        ->mBody->getWorldArrayIndex()];
        if (col_group1_data_main_group != col_group0_data_main_group)
        {
            mColGroupData[data->mbody0GroupId]->AddContactPoint(
                data, col_group0_data_main_group == data->mbody0GroupId);
            mColGroupData[data->mbody1GroupId]->AddContactPoint(
                data, col_group1_data_main_group == data->mbody0GroupId);
        }
        else
        {
            if (mEnableMultibodySelfCol == true &&
                mMultibodyArray[0]->GetEnableContactAwareController() == false)
            {
                if (mEnableDebugOutput)
                {
                    std::cout << "[log] add self collision contact point into "
                                 "ColGroup"
                              << col_group1_data_main_group << ", "
                              << data->mBodyA->GetName() << " and "
                              << data->mBodyB->GetName() << std::endl;
                }

                // only add once for self-collisoin contact point
                mColGroupData[data->mbody0GroupId]->AddContactPoint(data, true);
            }
            else if (mMultibodyArray[0]->GetEnableContactAwareController() ==
                     true)
            {
                std::cout << "[warn] self collision is ignored in contact "
                             "aware control\n";
                delete data;
                continue;
            }
            else
            {
                delete data;
                continue;
            }
        }

        mContactConstraintData.push_back(data);
        mNumContactPoints++;
    }
}

/**
 * \brief					Add JointLimit
 */
void btGenContactSolver::AddJointLimit()
{
    // std::cout << "[debug] begin to add joint limit for multibody num " <<
    // mMultibodyArray.size() << std::endl;
    for (int mb_id = 0; mb_id < mMultibodyArray.size(); mb_id++)
    {
        // 1. get the joint limit and validate them
        auto &model = mMultibodyArray[mb_id];
        const tVectorXd &q = model->Getq();
        tVectorXd lb, ub;
        model->GetJointLimit(lb, ub);

        // std::cout << "[debug] lb = " << lb.transpose() << std::endl;
        // std::cout << "[debug] ub = " << ub.transpose() << std::endl;
        if ((ub - lb).minCoeff() < -1e-10)
        {
            std::cout << "[error] add joint limit invalid\nub = "
                      << ub.transpose() << "\nlb = " << lb.transpose()
                      << std::endl;
            exit(0);
        }

        // only care the revolute and spherical joint, no root joint, no
        // prismatic joint

        // 2. judge for all dofs besides root joint
        int root_dof_end = model->GetJointById(0)->GetNumOfFreedom();
        for (int dof_id = root_dof_end; dof_id < model->GetNumOfFreedom();
             dof_id++)
        {
            bool is_violated = false;
            bool is_upper_bound_violated = false;
            double boundary = 0;
            double violate_value = 0;
            // 2.1 if the joint limit is violated
            if (q[dof_id] >= ub[dof_id])
            {
                is_violated = true;
                is_upper_bound_violated = true;
                boundary = ub[dof_id];
                violate_value = (ub[dof_id] - q[dof_id]) / cur_dt;
            }
            else if (q[dof_id] <= lb[dof_id])
            {
                is_violated = true;
                boundary = lb[dof_id];
                violate_value = (q[dof_id] - lb[dof_id]) / cur_dt; // < 0
            }

            if (is_violated == true)
            {
                mJointLimitConstraintData.push_back(new btGenJointLimitData(
                    mNumContactPoints + mJointLimitConstraintData.size(), model,
                    dof_id, is_upper_bound_violated, violate_value));
                mJointLimitConstraintData.back()->dt = cur_dt;

                // 3. add joint limit constraint into the collision group
                int group_id =
                    map_colobjid_to_groupid[model->GetLinkCollider(0)
                                                ->getWorldArrayIndex()];
                mColGroupData[group_id]->AddJointLimitConstraint(
                    mJointLimitConstraintData.back());
                mNumJointLimitConstraints++;
                if (mEnableDebugOutput)
                {
                    std::cout << "[joint limit] add limit for dof " << dof_id
                              << " joint "
                              << model->GetJointByDofId(dof_id)->GetName()
                              << " val = " << q[dof_id] << " "
                              << ((is_upper_bound_violated == true) ? ("upper")
                                                                    : ("lower"))
                              << " bound " << boundary
                              << " violate = " << violate_value * cur_dt
                              << std::endl;
                }

                // exit(0);
            }
        }
    }
}

/**
 * \brief				calculate another type of convert matrix
 *
 * 			Result vector is literally the solution vector, or the
 * solution. It consisits of:
 * 				1. the "mNumFrictions+2" decomposed contact
 * force,
 * 				2. and the constraint force of joint limit
 * 			This function tries to calculate the convert matrix from
 * the result vector to the decomposed relative velocity next frame.
 */
void btGenContactSolver::CalcResultVectorBasedConvertMat()
{
    int single_size = 1;
    if (mEnableFrictionalLCP)
        single_size += mNumFrictionDirs + 1;
    int result_vector_size =
        mNumContactPoints * single_size + mNumJointLimitConstraints;
    normal_vel_convert_result_based_mat.resize(
        mNumContactPoints + mNumJointLimitConstraints, result_vector_size);
    normal_vel_convert_result_based_mat.setZero();
    normal_vel_convert_result_based_vec.resize(mNumContactPoints +
                                               mNumJointLimitConstraints);

    int multibody_dof = 0;
    if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
    {
        multibody_dof = mMultibodyArray[0]->GetNumOfFreedom();
        normal_vel_convert_mat_result_based_of_residual.resize(
            mNumContactPoints + mNumJointLimitConstraints, multibody_dof);
    }

    if (mEnableFrictionalLCP)
    {
        tangent_vel_convert_result_based_mat.resize(
            mNumContactPoints * mNumFrictionDirs, result_vector_size);
        tangent_vel_convert_result_based_mat.setZero();
        tangent_vel_convert_result_based_vec.resize(mNumContactPoints *
                                                    mNumFrictionDirs);
        tangent_vel_convert_result_based_vec.setZero();
        if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
        {
            tangent_vel_convert_mat_result_based_of_residual.resize(
                mNumContactPoints * mNumFrictionDirs +
                    mNumJointLimitConstraints,
                multibody_dof);
        }
    }

    for (int i = 0; i < mNumContactPoints; i++)
    {
        for (int j = 0; j < mNumContactPoints; j++)
        {
            auto &data = mContactConstraintData[j];

            if (mEnableFrictionalLCP)
            {
                // frictional case
                normal_vel_convert_result_based_mat.block(i, j * single_size, 1,
                                                          single_size) =
                    normal_vel_convert_mat.block(i, j * 3, 1, 3) * data->mS;

                tangent_vel_convert_result_based_mat.block(
                    i * mNumFrictionDirs, j * single_size, mNumFrictionDirs,
                    single_size) =
                    tangent_vel_convert_mat.block(i * mNumFrictionDirs, j * 3,
                                                  mNumFrictionDirs, 3) *
                    data->mS;
            }
            else
            {
                // only normal case
                normal_vel_convert_result_based_mat.block(i, j * single_size, 1,
                                                          single_size) =
                    normal_vel_convert_mat.block(i, j * 3, 1, 3) *
                    data->mNormalPointToA.segment(0, 3);
            }
        }

        // set up the joint limit constraint
        for (int j = 0; j < mNumJointLimitConstraints; j++)
        {
            normal_vel_convert_result_based_mat.block(
                i, mNumContactPoints * single_size + j, 1, 1) =
                normal_vel_convert_mat.block(i, mNumContactPoints * 3 + j, 1,
                                             1);
            if (mEnableFrictionalLCP)
            {
                tangent_vel_convert_result_based_mat.block(
                    i * mNumFrictionDirs, mNumContactPoints * single_size + j,
                    mNumFrictionDirs, 1) =
                    tangent_vel_convert_mat.block(i * mNumFrictionDirs,
                                                  mNumContactPoints * 3 + j,
                                                  mNumFrictionDirs, 1);
            }
        }
    }

    // set up the convert matrix for generalized velocity
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        int row_st = mNumContactPoints;
        // write this part
        for (int j = 0; j < mNumContactPoints; j++)
        {
            auto &data = mContactConstraintData[j];

            if (mEnableFrictionalLCP == true)
            {
                normal_vel_convert_result_based_mat.block(
                    row_st + i, j * single_size, 1, single_size) =
                    normal_vel_convert_mat.block(row_st + i, j * 3, 1, 3) *
                    data->mS;
            }
            else
            {
                // only normal case
                normal_vel_convert_result_based_mat.block(
                    row_st + i, j * single_size, 1, single_size) =
                    normal_vel_convert_mat.block(row_st + i, j * 3, 1, 3) *
                    data->mNormalPointToA.segment(0, 3);
            }
        }

        for (int j = 0; j < mNumJointLimitConstraints; j++)
        {
            auto &data = mContactConstraintData[j];
            normal_vel_convert_result_based_mat.block(
                row_st + i, mNumContactPoints * single_size + j, 1, 1) =
                normal_vel_convert_mat.block(row_st + i,
                                             mNumContactPoints * 3 + j, 1, 1);
        }
    }
    normal_vel_convert_result_based_vec = normal_vel_convert_vec;
    if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
    {
        normal_vel_convert_mat_result_based_of_residual =
            normal_vel_convert_mat_of_residual;
    }
    if (mEnableFrictionalLCP)
    {
        tangent_vel_convert_result_based_vec = tangent_vel_convert_vec;
        if (mEnableGradientOverCtrlForce && mMultibodyArray.size())
        {
            tangent_vel_convert_mat_result_based_of_residual =
                tangent_vel_convert_mat_of_residual;
        }
    }
}

/**
 * \brief			Push & pop the state into stacks
 */
void btGenContactSolver::PushState(const std::string &tag)
{
    for (auto &i : mColGroupData)
    {
        switch (i->mBody->GetType())
        {
        case eColObjType::Rigidbody:
            i->mBody->PushState(tag);
            break;
        default:
            break;
        }
    }
    for (auto &x : mMultibodyArray)
        x->PushState(tag);
}
void btGenContactSolver::PopState(const std::string &name)
{
    for (auto &i : mColGroupData)
    {
        switch (i->mBody->GetType())
        {
        case eColObjType::Rigidbody:
            i->mBody->PopState(name);
            break;
        default:
            break;
        }
    }
    for (auto &x : mMultibodyArray)
        x->PopState(name);
}

void btGenContactSolver::UpdateVelocity(float dt)
{
    std::set<cRobotModel *> models_set;
    // update at the same time
    for (auto &i : mColGroupData)
    {
        switch (i->mBody->GetType())
        {
        case eColObjType::Rigidbody:
            i->mBody->UpdateVelocity(dt);
            break;
        case eColObjType::RobotCollder:
        {
            cRobotModelDynamics *model = UpcastRobotCollider(i->mBody)->mModel;
            if (models_set.find(model) == models_set.end())
            {
                model->UpdateVelocity(dt);
                models_set.insert(model);
            }
        }
        default:
            break;
        }
    }
}

/**
 * \brief				given a target vector and an array of
 * vector directions, find the most nearest one
 */
tVector FindMostNearDirection(const tMatrixXd &mat, const tVector &target)
{
    int num_of_dir = mat.cols();
    if (mat.rows() != 3)
    {
        std::cout << "mat rows != 3\n";
        exit(0);
    }

    double max_product_dot = -std::numeric_limits<double>::max();
    tVector near_dir = tVector::Zero();
    for (int i = 0; i < num_of_dir; i++)
    {
        tVectorXd dir = mat.col(i);
        dir.normalize();
        double dot = dir.dot(target.segment(0, 3));
        // std::cout << i << " dir " << dir.transpose() << ", target = " <<
        // target.transpose() << ", dot = " << dot << std::endl;
        if (dot > max_product_dot)
        {
            max_product_dot = dot;
            near_dir.segment(0, 3) = dir;
        }
    }
    // std::cout << "the nearest dir of " << target.transpose() << " is " <<
    // near_dir.transpose() << std::endl;
    return near_dir;
}
void btGenContactSolver::VerifySolution()
{
    std::cout << "[warn] verify solutions iscalled! it may slow down the "
                 "simulation\n";
    if (mEnableFrictionalLCP == false)
        return;
    if (mNumContactPoints == 0)
        return;
    PushState("verify");
    assert(mContactConstraintData.size() == mNumContactPoints);
    // 1. apply all constraint force, then update velocity
    for (int i = 0; i < mContactConstraintData.size(); i++)
    {
        auto &data = mContactConstraintData[i];
        tVector3d contact_force = f_lcp.segment(i * 3, 3);
        data->ApplyForceCartersian(btMathUtil::Expand(contact_force, 0));
    }

    for (int i = 0; i < mJointLimitConstraintData.size(); i++)
    {
        auto &data = mJointLimitConstraintData[i];
        data->ApplyGeneralizedForce(f_lcp[mNumContactPoints * 3 + i]);
    }
    UpdateVelocity(cur_dt);

    // 2. find the nearest direction for current velocity
    for (int i = 0; i < mNumContactPoints; i++)
    {
        auto &data = mContactConstraintData[i];
        tVector rel_vel = data->GetRelVel();
        tMatrixXd directions = data->mS.block(0, 1, 3, mNumFrictionDirs);
        tVector vel_dir = FindMostNearDirection(directions, rel_vel);
        tVector force = btMathUtil::Expand(f_lcp.segment(i * 3, 3), 0);
        tVector oppo_force = -force;
        oppo_force[1] = 0;
        tVector oppo_force_dir = FindMostNearDirection(directions, oppo_force);
        // std::cout << "contact " << i << " force = " << force.transpose() <<
        // std::endl; std::cout << "contact " << i << " rel_vel = " <<
        // rel_vel.transpose() << std::endl; std::cout << "contact " << i << "
        // vel dir = " << vel_dir.transpose() << ", oppo force dir " <<
        // oppo_force_dir.transpose() << std::endl; if ((vel_dir -
        // oppo_force_dir).norm() > 1e-10 && rel_vel.norm() > 1e-4 &&
        // oppo_force.norm() > 1e-4)
        // {
        // 	std::cout << "[error] ";
        // }
        // std::cout << "contact " << i << "\nrel vel = " << rel_vel.transpose()
        // << "\nrel vel dir = "
        // 		  << vel_dir.transpose() << "\noppo force = " <<
        // oppo_force.transpose() << "\noppo forde dir = " <<
        // oppo_force_dir.transpose() << std::endl;
    }

    PopState("verify");
    // exit(0);
}

/**
 * \brief           Collect multibody from all of collision datas
*/
cRobotModelDynamics *btGenContactSolver::CollectMultibody()
{
    cRobotModelDynamics *model = nullptr;
    for (auto &data : this->mColGroupData)
    {
        btGenRobotCollider *collider =
            dynamic_cast<btGenRobotCollider *>(data->mBody);

        // if it is robot collider
        if (collider != nullptr)
        {
            // if empty, directly give the value
            if (model == nullptr)
            {
                model = collider->mModel;
            }
            else
            {
                // else, there are many model, exit
                std::cout
                    << "[error] btGenContactSolver::CollectMultibody: there "
                       "are multiple models in the collision world\n";
                exit(0);
            }
        }
    }
    return model;
}

/**
 * \brief           Calculate d(constraint_force)/d(control_force)
 * 
 *      Here, the constraint force is the simply the direct solution of LCP problem, aka "x"
 *      the control force is what we get from the character "GetGeneralizedForce"
 * 
 *      For more details, please check the note "20210105 ..."
 * 
 *  1. define   Fi = xi * (A * x + b)_i.  b = b(u), u is the control force. x is the constraint force
 *              F = [F0, F1, ..., FN-1]
 *              w = A * x + b
 *  2. define jacobian
 *              Q = dFdx = diag(w) + diag(x) * A
 *              P = dFdu = diag(x) * dbdu
 *  3. the final d(cons_lcp)/dforce = -Q.inv * P
 * 
 *  4. gen_cons_force = T * cons_lcp
 *      gen_cons_force : generalized constraint force totally
 *      cons_lsp: the solution vector of LCP problem
 *      d(gen_cons_force)/d(force) = T * d(cons_lcp)/dforce
 *      
*/
tMatrixXd btGenContactSolver::CalcDGenConsForceDCtrlForce() const
{
    BTGEN_ASSERT(mEnableGradientOverCtrlForce == true);

    return CalcDGenConsForceDx() * CalcDxDCtrlForce();
}

tMatrixXd btGenContactSolver::CalcDxDCtrlForce() const
{
    const tVectorXd &w = M * x_lcp + n;
    tMatrixXd Q = (w.asDiagonal().toDenseMatrix() + x_lcp.asDiagonal() * M);

    // for stability
    if (std::fabs(Q.determinant()) < 1e-10)
    {
        double eps = 1e-10;
        // std::cout << "Q det = " << Q.determinant() << " add smal eps " << eps
        //           << std::endl;
        Q += tMatrixXd::Identity(x_lcp.size(), x_lcp.size()) * eps;
        // std::cout << "diag(Q) = \n" << Q.diagonal().transpose() << std::endl;
    }
    const tMatrixXd &P = x_lcp.asDiagonal() * CalcDnDCtrlForce();
    tMatrixXd dx_dctrlforce = (-Q.inverse() * P);

    if (dx_dctrlforce.hasNaN() == true)
    {
        std::cout << "[error] dx_dctrlforce has Nan\n";
        std::cout << "Q = \n" << Q << std::endl;
        std::cout << "P = \n" << P << std::endl;
        exit(0);
    }

    return dx_dctrlforce;
}
/**
 * \brief           Calculate d(n)/d(control_force)
 * here the n is the residual vector in LCP
*/
tMatrixXd btGenContactSolver::CalcDnDCtrlForce() const
{
    BTGEN_ASSERT(mMultibodyArray.size() && mEnableGradientOverCtrlForce);
    auto mb = mMultibodyArray[0];
    const tMatrixXd &M = mb->GetMassMatrix();
    const tMatrixXd &inv_M = mb->GetInvMassMatrix();
    const tMatrixXd &coriolis_mat = mb->GetCoriolisMatrix();
    const tMatrixXd &damping_mat = mb->GetDampingMatrix();
    const tMatrixXd &M_dt_C_D_inv =
        (M + cur_dt * (coriolis_mat + damping_mat)).inverse();

    return n_convert_mat_final * M_dt_C_D_inv;
}

/**
 * \brief           Test the gradient of d(n)/d(ctrl_force)
*/
void btGenContactSolver::TestDnDCtrlForce()
{
    // 1. save
    if (mMultibodyArray.size() == 0 ||
        mNumJointLimitConstraints + mNumContactPoints == 0)
        return;
    auto mb = mMultibodyArray[0];
    mb->PushState("test_dndctrl");
    tVectorXd old_n = n;
    tVectorXd Q_ctrl = mb->GetGeneralizedForce();
    tMatrixXd ideal_dn_dctrlf = CalcDnDCtrlForce();
    double eps = 1e-5;

    // 2. restore
    for (int i = 0; i < mb->GetNumOfFreedom(); i++)
    {
        Q_ctrl[i] += eps;
        mb->SetGeneralizedForce(Q_ctrl);
        mEnableGradientOverCtrlForce = false;
        ConstraintFinished();
        mEnableGradientOverCtrlForce = true;
        ConstraintSetup();
        ConstraintSolve();

        tVectorXd new_n = n;
        tVectorXd num_dn_dctrlfi = (new_n - old_n) / eps;
        tVectorXd diff = num_dn_dctrlfi - ideal_dn_dctrlf.col(i);
        if (diff.cwiseAbs().maxCoeff() > 10 * eps)
        {
            std::cout << "[error] TestDnDCtrlForce for idx " << i
                      << " failed = " << diff.transpose() << std::endl;
            std::cout << "old n = " << old_n.transpose() << std::endl;
            std::cout << "new n = " << new_n.transpose() << std::endl;
            std::cout << "num dndctrlf = " << num_dn_dctrlfi.transpose()
                      << std::endl;
            std::cout << "ideal dndctrlf = "
                      << ideal_dn_dctrlf.col(i).transpose() << std::endl;
            exit(0);
        }
        Q_ctrl[i] -= eps;
    }

    mb->PopState("test_dndctrl");
    n = old_n;
    n_convert_mat_final = ideal_dn_dctrlf;
    mb->SetGeneralizedForce(Q_ctrl);
    mEnableGradientOverCtrlForce = false;
    ConstraintFinished();
    mEnableGradientOverCtrlForce = true;
    ConstraintSetup();
    ConstraintSolve();
    {
        tVectorXd contact_x;
        if (mUseSIResult)
            contact_x = x_si;
        else if (mUseLCPResult)
            contact_x = x_lcp;

        // 3. calculate the cartesian & generalized contact torque / forces
        contact_x = ConvertLCPResult(contact_x);
        CalcContactForceTorqueArrays(contact_force_array, contact_torque_array,
                                     contact_x);
    }
    std::cout << "[debug] TestDnDCtrlForce succ\n";
}

/**
 * \brief           Calculate the jacobian of (d generalized_constraint_force ) / (d x)
 *          generalized_constraint_force is the total constraint force expressed as gen force
 *          x is the lcp solution vector
*/
tMatrixXd btGenContactSolver::CalcDGenConsForceDx() const
{
    if (mMultibodyArray.size() == 0)
        return tMatrixXd::Zero(0, 0);

    auto mb = mMultibodyArray[0];
    int multibody_dof = mb->GetNumOfFreedom();
    tMatrixXd dQconsdx = tMatrixXd::Zero(multibody_dof, GetLCPSolutionSize());
    // iterate on each contact point
    tMatrixXd buffer;
    int idx = 0;
    for (int i = 0; i < mContactConstraintData.size(); i++)
    {
        auto &data = mContactConstraintData[i];
        int size = mEnableFrictionalLCP == true ? mNumFrictionDirs + 2 : 1;

        if (data->mBodyA->GetType() == eColObjType::RobotCollder)
        {
            // is multibody contact force
            int link_id =
                static_cast<btGenRobotCollider *>(data->mBodyA)->mLinkId;
            mb->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                link_id, data->mContactPtOnA.segment(0, 3), buffer);
            if (mEnableFrictionalLCP)
            {
                // printf("buffer.T size = %d %d\n", buffer.transpose().rows(),
                //        buffer.transpose().cols());
                // printf("mS size = %d %d\n", data->mS.rows(), data->mS.cols());
                dQconsdx.block(0, idx, multibody_dof, size) +=
                    buffer.transpose() * data->mS;
            }
            else
            {
                dQconsdx.block(0, idx, multibody_dof, size) +=
                    buffer.transpose() * data->mNormalPointToA;
            }
        }

        if (data->mBodyB->GetType() == eColObjType::RobotCollder)
        {
            // is multibody contact force
            int link_id =
                static_cast<btGenRobotCollider *>(data->mBodyB)->mLinkId;
            mb->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                link_id, data->mContactPtOnB.segment(0, 3), buffer);
            if (mEnableFrictionalLCP)
            {
                dQconsdx.block(0, idx, multibody_dof, size) +=
                    -buffer.transpose() * data->mS;
            }
            else
            {
                dQconsdx.block(0, idx, multibody_dof, size) +=
                    -buffer.transpose() * data->mNormalPointToA;
            }
        }
        idx += size;
    }

    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        dQconsdx(data->dof_id, idx) += data->is_upper_bound == true ? -1 : 1;
        idx += 1;
    }
    BTGEN_ASSERT(idx == GetLCPSolutionSize());
    return dQconsdx;
}

/**
 * \brief           Get the solution size of LCP problem
*/
int btGenContactSolver::GetLCPSolutionSize() const
{
    int total_size = 0;
    if (mEnableFrictionalLCP == true)
        total_size += (mNumFrictionDirs + 2) *
                      mNumContactPoints; // normal 1, tangent num_dirs, lambda 1
    else
        total_size += mNumContactPoints;
    total_size += mNumJointLimitConstraints;
    return total_size;
}

/**
 * \brief           Calculate contact force & torque arrays, based on x_lcp vector
*/
void btGenContactSolver::CalcContactForceTorqueArrays(
    std::vector<btGenContactForce *> &contact_force_array,
    std::vector<btGenConstraintGeneralizedForce *> &contact_torque_array,
    const tVectorXd &result_vector) const
{
    ClearConstraintForceTorqueArrays(contact_force_array, contact_torque_array);
    for (int i = 0; i < mContactConstraintData.size(); i++)
    {
        // std::cout << "[bt] contact point " << i << std::endl;
        btGenContactForce *ptr;
        auto &data = mContactConstraintData[i];
        // std::cout << "[debug] for contact " << i << " data = " << data <<
        // std::endl; std::cout << "contact x size = " << contact_x.size() <<
        // std::endl; std::cout << "[debug] contact force = " <<
        // cMathUtil::Expand(contact_x.segment(i * 3, 3), 0).transpose() <<
        // std::endl; std::cout << "[debug] contact pos = " <<
        // data->mContactPtOnA.transpose() << std::endl;
        ptr = new btGenContactForce(
            data->mBodyA, data->mBodyB,
            btMathUtil::Expand(result_vector.segment(i * 3, 3), 0),
            data->mContactPtOnA, data->mIsSelfCollision);
        // std::cout << "[debug] contact point " << i << " is self collision = "
        // << data->mIsSelfCollision << std::endl; std::cout << "[bt] user ptr =
        // " << ptr->mObj->getUserPointer() << std::endl;
        contact_force_array.push_back(ptr);
        ptr = new btGenContactForce(
            data->mBodyB, data->mBodyA,
            -btMathUtil::Expand(result_vector.segment(i * 3, 3), 0),
            data->mContactPtOnB, data->mIsSelfCollision);
        // std::cout << "[bt] user ptr = " << ptr->mObj->getUserPointer() <<
        // std::endl;
        contact_force_array.push_back(ptr);
    }

    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        btGenConstraintGeneralizedForce *ptr =
            new btGenConstraintGeneralizedForce(
                data->multibody, data->dof_id,
                result_vector[mNumContactPoints * 3 + i]);
        contact_torque_array.push_back(ptr);
        if (mEnableDebugOutput)
        {
            std::cout << "[lcp] joint limit " << ptr->dof_id
                      << " constraint generalized force " << ptr->value
                      << std::endl;
        }
    }
}

/**
 * \brief           Test the jacobian d(gen_cons_force)/d(lcp_x_vector)
*/
void btGenContactSolver::TestDGenConsForceDx()
{
    if (mMultibodyArray.size() == 0)
        return;

    tVectorXd result_vec = x_lcp;
    tVectorXd old_gen_force = CalcConsGenForce(result_vec);
    tMatrixXd dGenConsF_dx = CalcDGenConsForceDx();
    int total_size = GetLCPSolutionSize();
    double eps = 1e-6;
    // std::cout << "dGenConsF_dx = \n" << dGenConsF_dx << std::endl;
    // std::cout << "x_lcp = " << x_lcp.transpose() << std::endl;
    // std::cout << "num of constraints = " << mNumConstraints << std::endl;
    // std::cout << "num of joint limit constraints = "
    //           << mNumJointLimitConstraints << std::endl;
    // std::cout << "num of contacts constraints = " << mNumContactPoints
    //           << std::endl;
    for (int i = 0; i < total_size; i++)
    {
        result_vec[i] += eps;
        tVectorXd new_gen_force = CalcConsGenForce(result_vec);

        tVectorXd num_deriv = (new_gen_force - old_gen_force) / eps;
        tVectorXd diff = num_deriv - dGenConsF_dx.col(i);
        if (diff.cwiseAbs().maxCoeff() > 10 * eps)
        {
            printf("[error] TestDGenConsForceDx %d idx failed\n", i);
            std::cout << "num_deriv = " << num_deriv.transpose() << std::endl;
            std::cout << "ideal_deriv = " << dGenConsF_dx.col(i).transpose()
                      << std::endl;
            std::cout << "old Q = " << old_gen_force.transpose() << std::endl;
            std::cout << "new Q = " << new_gen_force.transpose() << std::endl;
            std::cout << "diff = " << diff.transpose() << std::endl;
            exit(0);
        }
        result_vec[i] -= eps;
    }
    CalcConsGenForce(result_vec);
    std::cout << "[debug] TestDGenConsForceDx succ\n";
}

/**
 * \brief           given the solution vector of LCP (or SI), calculate the generalized constraint force on the multibody
*/
tVectorXd
btGenContactSolver::CalcConsGenForce(const tVectorXd &result_vec) const
{
    auto mb = mMultibodyArray[0];
    tVectorXd new_vec = ConvertLCPResult(result_vec);
    std::vector<btGenContactForce *> contact_force_array(0);
    std::vector<btGenConstraintGeneralizedForce *> contact_torque_array(0);
    CalcContactForceTorqueArrays(contact_force_array, contact_torque_array,
                                 new_vec);

    tMatrixXd jac;
    tVectorXd total_gen_force = tVectorXd::Zero(mb->GetNumOfFreedom());
    for (auto &f : contact_force_array)
    {
        if (f->mObj->GetType() == eColObjType::RobotCollder)
        {
            // mb-> f->mWorldPos
            mb->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                static_cast<btGenRobotCollider *>(f->mObj)->mLinkId,
                f->mWorldPos.segment(0, 3), jac);
            total_gen_force += jac.transpose() * f->mForce.segment(0, 3);
        }
    }

    for (auto &x : contact_torque_array)
    {
        total_gen_force[x->dof_id] += x->value;
    }

    ClearConstraintForceTorqueArrays(contact_force_array, contact_torque_array);
    return total_gen_force;
}

/**
 * \brief               Test the jacobian d(generalized_constraint_force)/d(generalized_control_force)
*/
void btGenContactSolver::TestDGenConsForceDCtrlForce()
{
    // 1. save
    if (mMultibodyArray.size() == 0 ||
        mNumJointLimitConstraints + mNumContactPoints == 0)
        return;
    auto mb = mMultibodyArray[0];
    mb->PushState("test_dndctrl");
    tMatrixXd old_n_convert_mat_final = n_convert_mat_final;
    tMatrixXd old_DGenConsForceDGenCtrlForce = DGenConsForceDGenCtrlForce;
    tVectorXd old_cons = CalcConsGenForce(x_lcp);
    tVectorXd Q_ctrl = mb->GetGeneralizedForce();
    tMatrixXd ideal_dgencons_dctrlf = CalcDGenConsForceDCtrlForce();
    double eps = 1e-5;

    // 2. restore
    bool err = false;
    for (int i = 0; i < mb->GetNumOfFreedom(); i++)
    {
        Q_ctrl[i] += eps;
        mb->SetGeneralizedForce(Q_ctrl);
        mEnableGradientOverCtrlForce = false;
        ConstraintFinished();
        mEnableGradientOverCtrlForce = true;
        ConstraintSetup();
        ConstraintSolve();

        tVectorXd new_cons = CalcConsGenForce(x_lcp);
        tVectorXd num_dn_dctrlfi = (new_cons - old_cons) / eps;
        tVectorXd ideal_dn_dctrlfi = ideal_dgencons_dctrlf.col(i);
        tVectorXd diff = num_dn_dctrlfi - ideal_dgencons_dctrlf.col(i);

        // calculate the element wise violation (percentage)
        tVectorXd base_value =
            ideal_dn_dctrlfi.cwiseAbs().cwiseMax(num_dn_dctrlfi.cwiseAbs());
        double violate_perc =
            diff.cwiseAbs().cwiseProduct(base_value.cwiseInverse()).maxCoeff();

        if (diff.cwiseAbs().maxCoeff() > 100 * eps && violate_perc > 0.01)
        {
            err = true;
            std::cout << "[error] TestDGenConsForceDCtrlForce for idx " << i
                      << " failed = " << diff.transpose() << std::endl;
            std::cout << "old gen cons force = " << old_cons.transpose()
                      << std::endl;
            std::cout << "new gen cons force = " << new_cons.transpose()
                      << std::endl;
            std::cout << "num dgencons_dctrlf = " << num_dn_dctrlfi.transpose()
                      << std::endl;
            std::cout << "ideal dgencons_dctrlf = "
                      << ideal_dgencons_dctrlf.col(i).transpose() << std::endl;
            std::cout << "q = " << mb->Getq().transpose() << std::endl;
            std::cout << "qdot = " << mb->Getqdot().transpose() << std::endl;
            // exit(0);
        }
        else
        {
            // std::cout << "old gen cons force = " << old_cons.transpose()
            //           << std::endl;
            // std::cout << "new gen cons force = " << new_cons.transpose()
            //           << std::endl;
            // std::cout << "num dgencons_dctrlf = " << num_dn_dctrlfi.transpose()
            //           << std::endl;
            // std::cout << "ideal dgencons_dctrlf = "
            //           << ideal_dgencons_dctrlf.col(i).transpose() << std::endl;
            // std::cout << "q = " << mb->Getq().transpose() << std::endl;
            // std::cout << "qdot = " << mb->Getqdot().transpose() << std::endl;
        }
        Q_ctrl[i] -= eps;
    }

    mb->PopState("test_dndctrl");
    mb->SetGeneralizedForce(Q_ctrl);
    n_convert_mat_final = old_n_convert_mat_final;
    DGenConsForceDGenCtrlForce = old_DGenConsForceDGenCtrlForce;
    mEnableGradientOverCtrlForce = false;
    ConstraintFinished();
    mEnableGradientOverCtrlForce = true;
    ConstraintSetup();
    ConstraintSolve();
    {
        tVectorXd contact_x;
        if (mUseSIResult)
            contact_x = x_si;
        else if (mUseLCPResult)
            contact_x = x_lcp;

        // 3. calculate the cartesian & generalized contact torque / forces
        contact_x = ConvertLCPResult(contact_x);
        CalcContactForceTorqueArrays(contact_force_array, contact_torque_array,
                                     contact_x);
    }
    if (err == false)
        std::cout << "[debug] TestDGenConsForceDCtrlForce succ\n";
    else
        std::cout << "[error] TestDGenConsForceDCtrlForce failed\n";
}

/**
 * \brief           
*/
// void btGenContactSolver::TestDxDn() {}

void btGenContactSolver::TestDxDCtrlForce()
{
    // 1. save
    if (mMultibodyArray.size() == 0 ||
        mNumJointLimitConstraints + mNumContactPoints == 0)
        return;
    auto mb = mMultibodyArray[0];
    mb->PushState("TestDxDCtrlForce");
    tVectorXd old_x = x_lcp;
    tVectorXd Q_ctrl = mb->GetGeneralizedForce();
    tMatrixXd ideal_dx_dctrlf = CalcDxDCtrlForce();
    double eps = 1e-4;

    // 2. restore
    for (int i = 0; i < mb->GetNumOfFreedom(); i++)
    {
        Q_ctrl[i] += eps;
        mb->SetGeneralizedForce(Q_ctrl);
        mEnableGradientOverCtrlForce = false;
        ConstraintFinished();
        mEnableGradientOverCtrlForce = true;
        ConstraintSetup();
        ConstraintSolve();

        tVectorXd new_x = x_lcp;
        tVectorXd num_dx_dctrlfi = (new_x - old_x) / eps;
        tVectorXd diff = num_dx_dctrlfi - ideal_dx_dctrlf.col(i);
        if (diff.cwiseAbs().maxCoeff() > 1e3 * eps)
        {
            std::cout << "[error] TestDxDCtrlForce for idx " << i
                      << " failed = " << diff.transpose() << std::endl;
            std::cout << "old n = " << old_x.transpose() << std::endl;
            std::cout << "new n = " << new_x.transpose() << std::endl;
            std::cout << "num dxdctrlf = " << num_dx_dctrlfi.transpose()
                      << std::endl;
            std::cout << "ideal dxdctrlf = "
                      << ideal_dx_dctrlf.col(i).transpose() << std::endl;
            // exit(0);
        }
        Q_ctrl[i] -= eps;
    }

    mb->PopState("TestDxDCtrlForce");
    x_lcp = old_x;
    mb->SetGeneralizedForce(Q_ctrl);
    mEnableGradientOverCtrlForce = false;
    ConstraintFinished();
    mEnableGradientOverCtrlForce = true;
    ConstraintSetup();
    ConstraintSolve();
    {
        tVectorXd contact_x;
        if (mUseSIResult)
            contact_x = x_si;
        else if (mUseLCPResult)
            contact_x = x_lcp;

        // 3. calculate the cartesian & generalized contact torque / forces
        contact_x = ConvertLCPResult(contact_x);
        CalcContactForceTorqueArrays(contact_force_array, contact_torque_array,
                                     contact_x);
    }
    std::cout << "[debug] TestDxDCtrlForce succ\n";
}