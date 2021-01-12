#include "ContactSolver.h"
#include "BulletGenDynamics/btGenModel/RobotCollider.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"

extern btGenRigidBody *UpcastRigidBody(const btCollisionObject *col);
extern btGenRobotCollider *UpcastRobotCollider(const btCollisionObject *col);
extern btGenCollisionObject *UpcastColObj(const btCollisionObject *col);
#define EPS (1e-6)
void btGenContactSolver::TestCartesianForceToCartesianVel()
{
    // std::cout << "----------------------------begin to
    // test---------------------------\n";
    PushState("TestCartesianForceToCartesianVel");
    int constraint_length = mNumJointLimitConstraints + mNumContactPairs * 3;
    if (0 == constraint_length)
        return;

    int n_group = mColGroupData.size();
    x_lcp.resize(constraint_length);
    x_lcp.setZero();
    // x_lcp[1] = 1.0;
    x_lcp.setRandom();
    // if (constraint_length > 3) x_lcp.segment(0, 3).setZero();

    // x *= 100;

    // get prediction
    tEigenArr<tVectorXd> contact_point_cartesian_vel(n_group);
    for (int i = 0; i < n_group; i++)
    {
        btMathUtil::RoundZero(
            mColGroupData[i]->mConvertCartesianForceToVelocityMat);
        btMathUtil::RoundZero(
            mColGroupData[i]->mConvertCartesianForceToVelocityVec);
        // std::cout << "for body " << mColGroupData[i]->mBody->GetName() << "
        // "; std::cout << "convert mat = \n"
        // 		  << mColGroupData[i]->mConvertCartesianForceToVelocityMat
        // << std::endl; std::cout << "convert vec = " <<
        // mColGroupData[i]->mConvertCartesianForceToVelocityVec.transpose() <<
        // std::endl;
        contact_point_cartesian_vel[i] =
            mColGroupData[i]->mConvertCartesianForceToVelocityMat * x_lcp +
            mColGroupData[i]->mConvertCartesianForceToVelocityVec;
        // std::cout << "for group " << i << " "
        //           << "c2v mat = \n"
        //           << mColGroupData[i]->mConvertCartesianForceToVelocityMat
        //           << std::endl;
        // std::cout
        //     << "for group " << i << " "
        //     << "c2v vec = \n"
        //     << mColGroupData[i]->mConvertCartesianForceToVelocityVec.transpose()
        //     << std::endl;
        // std::cout << "for group " << i << " force = " << x_lcp.transpose()
        //           << std::endl;
        // std::cout << "for group " << i
        //           << " vel = " << contact_point_cartesian_vel[i].transpose()
        //           << std::endl;
    }

    // apply active force
    tEigenArr<tVector> AppliedActiveForce(0);
    std::vector<int> AppliedGenForce(0);
    for (int i = 0; i < mNumContactPairs; i++)
    {
        AppliedActiveForce.push_back(
            btMathUtil::Expand(x_lcp.segment(3 * i, 3), 0));
        auto &data = mContactPairConsData[i];
        data->ApplyForceCartersian(btMathUtil::Expand(
            x_lcp.segment(3 * i, 3), 0)); // apply catesian force
    }
    // std::cout << "only contact force Q = " <<
    // mMultibodyArray[0]->GetGeneralizedForce().transpose() << std::endl;

    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        data->ApplyGeneralizedForce(x_lcp[3 * mNumContactPairs + i]);
        AppliedGenForce.push_back(x_lcp[3 * mNumContactPairs + i]);
    }
    // if the contact aware if enabled, we need to add control force
    TestAddContactAwareForceIfPossible(AppliedActiveForce, AppliedGenForce);

    // std::cout << "Q = " <<
    // mMultibodyArray[0]->GetGeneralizedForce().transpose() << std::endl;

    // update velocity
    UpdateVelocity(cur_dt);

    // get the true vel and update
    bool err = false;
    for (int i = 0; i < mNumContactPairs; i++)
    {
        auto &data = mContactPairConsData[i];
        if (data->mIsMBSelfCollision)
            continue; // ignore self collision in current logic

        tVector body0_true_vel = data->GetVelOnBody0();
        int body0_group = map_colobjid_to_groupid[data->GetBody0Id()];
        tVector body0_pred_vel = btMathUtil::Expand(
            contact_point_cartesian_vel[body0_group].segment(3 * i, 3), 0);

        tVector body1_true_vel = data->GetVelOnBody1();
        int body1_group = map_colobjid_to_groupid[data->GetBody1Id()];
        tVector body1_pred_vel = btMathUtil::Expand(
            contact_point_cartesian_vel[body1_group].segment(3 * i, 3), 0);

        double body0_diff =
                   (body0_true_vel - body0_pred_vel).cwiseAbs().maxCoeff(),
               body1_diff =
                   (body1_true_vel - body1_pred_vel).cwiseAbs().maxCoeff();
        if (body0_diff > EPS || body1_diff > EPS)
        {
            err = true;
            std::cout << "[error] convert cartesian abs vel: for contact " << i
                      << "/" << mNumContactPairs << "---------------\n";
            std::cout << "body0 is " << data->mBodyA->GetName() << " body1 is "
                      << data->mBodyB->GetName() << std::endl;
            std::cout << "contact normal = "
                      << data->mNormalPointToA.transpose() << std::endl;
            std::cout << "body 0 pred vel = " << body0_pred_vel.transpose()
                      << std::endl;
            std::cout << "body 0 true vel = " << body0_true_vel.transpose()
                      << std::endl;
            std::cout << "body 0 diff = "
                      << (body0_pred_vel - body0_true_vel).transpose()
                      << std::endl;
            std::cout << "body 1 pred vel = " << body1_pred_vel.transpose()
                      << std::endl;
            std::cout << "body 1 true vel = " << body1_true_vel.transpose()
                      << std::endl;
            std::cout << "body 1 diff = "
                      << (body1_pred_vel - body1_true_vel).transpose()
                      << std::endl;

            if (body0_diff > EPS)
            {
                if (data->mBodyA->GetType() == eColObjType::RobotCollder &&
                    true == UpcastRobotCollider(data->mBodyA)
                                ->mModel->IsGeneralizedMaxVel())
                {
                    err = false;
                    std::cout << "[veirfy abs vel] bodyA now qdot is up to "
                                 "max_vel, so it's trivial to verify "
                                 "failed(caused by clamp)\n";
                    std::cout << "[veirfy abs vel] qdot = "
                              << UpcastRobotCollider(data->mBodyA)
                                     ->mModel->Getqdot()
                                     .transpose()
                              << std::endl;
                }
            }
            if (body1_diff > EPS)
            {
                if (data->mBodyB->GetType() == eColObjType::RobotCollder &&
                    true == UpcastRobotCollider(data->mBodyB)
                                ->mModel->IsGeneralizedMaxVel())
                {
                    err = false;
                    std::cout << "[veirfy abs vel] bodyB now qdot is up to "
                                 "max_vel, so it's trivial to verify "
                                 "failed(caused by clamp)\n";
                    std::cout << "[veirfy abs vel] qdot = "
                              << UpcastRobotCollider(data->mBodyB)
                                     ->mModel->Getqdot()
                                     .transpose()
                              << std::endl;
                }
            }
        }
    }

    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        int col_id = data->multibody->GetLinkCollider(0)->getWorldArrayIndex();
        int group_id = map_colobjid_to_groupid[col_id];
        // std::cout << "group id = " << group_id << std::endl;
        // std::cout << "multibody pred qdot = " <<
        // (contact_point_cartesian_vel[group_id]).transpose() << std::endl;
        double qdot_true = data->multibody->Getqdot()[data->dof_id];
        double qdot_pred =
            contact_point_cartesian_vel[group_id][mNumContactPairs * 3 + i];
        if (data->is_upper_bound)
            qdot_pred *= -1;
        double diff = std::fabs(qdot_true - qdot_pred);
        // std::cout << "constraint " << i << " dof " << data->dof_id << " pred
        // vel = " << qdot_pred << ", true vel = " << qdot_true << std::endl;
        if (diff > EPS)
        {
            std::cout << "[error] convert cartesian abs vel: for joint limit "
                         "constraint "
                      << i << ", dof = " << data->dof_id << std::endl;
            std::cout << "pred vel " << qdot_pred << std::endl;
            std::cout << "true vel " << qdot_true << std::endl;
            std::cout << "diff = " << diff << std::endl;
            err = true;

            if (true == data->multibody->IsGeneralizedMaxVel())
            {
                err = false;
                std::cout << "[veirfy q] mb now qdot is up to max_vel, so it's "
                             "trivial to verify failed(caused by clamp)\n";
                std::cout << "[veirfy q] qdot = "
                          << data->multibody->Getqdot().transpose()
                          << std::endl;
            }
        }
    }
    if (err)
    {
        exit(0);
    }
    // std::cout << "test abs vel succ\n";
    // exit(02);
    // if (constraint_length > 3) exit(0);
    PopState("TestCartesianForceToCartesianVel");
}

void btGenContactSolver::TestCartesianForceToCartesianRelVel(
    const tMatrixXd &convert_mat, const tVectorXd &convert_vec)
{
    int constraint_length = mNumJointLimitConstraints + mNumContactPairs * 3;
    if (0 == constraint_length)
        return;
    PushState("TestCartesianForceToCartesianRelVel");

    int n_group = mColGroupData.size();
    x_lcp.resize(constraint_length);
    x_lcp.setRandom();
    // x_lcp.setZero();
    // x[1] = 1;

    // get prediction: relative velocity on each contact point
    // std::cout << "rel convert mat = \n"
    // 		  << convert_mat << std::endl;
    // std::cout << "rel convert vec = " << convert_vec.transpose() <<
    // std::endl;
    tVectorXd rel_vel_pred = convert_mat * x_lcp + convert_vec;

    // apply active force for contact forces
    int n_self_collision = 0;
    tEigenArr<tVector> AppliedActiveForce(0);
    std::vector<int> AppliedGenForce(0);
    for (int i = 0; i < mNumContactPairs; i++)
    {
        auto &data = mContactPairConsData[i];
        data->ApplyForceCartersian(
            btMathUtil::Expand(x_lcp.segment(3 * i, 3), 0));
        AppliedActiveForce.push_back(
            btMathUtil::Expand(x_lcp.segment(3 * i, 3), 0));
        if (data->mIsMBSelfCollision)
            n_self_collision++;
    }

    // for joint limits
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        AppliedGenForce.push_back(x_lcp[3 * mNumContactPairs + i]);
        data->ApplyGeneralizedForce(x_lcp[3 * mNumContactPairs + i]);
    }

    TestAddContactAwareForceIfPossible(AppliedActiveForce, AppliedGenForce);

    // get the true vel and update
    UpdateVelocity(cur_dt);

    // check the velocity for each contact point
    for (int i = 0; i < mNumContactPairs; i++)
    {
        auto &data = mContactPairConsData[i];
        tVector true_relvel = data->GetVelOnBody0() - data->GetVelOnBody1();
        tVector pred_relvel =
            btMathUtil::Expand(rel_vel_pred.segment(i * 3, 3), 0);
        double diff = (true_relvel - pred_relvel).cwiseAbs().maxCoeff();
        if (diff > EPS)
        {
            bool err = false;
            err = true;
            std::cout
                << "[error] convert cartesian force to rel vel: for contact "
                << i << "---------------\n";
            std::cout << "pred rel vel = " << pred_relvel.transpose()
                      << std::endl;
            std::cout << "true rel vel = " << true_relvel.transpose()
                      << std::endl;
            std::cout << "diff = " << diff << std::endl;

            std::cout << "total contact num = " << mNumContactPairs
                      << std::endl;
            std::cout << "self collision num = " << n_self_collision
                      << std::endl;
            if (IsMultibodyAndVelMax(data->mBodyA) ||
                IsMultibodyAndVelMax(data->mBodyB))
            {
                err = false;
            }
            if (err)
                exit(0);
        }
    }

    // check the generalized velocity for joint limit
    bool err = false;
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        int col_id = data->multibody->GetLinkCollider(0)->getWorldArrayIndex();
        int group_id = map_colobjid_to_groupid[col_id];
        double qdot_true = data->multibody->Getqdot()[data->dof_id];
        double qdot_pred = rel_vel_pred[3 * mNumContactPairs + i];
        if (data->is_upper_bound)
            qdot_pred *= -1;
        double diff = std::fabs(qdot_true - qdot_pred);
        // std::cout << "constraint " << i << " dof " << data->dof_id << " pred
        // vel = " << qdot_pred << ", true vel = " << qdot_true << std::endl;
        if (diff > EPS)
        {
            err = true;
            std::cout << "[error] convert cartesian force to rel vel: for "
                         "joint limit constraint "
                      << i << ", dof = " << data->dof_id << std::endl;
            std::cout << "pred vel " << qdot_pred << std::endl;
            std::cout << "true vel " << qdot_true << std::endl;
            std::cout << "diff = " << diff << std::endl;
            if (data->multibody->IsGeneralizedMaxVel() == true)
            {
                err = false;
            }
        }
        if (err)
            exit(0);
    }
    PopState("TestCartesianForceToCartesianRelVel");
}

void btGenContactSolver::TestCartesianForceToNormalAndTangetRelVel(
    const tMatrixXd &normal_mat, const tVectorXd &normal_vec,
    const tMatrixXd &tan_mat, const tVectorXd &tan_vec)
{
    int final_shape = mNumContactPairs * 3 + mNumJointLimitConstraints;
    if (0 == final_shape)
        return;
    PushState("TestCartesianForceToNormalAndTangetRelVel");

    int n_group = mColGroupData.size();
    x_lcp.resize(final_shape);
    // x_lcp.setRandom();
    // x_lcp.segment(0, x_lcp.size() - 1).setZero();
    x_lcp.setZero();
    // x[1] = 1;

    // get normal and tangent vel prediction on each point
    tVectorXd rel_vel_normal_pred = normal_mat * x_lcp + normal_vec;
    tVectorXd rel_vel_tangent_pred =
        tVectorXd::Zero(rel_vel_normal_pred.size());

    if (mEnableFrictionalLCP)
    {
        rel_vel_tangent_pred = tan_mat * x_lcp + tan_vec;
    }

    // apply active force
    tEigenArr<tVector> AppliedActiveForce(0);
    std::vector<int> AppliedGenForce(0);
    for (int i = 0; i < mNumContactPairs; i++)
    {
        auto &data = mContactPairConsData[i];
        data->ApplyForceCartersian(
            btMathUtil::Expand(x_lcp.segment(3 * i, 3), 0));
        AppliedActiveForce.push_back(
            btMathUtil::Expand(x_lcp.segment(3 * i, 3), 0));
    }

    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        data->ApplyGeneralizedForce(x_lcp[mNumContactPairs * 3 + i]);
        AppliedGenForce.push_back(x_lcp[mNumContactPairs * 3 + i]);
    }
    TestAddContactAwareForceIfPossible(AppliedActiveForce, AppliedGenForce);

    // get the true vel and update
    UpdateVelocity(cur_dt);

    for (int i = 0; i < mNumContactPairs; i++)
    {
        bool err = false;
        auto &data = mContactPairConsData[i];
        tVector true_rel_vel = data->GetVelOnBody0() - data->GetVelOnBody1();
        tVectorXd true_normal_relvel =
            data->mNormalPointToA.transpose() * true_rel_vel;

        tVectorXd pred_normal_relvel = rel_vel_normal_pred.segment(i, 1);

        // check frictional force
        if (mEnableFrictionalLCP)
        {
            tVectorXd true_tan_relvel =
                data->mD.transpose() *
                (true_rel_vel - data->mNormalPointToA * true_normal_relvel[0])
                    .segment(0, 3);
            tVectorXd pred_tan_relvel = rel_vel_tangent_pred.segment(
                i * mNumFrictionDirs, mNumFrictionDirs);
            double diff =
                (true_tan_relvel - pred_tan_relvel).cwiseAbs().maxCoeff();
            if (diff > EPS)
            {
                err = true;
                std::cout
                    << "[error] cartesian force to tangent rel vel for contact "
                    << i << "---------------\n";
                std::cout << "pred tan rel vel = "
                          << pred_tan_relvel.transpose() << std::endl;
                std::cout << "true tan rel vel = "
                          << true_tan_relvel.transpose() << std::endl;
                std::cout << "diff = " << diff << std::endl;
            }
        }

        double diff =
            (pred_normal_relvel - pred_normal_relvel).cwiseAbs().maxCoeff();
        if (diff > EPS)
        {
            err = true;
            std::cout
                << "[error] cartesian force to normal rel vel for contact " << i
                << "---------------\n";
            std::cout << "pred normal rel vel = "
                      << pred_normal_relvel.transpose() << std::endl;
            std::cout << "true normal rel vel = "
                      << true_normal_relvel.transpose() << std::endl;
            std::cout << "diff = " << diff << std::endl;
        }

        if (err == true)
        {
            if (IsMultibodyAndVelMax(data->mBodyA) ||
                IsMultibodyAndVelMax(data->mBodyB))
                err = false;
            else
                exit(0);
        }
    }

    // check the generalized vel of joint limit
    bool err = false;
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        double qdot_true = data->multibody->Getqdot()[data->dof_id];
        double qdot_pred = rel_vel_normal_pred[mNumContactPairs + i];
        if (data->is_upper_bound)
            qdot_pred *= -1;
        // std::cout << "qdot pred = " << rel_vel_normal_pred.transpose() <<
        // std::endl; std::cout << "qdot = " <<
        // data->multibody->Getqdot().transpose() << std::endl;
        double diff = qdot_true - qdot_pred;
        if (diff > EPS)
        {
            err = true;
            std::cout
                << "[error] cartesian force to joint limit qdot for contact "
                << i << "---------------\n";
            std::cout << "pred qdot = " << qdot_pred << std::endl;
            std::cout << "true qdot = " << qdot_true << std::endl;
            std::cout << "diff = " << diff << std::endl;
        }
        if (err == true)
        {
            if (data->multibody->IsGeneralizedMaxVel())
                err = false;
            else
                exit(0);
        }
    }

    PopState("TestCartesianForceToNormalAndTangetRelVel");
    // std::cout << "TestCartesianForceToNormalAndTangetRelVel tested well" <<
    // std::endl; exit(0);
}

void btGenContactSolver::TestCartesianForceToNormalAndTangetResultBasedRelVel(
    const tMatrixXd &normal_mat, const tVectorXd &normal_vec,
    const tMatrixXd &tan_mat, const tVectorXd &tan_vec)
{
    if (0 == mNumConstraints)
        return;
    PushState("TestCartesianForceToNormalAndTangetResultBasedRelVel");

    int n_group = mColGroupData.size();
    int single_size = 1;
    if (mEnableFrictionalLCP)
        single_size += mNumFrictionDirs + 1;
    int final_shape =
        single_size * mNumContactPairs + mNumJointLimitConstraints;
    tVectorXd applied_force(mNumContactPairs * 3 + mNumJointLimitConstraints);
    applied_force.setZero();
    x_lcp.resize(final_shape);
    x_lcp.setRandom();
    // x_lcp.setZero();

    // get normal and tangent vel prediction on each point
    tVectorXd rel_vel_normal_pred = normal_mat * x_lcp + normal_vec;
    tVectorXd rel_vel_tangent_pred =
        tVectorXd::Zero(mNumFrictionDirs * mNumContactPairs);
    if (mEnableFrictionalLCP)
    {
        // std::cout <<"[rel vel tan] vec = " << tan_vec.transpose() <<
        // std::endl;
        rel_vel_tangent_pred = tan_mat * x_lcp + tan_vec;
    }

    // apply active force

    tEigenArr<tVector> AppliedActiveForce(0);
    std::vector<int> AppliedGenForce(0);
    for (int i = 0; i < mNumContactPairs; i++)
    {
        auto &data = mContactPairConsData[i];
        if (mEnableFrictionalLCP)
        {
            applied_force.segment(3 * i, 3) =
                data->mS * x_lcp.segment(single_size * i, single_size);
        }
        else
        {
            applied_force.segment(3 * i, 3) =
                data->mNormalPointToA.segment(0, 3) *
                x_lcp.segment(single_size * i, single_size);
        }
        data->ApplyForceCartersian(
            btMathUtil::Expand(applied_force.segment(3 * i, 3), 0));
        AppliedActiveForce.push_back(
            btMathUtil::Expand(applied_force.segment(3 * i, 3), 0));
    }

    // apply joint torque
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        data->ApplyGeneralizedForce(x_lcp(single_size * mNumContactPairs + i));
        AppliedGenForce.push_back(x_lcp(single_size * mNumContactPairs + i));
    }
    TestAddContactAwareForceIfPossible(AppliedActiveForce, AppliedGenForce);

    UpdateVelocity(cur_dt);

    // get the true vel and update
    for (int i = 0; i < mNumContactPairs; i++)
    {
        bool err = false;
        auto &data = mContactPairConsData[i];
        tVector true_rel_vel = data->GetVelOnBody0() - data->GetVelOnBody1();
        tVectorXd true_normal_relvel =
            data->mNormalPointToA.transpose() * true_rel_vel;

        tVectorXd true_tan_relvel = tVectorXd::Zero(3);
        if (mEnableFrictionalLCP)
        {
            true_tan_relvel =
                data->mD.transpose() *
                (true_rel_vel - data->mNormalPointToA * true_normal_relvel[0])
                    .segment(0, 3);
        }

        // std::cout <<"true 1 = " << true_tan_relvel << std::endl;
        tVectorXd pred_normal_relvel = rel_vel_normal_pred.segment(i, 1);
        tVectorXd pred_tan_relvel = rel_vel_tangent_pred.segment(
            i * mNumFrictionDirs, mNumFrictionDirs);
        double diff =
            (pred_normal_relvel - true_normal_relvel).cwiseAbs().maxCoeff();
        if (diff > EPS)
        {
            std::cout
                << "[error] convert x into decomposed normal vel, for contact "
                << i << "---------------\n";
            std::cout << "total contact = " << mNumContactPairs << std::endl;
            std::cout << "S = " << data->mS << std::endl;
            std::cout << "pred normal x-based rel vel = "
                      << pred_normal_relvel.transpose() << std::endl;
            std::cout << "true normal x-based rel vel = "
                      << true_normal_relvel.transpose() << std::endl;
            err = true;
            std::cout << "diff = " << diff << std::endl;
            if (IsMultibodyAndVelMax(data->mBodyA) ||
                IsMultibodyAndVelMax(data->mBodyB))
            {
                err = false;
            }
        }

        if (mEnableFrictionalLCP &&
            (pred_tan_relvel - true_tan_relvel).cwiseAbs().maxCoeff() > EPS)
        {
            std::cout
                << "[error] convert x into decomposed tangent vel, for contact "
                << i << "---------------\n";
            std::cout << "total contact = " << mNumContactPairs << std::endl;
            std::cout << "S = " << data->mS << std::endl;
            std::cout << "pred tan x-based rel vel = "
                      << pred_tan_relvel.transpose() << std::endl;
            std::cout << "true tan x-based rel vel = "
                      << true_tan_relvel.transpose() << std::endl;
            std::cout
                << "diff tan x-based rel vel = "
                << (pred_tan_relvel - true_tan_relvel).cwiseAbs().maxCoeff()
                << std::endl;
            err = true;

            if (IsMultibodyAndVelMax(data->mBodyA) ||
                IsMultibodyAndVelMax(data->mBodyB))
            {
                err = false;
            }
        }

        if (err)
        {
            std::cout << "applied cartesian force = "
                      << applied_force.transpose() << std::endl;
            tVectorXd normal_vel_another =
                normal_vel_convert_mat * applied_force + normal_vel_convert_vec;
            tVectorXd tangent_vel_another =
                tangent_vel_convert_mat * applied_force +
                tangent_vel_convert_vec;
            std::cout << "normal vel another = "
                      << normal_vel_another.transpose() << std::endl;
            std::cout << "normal vel this = " << rel_vel_normal_pred.transpose()
                      << std::endl;

            std::cout << "tangent vel another = "
                      << tangent_vel_another.transpose() << std::endl;
            std::cout << "tangent vel this = "
                      << rel_vel_tangent_pred.transpose() << std::endl;
        }
        if (err)
            exit(0);
    }

    // get the true generalized velocity
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        double qdot_true = data->multibody->Getqdot()[data->dof_id];
        double qdot_pred = rel_vel_normal_pred[mNumContactPairs + i];
        if (data->is_upper_bound)
            qdot_pred *= -1;
        double diff = std::fabs(qdot_true - qdot_pred);
        bool err = false;
        if (diff > EPS)
        {
            if (false == data->multibody->IsGeneralizedMaxVel())
            {
                std::cout
                    << "[error] convert x into generalized force, for contact "
                    << i << "---------------\n";
                std::cout << "qdot pred = " << qdot_pred << std::endl;
                std::cout << "qdot true = " << qdot_true << std::endl;
                std::cout << "diff = " << diff << std::endl;
                err = true;
            }
        }
        if (err)
            exit(0);
    }

    // exit(0);
    PopState("TestCartesianForceToNormalAndTangetResultBasedRelVel");
}

/**
 * \brief       Given current randomed contact_force and constraint_gen_force, this function calculate the contact-aware control force and applied
 * \param contact_forces        contact forces on each contact point, size = mNumContactPairs
 * \param constraint_forces     gen constarint forces, size should be zero at this moment
 * 
*/
#include "BulletGenDynamics/btGenController/ContactAwareController/btGenContactAwareController.h"
void btGenContactSolver::TestAddContactAwareForceIfPossible(
    const tEigenArr<tVector> &contact_forces,
    const std::vector<int> &constraint_forces)
{
    // 1. judge if multibody & contact aware is enable. if not, return
    cRobotModelDynamics *model = CollectMultibody();
    if (model == nullptr || model->GetEnableContactAwareController() == false)
        return;

    // 2. confirm the constraint forces size = 0. if not, exit
    if (constraint_forces.size() != 0)
    {
        printf("[error] TestAddContactAwareForceIfPossible: constraint force "
               "size %d unsupported\n",
               constraint_forces.size());
        exit(0);
    }

    // 3. calculate contact-aware control force
    tVectorXd control_force;
    {
        // 3.1 get controller
        btGenContactAwareController *controller =
            mGenWorld->GetContactAwareController();
        assert(controller != nullptr);

        // 3.2 collect contact forces
        assert(contact_forces.size() == this->mNumContactPairs);
        tVectorXd gen_contact_force = tVectorXd::Zero(model->GetNumOfFreedom());
        for (int i = 0; i < mNumContactPairs; i++)
        {
            const auto &data = mContactPairConsData[i];
            tVector3d force = contact_forces[i].segment(0, 3);
            tMatrixXd jac;
            if (data->mTypeA == eColObjType::RobotCollder)
            {
                tVector3d global_pos = data->mContactPosOnA.segment(0, 3);
                int link_id =
                    dynamic_cast<btGenRobotCollider *>(data->mBodyA)->mLinkId;
                model->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                    link_id, global_pos, jac);
                gen_contact_force += jac.transpose() * force;
            }
            if (data->mTypeB == eColObjType::RobotCollder)
            {
                force *= -1;
                tVector3d global_pos = data->mContactPosOnB.segment(0, 3);
                int link_id =
                    dynamic_cast<btGenRobotCollider *>(data->mBodyB)->mLinkId;
                model->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                    link_id, global_pos, jac);
                gen_contact_force += jac.transpose() * force;
            }
        }
        // 3.3 fetch control force
        control_force = controller->CalcControlForce(gen_contact_force, false);
    }
    // 4. apply the contact-aware control force and return
    // std::cout << "[debug] TestAddContactAwareCtrlForce = "
    //           << control_force.transpose() << std::endl;
    int num_of_actuated_freedom = model->GetNumOfFreedom() - 6;
    assert(num_of_actuated_freedom == control_force.size());
    for (int i = 0; i < num_of_actuated_freedom; i++)
    {
        model->ApplyGeneralizedForce(i + 6, control_force[i]);
    }
}

/**
 * \brief           Test the gradient of d(n)/d(ctrl_force)
*/
void btGenContactSolver::TestDnDCtrlForce()
{
    // 1. save
    if (mMultibodyArray.size() == 0 ||
        mNumJointLimitConstraints + mNumContactPairs == 0)
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
        mNumJointLimitConstraints + mNumContactPairs == 0)
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
        mNumJointLimitConstraints + mNumContactPairs == 0)
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