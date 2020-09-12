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
    int constraint_length = mNumJointLimitConstraints + mNumContactPoints * 3;
    if (0 == constraint_length)
        return;

    int n_group = mColGroupData.size();
    x_lcp.resize(constraint_length);
    x_lcp.setZero();
    x_lcp[0] = 1.0;
    // x_lcp.setRandom();
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
        // 		  << "c2v mat = \n"
        // 		  << mColGroupData[i]->mConvertCartesianForceToVelocityMat
        // << std::endl; std::cout << "for group " << i << " "
        // 		  << "c2v vec = \n"
        // 		  <<
        // mColGroupData[i]->mConvertCartesianForceToVelocityVec.transpose() <<
        // std::endl;
    }

    // apply active force
    for (int i = 0; i < mNumContactPoints; i++)
    {
        auto &data = mContactConstraintData[i];
        data->ApplyForceCartersian(
            btMathUtil::Expand(x_lcp.segment(3 * i, 3), 0));
    }
    // std::cout << "only contact force Q = " <<
    // mMultibodyArray[0]->GetGeneralizedForce().transpose() << std::endl;

    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        data->ApplyGeneralizedForce(x_lcp[3 * mNumContactPoints + i]);
    }
    // std::cout << "Q = " <<
    // mMultibodyArray[0]->GetGeneralizedForce().transpose() << std::endl;

    // update velocity
    UpdateVelocity(cur_dt);

    // get the true vel and update
    bool err = false;
    for (int i = 0; i < mNumContactPoints; i++)
    {
        auto &data = mContactConstraintData[i];
        if (data->mIsSelfCollision)
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
                      << "/" << mNumContactPoints << "---------------\n";
            std::cout << "body0 is " << data->mBodyA->GetName() << " body1 is "
                      << data->mBodyB->GetName() << std::endl;
            std::cout << "contact normal = "
                      << data->mNormalPointToA.transpose() << std::endl;
            std::cout << "body 0 pred vel = " << body0_pred_vel.transpose()
                      << std::endl;
            std::cout << "body 0 true vel = " << body0_true_vel.transpose()
                      << std::endl;
            std::cout << "body0 diff = "
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
            contact_point_cartesian_vel[group_id][mNumContactPoints * 3 + i];
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
        exit(1);
    }
    // std::cout << "test abs vel succ\n";
    // exit(12);
    // if (constraint_length > 3) exit(1);
    PopState("TestCartesianForceToCartesianVel");
}

void btGenContactSolver::TestCartesianForceToCartesianRelVel(
    const tMatrixXd &convert_mat, const tVectorXd &convert_vec)
{
    int constraint_length = mNumJointLimitConstraints + mNumContactPoints * 3;
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
    for (int i = 0; i < mNumContactPoints; i++)
    {
        auto &data = mContactConstraintData[i];
        data->ApplyForceCartersian(
            btMathUtil::Expand(x_lcp.segment(3 * i, 3), 0));
        if (data->mIsSelfCollision)
            n_self_collision++;
    }

    // for joint limits
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        data->ApplyGeneralizedForce(x_lcp[3 * mNumContactPoints + i]);
    }

    // get the true vel and update
    UpdateVelocity(cur_dt);

    // check the velocity for each contact point
    for (int i = 0; i < mNumContactPoints; i++)
    {
        auto &data = mContactConstraintData[i];
        tVector true_relvel = data->GetVelOnBody0() - data->GetVelOnBody1();
        tVector pred_relvel =
            btMathUtil::Expand(rel_vel_pred.segment(i * 3, 3), 0);
        double diff = (true_relvel - pred_relvel).cwiseAbs().maxCoeff();
        if (diff > EPS)
        {
            bool err = false;
            err = true;
            std::cout
                << "error convert cartesian force to rel vel: for contact " << i
                << "---------------\n";
            std::cout << "pred rel vel = " << pred_relvel.transpose()
                      << std::endl;
            std::cout << "true rel vel = " << true_relvel.transpose()
                      << std::endl;
            std::cout << "diff = " << diff << std::endl;

            std::cout << "total contact num = " << mNumContactPoints
                      << std::endl;
            std::cout << "self collision num = " << n_self_collision
                      << std::endl;
            if (IsMultibodyAndVelMax(data->mBodyA) ||
                IsMultibodyAndVelMax(data->mBodyB))
            {
                err = false;
            }
            if (err)
                exit(1);
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
        double qdot_pred = rel_vel_pred[3 * mNumContactPoints + i];
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
            exit(1);
    }
    PopState("TestCartesianForceToCartesianRelVel");
}

void btGenContactSolver::TestCartesianForceToNormalAndTangetRelVel(
    const tMatrixXd &normal_mat, const tVectorXd &normal_vec,
    const tMatrixXd &tan_mat, const tVectorXd &tan_vec)
{
    int final_shape = mNumContactPoints * 3 + mNumJointLimitConstraints;
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
    for (int i = 0; i < mNumContactPoints; i++)
    {
        auto &data = mContactConstraintData[i];
        data->ApplyForceCartersian(
            btMathUtil::Expand(x_lcp.segment(3 * i, 3), 0));
    }

    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        data->ApplyGeneralizedForce(x_lcp[mNumContactPoints * 3 + i]);
    }

    // get the true vel and update
    UpdateVelocity(cur_dt);

    for (int i = 0; i < mNumContactPoints; i++)
    {
        bool err = false;
        auto &data = mContactConstraintData[i];
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
                exit(1);
        }
    }

    // check the generalized vel of joint limit
    bool err = false;
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        double qdot_true = data->multibody->Getqdot()[data->dof_id];
        double qdot_pred = rel_vel_normal_pred[mNumContactPoints + i];
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
                exit(1);
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
        single_size * mNumContactPoints + mNumJointLimitConstraints;
    tVectorXd applied_force(mNumContactPoints * 3 + mNumJointLimitConstraints);
    applied_force.setZero();
    x_lcp.resize(final_shape);
    x_lcp.setRandom();
    // x_lcp.setZero();

    // get normal and tangent vel prediction on each point
    tVectorXd rel_vel_normal_pred = normal_mat * x_lcp + normal_vec;
    tVectorXd rel_vel_tangent_pred =
        tVectorXd::Zero(mNumFrictionDirs * mNumContactPoints);
    if (mEnableFrictionalLCP)
    {
        // std::cout <<"[rel vel tan] vec = " << tan_vec.transpose() <<
        // std::endl;
        rel_vel_tangent_pred = tan_mat * x_lcp + tan_vec;
    }

    // apply active force
    for (int i = 0; i < mNumContactPoints; i++)
    {
        auto &data = mContactConstraintData[i];
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
    }

    // apply joint torque
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        data->ApplyGeneralizedForce(x_lcp(single_size * mNumContactPoints + i));
    }
    UpdateVelocity(cur_dt);

    // get the true vel and update
    for (int i = 0; i < mNumContactPoints; i++)
    {
        bool err = false;
        auto &data = mContactConstraintData[i];
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
            std::cout << "total contact = " << mNumContactPoints << std::endl;
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
            std::cout << "total contact = " << mNumContactPoints << std::endl;
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
            exit(1);
    }

    // get the true generalized velocity
    for (int i = 0; i < mNumJointLimitConstraints; i++)
    {
        auto &data = mJointLimitConstraintData[i];
        double qdot_true = data->multibody->Getqdot()[data->dof_id];
        double qdot_pred = rel_vel_normal_pred[mNumContactPoints + i];
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
            exit(1);
    }

    // exit(0);
    PopState("TestCartesianForceToNormalAndTangetResultBasedRelVel");
}