#include "btTraj.h"
#include "BulletGenDynamics/btGenModel/RobotCollider.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenSolver/ContactSolver.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include <iostream>

int GetModelPoseSize(cRobotModelDynamics *model)
{
    int pose_size = 0;
    for (int i = 0; i < model->GetNumOfJoint(); i++)
    {
        auto type = model->GetJointById(i)->GetJointType();
        switch (type)
        {
        case JointType::FIXED_JOINT:
            pose_size += 0;
            break;
        case JointType::REVOLUTE_JOINT:
            pose_size += 1;
            break;
        case JointType::SPHERICAL_JOINT:
            pose_size += 4;
            break;
        case JointType::NONE_JOINT:
            pose_size += 7;
            break;
        default:
            std::cout << "get model pose size unsuporeted joint type " << type
                      << std::endl;
            exit(0);
            break;
        }
    }
    return pose_size;
}
/**
 * \brief				Convert generalized coordinate q to char
 * pose
 *
 */
tVectorXd ConvertqToPose(const tVectorXd &q, cRobotModelDynamics *model)
{
    if (q.size() != model->GetNumOfFreedom())
    {
        std::cout << "[error] convert q to pose illegal, size = " << q.size()
                  << " " << model->GetNumOfFreedom() << std::endl;
        exit(0);
    }
    int pose_size = GetModelPoseSize(model);
    tVectorXd pose = tVectorXd::Zero(pose_size);
    int pose_st = 0, q_st = 0;

    // iteration on each joint
    // TODO: finish it here.
    int num_of_joints = model->GetNumOfJoint();
    for (int i = 0; i < num_of_joints; i++)
    {
        auto joint_type = model->GetJointById(i)->GetJointType();
        switch (joint_type)
        {
        case JointType::NONE_JOINT:
        {
            // for root
            pose.segment(pose_st, 3) = q.segment(q_st, 3);
            pose_st += 3, q_st += 3;

            tQuaternion root_rot = btMathUtil::EulerAnglesToQuaternion(
                btMathUtil::Expand(q.segment(q_st, 3), 0),
                btRotationOrder::bt_XYZ);
            pose[pose_st] = root_rot.w();
            pose[pose_st + 1] = root_rot.x();
            pose[pose_st + 2] = root_rot.y();
            pose[pose_st + 3] = root_rot.z();
            pose_st += 4, q_st += 3;
        }
        break;
        case JointType::FIXED_JOINT:
            break;
        case JointType::REVOLUTE_JOINT:
        {
            pose[pose_st] = q[q_st];
            q_st++, pose_st++;
        }
        break;
        case JointType::SPHERICAL_JOINT:
        {
            tQuaternion joint_rot = btMathUtil::EulerAnglesToQuaternion(
                btMathUtil::Expand(q.segment(q_st, 3), 0),
                btRotationOrder::bt_XYZ);
            pose[pose_st] = joint_rot.w();
            pose[pose_st + 1] = joint_rot.x();
            pose[pose_st + 2] = joint_rot.y();
            pose[pose_st + 3] = joint_rot.z();
            pose_st += 4, q_st += 3;
        }
        default:
            break;
        }
    }
    return pose;
}

/**
 * \brief				Convert char pose to the generalized
 * coordinate
 * q
 */
tVectorXd ConvertPoseToq(const tVectorXd &pose, cRobotModelDynamics *model)
{
    // std::cout << "convert pose to q hasn't been implemented\n";
    int num_of_links = model->GetNumOfLinks();
    int num_of_freedom = model->GetNumOfFreedom();
    tVectorXd q = tVectorXd::Zero(num_of_freedom);
    int pose_st = 0, pose_dof = 0, q_st = 0, q_dof = 0;
    for (int joint_id = 0; joint_id < num_of_links; joint_id++)
    {
        auto cur_joint = model->GetJointById(joint_id);
        switch (cur_joint->GetJointType())
        {
        case JointType::NONE_JOINT:
        {
            pose_dof = 7;
            q_dof = 6;
            if (pose_st + pose_dof > pose.size() || q_dof + q_st > q.size())
                std::cout << "[error] ConvertPoseToq failed, joint id "
                          << joint_id << " cause an illegal dof, exit\n",
                    exit(0);
            else
            {
                q.segment(q_st, 3) = pose.segment(pose_st, 3);
                tVector w_x_y_z = pose.segment(pose_st + 3, 4);
                // std::cout << "joint " << joint_id << "pose = " <<
                // pose.transpose() << std::endl;
                q.segment(q_st + 3, 3) =
                    btMathUtil::QuaternionToEulerAngles(
                        tQuaternion(w_x_y_z[0], w_x_y_z[1], w_x_y_z[2],
                                    w_x_y_z[3]),
                        btRotationOrder::bt_XYZ)
                        .segment(0, 3);
                // std::cout << "joint " << joint_id << "pose = " <<
                // pose.transpose() << std::endl;
            }
        }
        break;
        case JointType::SPHERICAL_JOINT:
        {
            pose_dof = 4;
            q_dof = 3;
            if (pose_st + pose_dof > pose.size() || q_dof + q_st > q.size())
                std::cout << "［error] ConvertPoseToq failed, joint id "
                          << joint_id << " cause an illegal dof, exit\n",
                    exit(0);
            else
            {
                tVector w_x_y_z = pose.segment(pose_st, 4);
                q.segment(q_st, 3) = btMathUtil::QuaternionToEulerAngles(
                                         tQuaternion(w_x_y_z[0], w_x_y_z[1],
                                                     w_x_y_z[2], w_x_y_z[3]),
                                         btRotationOrder::bt_XYZ)
                                         .segment(0, 3);
            }
        }
        break;
        case JointType::REVOLUTE_JOINT:
        {
            pose_dof = 1;
            q_dof = 1;
            if (pose_st + pose_dof > pose.size() || q_dof + q_st > q.size())
                std::cout
                    << "［error] ConvertPoseToq failed, joint id joint id "
                    << joint_id << " cause an illegal dof, exit\n",
                    exit(0);
            else
            {
                q[q_st] = pose[pose_st];
            }
        }
        break;
        case JointType::FIXED_JOINT:
            pose_dof = 0;
            q_dof = 0;
            break;
        default:
            std::cout
                << "［error] ConvertPoseToq failed, joint id joint id joint id "
                << joint_id << " cause an illegal dof, type "
                << cur_joint->GetJointType() << std::endl;
            break;
        }

        pose_st += pose_dof;
        q_st += q_dof;
    }
    // std::cout << "[conv] q = " << q.transpose() << std::endl;
    // std::cout << "[conv] pose = " << pose.transpose() << std::endl;
    // exit(0);
    return q;
}

btTraj::btTraj()
{
    mq.clear();
    mqdot.clear();
    mqddot.clear();
    mActiveForce.clear();
    mContactForce.clear();
    mTruthJointForce.clear();
    mTimestep = 0;
}
btTraj::~btTraj()
{
    for (auto &frame_fcs : mContactForce)
    {
        for (auto &i : frame_fcs)
            delete i;
        frame_fcs.clear();
    }
    mContactForce.clear();
}
#include <algorithm>
bool btTraj::LoadTraj(const std::string &path, cRobotModelDynamics *model,
                      int max_frame /* = -1*/)
{
    std::cout << "[btTraj] Load traj v2 from " << path << std::endl;
    // 1. clear the ref traj
    {
        mq.clear();
        mqdot.clear();
        mqddot.clear();
        mActiveForce.clear();
        mContactForce.clear();
        mTruthJointForce.clear();
    }

    Json::Value root;
    btJsonUtil::LoadJson(path, root);
    if (2 != btJsonUtil::ParseAsInt("version", root))
    {
        std::cout << "[error] traj version is not 2, exit\n";
        exit(0);
    }

    Json::Value data_lst = btJsonUtil::ParseAsValue("list", root);
    mNumOfFrames = data_lst.size();
    if (max_frame != -1)
        mNumOfFrames = std::min(max_frame, mNumOfFrames);

    mq.resize(mNumOfFrames);
    mqdot.resize(mNumOfFrames);
    mqddot.resize(mNumOfFrames);
    mActiveForce.resize(mNumOfFrames);
    mContactForce.resize(mNumOfFrames);
    mTruthJointForce.resize(mNumOfFrames);
    for (auto &f : mTruthJointForce)
        f.resize(model->GetNumOfJoint() - 1, tVector::Zero());
    // mContactLocaPos.resize(mNumOfFrames);

    for (auto &x : mContactForce)
        x.clear();

    for (int frame_id = 0; frame_id < mNumOfFrames; frame_id++)
    {
        Json::Value cur_data = data_lst[frame_id];

        Json::Value char_pose_value =
            btJsonUtil::ParseAsValue("char_pose", cur_data);
        mTimestep = btJsonUtil::ParseAsDouble("timestep", cur_data);
        // std::cout << "dt = " << dt << std::endl;
        Json::Value action_value =
            btJsonUtil::ParseAsValue("truth_action", cur_data);
        int num_of_contact = btJsonUtil::ParseAsInt("contact_num", cur_data);
        Json::Value contact_info_value =
            btJsonUtil::ParseAsValue("contact_info", cur_data);

        // load motion
        // std::cout << "---------------frame " << frame_id <<
        // "------------------\n";
        tVectorXd char_pose;
        btJsonUtil::ReadVectorJson(char_pose_value, char_pose);
        mq[frame_id] = ConvertPoseToq(char_pose, model);
        btJsonUtil::ReadVectorJson(action_value, mActiveForce[frame_id]);
        // std::cout << "char pose = " << char_pose.transpose() << std::endl;
        // std::cout << "action = " << mActiveForce[frame_id].transpose() <<
        // std::endl;

        // contact force
        mContactForce[frame_id].resize(num_of_contact);
        // mContactLocaPos[frame_id].resize(num_of_contact);
        for (int c_id = 0; c_id < num_of_contact; c_id++)
        {
            auto &cur_contact_value_json = contact_info_value[c_id];
            auto &cur_contact_value = mContactForce[frame_id][c_id];
            int link_id =
                btJsonUtil::ParseAsInt("force_link_id", cur_contact_value_json);
            tVectorXd pos, force_value;
            btJsonUtil::ReadVectorJson(
                btJsonUtil::ParseAsValue("force_pos", cur_contact_value_json),
                pos);
            btJsonUtil::ReadVectorJson(
                btJsonUtil::ParseAsValue("force_value", cur_contact_value_json),
                force_value);
            bool is_self_collision = btJsonUtil::ParseAsBool(
                "is_self_collision", cur_contact_value_json);
            // if (is_self_collision == true)
            // {
            // 	std::cout << "[error] btGenContactAdviser: self collision hasn't
            // been supported\n"; 	exit(0);
            // }
            btGenCollisionObject *obj = model->GetLinkCollider(link_id);
            // std::cout << "contact " << c_id << "link id " << link_id << " pos
            // " << pos.transpose() << " force " << force_value.transpose() <<
            // std::endl;
            cur_contact_value =
                new btGenContactForce(obj, force_value.segment(0, 4),
                                      pos.segment(0, 4), is_self_collision);
        }
        // truth joint forces
        tVectorXd joint_forces;
        btJsonUtil::ReadVectorJson(
            btJsonUtil::ParseAsValue("truth_joint_force", cur_data),
            joint_forces);
        // std::cout << "frame " << frame_id
        //           << " joint force = " << joint_forces.transpose() << std::endl;
        int num_of_actuated_joint = model->GetNumOfJoint() - 1;
        if (num_of_actuated_joint * 4 != joint_forces.size())
        {
            std::cout << "[error] bt load traj v2 requsted joint force length "
                      << num_of_actuated_joint << " != " << joint_forces.size()
                      << std::endl;
            exit(0);
        }
        for (int i = 0; i < num_of_actuated_joint; i++)
        {
            for (int j = 0; j < 4; j++)
                mTruthJointForce[frame_id][i][j] = joint_forces[i * 4 + j];
        }
    }

    // calcualte qdot and qddot
    for (int frame_id = 1; frame_id < mNumOfFrames - 1; frame_id++)
    {
        // std::cout << mNumOfFrames << " " << frame_id << std::endl;
        // std::cout << "---------------frame " << frame_id <<
        // "------------------\n";
        mqdot[frame_id] = (mq[frame_id] - mq[frame_id - 1]) / mTimestep;
        mqddot[frame_id] =
            (-2 * mq[frame_id] + mq[frame_id - 1] + mq[frame_id + 1]) /
            (mTimestep * mTimestep);
        // std::cout << "mq = " << mq[frame_id].transpose() << std::endl;
        // std::cout << "mqdot = " << mqdot[frame_id].transpose() << std::endl;
        // std::cout << "mqddot = " << mqddot[frame_id].transpose() <<
        // std::endl; exit(0);
    }

    // get all contact forces, sort and find some biggest
    // {
    //     tEigenArr<btGenContactForce *> forces(0);
    //     for (auto &frame : mContactForce)
    //     {
    //         for (auto &f : frame)
    //         {
    //             forces.push_back(f);
    //         }
    //     }
    //     auto comp = [](const btGenContactForce *f1,
    //                    const btGenContactForce *f2) {
    //         return f1->mForce.norm() > f2->mForce.norm();
    //     };

    //     std::sort(forces.begin(), forces.end(), comp);
    //     for (int i = 0; i < 10; i++)
    //     {
    //         std::cout << forces[i]->mForce.transpose() << std::endl;
    //     }
    //     exit(1);
    // }
    return true;
}

bool btTraj::SaveTraj(const std::string &path, cRobotModelDynamics *model)
{
    Json::Value root;
    root["version"] = 2;
    root["list"] = Json::arrayValue;

    for (int i = 0; i < mNumOfFrames; i++)
    {
        // std::cout << "begin to save frame " << i << std::endl;
        Json::Value data_perframe;

        data_perframe["char_pose"] =
            btJsonUtil::BuildVectorJsonValue(ConvertqToPose(mq[i], model));
        data_perframe["timestep"] = this->mTimestep;
        data_perframe["truth_action"] =
            btJsonUtil::BuildVectorJsonValue(this->mActiveForce[i]);
        data_perframe["contact_num"] = this->mContactForce[i].size();
        data_perframe["contact_info"] = Json::arrayValue;
        for (auto &f : mContactForce[i])
        {
            // std::cout << "f obj = " << f->mObj << std::endl;
            auto collider = dynamic_cast<btGenRobotCollider *>(f->mObj);
            // std::cout << "f collider = " << collider << std::endl;
            // std::cout << "f link id = " << collider->mLinkId << std::endl;
            Json::Value contact_info;
            contact_info["force_link_id"] = collider->mLinkId;
            contact_info["force_pos"] =
                btJsonUtil::BuildVectorJsonValue(f->mWorldPos);
            contact_info["force_value"] =
                btJsonUtil::BuildVectorJsonValue(f->mForce);
            contact_info["is_self_collision"] = f->mIsSelfCollision;
            data_perframe["contact_info"].append(contact_info);
        }
        root["list"].append(data_perframe);
    }
    btJsonUtil::WriteJson(path, root, true);
    std::cout << "Save traj v2 " << path << std::endl;
    return true;
    /*
1. version
2. list:
[
    {
            char_pose,
            timestep,
            truth_action,
            contact_num,
            contact_info :
            [
                    {
                            force_link_id
                            force_pos,
                            force_value,
                            is_self_collision
                    }
            ]

    }
]
*/
}

void btTraj::Reshape(int num_of_frame_new)
{
    mNumOfFrames = num_of_frame_new;
    mq.resize(num_of_frame_new);
    mqdot.resize(num_of_frame_new);
    mqddot.resize(num_of_frame_new);
    mActiveForce.resize(num_of_frame_new);
    mContactForce.resize(num_of_frame_new);
}

double btTraj::GetTimeLength() const { return mTimestep * (mNumOfFrames - 1); }

tVectorXd btTraj::GetGenContactForce(int frame_id, cRobotModelDynamics *model)

{
    if (frame_id >= mNumOfFrames)
    {
        std::cout << "[error] btTraj::GetGenContactForce for frame " << frame_id
                  << " illegal\n";
        exit(0);
    }
    model->PushState("calc contact force");
    model->SetqAndqdot(this->mq[frame_id], mqdot[frame_id]);
    auto contact_forces = this->mContactForce[frame_id];
    tVectorXd Q = tVectorXd::Zero(model->GetNumOfFreedom());
    for (auto fc : contact_forces)
    {
        auto link = dynamic_cast<btGenRobotCollider *>(fc->mObj);
        tMatrixXd jac;
        model->ComputeJacobiByGivenPointTotalDOFWorldFrame(
            link->mLinkId, fc->mWorldPos.segment(0, 3), jac);
        Q += jac.transpose() * fc->mForce.segment(0, 3);
    }
    model->PopState("calc contact force");
    return Q;
}

tVectorXd btTraj::GetGenControlForce(int frame_id, cRobotModelDynamics *model)
{
    tVectorXd legacy_active_force = tVectorXd::Zero(model->GetNumOfFreedom());
    int num_of_actuated_joint = model->GetNumOfJoint() - 1;
    auto truth_joint_forces = mTruthJointForce[frame_id];
    if (truth_joint_forces.size() != num_of_actuated_joint)
    {

        std::cout << "[error] the num of actuated joints is different "
                  << num_of_actuated_joint
                  << " != " << truth_joint_forces.size() << std::endl;
        exit(0);
    }
    for (int joint_id = 1; joint_id < model->GetNumOfJoint(); joint_id++)
    {
        auto joint = model->GetJointById(joint_id);
        int offset = joint->GetFreedoms(0)->id;
        int dof = joint->GetNumOfFreedom();
        legacy_active_force.segment(offset, dof) =
            truth_joint_forces[joint_id - 1].segment(0, dof);
    }
    legacy_active_force =
        legacy_active_force.segment(6, legacy_active_force.size() - 6);
    return legacy_active_force;
}