#include "ShowLimitController.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"

void system_generate(int digits, int pos_notation, int cur_id, tVectorXd &res,
                     tEigenArr<tVectorXd> &collections) //求所有n位k进制数
{
    BTGEN_ASSERT(pos_notation <= 10);
    if (cur_id == digits)
    {
        collections.push_back(res);
        return;
    }
    else
    {
        for (int i = 0; i < pos_notation; ++i)
        {
            res[cur_id] = i;
            system_generate(digits, pos_notation, cur_id + 1, res, collections);
        }
    }
}

btGenShowLimitController::btGenShowLimitController(btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::ShowLimitController, world)
{
    mDrawLimit = false;
    mCurFrame = 0;
    // mTargetJoint = nullptr;
}
btGenShowLimitController::~btGenShowLimitController() {}
void btGenShowLimitController::Init(cRobotModelDynamics *model,
                                    const std::string &conf)
{
    btGenControllerBase::Init(model, conf);

    // 1. load the joint name
    Json::Value root;
    btJsonUtil::LoadJson(conf, root);
    // std::string target_name = btJsonUtil::ParseAsString("target_joint", root);
    // mTargetJoint = dynamic_cast<Joint *>(model->GetJoint(target_name));
    // BTGEN_ASSERT(mTargetJoint != nullptr);
    // std::cout << 'show the limit of joint ' << mTargetJoint->GetId()
    //           << mTargetJoint->GetName() << std::endl;

    mInitPose = tVectorXd::Zero(mModel->GetNumOfFreedom());
    switch (mModel->GetRoot()->GetJointType())
    {
    case JointType::NONE_JOINT:
        mInitPose[1] = 1;
        break;
    case JointType::BIPEDAL_NONE_JOINT:
        mInitPose[0] = 1;
        break;

    default:
        BTGEN_ASSERT(false);
        break;
    }
    mModel->Setq(mInitPose);
}

void btGenShowLimitController::Update(double dt)
{
    mCurFrame += 1;
    // std::cout << "init pose = " << mInitPose.transpose() << std::endl;
    if (mDrawLimit == false)
    {
        for (int i = 1; i < mModel->GetNumOfJoint(); i++)
        {
            DrawLimit(dynamic_cast<Joint *>(mModel->GetJointById(i)));
        }
        mDrawLimit = true;
    }
    mModel->ClearForce();
    mModel->Setq(mInitPose);
}

void btGenShowLimitController::Reset() {}

/**
 * \brief       Draw spheres on the joint limit 
*/
void btGenShowLimitController::DrawLimit(Joint *joint)
{
    mModel->PushState("draw");

    tVectorXd lb, ub;
    mModel->GetJointLimit(lb, ub);
    int offset = joint->GetOffset(), size = joint->GetNumOfFreedom();
    tVectorXd joint_lb = lb.segment(offset, size),
              joint_ub = ub.segment(offset, size);
    tEigenArr<tVectorXd> joint_angle_list(0);
    const int sample_num = 10;
    // std::cout << "joint lb = " << joint_lb.transpose() << std::endl;
    // std::cout << "joint ub = " << joint_ub.transpose() << std::endl;
    {
        for (int i = 0; i < size; i++)
        {
            BTGEN_ASSERT(joint_ub[i] >= joint_lb[i]);
            double gap = (joint_ub[i] - joint_lb[i]) / (sample_num - 1);
            tVectorXd angle = tVectorXd::Zero(sample_num);
            for (int j = 0; j < sample_num; j++)
            {
                angle[j] = joint_lb[i] + gap * j;
            }
            // std::cout << "for dof " << i << " angle = " << angle.transpose()
            //           << std::endl;
            joint_angle_list.push_back(angle);
        }
    }
    // exit(0);
    tVectorXd buf = tVectorXd::Zero(size);
    tEigenArr<tVectorXd> collections(0);
    system_generate(size, sample_num, 0, buf, collections);

    tVectorXd q = mModel->Getq();
    mModel->SetComputeSecondDerive(false);
    mModel->SetComputeThirdDerive(false);
    double length =
        mModel->GetLinkById(joint->GetId())->GetMeshScale().maxCoeff() / 3;
    // std::cout << "length = " << length << std::endl;
    tVector outgoing_dir = GetJointOutgoingDir(joint);
    for (auto &x : collections)
    {
        tVectorXd cur_joint_theta = tVectorXd::Zero(size);
        for (int i = 0; i < size; i++)
        {
            cur_joint_theta[i] = joint_angle_list[i][x[i]];
            joint->GetFreedoms(i)->v = joint_angle_list[i][x[i]];
        }
        joint->UpdateState(false);

        // mModel->SetqAndqdot(q, tVectorXd::Zero(mModel->GetNumOfFreedom()));

        tVector3d pos = (joint->GetGlobalTransform() *
                         btMathUtil::Expand(outgoing_dir * length, 1))
                            .segment(0, 3);
        // DrawPoint(tVector3d::Random(), 0.05);
        DrawPoint(pos, 0.05);
        // std::cout << "draw pos = " << pos.transpose() << std::endl;
    }

    
    // exit(0);
    mModel->PopState("draw");
}

/**
 * \brief           Given a joint, calculate the local outgoing direction, in current pose
*/
tVector btGenShowLimitController::GetJointOutgoingDir(Joint *joint) const
{
    tVector3d link_world_pos =
                  mModel->GetLinkById(joint->GetId())->GetWorldPos(),
              joint_world_pos = joint->GetWorldPos();
    return btMathUtil::Expand((joint->GetWorldOrientation().transpose() *
                               (link_world_pos - joint_world_pos))
                                  .normalized(),
                              0);
}