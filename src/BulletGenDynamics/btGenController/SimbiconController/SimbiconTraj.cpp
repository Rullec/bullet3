#include "SimbiconTraj.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include <iostream>
//======================base trajectory begin=======================
BaseTrajectory::BaseTrajectory(const Json::Value &conf)
{
    mRotationAxis = btJsonUtil::ReadVectorJson(
                        btJsonUtil::ParseAsValue("rotation_axis", conf))
                        .segment(0, 3);
    mTimeKnots = btJsonUtil::ReadVectorJson(
        btJsonUtil::ParseAsValue("time_knots", conf));
    mValueKnots = btJsonUtil::ReadVectorJson(
        btJsonUtil::ParseAsValue("value_knots", conf));

    // create balance feedback policy
    std::cout << "[debug] rotation axis = " << mRotationAxis.transpose()
              << " time knots = " << mTimeKnots.transpose()
              << " value knots = " << mValueKnots.transpose() << std::endl;

    if (conf.isMember("feedback") == true)
    {
        Json::Value feedback = btJsonUtil::ParseAsValue("feedback", conf);
        tVector3d feedback_proj_axis =
            btJsonUtil::ReadVectorJson(
                btJsonUtil::ParseAsValue("feedback_projection_axis", feedback))
                .segment(0, 3);
        double cd = btJsonUtil::ParseAsDouble("cd", feedback),
               cv = btJsonUtil::ParseAsDouble("cv", feedback);
        std::cout << "[debug] init feedback, proj axis = "
                  << feedback_proj_axis.transpose() << " cd = " << cd << " cv "
                  << cv << std::endl;
        mBalanceFeedback = new tFeedBack(feedback_proj_axis, cd, cv);
    }
    else
    {
        mBalanceFeedback = nullptr;
    }
    mLeftStanceIndex = mRightStanceIndex = -1;
}

BaseTrajectory::~BaseTrajectory() { delete mBalanceFeedback; }

BaseTrajectory::tFeedBack::tFeedBack(const tVector3d &proj_axis, double cd,
                                     double cv)
{
    mFeedbackProjAxis = proj_axis;
    Cd = cd;
    Cv = cv;
}

//======================base trajectory end=======================
btGenSimbiconTraj::btGenSimbiconTraj(const Json::Value &conf,
                                     cRobotModelDynamics *model)
{
    mJointName = btJsonUtil::ParseAsString("joint", conf);

    mBaseTrajs.clear();

    std::cout << "-----------------------\n";
    std::cout << "for joint " << mJointName << std::endl;
    const Json::Value base_trajs = btJsonUtil::ParseAsValue("base_trajs", conf);
    for (int i = 0; i < base_trajs.size(); i++)
    {
        auto new_base = new BaseTrajectory(base_trajs[i]);

        // resolve the joint id
        {
            std::string SWING_STR = "SWING_", STANCE_STR = "STANCE_";
            std::string base_joint_name = "";
            if (mJointName.find(SWING_STR) != -1)
                base_joint_name = mJointName.substr(SWING_STR.size());
            else if (mJointName.find(STANCE_STR) != -1)
                base_joint_name = mJointName.substr(STANCE_STR.size());
            else
                BTGEN_ASSERT(false);

            std::string left_joint_name = "Left" + base_joint_name,
                        right_joint_name = "Right" + base_joint_name;

            auto left_joint = model->GetLink(left_joint_name),
                 right_joint = model->GetLink(right_joint_name);
            std::cout << "[debug] left name = " << left_joint_name
                      << " id = " << left_joint->GetId() << std::endl;
            std::cout << "[debug] right name = " << right_joint_name
                      << " id = " << right_joint->GetId() << std::endl;
            BTGEN_ASSERT(left_joint != nullptr);
            BTGEN_ASSERT(right_joint != nullptr);
            if (mJointName.find(SWING_STR) != -1)
            {
                // swing joint
                new_base->mLeftStanceIndex = right_joint->GetId();
                new_base->mRightStanceIndex = left_joint->GetId();
            }
            else if (mJointName.find(STANCE_STR) != -1)
            {
                new_base->mRightStanceIndex = right_joint->GetId();
                new_base->mLeftStanceIndex = left_joint->GetId();
            }
            else
            {
                BTGEN_ASSERT(false);
            }
            std::cout << "left stance index = " << new_base->mLeftStanceIndex
                      << " right stance index = " << new_base->mRightStanceIndex
                      << std::endl;
        }

        mBaseTrajs.push_back(new_base);
    }
}

btGenSimbiconTraj::~btGenSimbiconTraj()
{
    for (auto &x : mBaseTrajs)
        delete x;
}
/**
 * \brief           Given the stance situation, get the control joint index
*/
int btGenSimbiconTraj::getJointIndex(int stance)
{
    BTGEN_ASSERT(false);
    return -1;
}

/**
 * \brief           Given contorl info, calculate the target quaterion of this joint
*/
tQuaternion btGenSimbiconTraj::evaluateTrajectory(
    btGenSimbiconControllerBase *ctrl, Joint *joint, int stance, double phi,
    const tVector3d &d, const tVector3d &v) const
{
    std::cout << "begin to evaluate\n";
    exit(0);
}

//======================Simbicon trajectory end=======================
