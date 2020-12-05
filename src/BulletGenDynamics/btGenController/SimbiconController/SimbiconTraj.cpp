#include "SimbiconTraj.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
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
    lastIndex = 0;
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

        mBaseTrajs.push_back(new_base);
    }

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
            mLeftStanceIndex = right_joint->GetId();
            mRightStanceIndex = left_joint->GetId();
        }
        else if (mJointName.find(STANCE_STR) != -1)
        {
            mRightStanceIndex = right_joint->GetId();
            mLeftStanceIndex = left_joint->GetId();
        }
        else
        {
            BTGEN_ASSERT(false);
        }
        std::cout << "left stance index = " << mLeftStanceIndex
                  << " right stance index = " << mRightStanceIndex << std::endl;
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
    if (stance == LEFT_STANCE)
        return mLeftStanceIndex;
    else if (stance == RIGHT_STANCE)
        return mRightStanceIndex;
    else
    {
        BTGEN_ASSERT(false);
        return -1;
    }
}

/**
 * \brief           Given contorl info, calculate the target quaterion of this joint
*/
tQuaternion btGenSimbiconTraj::evaluateTrajectory(
    btGenSimbiconControllerBase *ctrl, Joint *joint, int stance, double phi,
    const tVector3d &d, const tVector3d &v) const
{
    // world orientation target
    std::cout << "-------begin to evaluate joint " << joint->GetName()
              << "---------\n";
    tQuaternion world_tar = tQuaternion(1, 0, 0, 0);
    for (int i = 0; i < mBaseTrajs.size(); i++)
    {
        world_tar =
            mBaseTrajs[i]->evaluateTrajectory(ctrl, joint, stance, phi, d, v) *
            world_tar;
    }
    std::cout << "[debug] for joint " << joint->GetName()
              << " world target = " << world_tar.coeffs().transpose()
              << std::endl;
    return world_tar;
    // convert this to local target
    // {
    //     tMatrix3d joint_world_rot = joint->GetWorldOrientation();
    //     tMatrix3d joint_local_rot = joint->GetRotations();
    //     // world = rest * local
    //     // rest = world * local.T
    //     tMatrix3d rest = joint_world_rot * joint_local_rot.transpose();
    //     tMatrix3d joint_world_rot_tar =
    //         btMathUtil::RotMat(world_tar).block(0, 0, 3, 3);
    //     // world_tar = rest * local_tar;
    //     // local_tar = rest.T * world_tar
    //     tMatrix3d joint_local_rot_tar = rest.transpose() * joint_world_rot_tar;

    //     tQuaternion local_tar = btMathUtil::RotMatToQuaternion(
    //         btMathUtil::ExpandMat(joint_local_rot_tar, 0));
    //     // std::cout << "local target = " << local_tar.coeffs().transpose()
    //     //           << std::endl;
    //     return local_tar;
    // }
}

//======================Simbicon trajectory end=======================
tQuaternion
BaseTrajectory::evaluateTrajectory(btGenSimbiconControllerBase *ctrl,
                                   Joint *joint, int stance, double phi,
                                   const tVector3d &d, const tVector3d &v)
{
    double baseAngle = 0;
    if (mTimeKnots.size() > 0)
    {
        baseAngle += EvaluateCatmullrom(phi);
    }
    std::cout << "catmull angle = " << baseAngle << std::endl;
    double feedback = ComputeFeedback(ctrl, joint, d, v);
    std::cout << "feedback = " << feedback << std::endl;
    return btMathUtil::AxisAngleToQuaternion(
        btMathUtil::Expand(mRotationAxis * (baseAngle + feedback), 0));
}

double BaseTrajectory::EvaluateCatmullrom(double t)
{
    const double TINY = 1e-9;
    int size = this->mTimeKnots.size();
    if (t <= this->mTimeKnots[0])
        return mValueKnots[0];
    if (t >= this->mTimeKnots[size - 1])
        return mValueKnots[size - 1];
    int index = getFirstLargerIndex(t);

    //now that we found the interval, get a value that indicates how far we are along it
    t = (t - this->mTimeKnots[index - 1]) /
        (this->mTimeKnots[index] - this->mTimeKnots[index - 1]);

    //approximate the derivatives at the two ends
    double t0, t1, t2, t3;
    double p0, p1, p2, p3;
    p0 = (index - 2 < 0) ? (mValueKnots[index - 1]) : (mValueKnots[index - 2]);
    p1 = mValueKnots[index - 1];
    p2 = mValueKnots[index];
    p3 = (index + 1 >= size) ? (mValueKnots[index]) : (mValueKnots[index + 1]);

    t0 = (index - 2 < 0) ? (this->mTimeKnots[index - 1])
                         : (this->mTimeKnots[index - 2]);
    t1 = this->mTimeKnots[index - 1];
    t2 = this->mTimeKnots[index];
    t3 = (index + 1 >= size) ? (this->mTimeKnots[index])
                             : (this->mTimeKnots[index + 1]);

    double d1 = (t2 - t0);
    double d2 = (t3 - t1);

    if (d1 > -TINY && d1 < 0)
        d1 = -TINY;
    if (d1 < TINY && d1 >= 0)
        d1 = TINY;
    if (d2 > -TINY && d2 < 0)
        d2 = -TINY;
    if (d2 < TINY && d2 >= 0)
        d2 = TINY;

    double m1 = 0.5 * (p2 - p0);
    double m2 = 0.5 * (p3 - p1);

    t2 = t * t;
    t3 = t2 * t;

    //and now perform the interpolation using the four hermite basis functions from wikipedia
    return p1 * (2 * t3 - 3 * t2 + 1) + m1 * (t3 - 2 * t2 + t) +
           p2 * (-2 * t3 + 3 * t2) + m2 * (t3 - t2);
}
double BaseTrajectory::ComputeFeedback(btGenSimbiconControllerBase *ctrl,
                                       Joint *j, const tVector3d &d,
                                       const tVector3d &v)
{
    if (mBalanceFeedback == nullptr)
        return 0;

    double dToUse = d.dot(mBalanceFeedback->mFeedbackProjAxis);
    double vToUse = v.dot(mBalanceFeedback->mFeedbackProjAxis);
    double vMin = -1000, dMin = -1000, vMax = 1000, dMax = 1000;
    if (dToUse < dMin)
        dToUse = dMin;
    if (vToUse < vMin)
        vToUse = vMin;
    if (dToUse > dMax)
        dToUse = dMax;
    if (vToUse > vMax)
        vToUse = vMax;

    return dToUse * mBalanceFeedback->Cd + vToUse * mBalanceFeedback->Cv;
}

double BaseTrajectory::getFirstLargerIndex(double t)
{
    int size = mTimeKnots.size();
    if (size == 0)
        return 0;
    if (t < mTimeKnots[(lastIndex + size - 1) % size])
        lastIndex = 0;
    for (int i = 0; i < size; i++)
    {
        int index = (i + lastIndex) % size;
        if (t < mTimeKnots[index])
        {
            lastIndex = index;
            return index;
        }
    }
    return size;
}