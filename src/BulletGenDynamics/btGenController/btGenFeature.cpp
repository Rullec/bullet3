#include "btGenFeature.h"
#include "BulletGenDynamics/btGenModel/Link.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "btGenFeatureUtils.h"
#include "btTraj.h"
#include <fstream>
#include <iostream>
#include <map>

std::string feature_energy_path = "feature_energy.txt";
btGenFeatureArray::btGenFeatureArray()
{
    mModel = nullptr;
    mWeight_tau.resize(0);
    mCurTimestep = 0;
    mFeatureArrays.clear();
    // init convert matrix
    {
        mConvertPosToY = tMatrixXd::Zero(1, 3);
        mConvertPosToY(0, 1) = 1;
        mConvertPosToXZ = tMatrixXd::Zero(2, 3);
        mConvertPosToXZ(0, 0) = 1;
        mConvertPosToXZ(1, 2) = 1;
        // std::cout << "convert pos to y = \n"
        // 		  << mConvertPosToY << std::endl;
        // std::cout << "convert pos to xz = \n"
        // 		  << mConvertPosToXZ << std::endl;
        // exit(0);
    }
    std::ofstream fout(feature_energy_path);
    fout << "";
    fout.close();
}

btGenFeatureArray::~btGenFeatureArray()
{
    for (auto &x : mFeatureArrays)
        delete x;
}

/**
 * \brief               Init the feature vector
 *
 *  Given the dyanmic model and guided trajectory, \
 * this function will init the feature vector,
 * validate the loaded config feature size,
 * check the entireness of controllable DOF
 */
void btGenFeatureArray::Init(const std::string &conf,
                             cRobotModelDynamics *model, const tVector &g)
{
    mModel = model;
    mModel->SetComputeSecondDerive(true);
    // mTraj = traj;
    mGravity = g;

    // init mN matrix
    {
        int num_of_freedom = mModel->GetNumOfFreedom();
        int num_of_underactuated_freedom = num_of_freedom - 6;
        mN = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
        mN.block(6, 0, num_of_underactuated_freedom,
                 num_of_underactuated_freedom)
            .setIdentity();
    }

    InitFeature(conf);
    InitWeightTau(conf);
    // InitRefFeature();
}

/**
 * \brief				given config file, create features
 */
void btGenFeatureArray::InitFeature(const std::string &conf)
{
    std::cout << "[log] begin to init feature array from " << conf << std::endl;
    Json::Value root;
    if (false == btJsonUtil::LoadJson(conf, root))
    {
        exit(0);
    }

    // 1. create all features
    Json::Value::Members keys = root.getMemberNames();
    mFeatureArrays.resize(3);
    mFeatureArrays[0] = new tFeatureArraySingleOrder(0);
    mFeatureArrays[1] = new tFeatureArraySingleOrder(1);
    mFeatureArrays[2] = new tFeatureArraySingleOrder(2);

    // std::cout << "num of keys = " << keys.size() << std::endl;
    // std::cout << "before total feature size = " << mTotalFeatureSize <<
    // std::endl;
    for (int i = 0; i < keys.size(); i++)
    {
        // std::cout << "handle key " << i << std::endl;
        std::string feature_name = keys[i];
        std::string suffix = GetFeatureSuffix(feature_name);
        std::string joint_name = GetFeatureLinkName(feature_name);
        if ("weight" == suffix)
            continue;
        std::map<int, double> order_weight_map =
            GetFeatureOrderAndWeight(root[feature_name]);
        // feature->mFeatureOrder = GetFeatureOrder();
        // feature->mWeight = GetFeatureWeight(root[feature_name]);
        for (auto &iter : order_weight_map)
        {
            btGenFeature *feature = nullptr;
            // add the joint accel into the feature vector
            if ("joint" == suffix)
                feature = CreateJointFeature(joint_name);
            else if ("height" == suffix)
                feature = CreateHeightFeature(joint_name);
            else if ("location" == suffix)
                feature = CreateLocationFeature(joint_name);
            else
            {
                std::cout << "[error] unsupported feature suffix = " << suffix
                          << std::endl;
                exit(0);
            }
            feature->mFeatureName =
                feature_name + "_" + std::to_string(iter.first);
            feature->mFeatureOrder = iter.first;
            feature->mWeight = iter.second;
            if (feature->mFeatureOrder < 0 || feature->mFeatureOrder > 2)
            {
                std::cout << "[error] exceed the feature order: "
                          << feature->mFeatureOrder << std::endl;
                exit(1);
            }
            mFeatureArrays[feature->mFeatureOrder]->AddFeature(feature);
        }
    }

    // 2. sort the feature according to link id, because the json file has an
    // random key order then build the feature offset (the begin position of a
    // specieifed feature in a vector)
    for (int i = 0; i < 3; i++)
        mFeatureArrays[i]->Done();

    // std::cout << "total feature size = " << mTotalFeatureSize << std::endl;
    // exit(0);

    // 3. validate the feature array: are all dof controlled by this feature? If
    // not, exit
    PrintFeatureInfo();
    std::cout << "[log] end to init feature array\n";
}

/**
 * \brief					Init the control weight of tau
 * 		Wm \in R{n-6 \times n-6}
 */
void btGenFeatureArray::InitWeightTau(const std::string &conf)
{

    // init Wm
    Json::Value root;
    btJsonUtil::LoadJson(conf, root);
    double tau_weight = btJsonUtil::ParseAsDouble("tau_weight", root);
    int num_of_links = mModel->GetNumOfLinks();
    int num_of_freedom = mModel->GetNumOfFreedom();
    mWeight_tau.resize(num_of_freedom - 6);
    int offset = 0;
    for (int link_id = 1; link_id < num_of_links; link_id++)
    {
        auto joint = mModel->GetLinkById(link_id)->GetParent();
        int dof = joint->GetNumOfFreedom();
        mWeight_tau.segment(offset, dof).fill(tau_weight);
        offset += dof;
    }

    std::cout << "mWeight_tau = " << mWeight_tau.transpose() << std::endl;
    // exit(0);
}

/**
 * \brief						Evaluate in this
 * reference frame, get the feature matrix H, E, f whicah can be used later
 *
 * 1. Given target active force and target qddot, calculate target feature
 * vector:
 *
 * 2. Calculate current dynamic terms:
 * 	H = -E^{-1} * D_\tau^T * W^T * W * M_Q
 * 	E = D_\tau^T * W^T * W * M_\tau + W_m^T * W_m
 *  f = W_m^T * W_m \tilde{\tau} - D_\tau^T * W^T * W * M_Q * (n_\tau -
 * \tilde{\alpha})
 *
 * 3. Prerequisite:
 * 	D_\tau = C_\alpha * M^{-1} * N
 * 	M_Q = C_\alpha * M^{-1}
 *  n_\tau = d_\alpha - C_\alpha * M^{-1} * C_d \dot{q}
 *  C_\alpha = ConvertMat from qddot to feature vector
 * 	d_\alpha = residual in the above
 */
void btGenFeatureArray::Eval(double dt, const tVectorXd &qddot_target,
                             const tVectorXd &qdot_target,
                             const tVectorXd &q_target,
                             const tVectorXd &tau_target, tMatrixXd &H,
                             tMatrixXd &E, tVectorXd &f)
{
    int num_of_underactuated_freedom = mModel->GetNumOfFreedom() - 6;
    if (tau_target.size() != num_of_underactuated_freedom)
    {
        std::cout << "[error] FeatureArray::eval tau_target size "
                  << tau_target.size() << " != " << num_of_underactuated_freedom
                  << std::endl;
        exit(0);
    }
    mCurTimestep = dt;
    mTargetAccel = qddot_target;
    mTargetVel = qdot_target;
    mTargetPos = q_target;
    mTargetTau = tau_target;
    // if (target_frame >= mTraj->mNumOfFrames - 2)
    // {
    // 	std::cout << "[error] void btGenFeatureArray::Eval frame " <<
    // target_frame << " exceeds the range\n"; 	exit(0);
    // }

    // ContactForce & ControlForce -> qddot, qdot, q
    {
        EvalConvertMatFromForceToqddot(
            mFeatureArrays[2]->mConvertMatFromTauToq,
            mFeatureArrays[2]->mConvertMatFromContactForceToq,
            mFeatureArrays[2]->mConvertResFromForceToq);
        EvalConvertMatFromForceToqdot(
            mFeatureArrays[1]->mConvertMatFromTauToq,
            mFeatureArrays[1]->mConvertMatFromContactForceToq,
            mFeatureArrays[1]->mConvertResFromForceToq);
        EvalConvertMatFromForceToq(
            mFeatureArrays[0]->mConvertMatFromTauToq,
            mFeatureArrays[0]->mConvertMatFromContactForceToq,
            mFeatureArrays[0]->mConvertResFromForceToq);
    }
    {
        EvalConvertMatAndResidualFromqddotToFeature(
            mFeatureArrays[2]->mConvertMatFromqToFeature,
            mFeatureArrays[2]->mConvertResFromqToFeature);
        EvalConvertMatAndResidualFromqdotToFeature(
            mFeatureArrays[1]->mConvertMatFromqToFeature,
            mFeatureArrays[1]->mConvertResFromqToFeature);
        EvalConvertMatAndResidualFromqToFeature(
            mFeatureArrays[0]->mConvertMatFromqToFeature,
            mFeatureArrays[0]->mConvertResFromqToFeature);
        // std::cout << "From q to feature Mat = \n"
        //           << mFeatureArrays[0]->mConvertMatFromqToFeature << std::endl;
        // std::cout << "From q to feature Res = "
        //           << mFeatureArrays[0]->mConvertResFromqToFeature.transpose()
        //           << std::endl;
    }
    // std::cout << "calculate convert mat done, begin to debug\n";
    // EvalConvertMatFromForceToqdot(mConvertFromTauToVel,
    //                               mConvertFromContactForceToVel, mResToVel);
    // EvalConvertMatFromForceToq(mConvertFromTauToPos,
    //                            mConvertFromContactForceToPos, mResToPos);

    // qddot, qdot, q -> accel & vel & pos feature

    mFeatureArrays[2]->mRefFeature = CalcTargetAccelFeature(qddot_target);
    mFeatureArrays[1]->mRefFeature = CalcTargetVelFeature(qdot_target);
    mFeatureArrays[0]->mRefFeature = CalcTargetPosFeature(q_target);

    // std::cout << "ref q = " << q_target.transpose() << std::endl;
    // std::cout << "ref feature = " << mFeatureArrays[0]->mRefFeature.transpose()
    //           << std::endl;
    EvalDynamicTerms(mFeatureArrays[2]->mRefFeature,
                     mFeatureArrays[1]->mRefFeature,
                     mFeatureArrays[0]->mRefFeature, tau_target, H, E, f);

    // std::cout << "ref feature vector " <<
    // mRefAccelFeature[target_frame].transpose() << std::endl; std::cout << "mH
    // = \n"
    // 		  << H << std::endl;
    // exit(0);
}

// int btGenFeatureArray::GetSingleFeatureOffset(int feature_id) const
// {
//     return mFeatureOffset[feature_id];
// }

/**
 * \brief					Create joint features
 */
btGenFeature *
btGenFeatureArray::CreateJointFeature(const std::string &joint_name) const
{
    auto link = mModel->GetLink(joint_name);
    if (link == nullptr)
    {
        std::cout << "[error] CreateJointFeature: Get link " << joint_name
                  << " fail\n";
        exit(1);
    }
    BaseObject *joint = link->GetParent();
    if (joint == nullptr)
    {
        std::cout << "[error] target joint " << joint_name << " not found\n";
        exit(0);
    }

    btGenFeature *feature = new btGenFeature();
    feature->mLinkId = joint->GetId();
    feature->mLinkName = joint_name;
    switch (joint->GetJointType())
    {
    case JointType::NONE_JOINT:
        feature->mFeatureType = btGenFeatureType::NoneJoint;
        break;
    case JointType::SPHERICAL_JOINT:
        feature->mFeatureType = btGenFeatureType::SphereJoint;
        break;
    case JointType::REVOLUTE_JOINT:
        feature->mFeatureType = btGenFeatureType::RevoluteJoint;
        break;
    case JointType::FIXED_JOINT:
        feature->mFeatureType = btGenFeatureType::FixedJoint;
        break;
    default:
        std::cout << "[error] unsupported joint type " << joint->GetJointType()
                  << std::endl;
        exit(0);
        break;
    }
    // std::cout << "pu"sh " << joint_name << " type " << joint->GetJointType()
    // << " id " << feature->mLinkId << std::endl;

    return feature;
}

btGenFeature *
btGenFeatureArray::CreateHeightFeature(const std::string &link_name) const
{
    BaseObject *link = mModel->GetLink(link_name);
    btGenFeature *feature = new btGenFeature();
    feature->mLinkId = link->GetId();
    feature->mLinkName = link->GetName();
    feature->mFeatureType = btGenFeatureType::Height;

    return feature;
}
btGenFeature *
btGenFeatureArray::CreateLocationFeature(const std::string &link_name) const
{
    BaseObject *link = mModel->GetLink(link_name);
    btGenFeature *feature = new btGenFeature();
    feature->mLinkId = link->GetId();
    feature->mLinkName = link->GetName();
    feature->mFeatureType = btGenFeatureType::Location;

    return feature;
}

void btGenFeatureArray::PrintFeatureInfo() const
{
    std::cout << "-----------------PrintFeatureInfo begin-----------------\n";
    int num_of_links = mModel->GetNumOfLinks();
    for (int i = 0; i < 3; i++)
    {
        auto cur_feature = mFeatureArrays[i];
        std::string prefix = "";
        switch (i)
        {
        case 0:
            prefix = "[pos] ";
            break;
        case 1:
            prefix = "[vel] ";
            break;
        case 2:
            prefix = "[accel] ";
            break;
        default:
            std::cout << "Unsupported feature vector order\n";
            exit(1);
            break;
        }
        std::vector<bool> dof_controllable_lst(num_of_links, false);
        for (const auto &x : cur_feature->mFeatureVector)
        {
            // std::cout << "feature " << x->mLinkName << " type " <<
            // gGenFeatureTypeStr[x->mFeatureType] << " size " <<
            // GetSingleFeatureSize(x) << " order " << x->mFeatureOrder
            // 		  << " weight " << x->mWeight << std::endl;
            if (x->mFeatureType >= btGenFeatureType::SphereJoint &&
                x->mFeatureType <= btGenFeatureType::FixedJoint)
            {
                dof_controllable_lst[x->mLinkId] = true;
            }
            else
            {
                switch (x->mFeatureType)
                {
                case btGenFeatureType::Height:
                    std::cout
                        << prefix << "link " << x->mLinkName
                        << " height is controlled, weight = " << x->mWeight
                        << std::endl;
                    break;
                case btGenFeatureType::Location:
                    std::cout
                        << prefix << "link " << x->mLinkName
                        << " location is controlled, weight = " << x->mWeight
                        << std::endl;
                    break;
                default:
                    continue;
                    break;
                }
            }
        }

        // check whether all dof are controlled?
        // if (cur_feature->mFeatureVector.size() != 0)
        // {
        //     for (int link_id = 0; link_id < num_of_links; link_id++)
        //     {
        //         if (false == dof_controllable_lst[link_id])
        //         {
        //             std::cout << prefix << "link " << link_id << " "
        //                       << mModel->GetLinkById(link_id)->GetName()
        //                       << " is not controlled\n";
        //             exit(0);
        //         }
        //     }
        //     std::cout << prefix << "all dof are controlled\n";
        // }
        std::cout << prefix
                  << "total feature size = " << cur_feature->mTotalFeatureSize
                  << std::endl;
    }
    std::cout << "model dof = " << mModel->GetNumOfFreedom() << std::endl;
    std::cout << "-----------------PrintFeatureInfo end-----------------\n";
}
#include "BulletGenDynamics/btGenModel/Link.h"
void btGenFeatureArray::TestdJvdq(tVectorXd &q, tVectorXd &qdot,
                                  tVectorXd &qddot, int link_id)
{
    // test the numerical derivation of Jv w.r.t to each link
    // tVectorXd q = mTraj->mq[frame_id],
    // 		  qdot = mTraj->mqdot[frame_id],
    // 		  qddot = mTraj->mqddot[frame_id];
    mModel->SetComputeSecondDerive(true);
    mModel->Apply(q, true);
    auto link = dynamic_cast<Link *>(mModel->GetLinkById(link_id));
    tMatrixXd Jv_mid = link->GetJKv();
    int num_of_freedom = mModel->GetNumOfFreedom();
    tEigenArr<tMatrixXd> dJv_dq_ana(num_of_freedom);
    for (int dof = 0; dof < num_of_freedom; dof++)
    {
        dJv_dq_ana[dof] = link->GetTotalDofdJKv_dq(dof);
        // std::cout << "now = " << dJv_dq_ana[dof] << std::endl;
        // std::cout << "now = \n"
        // 		  << dJv_dq_ana[dof] << std::endl;
    }

    // 2. change q,
    double eps = 1e-5;
    mModel->GetLinkById(link_id);
    for (int dof = 0; dof < num_of_freedom; dof++)
    {
        q[dof] += eps;
        mModel->Apply(q, true);
        tMatrixXd Jv_new = link->GetJKv();
        tMatrixXd derive = (Jv_new - Jv_mid) / eps;
        q[dof] -= eps;
        tMatrixXd diff = derive - dJv_dq_ana[dof];
        double diff_norm = diff.norm();

        if (diff_norm > eps)
        {
            std::cout << "[error] link " << link_id << " dof " << dof
                      << " diff norm = " << diff_norm << std::endl;
            exit(0);
        }
    }
    std::cout << "link " << link_id << " tested succ\n";
    // exit(0);
}

/**
 * \brief				Calculate the formula:
 *
 * 		Accel feature Vector = ConvertMatFromqddotToFeature * qddot +
 * ResFromqddotToFeature
 *
 * 	accel feature vector is consist of generalized accel or cartesian accel
 * of any link COM.
 */
void btGenFeatureArray::EvalConvertMatAndResidualFromqddotToFeature(
    tMatrixXd &ConvertMatFromqddotToFeature,
    tVectorXd &ResFromqddotToFeature) const
{
    auto accel_feature = mFeatureArrays[2];
    int num_of_freedom = mModel->GetNumOfFreedom();
    ConvertMatFromqddotToFeature.noalias() =
        tMatrixXd::Zero(accel_feature->mTotalFeatureSize, num_of_freedom);
    ResFromqddotToFeature = tVectorXd::Zero(accel_feature->mTotalFeatureSize);
    for (int id = 0; id < accel_feature->mNumOfFeature; id++)
    {
        auto feature = accel_feature->mFeatureVector[id];
        int feature_offset = accel_feature->mFeatureOffset[id];
        int feature_size = GetSingleFeatureSize(feature);
        int link_id = feature->mLinkId;
        auto link = mModel->GetLinkById(link_id);
        auto joint = mModel->GetJointById(link_id);
        // for each feature type, we have different convert formulae
        switch (feature->mFeatureType)
        {
        // these joints' type features behavior the same
        // their feature vector = gen accel
        case btGenFeatureType::FixedJoint:
        case btGenFeatureType::NoneJoint:
        case btGenFeatureType::SphereJoint:
        case btGenFeatureType::RevoluteJoint:
        {
            int q_size = joint->GetNumOfFreedom();
            int q_offset = joint->GetFreedoms(0)->id;
            for (int i = 0; i < q_size; i++)
            {
                ConvertMatFromqddotToFeature(feature_offset + i, q_offset + i) =
                    1.0;
                // ResFromqddotToFeature[feature_offset + i] =
                //     accel_feature->mConvertResFromForceToq[q_offset + i];
                ResFromqddotToFeature[feature_offset + i] = 0;
            }
            // ResFromqddotToFeature.segment(feature_offset, feature_size)
        }
        break;

        case btGenFeatureType::Location:
        {
            const tMatrixXd Jv = link->GetJKv();
            const tVectorXd qdot = mModel->Getqdot();
            const tMatrixXd Jv_dot = link->GetJKv_dot();
            ConvertMatFromqddotToFeature.block(feature_offset, 0, feature_size,
                                               num_of_freedom) =
                mConvertPosToXZ * Jv;
            ResFromqddotToFeature.segment(feature_offset, feature_size) =
                mConvertPosToXZ * Jv_dot * qdot;
        }
        break;
        case btGenFeatureType::Height:
        {
            const tMatrixXd Jv = link->GetJKv();
            const tVectorXd qdot = mModel->Getqdot();
            const tMatrixXd Jv_dot = link->GetJKv_dot();
            // std::cout << 'mConvertPosToY = \n' << mConvertPosToY <<
            // std::endl;
            ConvertMatFromqddotToFeature.block(feature_offset, 0, feature_size,
                                               num_of_freedom) =
                mConvertPosToY * Jv;
            ResFromqddotToFeature.segment(feature_offset, feature_size) =
                mConvertPosToY * Jv_dot * qdot;
        }
        break;
        default:
            std::cout << "[error] Unsupported feature type "
                      << feature->mFeatureType,
                exit(0);
            break;
        }
    }
}

/**
 * \brief				Calculate the formula:
 *
 * 		Vccel feature Vector = ConvertMatFromqdotToFeature * qdot_next +
 * ResFromqdotToFeature
 *
 * 	vel feature vector is consist of generalized vel or cartesian vel
 * of any link COM
 */
void btGenFeatureArray::EvalConvertMatAndResidualFromqdotToFeature(
    tMatrixXd &ConvertMatFromqdotToFeature,
    tVectorXd &ResFromqdotToFeature) const
{
    // 1. for joint feature, the convert mat is selection matrix, the residual
    // is zero
    auto &vel_feature = mFeatureArrays[1];
    int num_of_freedom = mModel->GetNumOfFreedom();
    ConvertMatFromqdotToFeature.noalias() =
        tMatrixXd::Zero(vel_feature->mTotalFeatureSize, num_of_freedom);
    ResFromqdotToFeature.noalias() =
        tVectorXd::Zero(vel_feature->mTotalFeatureSize);

    // 2. for location & height feature, the convert mat is dt * Jv, the
    // residual is cur_link_vel
    for (int id = 0; id < vel_feature->mNumOfFeature; id++)
    {
        auto feature = vel_feature->mFeatureVector[id];
        int feature_offset = vel_feature->mFeatureOffset[id];
        int feature_size = GetSingleFeatureSize(feature);

        int link_id = feature->mLinkId;
        auto link = mModel->GetLinkById(link_id);
        auto joint = mModel->GetJointById(link_id);

        switch (feature->mFeatureType)
        {
        case btGenFeatureType::FixedJoint:
        case btGenFeatureType::NoneJoint:
        case btGenFeatureType::SphereJoint:
        case btGenFeatureType::RevoluteJoint:
        {
            int q_size = joint->GetNumOfFreedom();
            int q_offset = joint->GetFreedoms(0)->id;
            for (int i = 0; i < q_size; i++)
            {
                ConvertMatFromqdotToFeature(feature_offset + i, q_offset + i) =
                    1.0;
            }
            break;
        }

        case btGenFeatureType::Location:
        {
            /*
                    v = Jv * qdot
                so the convert mat is jv, and the residual is zero
            */
            const tMatrixXd &Jv = link->GetJKv();
            ConvertMatFromqdotToFeature.block(feature_offset, 0, feature_size,
                                              num_of_freedom) =
                mConvertPosToXZ * Jv;

            ResFromqdotToFeature.segment(feature_offset, feature_size)
                .setZero();
            break;
        }

        case btGenFeatureType::Height:
        {
            const tMatrixXd &Jv = link->GetJKv();
            ConvertMatFromqdotToFeature.block(feature_offset, 0, feature_size,
                                              num_of_freedom) =
                mConvertPosToY * Jv;

            ResFromqdotToFeature.segment(feature_offset, feature_size)
                .setZero();
            break;
        }
        default:
            std::cout << "[error] Unsupported feature type "
                      << feature->mFeatureType << std::endl;
            exit(0);
            break;
        }
    }
    // 3. finally, write all these sub-mat down to the ConvertMat
}

/**
 * \brief				Calculate the formula:
 *
 * 		Pos feature Vector = ConvertMatFromqToFeature * q_next +
 * ResFromqToFeature
 *
 * 	Pos feature vector is consist of generalized pos or cartesian pos
 * of any link COM
 */
void btGenFeatureArray::EvalConvertMatAndResidualFromqToFeature(
    tMatrixXd &ConvertMatFromqToFeature, tVectorXd &ResFromqToFeature) const
{
    // 1. for joint feature, the convert mat is selection matrix, the residual
    // is zero
    auto &pos_feature = mFeatureArrays[0];
    int num_of_freedom = mModel->GetNumOfFreedom();
    ConvertMatFromqToFeature.noalias() =
        tMatrixXd::Zero(pos_feature->mTotalFeatureSize, num_of_freedom);
    ResFromqToFeature = tVectorXd::Zero(pos_feature->mTotalFeatureSize);

    // 2. for location & height feature, the convert mat is dt^2 * Jv, the
    // residual is selection * cur_link_pos
    // 3. finally, write all these sub-mat down to the ConvertMat
    for (int id = 0; id < pos_feature->mNumOfFeature; id++)
    {
        auto feature = pos_feature->mFeatureVector[id];
        int feature_offset = pos_feature->mFeatureOffset[id];
        int feature_size = GetSingleFeatureSize(feature);
        int link_id = feature->mLinkId;
        auto link = mModel->GetLinkById(link_id);
        auto joint = mModel->GetJointById(link_id);
        // for each feature type, we have different convert formulae
        switch (feature->mFeatureType)
        {
        case btGenFeatureType::FixedJoint:
        case btGenFeatureType::NoneJoint:
        case btGenFeatureType::SphereJoint:
        case btGenFeatureType::RevoluteJoint:
        {
            int q_size = joint->GetNumOfFreedom();
            int q_offset = joint->GetFreedoms(0)->id;
            for (int i = 0; i < q_size; i++)
            {
                ConvertMatFromqToFeature(feature_offset + i, q_offset + i) =
                    1.0;
            }
            break;
        }

        case btGenFeatureType::Location:
        {
            const tMatrixXd &Jv = link->GetJKv();
            const tVector3d &link_pos = link->GetWorldPos();
            ConvertMatFromqToFeature.block(feature_offset, 0, feature_size,
                                           num_of_freedom) =
                mConvertPosToXZ * Jv;
            ResFromqToFeature.segment(feature_offset, feature_size) =
                mConvertPosToXZ * (link_pos - Jv * mModel->Getq());

            break;
        }

        case btGenFeatureType::Height:
        {
            const tMatrixXd &Jv = link->GetJKv();
            const tVector3d &link_pos = link->GetWorldPos();
            ConvertMatFromqToFeature.block(feature_offset, 0, feature_size,
                                           num_of_freedom) =
                mConvertPosToY * Jv;
            ResFromqToFeature.segment(feature_offset, feature_size) =
                mConvertPosToY * (link_pos - Jv * mModel->Getq());
            break;
        }
        default:
            std::cout << "[error] Unsupported feature type "
                      << feature->mFeatureType << std::endl;
            exit(0);
            break;
        }
    }
}

/**
 * \brief					Calculate the convert
 * mat/convert residual from the control torque \tau to qddot
 *
 * 		ConvertMatFromTauToqddot = Minv * N
 * 		ConvertMatFromContactToqddot = Minv
 * 		ResToqddot = Minv * (Q_G - C_d * qdot)
 *
 */
void btGenFeatureArray::EvalConvertMatFromForceToqddot(
    tMatrixXd &ConvertMatFromTauToqddot,
    tMatrixXd &ConvertMatFromContactToqddot, tVectorXd &ResToqddot) const
{
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    ConvertMatFromTauToqddot.noalias() = Minv * mN;
    ConvertMatFromContactToqddot.noalias() = Minv;

    tVectorXd Q_G = mModel->CalcGenGravity(mGravity);
    ResToqddot.noalias() =
        Minv * (Q_G - mModel->GetCoriolisMatrix() * mModel->Getqdot());
}

/**
 * \brief                   calculate the convert mat from tau/contact force to
 * qdot
 *
 * 		ConvertMatFromTauToqdot = dt * Minv * N
 * 		ConvertMatFromContactToqdot = dt * Minv
 * 		ResToqdot = dt * Minv * (Q_G - C_d * qdot) + \dot{q}_t
 */
void btGenFeatureArray::EvalConvertMatFromForceToqdot(
    tMatrixXd &ConvertMatFromTauToqdot, tMatrixXd &ConvertMatFromContactToqdot,
    tVectorXd &ResToqdot) const
{
    const double dt = this->mCurTimestep;
    const double dt2 = dt * dt;

    const tMatrixXd Minv = mModel->GetInvMassMatrix();
    ConvertMatFromTauToqdot.noalias() = dt * Minv * mN;
    ConvertMatFromContactToqdot.noalias() = dt * Minv;

    tVectorXd Q_G = mModel->CalcGenGravity(mGravity);
    ResToqdot.noalias() =
        dt * Minv * (Q_G - mModel->GetCoriolisMatrix() * mModel->Getqdot()) +
        mModel->Getqdot();
}

/**
 * \brief                   calculate the convert mat from tau/contact force to
 * q
 *
 * 		ConvertMatFromTauToq = dt^2 * Minv * N
 * 		ConvertMatFromContactToq = dt^2 * Minv
 * 		ResToq = dt^2 * Minv * (Q_G - C_d * qdot) + dt * \dot{q}_t + qt
 */
void btGenFeatureArray::EvalConvertMatFromForceToq(
    tMatrixXd &ConvertMatFromTauToq, tMatrixXd &ConvertMatFromContactToq,
    tVectorXd &ResToq) const
{
    const double dt = this->mCurTimestep;
    const double dt2 = dt * dt;
    const tMatrixXd Minv = mModel->GetInvMassMatrix();
    ConvertMatFromTauToq.noalias() = dt2 * Minv * mN;
    ConvertMatFromContactToq.noalias() = dt2 * Minv;

    tVectorXd Q_G = mModel->CalcGenGravity(mGravity);
    ResToq.noalias() =
        mModel->Getq() + dt * mModel->Getqdot() +
        dt2 * Minv * (Q_G - mModel->GetCoriolisMatrix() * mModel->Getqdot());
}

/**
 * \brief						The core functionality
 * of GenFeature, is to calculate three control matrix H, E, f and then offer
 * them to the outsider control-adviser. They are calculated in this function.
 *
 * For more details, please check the note.
 *
 * 1. The relationship between control \tau, contact force Q and the feature
 * vector
 *
 *          For pos feature vector \phi:
 *               \phi = D_\phi * \tau + M_\phi * Q + n_\phi
 *          For vel feature vector \nu:
 *               \nu = D_\nu * \tau + M_\nu * Q + n_\nu
 *          For accel feature vector
 *               \alpha = D_\alpha * \tau + M_\alpha * Q + n_\alpha
 *
 *      D_x = ConvertMatFromqxToFeature * ConvertMatFromTauToqx
 *      M_x = ConvertMatFromqxToFeature * ConvertMatFromContactForceToqx
 *      n_x = ConvertMatFromqxToFeature * ConvertResFromForceToqx
 *
 * 2. The formulaes of H, E, f. W_1, W_2, W_3 are the weight vector for accel,
 * vel and pos feature respectively
 *
 *      E = (D_\alpha^T * W_1^T * W_1 * D_\alpha + D_\nu^T * W_2^T * W_2 * D_\nu
 *
 *          + D_\phi^T * W_3^T * W_3 * D_\phi + W_tau^T * W_tau)
 *
 *      F = D_\alpha^T * W_1^T * W_1 * M_\alpha + D_\nu^T * W_2^T * W_2 * M_\nu
 *
 *          + D_\phi^T * W_3^T * W_3 * M_\phi
 *
 *      f =  W_tau^T * W_tau * \tilde{\tau}
 *
 *          - D_\alpha^T * W_1^T * W_1 * (n_\alpha - \tilde{\alpha})
 *
 *          - D_\nu^T * W_2^T * W_2 * (n_\nu - \tilde{\nu})
 *
 *          - D_\phi^T * W_3^T * W_3 * (n_\phi - \tilde{\phi})
 *
 *      H = E^{-1} * F
 */

tMatrixXd H_debug, E_debug, F_debug;
tVectorXd f_debug;
void btGenFeatureArray::EvalDynamicTerms(const tVectorXd &ref_accel_feature,
                                         const tVectorXd &ref_vel_feature,
                                         const tVectorXd &ref_pos_feature,
                                         const tVectorXd &ref_tau, tMatrixXd &H,
                                         tMatrixXd &E, tVectorXd &f) const
{
    {
        int num_of_underactuated_freedom = mModel->GetNumOfFreedom() - 6;
        if (ref_tau.size() != num_of_underactuated_freedom)
        {
            std::cout << "[error] ref tau size " << ref_tau.size()
                      << " != " << num_of_underactuated_freedom << std::endl;
            exit(0);
        }
    }

    // 1. calculate D Matrix and M matrix and n
    tMatrixXd D_alpha, M_alpha, D_nu, M_nu, D_phi, M_phi;
    tVectorXd n_alpha, n_nu, n_phi;
    {
        auto alpha_feature = mFeatureArrays[2];
        D_alpha = alpha_feature->mConvertMatFromqToFeature *
                  alpha_feature->mConvertMatFromTauToq;
        M_alpha = alpha_feature->mConvertMatFromqToFeature *
                  alpha_feature->mConvertMatFromContactForceToq;
        n_alpha = alpha_feature->mConvertMatFromqToFeature *
                      alpha_feature->mConvertResFromForceToq +
                  alpha_feature->mConvertResFromqToFeature;
        // std::cout << "D alpha = \n" << D_alpha << std::endl;
        // std::cout << "M alpha = \n" << M_alpha << std::endl;
        // std::cout << "n alpha = " << n_alpha << std::endl;
    }

    {
        auto nu_feature = mFeatureArrays[1];
        D_nu = nu_feature->mConvertMatFromqToFeature *
               nu_feature->mConvertMatFromTauToq;
        M_nu = nu_feature->mConvertMatFromqToFeature *
               nu_feature->mConvertMatFromContactForceToq;
        n_nu = nu_feature->mConvertMatFromqToFeature *
                   nu_feature->mConvertResFromForceToq +
               nu_feature->mConvertResFromqToFeature;

        // std::cout << "D nu = \n" << D_nu << std::endl;
        // std::cout << "M nu = \n" << M_nu << std::endl;
        // std::cout << "n nu = " << n_nu << std::endl;
    }
    {
        auto phi_feature = mFeatureArrays[0];
        D_phi = phi_feature->mConvertMatFromqToFeature *
                phi_feature->mConvertMatFromTauToq;
        M_phi = phi_feature->mConvertMatFromqToFeature *
                phi_feature->mConvertMatFromContactForceToq;
        n_phi = phi_feature->mConvertMatFromqToFeature *
                    phi_feature->mConvertResFromForceToq +
                phi_feature->mConvertResFromqToFeature;

        // std::cout << "D phi = \n" << D_phi << std::endl;
        // std::cout << "M phi = \n" << M_phi << std::endl;
        // std::cout << "n phi = " << n_phi << std::endl;
    }

    // 2. calculate the E, F, f, H
    {
        auto &pos_feature = mFeatureArrays[0], &vel_feature = mFeatureArrays[1],
             &accel_feature = mFeatureArrays[2];

        // tMatrixXd tmp = pos_feature->mWeight.asDiagonal();
        // std::cout << "pos feature weight = " << pos_feature->mWeight
        //           << std::endl;
        // std::cout << "pos feature weight mat = " << tmp << std::endl;

        tMatrixXd W1TW1 = (accel_feature->mWeight.cwiseProduct(
                               accel_feature->mWeight))
                              .asDiagonal(),
                  W2TW2 =
                      (vel_feature->mWeight.cwiseProduct(vel_feature->mWeight))
                          .asDiagonal(),
                  W3TW3 =
                      (pos_feature->mWeight.cwiseProduct(pos_feature->mWeight))
                          .asDiagonal(),
                  WtauTWtau =
                      (mWeight_tau.cwiseProduct(mWeight_tau)).asDiagonal();

        // std::cout << "W1TW1 = \n " << W1TW1 << std::endl;
        // std::cout << "W1 = \n " << pos_feature->mWeight.transpose()
        //           << std::endl;
        // std::cout << "W2TW2 = \n " << W2TW2 << std::endl;
        // std::cout << "W3TW3 = \n " << W3TW3 << std::endl;
        // if (pos_feature->mWeight.size() == 0)
        //     W1TW1.resize(0, 0);
        // if (vel_feature->mWeight.size() == 0)
        //     W2TW2.resize(0, 0);
        // if (accel_feature->mWeight.size() == 0)
        //     W3TW3.resize(0, 0);

        // std::cout << "W1TW1 = \n " << W1TW1 << std::endl;
        // std::cout << "W1 = \n " << pos_feature->mWeight.transpose()
        //           << std::endl;
        // std::cout << "W2TW2 = \n " << W2TW2 << std::endl;
        // std::cout << "W3TW3 = \n " << W3TW3 << std::endl;
        // exit(0);
        // std::cout << "begin to calc E \n";
        // std::cout << "D alpha = \n" << D_alpha << std::endl;
        // std::cout << "W1TW1 = \n" << W1TW1 << std::endl;
        // {
        //     tMatrixXd E1 = D_alpha.transpose() * W1TW1 * D_alpha;
        //     // std::cout << "E1 = \n" << E1 << std::endl;
        //     tMatrixXd E2 = D_nu.transpose() * W2TW2 * D_nu;
        //     // std::cout << "E2 = \n" << E2 << std::endl;
        //     tMatrixXd E3 = D_phi.transpose() * W3TW3 * D_phi;
        //     // std::cout << "E3 = \n" << E3 << std::endl;
        //     E.noalias() = WtauTWtau;
        //     if (E1.size() > 0)
        //         E += E1;
        //     if (E2.size() > 0)
        //         E += E2;
        //     if (E3.size() > 0)
        //         E += E3;
        // }
        {
            E.noalias() = WtauTWtau

                          + D_alpha.transpose() * W1TW1 * D_alpha +
                          D_nu.transpose() * W2TW2 * D_nu +
                          D_phi.transpose() * W3TW3 * D_phi;
            // std::cout << "[debug] E = " << E.norm() << std::endl;
            // std::cout << "[debug] D_nu = " << D_nu.norm() << std::endl;
            // std::cout << "[debug] D_nu.T * W2TW2 = "
            //           << (D_nu.transpose() * W2TW2).norm() << std::endl;
            // std::cout << "[debug] W2TW2 = " << W2TW2.norm() << std::endl;
            // std::cout << "E pos part = "
            //           << (D_phi.transpose() * W3TW3 * D_phi).norm()
            //           << std::endl;
        }
        // std::cout << "E = \n" << E << std::endl;
        // exit(1);
        tMatrixXd F = D_alpha.transpose() * W1TW1 * M_alpha +
                      D_nu.transpose() * W2TW2 * M_nu +
                      D_phi.transpose() * W3TW3 * M_phi;
        // std::cout << "F = " << F << std::endl;
        /*
        
                    const tVectorXd f_local = N.transpose() * Minv.transpose() *
                                      alpha_weight.transpose() * alpha_weight *
                                      (Cqdot - QG + mTargetAccel);
                                      
                                      */
        f = WtauTWtau * ref_tau -
            D_alpha.transpose() * W1TW1 * (n_alpha - ref_accel_feature) -
            D_nu.transpose() * W2TW2 * (n_nu - ref_vel_feature) -
            D_phi.transpose() * W3TW3 * (n_phi - ref_pos_feature);
        // std::cout << "f = " << f.transpose() << std::endl;
        H = -E.inverse() * F;
        // std::cout << "H = " << H << std::endl;
        // exit(0);

        // save the debug vars
        {
            E_debug = E;
            F_debug = F;
            f_debug = f;
            H_debug = H;
        }
    }

    // std::cout << "finished\b";
    // exit(0);
}
/**
 * \brief               Given the target qdot, calculate the vel target feature
 * used in the compaute of matrix H, E and vector f
 */
tVectorXd CalcTargetVelFeature(const tVectorXd &qdot)
{
    std::cout << "calc target vel feawture hasn't been finished yet\n";
    exit(0);
    return tVectorXd::Zero(0);
}

/**
 * \brief               Given the target q, calculate the pos target feature,
 * used in the compuatetaion of matrix H, E and vector f
 */
tVectorXd CalcTargetPosFeature(const tVectorXd &q)
{

    std::cout << "calc target pose feawture hasn't been finished yet\n";
    exit(0);
    return tVectorXd::Zero(0);
}
/**
 * \brief					Given the gen accel qddot,
 * calculate current target feature vector
 */
tVectorXd btGenFeatureArray::CalcTargetAccelFeature(const tVectorXd &qddot)
{
    // std::cout << "init ref feature begin " << mTraj->mNumOfFrames <<
    // std::endl; whatever feature type is defined, we only control from 1 - N-2
    // mRefAccelFeature.resize(mTraj->mNumOfFrames);
    auto &accel_feature = mFeatureArrays[2];
    tVectorXd ref_feature;
    ref_feature = tVectorXd::Zero(accel_feature->mTotalFeatureSize);
    const tVectorXd &qdot = mModel->Getqdot();
    for (int fea_id = 0; fea_id < accel_feature->mNumOfFeature; fea_id++)
    {
        auto feature = accel_feature->mFeatureVector[fea_id];
        int offset = accel_feature->mFeatureOffset[fea_id];
        int size = GetSingleFeatureSize(feature);

        // only control the accel feature at this moment
        if (feature->mFeatureOrder != 2)
        {
            std::cout << "[error] CalcTargetAccelFeature feature " << fea_id
                      << "is not 2-order(accel), exit\n";
            exit(0);
        }

        // for joint feature, the feature vector is simply genearlized
        // coordinate
        if (IsJointFeature(feature->mFeatureType))
        {
            auto jont = mModel->GetJointById(feature->mLinkId);
            int q_size = jont->GetNumOfFreedom();
            int q_st = jont->GetFreedoms(0)->id;

            ref_feature.segment(offset, size) = qddot.segment(q_st, q_size);
        }
        else if (IsPosFeature(feature->mFeatureType))
        {
            // for pos feature, the feture vector is d^2(p)/dt^2
            auto link =
                dynamic_cast<Link *>(mModel->GetLinkById(feature->mLinkId));
            tVector3d accel =
                link->GetJKv() * qddot + link->GetJKv_dot() * qdot;

            switch (feature->mFeatureType)
            {
            case btGenFeatureType::Height:
                ref_feature.segment(offset, size) = mConvertPosToY * accel;
                break;

            case btGenFeatureType::Location:
                ref_feature.segment(offset, size) = mConvertPosToXZ * accel;
                break;

            default:
                std::cout << "[error] illegal feature type "
                          << feature->mFeatureType << std::endl,
                    exit(0);
                break;
            }
        }
        else
        {
            std::cout << "[error] Neither joint nor pos feature, illegal "
                         "feature type "
                      << feature->mFeatureType << std::endl,
                exit(0);
        }
    }

    return ref_feature;
}

/**
 * \brief           Given the goal qdot in next frame ,calculate the target
 * velocity feature, \tilde{\nu}
 *
 * the definition of \tilde{\nu} depends on the definition of
 * "VelFeatureVectorArray".
 *
 * 1. For joint angle feature, the target velocity = \dot{q}_i
 * 2. For height feature, the target velocity feature = ConvertY * Jv_t^i *
 * qdot_next
 * 3. For location feature, the target pos feature = ConvertXZ * Jv_t^i *
 * qdot_next
 */
tVectorXd btGenFeatureArray::CalcTargetVelFeature(const tVectorXd &qdot)
{
    auto &vel_feature = mFeatureArrays[1];
    tVectorXd ref_feature = tVectorXd::Zero(vel_feature->mTotalFeatureSize);
    // std::cout << "vel feature size " << vel_feature->mTotalFeatureSize
    //           << std::endl;
    for (int fea_id = 0; fea_id < vel_feature->mNumOfFeature; fea_id++)
    {
        auto feature = vel_feature->mFeatureVector[fea_id];
        int offset = vel_feature->mFeatureOffset[fea_id];
        int size = GetSingleFeatureSize(feature);

        // only control the accel feature at this moment
        if (feature->mFeatureOrder != 1)
        {
            std::cout << "[error] CalcTargetVelFeature feature " << fea_id
                      << " is not 1-order(vel), exit\n";
            exit(0);
        }

        // for joint feature, the feature vector is simply genearlized
        // coordinate
        if (IsJointFeature(feature->mFeatureType))
        {
            auto jont = mModel->GetJointById(feature->mLinkId);
            int q_size = jont->GetNumOfFreedom();
            int q_st = jont->GetFreedoms(0)->id;

            ref_feature.segment(offset, size) = qdot.segment(q_st, q_size);
        }
        else if (IsPosFeature(feature->mFeatureType))
        {
            // for pos feature, the feture vector is d^2(p)/dt^2
            auto link =
                dynamic_cast<Link *>(mModel->GetLinkById(feature->mLinkId));
            tVector3d vel_next = link->GetJKv() * qdot;

            switch (feature->mFeatureType)
            {
            case btGenFeatureType::Height:
                ref_feature.segment(offset, size) = mConvertPosToY * vel_next;
                break;

            case btGenFeatureType::Location:
                ref_feature.segment(offset, size) = mConvertPosToXZ * vel_next;
                break;

            default:
                std::cout << "[error] illegal feature type "
                          << feature->mFeatureType << std::endl,
                    exit(0);
                break;
            }
        }
        else
        {
            std::cout << "[error] Neither joint nor pos feature, illegal "
                         "feature type "
                      << feature->mFeatureType << std::endl,
                exit(0);
        }
    }
    // std::cout << "vel feature = " << ref_feature.transpose() << std::endl;
    return ref_feature;
}

/**
 * \brief           Given the goal q in next frame, calculate the target pos
 * feature, \tilde{\phi}
 *
 * the definition of "\tilde{\phi}" depends on the definition of
 * "PosFeatureVectorArray"
 * 1. For joint angle feature, the target pos = q_next_i
 * 2. For height feature, the target pos feature = ConvertY * pos_next
 * 3. For location feature, the target pos feature = ConvertXZ * pos_next
 */
tVectorXd btGenFeatureArray::CalcTargetPosFeature(const tVectorXd &q)
{
    // 1. save the old status
    mModel->PushState("CalcTargetPosFeature");
    mModel->SetqAndqdot(q, mModel->Getqdot());

    // 2. begin to check each pos sub feature
    auto &pos_feature = mFeatureArrays[0];
    tVectorXd ref_feature = tVectorXd::Zero(pos_feature->mTotalFeatureSize);
    // std::cout << "pos feature size " << pos_feature->mTotalFeatureSize
    //           << std::endl;
    for (int fea_id = 0; fea_id < pos_feature->mNumOfFeature; fea_id++)
    {
        auto feature = pos_feature->mFeatureVector[fea_id];
        int offset = pos_feature->mFeatureOffset[fea_id];
        int size = GetSingleFeatureSize(feature);

        // only control the pos feature at this moment
        if (feature->mFeatureOrder != 0)
        {
            std::cout << "[error] CalcTargetPosFeature feature " << fea_id
                      << "is not 0-order(Pos), exit\n";
            exit(0);
        }

        if (IsJointFeature(feature->mFeatureType))
        {
            auto joint = mModel->GetJointById(feature->mLinkId);
            int q_size = joint->GetNumOfFreedom();
            int q_st = joint->GetFreedoms(0)->id;

            ref_feature.segment(offset, size) = q.segment(q_st, q_size);
        }
        else if (IsPosFeature(feature->mFeatureType))
        {
            // this feature is height or location
            auto link =
                dynamic_cast<Link *>(mModel->GetLinkById(feature->mLinkId));
            tVector3d pos = link->GetWorldPos();

            switch (feature->mFeatureType)
            {
            case btGenFeatureType::Height:
                ref_feature.segment(offset, size) = mConvertPosToY * pos;
                break;

            case btGenFeatureType::Location:
                ref_feature.segment(offset, size) = mConvertPosToXZ * pos;
                break;

            default:
                std::cout << "[error] illegal feature type "
                          << feature->mFeatureType << std::endl,
                    exit(0);
                break;
            }
        }
        else
        {
            std::cout << "[error] Neither joint nor pos feature, illegal "
                         "feature type "
                      << feature->mFeatureType << std::endl,
                exit(0);
        }
    }

    // restore the old status
    mModel->PopState("CalcTargetPosFeature");
    // std::cout << "pos feature = " << ref_feature.transpose() << std::endl;
    return ref_feature;
}

/**
 * \brief               Calculate the energy 
*/
void btGenFeatureArray::CalcEnergy(const tVectorXd &control_force,
                                   const tVectorXd &contact_force)
{
    // std::cout << "feature array: calculate energy begin\n";
    // 1. calculate the energy term
    int dof = mModel->GetNumOfFreedom();
    int under_dof =
        mModel->GetNumOfFreedom() - mModel->GetJointById(0)->GetNumOfFreedom();
    if (control_force.size() != under_dof || contact_force.size() != dof)
    {
        std::cout << "[error] CalcEnergy control_size " << dof
                  << " != " << under_dof << " or contact_size "
                  << contact_force.size() << " != " << dof << std::endl;
        exit(0);
    }
    const tVectorXd &tau_diff = mTargetTau - control_force;
    // std::cout << "tau diff = " << tau_diff.transpose() << std::endl;
    tEigenArr<tVectorXd> mCtrledFeature;
    for (int i = 0; i < 3; i++)
    {
        // std::cout << "begin calc " << i << std::endl;
        auto feature = mFeatureArrays[i];
        // std::cout << feature->mConvertMatFromContactForceToq.rows() << " "
        //           << feature->mConvertMatFromContactForceToq.cols()
        //           << std::endl;
        // std::cout << feature->mConvertMatFromTauToq.rows() << " "
        //           << feature->mConvertMatFromTauToq.cols() << std::endl;
        // std::cout << feature->mConvertResFromForceToq.rows() << " "
        //           << feature->mConvertResFromForceToq.cols() << std::endl;
        // std::cout << contact_force.rows() << " " << contact_force.cols()
        //           << std::endl;
        // std::cout << contact_force.rows() << " " << contact_force.cols()
        //           << std::endl;
        tVectorXd ctrl_q =
            (feature->mConvertMatFromContactForceToq * contact_force +
             feature->mConvertMatFromTauToq * control_force +
             feature->mConvertResFromForceToq);
        tVectorXd ctrl_feature = feature->mConvertMatFromqToFeature * ctrl_q +
                                 feature->mConvertResFromqToFeature;
        mCtrledFeature.push_back(ctrl_feature);
    }

    // const tVectorXd pos_diff =
    //     mCtrledFeature[0] - mFeatureArrays[0]->mRefFeature;
    // const tVectorXd vel_diff =
    //     mCtrledFeature[1] - mFeatureArrays[1]->mRefFeature;
    // const tVectorXd accel_diff =
    //     mCtrledFeature[2] - mFeatureArrays[2]->mRefFeature;
    std::ofstream fout(feature_energy_path, std::ios::app);
    for (int i = 0; i < 3; i++)
    {
        // std::cout << "---------------begin order " << i << std::endl;
        tVectorXd diff = mCtrledFeature[i] - mFeatureArrays[i]->mRefFeature;
        tVectorXd weighted_diff = mFeatureArrays[i]->mWeight.cwiseProduct(diff);
        // std::cout << "diff = " << diff.transpose() << std::endl;
        // std::cout << "weighted diff = " << weighted_diff.transpose()
        //           << std::endl;
        // std::cout << "diff size = " << diff.size() << std::endl;
        auto feature_array = mFeatureArrays[i];
        std::string prefix = "";
        switch (i)
        {
        case 0:
            prefix = "[pos]";
            break;
        case 1:
            prefix = "[vel]";
            break;
        case 2:
            prefix = "[accel]";
            break;

        default:
            std::cout << "[error] gen feature order " << i << " unsupported\n";
            exit(1);
            break;
        }
        for (int id = 0; id < feature_array->mNumOfFeature; id++)
        {
            auto cur_feature = feature_array->mFeatureVector[id];
            int order = cur_feature->mFeatureOrder;
            int offset = feature_array->mFeatureOffset[id];
            int size = GetSingleFeatureSize(cur_feature);
            std::string link_name = cur_feature->mLinkName;
            std::string feature_name = cur_feature->mFeatureName;
            fout << "[fea] " << feature_name
                 << " diff = " << diff.segment(offset, size).transpose()
                 << " weighted diff = "
                 << weighted_diff.segment(offset, size).transpose()
                 << std::endl;
        }
        fout << "[fea] " << prefix << " energy = " << weighted_diff.norm()
             << std::endl;
    }

    // std::cout << "pos diff = " << pos_diff.transpose() << std::endl;
    // std::cout << "vel diff = " << vel_diff.transpose() << std::endl;
    // std::cout << "accel diff = " << accel_diff.transpose() << std::endl;

    // exit(0);
    fout.close();
}

extern bool gPauseSimulation;
void btGenFeatureArray::DebugAccelFeatureDFdtauIsZero(
    const tVectorXd &control_force, const tVectorXd &contact_force)
{
    // 1. confirm there is only joint accel feature, no vel & pos & control torque feature
    auto alpha_feature = mFeatureArrays[2];
    if (mFeatureArrays[0]->mNumOfFeature != 0)
    {
        std::cout << "[error] pos size is not zero\n";
        exit(1);
    }
    if (mFeatureArrays[1]->mNumOfFeature != 0)
    {
        std::cout << "[error] vel size is not zero\n";
        exit(1);
    }
    if (mWeight_tau.norm() > 0)
    {
        std::cout << "[error] tau weight is not zero "
                  << mWeight_tau.transpose();
        exit(1);
    }
    // for (int i = 0; i < alpha_feature->mNumOfFeature; i++)
    // {
    //     auto cur_feature = alpha_feature->mFeatureVector[i];

    //     if (IsJointFeature(cur_feature->mFeatureType) == false)
    //     {
    //         std::cout << "[error] alpha feature " << i << " type "
    //                   << cur_feature->mFeatureType << std::endl;
    //         exit(0);
    //     }
    // }
    // 2. begin to check the result

    const tMatrixXd &alpha_weight = mFeatureArrays[2]->mWeight.asDiagonal();
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    int num_of_freedom = mModel->GetNumOfFreedom(),
        num_of_underactuated_freedom = mModel->GetNumOfFreedom() - 6;
    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    const tVectorXd &QG = mModel->CalcGenGravity(mGravity);
    const tVectorXd &Cqdot = mModel->GetCoriolisMatrix() * mModel->Getqdot();

    // 2.1 calculate the relationship between accel feature vector, tau and contact force
    // qddot = M_tau_to_q * tau + M_contact_to_q * contact_force + R_force_to_q
    // feature_alpha = M_q_to_feature * qddot + R_q_to_feature
    // tMatrixXd A1 = tMatrixXd::Zero(alpha_feature->mTotalFeatureSize,
    //                                num_of_underactuated_freedom),
    //           A2 = tMatrixXd::Zero(alpha_feature->mTotalFeatureSize,
    //                                num_of_freedom);
    // tVectorXd b1 = tVectorXd::Zero(alpha_feature->mTotalFeatureSize);
    tMatrixXd M_tau_to_q = Minv * N, M_contact_to_q = Minv,
              M_q_to_feature = tMatrixXd::Zero(alpha_feature->mTotalFeatureSize,
                                               num_of_freedom);
    tVectorXd R_force_to_q = Minv * (QG - Cqdot),
              R_q_to_feature =
                  tVectorXd::Zero(alpha_feature->mTotalFeatureSize);
    // now shape the convert mat & convert res from q to feature
    {
        for (int id = 0; id < alpha_feature->mNumOfFeature; id++)
        {
            auto sub_feature = alpha_feature->mFeatureVector[id];
            int fea_offset = alpha_feature->mFeatureOffset[id];
            int fea_size = GetSingleFeatureSize(sub_feature);
            auto link = mModel->GetLinkById(sub_feature->mLinkId);
            const tMatrixXd &Jv = link->GetJKv();
            const tVectorXd &dJvdt_qdot =
                link->GetJKv_dot() * mModel->Getqdot();

            auto joint = link->GetParent();
            if (IsJointFeature(sub_feature->mFeatureType))
            {
                int joint_offset = joint->GetFreedoms(0)->id;
                int joint_dof = joint->GetNumOfFreedom();
                if (joint_dof != fea_size)
                {
                    std::cout << "[error] debugging accel feature: joint "
                              << sub_feature->mLinkId << " feature size "
                              << fea_size << " != " << joint_dof << std::endl;
                    exit(0);
                }

                for (int i = 0; i < joint_dof; i++)
                {
                    M_q_to_feature(fea_offset + i, joint_offset + i) = 1.0;
                    R_q_to_feature[fea_offset + i] = 0;
                }
                // // fill in the A1
                // A1.block(fea_offset, 0, fea_size,
                //          num_of_underactuated_freedom) =
                //     (Minv * N).block(joint_offset, 0, joint_dof,
                //                      num_of_underactuated_freedom);
                // // fill in the A2
                // A2.block(fea_offset, 0, fea_size, num_of_freedom) =
                //     Minv.block(joint_offset, 0, fea_size, num_of_freedom);

                // // fill in the b1
                // b1.segment(fea_offset, fea_size) =
                //     (Minv * (QG - Cqdot)).segment(joint_offset, joint_dof);

                // fill in vars from contact_force/control_force to q
            }
            else if (IsPosFeature(sub_feature->mFeatureType))
            {
                if (sub_feature->mFeatureType == btGenFeatureType::Height)
                {
                    M_q_to_feature.block(fea_offset, 0, fea_size,
                                         num_of_freedom) = mConvertPosToY * Jv;
                    R_q_to_feature.segment(fea_offset, fea_size) =
                        mConvertPosToY * dJvdt_qdot;
                    // // fill in the A1
                    // A1.block(fea_offset, 0, fea_size,
                    //          num_of_underactuated_freedom) =
                    //     mConvertPosToY * (Jv * Minv * N);
                    // // fill in the A2
                    // A2.block(fea_offset, 0, fea_size, num_of_freedom) =
                    //     mConvertPosToY * Jv * Minv;

                    // // fill in the b1
                    // b1.segment(fea_offset, fea_size) =
                    //     mConvertPosToY * (Jv * Minv * (QG - Cqdot) +
                    //                       dJvdt_qdot * mModel->Getqdot());
                }
                else if (sub_feature->mFeatureType ==
                         btGenFeatureType::Location)
                {
                    M_q_to_feature.block(fea_offset, 0, fea_size,
                                         num_of_freedom) = mConvertPosToXZ * Jv;
                    R_q_to_feature.segment(fea_offset, fea_size) =
                        mConvertPosToXZ * dJvdt_qdot;
                    // std::cout << "[debug] qdot norm " << qdot.norm()
                    //           << std::endl;
                    // std::cout << "[debug] Jvdot norm " << Jv_dot.norm()
                    //           << std::endl;
                    // std::cout << "[debug] convert to xz norm "
                    //           << mConvertPosToXZ.norm() << std::endl;
                    // // fill in the A1
                    // A1.block(fea_offset, 0, fea_size,
                    //          num_of_underactuated_freedom) =
                    //     mConvertPosToXZ * (Jv * Minv * N);
                    // // fill in the A2
                    // A2.block(fea_offset, 0, fea_size, num_of_freedom) =
                    //     mConvertPosToXZ * Jv * Minv;

                    // // fill in the b1
                    // b1.segment(fea_offset, fea_size) =
                    //     mConvertPosToXZ *
                    //     (Jv * Minv * (QG - Cqdot) + dJvdt_qdot);
                }
                else
                {
                    std::cout << "[error] pos feature type is not recognized: "
                              << sub_feature->mFeatureType << std::endl;
                    exit(1);
                }
            }
            else
            {
                std::cout
                    << "[error] feature cannot be recognized in debugging\n";
                exit(0);
            }
        }
    }
    // tMatrixXd part1 = (alpha_weight * Minv * N).transpose() * alpha_weight;
    // tMatrixXd part2a = Minv * N;
    // tMatrixXd part2b = Minv;
    // tVectorXd part3 = Minv * (QG - Cqdot) - mTargetAccel;
    tMatrixXd O = M_q_to_feature * M_contact_to_q,
              P = M_q_to_feature * M_tau_to_q;
    tVectorXd R = M_q_to_feature * R_force_to_q + R_q_to_feature;
    // tMatrixXd part1 =
    //     2 * A1.transpose() * alpha_weight.transpose() * alpha_weight;
    // tMatrixXd part2a = A1;
    // tMatrixXd part2b = A2;
    // tVectorXd part3 = b1 - alpha_feature->mRefFeature;
    // std::cout << "O norm " << O.norm() << std::endl;
    // std::cout << "P norm " << P.norm() << std::endl;
    // std::cout << "R norm " << R.norm() << std::endl;
    tVectorXd res = 2 * P.transpose() * alpha_weight.transpose() *
                    alpha_weight *
                    (O * contact_force + P * control_force + R -
                     alpha_feature->mRefFeature);
    std::cout << "res = " << res.transpose() << std::endl;
    const double eps = 1e-4;
    if (res.norm() > eps)
    {
        std::cout << "[error] judge failed!\n";
        // check convert mat from contact force to qddot
        {
            const tMatrixXd mat_from_contact_to_q = M_contact_to_q;
            const tMatrixXd mat_from_contact_to_q_diff =
                Minv - alpha_feature->mConvertMatFromContactForceToq;
            if (mat_from_contact_to_q_diff.norm() > eps)
            {
                std::cout << "[error] from contact to q mat error, diff = "
                          << mat_from_contact_to_q_diff.norm() << std::endl;
                exit(1);
            }
        }

        // check convert mat from control force to qddot
        {
            const tMatrixXd mat_from_control_force_to_q = M_tau_to_q;
            const tMatrixXd mat_from_control_force_to_q_diff =
                mat_from_control_force_to_q -
                alpha_feature->mConvertMatFromTauToq;
            if (mat_from_control_force_to_q_diff.norm() > eps)
            {
                std::cout << "[error] from control to q mat error, diff = "
                          << mat_from_control_force_to_q_diff.norm()
                          << std::endl;
                exit(1);
            }
        }

        // check the convert mat from q to feature
        {
            const tMatrixXd mat_from_q_to_feature = M_q_to_feature;
            const tMatrixXd diff = alpha_feature->mConvertMatFromqToFeature -
                                   mat_from_q_to_feature;
            if (diff.norm() > eps)
            {
                std::cout << "[error] from q to feature mat error, diff = "
                          << diff.norm() << std::endl;
                exit(1);
            }
        }

        // check the convert res from force to q
        {
            const tVectorXd res_from_force_to_q = R_force_to_q;
            const tVectorXd diff =
                res_from_force_to_q - alpha_feature->mConvertResFromForceToq;
            if (diff.norm() > eps)
            {
                std::cout << "[error] from force to q res error, diff = "
                          << diff.norm() << std::endl;
                exit(1);
            }
        }

        // check the convert res from qddot to feature
        {
            const tVectorXd res_from_q_to_feature = R_q_to_feature;
            const tVectorXd diff = res_from_q_to_feature -
                                   alpha_feature->mConvertResFromqToFeature;
            if (diff.norm() > eps)
            {
                std::cout << "[error] from q to feature res error, diff = "
                          << diff.norm() << std::endl;
                std::cout
                    << "applied var = "
                    << alpha_feature->mConvertResFromqToFeature.transpose()
                    << std::endl;
                std::cout << "debug var = " << res_from_q_to_feature.transpose()
                          << std::endl;
                exit(1);
            }
            // std::cout << "convert res to feature "
            //           << alpha_feature->mConvertResFromqToFeature.transpose()
            //           << std::endl;
        }

        {
            const tMatrixXd E_local =
                P.transpose() * alpha_weight.transpose() * alpha_weight * P;
            const tMatrixXd F_local =
                P.transpose() * alpha_weight.transpose() * alpha_weight * O;
            const tVectorXd f_local = -P.transpose() *
                                      alpha_weight.transpose() * alpha_weight *
                                      (R - alpha_feature->mRefFeature);
            const tMatrixXd H_local = -E_local.inverse() * F_local;

            const tMatrixXd H_diff = H_local - H_debug,
                            E_diff = E_local - E_debug,
                            F_diff = F_local - F_debug;
            const tVectorXd f_diff = f_local - f_debug;
            if (E_diff.norm() > eps)
            {
                std::cout << "[error] E error, diff = " << E_diff.norm()
                          << std::endl;
                exit(0);
            }
            if (F_diff.norm() > eps)
            {
                std::cout << "[error] F error, diff = " << E_diff.norm()
                          << std::endl;
                exit(0);
            }
            if (H_diff.norm() > eps)
            {
                std::cout << "[error] H error, diff = " << H_diff.norm()
                          << std::endl;
                exit(0);
            }

            if (f_diff.norm() > eps)
            {
                std::cout << "[error] f error, diff = " << f_diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        gPauseSimulation = true;
    }
    // alpha_feature->mConvertMatFromContactForceToq;
    // alpha_feature->mConvertMatFromqToFeature;
    // alpha_feature->mConvertMatFromTauToq;
    // alpha_feature->mConvertResFromqToFeature;
    // alpha_feature->mConvertResFromForceToq;
}

void btGenFeatureArray::DebugTauFeatureDFdtauIsZero(
    const tVectorXd &control_force, const tVectorXd &contact_force)
{
    for (int i = 0; i < 3; i++)
    {
        if (mFeatureArrays[i]->mNumOfFeature != 0)
        {
            std::cout << "[error] tau feature verified is enable but there is "
                         "other feature, order "
                      << i << std::endl;
            exit(0);
        }
    }
    tMatrixXd tau_weight = mWeight_tau.asDiagonal();
    tVectorXd res =
        2 * tau_weight.transpose() * tau_weight * (control_force - mTargetTau);

    std::cout << "res = " << res.transpose() << std::endl;
    if (res.norm() > 1e-8)
    {
        tVectorXd diff = control_force - mTargetTau;
        std::cout << "[error] tau feature verified failed, diff "
                  << diff.transpose() << std::endl;
        exit(1);
    }
}

void btGenFeatureArray::DebugVelFeatureDFdtauIsZero(
    const tVectorXd &control_force, const tVectorXd &contact_force)
{
    // 1. make sure that pos and accel feature is disabled
    if (mFeatureArrays[0]->mNumOfFeature != 0 ||
        mFeatureArrays[2]->mNumOfFeature != 0)
    {
        std::cout << "[error] debug vel: pos & accel feature num is not zero: "
                  << mFeatureArrays[0]->mNumOfFeature << " "
                  << mFeatureArrays[2]->mNumOfFeature << std::endl;
        exit(0);
    }

    // 2. make sure that tau weight is disable
    if (mWeight_tau.norm() > 1e-10)
    {
        std::cout << "[error] debug vel: weight tau is zero\n";
        exit(0);
    }

    // 3. begin to form the convert matrices

    auto vel_feature = mFeatureArrays[1];
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    int num_of_freedom = mModel->GetNumOfFreedom(),
        num_of_underactuated_freedom = mModel->GetNumOfFreedom() - 6;
    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    const tVectorXd &QG = mModel->CalcGenGravity(mGravity);
    const tVectorXd &Cqdot = mModel->GetCoriolisMatrix() * mModel->Getqdot();

    tMatrixXd M_tau_to_q = mCurTimestep * Minv * N,
              M_contact_to_q = mCurTimestep * Minv,
              M_q_to_feature = tMatrixXd::Zero(vel_feature->mTotalFeatureSize,
                                               num_of_freedom);
    tVectorXd R_force_to_q =
                  mCurTimestep * Minv * (QG - Cqdot) + mModel->Getqdot(),
              R_q_to_feature = tVectorXd::Zero(vel_feature->mTotalFeatureSize);

    for (int id = 0; id < vel_feature->mNumOfFeature; id++)
    {
        auto cur_feature = vel_feature->mFeatureVector[id];
        int feature_offset = vel_feature->mFeatureOffset[id];
        int feature_size = GetSingleFeatureSize(cur_feature);
        auto link = mModel->GetLinkById(cur_feature->mLinkId);
        if (IsJointFeature(cur_feature->mFeatureType))
        {
            auto joint = link->GetParent();
            int joint_offset = joint->GetFreedoms(0)->id;
            if (joint->GetNumOfFreedom() != feature_size)
            {
                std::cout << "[error] vel feature check: feature " << id
                          << " link " << cur_feature->mLinkId
                          << " dof inconsisitent\n";
                exit(0);
            }
            for (int i = 0; i < joint->GetNumOfFreedom(); i++)
            {
                M_q_to_feature(feature_offset + i, joint_offset + i) = 1.0;
                R_q_to_feature[feature_offset + i] = 0;
            }
        }
        else if (IsPosFeature(cur_feature->mFeatureType))
        {
            if (cur_feature->mFeatureType == btGenFeatureType::Height)
            {
                M_q_to_feature.block(feature_offset, 0, feature_size,
                                     num_of_freedom) =
                    mConvertPosToY * link->GetJKv();
                R_q_to_feature.segment(feature_offset, feature_size).setZero();
            }
            else if (cur_feature->mFeatureType == btGenFeatureType::Location)
            {
                M_q_to_feature.block(feature_offset, 0, feature_size,
                                     num_of_freedom) =
                    mConvertPosToXZ * link->GetJKv();
                R_q_to_feature.segment(feature_offset, feature_size).setZero();
            }

            // std::cout << "[error] Vel feature check, unsupported pos feature\n";
            // exit(0);
        }
        else
        {
            std::cout << "[error] Vel feature check, unsupported feature type "
                      << cur_feature->mFeatureType << std::endl;
            exit(0);
        }
    }

    // 4. shape the dF/d\tau formula
    tMatrixXd vel_weight = vel_feature->mWeight.asDiagonal();
    tMatrixXd part1 = M_tau_to_q.transpose() * M_q_to_feature.transpose() *
                      vel_weight.transpose() * vel_weight;
    tMatrixXd part2_1 = M_q_to_feature * M_contact_to_q;
    tMatrixXd part2_2 = M_q_to_feature * M_tau_to_q;
    tVectorXd part3 = M_q_to_feature * R_force_to_q + R_q_to_feature -
                      vel_feature->mRefFeature;
    tVectorXd vel_res =
        part1 * (part2_1 * contact_force + part2_2 * control_force + part3);
    std::cout << "[log] vel res = " << vel_res.transpose() << std::endl;
    double eps = 1e-5;
    if (vel_res.norm() > eps)
    {
        std::cout << "[error] vel feature verified failed\n";

        // 1. check the convert matrices
        {
            tMatrixXd diff = M_tau_to_q - vel_feature->mConvertMatFromTauToq;

            if (diff.norm() > eps)
            {
                std::cout << "[error] M tau to q diff = " << diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        {
            tMatrixXd diff =
                M_contact_to_q - vel_feature->mConvertMatFromContactForceToq;

            if (diff.norm() > eps)
            {
                std::cout << "[error] M contact to q diff = " << diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        {
            tMatrixXd diff =
                M_q_to_feature - vel_feature->mConvertMatFromqToFeature;

            if (diff.norm() > eps)
            {
                std::cout << "[error] M q to feature diff = " << diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        {
            tMatrixXd diff =
                R_q_to_feature - vel_feature->mConvertResFromqToFeature;

            if (diff.norm() > eps)
            {
                std::cout << "[error] R q to feature diff = " << diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        {
            tMatrixXd diff =
                R_force_to_q - vel_feature->mConvertResFromForceToq;

            if (diff.norm() > eps)
            {
                std::cout << "[error] R force to q diff = " << diff.norm()
                          << std::endl;
                exit(0);
            }
        }

        // verify H,
        {
            const tMatrixXd E_local = part1 * part2_2;
            const tMatrixXd F_local = part1 * part2_1;
            const tVectorXd f_local = -part1 * part3;
            const tMatrixXd H_local = -E_local.inverse() * F_local;

            const tMatrixXd H_diff = H_local - H_debug,
                            E_diff = E_local - E_debug,
                            F_diff = F_local - F_debug;
            const tVectorXd f_diff = f_local - f_debug;
            if (E_diff.norm() > eps)
            {
                std::cout << "[error] E error, diff = " << E_diff.norm()
                          << std::endl;
                std::cout << "[verify] E = " << E_local.norm() << std::endl;
                std::cout << "[verify] D_nu = " << part2_2.norm() << std::endl;
                std::cout << "[verify] D_nu.T * W2TW2 legacy = " << part1.norm()
                          << std::endl;
                // std::cout << "[verify] D_nu.T * W2TW2 new = " << (part2_2.transpose() * ).norm() << std::endl;
                std::cout << "[verify] W2TW2 = "
                          << (vel_weight.transpose() * vel_weight).norm()
                          << std::endl;

                exit(0);
            }
            if (F_diff.norm() > eps)
            {
                std::cout << "[error] F error, diff = " << E_diff.norm()
                          << std::endl;
                exit(0);
            }
            if (H_diff.norm() > eps)
            {
                std::cout << "[error] H error, diff = " << H_diff.norm()
                          << std::endl;
                exit(0);
            }

            if (f_diff.norm() > eps)
            {
                std::cout << "[error] f error, diff = " << f_diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        exit(0);
    }
}

void btGenFeatureArray::DebugPosFeatureDFdtauIsZero(
    const tVectorXd &control_force, const tVectorXd &contact_force)
{
    // 1. judge that the vel & accel feature is empty
    if (mFeatureArrays[1]->mNumOfFeature != 0 ||
        mFeatureArrays[2]->mNumOfFeature != 0)
    {
        std::cout
            << "[error] pos feature debug: vel || accel feature isn't empty\n";
        exit(0);
    }

    // 2. make sure that tau weight is disabled
    if (mWeight_tau.norm() > 1e-10)
    {
        std::cout << "[error] pos feature debug: weight tau is zero\n";
        exit(0);
    }

    // 3. begin to from the convert matrix
    auto pos_feature = mFeatureArrays[0];
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    int num_of_freedom = mModel->GetNumOfFreedom(),
        num_of_underactuated_freedom = mModel->GetNumOfFreedom() - 6;
    tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
    const tVectorXd &QG = mModel->CalcGenGravity(mGravity);
    const tVectorXd &Cqdot = mModel->GetCoriolisMatrix() * mModel->Getqdot();
    double dt = mCurTimestep;
    double dt2 = mCurTimestep * mCurTimestep;

    tMatrixXd M_tau_to_q = dt2 * Minv * N, M_contact_to_q = dt2 * Minv,
              M_q_to_feature = tMatrixXd::Zero(pos_feature->mTotalFeatureSize,
                                               num_of_freedom);
    tVectorXd R_force_to_q = dt2 * Minv * (QG - Cqdot) +
                             dt * mModel->Getqdot() + mModel->Getq(),
              R_q_to_feature = tVectorXd::Zero(pos_feature->mTotalFeatureSize);

    // from q to feature, mat & res
    tMatrixXd pos_weight = pos_feature->mWeight.asDiagonal();

    for (int id = 0; id < pos_feature->mNumOfFeature; id++)
    {
        auto &cur_feature = pos_feature->mFeatureVector[id];
        auto link = mModel->GetLinkById(cur_feature->mLinkId);
        auto joint = link->GetParent();
        const tMatrixXd &Jv = link->GetJKv();
        int fea_offset = pos_feature->mFeatureOffset[id];
        int fea_size = GetSingleFeatureSize(cur_feature);

        if (IsJointFeature(cur_feature->mFeatureType))
        {
            int joint_dof = joint->GetNumOfFreedom();
            int joint_offset = joint->GetFreedoms(0)->id;
            for (int i = 0; i < joint_dof; i++)
            {
                M_q_to_feature(fea_offset + i, joint_offset + i) = 1.0;
                R_q_to_feature[fea_offset + i] = 0;
            }
        }
        else if (IsPosFeature(cur_feature->mFeatureType))
        {
            tVector3d link_pos = link->GetWorldPos();
            if (cur_feature->mFeatureType == btGenFeatureType::Height)
            {
                M_q_to_feature.block(fea_offset, 0, fea_size, num_of_freedom) =
                    mConvertPosToY * Jv;
                R_q_to_feature.segment(fea_offset, fea_size) =
                    mConvertPosToY * (link_pos - Jv * mModel->Getq());
                // std::cout << "[veri] link pos " << link_pos.transpose() << std::endl;
            }
            else if (cur_feature->mFeatureType == btGenFeatureType::Location)
            {
                M_q_to_feature.block(fea_offset, 0, fea_size, num_of_freedom) =
                    mConvertPosToXZ * Jv;
                R_q_to_feature.segment(fea_offset, fea_size) =
                    mConvertPosToXZ * (link_pos - Jv * mModel->Getq());
                        }
            else
            {
                std::cout << "[error] pos verify, type "
                          << cur_feature->mFeatureType << " unsupported\n";
                exit(0);
            }
        }
        else
        {
            std::cout << "[error] unrecognized feature type "
                      << cur_feature->mFeatureType << std::endl;
            exit(0);
        }
    }

    // 4. begin to form the dFdtau convert matrices
    tMatrixXd part1 = M_tau_to_q.transpose() * M_q_to_feature.transpose() *
                      pos_weight.transpose() * pos_weight,
              part21 = M_q_to_feature * M_tau_to_q,
              part22 = M_q_to_feature * M_contact_to_q;
    tVectorXd part3 = M_q_to_feature * R_force_to_q + R_q_to_feature -
                      pos_feature->mRefFeature;

    tVectorXd res =
        part1 * (part21 * control_force + part22 * contact_force + part3);
    std::cout << "pos res = " << res.transpose() << std::endl;
    double eps = 1e-8;
    if (res.norm() > eps)
    {
        std::cout << "[error] pos feature verified failed\n";
        // 1. check the convert matrices
        {
            tMatrixXd diff = M_tau_to_q - pos_feature->mConvertMatFromTauToq;

            if (diff.norm() > eps)
            {
                std::cout << "[error] M tau to q diff = " << diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        {
            tMatrixXd diff =
                M_contact_to_q - pos_feature->mConvertMatFromContactForceToq;

            if (diff.norm() > eps)
            {
                std::cout << "[error] M contact to q diff = " << diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        {
            tMatrixXd diff =
                M_q_to_feature - pos_feature->mConvertMatFromqToFeature;

            if (diff.norm() > eps)
            {
                std::cout << "[error] M q to feature diff = " << diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        {
            tMatrixXd diff =
                R_q_to_feature - pos_feature->mConvertResFromqToFeature;

            if (diff.norm() > eps)
            {
                std::cout << "[error] R q to feature diff = " << diff.norm()
                          << std::endl;
                std::cout << "[veri] Res q to feature = "
                          << R_q_to_feature.transpose() << std::endl;
                std::cout << "[raw] Res q to feature = "
                          << pos_feature->mConvertResFromqToFeature.transpose()
                          << std::endl;

                exit(0);
            }
        }
        {
            tMatrixXd diff =
                R_force_to_q - pos_feature->mConvertResFromForceToq;

            if (diff.norm() > eps)
            {
                std::cout << "[error] R force to q diff = " << diff.norm()
                          << std::endl;
                exit(0);
            }
        }

        // verify H,
        {
            const tMatrixXd E_local = part1 * part21;
            const tMatrixXd F_local = part1 * part22;
            const tVectorXd f_local = -part1 * part3;
            const tMatrixXd H_local = -E_local.inverse() * F_local;

            const tMatrixXd H_diff = H_local - H_debug,
                            E_diff = E_local - E_debug,
                            F_diff = F_local - F_debug;
            const tVectorXd f_diff = f_local - f_debug;
            if (E_diff.norm() > eps)
            {
                std::cout << "[error] E error, diff = " << E_diff.norm()
                          << std::endl;

                exit(0);
            }
            if (F_diff.norm() > eps)
            {
                std::cout << "[error] F error, diff = " << E_diff.norm()
                          << std::endl;
                exit(0);
            }
            if (H_diff.norm() > eps)
            {
                std::cout << "[error] H error, diff = " << H_diff.norm()
                          << std::endl;
                exit(0);
            }

            if (f_diff.norm() > eps)
            {
                std::cout << "[error] f error, diff = " << f_diff.norm()
                          << std::endl;
                exit(0);
            }
        }
        exit(0);
    }
}