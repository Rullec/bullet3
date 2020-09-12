#include "btGenFeature.h"
#include "BulletGenDynamics/btGenModel/Link.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "btTraj.h"
#include <iostream>
bool IsJointFeature(btGenFeatureType type)
{
    return type >= btGenFeatureType::SphereJoint &&
           type <= btGenFeatureType::FixedJoint;
}
bool IsPosFeature(btGenFeatureType type)
{
    return type >= btGenFeatureType::Height &&
           type <= btGenFeatureType::Location;
}

bool FeatureSort(const btGenFeature *f1, const btGenFeature *f2)
{
    return f1->mLinkId < f2->mLinkId;
}
std::string GetFeatureSuffix(const std::string &name)
{
    int st = name.find("_");
    if (st == -1)
    {
        std::cout << "[error] GetFeatureSuffix _ not found in " << name
                  << std::endl;
        exit(1);
    }
    return name.substr(st + 1, name.size() - st);
}
std::string GetFeatureLinkName(const std::string &name)
{
    int st = name.find("_");
    if (st == -1)
    {
        std::cout << "[error] GetFeatureLinkName _ not found in " << name
                  << std::endl;
        exit(1);
    }
    return name.substr(0, st);
}

int GetFeatureOrder(const Json::Value &value)
{
    // std::cout << value << std::endl;
    if (value.size() != 2)
    {
        std::cout << "GetFeatureOrder: format error\n";
    }
    std::string order_str = value[0].asString();
    int order = -1;
    if (order_str == "accel")
        order = 2;
    else if (order_str == "vel")
        order = 1;
    else if (order_str == "pos")
        order = 0;
    else
    {
        std::cout << "invalid feature order " << order_str << std::endl;
        exit(1);
    }
    return order;
    // return value[0].;
}

double GetFeatureWeight(const Json::Value &value)
{
    // std::cout << value << std::endl;
    if (value.size() != 2)
    {
        std::cout << "GetFeatureWeight: format error\n";
    }

    return value[1].asDouble();
}

btGenFeatureArray::btGenFeatureArray()
{
    mModel = nullptr;
    // mTraj = nullptr;
    mNumOfFeature = 0;
    mTotalFeatureSize = 0;
    mFeatureVector.clear();
    mFeatureOffset.clear();
    mRefFeature.resize(0);
    mW.resize(0);
    mWm.resize(0);

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
        // exit(1);
    }
}

btGenFeatureArray::~btGenFeatureArray()
{
    for (auto &x : mFeatureVector)
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
                             cRobotModelDynamics *model, btTraj *traj,
                             const tVector &g)
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
    InitWeight(conf);
    // InitRefFeature();
}

/**
 * \brief				given config file, create features
 */
void btGenFeatureArray::InitFeature(const std::string &conf)
{
    std::cout << "[log] begin to init feature array from " << conf << std::endl;
    Json::Value root;
    btJsonUtil::LoadJson(conf, root);

    // 1. create all features
    Json::Value::Members keys = root.getMemberNames();
    // std::cout << "num of keys = " << keys.size() << std::endl;
    // std::cout << "before total feature size = " << mTotalFeatureSize <<
    // std::endl;
    for (int i = 0; i < keys.size(); i++)
    {
        // std::cout << "handle key " << i << std::endl;
        std::string feature_name = keys[i];
        std::string suffix = GetFeatureSuffix(feature_name);
        std::string joint_name = GetFeatureLinkName(feature_name);
        btGenFeature *feature = nullptr;
        // add the joint accel into the feature vector
        if ("joint" == suffix)
            feature = CreateJointFeature(joint_name);
        else if ("height" == suffix)
            feature = CreateHeightFeature(joint_name);
        else if ("location" == suffix)
            feature = CreateLocationFeature(joint_name);
        else if ("weight" == suffix)
            continue;
        else
        {
            std::cout << "[error] unsupported feature suffix = " << suffix
                      << std::endl;
            exit(1);
        }
        feature->mFeatureOrder = GetFeatureOrder(root[feature_name]);
        feature->mWeight = GetFeatureWeight(root[feature_name]);
        // std::cout << "feature name " << feature_name << std::endl;
        // std::cout << "feature type " << feature->mFeatureType << std::endl;

        mTotalFeatureSize += GetSingleFeatureSize(feature);
        // std::cout << "cur size = " << mTotalFeatureSize << std::endl;
        mFeatureVector.push_back(feature);
    }

    // std::cout << "total feature size = " << mTotalFeatureSize << std::endl;
    // exit(1);
    // 2. sort the feature according to link id, because the json file has an
    // random key order then build the feature offset (the begin position of a
    // specieifed feature in a vector)
    std::sort(mFeatureVector.begin(), mFeatureVector.end(), FeatureSort);
    mNumOfFeature = mFeatureVector.size();
    mFeatureOffset.resize(mNumOfFeature, 0);
    for (int i = 0, offset = 0; i < mNumOfFeature; i++)
    {
        mFeatureOffset[i] = offset;
        offset += GetSingleFeatureSize(mFeatureVector[i]);
    }

    // 3. validate the feature array: are all dof controlled by this feature? If
    // not, exit
    PrintFeatureInfo();
    std::cout << "[log] end to init feature array\n";
}

/**
 * \brief					Init the control weight
 *
 *
 * 		W \in R{k \times k}
 * 		Wm \in R{n-6 \times n-6}
 */
void btGenFeatureArray::InitWeight(const std::string &conf)
{
    // std::cout << "begin to init weight\n";

    // init W
    mW = tVectorXd::Zero(mTotalFeatureSize);
    // std::cout << "feature size = " << mTotalFeatureSize << std::endl;
    int offset = 0;
    for (int i = 0; i < mNumOfFeature; i++)
    {
        // std::cout << "handle " << i << std::endl;
        const auto &feature = mFeatureVector[i];
        int size = GetSingleFeatureSize(feature);
        mW.segment(offset, size).fill(feature->mWeight);
        offset += size;
    }

    // init Wm
    Json::Value root;
    btJsonUtil::LoadJson(conf, root);
    double tau_weight = btJsonUtil::ParseAsDouble("tau_weight", root);
    int num_of_links = mModel->GetNumOfLinks();
    int num_of_freedom = mModel->GetNumOfFreedom();
    mWm.resize(num_of_freedom - 6);
    offset = 0;
    for (int link_id = 1; link_id < num_of_links; link_id++)
    {
        auto joint = mModel->GetLinkById(link_id)->GetParent();
        int dof = joint->GetNumOfFreedom();
        mWm.segment(offset, dof).fill(tau_weight);
        offset += dof;
    }

    std::cout << "mW = " << mW.transpose() << std::endl;
    std::cout << "mWm = " << mWm.transpose() << std::endl;
    // exit(1);
}

/**
 * \brief						Evaluate in this reference frame, get the
 * feature matrix H, E, f whicah can be used later
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
void btGenFeatureArray::Eval(const tVectorXd &qddot_target,
                             const tVectorXd &tau_target, tMatrixXd &H,
                             tMatrixXd &E, tVectorXd &f)
{
    // if (target_frame >= mTraj->mNumOfFrames - 2)
    // {
    // 	std::cout << "[error] void btGenFeatureArray::Eval frame " <<
    // target_frame << " exceeds the range\n"; 	exit(1);
    // }
    mRefFeature = CalcTargetFeature(qddot_target);
    EvalConvertMatAndResidual(mC_alpha, md_alpha);
    // std::cout << "mC alpha = \n"
    // 		  << mC_alpha << std::endl;
    // std::cout << "md alpha = \n"
    // 		  << md_alpha.transpose() << std::endl;
    // exit(1);
    EvalPrerequistite(mD_tau, mM_Q, mn_tau);
    EvalDynamicTerms(mRefFeature, tau_target, H, E, f);
    // std::cout << "ref feature vector " <<
    // mRefFeature[target_frame].transpose() << std::endl; std::cout << "mH =
    // \n"
    // 		  << H << std::endl;
    // exit(1);
}

int btGenFeatureArray::GetSingleFeatureSize(btGenFeature *feature) const
{
    if (btGenFeatureType::INVALID_FEATURE_TYPE == feature->mFeatureType ||
        feature->mFeatureType >= btGenFeatureType::NUM_OF_GEN_FEATURE_TYPE)
    {
        std::cout << "[error] GetSingleFeatureSize: invalid feature type\n";
        exit(1);
    }
    return gGenFeatureSize[feature->mFeatureType];
}

int btGenFeatureArray::GetSingleFeatureOffset(int feature_id) const
{
    return mFeatureOffset[feature_id];
}
/**
 * \brief					Create joint features
 */
btGenFeature *
btGenFeatureArray::CreateJointFeature(const std::string &joint_name) const
{
    BaseObject *joint = mModel->GetLink(joint_name)->GetParent();
    if (joint == nullptr)
    {
        std::cout << "[error] target joint " << joint_name << " not found\n";
        exit(1);
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
        exit(1);
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
    std::vector<bool> dof_controllable_lst(num_of_links, false);
    for (const auto &x : mFeatureVector)
    {
        // std::cout << "feature " << x->mLinkName << " type " <<
        // gGenFeatureTypeStr[x->mFeatureType] << " size " <<
        // GetSingleFeatureSize(x) << " order " << x->mFeatureOrder
        // 		  << " weight " << x->mWeight << std::endl;
        if (x->mFeatureType >= btGenFeatureType::SphereJoint &&
            x->mFeatureType <= btGenFeatureType::FixedJoint &&
            x->mFeatureOrder == 2)
        {
            dof_controllable_lst[x->mLinkId] = true;
        }
        else
        {
            switch (x->mFeatureType)
            {
            case btGenFeatureType::Height:
                std::cout << "link " << x->mLinkName
                          << " height is controlled, weight = " << x->mWeight
                          << std::endl;
                break;
            case btGenFeatureType::Location:
                std::cout << "link " << x->mLinkName
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
    for (int link_id = 0; link_id < num_of_links; link_id++)
    {
        if (false == dof_controllable_lst[link_id])
        {
            std::cout << "[error] link " << link_id << " "
                      << mModel->GetLinkById(link_id)->GetName()
                      << " is not controlled\n";
            exit(1);
        }
    }
    std::cout << "all accel dof are controlled\n";
    std::cout << "total feature size = " << mTotalFeatureSize << std::endl;
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
            exit(1);
        }
    }
    std::cout << "link " << link_id << " tested succ\n";
    // exit(1);
}

/**
 * \brief				Calculate the formula:
 *
 * 		Feature Vector = C_\alpha * qddot + d_\alpha
 *
 * 	Feature vector is consist of generalized accel or cartesian accel of any
 * link COM.
 */
void btGenFeatureArray::EvalConvertMatAndResidual(tMatrixXd &C_alpha,
                                                  tVectorXd &d_alpha) const
{
    int num_of_freedom = mModel->GetNumOfFreedom();
    C_alpha.noalias() = tMatrixXd::Zero(mTotalFeatureSize, num_of_freedom);
    d_alpha = tVectorXd::Zero(mTotalFeatureSize);
    for (int id = 0; id < mNumOfFeature; id++)
    {
        auto feature = mFeatureVector[id];
        int feature_offset = GetSingleFeatureOffset(id);
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
                C_alpha(feature_offset + i, q_offset + i) = 1.0;
            }
        }
        break;

        case btGenFeatureType::Location:
        {
            const tMatrixXd Jv = link->GetJKv();
            const tVectorXd qdot = mModel->Getqdot();
            const tMatrixXd Jv_dot = link->GetJKv_dot();
            C_alpha.block(feature_offset, 0, feature_size, num_of_freedom) =
                mConvertPosToXZ * Jv;
            d_alpha.segment(feature_offset, feature_size) =
                mConvertPosToXZ * Jv_dot * qdot;
            // const tMatrixXd J link->GetJKv_dot
        }
        break;
        case btGenFeatureType::Height:
        {
            const tMatrixXd Jv = link->GetJKv();
            const tVectorXd qdot = mModel->Getqdot();
            const tMatrixXd Jv_dot = link->GetJKv_dot();
            // std::cout << 'mConvertPosToY = \n' << mConvertPosToY <<
            // std::endl;
            C_alpha.block(feature_offset, 0, feature_size, num_of_freedom) =
                mConvertPosToY * Jv;
            d_alpha.segment(feature_offset, feature_size) =
                mConvertPosToY * Jv_dot * qdot;
        }
        break;
        default:
            std::cout << "[error] Unsupported feature type "
                      << feature->mFeatureType,
                exit(1);
            break;
        }
    }

    // std::cout << "C = " << C_alpha << std::endl;
    // std::cout << "Cd= " << d_alpha.transpose() << std::endl;
    // d_alpha.setZero();
}

/**
 * \brief					Calculate the convert mat/convert residual
 * from the control torque \tau to qddot
 *
 * 		D_\tau = C_alpha * Minv * N
 * 		M_Q = C_alpha * Minv
 * 		n_tau = d_alpha + C_alpha * Minv * (Q_G - C_d * qdot)
 *
 */
void btGenFeatureArray::EvalPrerequistite(tMatrixXd &D_tau, tMatrixXd &M_Q,
                                          tVectorXd &n_tau) const
{
    const tMatrixXd Minv = mModel->GetInvMassMatrix();
    D_tau.noalias() = mC_alpha * Minv * mN;
    M_Q.noalias() = mC_alpha * Minv;

    tVectorXd Q_G = mModel->CalcGenGravity(mGravity);
    n_tau.noalias() =
        md_alpha + mC_alpha * Minv *
                       (Q_G - mModel->GetCoriolisMatrix() * mModel->Getqdot());
    // n_tau.noalias() = md_alpha + mC_alpha * Minv * Q_G - mC_alpha * Minv *
    // mModel->GetCoriolisMatrix() * mModel->Getqdot();
}

/**
 * \brief						Calculate the dynamic terms, matrices E,
 * H and vector f
 *
 * 	E = D_\tau^T * W^T * W * M_\tau + W_m^T * W_m
 *  H = -E^{-1} * D_\tau^T * W^T * W * M_Q
 *  f = W_M^T * W_m * \tilde{\tau} - D_\tau^T * W^T * W *(n_\tau -
 * \tilde{\alpha})
 */
// #include <fstream>
// extern std::string new_path;
void btGenFeatureArray::EvalDynamicTerms(const tVectorXd &ref_feature,
                                         const tVectorXd &ref_tau, tMatrixXd &H,
                                         tMatrixXd &E, tVectorXd &f) const
{
    const tMatrixXd WTW = (mW.cwiseProduct(mW)).asDiagonal();
    const tMatrixXd WmTWm = (mWm.cwiseProduct(mWm)).asDiagonal();
    // std::cout << "WTW = \n"
    // 		  << WTW << std::endl;
    // std::cout << "WmTWm = \n"
    // 		  << WmTWm << std::endl;
    E.noalias() = mD_tau.transpose() * WTW * mD_tau + WmTWm;
    H.noalias() = -E.inverse() * mD_tau.transpose() * WTW * mM_Q;
    f.noalias() = WmTWm * ref_tau - mD_tau.transpose() * WTW * mn_tau +
                  mD_tau.transpose() * WTW * ref_feature;

    // const tVectorXd& ref_tau = mTraj->mActiveForce[target_frame].segment(6,
    // num_of_underactuated_freedom); const tVectorXd& ref_feature =
    // mRefFeature[target_frame];
    // {
    // 	std::cout << "ref tau = " << ref_tau.transpose() << std::endl;
    // 	f.noalias() = WmTWm * ref_tau;
    // 	f = f - mD_tau.transpose() * WTW * mM_Q * (mn_tau - ref_feature);
    // }
    // std::cout << "[eval] WTW = " << WTW.norm() << std::endl;
    // std::cout << "[eval] WTW = " << WTW << std::endl;
    // std::cout << "[eval] WmTWm = " << WmTWm.norm() << std::endl;
    // std::cout << "[eval] Dinv norm = " << (E.inverse()).norm() << std::endl;
    // int num_of_underactuated_freedom = mModel->GetNumOfFreedom() - 6;
    // std::cout << "[eval] C norm = " << (mD_tau.transpose() * WTW).norm() <<
    // std::endl; std::cout << "[eval] Minv norm = " << (mM_Q).norm() <<
    // std::endl; std::cout << "[eval] C norm = " << (mD_tau.transpose() *
    // WTW).norm() << std::endl; std::cout << "[eval] ref tau norm = " <<
    // ref_tau.norm() << std::endl; std::cout << "[eval] ref feature norm = " <<
    // ref_feature.norm() << std::endl; std::cout << "[eval] n_tau norm = " <<
    // mn_tau.norm() << std::endl; std::cout << "[eval] D_tau norm = " <<
    // mD_tau.norm() << std::endl;
    //

    {
        //
        // f.noalias() = WmTWm * ref_tau - mD_tau.transpose() * WTW * (mn_tau -
        // ref_feature);
    } {
        // sometimes, n_tau and ref feature are all quite small. Multiply them
        // dividely can reduce the truncation error the relative error is small
    }

    // tVectorXd f_p1 = WmTWm * ref_tau;
    // tVectorXd f_p2 = -mD_tau.transpose() * WTW * (mn_tau - ref_feature);
    // tVectorXd f_p21 = mn_tau;
    // tVectorXd f_p22 = -mD_tau.transpose() * WTW * mn_tau;
    // tVectorXd f_p23 = mD_tau.transpose() * WTW * ref_feature;
    // tVectorXd f_p24 = -mD_tau.transpose() * WTW * mn_tau + mD_tau.transpose()
    // * WTW * ref_feature;

    // // 22 + 23
    // // std::ofstream fout(new_path, std::ios::app);
    // // fout << "E p1 = \n"
    // // 	 << -E.inverse() << std::endl;
    // // fout << "E p2 = \n"
    // // 	 << mD_tau.transpose() * WTW * mM_Q << std::endl;
    // // fout << "E p3 = \n"
    // // 	 << -E.inverse() * mD_tau.transpose() * WTW * mM_Q << std::endl;
    // // // fout << "f p1 = " << f_p1.transpose() << std::endl;
    // // // fout << "f p2 = " << f_p2.transpose() << std::endl;
    // // // fout << "f p21 = " << f_p21.transpose() << std::endl;
    // // // fout << "f p22 = " << f_p22.transpose() << std::endl;
    // // // fout << "f p23 = " << f_p23.transpose() << std::endl;
    // // // fout << "f p24 = " << f_p24.transpose() << std::endl;
    // // fout.close();
}

/**
 * \brief					Given the gen accel qddot, calculate current
 * target feature vector
 */
tVectorXd btGenFeatureArray::CalcTargetFeature(const tVectorXd &qddot)
{
    // std::cout << "init ref feature begin " << mTraj->mNumOfFrames <<
    // std::endl; whatever feature type is defined, we only control from 1 - N-2
    // mRefFeature.resize(mTraj->mNumOfFrames);
    tVectorXd ref_feature;
    ref_feature = tVectorXd::Zero(mTotalFeatureSize);
    const tVectorXd &qdot = mModel->Getqdot();
    for (int fea_id = 0; fea_id < mNumOfFeature; fea_id++)
    {
        auto feature = mFeatureVector[fea_id];
        int offset = GetSingleFeatureOffset(fea_id);
        int size = GetSingleFeatureSize(feature);

        // only control the accel feature at this moment
        if (feature->mFeatureOrder != 2)
        {
            std::cout << "[error] InitReffeature feature " << fea_id
                      << "is not 2-order(accel), exit\n";
            exit(1);
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
            tVector3d accel = tVector3d::Zero();
            accel += link->GetJKv() * qddot;
            for (int dof = 0; dof < mModel->GetNumOfFreedom(); dof++)
                accel + link->GetTotalDofdJKv_dq(dof) * qdot;

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
                    exit(1);
                break;
            }
        }
        else
        {
            std::cout << "[error] Neither joint nor pos feature, illegal "
                         "feature type "
                      << feature->mFeatureType << std::endl,
                exit(1);
        }
    }

    return ref_feature;
}