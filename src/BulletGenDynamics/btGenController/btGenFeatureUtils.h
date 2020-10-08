#ifndef __BT_GEN_FEATURE_UTILS_HPP__
#define __BT_GEN_FEATURE_UTILS_HPP__
#include "btGenFeature.h"

enum btGenFeatureType
{
    INVALID_FEATURE_TYPE,
    SphereJoint,
    RevoluteJoint,
    NoneJoint,
    FixedJoint,
    Height,
    Location,
    NUM_OF_GEN_FEATURE_TYPE
};

const std::string gGenFeatureTypeStr[] = {
    "INVALID_FEATURE_TYPE",
    "SphereJoint",
    "RevoluteJoint",
    "NoneJoint",
    "FixedJoint",
    "Height",   // Y axis
    "Location", // X-Z axis
};

const int gGenFeatureSize[] = {-1, 3, 1, 6, 0, 1, 2};

/**
 * \brief			A btGenFeature is a consist element in the total
 * feature vector.
 *
 * 		It can be:
 * 			1. joint angle
 * 			2. link location
 * 			3. link height
 * 		And we can also control the
 */
struct btGenFeature
{
    btGenFeature();
    int mLinkId;           // the belonging link of this feature
    std::string mLinkName; // link name is also dummy specified
    std::string
        mFeatureName; // the name of this feature. format: linkname + suffix
    btGenFeatureType mFeatureType; //
    int mFeatureOrder;             // order: 0-pos, 1-vel, 2-accel
    double mWeight;                // feature weight
};

/**
 * \brief           Manage one whole order of feature arrays, this array is
 * literally a group of feature which share the same order.
 */
struct tFeatureArraySingleOrder
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    tFeatureArraySingleOrder(int order);
    ~tFeatureArraySingleOrder();
    void AddFeature(btGenFeature *);
    void Done();
    const int mFeatureOrder; // the order of feature, can be 0, 1, 2 which
                             // stand for pos, vel, accel order of features
    int mNumOfFeature;       // the number of feature in this order
    int mTotalFeatureSize;   // the size of feature in this order
    std::vector<btGenFeature *> mFeatureVector;
    std::vector<int> mFeatureOffset;
    tVectorXd mRefFeature; // reference feature, given by the
    tVectorXd mWeight;     //

    // q/qdot/qddot = mConvertMatFromTauToq * control_force + mConvertMatFromContactForceToq * Q_chi + mConvertResFromForceToq
    tMatrixXd mConvertMatFromTauToq;
    tMatrixXd mConvertMatFromContactForceToq;
    tVectorXd mConvertResFromForceToq;

    // feature = mConvertMatFromqToFeature * q/qdot/qddot + mConvertResFromqToFeature
    tMatrixXd mConvertMatFromqToFeature;
    tVectorXd mConvertResFromqToFeature;
};

// other utils
int GetSingleFeatureSize(btGenFeature *feature);
bool FeatureSort(const btGenFeature *f1, const btGenFeature *f2);
bool IsJointFeature(btGenFeatureType type);
bool IsPosFeature(btGenFeatureType type);
std::string GetFeatureSuffix(const std::string &name);
std::string GetFeatureLinkName(const std::string &name);
std::map<int, double> GetFeatureOrderAndWeight(const Json::Value &value);
#endif