#include "btGenFeatureUtils.h"
#include <iostream>

btGenFeature::btGenFeature()
{
    mLinkId = -1;
    mLinkName = "";
    mFeatureName = "";
    mFeatureType = btGenFeatureType::INVALID_FEATURE_TYPE;
    mFeatureOrder = -1;
    mWeight = 0;
}

/**
 * \brief           Constructor of the single order feature array
 */
tFeatureArraySingleOrder::tFeatureArraySingleOrder(int order)
    : mFeatureOrder(order)
{
    mNumOfFeature = 0;
    mTotalFeatureSize = 0;
    mFeatureVector.clear();
    mFeatureOffset.clear();
    mRefFeature.resize(0);
    mWeight.resize(0);
    mConvertMatFromqToFeature.resize(0, 0);
    mConvertResFromqToFeature.resize(0);
    mConvertMatFromTauToq.resize(0, 0);
    mConvertMatFromContactForceToq.resize(0, 0);
    mConvertResFromForceToq.resize(0);
}

/**
 * \brief           Deconstructor
 */
tFeatureArraySingleOrder::~tFeatureArraySingleOrder()
{
    for (auto &x : mFeatureVector)
        delete x;
}

/**
 * \brief            Add a feature which has a typical order
 */
void tFeatureArraySingleOrder::AddFeature(btGenFeature *f)
{
    if (mFeatureOrder != f->mFeatureOrder)
    {
        std::cout << "[error] Given feature order " << f->mFeatureOrder
                  << " doesn't match with the array property "
                  << this->mFeatureOrder << std::endl;
        exit(0);
    }
    std::cout << "[feature] add link " << f->mLinkName << " order "
              << f->mFeatureOrder << " type " << f->mFeatureType << " weight "
              << f->mWeight << std::endl;
    mTotalFeatureSize += GetSingleFeatureSize(f);
    mNumOfFeature += 1;
    mFeatureVector.push_back(f);
}
void tFeatureArraySingleOrder::Done()
{
    // 1. sort and shape the weight vector
    std::sort(mFeatureVector.begin(), mFeatureVector.end(), FeatureSort);

    mFeatureOffset.resize(mNumOfFeature, 0);
    mWeight.resize(mTotalFeatureSize);

    for (int i = 0, offset = 0; i < mNumOfFeature; i++)
    {
        int size = GetSingleFeatureSize(mFeatureVector[i]);
        mFeatureOffset[i] = offset;
        mWeight.segment(offset, size).fill(mFeatureVector[i]->mWeight);
        offset += GetSingleFeatureSize(mFeatureVector[i]);

        // 2.shape the weight vector
        if (mFeatureOrder != mFeatureVector[i]->mFeatureOrder)
        {
            std::cout << "[error] in feature array " << this->mFeatureOrder
                      << " ";
            std::cout << "order " << mFeatureVector[i]->mFeatureOrder
                      << " weight = " << mWeight.transpose() << std::endl;
            exit(1);
        }
    }
}

int GetSingleFeatureSize(btGenFeature *feature)
{
    if (btGenFeatureType::INVALID_FEATURE_TYPE == feature->mFeatureType ||
        feature->mFeatureType >= btGenFeatureType::NUM_OF_GEN_FEATURE_TYPE)
    {
        std::cout << "[error] GetSingleFeatureSize: invalid feature type\n";
        exit(0);
    }
    return gGenFeatureSize[feature->mFeatureType];
}

bool FeatureSort(const btGenFeature *f1, const btGenFeature *f2)
{
    if (f1->mLinkId != f2->mLinkId)
        return f1->mLinkId < f2->mLinkId;
    else if (f1->mFeatureOrder != f2->mFeatureOrder)
        return f1->mFeatureOrder < f2->mFeatureOrder;
    else if (f1->mFeatureType != f2->mFeatureType)
        return f1->mFeatureType < f2->mFeatureType;
    else
    {
        std::cout
            << "[error] illegal feature pair when sorting: the same link id "
            << f1->mLinkId << " and the same feature order "
            << f1->mFeatureOrder << ", the same feature type "
            << f1->mFeatureType << std::endl;
        exit(0);
    }
}

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

std::string GetFeatureSuffix(const std::string &name)
{
    int st = name.find("_");
    if (st == -1)
    {
        std::cout << "[error] GetFeatureSuffix _ not found in " << name
                  << std::endl;
        exit(0);
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
        exit(0);
    }
    return name.substr(0, st);
}

/**
 * \brief               Given an json value, parse all of the feature order ->
 * feature weight map from it
 * \return the map
 */
std::map<int, double> GetFeatureOrderAndWeight(const Json::Value &value)
{
    std::map<int, double> order_weight_map;
    order_weight_map.clear();
    Json::Value::Members members = value.getMemberNames();
    for (Json::Value::Members::iterator it = members.begin();
         it != members.end(); it++)
    {
        std::string order_str = *it;
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
            exit(0);
        }

        double weight = btJsonUtil::ParseAsDouble(order_str, value);

        // if there are duplicate keys, raise an error
        if (order_weight_map.find(order) != order_weight_map.end())
        {
            std::cout << "duplicate feature key in json value : " << value
                      << std::endl;
            exit(0);
        }
        else
        {
            order_weight_map[order] = weight;
        }
    }
    return order_weight_map;
}