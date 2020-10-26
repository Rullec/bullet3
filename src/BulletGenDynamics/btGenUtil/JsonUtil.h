#pragma once
// #include "LogUtil.hpp"
#include "MathUtil.h"
#include "json/json.h"
#include <string>

namespace Json
{
};
class btJsonUtil
{
public:
    // static std::string BuildVectorJson(const tVector &vec);
    // static bool ReadVectorJson(const Json::Value &root, tVector &out_vec);
    // static std::string BuildVectorJson(const Eigen::VectorXd &vec);
    // static std::string BuildVectorString(const Eigen::VectorXd &vec);
    // static bool ReadVectorJson(const Json::Value &root,
    //                            Eigen::VectorXd &out_vec);
    static bool LoadJson(const std::string &path, Json::Value &value);
    static bool WriteJson(const std::string &path, Json::Value &value,
                          bool indent = true);
    static bool ReadVectorJson(const Json::Value &root,
                               Eigen::VectorXd &out_vec);
    static Eigen::VectorXd ReadVectorJson(const Json::Value &root);
    static int ParseAsInt(const std::string &data_field_name,
                          const Json::Value &root);
    static std::string ParseAsString(const std::string &data_field_name,
                                     const Json::Value &root);
    static double ParseAsDouble(const std::string &data_field_name,
                                const Json::Value &root);
    static float ParseAsFloat(const std::string &data_field_name,
                              const Json::Value &root);
    static bool ParseAsBool(const std::string &data_field_name,
                            const Json::Value &root);
    static Json::Value ParseAsValue(const std::string &data_field_name,
                                    const Json::Value &root);
    static Json::Value BuildVectorJsonValue(const tVectorXd &vec);
    static Json::Value BuildMatrixJsonValue(const tMatrixXd &vec);

private:
    // static tLogger mLogger;
};
