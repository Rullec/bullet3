#pragma once
#include "../Extras/InverseDynamics/btMultiBodyTreeCreator.hpp"
#include "BulletInverseDynamics/IDConfig.hpp"
#include "LinearMath/btMatrix3x3.h"
#include "MathUtil.h"
#include <iostream>

class btBulletUtil
{
public:
    static tVector btVectorTotVector0(const btVector3 &v);
    static tVector btVectorTotVector1(const btVector3 &v);
    static btVector3 tVectorTobtVector(const tVector &t);
    static tMatrix btMatrixTotMatrix0(const btMatrix3x3 &mat);
    static tMatrix btMatrixTotMatrix1(const btMatrix3x3 &mat);
    static tQuaternion btQuaternionTotQuaternion(const btQuaternion &qua);
    static btQuaternion tQuaternionTobtQuaternion(const tQuaternion &qua);

    static Eigen::VectorXd btArrayToEigenArray(const btScalar *begin,
                                               const int size);
    static Eigen::VectorXd
    btArrayToEigenArray(const btAlignedObjectArray<btScalar> &info);
    static tVector
    btIDVectorTotVector0(const btInverseDynamicsBullet3::vec3 &vec);
    static Eigen::VectorXd
    btIDArrayToEigenArray(const btInverseDynamicsBullet3::vecx &vec);
    static btInverseDynamicsBullet3::vecx
    EigenArrayTobtIDArray(const tVectorXd &vec);
    static tVector
    btIDVectorTotVector1(const btInverseDynamicsBullet3::vec3 &vec);
    static tMatrix
    btIDMatrixTotMatrix(const btInverseDynamicsBullet3::mat33 &mat);
    static btMatrixXu tMatrixXdTobtMatrixX(const tMatrixXd &mat);
    static btTransform tMatrixTobtTransform(const tMatrix &trans_eigen);
    static tMatrix btTransformTotMatrix(const btTransform &trans_bt);
    static tMatrixXd btMatrixXTotMatrixXd(const btMatrixXu &mat);

    static btVectorXu tVectorXdTobtVectorXd(const tVectorXd &vec);

    static tVectorXd btVectorXdTotVectorXd(const btVectorXu &vec);
    static btMatrix3x3 AsDiagnoal(const btVector3 &vec);

    template <typename T> static void PrintBulletVars(const T &mat)
    {
        int rows = mat.rows(), cols = mat.cols();
        if (typeid(T) == typeid(btVectorXu))
        {
            cols = rows;
            rows = 1;
        }
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                std::cout << mat(i, j) << " ";
            }
            std::cout << std::endl;
        }
    }
};