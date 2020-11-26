#include "BulletUtil.h"

tVector btBulletUtil::btVectorTotVector0(const btVector3 &v)
{
    return tVector(v[0], v[1], v[2], 0);
}
tVector btBulletUtil::btVectorTotVector1(const btVector3 &v)
{
    return tVector(v[0], v[1], v[2], 1);
}
btVector3 btBulletUtil::tVectorTobtVector(const tVector &t)
{
    return btVector3(t[0], t[1], t[2]);
}
tMatrix btBulletUtil::btMatrixTotMatrix0(const btMatrix3x3 &mat)
{
    tMatrix res = tMatrix::Zero();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res(i, j) = mat[i][j];
    return res;
}
tMatrix btBulletUtil::btMatrixTotMatrix1(const btMatrix3x3 &mat)
{
    tMatrix res = tMatrix::Identity();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res(i, j) = mat[i][j];
    return res;
}
tQuaternion btBulletUtil::btQuaternionTotQuaternion(const btQuaternion &qua)
{
    return tQuaternion(qua.w(), qua.x(), qua.y(), qua.z());
}
btQuaternion btBulletUtil::tQuaternionTobtQuaternion(const tQuaternion &qua)
{
    return btQuaternion(qua.x(), qua.y(), qua.z(), qua.w());
}

Eigen::VectorXd btBulletUtil::btArrayToEigenArray(const btScalar *begin,
                                                  const int size)
{
    Eigen::VectorXd array(size);
    for (int i = 0; i < size; i++)
        array[i] = begin[i];
    return array;
}
Eigen::VectorXd
btBulletUtil::btArrayToEigenArray(const btAlignedObjectArray<btScalar> &info)
{
    Eigen::VectorXd array(info.size());
    for (int i = 0; i < info.size(); i++)
        array[i] = info[i];
    return array;
}
tVector
btBulletUtil::btIDVectorTotVector0(const btInverseDynamicsBullet3::vec3 &vec)
{
    return tVector(vec(0), vec(1), vec(2), 0);
}
Eigen::VectorXd
btBulletUtil::btIDArrayToEigenArray(const btInverseDynamicsBullet3::vecx &vec)
{
    Eigen::VectorXd res(vec.size());
    for (int i = 0; i < res.size(); i++)
        res[i] = vec(i);
    return res;
}
btInverseDynamicsBullet3::vecx
btBulletUtil::EigenArrayTobtIDArray(const tVectorXd &vec)
{
    btInverseDynamicsBullet3::vecx res(vec.size());
    for (int i = 0; i < vec.size(); i++)
        res(i) = vec[i];
    return res;
}
tVector
btBulletUtil::btIDVectorTotVector1(const btInverseDynamicsBullet3::vec3 &vec)
{
    return tVector(vec(0), vec(1), vec(2), 1);
}
tMatrix
btBulletUtil::btIDMatrixTotMatrix(const btInverseDynamicsBullet3::mat33 &mat)
{
    tMatrix res = tMatrix::Identity();
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res(i, j) = mat(i, j);

    return res;
}

btMatrixXu btBulletUtil::tMatrixXdTobtMatrixX(const tMatrixXd &mat)
{
    // TODO: alternative memcpy
    btMatrixXu res(mat.rows(), mat.cols());
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
            res.setElem(i, j, mat(i, j));
    return res;
}

btTransform btBulletUtil::tMatrixTobtTransform(const tMatrix &trans_eigen)
{
    btTransform trans_bt;
    trans_bt.setOrigin(
        btBulletUtil::tVectorTobtVector(trans_eigen.block(0, 3, 4, 1)));
    trans_bt.setRotation(btBulletUtil::tQuaternionTobtQuaternion(
        btMathUtil::RotMatToQuaternion(trans_eigen)));
    return trans_bt;
}
tMatrixXd btBulletUtil::btMatrixXTotMatrixXd(const btMatrixXu &mat)
{
    // TODO: alternative memcpy
    tMatrixXd res(mat.rows(), mat.cols());
    for (int i = 0; i < mat.rows(); i++)
        for (int j = 0; j < mat.cols(); j++)
            res(i, j) = mat(i, j);
    return res;
}

btVectorXu btBulletUtil::tVectorXdTobtVectorXd(const tVectorXd &vec)
{
    btVectorXu res(vec.size());
    for (int i = 0; i < vec.size(); i++)
        res[i] = vec[i];
    return res;
}

tVectorXd btBulletUtil::btVectorXdTotVectorXd(const btVectorXu &vec)
{
    tVectorXd res(vec.size());
    for (int i = 0; i < vec.size(); i++)
        res[i] = vec[i];
    return res;
}

btMatrix3x3 btBulletUtil::AsDiagnoal(const btVector3 &vec)
{
    btMatrix3x3 mat = btMatrix3x3::getIdentity();
    for (int i = 0; i < 3; i++)
        mat[i][i] = vec[i];
    return mat;
}

tMatrix btBulletUtil::btTransformTotMatrix(const btTransform &trans_bt)
{
    tMatrix eigen_trans = tMatrix::Identity();
    eigen_trans.block(0, 3, 3, 1) =
        btBulletUtil::btVectorTotVector0(trans_bt.getOrigin()).segment(0, 3);
    eigen_trans.block(0, 0, 3, 3) =
        btMathUtil::RotMat(
            btBulletUtil::btQuaternionTotQuaternion(trans_bt.getRotation()))
            .block(0, 0, 3, 3);
    return eigen_trans;
}