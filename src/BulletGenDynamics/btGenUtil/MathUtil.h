#ifndef BTMATHUTIL_H_
#define BTMATHUTIL_H_
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#define THROW_IF(val)                                                          \
    if (val)                                                                   \
        throw "[error] in " __FUNCTION__;
#define THROW_IF_LOG(val, log)                                                 \
    if (val)                                                                   \
        throw "[error] in " + __FUNCTION__ + log;
// #define M_PI  3.14159265358979323

typedef Eigen::Vector4d tVector;
typedef Eigen::VectorXd tVectorXd;
typedef Eigen::Matrix4d tMatrix;
typedef Eigen::Matrix4f tMatrix4f;
typedef Eigen::MatrixXd tMatrixXd;
typedef Eigen::Vector3d tVector3d;
typedef Eigen::Vector3f tVector3f;
typedef Eigen::Matrix3d tMatrix3d;
typedef Eigen::Quaterniond tQuaternion;
typedef Eigen::Affine3d aff3;
typedef Eigen::Affine3f aff3f;

template <typename T>
using tEigenArr = std::vector<T, Eigen::aligned_allocator<T>>;
typedef tEigenArr<tVector> tVectorArr;

// "XYZ" -> R = Rz * Ry * Rx -> p' = Rp
enum btRotationOrder
{
    bt_XYZ = 0,
    bt_ZYX,
};

class btMathUtil
{
public:
    static tMatrix RotMat(const tQuaternion &quater);
    static tQuaternion RotMatToQuaternion(const tMatrix &mat);
    static tQuaternion CoefToQuaternion(const tVector &);
    static tQuaternion AxisAngleToQuaternion(const tVector &angvel);
    static tQuaternion EulerAnglesToQuaternion(const tVector &vec,
                                               const btRotationOrder &order);
    static tQuaternion MinusQuaternion(const tQuaternion &quad);
    static tVector QuaternionToCoef(const tQuaternion &quater);
    static tVector QuaternionToAxisAngle(const tQuaternion &);
    static tVector CalcAngularVelocity(const tQuaternion &old_rot,
                                       const tQuaternion &new_rot,
                                       double timestep);
    static tVector CalcAngularVelocityFromAxisAngle(const tQuaternion &old_rot,
                                                    const tQuaternion &new_rot,
                                                    double timestep);
    static tVector QuatRotVec(const tQuaternion &, const tVector &vec);
    static tVector QuaternionToEulerAngles(const tQuaternion &,
                                           const btRotationOrder &order);

    static tMatrix EulerAnglesToRotMat(const tVector &euler,
                                       const btRotationOrder &order);
    static tMatrix EulerAnglesToRotMatDot(const tVector &euler,
                                          const btRotationOrder &order);
    static tVector AngularVelToqdot(const tVector &omega, const tVector &cur_q,
                                    const btRotationOrder &order);
    static tMatrix VectorToSkewMat(const tVector &);
    template <typename T> static tVector SkewMatToVector(const T &mat)
    {
        // verify mat is a skew matrix
        assert((mat + mat.transpose()).norm() < 1e-10);

        // squeeze a mat to a vector
        tVector res = tVector::Zero();
        res[0] = mat(2, 1);
        res[1] = mat(0, 2);
        res[2] = mat(1, 0);
        return res;
    }
    static tMatrixXd ExpandFrictionCone(int num_friction_dirs,
                                        const tVector &normal);
    static tMatrix InverseTransform(const tMatrix &);
    static tMatrix DirToRotMat(const tVector &dir, const tVector &up);
    static double CalcConditionNumber(const tMatrixXd &mat);
    static tMatrixXd JacobPreconditioner(const tMatrixXd &mat);
    static bool IsHomogeneousPos(const tVector &pos, bool exit_if_not = true);
    // static void RoundZero(tMatrixXd &mat, double threshold = 1e-10);

    template <typename T>
    static void RoundZero(T &mat, double threshold = 1e-10)
    {
        mat = (threshold < mat.array().abs()).select(mat, 0.0f);
    }
    template <typename T> static tVector Expand(const T &vec, double n)
    {
        return tVector(vec[0], vec[1], vec[2], n);
    }

    template <typename T> static tMatrix ExpandMat(const T &mat)
    {
        tMatrix tmp = tMatrix::Zero();
        tmp.block(0, 0, 3, 3) = mat.block(0, 0, 3, 3);
        return tmp;
    }

    template <typename T> static int sign(const T &x)
    {
        return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
    }
    template <typename T> static void Swap(T &a, T &b)
    {
        T tmp = a;
        a = b;
        b = tmp;
    }
};
#endif