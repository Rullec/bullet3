#pragma once
#include <Eigen/Dense>
#include <vector>
#define THROW_IF(val) if (val) throw "[error] in " __FUNCTION__;
#define THROW_IF_LOG(val, log) if(val) throw "[error] in " + __FUNCTION__ + log;
// #define M_PI  3.14159265358979323

typedef Eigen::Vector4d tVector;
typedef Eigen::VectorXd tVectorXd;
typedef Eigen::Matrix4d tMatrix;
typedef Eigen::MatrixXd tMatrixXd;
typedef Eigen::Quaterniond tQuaternion;
template <typename T>
using tEigenArr = std::vector<T, Eigen::aligned_allocator<T>>;
typedef tEigenArr<tVector> tVectorArr;

// "XYZ" -> R = Rz * Ry * Rx -> p' = Rp
enum eRotationOrder {
	XYZ = 0,
	ZYX,
};

class cMathUtil {
public:
	static tMatrix RotMat(const tQuaternion & quater);
	static tQuaternion RotMatToQuaternion(const tMatrix &mat);
	static tQuaternion CoefToQuaternion(const tVector & );
	static tQuaternion AxisAngleToQuaternion(const tVector & angvel);
	static tQuaternion EulerAnglesToQuaternion(const tVector & vec, const eRotationOrder & order);
	static tQuaternion MinusQuaternion(const tQuaternion & quad);
	static tVector QuaternionToCoef(const tQuaternion & quater);
	static tVector QuaternionToAxisAngle(const tQuaternion & );
	static tVector CalcAngularVelocity(const tQuaternion & old_rot, const tQuaternion & new_rot, double timestep);
	static tVector CalcAngularVelocityFromAxisAngle(const tQuaternion & old_rot, const tQuaternion & new_rot, double timestep);
	static tVector QuatRotVec(const tQuaternion &, const tVector & vec);
	static tVector QuaternionToEulerAngles(const tQuaternion &, const eRotationOrder & order);
	
	static tMatrix EulerAnglesToRotMat(const tVector & euler, const eRotationOrder & order);
	static tMatrix EulerAnglesToRotMatDot(const tVector & euler, const eRotationOrder & order);
	static tVector AngularVelToqdot(const tVector & omega, const tVector & cur_q, const eRotationOrder & order);
	static tMatrix VectorToSkewMat(const tVector &);
	static tVector SkewMatToVector(const tMatrix &);
	template<typename T>
	static int sign(const T &x) {return (x > 0) ? 1 : ((x < 0) ? -1 : 0);}
};


template <typename T>
void JudgeSameVec(T &a, T & b, const char * log_info, double eps = 1e-5)
{
	THROW_IF_LOG((a - b).norm() < eps, "JudgeSameVec failed");
}