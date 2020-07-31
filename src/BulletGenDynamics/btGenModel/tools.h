#pragma once
#include <vector>
#include <string>
#include <regex>
#include "ModelEigenUtils.h"


class Tools {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	static tVector3d ConvertUnit(tVector3d t) {
		//return t * 0.0254; // 1 unit = 0.05644444 m (2.54cm/0.45inch)
		return t * 0.0564;

	}

	static double ConvertUnit(double d) {
		//return d * 0.0254;
		return d * 0.0564;	// 1/0.0564
	}

	static double ConvertUnitInv(double d) {
		return d * 17.7305;
	}

	static double DegToRad(double d) {
		//while (d > 180.0) d -= 360.;
		//while (d < -180.0) d += 360.;
		return d / 180.0 * M_PI;
	}

	static double RadToDeg(double r) {
		//while (r > M_PI) r -= 2 * M_PI;
		//while (r < -M_PI) r += 2 * M_PI;
		return r / M_PI * 180.0;
	}

	static tVector3d RadToDeg(tVector3d angle) {
		return angle / M_PI * 180;
	}

	static std::vector<std::string> split(const std::string& input, const std::string& regex) {
		// passing -1 as the submatch index parameter performs splitting
		std::regex re(regex);
		std::sregex_token_iterator
			first{ input.begin(), input.end(), re, -1 },
			last;
		return { first, last };
	}

	static tVector3d RotationMatrix2EulerAngles(tMatrix3d rot) {
		// first x->then y->then z
		double theta_x = atan2(rot(2, 1), rot(2, 2));
		double theta_y = atan2(-rot(2, 0), sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
		double theta_z = atan2(rot(1, 0), rot(0, 0));
		return tVector3d(theta_x, theta_y, theta_z);
	}

	static tMatrix3d EulerAngles2RotationMatrix(tVector3d & euler) {
		// rotation order : x->y->z
		assert(euler.size() == 3);
		Eigen::Quaterniond qua = EulerAngles2Quaternion(euler);
		tMatrix3d rot = qua.toRotationMatrix();
		return rot;
	}

	// from EulerAngles to Quaternion
	static Eigen::Quaterniond EulerAngles2Quaternion(const tVector3d & euler) {
		/* angles[0, 1, 2] - x, y, z*/
		// rotation order : first x, then y, then z
		Eigen::AngleAxisd ax = Eigen::AngleAxisd(euler(0), tVector3d::UnitX()),
			ay = Eigen::AngleAxisd(euler(1), tVector3d::UnitY()),
			az = Eigen::AngleAxisd(euler(2), tVector3d::UnitZ());

		return az * ay * ax;
	}

	// from Rotation Matrix to Quaternion, added by Xudong
	static Eigen::Quaterniond RotationMatrix2Quaternion(const tMatrix3d& mat)
	{
		double tr = mat(0, 0) + mat(1, 1) + mat(2, 2);
		Eigen::Quaterniond q;
		if (tr > 0) {
			double S = sqrt(tr + 1.0) * 2; // S=4*qw 
			q.w() = 0.25 * S;
			q.x() = (mat(2, 1) - mat(1, 2)) / S;
			q.y() = (mat(0, 2) - mat(2, 0)) / S;
			q.z() = (mat(1, 0) - mat(0, 1)) / S;
		}
		else if ((mat(0, 0) > mat(1, 1) && (mat(0, 0) > mat(2, 2)))) {
			double S = sqrt(1.0 + mat(0, 0) - mat(1, 1) - mat(2, 2)) * 2; // S=4*qx 
			q.w() = (mat(2, 1) - mat(1, 2)) / S;
			q.x() = 0.25 * S;
			q.y() = (mat(0, 1) + mat(1, 0)) / S;
			q.z() = (mat(0, 2) + mat(2, 0)) / S;
		}
		else if (mat(1, 1) > mat(2, 2)) {
			double S = sqrt(1.0 + mat(1, 1) - mat(0, 0) - mat(2, 2)) * 2; // S=4*qy
			q.w() = (mat(0, 2) - mat(2, 0)) / S;
			q.x() = (mat(0, 1) + mat(1, 0)) / S;
			q.y() = 0.25 * S;
			q.z() = (mat(1, 2) + mat(2, 1)) / S;
		}
		else {
			double S = sqrt(1.0 + mat(2, 2) - mat(0, 0) - mat(1, 1)) * 2; // S=4*qz
			q.w() = (mat(1, 0) - mat(0, 1)) / S;
			q.x() = (mat(0, 2) + mat(2, 0)) / S;
			q.y() = (mat(1, 2) + mat(2, 1)) / S;
			q.z() = 0.25 * S;
		}

		return q;
	}

	// Calculate angular velocity from 2 quaternions
	static tVector3d CalcQuaternionVel(const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1, double dt)
	{
		Eigen::Quaterniond q_diff = q1 * q0.conjugate();
		tVector3d axis;
		double theta;
		QuaternionToAxisAngle(q_diff, axis, theta);
		return (theta / dt) * axis;
	}

	// Convert Quaternion to Axis Angle (tVector3d)
	static void QuaternionToAxisAngle(const Eigen::Quaterniond& q, tVector3d & out_axis, double& out_theta)
	{
		out_theta = 0;
		out_axis = tVector3d(0, 0, 1);

		Eigen::Quaterniond q1 = q;
		if (q1.w() > 1)
		{
			q1.normalize();
		}

		double sin_theta = std::sqrt(1 - q1.w() * q1.w());
		if (sin_theta > 0.000001)
		{
			out_theta = 2 * std::acos(q1.w());
			out_theta = Tools::ClipAngle(out_theta);
			out_axis = tVector3d(q1.x(), q1.y(), q1.z()) / sin_theta;
		}
	}

	static int Clip(int d, int lb, int ub) {
		if (d <= lb) d = lb;
		if (d > ub) d = ub;
		return d;
 	}

	static double Clip(double d, double lb, double ub) {
		if (d <= lb) d = lb;
		if (d > ub) d = ub;
		return d;
	}

	static tVector3d GettVector3d(tVector v) {
		return tVector3d(v[0], v[1], v[2]);
	}

	static tVector GettVector(tVector3d v) {
		return tVector(v[0], v[1], v[2], 1);
	}

	static tVector GettVector(tVector3d v, double w) {
		return tVector(v[0], v[1], v[2], w);
	}

	static void SkewMatrix(const tVector3d & w, tMatrix3d & result) {
		result.setZero();
		result.data()[1] = w.data()[2];
		result.data()[2] = -w.data()[1];
		result.data()[5] = w.data()[0];
		tMatrix3d temp = result.transpose();
		result -= temp;
	}

	static tMatrix3d SkewMatrix(const tVector3d & w)
	{
		tMatrix3d skew_mat;
		SkewMatrix(w, skew_mat);
		return skew_mat;
	}

	static void SkewVector(tVector3d& result, tMatrix3d& w) {
		tMatrix3d A = (w - w.transpose()) / 2;
		result.setZero();
		result.data()[0] = A.data()[5];
		result.data()[1] = -A.data()[2];
		result.data()[2] = A.data()[1];
	}

	static tVector3d FromSkewSymmetric(const tMatrix3d& R) {
		tVector3d temp;
		temp.data()[0] = R.data()[5];
		temp.data()[1] = R.data()[6];
		temp.data()[2] = R.data()[1];
		return temp;
	}

	static double ClipAngle(double a) {
		while (a < -M_PI) a += 2 * M_PI;
		while (a > M_PI)  a -= 2 * M_PI;
		return a;
	}

	// static void Multiply3x3(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r);
	// static void Multiply4x4(tMatrix& a, tMatrix& b, tMatrix& r);

	// static void SSE3x3v1(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r);
	// static void SSE3x3v2(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r);

	// static void AVX3x3v1(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r);
	// static void AVX3x3v2(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r);
	// static void AVX3x3v3(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r);
	// static void AVX3x3v4(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r);
	// static void AVX3x3v5(tMatrix3d& a, tMatrix3d& b, tMatrix3d& r);
	static void AVX4x4v1(const tMatrix& a, const tMatrix& b, tMatrix& r);
	static void AVX4x4v1_3mat(const tMatrix& a, const tMatrix& b, const tMatrix& c, tMatrix& r);
	static void AVX4x4v1_6mat(const tMatrix& a, const tMatrix& b, const tMatrix& c, const tMatrix& d, const tMatrix& e, const tMatrix& f, tMatrix& r);

	static void MatMul4x1(const tMatrix& m, const tVector& v, tVector& r);
	
	static tMatrix t, t1, t2;
};
