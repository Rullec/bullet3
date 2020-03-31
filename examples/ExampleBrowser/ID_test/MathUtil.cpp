#include "MathUtil.h"
#include <iostream>
//enum eRotationOrder order = eRotationOrder::XYZ;

// 0 order
tMatrix xconventionRotation(double x);
tMatrix yconventionRotation(double y);
tMatrix zconventionRotation(double z);

// 1 order
tMatrix xconventionRotation_dx(double x);
tMatrix yconventionRotation_dy(double y);
tMatrix zconventionRotation_dz(double z);

tMatrix cMathUtil::RotMat(const tQuaternion & quater_)
{
	// https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix

	tMatrix res = tMatrix::Zero();
	double w = quater_.w(), x = quater_.x(), y = quater_.y(), z = quater_.z();
	res << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0,
		2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0,
		2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0,
		0, 0, 0, 1;
	return res;
}

tQuaternion cMathUtil::RotMatToQuaternion(const tMatrix &mat)
{
	// http://www.iri.upc.edu/files/scidoc/2068-Accurate-Computation-of-Quaternions-from-Rotation-Matrices.pdf
	double eta = 0;
	double q1, q2, q3, q4;	// = [w, x, y, z]

	// determine q1
	{
		double detect_value = mat(0, 0) + mat(1, 1) + mat(2, 2);
		if (detect_value > eta)
		{
			q1 = 0.5 * std::sqrt(1 + detect_value);
		}
		else
		{
			double numerator = 0;
			numerator += std::pow(mat(2, 1) - mat(1, 2), 2);
			numerator += std::pow(mat(0, 2) - mat(2, 0), 2);
			numerator += std::pow(mat(1, 0) - mat(0, 1), 2);
			q1 = 0.5 *  std::sqrt(numerator / (3 - detect_value));
		}
	}

	// determine q2
	{
		double detect_value = mat(0, 0) - mat(1, 1) - mat(2, 2);
		if (detect_value > eta)
		{
			q2 = 0.5 * std::sqrt(1 + detect_value);
		}
		else
		{
			double numerator = 0;
			numerator += std::pow(mat(2, 1) - mat(1, 2), 2);
			numerator += std::pow(mat(0, 1) + mat(1, 0), 2);
			numerator += std::pow(mat(2, 0) + mat(0, 2), 2);
			q2 = 0.5 * std::sqrt(numerator / (3 - detect_value));
		}
	}

	// determine q3
	{
		double detect_value = -mat(0, 0) + mat(1, 1) - mat(2, 2);
		if (detect_value > eta)
		{
			q3 = 0.5 * std::sqrt(1 + detect_value);
		}
		else
		{
			double numerator = 0;
			numerator += std::pow(mat(0, 2) - mat(2, 0), 2);
			numerator += std::pow(mat(0, 1) + mat(1, 0), 2);
			numerator += std::pow(mat(1, 2) + mat(2, 1), 2);
			q3 = 0.5 * std::sqrt(numerator / (3 - detect_value));
		}
	}

	// determine q4
	{
		double detect_value = -mat(0, 0) - mat(1, 1) + mat(2, 2);
		if (detect_value > eta)
		{
			q4 = 0.5 * std::sqrt(1 + detect_value);
		}
		else
		{
			double numerator = 0;
			numerator += std::pow(mat(1, 0) - mat(0, 1), 2);
			numerator += std::pow(mat(2, 0) + mat(0, 2), 2);
			numerator += std::pow(mat(2, 1) + mat(1, 2), 2);
			q4 = 0.5 * std::sqrt(numerator / (3 - detect_value));
		}
	}

	// shape sign
	
	int sign[4] = {};
	sign[0] = cMathUtil::sign(q1);
	sign[1] = cMathUtil::sign(mat(2, 1) - mat(1, 2));
	sign[2] = cMathUtil::sign(mat(0, 2) - mat(2, 0));
	sign[3] = cMathUtil::sign(mat(1, 0) - mat(0, 1));
	if(sign[0]<0) for(int i=1; i<4; i++) sign[i] *= -1;

	return tQuaternion(sign[0] * q1, sign[1] * q2, sign[2] * q3, sign[3] * q4);
}

tVector cMathUtil::QuaternionToCoef(const tQuaternion & quater)
{
	// quaternion -> vec = [x, y, z, w]
	return tVector(quater.x(), quater.y(), quater.z(), quater.w());
}

tQuaternion cMathUtil::CoefToQuaternion(const tVector & vec)
{
	// vec = [x, y, z, w] -> quaternion
	if(vec[3] > 0) return tQuaternion(vec[3], vec[0], vec[1], vec[2]);
	else return tQuaternion(-vec[3], -vec[0], -vec[1], -vec[2]);
}

tQuaternion cMathUtil::AxisAngleToQuaternion(const tVector & angvel)
{
	double theta = angvel.norm();
	double theta_2 = theta / 2;
	double cos_theta_2 = std::cos(theta_2),
		sin_theta_2 = std::sin(theta_2);
	
	tVector norm_angvel = angvel.normalized();
	return tQuaternion(cos_theta_2, norm_angvel[0] * sin_theta_2,\
		norm_angvel[1] * sin_theta_2,\
		norm_angvel[2] * sin_theta_2);
}

tVector cMathUtil::QuaternionToAxisAngle(const tQuaternion & quater)
{
	/* 	quater = [w, x, y, z]
			w = cos(theta / 2)
			x = ax * sin(theta/2)
			y = ay * sin(theta/2)
			z = az * sin(theta/2)
		axis angle = theta * [ax, ay, az, 0]
	*/
	tVector axis_angle = tVector::Zero();

	double theta = 2 * std::acos(quater.w());

	if (theta < 1e-4) return tVector::Zero();

	//std::cout << theta << " " << std::sin(theta / 2) << std::endl;
	double ax = quater.x() / std::sin(theta / 2),
		ay = quater.y() / std::sin(theta / 2),
		az = quater.z() / std::sin(theta / 2);
	return theta * tVector(ax, ay, az, 0);
}

tVector cMathUtil::CalcAngularVelocity(const tQuaternion & old_rot,\
	const tQuaternion & new_rot, double timestep)
{
	tQuaternion trans = new_rot * old_rot.conjugate();
	double theta = std::acos(trans.w()) * 2;	// std::acos() output range [0, pi]
	if (theta > 2 * M_PI - theta)
	{
		// theta = theta - 2*pi
		theta = theta - 2 * M_PI;	// -pi - pi
		trans.coeffs().segment(0, 3) *= -1;
	}
	else if (std::abs(theta) < 1e-10)
	{
		return tVector::Zero();
	}
	tVector vel = tVector::Zero();
	double coef = theta / (sin(theta / 2) * timestep);
	vel.segment(0, 3) = trans.coeffs().segment(0, 3) * coef;
	return vel;
}

tVector cMathUtil::CalcAngularVelocityFromAxisAngle(const tQuaternion & old_rot, const tQuaternion & new_rot, double timestep)
{
	std::cout << "cMathUtil::CalcAngularVelocityFromAxisAngle: this func hasn't been well-tested, call another one\n";
	exit(1);
	tVector old_aa = cMathUtil::QuaternionToAxisAngle(old_rot),
		new_aa = cMathUtil::QuaternionToAxisAngle(new_rot);
	return (new_aa - old_aa) / timestep;
}

tVector cMathUtil::QuatRotVec(const tQuaternion & quater, const tVector & vec)
{
	tVector res = tVector::Zero();
	res.segment(0, 3) = quater * vec.segment(0, 3);
	return res;
}

tVector cMathUtil::QuaternionToEulerAngles(const tQuaternion & q, const eRotationOrder & order)
{
	tVector res = tVector::Zero();
	double w = q.w(),
		x = q.x(),
		y = q.y(),
		z = q.z();

	// please check the note for details
	if (order == eRotationOrder::XYZ)
	{
		res[0] = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
		res[1] = std::asin(2 * (w * y - z * x));
		res[2] = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
	}
	else if (order == eRotationOrder::ZYX)
	{
		res[0] = std::atan2(2 * (w * x - y * z), 1 - 2 * (x * x + y * y));
		res[1] = std::asin(2 * (w * y + z * x));
		res[2] = std::atan2(2 * (w * z - x * y), 1 - 2 * (y * y + z * z));
	}
	else
	{
		std::cout << "[error] tVector cMathUtil::QuaternionToEulerAngles Unsupported rotation order = " << order;
		exit(1);
	}
	return res;
}

tQuaternion cMathUtil::EulerAnglesToQuaternion(const tVector & vec, const eRotationOrder & order)
{
	tQuaternion q[3];
	for (int i = 0; i < 3; i++)
	{
		tVector axis = tVector::Zero();
		axis[i] = 1.0;

		double theta_2 = vec[i] / 2.0;
		axis = axis * std::sin(theta_2);
		axis[3] = std::cos(theta_2);

		q[i] = tQuaternion(axis[3], axis[0], axis[1], axis[2]);
	}
	
	tQuaternion res;
	if (order == eRotationOrder::XYZ)
	{
		res = q[2] * q[1] * q[0];
	}
	else if (order == eRotationOrder::ZYX)
	{
		res = q[0] * q[1] * q[2];
	}

	if (res.w() < 0)res = cMathUtil::MinusQuaternion(res);
	return res;
}

tQuaternion cMathUtil::MinusQuaternion(const tQuaternion & quad)
{
	return tQuaternion(-quad.w(), -quad.x(), -quad.y(), -quad.z());
}

tMatrix cMathUtil::EulerAnglesToRotMat(const tVector & euler, const eRotationOrder & order)
{
	// input euler angles: the rotation theta from parent to local
	// output rot mat: a rot mat that can convert a vector FROM LOCAL FRAME TO PARENT FRAME
	double x = euler[0],
		y = euler[1],
		z = euler[2];
	tMatrix mat = tMatrix::Identity();
	if (order == eRotationOrder::XYZ)
	{
		tMatrix x_mat, y_mat, z_mat;
		x_mat = xconventionRotation(x);
		y_mat = yconventionRotation(y);
		z_mat = zconventionRotation(z);
		mat = z_mat * y_mat * x_mat;
	}
	else if (order == eRotationOrder::ZYX)
	{
		tMatrix x_mat, y_mat, z_mat;
		x_mat = xconventionRotation(x);
		y_mat = yconventionRotation(y);
		z_mat = zconventionRotation(z);
		mat = x_mat * y_mat * z_mat;
	}
	else
	{
		std::cout << "[error] cMathUtil::EulerAnglesToRotMat(const tVector& euler): Unsupported rotation order" << std::endl;
		exit(1);
	}
	return mat;
}

tMatrix cMathUtil::EulerAnglesToRotMatDot(const tVector & euler, const eRotationOrder & order)
{
	double x = euler[0], y = euler[1], z = euler[2];
	tMatrix mat = tMatrix::Identity();
	if (order == eRotationOrder::XYZ)
	{
		tMatrix Rz = zconventionRotation(z), Ry = yconventionRotation(y), Rx = xconventionRotation(x);
		tMatrix Rz_dot = zconventionRotation_dz(z), Ry_dot = yconventionRotation_dy(y), Rx_dot = xconventionRotation_dx(x);
		mat = Rz * Ry * Rx_dot + Rz_dot * Ry * Rx + Rz * Ry_dot * Rx;
	}
	else if (order == eRotationOrder::ZYX)
	{
		tMatrix Rz = zconventionRotation(z), Ry = yconventionRotation(y), Rx = xconventionRotation(x);
		tMatrix Rz_dot = zconventionRotation_dz(z), Ry_dot = yconventionRotation_dy(y), Rx_dot = xconventionRotation_dx(x);
		mat = Rx * Ry * Rz_dot + Rx_dot * Ry * Rz + Rx * Ry_dot * Rz;
	}
	else
	{
		std::cout << "[error] cMathUtil::EulerAnglesToRotMatDot(const tVector& euler): Unsupported rotation order" << std::endl;
		exit(1);
	}
	return mat;
}

tVector cMathUtil::AngularVelToqdot(const tVector & omega, const tVector & cur_q, const eRotationOrder & order)
{
	// w = Jw * q'
	// q' = (Jw)^{-1} * omega
	//[w] = R' * R^T

	// step1: get Jw
	// please read P8 formula (30) in C.K Liu's tutorial "A Quick Tutorial on Multibody Dynamics" for more details
	double x = cur_q[0], y = cur_q[1], z = cur_q[2];
	tMatrix Rx = xconventionRotation(x), Ry = yconventionRotation(y), Rz = zconventionRotation(z);
	tMatrix Rx_dotx = xconventionRotation_dx(x),
			Ry_doty = yconventionRotation_dy(y),
			Rz_dotz = zconventionRotation_dz(z);

	if (order == eRotationOrder::XYZ)
	{
		tMatrix R = Rz * Ry * Rx;
		tMatrix dR_dx = Rz * Ry * Rx_dotx,
				dR_dy = Rz * Ry_doty * Rx,
				dR_dz = Rz_dotz * Ry * Rx;
		tMatrix x_col_mat = dR_dx * R.transpose(),
				y_col_mat = dR_dy * R.transpose(),
				z_col_mat = dR_dz * R.transpose();
		tVector x_col = cMathUtil::SkewMatToVector(x_col_mat);
		tVector y_col = cMathUtil::SkewMatToVector(y_col_mat);
		tVector z_col = cMathUtil::SkewMatToVector(z_col_mat);
		Eigen::Matrix3d Jw = Eigen::Matrix3d::Zero();
		Jw.block(0, 0, 3, 1) = x_col.segment(0, 3);
		Jw.block(0, 1, 3, 1) = y_col.segment(0, 3);
		Jw.block(0, 2, 3, 1) = z_col.segment(0, 3);
		tVector res = tVector::Zero();
		res.segment(0, 3) = Jw.inverse() * omega.segment(0, 3);
		return res;
		
	}
	else if (order == eRotationOrder::ZYX)
	{
		tMatrix R = Rx * Ry * Rz;
		tMatrix dR_dx = Rx_dotx * Ry * Rz,
			dR_dy = Rx * Ry_doty * Rz,
			dR_dz = Rx * Ry * Rz_dotz;
		tMatrix x_col_mat = dR_dx * R.transpose(),
			y_col_mat = dR_dy * R.transpose(),
			z_col_mat = dR_dz * R.transpose();
		tVector x_col = cMathUtil::SkewMatToVector(x_col_mat);
		tVector y_col = cMathUtil::SkewMatToVector(y_col_mat);
		tVector z_col = cMathUtil::SkewMatToVector(z_col_mat);
		Eigen::Matrix3d Jw = Eigen::Matrix3d::Zero();
		Jw.block(0, 0, 3, 1) = x_col.segment(0, 3);
		Jw.block(0, 1, 3, 1) = y_col.segment(0, 3);
		Jw.block(0, 2, 3, 1) = z_col.segment(0, 3);
		tVector res = tVector::Zero();
		res.segment(0, 3) = Jw.inverse() * omega.segment(0, 3);
		return res;
	}
	else
	{

		std::cout << "[error] cMathUtil::AngularVelToqdot: Unsupported rotation order" << std::endl;
		exit(1);
	}
}

tMatrix cMathUtil::VectorToSkewMat(const tVector &vec)
{
	tMatrix res = tMatrix::Zero();
	double a = vec[0], b = vec[1], c = vec[2];
	res(0, 1) = -c;
	res(0, 2) = b;
	res(1, 0) = c;
	res(1, 2) = -a;
	res(2, 0) = -b;
	res(2, 1) = a;

	return res;
}

tVector cMathUtil::SkewMatToVector(const tMatrix & mat)
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

tMatrix xconventionRotation(double x)
{
	// 这是在转坐标系，把child frame旋转掉
	// 所以得到的rotation matrix是local -> parent
	//return AngleAxisd(x, Vector3d::UnitX()).toRotationMatrix();

	tMatrix m = tMatrix::Identity();

	double cosx = cos(x);
	double sinx = sin(x);

	m(0, 0) = 1;
	m(1, 1) = cosx;
	m(1, 2) = -sinx;
	m(2, 1) = sinx;
	m(2, 2) = cosx;

	return m;
}

tMatrix yconventionRotation(double y)
{
	//return AngleAxisd(y, Vector3d::UnitY()).toRotationMatrix();
	tMatrix m = tMatrix::Identity();

	double cosy = cos(y);
	double siny = sin(y);

	m(1, 1) = 1;
	m(0, 0) = cosy;
	m(0, 2) = siny;
	m(2, 0) = -siny;
	m(2, 2) = cosy;
	return m;
}

tMatrix zconventionRotation(double z)
{
	//return AngleAxisd(z, Vector3d::UnitZ()).toRotationMatrix();
	tMatrix m = tMatrix::Identity();
	m.setZero();

	double cosz = cos(z);
	double sinz = sin(z);

	m(2, 2) = 1;
	m(0, 0) = cosz;
	m(0, 1) = -sinz;
	m(1, 0) = sinz;
	m(1, 1) = cosz;

	return m;
}
tMatrix zconventionRotation_dz(double z)
{
	tMatrix output = tMatrix::Zero();
	double cosz = cos(z);
	double sinz = sin(z);

	output(0, 0) = -sinz;
	output(0, 1) = -cosz;
	output(1, 0) = cosz;
	output(1, 1) = -sinz;
	return output;
}

tMatrix yconventionRotation_dy(double y)
{
	tMatrix output = tMatrix::Zero();
	double cosy = cos(y);
	double siny = sin(y);

	output(0, 0) = -siny;
	output(0, 2) = cosy;
	output(2, 0) = -cosy;
	output(2, 2) = -siny;
	return output;
}

tMatrix xconventionRotation_dx(double x)
{
	tMatrix output = tMatrix::Zero();

	double cosx = cos(x);
	double sinx = sin(x);

	output(1, 1) = -sinx;
	output(1, 2) = -cosx;
	output(2, 1) = cosx;
	output(2, 2) = -sinx;
	return output;
}