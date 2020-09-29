#include "MathUtil.h"
#include <iostream>
// enum eRotationOrder order = eRotationOrder::XYZ;

// 0 order
tMatrix xconventionRotation_mimic(double x);
tMatrix yconventionRotation_mimic(double y);
tMatrix zconventionRotation_mimic(double z);

// 1 order
tMatrix xconventionRotation_mimic_dx(double x);
tMatrix yconventionRotation_mimic_dy(double y);
tMatrix zconventionRotation_mimic_dz(double z);

tMatrix btMathUtil::RotMat(const tQuaternion &quater_)
{
    // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix

    tMatrix res = tMatrix::Zero();
    double w = quater_.w(), x = quater_.x(), y = quater_.y(), z = quater_.z();
    res << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0,
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0,
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0, 0,
        0, 0, 1;
    return res;
}

tQuaternion btMathUtil::RotMatToQuaternion(const tMatrix &mat)
{
    // http://www.iri.upc.edu/files/scidoc/2068-Accurate-Computation-of-Quaternions-from-Rotation-Matrices.pdf
    double eta = 0;
    double q1, q2, q3, q4; // = [w, x, y, z]

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
            q1 = 0.5 * std::sqrt(numerator / (3 - detect_value));
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
    sign[0] = btMathUtil::sign(q1);
    sign[1] = btMathUtil::sign(mat(2, 1) - mat(1, 2));
    sign[2] = btMathUtil::sign(mat(0, 2) - mat(2, 0));
    sign[3] = btMathUtil::sign(mat(1, 0) - mat(0, 1));
    if (sign[0] < 0)
        for (int i = 1; i < 4; i++)
            sign[i] *= -1;

    return tQuaternion(sign[0] * q1, sign[1] * q2, sign[2] * q3, sign[3] * q4);
}

tVector btMathUtil::QuaternionToCoef(const tQuaternion &quater)
{
    // quaternion -> vec = [x, y, z, w]
    return tVector(quater.x(), quater.y(), quater.z(), quater.w());
}

tQuaternion btMathUtil::CoefToQuaternion(const tVector &vec)
{
    // vec = [x, y, z, w] -> quaternion
    if (vec[3] > 0)
        return tQuaternion(vec[3], vec[0], vec[1], vec[2]);
    else
        return tQuaternion(-vec[3], -vec[0], -vec[1], -vec[2]);
}

tQuaternion btMathUtil::AxisAngleToQuaternion(const tVector &angvel)
{
    double theta = angvel.norm();
    double theta_2 = theta / 2;
    double cos_theta_2 = std::cos(theta_2), sin_theta_2 = std::sin(theta_2);

    tVector norm_angvel = angvel.normalized();
    return tQuaternion(cos_theta_2, norm_angvel[0] * sin_theta_2,
                       norm_angvel[1] * sin_theta_2,
                       norm_angvel[2] * sin_theta_2);
}

tVector btMathUtil::QuaternionToAxisAngle(const tQuaternion &quater)
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

    if (theta < 1e-4)
        return tVector::Zero();

    // std::cout << theta << " " << std::sin(theta / 2) << std::endl;
    double ax = quater.x() / std::sin(theta / 2),
           ay = quater.y() / std::sin(theta / 2),
           az = quater.z() / std::sin(theta / 2);
    return theta * tVector(ax, ay, az, 0);
}

tVector btMathUtil::CalcAngularVelocity(const tQuaternion &old_rot,
                                        const tQuaternion &new_rot,
                                        double timestep)
{
    tQuaternion trans = new_rot * old_rot.conjugate();
    double theta = std::acos(trans.w()) * 2; // std::acos() output range [0, pi]
    if (theta > 2 * M_PI - theta)
    {
        // theta = theta - 2*pi
        theta = theta - 2 * M_PI; // -pi - pi
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

tVector btMathUtil::CalcAngularVelocityFromAxisAngle(const tQuaternion &old_rot,
                                                     const tQuaternion &new_rot,
                                                     double timestep)
{
    std::cout << "cMathUtil::CalcAngularVelocityFromAxisAngle: this func "
                 "hasn't been well-tested, call another one\n";
    exit(0);
    tVector old_aa = btMathUtil::QuaternionToAxisAngle(old_rot),
            new_aa = btMathUtil::QuaternionToAxisAngle(new_rot);
    return (new_aa - old_aa) / timestep;
}

tVector btMathUtil::QuatRotVec(const tQuaternion &quater, const tVector &vec)
{
    tVector res = tVector::Zero();
    res.segment(0, 3) = quater * vec.segment(0, 3);
    return res;
}

tVector btMathUtil::QuaternionToEulerAngles(const tQuaternion &q,
                                            const btRotationOrder &order)
{
    tVector res = tVector::Zero();
    double w = q.w(), x = q.x(), y = q.y(), z = q.z();

    // please check the note for details
    if (order == btRotationOrder::bt_XYZ)
    {
        res[0] = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        res[1] = std::asin(2 * (w * y - z * x));
        res[2] = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    }
    else if (order == btRotationOrder::bt_ZYX)
    {
        res[0] = std::atan2(2 * (w * x - y * z), 1 - 2 * (x * x + y * y));
        res[1] = std::asin(2 * (w * y + z * x));
        res[2] = std::atan2(2 * (w * z - x * y), 1 - 2 * (y * y + z * z));
    }
    else
    {
        std::cout << "[error] tVector cMathUtil::QuaternionToEulerAngles "
                     "Unsupported rotation order = "
                  << order;
        exit(0);
    }
    return res;
}

tQuaternion btMathUtil::EulerAnglesToQuaternion(const tVector &vec,
                                                const btRotationOrder &order)
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
    if (order == btRotationOrder::bt_XYZ)
    {
        res = q[2] * q[1] * q[0];
    }
    else if (order == btRotationOrder::bt_ZYX)
    {
        res = q[0] * q[1] * q[2];
    }

    if (res.w() < 0)
        res = btMathUtil::MinusQuaternion(res);
    return res;
}

tQuaternion btMathUtil::MinusQuaternion(const tQuaternion &quad)
{
    return tQuaternion(-quad.w(), -quad.x(), -quad.y(), -quad.z());
}

tMatrix btMathUtil::EulerAnglesToRotMat(const tVector &euler,
                                        const btRotationOrder &order)
{
    // input euler angles: the rotation theta from parent to local
    // output rot mat: a rot mat that can convert a vector FROM LOCAL FRAME TO
    // PARENT FRAME
    double x = euler[0], y = euler[1], z = euler[2];
    tMatrix mat = tMatrix::Identity();
    if (order == btRotationOrder::bt_XYZ)
    {
        tMatrix x_mat, y_mat, z_mat;
        x_mat = xconventionRotation_mimic(x);
        y_mat = yconventionRotation_mimic(y);
        z_mat = zconventionRotation_mimic(z);
        mat = z_mat * y_mat * x_mat;
    }
    else if (order == btRotationOrder::bt_ZYX)
    {
        tMatrix x_mat, y_mat, z_mat;
        x_mat = xconventionRotation_mimic(x);
        y_mat = yconventionRotation_mimic(y);
        z_mat = zconventionRotation_mimic(z);
        mat = x_mat * y_mat * z_mat;
    }
    else
    {
        std::cout << "[error] cMathUtil::EulerAnglesToRotMat(const tVector& "
                     "euler): Unsupported rotation order"
                  << std::endl;
        exit(0);
    }
    return mat;
}

tMatrix btMathUtil::EulerAnglesToRotMatDot(const tVector &euler,
                                           const btRotationOrder &order)
{
    double x = euler[0], y = euler[1], z = euler[2];
    tMatrix mat = tMatrix::Identity();
    if (order == btRotationOrder::bt_XYZ)
    {
        tMatrix Rz = zconventionRotation_mimic(z),
                Ry = yconventionRotation_mimic(y),
                Rx = xconventionRotation_mimic(x);
        tMatrix Rz_dot = zconventionRotation_mimic_dz(z),
                Ry_dot = yconventionRotation_mimic_dy(y),
                Rx_dot = xconventionRotation_mimic_dx(x);
        mat = Rz * Ry * Rx_dot + Rz_dot * Ry * Rx + Rz * Ry_dot * Rx;
    }
    else if (order == btRotationOrder::bt_ZYX)
    {
        tMatrix Rz = zconventionRotation_mimic(z),
                Ry = yconventionRotation_mimic(y),
                Rx = xconventionRotation_mimic(x);
        tMatrix Rz_dot = zconventionRotation_mimic_dz(z),
                Ry_dot = yconventionRotation_mimic_dy(y),
                Rx_dot = xconventionRotation_mimic_dx(x);
        mat = Rx * Ry * Rz_dot + Rx_dot * Ry * Rz + Rx * Ry_dot * Rz;
    }
    else
    {
        std::cout << "[error] cMathUtil::EulerAnglesToRotMatDot(const tVector& "
                     "euler): Unsupported rotation order"
                  << std::endl;
        exit(0);
    }
    return mat;
}

tVector btMathUtil::AngularVelToqdot(const tVector &omega, const tVector &cur_q,
                                     const btRotationOrder &order)
{
    // w = Jw * q'
    // q' = (Jw)^{-1} * omega
    //[w] = R' * R^T

    // step1: get Jw
    // please read P8 formula (30) in C.K Liu's tutorial "A Quick Tutorial on
    // Multibody Dynamics" for more details
    double x = cur_q[0], y = cur_q[1], z = cur_q[2];
    tMatrix Rx = xconventionRotation_mimic(x),
            Ry = yconventionRotation_mimic(y),
            Rz = zconventionRotation_mimic(z);
    tMatrix Rx_dotx = xconventionRotation_mimic_dx(x),
            Ry_doty = yconventionRotation_mimic_dy(y),
            Rz_dotz = zconventionRotation_mimic_dz(z);

    if (order == btRotationOrder::bt_XYZ)
    {
        tMatrix R = Rz * Ry * Rx;
        tMatrix dR_dx = Rz * Ry * Rx_dotx, dR_dy = Rz * Ry_doty * Rx,
                dR_dz = Rz_dotz * Ry * Rx;
        tMatrix x_col_mat = dR_dx * R.transpose(),
                y_col_mat = dR_dy * R.transpose(),
                z_col_mat = dR_dz * R.transpose();
        tVector x_col = btMathUtil::SkewMatToVector(x_col_mat);
        tVector y_col = btMathUtil::SkewMatToVector(y_col_mat);
        tVector z_col = btMathUtil::SkewMatToVector(z_col_mat);
        Eigen::Matrix3d Jw = Eigen::Matrix3d::Zero();
        Jw.block(0, 0, 3, 1) = x_col.segment(0, 3);
        Jw.block(0, 1, 3, 1) = y_col.segment(0, 3);
        Jw.block(0, 2, 3, 1) = z_col.segment(0, 3);
        tVector res = tVector::Zero();
        res.segment(0, 3) = Jw.inverse() * omega.segment(0, 3);
        return res;
    }
    else if (order == btRotationOrder::bt_ZYX)
    {
        tMatrix R = Rx * Ry * Rz;
        tMatrix dR_dx = Rx_dotx * Ry * Rz, dR_dy = Rx * Ry_doty * Rz,
                dR_dz = Rx * Ry * Rz_dotz;
        tMatrix x_col_mat = dR_dx * R.transpose(),
                y_col_mat = dR_dy * R.transpose(),
                z_col_mat = dR_dz * R.transpose();
        tVector x_col = btMathUtil::SkewMatToVector(x_col_mat);
        tVector y_col = btMathUtil::SkewMatToVector(y_col_mat);
        tVector z_col = btMathUtil::SkewMatToVector(z_col_mat);
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
        std::cout
            << "[error] cMathUtil::AngularVelToqdot: Unsupported rotation order"
            << std::endl;
        exit(0);
    }
}

tMatrix btMathUtil::VectorToSkewMat(const tVector &vec)
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
// Nx3 friction cone
// each row is a direction now
tMatrixXd btMathUtil::ExpandFrictionCone(int num_friction_dirs,
                                         const tVector &normal_)
{
    // 1. check the input
    tVector normal = normal_;
    normal[3] = 0;
    normal.normalize();
    if (normal.norm() < 1e-6)
    {
        std::cout << "[error] ExpandFrictionCone normal = "
                  << normal_.transpose() << std::endl;
        exit(0);
    }

    // 2. generate a standard friction cone
    tMatrixXd D = tMatrixXd::Zero(4, num_friction_dirs);
    double gap = 2 * M_PI / num_friction_dirs;
    for (int i = 0; i < num_friction_dirs; i++)
    {
        D(0, i) = std::cos(gap * i);
        D(2, i) = std::sin(gap * i);
    }

    // 3. rotate the fricition cone
    tVector Y_normal = tVector(0, 1, 0, 0);
    tVector axis = Y_normal.cross3(normal).normalized();
    double theta = std::acos(Y_normal.dot(normal)); // [0, pi]
    D = btMathUtil::RotMat(btMathUtil::AxisAngleToQuaternion(axis * theta)) * D;
    D.transposeInPlace();
    // each row is a direction now
    return D;
}

tMatrix xconventionRotation_mimic(double x)
{
    // ������ת����ϵ����child frame��ת��
    // ���Եõ���rotation matrix��local -> parent
    // return AngleAxisd(x, Vector3d::UnitX()).toRotationMatrix();

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

tMatrix yconventionRotation_mimic(double y)
{
    // return AngleAxisd(y, Vector3d::UnitY()).toRotationMatrix();
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

tMatrix zconventionRotation_mimic(double z)
{
    // return AngleAxisd(z, Vector3d::UnitZ()).toRotationMatrix();
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
tMatrix zconventionRotation_mimic_dz(double z)
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

tMatrix yconventionRotation_mimic_dy(double y)
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

tMatrix xconventionRotation_mimic_dx(double x)
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

tMatrix btMathUtil::DirToRotMat(const tVector &dir, const tVector &up)
{
    tVector Y_normal = up;
    tVector axis = Y_normal.cross3(dir).normalized();
    double theta = std::acos(Y_normal.dot(dir)); // [0, pi]
    return btMathUtil::RotMat(btMathUtil::AxisAngleToQuaternion(axis * theta));
}

tMatrix btMathUtil::InverseTransform(const tMatrix &raw_trans)
{
    tMatrix inv_trans = tMatrix::Identity();
    inv_trans.block(0, 0, 3, 3) = raw_trans.block(0, 0, 3, 3).transpose();
    inv_trans.block(0, 3, 3, 1) =
        -inv_trans.block(0, 0, 3, 3) * raw_trans.block(0, 3, 3, 1);
    return inv_trans;
}

double btMathUtil::CalcConditionNumber(const tMatrixXd &mat)
{
    Eigen::EigenSolver<tMatrixXd> solver(mat);
    tVectorXd eigen_values = solver.eigenvalues().real();
    return eigen_values.maxCoeff() / eigen_values.minCoeff();
}

/**
 * \brief		Get the jacobian preconditioner P = diag(A)
 *
 */
tMatrixXd btMathUtil::JacobPreconditioner(const tMatrixXd &A)
{
    if (A.rows() != A.cols())
    {
        std::cout << "cMathUtil::JacobPreconditioner: A is not a square matrix "
                  << A.rows() << " " << A.cols() << std::endl;
        exit(0);
    }
    tVectorXd diagonal = A.diagonal();
    if (diagonal.cwiseAbs().minCoeff() < 1e-10)
    {
        std::cout
            << "cMathUtil::JacobPreconditioner: diagnoal is nearly zero for "
            << diagonal.transpose() << std::endl;
        exit(0);
    }

    return diagonal.cwiseInverse().asDiagonal();
}