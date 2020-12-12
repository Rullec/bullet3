#include "EulerAngleRotationMatrix.h"
#include "ModelEigenUtils.h"
#include <iostream>
#include <math.h>

tMatrix (*RotFunc4d[3])(double) = {xconventionTransform, yconventionTransform,
                                   zconventionTransform};
tMatrix3d (*RotFunc3d[3])(double) = {xconventionRotation, yconventionRotation,
                                     zconventionRotation};

tVector3d QuaternionToEuler(const tQuaternion &q)
{
    tMatrix3d m = q.toRotationMatrix();
    double c = tVector3d(m.coeff(2, 1), m.coeff(2, 2), 0).norm();
    double gamma = atan2(m.coeff(1, 0), m.coeff(0, 0));
    double beta = atan2(-m.coeff(2, 0), c);
    double alpha = atan2(m.coeff(2, 1), m.coeff(2, 2));
    return tVector3d(alpha, beta, gamma);
}

tVector3d XYZtoYZX(double x, double y, double z)
{
    tMatrix3d m = zconventionRotation(z) * yconventionRotation(y) *
                  xconventionRotation(x);
    double theta_z = asin(-m.coeff(0, 1));
    double theta_y = atan2(m.coeff(0, 2), m.coeff(0, 0));
    double theta_x = atan2(m.coeff(2, 1), m.coeff(1, 1));
    return tVector3d(theta_x, theta_y, theta_z);
}

tMatrix3d EulerAngleRotation(const tVector3d &v, const Eigen::Vector3i &order)
{
    // std::cout << order.transpose() << std::endl;
    // return zconventionRotation(v[0]) * yconventionRotation(v[1]) *
    // xconventionRotation(v[0]); return zconventionRotation(v[order[0]]) *
    // yconventionRotation(v[order[1]]) * xconventionRotation(v[order[0]]);
    return RotFunc3d[2](v[2]) * RotFunc3d[1](v[1]) * RotFunc3d[0](v[0]);
}

tVector3d RotMatToAxisAngle(const tMatrix3d &R)
{

    Eigen::EigenSolver<tMatrix3d> eigen_solver(R);
    const tVector3d &e = eigen_solver.eigenvalues().real();
    const tMatrix3d &m = eigen_solver.eigenvectors().real();

    for (int i = 0; i < 3; ++i)
    {
        if (abs(e.coeff(i) - 1.) < 1e-6)
        {

            const tVector3d &u = m.col(i);
            tVector3d v;
            if (u[0] != 0)
                v = tVector3d(-(u[1] + u[2]) / u[0], 1., 1.);
            else if (u[1] != 0)
                v = tVector3d(1., -(u[0] + u[2]) / u[1], 1.);
            else if (u[2] != 0)
                v = tVector3d(1., 1., -(u[0] + u[1]) / u[2]);
            else
            {
                return {0, 0, 0};
            }

            const tVector3d &rv = R * v;
            double a = acos(rv.dot(v) / (rv.norm() * v.norm()));
            double sign = 1;
            if (v.cross(rv).dot(u) < 0)
                sign = -1;

            return m.col(i) * a * sign;
        }
    }
    return {0., 0., 0.};
}

tMatrix EulerAngleTransform(const tVector3d &v, const Eigen::Vector3i &order)
{
    return RotFunc4d[order[0]](v[order[0]]) * RotFunc4d[order[1]](v[order[1]]) *
           RotFunc4d[order[2]](v[order[2]]);
}

tMatrix3d EulerAngleRotation(double x, double y, double z)
{
    tMatrix3d m;
    m = xconventionRotation(x) * yconventionRotation(y) *
        zconventionRotation(z);
    return m;
}

tMatrix3d xconventionRotation(double x)
{
    // return AngleAxisd(x, tVector3d::UnitX()).toRotationMatrix();

    tMatrix3d m;

    double cosx = cos(x);
    double sinx = sin(x);

    m.setZero();
    m.data()[0] = 1;
    m.data()[4] = cosx;
    m.data()[5] = sinx;
    m.data()[7] = -sinx;
    m.data()[8] = cosx;
    return m;
}

tMatrix3d yconventionRotation(double y)
{
    // return AngleAxisd(y, tVector3d::UnitY()).toRotationMatrix();
    tMatrix3d m;
    double cosy = cos(y);
    double siny = sin(y);

    m.setZero();
    m.data()[0] = cosy;
    m.data()[2] = -siny;
    m.data()[4] = 1;
    m.data()[6] = siny;
    m.data()[8] = cosy;
    return m;
}

tMatrix3d zconventionRotation(double z)
{
    // return AngleAxisd(z, tVector3d::UnitZ()).toRotationMatrix();
    tMatrix3d m;
    m.setZero();

    double cosq = cos(z);
    double sinq = sin(z);

    m.data()[0] = cosq;
    m.data()[1] = sinq;
    m.data()[3] = -sinq;
    m.data()[4] = cosq;
    m.data()[8] = 1;

    return m;
}

tMatrix xconventionTransform(double x)
{
    tMatrix m;
    double cosx = cos(x);
    double sinx = sin(x);

    m.data()[0] = 1;
    m.data()[5] = cosx;
    m.data()[6] = sinx;
    m.data()[9] = -sinx;
    m.data()[10] = cosx;
    m.data()[15] = 1;
    return m;
}

tMatrix yconventionTransform(double y)
{
    tMatrix m;
    m.setZero();
    double cosy = cos(y);
    double siny = sin(y);

    m.data()[0] = cosy;
    m.data()[2] = -siny;
    m.data()[5] = 1;
    m.data()[8] = siny;
    m.data()[10] = cosy;
    m.data()[15] = 1;
    return m;
}

tMatrix zconventionTransform(double z)
{
    tMatrix m;
    m.setZero();

    double cosz = cos(z);
    double sinz = sin(z);

    m.data()[0] = cosz;
    m.data()[1] = sinz;
    m.data()[4] = -sinz;
    m.data()[5] = cosz;
    m.data()[10] = 1;
    m.data()[15] = 1;
    return m;
}

void xconventionTransform(tMatrix &output, double x)
{
    output.setZero();

    double cosx = cos(x);
    double sinx = sin(x);

    output.data()[0] = 1;
    output.data()[5] = cosx;
    output.data()[6] = sinx;
    output.data()[9] = -sinx;
    output.data()[10] = cosx;
    output.data()[15] = 1;
}

void xconventionRotation_dx(tMatrix &output, double x)
{
    output.setZero();

    double cosx = cos(x);
    double sinx = sin(x);

    output.data()[0] = 0;
    output.data()[5] = -sinx;
    output.data()[6] = cosx;
    output.data()[9] = -cosx;
    output.data()[10] = -sinx;
}

void xconventionRotation_dxdx(tMatrix &output, double x)
{
    output.setZero();

    double cosx = cos(x);
    double sinx = sin(x);

    output.data()[0] = 0;
    output.data()[5] = -cosx;
    output.data()[6] = -sinx;
    output.data()[9] = sinx;
    output.data()[10] = -cosx;
}

void xconventionRotation_dxdxdx(tMatrix &output, double x)
{
    output.setZero();

    double cosx = cos(x);
    double sinx = sin(x);

    output.data()[0] = 0;
    output.data()[5] = sinx;
    output.data()[6] = -cosx;
    output.data()[9] = cosx;
    output.data()[10] = sinx;
}

void yconventionTransform(tMatrix &output, double y)
{
    output.setZero();
    double cosy = cos(y);
    double siny = sin(y);

    output.data()[0] = cosy;
    output.data()[2] = -siny;
    output.data()[5] = 1;
    output.data()[8] = siny;
    output.data()[10] = cosy;
    output.data()[15] = 1;
}

void yconventionRotation_dy(tMatrix &output, double y)
{
    output.setZero();

    double cosy = cos(y);
    double siny = sin(y);

    output.data()[0] = -siny;
    output.data()[2] = -cosy;
    output.data()[5] = 0;
    output.data()[8] = cosy;
    output.data()[10] = -siny;
}

void yconventionRotation_dydy(tMatrix &output, double y)
{
    output.setZero();

    double cosy = cos(y);
    double siny = sin(y);

    output.data()[0] = -cosy;
    output.data()[2] = siny;
    output.data()[5] = 0;
    output.data()[8] = -siny;
    output.data()[10] = -cosy;
}

void yconventionRotation_dydydy(tMatrix &output, double y)
{
    output.setZero();

    double cosy = cos(y);
    double siny = sin(y);

    output.data()[0] = siny;
    output.data()[2] = cosy;
    output.data()[5] = 0;
    output.data()[8] = -cosy;
    output.data()[10] = siny;
}

void zconventionTransform(tMatrix &output, double z)
{
    output.setZero();

    double cosz = cos(z);
    double sinz = sin(z);

    output.data()[0] = cosz;
    output.data()[1] = sinz;
    output.data()[4] = -sinz;
    output.data()[5] = cosz;
    output.data()[10] = 1;
    output.data()[15] = 1;
}

void zconventionRotation_dz(tMatrix &output, double z)
{
    output.setZero();
    double cosz = cos(z);
    double sinz = sin(z);

    output.data()[0] = -sinz;
    output.data()[1] = cosz;
    output.data()[4] = -cosz;
    output.data()[5] = -sinz;
    output.data()[10] = 0;
}

void zconventionRotation_dzdz(tMatrix &output, double z)
{
    output.setZero();
    double cosz = cos(z);
    double sinz = sin(z);

    output.data()[0] = -cosz;
    output.data()[1] = -sinz;
    output.data()[4] = sinz;
    output.data()[5] = -cosz;
    output.data()[10] = 0;
}

void zconventionRotation_dzdzdz(tMatrix &output, double z)
{
    output.setZero();
    double cosz = cos(z);
    double sinz = sin(z);

    output.data()[0] = sinz;
    output.data()[1] = -cosz;
    output.data()[4] = cosz;
    output.data()[5] = sinz;
    output.data()[10] = 0;
}

tMatrix3d xyzconventionRotation(double x, double y, double z)
{
    tMatrix3d m;

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    m.data()[0] = cosy * cosz;
    m.data()[1] = cosz * sinx * siny + cosx * sinz;
    m.data()[2] = -cosx * cosz * siny + sinx * sinz;

    m.data()[3] = -cosy * sinz;
    m.data()[4] = cosx * cosz - sinx * siny * sinz;
    m.data()[5] = cosz * sinx + cosx * siny * sinz;

    m.data()[6] = siny;
    m.data()[7] = -cosy * sinx;
    m.data()[8] = cosx * cosy;

    return m;
}

EIGEN_V_tMatrix3dD xyzRotationDeriv(double x, double y, double z)
{
    EIGEN_V_tMatrix3dD m(3);
    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    // repect to x
    m[0].setZero();
    m[0].data()[1] = cosz * cosx * siny - sinx * sinz;
    m[0].data()[2] = sinx * cosz * siny + cosx * sinz;
    m[0].data()[4] = -sinx * cosz - cosx * siny * sinz;
    m[0].data()[5] = cosz * cosx - sinx * siny * sinz;
    m[0].data()[7] = -cosy * cosx;
    m[0].data()[8] = -sinx * cosy;

    // respect y
    m[1].setZero();
    m[1].data()[0] = -siny * cosz;
    m[1].data()[1] = cosz * sinx * cosy;
    m[1].data()[2] = -cosx * cosz * cosy;
    m[1].data()[3] = siny * sinz;
    m[1].data()[4] = -sinx * cosy * sinz;
    m[1].data()[5] = cosx * cosy * sinz;
    m[1].data()[6] = cosy;
    m[1].data()[7] = siny * sinx;
    m[1].data()[8] = -cosx * siny;

    // respect z
    m[2].setZero();
    m[2].data()[0] = -cosy * sinz;
    m[2].data()[1] = -sinz * sinx * siny + cosx * cosz;
    m[2].data()[2] = cosx * sinz * siny + sinx * cosz;
    m[2].data()[3] = -cosy * cosz;
    m[2].data()[4] = -cosx * sinz - sinx * siny * cosz;
    m[2].data()[5] = -sinz * cosx + cosx * siny * cosz;

    return m;
}

void rotationFirstDerive_dx(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    // output.data()[0] = 0;
    output.data()[1] = cosz * cosx * siny - sinx * sinz;
    output.data()[2] = sinx * cosz * siny + cosx * sinz;
    // output.data()[4] = 0;
    output.data()[5] = -sinx * cosz - cosx * siny * sinz;
    output.data()[6] = cosz * cosx - sinx * siny * sinz;
    // output.data()[8] = 0;
    output.data()[9] = -cosy * cosx;
    output.data()[10] = -sinx * cosy;
}

void rotationFirstDerive_dy(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    output.data()[0] = -siny * cosz;
    output.data()[1] = cosz * sinx * cosy;
    output.data()[2] = -cosx * cosz * cosy;
    output.data()[4] = siny * sinz;
    output.data()[5] = -sinx * cosy * sinz;
    output.data()[6] = cosx * cosy * sinz;
    output.data()[8] = cosy;
    output.data()[9] = siny * sinx;
    output.data()[10] = -cosx * siny;
}

void rotationFirstDerive_dz(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    output.data()[0] = -cosy * sinz;
    output.data()[1] = -sinz * sinx * siny + cosx * cosz;
    output.data()[2] = cosx * sinz * siny + sinx * cosz;
    output.data()[4] = -cosy * cosz;
    output.data()[5] = -cosx * sinz - sinx * siny * cosz;
    output.data()[6] = -sinz * sinx + cosx * siny * cosz;
}

void rotationSecondDerive_dxdx(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    output.data()[1] = -cosz * sinx * siny - cosx * sinz;
    output.data()[2] = cosx * cosz * siny - sinx * sinz;
    output.data()[5] = -cosx * cosz + sinx * siny * sinz;
    output.data()[6] = -cosz * sinx - cosx * siny * sinz;
    output.data()[9] = cosy * sinx;
    output.data()[10] = -cosx * cosy;
}

void rotationSecondDerive_dxdy(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    output.data()[1] = cosz * cosx * cosy;
    output.data()[2] = sinx * cosz * cosy;
    output.data()[5] = -cosx * cosy * sinz;
    output.data()[6] = -sinx * cosy * sinz;
    output.data()[9] = siny * cosx;
    output.data()[10] = sinx * siny;
}

void rotationSecondDerive_dxdz(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    output.data()[1] = -sinz * cosx * siny - sinx * cosz;
    output.data()[2] = -sinx * sinz * siny + cosx * cosz;
    output.data()[5] = sinx * sinz - cosx * siny * cosz;
    output.data()[6] = -sinz * cosx - sinx * siny * cosz;

    /*output.data()[9] = 0;
    output.data()[10] = 0;*/
}

void rotationSecondDerive_dydx(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    // output.data()[0] = 0;
    output.data()[1] = cosz * cosx * cosy;
    output.data()[2] = sinx * cosz * cosy;
    // output.data()[4] = 0;
    output.data()[5] = -cosx * cosy * sinz;
    output.data()[6] = -sinx * cosy * sinz;
    // output.data()[8] = 0;
    output.data()[9] = siny * cosx;
    output.data()[10] = sinx * siny;
}

void rotationSecondDerive_dydy(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    output.data()[0] = -cosy * cosz;
    output.data()[1] = -cosz * sinx * siny;
    output.data()[2] = cosx * cosz * siny;
    output.data()[4] = cosy * sinz;
    output.data()[5] = sinx * siny * sinz;
    output.data()[6] = -cosx * siny * sinz;
    output.data()[8] = -siny;
    output.data()[9] = cosy * sinx;
    output.data()[10] = -cosx * cosy;
}

void rotationSecondDerive_dydz(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    output.data()[0] = siny * sinz;
    output.data()[1] = -sinz * sinx * cosy;
    output.data()[2] = cosx * sinz * cosy;
    output.data()[4] = siny * cosz;
    output.data()[5] = -sinx * cosy * cosz;
    output.data()[6] = cosx * cosy * cosz;
    // output.data()[8] = 0;
    // output.data()[9] = 0;
    // output.data()[10] = 0;
}

void rotationSecondDerive_dzdx(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    // output.data()[0] = 0;
    output.data()[1] = -sinz * cosx * siny - sinx * cosz;
    output.data()[2] = -sinx * sinz * siny + cosx * cosz;
    // output.data()[4] = 0;
    output.data()[5] = sinx * sinz - cosx * siny * cosz;
    output.data()[6] = -sinz * cosx - sinx * siny * cosz;
}

void rotationSecondDerive_dzdy(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    output.data()[0] = siny * sinz;
    output.data()[1] = -sinz * sinx * cosy;
    output.data()[2] = cosx * sinz * cosy;
    output.data()[4] = siny * cosz;
    output.data()[5] = -sinx * cosy * cosz;
    output.data()[6] = cosx * cosy * cosz;
}

void rotationSecondDerive_dzdz(tMatrix &output, double x, double y, double z)
{
    output.topLeftCorner<3, 3>().setZero();

    double sinx = sin(x);
    double cosx = cos(x);

    double siny = sin(y);
    double cosy = cos(y);

    double sinz = sin(z);
    double cosz = cos(z);

    output.data()[0] = -cosy * cosz;
    output.data()[1] = -cosz * sinx * siny - cosx * sinz;
    output.data()[2] = cosx * cosz * siny - sinx * sinz;
    output.data()[4] = cosy * sinz;
    output.data()[5] = -cosx * cosz + sinx * siny * sinz;
    output.data()[6] = -cosz * sinx - cosx * siny * sinz;
}

tMatrix3d xRotationDeriv(double x)
{
    tMatrix3d m;
    return m;
}
