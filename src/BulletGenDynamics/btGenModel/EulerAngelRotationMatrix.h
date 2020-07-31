#pragma once
#include "ModelEigenUtils.h"

tVector3d QuaternionToEuler(const tQuaternion& q);
tMatrix3d EulerAngelRotation(double x, double y, double z);
tMatrix EulerAngleTransform(const tVector3d& v, const Eigen::Vector3i& order);
tMatrix3d EulerAngleRotation(const tVector3d& v, const Eigen::Vector3i& order);

tVector3d RotMatToAxisAngle(const tMatrix3d& R);

tVector3d XYZtoYZX(double x, double y, double z);

tMatrix3d xconventionRotation(double x);
tMatrix3d yconventionRotation(double y);
tMatrix3d zconventionRotation(double z);

tMatrix xconventionTransform(double x);
tMatrix yconventionTransform(double y);
tMatrix zconventionTransform(double z);

void xconventionTransform(tMatrix& output, double x);
void xconventionRotation_dx(tMatrix& output, double x);
void xconventionRotation_dxdx(tMatrix& output, double x);
void xconventionRotation_dxdxdx(tMatrix& output, double x);

void yconventionTransform(tMatrix& output, double y);
void yconventionRotation_dy(tMatrix& output, double y);
void yconventionRotation_dydy(tMatrix& output, double y);
void yconventionRotation_dydydy(tMatrix& output, double y);

void zconventionTransform(tMatrix& output, double z);
void zconventionRotation_dz(tMatrix& output, double z);
void zconventionRotation_dzdz(tMatrix& output, double z);
void zconventionRotation_dzdzdz(tMatrix& output, double z);

tMatrix3d xyzconventionRotation(double x, double y, double z);
EIGEN_V_tMatrix3dD xyzRotationDeriv(double x, double y, double z);

//************************************
// Method:    rotationFirstDervie_dx
// FullName:  rotationFirstDervie_dx
// Access:    public
// Returns:   void
// Qualifier:
// Parameter: tMatrix & output need to inital outside this function will only set the rotation part
// Parameter: double x
// Parameter: double y
// Parameter: double z
//************************************
void rotationFirstDerive_dx(tMatrix& output, double x, double y, double z);
void rotationFirstDerive_dy(tMatrix& output, double x, double y, double z);
void rotationFirstDerive_dz(tMatrix& output, double x, double y, double z);

void rotationSecondDerive_dxdx(tMatrix& output, double x, double y, double z);
void rotationSecondDerive_dxdy(tMatrix& output, double x, double y, double z);
void rotationSecondDerive_dxdz(tMatrix& output, double x, double y, double z);

void rotationSecondDerive_dydx(tMatrix& output, double x, double y, double z);
void rotationSecondDerive_dydy(tMatrix& output, double x, double y, double z);
void rotationSecondDerive_dydz(tMatrix& output, double x, double y, double z);

void rotationSecondDerive_dzdx(tMatrix& output, double x, double y, double z);
void rotationSecondDerive_dzdy(tMatrix& output, double x, double y, double z);
void rotationSecondDerive_dzdz(tMatrix& output, double x, double y, double z);

//thirdDerive

tMatrix3d xRotationDeriv(double x);