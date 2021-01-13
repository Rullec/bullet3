#pragma once
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

/**
 * \brief           the class for exponential map rotation
 * 
 *          Given an axix-angle (axis * theta, in another word, the "exponential map" representation)
 *          Calc the rotmat
 *          Calc the d(rotmat)/daa
 *          Calc the d^2(rotmat)/daa^2
 *          Calc the d^3(rotmat)/daa^3
*/
class btGenExpMapRotation
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenExpMapRotation();

    void SetAxis(const tVector &axis);
    tVector GetAxis() const;
    tMatrix GetRotation() const;
    tMatrix GetFirstDeriv(int i) const;
    tMatrix GetSecondDeriv(int i, int j) const;
    tMatrix GetThirdDeriv(int i, int j, int k) const;
    tMatrix GetFirstJacobian() const;
    void Test();
    static int GetFreedomAxisId(const tVector3d &axis);

protected:
    tVector mAxis; // unnormalized exp map (axis * theta), a
    double mt, mt2, mt3, mt4, mt5, mt6, mt7; // theta, theta^2, theta^3, theta^4
    double mst;                              // sin theta
    double mct;                              // cos the
    tMatrix mSkewA;                          // [a]
    tMatrix mSkewA2;                         // [a]^2
    tMatrix mRotation;                       // rotation
    tEigenArr<tMatrix> mdRdq;                // dRdq
    tEigenArr<tEigenArr<tMatrix>>
        mdR2dq2; // dR^2dqiqj (only stored the lower triangle)

    tMatrix3d maidaj;
    tMatrix GetFirstDeriv_method1(int i) const;
    tMatrix GetFirstDeriv_method2(int i) const;
    tMatrix GetFirstDerivPart1_1(int i) const;
    tMatrix GetSecondDerivPart1_1(int i, int j) const;
    tMatrix GetSecondDerivPart1_2(int i, int j) const;
    tMatrix GetFirstDerivPart1(int i) const;
    tMatrix GetFirstDerivPart2(int i) const;
    tMatrix GetFirstDerivPart3(int i) const;
    tMatrix GetFirstDerivPart4(int i) const;
    tMatrix GetSecondDerivPart1(int i, int j) const;
    tMatrix GetSecondDerivPart2(int i, int j) const;
    tMatrix GetSecondDerivPart3(int i, int j) const;
    tMatrix GetSecondDerivPart3_1(int i, int j) const;
    tMatrix GetSecondDerivPart3_1R(int i, int j) const;
    tMatrix GetSecondDerivPart3_2(int i, int j) const;
    tMatrix GetSecondDerivPart4(int i, int j) const;

    tMatrix GetThirdDerivPart1(int i, int j, int k) const;
    tMatrix GetThirdDerivPart1_1(int i, int j, int k) const;
    tMatrix GetThirdDerivPart1_2(int i, int j, int k) const;
    tMatrix GetThirdDerivPart2(int i, int j, int k) const;
    tMatrix GetThirdDerivPart3(int i, int j, int k) const;
    tMatrix GetThirdDerivPart3_1(int i, int j, int k) const;
    tMatrix GetThirdDerivPart3_1R(int i, int j, int k) const;
    tMatrix GetThirdDerivPart3_2(int i, int j, int k) const;
    tMatrix GetThirdDerivPart4(int i, int j, int k) const;
    void CalcRotation();
    void CalcdRdq();
    void CalcdR2dq2();
    void TestSimpleFormula();
    void TestFirstDeriv();
    void TestSecondDeriv();
    void TestThirdDeriv();
};