#include "ExpMapRotMat.h"
#include <iostream>
#define THETA_EPS 1e-3
btGenExpMapRotation::btGenExpMapRotation()
{
    mAxis.setZero();
    mAxis[0] += 1e-10;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            maidaj(i, j) = (i == j);
        }
    mdRdq.resize(3);
    mdR2dq2.resize(3);
    for (int i = 0; i < 3; i++)
    {
        mdR2dq2[i].resize(i + 1);
    }
    for (auto &x : this->mdR2dq2)
        SetAxis(mAxis);
}

tVector btGenExpMapRotation::GetAxis() const { return mAxis; }
void btGenExpMapRotation::SetAxis(const tVector &axis)
{
    mAxis = axis;
    mAxis[3] = 0;

    if (mAxis.norm() < 1e-10)
        mAxis[0] += 1e-10;
    // update coefficients
    mt = mAxis.norm();
    mt2 = mt * mt;
    mt3 = mt2 * mt;
    mt4 = mt3 * mt;
    mt5 = mt4 * mt;
    mt6 = mt5 * mt;
    mt7 = mt6 * mt;

    mst = std::sin(mt);
    mct = std::cos(mt);

    mSkewA = btMathUtil::VectorToSkewMat(mAxis);
    mSkewA2 = mSkewA * mSkewA;

    // calculate rotation
    CalcRotation();

    // caculate the first order jacobian
    CalcdRdq();
    CalcdR2dq2();
}

void btGenExpMapRotation::CalcdR2dq2()
{
    tMatrix part1 = tMatrix::Zero(), part2 = tMatrix::Zero(),
            part3 = tMatrix::Zero(), part4 = tMatrix::Zero();
    double t = mt, t2 = mt2, st = mst, ct = mct;
    double coef1 = 0, coef2 = 0;
    for (int i = 0; i < 3; i++)
    {
        mdR2dq2[i].resize(i + 1);
        for (int j = 0; j <= i; j++)
        {
            double ai = mAxis[i], aj = mAxis[j];
            {

                if (std::fabs(mt) < THETA_EPS)
                {
                    // BTGEN_ASSERT(false);
                    // t is very small
                    coef1 = 1 / 15;
                    coef2 = -1 / 3;
                }
                else
                {
                    coef1 = ((3 - t2) * st - 3 * t * ct) / (mt5);
                    coef2 = (t * ct - st) / (mt3);
                }
                part1 = coef1 * ai * aj * mSkewA +
                        coef2 * (ai * btMathUtil::SkewMatFirstDeriv(mAxis, j) +
                                 maidaj(i, j) * mSkewA);
            }
            {
                if (std::fabs(mt) < THETA_EPS)
                {
                    // BTGEN_ASSERT(false);
                    // t is very small
                    coef1 = -1 / 3;
                }
                else
                {
                    coef1 = (t * ct - st) / (mt3);
                }
                part2 = coef1 * aj * btMathUtil::SkewMatFirstDeriv(mAxis, i);
            }
            {
                if (std::fabs(mt) < THETA_EPS)
                {
                    // BTGEN_ASSERT(false);
                    // t is very small
                    coef1 = 1 / 90;
                    coef2 = -1 / 12;
                }
                else
                {
                    coef1 = ((t2 - 8) * ct - 5 * t * st + 8) / (mt6);
                    coef2 = (t * st - 2 + 2 * ct) / (mt4);
                }
                part3 = coef1 * ai * aj * mSkewA2 +
                        coef2 * (maidaj(i, j) * mSkewA2 +
                                 ai * btMathUtil::SkewMat2FirstDeriv(mAxis, j));
            }
            {
                if (std::fabs(mt) < THETA_EPS)
                {
                    // BTGEN_ASSERT(false);
                    // t is very small
                    coef1 = -1 / 12;
                    coef2 = 1 / 2;
                }
                else
                {
                    coef1 = (t * st - 2 * (1 - ct)) / (mt4);
                    coef2 = (1 - ct) / t2;
                }
                part4 = coef1 * aj * btMathUtil::SkewMat2FirstDeriv(mAxis, i) +
                        coef2 * btMathUtil::SkewMat2SecondDeriv(mAxis, i, j);
            };

            mdR2dq2[i][j] = part1 + part2 + part3 + part4;
        }
    }
}
void btGenExpMapRotation::CalcdRdq()
{
    for (int i = 0; i < 3; i++)
    {
        if (std::fabs(mt) < THETA_EPS)
        {
            // BTGEN_ASSERT(false);
            // the mt is very small
            mdRdq[i] = -1 / 3 * mAxis[i] * mSkewA +
                       btMathUtil::SkewMatFirstDeriv(mAxis, i) -
                       1 / 12 * mAxis[i] * mSkewA2 +
                       1 / 2 * btMathUtil::SkewMat2FirstDeriv(mAxis, i);
        }
        else
        {
            tVector ei = tVector::Zero();
            ei[i] = 1;
            mdRdq[i] = (mAxis[i] * mSkewA +
                        btMathUtil::VectorToSkewMat(mAxis.cross3(
                            (tMatrix::Identity() - GetRotation()) * ei))) *
                       GetRotation() / mt2;
        }
    }
}

void btGenExpMapRotation::CalcRotation()
{
    if (std::fabs(mt) < THETA_EPS)
    {
        // BTGEN_ASSERT(false);
        mRotation = tMatrix::Identity() + mSkewA + 1 / 2 * mSkewA2;
    }
    else
    {
        mRotation =
            tMatrix::Identity() + mst / mt * mSkewA + (1 - mct) / mt2 * mSkewA2;
    }
}
/**
 * \brief           Get the rot mat of this exp map
 * 
*/
tMatrix btGenExpMapRotation::GetRotation() const { return mRotation; }
/**
 * \brief           Get the dR/dqi of this exp map
*/

tMatrix btGenExpMapRotation::GetFirstDeriv(int i) const { return mdRdq[i]; }

tMatrix btGenExpMapRotation::GetFirstDeriv_method1(int i) const
{
    // =================== old method =================
    /*
        let t =  theta
        dRdai = 
             (t * cos(t) - sin(t))/ (t^3) * ai * [a]
        +    sin(t)/t * d[a]/dqi
        +    (t * sin(t) - 2 ( 1 - cos(t))) / (t^4) * ai * [a]^2
        +    (1 - cos(t))/(t^2) * d[a]^2/dai
    */
    double ai = mAxis[i];
    return (mt * mct - mst) / (mt3)*ai * mSkewA +
           mst / mt * btMathUtil::SkewMatFirstDeriv(mAxis, i) +
           (mt * mst - 2 * (1 - mct)) / mt4 * ai * mSkewA2 +
           (1 - mct) / (mt2)*btMathUtil::SkewMat2FirstDeriv(mAxis, i);
}
tMatrix btGenExpMapRotation::GetFirstDeriv_method2(int i) const
{
    // =================== new method ==================
    /*
        1. get the jacobian
        2. get the ith column, scatter it to the skew matrix W
        3. dRdq = W * R
    */
    tMatrix jac = GetFirstJacobian();
    return btMathUtil::VectorToSkewMat(jac.col(i)) * GetRotation();
}

/**
 * \brief           Get the dR^2/dqiqj of this exp map
 * 
 * let t = theta
 * (d^2R)/d(ai aj) = 
 *          ((3 - t^2) * sin(t) - 3 * t * cos(t))/ (t^5) * ai * aj * [a]
 * +        (t * cos(t) - sin(t))/(t^3) * (aj * d[a]/dai + ai * d[a]/daj)
 * +        ((t^2 - 8) * cos(t) - 5 * t * sin(t) + 8)/(t^6) * ai * aj * [a]^2
 * +        (t * sin(t) - 2 * (1 - cos(t)))/(t^4) * (aj * d[a]^2/dai + ai * d[a]^2/daj)
 * +        (1 - cos(t))/(t^2) * (d^2[a]^2)/(daiaj)
*/
tMatrix btGenExpMapRotation::GetSecondDeriv(int i, int j) const
{
    // the storage require i>=j
    if (i < j)
        std::swap(i, j);
    return mdR2dq2[i][j];
}

/**
 * 
*/
tMatrix btGenExpMapRotation::GetFirstDerivPart1(int i) const
{
    return (mt * mct - mst) / (mt3)*mAxis[i] * mSkewA;
}

/**
 * \brief   
 * 
*/
tMatrix btGenExpMapRotation::GetFirstDerivPart1_1(int i) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct, t3 = mt * mt * mt;
    double ai = mAxis[i];
    return ai * (t * ct - st) / t3 * mSkewA;
    // return mSkewA * ai * mTheta;
    // return ai;
    // return mTheta;
}

/**
 * \brief               get value of d(ai/theta + theta * d[a]/dai)/daj
 * 
*/
tMatrix btGenExpMapRotation::GetSecondDerivPart1_1(int i, int j) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i], aj = mAxis[j];
    tMatrix res = ((3 - t2) * st - 3 * t * ct) / (mt5)*ai * aj * mSkewA;
    return res;
}

tMatrix btGenExpMapRotation::GetSecondDerivPart1_2(int i, int j) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i], aj = mAxis[j];
    return (t * ct - st) / (mt3) *
           (ai * btMathUtil::SkewMatFirstDeriv(mAxis, j) +
            maidaj(i, j) * mSkewA);
}

tMatrix btGenExpMapRotation::GetFirstDerivPart2(int i) const
{
    return mst / mt * btMathUtil::SkewMatFirstDeriv(mAxis, i);
}
tMatrix btGenExpMapRotation::GetFirstDerivPart3(int i) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i];
    return ai * (t * st - 2 + 2 * ct) / (mt4)*mSkewA2;
}
tMatrix btGenExpMapRotation::GetSecondDerivPart1(int i, int j) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i], aj = mAxis[j];

    return GetSecondDerivPart1_1(i, j) + GetSecondDerivPart1_2(i, j);
}
tMatrix btGenExpMapRotation::GetSecondDerivPart2(int i, int j) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i], aj = mAxis[j];
    return (t * ct - st) / (mt3)*aj * btMathUtil::SkewMatFirstDeriv(mAxis, i);
}

tMatrix btGenExpMapRotation::GetSecondDerivPart3(int i, int j) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i], aj = mAxis[j];
    return ((t2 - 8) * ct - 5 * t * st + 8) / (mt6)*ai * aj * mSkewA2 +
           (t * st - 2 + 2 * ct) / (mt4) *
               (maidaj(i, j) * mSkewA2 +
                ai * btMathUtil::SkewMat2FirstDeriv(mAxis, j));
}

tMatrix btGenExpMapRotation::GetSecondDerivPart3_1(int i, int j) const
{

    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i], aj = mAxis[j];
    return ((t2 - 8) * ct - 5 * t * st + 8) / (mt6)*ai * aj * mSkewA2;
}

tMatrix btGenExpMapRotation::GetSecondDerivPart3_1R(int i, int j) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i], aj = mAxis[j];
    // return ((t2 - 8) * ct - 5 * t * st + 8) / (mt6) *
    //        tMatrix::Identity();
    // return (1 / mt6) * tMatrix::Identity();
    return (1 / t) * tMatrix::Identity();
}

tMatrix btGenExpMapRotation::GetSecondDerivPart3_2(int i, int j) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i], aj = mAxis[j];
    return (t * st - 2 + 2 * ct) / (mt4) *
           (maidaj(i, j) * mSkewA2 +
            ai * btMathUtil::SkewMat2FirstDeriv(mAxis, j));
}
tMatrix btGenExpMapRotation::GetFirstDerivPart4(int i) const
{
    return (1 - mct) / (mt2)*btMathUtil::SkewMat2FirstDeriv(mAxis, i);
}
tMatrix btGenExpMapRotation::GetSecondDerivPart4(int i, int j) const
{
    double t = mt, t2 = mt * mt, st = mst, ct = mct;
    double ai = mAxis[i], aj = mAxis[j];

    return (t * st - 2 * (1 - ct)) / (mt4)*aj *
               btMathUtil::SkewMat2FirstDeriv(mAxis, i) +
           (1 - ct) / t2 * btMathUtil::SkewMat2SecondDeriv(mAxis, i, j);
}

tMatrix btGenExpMapRotation::GetFirstJacobian() const
{
    return GetRotation() *
           (tMatrix::Identity() - (1 - mct) / (mt * mt) * mSkewA +
            ((mt - mst) / (mt * mt * mt)) * mSkewA2);
}
/**
 * \brief           Get the dR^3/d(qiqjqk) of this exp map
*/
tMatrix btGenExpMapRotation::GetThirdDeriv(int i, int j, int k) const
{
    return GetThirdDerivPart1(i, j, k) + GetThirdDerivPart2(i, j, k) +
           GetThirdDerivPart3(i, j, k) + GetThirdDerivPart4(i, j, k);
}

/**
 * \brief           Test the exp map
*/
void btGenExpMapRotation::Test()
{
    // btMathUtil::TestSkewMatDeriv();
    // TestSimpleFormula();
    // TestFirstDeriv();
    // TestSecondDeriv();
    TestThirdDeriv();
}

void btGenExpMapRotation::TestFirstDeriv()
{
    tVectorXd old_axis = mAxis;
    double eps = 1e-6;
    {
        tEigenArr<tMatrix> dRdq(3);
        for (int i = 0; i < 3; i++)
            dRdq[i] = GetFirstDeriv(i);

        tMatrix old_rot = GetRotation();
        for (int i = 0; i < 3; i++)
        {
            mAxis[i] += eps;
            SetAxis(mAxis);

            tMatrix new_rot = GetRotation();
            tMatrix dRdq_num = (new_rot - old_rot) / eps;
            // std::cout << "dRdq" << i << " = \n" << dRdq_num << std::endl;
            tMatrix diff = dRdq_num - dRdq[i];
            BTGEN_ASSERT(diff.norm() < 10 * eps);
            mAxis[i] -= eps;
            SetAxis(mAxis);
        }
        std::cout << "[debug] ExpMap Test dRdq succ\n";
    }
    SetAxis(old_axis);
}
void btGenExpMapRotation::TestSecondDeriv()
{
    tVector old_axis = mAxis;
    double eps = 1e-5;
    // test second order derivative
    {
        tEigenArr<tMatrix> old_dRdq(3);
        for (int i = 0; i < 3; i++)
            old_dRdq[i] = GetFirstDeriv(i);

        tEigenArr<tEigenArr<tMatrix>> ddRdqiqj(3);
        for (int i = 0; i < 3; i++)
        {
            ddRdqiqj[i].resize(3);
            for (int j = 0; j < 3; j++)
            {
                ddRdqiqj[i][j] = GetSecondDeriv(i, j);
            }
        }

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                mAxis[j] += eps;
                SetAxis(mAxis);
                tEigenArr<tMatrix> new_dRdq(3);
                for (int k = 0; k < 3; k++)
                    new_dRdq[k] = GetFirstDeriv(k);
                tMatrix num = (new_dRdq[i] - old_dRdq[i]) / eps;
                double diff = (num - ddRdqiqj[i][j]).norm();
                // double diff = std::fabs(num - ddRdqiqj[i][j]);
                if (diff > 10 * eps)
                {
                    std::cout << "fot i = " << i << " j = " << j << std::endl;
                    std::cout << "ddR ana = \n" << ddRdqiqj[i][j] << std::endl;
                    std::cout << "ddR num = \n" << num << std::endl;
                    exit(0);
                }
                else
                {
                    printf("[debug] ExpMap Test ddRdq%dq%d succ\n", i, j);
                }

                mAxis[j] -= eps;
                SetAxis(mAxis);
            }
        }
        std::cout << "[debug] ExpMap Test ddRdqiqj succ\n";
    }
    SetAxis(old_axis);
}

void btGenExpMapRotation::TestThirdDeriv()
{
    tVector old_axis = mAxis;
    // 1. given old second order deriv
    tEigenArr<tEigenArr<tMatrix>> old_second_deriv(3);
    for (int i = 0; i < 3; i++)
    {
        old_second_deriv[i].resize(3);
        for (int j = 0; j < 3; j++)
        {
            old_second_deriv[i][j] = GetSecondDeriv(i, j);
        }
    }

    // 2. calculate third order deriv
    tEigenArr<tEigenArr<tEigenArr<tMatrix>>> third_deriv(3);
    for (int i = 0; i < 3; i++)
    {
        third_deriv[i].resize(3);
        for (int j = 0; j < 3; j++)
        {
            third_deriv[i][j].resize(3);
            for (int k = 0; k < 3; k++)
            {
                third_deriv[i][j][k].noalias() = GetThirdDeriv(i, j, k);
            }
        }
    }

    // 3. begin to compare
    double eps = 1e-6;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
            {
                // begin to test d(R^3)/(dqiqjqk) = (f(i, j, k + dk) -f(i, j, k)) / dk
                const tMatrix &deriv_ana = third_deriv[i][j][k];
                mAxis[k] += eps;
                SetAxis(mAxis);

                const tMatrix &new_second_deriv = GetSecondDeriv(i, j);
                const tMatrix &deriv_num =
                    (new_second_deriv - old_second_deriv[i][j]) / eps;

                tMatrix diff = deriv_num - deriv_ana;
                double diff_norm = diff.norm();
                if (diff_norm > eps)
                {
                    printf("[error] test dR3/dq%dq%dq%d failed\n", i, j, k);
                    std::cout << "diff = \n" << diff << std::endl;
                    std::cout << "deriv num = \n" << deriv_num << std::endl;
                    std::cout << "deriv ana = \n" << deriv_ana << std::endl;
                    std::cout << "diff norm = " << diff_norm << std::endl;
                    exit(0);
                }
                // printf("[log] test dR3/dq%dq%dq%d succ\n", i, j, k);
                mAxis[k] -= eps;
            }
    SetAxis(old_axis);
}
/**
 * \brief           dR/dq = R * (I3 - (1-cos(t))/t^2 * [q] + (t - sin(t))/t^3 * [q]^2) * d[q]dq
*/
void btGenExpMapRotation::TestSimpleFormula()
{
    tMatrix jac = GetFirstJacobian();

    for (int i = 0; i < 3; i++)
    {
        tMatrix dRdqi = btMathUtil::VectorToSkewMat(jac.col(i)) * GetRotation();
        tMatrix dRdqi_ideal = GetFirstDeriv(i);
        tMatrix diff = dRdqi_ideal - dRdqi;
        // std::cout << 'diff = \n' << diff << std::endl;
    }
    // exit(0);
}

/**
 * \brief       input axis vector (united), return the one id
*/
int btGenExpMapRotation::GetFreedomAxisId(const tVector3d &axis)
{
    BTGEN_ASSERT(std::fabs(axis.norm() - 1) < 1e-10);
    BTGEN_ASSERT(axis.minCoeff() >= 0);
    for (int i = 0; i < 3; i++)
    {
        if (std::fabs(axis[i] - 1) < 1e-10)
            return i;
    }
    BTGEN_ASSERT(false);
    return -1;
}

tMatrix btGenExpMapRotation::GetThirdDerivPart1_1(int i, int j, int k) const
{
    double t2 = mt * mt, t3 = mt * mt * mt;
    double ai = mAxis[i], aj = mAxis[j], ak = mAxis[k];
    tMatrix p1 = (6 * t2 * mst - t3 * mct + 15 * (mt * mct - mst)) /
                 (std::pow(mt, 7)) * ai * aj * ak * mSkewA;

    tMatrix p2 = ((3 - t2) * mst - 3 * mt * mct) / (mt5) *
                 ((maidaj(i, k) * aj + ai * maidaj(j, k)) * mSkewA +
                  ai * aj * btMathUtil::SkewMatFirstDeriv(mAxis, k));
    return p1 + p2;
}
tMatrix btGenExpMapRotation::GetThirdDerivPart1_2(int i, int j, int k) const
{
    double t2 = mt * mt, t3 = mt * mt * mt;
    double ai = mAxis[i], aj = mAxis[j], ak = mAxis[k];
    return ((3 - t2) * mst - 3 * mt * mct) / (mt5) *
               (ai * ak * btMathUtil::SkewMatFirstDeriv(mAxis, j) +
                ak * maidaj(i, j) * mSkewA) +
           (mt * mct - mst) / t3 *
               (maidaj(i, k) * btMathUtil::SkewMatFirstDeriv(mAxis, j) +
                maidaj(i, j) * btMathUtil::SkewMatFirstDeriv(mAxis, k));
}
tMatrix btGenExpMapRotation::GetThirdDerivPart1(int i, int j, int k) const
{
    double t2 = mt * mt, t3 = mt * mt * mt;
    double ai = mAxis[i], aj = mAxis[j], ak = mAxis[k];

    double coef1 = 0, coef2 = 0, coef3 = 0;
    if (std::fabs(mt) < THETA_EPS)
    {
        // mt \approx 0
        coef1 = -1 / 105;
        coef2 = 1 / 15;
        coef3 = -1 / 3;
    }
    else
    {
        coef1 = (6 * t2 * mst - t3 * mct + 15 * (mt * mct - mst)) /
                (std::pow(mt, 7));
        coef2 = ((3 - t2) * mst - 3 * mt * mct) / (mt5);
        coef3 = (mt * mct - mst) / t3;
    }
    tMatrix p1 = coef1 * ai * aj * ak * mSkewA;

    tMatrix p2 =
        coef2 *
        ((maidaj(i, k) * aj + ai * maidaj(j, k) + ak * maidaj(i, j)) * mSkewA +
         ai * aj * btMathUtil::SkewMatFirstDeriv(mAxis, k) +
         ai * ak * btMathUtil::SkewMatFirstDeriv(mAxis, j));

    tMatrix p4 =
        coef3 * (maidaj(i, k) * btMathUtil::SkewMatFirstDeriv(mAxis, j) +
                 maidaj(i, j) * btMathUtil::SkewMatFirstDeriv(mAxis, k));
    return p1 + p2 + p4;
}
tMatrix btGenExpMapRotation::GetThirdDerivPart2(int i, int j, int k) const
{
    double t2 = mt * mt, t3 = mt * mt * mt;
    double ai = mAxis[i], aj = mAxis[j], ak = mAxis[k];

    double coef1 = 0, coef2 = 0;
    if (std::fabs(mt) < THETA_EPS)
    {
        // mt \approx 0
        coef1 = 1 / 15;
        coef2 = -1 / 3;
    }
    else
    {
        coef1 = ((3 - t2) * mst - 3 * mt * mct) / (mt5);
        coef2 = (mt * mct - mst) / t3;
    }

    return coef1 * ak * aj * btMathUtil::SkewMatFirstDeriv(mAxis, i) +
           coef2 * maidaj(j, k) * btMathUtil::SkewMatFirstDeriv(mAxis, i);
}
tMatrix btGenExpMapRotation::GetThirdDerivPart3_1(int i, int j, int k) const
{
    double t2 = mt * mt, t3 = mt * mt * mt, t5 = mt5, t4 = mt4, t6 = mt6,
           t8 = std::pow(mt, 8);
    double ai = mAxis[i], aj = mAxis[j], ak = mAxis[k];
    double coef1 = 0, coef2 = 0;
    if (std::fabs(mt) < THETA_EPS)
    {
        // mt \approx 0
        coef1 = -1 / 840;
        coef2 = 1 / 90;
    }
    else
    {
        coef1 = -((9 * t2 - 48) * mct + (t3 - 33 * mt) * mst + 48) / t8;
        coef2 = ((t2 - 8) * mct - 5 * mt * mst + 8) / (mt6);
    }
    return coef1 * ak * ai * aj * mSkewA2 +
           coef2 * ((maidaj(i, k) * aj + maidaj(j, k) * ai) * mSkewA2 +
                    ai * aj * btMathUtil::SkewMat2FirstDeriv(mAxis, k));
}

tMatrix btGenExpMapRotation::GetThirdDerivPart3_1R(int i, int j, int k) const
{
    double t2 = mt * mt, t3 = mt * mt * mt, t5 = mt5, t4 = mt4, t6 = mt6,
           t8 = std::pow(mt, 8);
    double ai = mAxis[i], aj = mAxis[j], ak = mAxis[k];
    double method_1 = -((9 * t2 - 48) * mct + (t3 - 33 * mt) * mst + 48) / t8;
    // return method_1 * tMatrix::Identity();
    return (-(6 * (mct * (t2 - 8) - 5 * mt * mst + 8)) / t8 -
            (5 * mst + mst * (t2 - 8) + 3 * mt * mct) / t6) *
           tMatrix::Identity();
    // return -6 / t7 * tMatrix::Identity();
    // return -1 / t2 * tMatrix::Identity();
}
tMatrix btGenExpMapRotation::GetThirdDerivPart3_2(int i, int j, int k) const
{
    double t2 = mt * mt, t3 = mt * mt * mt;
    double ai = mAxis[i], aj = mAxis[j], ak = mAxis[k];
    double coef1 = 0, coef2 = 0;

    if (std::fabs(mt) < THETA_EPS)
    {
        // theta is small
        coef1 = 1 / 90;
        coef2 = -1 / 12;
    }
    else
    {
        coef1 = ((t2 - 8) * mct - 5 * mt * mst + 8) / (mt6);
        coef2 = (mt * mst - 2 * (1 - mct)) / mt4;
    }
    return coef1 * ak *
               (ai * btMathUtil::SkewMat2FirstDeriv(mAxis, j) +
                maidaj(i, j) * mSkewA2)

           + coef2 * (maidaj(i, k) * btMathUtil::SkewMat2FirstDeriv(mAxis, j) +
                      ai * btMathUtil::SkewMat2SecondDeriv(mAxis, j, k) +
                      maidaj(i, j) * btMathUtil::SkewMat2FirstDeriv(mAxis, k));
}
tMatrix btGenExpMapRotation::GetThirdDerivPart3(int i, int j, int k) const
{
    return GetThirdDerivPart3_1(i, j, k) + GetThirdDerivPart3_2(i, j, k);
}
tMatrix btGenExpMapRotation::GetThirdDerivPart4(int i, int j, int k) const
{
    double t2 = mt * mt, t3 = mt * mt * mt;
    double ai = mAxis[i], aj = mAxis[j], ak = mAxis[k];
    double coef1 = 0, coef2 = 0, coef3 = 0;

    if (std::fabs(mt) < THETA_EPS)
    {
        // theta is small
        coef1 = 1 / 90;
        coef2 = -1 / 12;
        coef3 = -1 / 12;
    }
    else
    {
        coef1 = ((t2 - 8) * mct - 5 * mt * mst + 8) / (mt6);
        coef2 = (mt * mst - 2 * (1 - mct)) / mt4;
        coef3 = (mt * mst - 2 * (1 - mct)) / mt4;
    }
    tMatrix p1 = coef1 * ak * aj * btMathUtil::SkewMat2FirstDeriv(mAxis, i);
    tMatrix p2 =
        coef2 * (maidaj(j, k) * btMathUtil::SkewMat2FirstDeriv(mAxis, i) +
                 aj * btMathUtil::SkewMat2SecondDeriv(mAxis, i, k));
    tMatrix p3 = coef3 * ak * btMathUtil::SkewMat2SecondDeriv(mAxis, i, j);
    return p1 + p2 + p3;
}