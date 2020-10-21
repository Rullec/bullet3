#include "LQRController.h"
#include "riccati_solver.h"

cLQRController::cLQRController(cRobotModelDynamics *model, std::string policy)
    : cInvertPendulumController(model), mPolicy(policy)
{
    mA.resize(0, 0);
    mB.resize(0, 0);
    auto root_joint = mModel->GetJointById(0);
    if (root_joint->GetJointType() != JointType::LIMIT_NONE_JOINT)
    {
        std::cout << "[error] it is not limit none root joint \n";
        exit(0);
    }
}

/**
 * \brief           calculate & apply the lqr control force
*/
void cLQRController::LQRControl(const tMatrixXd &K)
{
    int dof = this->mModel->GetNumOfFreedom();

    tVectorXd x = GetState();
    double u = (-K * x)[0];
    double f_max = 100;
    if (std::fabs(u) > f_max)
        u = u / std::fabs(u) * f_max;
    std::cout << "[" << mPolicy << "] u = " << u << std::endl;

    tVector3d f = tVector3d(0, 0, u);

    mModel->ApplyForce(
        0, btMathUtil::Expand(f, 0),
        btMathUtil::Expand(mModel->GetLinkById(0)->GetWorldPos(), 1));
}

/**
 * \brief           main control cycle
*/
void cLQRController::UpdateCtrl(double dt)
{
    // get state equation
    GetSystem(mA, mB);

    // coef
    tMatrixXd Q, R;
    GetCoef(Q, R);

    // solve ricattic
    tMatrixXd P;
    solveRiccatiArimotoPotter(mA, mB, Q, R, P);
    tMatrixXd K = R.inverse() * mB.transpose() * P;

    // apply lqr
    LQRControl(K);
}

/**
 * \brief           Get the system state eqaution
*/
void cLQRController::GetSystem(tMatrixXd &A, tMatrixXd &B)
{
    int dof = this->mModel->GetNumOfFreedom();
    int x_size = 2 * dof;
    if (mPolicy == "zero_point_lagragian")
    {
        if (mA.rows() != x_size)
            GetSystemLagragianZeroPoint(A, B);
    }
    else if (mPolicy == "simple")
    {
        GetSystemSimple(A, B);
    }
    else if (mPolicy == "lagragian")
    {
        GetSystemLagragian(A, B);
    }
    else
    {
        std::cout << "unsupported policy " << mPolicy << std::endl;
        exit(0);
    }
}

/**
 * \brief               Get the system state equation by lagragian equation, but stand at the zero point of the system
*/
void cLQRController::GetSystemLagragianZeroPoint(tMatrixXd &A, tMatrixXd &B)
{
    mModel->PushState("lqr");
    int dof = this->mModel->GetNumOfFreedom();
    mModel->SetqAndqdot(tVectorXd::Ones(dof), tVectorXd::Zero(dof));
    int x_size = 2 * dof;
    // 1. A part1
    tMatrixXd A_part1;
    tMatrixXd A_part2;

    tMatrixXd M = mModel->GetMassMatrix();
    tMatrixXd Minv = mModel->GetInvMassMatrix();
    tMatrixXd C = mModel->GetCoriolisMatrix();
    const tVector g = tVector(0, -9.8, 0, 0);
    tVectorXd QG = mModel->CalcGenGravity(g);
    tMatrixXd dQGdq = mModel->CalcdGenGravitydq(g);
    tEigenArr<tMatrixXd> dMdq;
    mModel->ComputedMassMatrixdq(dMdq);

    // mModel->GetMassMatrix
    {
        A_part1 = tMatrixXd::Zero(2 * dof, 2 * dof);
        A_part1.block(0, dof, dof, dof).setIdentity();
        A_part1.block(dof, dof, dof, dof) = -Minv * C;
        // std::cout << "A part 1 = \n" << A_part1 << std::endl;
        // std::cout << "C = \n" << C << std::endl;
    }
    // 2. A part2
    {
        A_part2 = tMatrixXd::Zero(2 * dof, 2 * dof);
        // 2.1 calculate the dMinv dx = -Minv * dMdx * Minv
        tEigenArr<tMatrixXd> dMinvdx(2 * dof, tMatrixXd::Zero(dof, dof));
        for (int i = 0; i < 2 * dof; i++)
        {
            // only dMinvdq is valid
            // all others are dMinvdqdot, all zero
            if (i < dof)
            {
                // std::cout << "dMdq " << i << " = \n" << dMdq[i] << std::endl;
                dMinvdx[i] = -Minv * dMdq[i] * Minv;
            }
        }

        // 2.2 calculate (dMinvdx * QG + Minv * dQGdx), shape = n * 2n
        for (int i = 0; i < 2 * dof; i++)
        {
            // std::cout << "dMinvdx " << i << " = \n" << dMinvdx[i] << std::endl;
            // std::cout << "QG = " << QG.transpose() << std::endl;
            tMatrixXd res = dMinvdx[i] * QG;
            A_part2.block(dof, i, dof, 1) += res;
        }
        A_part2.block(dof, 0, dof, dof) += Minv * dQGdq;
    }
    A = A_part1 + A_part2;
    // 3. B
    B = tMatrixXd::Zero(2 * dof, 1);

    tMatrixXd root_jkvT = mModel->GetLinkById(0)->GetJKv().transpose();
    tVector3d n = tVector3d(0, 0, 1);
    B.block(dof, 0, dof, 1) = Minv * root_jkvT * n;

    mModel->PopState("lqr");
}

void cLQRController::GetCoef(tMatrixXd &Q, tMatrixXd &R)
{
    Q = tMatrixXd::Zero(4, 4);
    R = tMatrixXd::Zero(1, 1);
    Q(0, 0) = 100;
    Q(1, 1) = 100;
    Q(2, 2) = 100;
    Q(3, 3) = 100;
    R(0, 0) = 1;
}

/**
 * \brief           Get the system state law A and B by simple physic
*/
void cLQRController::GetSystemSimple(tMatrixXd &A, tMatrixXd &B)
{
    auto root = mModel->GetLinkById(0);
    auto pole = mModel->GetLinkById(1);
    double M = root->GetMass();
    double m = pole->GetMass();

    double g = 9.8;
    double l = pole->GetMeshScale().maxCoeff() / 2;
    double I = pole->GetInertiaTensorBody().diagonal()[0];
    // std::cout << "inertia = \n" << pole->GetInertiaTensorBody() << std::endl;
    // std::cout << "I = " << I << std::endl;
    // exit(0);
    double p = I * (m + M) + M * m * l * l;
    double mu = 0.0;
    A = tMatrixXd::Zero(4, 4);
    A(0, 1) = 1;
    A(1, 1) = -(I + m * l * l) * mu / p;
    A(1, 2) = m * m * g * l * l / p;
    A(2, 3) = 1;
    A(3, 1) = -m * l * mu / p;
    A(3, 2) = m * g * l * (M + m) / p;
    B = tMatrixXd::Zero(4, 1);
    B(1, 0) = (I + m * l * l) / p;
    B(3, 0) = m * l / p;
}

/**
 * \brief               Get current working state
*/
tVectorXd cLQRController::GetState()
{
    tVectorXd q = mModel->Getq(), qdot = mModel->Getqdot();
    tVectorXd x = tVectorXd::Zero(4);
    if (mPolicy == "zero_point_lagragian")
    {
        x.segment(0, 2) = mModel->Getq();
        x.segment(2, 2) = mModel->Getqdot();
    }
    else if (mPolicy == "simple")
    {
        x = tVectorXd::Zero(4);
        x[0] = q[0];
        x[1] = qdot[0];
        x[2] = q[1];
        x[3] = qdot[1];
    }
    else
    {
        std::cout << "unsupported policy " << mPolicy << std::endl;
        exit(0);
    }
    return x;
}

/**
 * \brief           Get the accurate system linearzation in each time step
*/
void cLQRController::GetSystemLagragian(tMatrixXd &A, tMatrixXd &B) 
{
    
}