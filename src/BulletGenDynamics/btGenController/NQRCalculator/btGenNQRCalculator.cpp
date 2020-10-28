#include "BulletGenDynamics/btGenController/NQRCalculator/btGenNQRCalculator.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#include <iostream>

btGenNQRCalculator::tNQRFrameInfo::tNQRFrameInfo()
{
    Sk_mat.resize(0, 0);
    sk_vec.resize(0);
    Q_coef.resize(0);
    R_coef.resize(0);
    P_coef.resize(0);
}

btGenNQRCalculator::tSupposedContactPt::tSupposedContactPt(
    const cRobotModelDynamics *model, int link_id, const tVector &local_pos)
    : mLinkId(link_id), mLocalPos(local_pos), mModel(model)
{
}

tVector btGenNQRCalculator::tSupposedContactPt::GetGlobalPos()
{
    return mModel->GetLinkById(mLinkId)->GetGlobalTransform() *
           btMathUtil::Expand(this->mLocalPos, 1);
}
btGenNQRCalculator::btGenNQRCalculator()
{
    std::cout << "nqr build begin\n";
    mNQRFrameInfos.clear();
    // mSupposedContactPt.clear();
}
btGenNQRCalculator::~btGenNQRCalculator(){};

void btGenNQRCalculator::Init(btGeneralizeWorld *mWorld, const std::string conf)
{
    btGenTargetCalculator::Init(mWorld, conf);

    // init the supposed contact point

    Json::Value conf_json;
    btJsonUtil::LoadJson(conf, conf_json);
    std::string sup_pt_conf =
        btJsonUtil::ParseAsString("supposed_contact_pt_conf", conf_json);
    // InitSupposdedContactPoints(sup_pt_conf);
    // InitSupposdedContactPoints(sup_pt_conf);
}
void btGenNQRCalculator::SetTraj(btTraj *traj_)
{
    btGenTargetCalculator::SetTraj(traj_);
    traj_->CalculateLocalContactPos(mModel);

    // do precomputation for this traj
    mdt = traj_->mTimestep;
    CalcNQR();
}
void btGenNQRCalculator::SetCoef(const Json::Value &conf)
{
    std::cout << "[error] setcoef hasn't been finished yet\n";
    exit(1);
}

/**
 * \brief       Calculate the control target by NQR. These values will be used in LCP contact adaption
*/
void btGenNQRCalculator::CalcTarget(double dt, int target_frame_id,
                                    tVectorXd &tilde_qddot,
                                    tVectorXd &tilde_qdot, tVectorXd &tilde_q,
                                    tVectorXd &tilde_tau)
{
    // 1. do preparation
    PreCalcTarget(dt, target_frame_id);

    // 2. fetch the Sk_mat and sk_vec, P/R coef, G/h calculate the control policy
    {

    }

    // 3. apply the control policy AND THE GRAVITY, get the qddot, qdotnext, qnext
    {
    }
}
int btGenNQRCalculator::GetCalculatedNumOfContact() const { return -1; }
void btGenNQRCalculator::ControlByAdaptionController()
{
    std::cout << "control by adaptation controller "
              << ", ptr = " << this->mBulletGUIHelper << std::endl;

    // draw the contact points
    // DrawSupposedContactPoints();
}
void btGenNQRCalculator::Reset() {}

/**
 * \brief       calculate the NQR control policy, the final result ( Sk_mat and sk_vec ) will be used to do optimal control
 * 
 * - state size (in state equation), 2 * dof
 * - contact force size (2 points on each foot indepently, load from an external file)
 * - control force size (n-6 dof, we have proved it's equivalent to the cartesian torque)
 * 
 *              then set the control weight coefficient in NQR Info, including
 * - tVectorXd Q_coef; // state vector x close to origin
 * - tVectorXd R_coef; // control vector u close to origin
 * - tVectorXd P_coef; // control vector u minimize
 * 
 *              at last, Given the traj, calculate the active force & contact force from the mocap data
 *              Put the result in NQRInfo
 * - contact_force: in SUPPOSED contract points
 * - control force: reestimate
*/
void btGenNQRCalculator::CalcNQR()
{
    mModel->PushState("calc_nqr");
    // 1. allocation
    mNQRFrameInfos.resize(mTraj->mNumOfFrames);

    // 2. var size
    mStateSize = 2 * num_of_freedom;
    // mContactForceSize = 3 * mSupposedContactPt.size();
    auto root = mModel->GetRoot();
    // mJointControlForceSize = num_of_freedom - root->GetNumOfFreedom();
    // mTotalControlForceSize = mJointControlForceSize + mContactForceSize;

    // 3. coef and variable size setting
    for (int i = 0; i < mTraj->mNumOfFrames; i++)
    {
        auto &info = mNQRFrameInfos[i];
        info.mContactForceSize = 3 * mTraj->mContactForce[i].size();
        info.mJointControlForceSize = num_of_underactuated_freedom;
        info.mTotalControlForceSize =
            info.mContactForceSize + info.mJointControlForceSize;

        info.Q_coef = tVectorXd::Ones(mStateSize);
        info.R_coef = tVectorXd::Ones(info.mTotalControlForceSize);
        info.P_coef = tVectorXd::Ones(info.mTotalControlForceSize);
    }
    // 4. begin the main iter
    for (int frame_id = mTraj->mNumOfFrames - 1; frame_id >= 1; frame_id--)
    {
        // set q and qdot
        std::cout << "--------------- frame " << frame_id << " -------------\n";

        const tVectorXd &q = mTraj->mq[frame_id],
                        &qdot = mTraj->mqdot[frame_id];

        mModel->SetqAndqdot(q, qdot);

        CalcNQRContactAndControlForce(frame_id);
        // VerifyContactAndControlJacobian(frame_id);

        CalcNQRSystemLinearzation(frame_id);
    }

    // 3. solve the ARE (algebraic riccati equation) recursively, inversely
    CalcNQRRiccati();
    mModel->PopState("calc_nqr");
}

/**
 * \brief           Solve the algebraic riccati equation
*/
void btGenNQRCalculator::CalcNQRRiccati()
{
    int num_of_frames = this->mTraj->mNumOfFrames;
    for (int cur_frame = num_of_frames; cur_frame >= 0; cur_frame--)
    {
        if (cur_frame == num_of_frames)
        {
            // given the init S_last and s_last
        }
        else
        {
        }
    }
}

// /**
//  * \brief       calculate contact points
// */
// void btGenNQRCalculator::DrawSupposedContactPoints()
// {
//     std::cout << "suppose contact pts num = " << mSupposedContactPt.size()
//               << std::endl;
//     if (mDrawPointsList.size() == 0)
//     {
//         for (auto &pt : this->mSupposedContactPt)
//         {
//             DrawPoint(pt->GetGlobalPos().segment(0, 3));
//         }
//     }
//     else
//     {
//         for (int i = 0; i < mSupposedContactPt.size(); i++)
//         {
//             btTransform trans = mDrawPointsList[i]->getWorldTransform();
//             trans.setOrigin(btBulletUtil::tVectorTobtVector(
//                 mSupposedContactPt[i]->GetGlobalPos()));
//             std::cout << "pt " << i << " global pos = "
//                       << mSupposedContactPt[i]->GetGlobalPos().transpose()
//                       << std::endl;
//             mDrawPointsList[i]->setWorldTransform(trans);
//         }
//     }
// }

// /**
//  * \brief           Load the supposed contact points from file
// */
// void btGenNQRCalculator::InitSupposdedContactPoints(
//     const std::string &conf_path)
// {
//     mSupposedContactPt.clear();
//     Json::Value root;
//     btJsonUtil::LoadJson(conf_path, root);
//     std::string skeleton_path =
//         btJsonUtil::ParseAsString("skeleton_path", root);
//     int num = btJsonUtil::ParseAsInt("num_of_supposed_contact_points", root);
//     const Json::Value &lst =
//         btJsonUtil::ParseAsValue("supposed_contact_point_lst", root);

//     if (skeleton_path != this->mModel->GetCharFile())
//     {
//         std::cout << "[error] nqr supposed contact skeleton file inconsistent "
//                   << skeleton_path << " != " << mModel->GetCharFile()
//                   << std::endl;
//         exit(0);
//     }
//     if (num != lst.size())
//     {
//         std::cout << "[error] the supposed contact points num is "
//                      "consistent "
//                   << num << " != " << lst.size() << std::endl;
//         exit(1);
//     }

//     for (int i = 0; i < num; i++)
//     {
//         auto cur_pt = lst[i];
//         std::string link_name = btJsonUtil::ParseAsString("link_name", cur_pt);
//         int link_id = btJsonUtil::ParseAsInt("link_id", cur_pt);
//         tVector3d local_pos = btJsonUtil::ReadVectorJson(
//                                   btJsonUtil::ParseAsValue("local_pos", cur_pt))
//                                   .segment(0, 3);

//         // check link id and link name
//         auto link = mModel->GetLinkById(link_id);
//         if (link->GetName() != link_name)
//         {
//             std::cout << "[error] nqr supposed pt: the given link name "
//                       << link_name << " != "
//                       << " model link name " << link->GetName() << std::endl;
//             exit(1);
//         }

//         mSupposedContactPt.push_back(new tSupposedContactPt(
//             this->mModel, link_id, btMathUtil::Expand(local_pos, 1)));
//     }
// }

/**
 * \brief       Check the solution of riccati equation in each frame
*/
void btGenNQRCalculator::VerifyNQRRiccati()
{
    std::cout << "begin to check ARE\n";
    exit(0);
}

/**
 * \brief       According to the optimal control literature, given the raw system
 * 
 * x_{t+1} = G(x_t)u_t +  + h(x_t) = f(x_t, u_t)
 * 
 * calculate the linearalized system:
 * 
 * A_t = \frac{d f}{d x}
 * B_t = \frac{d f}{d u}
 * 
 * so that
 * 
 * \xi_{t+1} = A_t * \xi_t + B_t * \eta_t + d_k
 * 
 * \xi_t = x_t - \bar{x}_t,  \bar means it is in given mocap data
 * \eta_t = u_t - \bar{u}_t,  \bar means it is in given mocap data
 * d_k = 0 if the integration scheme keeps the same between the up&downstream, we took the semi-implicit from a-z
 * 
*/
void btGenNQRCalculator::CalcNQRSystemLinearzation(int frame)
{
    // 1. validate the frame id, and assume the model has been set before
    CheckModelState(frame, "system_linearize");

    // 2. get the A
    /*
        A = \frac{dG}{dx} * u + \frac{dh}{dx}
    */
    {
        // 2.1 get dGdx
        tEigenArr<tMatrixXd> dGdx;
        GetdGdx(frame, dGdx);
        // VerifydGdx(frame, dGdx);

        // 2.2 get dhdx
        tMatrixXd dhdx;
        Getdhdx(frame, dhdx);
    }
    // 3. get the B
    /*
    B = G
    */
}

/**
 * \brief           
*/
void btGenNQRCalculator::VerifyNQRSystemLinearzation(int frame) {}

/**
 * \brief       calculate the contact jacobian, and mocap ground truth vector 
 *  the control vecotr u = [joint_torque \in R^{n-6}, contact_forces \in R^3k]. so u \in R^{3k + n-6}
 *  control_jacobian = [joint_torque_jacobian(0;I): contact_force_jacobian(Jvc^T)] \in R^{n \times 3k + n - 6}
 *  d(control_jacobian)/dq  =   [[d(joint_torque_jacobian)/dq: dJvc^T/dq]]
 *                          =   [0: dJvc^T/dq]
*/
void btGenNQRCalculator::CalcNQRContactAndControlForce(int frame_id)
{
    CheckModelState(frame_id, "contact and control force");
    std::cout << "[warn] tmp hang out the check model state for contact and "
                 "control force\n";

    // 1. contact jacobian
    tMatrixXd contact_jacobian =
        mTraj->GetGenContactJacobianNoSet(frame_id, mModel);

    // 2. joint torque jacobian
    tMatrixXd joint_force_jacobian =
        tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    int root_dof = num_of_freedom - num_of_underactuated_freedom;
    joint_force_jacobian
        .block(root_dof, 0, num_of_underactuated_freedom,
               num_of_underactuated_freedom)
        .setIdentity();

    // 3. total jacobian = [joint_force_jacobian, contact_force_jacobian]
    auto &info = mNQRFrameInfos[frame_id];
    info.mTotalControlJacobian =
        tMatrixXd::Zero(num_of_freedom, info.mTotalControlForceSize);
    info.mTotalControlJacobian.block(0, 0, num_of_freedom,
                                     num_of_underactuated_freedom) =
        joint_force_jacobian;
    info.mTotalControlJacobian.block(0, num_of_underactuated_freedom,
                                     num_of_freedom, info.mContactForceSize) =
        contact_jacobian.transpose();

    /*
        4. calculate d(total_jacobian)/dq
        d(control_jacobian)/dq  =   [[d(joint_torque_jacobian)/dq: dJvc/dq]]
                                =   [0: dJvc^T/dq]
    */
    info.m_dControlJac_dq.resize(
        num_of_freedom,
        tMatrixXd::Zero(num_of_freedom, info.mTotalControlForceSize));
    tEigenArr<tMatrixXd> dContactJacdq;
    mTraj->GetGen_dContactJacobian_dq_NoSet(frame_id, mModel, dContactJacdq);
    for (int dof = 0; dof < num_of_freedom; dof++)
    {
        info.m_dControlJac_dq[dof].block(
            0, info.mJointControlForceSize, num_of_freedom,
            info.mContactForceSize) = dContactJacdq[dof].transpose();
    }
}

void btGenNQRCalculator::CheckFrameId(int frame_id, std::string prefix)
{
    if (frame_id < 0 || frame_id >= mTraj->mNumOfFrames)
    {
        std::cout << "[error] nqr " << prefix << " illegal frame " << frame_id
                  << std::endl;
        exit(0);
    }
}

void btGenNQRCalculator::CheckModelState(int frame_id, std::string prefix)
{
    CheckFrameId(frame_id, "CheckModelState");
    tVectorXd q_diff = mModel->Getq() - mTraj->mq[frame_id];
    tVectorXd qdot_diff = mModel->Getqdot() - mTraj->mqdot[frame_id];
    double eps = 1e-6;
    if (q_diff.norm() > eps || qdot_diff.norm() > eps)
    {
        std::cout << "[error] CheckModelstate " << prefix << " qdiff "
                  << q_diff.norm() << " qdot diff " << qdot_diff.norm()
                  << std::endl;
        exit(1);
    }
}

/**
 * G =  [dt * Minv * Jac, 0]
 *      [0, dt^2 * Minv * Jac]
 * x = [q, qdot]
 * Jac is the jacobian for TOTAL control vector
*/
tMatrixXd btGenNQRCalculator::GetG(int frame_id)
{
    auto &info = mNQRFrameInfos[frame_id];
    tMatrixXd G = tMatrixXd::Zero(mStateSize, info.mTotalControlForceSize);
    const tMatrixXd &Jac_control = info.mTotalControlJacobian;
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    double mdt2 = mdt * mdt;
    G.block(0, 0, num_of_freedom, info.mTotalControlForceSize) =
        mdt * Minv * Jac_control;
    G.block(num_of_freedom, 0, num_of_freedom, info.mTotalControlForceSize) =
        mdt2 * Minv * Jac_control;
    return G;
}

/**
 * \brief       Get dGdx = [dGdq, dGdqdot]
 * the definition of matrix G has been shown above
 * 
 * x_{t+1} = G * u_t + h
 * 
 * x is the state vector
 * u is the control vector
 * 
 * G is the state transition matrix
 * h is the state transition residual
*/
void btGenNQRCalculator::GetdGdx(int frame_id, tEigenArr<tMatrixXd> &dGdx)
{
    // 1. dGdq
    const auto &info = mNQRFrameInfos[frame_id];
    const int total_control_size = info.mTotalControlForceSize;
    dGdx.resize(mStateSize, tMatrixXd::Zero(mStateSize, total_control_size));
    tEigenArr<tMatrixXd> dMinvdq;
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    {
        // 1.1 give the dMinv/dq = -M^{-1} * dMdq * M^{-1}
        mModel->ComputedMassMatrixdq(dMinvdq); // now it's dMdq
        for (int dof = 0; dof < num_of_freedom; dof++)
        {
            dMinvdq[dof] = -Minv * dMinvdq[dof] * Minv;
            // std::cout << "dof " << dof << " dMinvdq norm "
            //           << dMinvdq[dof].norm() << std::endl;
        }

        // 1.2 get the dJac/dq from before, then gives the final
        const tMatrixXd &control_jac = info.mTotalControlJacobian;
        /*
            dGdq =  [ dt * dMinvdq * Jac + dt * Minv * dJac/dq ]
                    [ dt2 * dMinvdq * Jac + dt2 * Minv * dJac/dq ]

            dGdx = [dGdq(this part); dGdqdot]
        */
        // std::cout << "dt = " << mdt << std::endl;
        for (int dof = 0; dof < num_of_freedom; dof++)
        {
            // first part: [ dt * dMinvdq * Jac + dt * Minv * dJac/dq ]
            dGdx[dof].block(0, 0, num_of_freedom, info.mTotalControlForceSize) =
                mdt * dMinvdq[dof] * control_jac +
                mdt * Minv * info.m_dControlJac_dq[dof];

            // second part: dt * first part
            dGdx[dof].block(num_of_freedom, 0, num_of_freedom,
                            info.mTotalControlForceSize) =
                mdt * dGdx[dof].block(0, 0, num_of_freedom,
                                      info.mTotalControlForceSize);
        }
    }

    // 2. dGdqdot = 0
    {
        // statesize = 2 * dof. the last half portion of vector "x" means qdot
        // here we set all dGdqdot = 0
        // dGdx = [dGdq; dGdqdot(this part)]
        for (int i = num_of_freedom; i < mStateSize; i++)
        {
            dGdx[i].setZero();
        }
    }
    // for (int i = 0; i < mStateSize; i++)
    // {
    //     std::cout << "dGdx " << i << " norm = " << dGdx[i].norm() << std::endl;
    // }
    // exit(0);
}

/**
 * \brief               Get dh/dx
 * x_{t+1} = G * u + h
 * h is the state transition residual
 * 
*/
void btGenNQRCalculator::Getdhdx(int frame_id, tMatrixXd &dhdx)
{
    std::cout << "dhdx hasn't been implemented\n";
    exit(0);
}

// verify numerical gradient of CalcNQRContactAndControlForce
void btGenNQRCalculator::VerifyContactAndControlJacobian(int frame_id)
{
    // 1. remember current control jacobian value, old dJacdq
    mModel->PushState("verify_contact_and_control_jac");
    CheckFrameId(frame_id, "verify_contact_and_control_jac");
    auto &info = mNQRFrameInfos[frame_id];
    tMatrixXd old_control_jac = info.mTotalControlJacobian;
    tEigenArr<tMatrixXd> old_control_djac_dq = info.m_dControlJac_dq;
    int num_of_freedom = mModel->GetNumOfFreedom();
    tEigenArr<tMatrixXd> new_control_djac_dq(
        num_of_freedom,
        tMatrixXd::Zero(mStateSize, info.mTotalControlForceSize));

    // 2. forward computation
    tVectorXd q = mModel->Getq();
    double eps = 1e-8;
    for (int dof = 0; dof < num_of_freedom; dof++)
    {
        q[dof] += eps;
        mModel->SetqAndqdot(q, mModel->Getqdot());
        CalcNQRContactAndControlForce(frame_id);
        tMatrixXd new_control_jac = info.mTotalControlJacobian;
        tMatrixXd dcontrol_jac_dqi = (new_control_jac - old_control_jac) / eps;
        tMatrixXd dcontrol_jac_dq_diff =
            dcontrol_jac_dqi - old_control_djac_dq[dof];
        std::cout << "dof " << dof << " diff norm "
                  << dcontrol_jac_dq_diff.norm() << std::endl;
        if (dcontrol_jac_dq_diff.norm() > eps * 10)
        {
            std::cout << "analytic dJacdq = \n"
                      << old_control_djac_dq[dof] << std::endl;
            std::cout << "numerical dJacdq = \n"
                      << dcontrol_jac_dqi << std::endl;
            std::cout << "diff = \n" << dcontrol_jac_dq_diff << std::endl;
            exit(0);
        }
        q[dof] -= eps;
    }
    std::cout << "[log] frame " << frame_id
              << " total control jacobian derivation verified succ\n";
    mModel->PopState("verify_contact_and_control_jac");
}

// Verify the numerical dGdx
void btGenNQRCalculator::VerifydGdx(int frame_id,
                                    const tEigenArr<tMatrixXd> &analytic_dGdx)
{
    mModel->PushState("verify_dGdx");
    const auto &info = mNQRFrameInfos[frame_id];
    const tMatrixXd old_G = GetG(frame_id);

    tVectorXd q = mModel->Getq();
    double eps = 1e-8;
    for (int i = 0; i < num_of_freedom; i++)
    {
        q[i] += eps;
        mModel->SetqAndqdot(q, mModel->Getqdot());
        CalcNQRContactAndControlForce(frame_id);
        tMatrixXd new_G = GetG(frame_id);
        tMatrixXd dGdqi = (new_G - old_G) / eps;
        tMatrixXd diff = dGdqi - analytic_dGdx[i];
        double diff_max = diff.cwiseAbs().maxCoeff();
        std::cout << "[log] dof " << i << " dGdx numerically diff max "
                  << diff_max << ", ana norm " << analytic_dGdx[i].norm()
                  << " , numerical norm " << dGdqi.norm() << std::endl;
        if (diff_max > 1e-5)
        {
            std::cout << "anlytic dGdq = \n" << analytic_dGdx[i] << std::endl;
            std::cout << "numerical dGdq = \n" << dGdqi << std::endl;
            std::cout << "dGdq diff = \n" << diff << std::endl;
            exit(0);
        }
        q[i] -= eps;
    }
    mModel->PopState("verify_dGdx");
}

void btGenNQRCalculator::Verifydhdx(int frame_id,
                                    const tMatrixXd &analytic_dhdx)
{
}