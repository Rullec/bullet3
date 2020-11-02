#include "BulletGenDynamics/btGenController/NQRCalculator/btGenNQRCalculator.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenModel/Link.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include <iostream>

btGenNQRCalculator::tNQRFrameInfo::tNQRFrameInfo()
{
    Sk_mat.resize(0, 0);
    sk_vec.resize(0);
    Q_coef.resize(0);
    R_coef.resize(0);
    P_coef.resize(0);

    mContactForceSize = -1;
    mJointControlForceSize = -1;
    mTotalControlForceSize = -1;
    mTotalControlJacobian.resize(0, 0);
    m_dControlJac_dq.clear();
    mCartesianContactForce_mocap.resize(0);
    mJointControlForce_mocap.resize(0);
    mTotalControlVector_mocap.resize(0);
    mStateVector_mocap.resize(0);
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
    mStateSize = -1;
    Q_coef = 0;
    R_coef = 0;
    P_coef = 0;

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

    Q_coef = btJsonUtil::ParseAsDouble("Q", conf_json);
    R_coef = btJsonUtil::ParseAsDouble("R", conf_json);
    P_coef = btJsonUtil::ParseAsDouble("P", conf_json);
    printf("[log] Q(xi min) = %.5f, R(eta min) %.5f, P(f close) = %.5f\n",
           Q_coef, R_coef, P_coef);
}
void btGenNQRCalculator::SetTraj(btTraj *traj_)
{
    btGenTargetCalculator::SetTraj(traj_);

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

    // // 2. fetch the Sk_mat and sk_vec, P/R coef, G/h calculate the control policy
    // std::cout << "[log] control by NQR controller purely "
    //           << ", gui ptr = " << this->mBulletGUIHelper << " frame "
    //           << mRefFrameId << std::endl;
    // {
    //     tVectorXd cur_q = mModel->Getq(), cur_qdot = mModel->Getqdot();
    //     tVectorXd mocap_q = mTraj->mq[mRefFrameId],
    //               mocap_qdot = mTraj->mqdot[mRefFrameId];
    //     tVectorXd q_diff = cur_q - mocap_q, qdot_diff = cur_qdot - mocap_qdot;
    //     printf("[nqr] q diff %.5f, qdot diff %.5f\n", q_diff.norm(),
    //            qdot_diff.norm());
    // }
    // // draw the contact points
    // // 1. get current control force = [joint_force, contact_force] from NQR
    // const auto &cur_contact_info = mTraj->mContactForce[mRefFrameId];
    // tVectorXd uk = CalcControlVector(cur_contact_info);

    // // 2. apply it to the model
    // mModel->PushState("apply_target_try");
    // ApplyControlVector(cur_contact_info, mNQRFrameInfos[mRefFrameId], uk);
    // mModel->ApplyGravity(mWorld->GetGravity());

    // // 3. apply the control policy AND THE GRAVITY, get the qddot, qdotnext, qnext
    // tilde_qddot = mModel->Getqddot();
    // tilde_qdot = mdt * tilde_qddot + mModel->Getqdot();
    // tilde_q = mdt * tilde_qdot + mModel->Getq();
    // tilde_tau =
    //     uk.segment(3 * cur_contact_info.size(), num_of_underactuated_freedom);
    // std::cout << "[nqr] target: qddot = " << tilde_qddot.transpose()
    //           << std::endl;
    // std::cout << "[nqr] target: tau = " << tilde_tau.transpose() << std::endl;

    // mModel->PopState("apply_target_try");
}
int btGenNQRCalculator::GetCalculatedNumOfContact() const { return -1; }
void btGenNQRCalculator::ControlByAdaptionController()
{
    std::cout << "[log] control by NQR controller purely "
              << ", gui ptr = " << this->mBulletGUIHelper << " frame "
              << mRefFrameId << std::endl;
    tVectorXd cur_q = mModel->Getq(), cur_qdot = mModel->Getqdot();
    tVectorXd mocap_q = mTraj->mq[mRefFrameId],
              mocap_qdot = mTraj->mqdot[mRefFrameId];
    tVectorXd q_diff = cur_q - mocap_q, qdot_diff = cur_qdot - mocap_qdot;
    printf("[nqr] ctrl myself: q diff %.5f, qdot diff %.5f\n", q_diff.norm(),
           qdot_diff.norm());
    // draw the contact points
    // 1. get current control force = [joint_force, contact_force] from NQR
    const auto &cur_contact_info = mTraj->mContactForce[mRefFrameId];
    tVectorXd uk = CalcControlVector(cur_contact_info);

    // 2. apply it to the model
    ApplyControlVector(cur_contact_info, mNQRFrameInfos[mRefFrameId], uk);

    // 3. update info stuff
    mRefFrameId++;
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
    // auto root = mModel->GetRoot();
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

        info.Q_coef = tVectorXd::Ones(mStateSize) * Q_coef;
        info.R_coef = tVectorXd::Ones(info.mTotalControlForceSize) * R_coef;
        info.P_coef = tVectorXd::Ones(info.mTotalControlForceSize) * P_coef;
    }
    // 4. begin the main iter
    mModel->SetComputeThirdDerive(true);
    for (int frame_id = mTraj->mNumOfFrames - 1; frame_id >= 1; frame_id--)
    {
        // set q and qdot
        // printf("[log] solve nqr for frame %d\n", frame_id);
        // std::cout << "--------------- frame " << frame_id << " -------------\n";

        const tVectorXd &q = mTraj->mq[frame_id],
                        &qdot = mTraj->mqdot[frame_id];

        mModel->SetqAndqdot(q, qdot);

        // set state vector
        mNQRFrameInfos[frame_id].mStateVector_mocap.resize(mStateSize);
        mNQRFrameInfos[frame_id].mStateVector_mocap.segment(0, num_of_freedom) =
            q;
        mNQRFrameInfos[frame_id].mStateVector_mocap.segment(
            num_of_freedom, num_of_freedom) = qdot;

        // calculate control force
        CalcContactAndControlForce(frame_id, mNQRFrameInfos[frame_id]);
        // VerifyContactAndControlJacobian(frame_id);

        // Calculate A and B
        CalcSystemLinearzation(mTraj->mContactForce[frame_id],
                               mNQRFrameInfos[frame_id]);

        // solve the ARE (algebraic riccati equation) recursively, inversely
        tNQRFrameInfo *cur_frame_info = &(mNQRFrameInfos[frame_id]),
                      *next_frame_info = (frame_id == (mTraj->mNumOfFrames - 1))
                                             ? nullptr
                                             : &(mNQRFrameInfos[frame_id + 1]);
        CalcNQRRiccati(cur_frame_info, next_frame_info);
    }
    // VerifyNQRRiccati();
    mModel->PopState("calc_nqr");
}

/**
 * \brief           Solve the algebraic riccati equation at given frame
 * \param frame     frame id
*/
void btGenNQRCalculator::CalcNQRRiccati(tNQRFrameInfo *cur_info,
                                        const tNQRFrameInfo *const next_info)
{
    // this frame is the last one: set up SN = QN and sN  = 0
    // auto &cur_info = mNQRFrameInfos[frame];

    if (next_info == nullptr)
    {
        cur_info->sk_vec = tVectorXd::Zero(mStateSize);
        cur_info->Sk_mat = cur_info->Q_coef.asDiagonal();
    }
    else
    {
        // frame is not the last one
        const tVectorXd &s_vec_next = next_info->sk_vec; // vector: s_{k plus 1}
        const tMatrixXd &S_mat_next = next_info->Sk_mat; // mat: S_{k plus 1}
                                                         /*
            1. calculate Sk

            Sk = QK + AKT * (S_next^{-1} + Bk * (Pk + Rk)^{-1} * BkT)^{-1} * Ak
        */
        const tMatrixXd &Qk = cur_info->Q_coef.asDiagonal();
        const tMatrixXd &Ak = cur_info->mA;
        const tMatrixXd &Bk = cur_info->mB;
        const tMatrixXd &Pk = cur_info->P_coef.asDiagonal();
        const tMatrixXd &Rk = cur_info->R_coef.asDiagonal();
        {
            tMatrixXd middle = (S_mat_next.inverse() +
                                Bk * (Pk + Rk).inverse() * Bk.transpose())
                                   .inverse();
            cur_info->Sk_mat = Qk + Ak.transpose() * middle * Ak;
        }

        /*
            sk = AkT * s_{k+1} 
                + AkT * 
            part1:  (S_mat_{k+1}^{-1} + Bk * (Pk + RK)^{-1} * BkT)^{-1}
                     * 
            part2:  (dk - Bk * (Pk + Rk)^{-1} *（BkT * s_k+1 + Pk * \bar{u}_k))
        */
        {
            tVectorXd dk = tVectorXd::Zero(mStateSize);
            tMatrixXd part1 = (S_mat_next.inverse() +
                               Bk * (Pk + Rk).inverse() * Bk.transpose())
                                  .inverse();
            tVectorXd part2 =
                (dk - Bk * (Pk + Rk).inverse() *
                          (Bk.transpose() * s_vec_next +
                           Pk * cur_info->mTotalControlVector_mocap));
            cur_info->sk_vec =
                Ak.transpose() * s_vec_next + Ak.transpose() * part1 * part2;
        }
    }
    // std::cout << "[debug] frame " << frame
    //           << " sk_vec = " << cur_info.sk_vec.transpose() << std::endl;
    // std::cout << "[debug] frame " << frame << " Sk_mat = \n" << cur_info.Sk_mat
    //           << std::endl;
    // exit(1);
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
    double eps = 1e-7;
    for (int frame = 1; frame < mTraj->mNumOfFrames - 1; frame++)
    {
        const auto &cur_info = mNQRFrameInfos[frame];
        const auto &next_info = mNQRFrameInfos[frame + 1];
        const tMatrixXd &Qk = cur_info.Q_coef.asDiagonal();
        const tMatrixXd &Pk = cur_info.P_coef.asDiagonal();
        const tMatrixXd &Rk = cur_info.R_coef.asDiagonal();
        const tMatrixXd &Ak = cur_info.mA;
        const tMatrixXd &S_mat_next = next_info.Sk_mat;
        const tMatrixXd &s_vec_next = next_info.sk_vec;
        const tMatrixXd &Bk = cur_info.mB;
        const tMatrixXd &S_mat_cur = cur_info.Sk_mat;
        const tMatrixXd &s_vec_cur = cur_info.sk_vec;
        const tVectorXd &bar_u_cur = cur_info.mTotalControlVector_mocap;
        tMatrixXd formula1_res;
        {
            // printf("Pk size %d %d\n", Pk.rows(), Pk.cols());
            // printf("Rk size %d %d\n", Rk.rows(), Rk.cols());
            // printf("Bk size %d %d\n", Bk.rows(), Bk.cols());
            tMatrixXd part1 = Bk * (Pk + Rk).inverse() * Bk.transpose();
            tMatrixXd part2 = (S_mat_next.inverse() + part1).inverse();
            formula1_res = S_mat_cur - Qk - Ak.transpose() * part2 * Ak;
        }

        tVectorXd formula2_res =
            s_vec_cur - Ak.transpose() * s_vec_next -
            Ak.transpose() *
                (S_mat_next.inverse() +
                 Bk * (Pk + Rk).inverse() * Bk.transpose())
                    .inverse() *
                (-Bk * (Pk + Rk).inverse() *
                 (Bk.transpose() * s_vec_next + Pk * bar_u_cur));
        double formula1_res_norm = formula1_res.norm();
        double formula2_res_norm = formula2_res.norm();
        printf("[log] frame %d verify ARE: diff1 %.10f, diff2 "
               "%.10f\n",
               frame, formula1_res_norm, formula2_res_norm);
        if (formula1_res_norm > eps || formula2_res_norm > eps)
        {
            printf("[error] frame %d verify ARE failed: diff1 %.10f, diff2 "
                   "%.10f\n",
                   frame, formula1_res_norm, formula2_res_norm);
            exit(1);
        }
    }
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
void btGenNQRCalculator::CalcSystemLinearzation(
    const tContactInfo &contact_info, tNQRFrameInfo &info)
{
    // 1. validate the frame id, and assume the model has been set before
    assert(true == mModel->GetComputeThirdDerive());

    // 2. get the A \frac{dG}{dx} * u + \frac{dh}{dx}
    {
        // 2.1 get dGdx
        tEigenArr<tMatrixXd> dGdx;
        GetdGdx(contact_info, dGdx);
        // VerifydGdx(frame, dGdx);

        // 2.2 get dhdx
        tMatrixXd dhdx;
        Getdhdx(dhdx);
        // Verifydhdx(frame, dhdx);

        // 2.3 get A

        info.mA = dhdx;
        for (int i = 0; i < mStateSize; i++)
        {
            info.mA.col(i) += dGdx[i] * info.mTotalControlVector_mocap;
        }
    }
    // 3. get the B = G
    info.mB = GetG(contact_info);

    // verify system lineazation
    // std::cout << "begin system lineazation verification\n";
    // tVectorXd x = tVectorXd::Zero(mStateSize);
    // x.segment(0, num_of_freedom) = mModel->Getq();
    // x.segment(num_of_freedom, num_of_freedom) = mModel->Getqdot();
    // tVectorXd u = info.mTotalControlVector_mocap;
    // VerifySystemLinearzation(contact_info, info.mA, info.mB, x, u);
}

/**
 * \brief           
*/
void btGenNQRCalculator::VerifySystemLinearzation(const tContactInfo &info,
                                                  const tMatrixXd &dfdx,
                                                  const tMatrixXd &dfdu,
                                                  const tVectorXd &x_old,
                                                  const tVectorXd &u_old)
{
    mModel->PushState("verify_linearization");
    // x_{t+1} = G(x) * u + h(x) = f(x, u)
    tVectorXd x = x_old, u = u_old;
    // 1. calculate f_old
    tVectorXd f_old = Getxnext(info, x, u);
    double eps = 1e-7;
    for (int i = 0; i < mStateSize; i++)
    {
        x[i] += eps;
        tVectorXd f_new = Getxnext(info, x, u);
        tVectorXd dfdxi = (f_new - f_old) / eps;
        tVectorXd dfdx_diff = dfdxi - dfdx.col(i);

        double norm = dfdx_diff.norm();
        printf("[log] dfdx%d diff norm %.10f!\n", i, norm);
        if (norm > eps)
        {
            printf("[error] system linearzation dfdx%d diff norm %.10f\n", i,
                   norm);
            exit(0);
        }
        x[i] -= eps;
    }
    // std::cout << "[log] dfdx tested well!\n";
    int contact_size = u.size();
    for (int i = 0; i < contact_size; i++)
    {
        u[i] += eps;
        tVectorXd f_new = Getxnext(info, x, u);
        tVectorXd dfdui = (f_new - f_old) / eps;
        tVectorXd dfdu_diff = dfdui - dfdu.col(i);
        double norm = dfdu_diff.norm();
        if (norm > eps)
        {
            printf("[error] system linearzation dfdu%d diff norm %.10f\n", i,
                   norm);

            exit(0);
        }
        u[i] -= eps;
    }
    std::cout << "[log] dfdu/dfdx tested well!\n";
    mModel->PopState("verify_linearization");
}

/**
 * \brief           calculate the jacobian for the total control vector [\chi, f_c]
 * \chi is contact force (cartesian)
 * f_c is joint control force (n-6)
*/
tMatrixXd
btGenNQRCalculator::CalcTotalControlJacobian(const tContactInfo &contact_info)
{
    // 1. joint force jacobian
    tMatrixXd joint_force_jacobian =
        tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    int root_dof = num_of_freedom - num_of_underactuated_freedom;
    joint_force_jacobian
        .block(root_dof, 0, num_of_underactuated_freedom,
               num_of_underactuated_freedom)
        .setIdentity();

    // 2. contact jacobian
    int contact_size = contact_info.size();
    tMatrixXd total_jac = tMatrixXd::Zero(3 * contact_size, num_of_freedom),
              single_jac = tMatrixXd::Zero(3, num_of_freedom);
    for (int i = 0; i < contact_size; i++)
    {
        auto link = mModel->GetLinkById(contact_info[i]->mLinkId);
        mModel->ComputeJacobiByGivenPointTotalDOFLinkLocalFrame(
            link->GetId(), contact_info[i]->mLocalPos.segment(0, 3),
            single_jac);
        total_jac.block(i * 3, 0, 3, num_of_freedom).noalias() = single_jac;
    }

    // 3. assemble: total jacobian = [contact_force_jacobian, joint_force_jacobian]
    int total_size = 3 * contact_info.size() + num_of_underactuated_freedom;
    tMatrixXd total_control_jacobian =
        tMatrixXd::Zero(num_of_freedom, total_size);

    total_control_jacobian.block(
        0, 0, num_of_freedom, 3 * contact_info.size()) = total_jac.transpose();

    total_control_jacobian.block(0, 3 * contact_info.size(), num_of_freedom,
                                 num_of_underactuated_freedom) =
        joint_force_jacobian;
    return total_control_jacobian;
}

/**
 * \brief               Fetch the active joint force + contact force from mocap data
 * 
 * total_control_vector = [cartesian_contact_forces, joint_forces]
*/
void btGenNQRCalculator::CalcContactAndControlForce(int frame_id,
                                                    tNQRFrameInfo &info)
{
    CheckModelState(frame_id, "contact and control force");

    // 5. mocap cartesian contact force, mocap joint control force

    // 5.1 cartesian contact force
    const auto &mocap_contact_info = mTraj->mContactForce[frame_id];
    int num_of_contact = mocap_contact_info.size();
    assert(num_of_contact * 3 == info.mContactForceSize);
    info.mCartesianContactForce_mocap.resize(3 * num_of_contact);
    for (int i = 0; i < num_of_contact; i++)
    {
        info.mCartesianContactForce_mocap.segment(i * 3, 3) =
            mocap_contact_info[i]->mForce.segment(0, 3);
        // std::cout << "contact " << i << " force = "
        //           << mocap_contact_info[i]->mForce.segment(0, 3).transpose()
        //           << std::endl;
    }
    // 5.2 mocap joint control force
    assert(info.mJointControlForceSize == num_of_underactuated_freedom);
    info.mJointControlForce_mocap.resize(num_of_underactuated_freedom);
    info.mJointControlForce_mocap.segment(0, num_of_underactuated_freedom) =
        mTraj->mTruthJointForceVec[frame_id];

    // 5.3 mocap total control force: concanate
    info.mTotalControlVector_mocap.resize(info.mTotalControlForceSize);
    info.mTotalControlVector_mocap.segment(0, info.mContactForceSize) =
        info.mCartesianContactForce_mocap;
    info.mTotalControlVector_mocap.segment(info.mContactForceSize,
                                           info.mJointControlForceSize) =
        info.mJointControlForce_mocap;
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
 * \brief       Get the G matrix when the character is running
 * G =  [dt * Minv * Jac, 0]
 *      [0, dt^2 * Minv * Jac]
 * x = [q, qdot]
 * Jac is the jacobian for TOTAL control vector
 *  
*/
tMatrixXd btGenNQRCalculator::GetG(const tContactInfo &contact_info)
{
    // compute the jacobian from the contact state in the mocap data
    int contact_size = contact_info.size() * 3;
    int joint_size = num_of_underactuated_freedom;
    int total_size = contact_size + joint_size;
    tMatrixXd G = tMatrixXd::Zero(mStateSize, total_size);
    const tMatrixXd &Jac_control = CalcTotalControlJacobian(contact_info);
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    double mdt2 = mdt * mdt;
    G.block(0, 0, num_of_freedom, total_size) = mdt * Minv * Jac_control;
    G.block(num_of_freedom, 0, num_of_freedom, total_size) =
        mdt2 * Minv * Jac_control;
    return G;
}

/**
 * \brief       Get current "R" vector
 * 
 * R = \dot{q}_t + dt * Minv * (QG - C * \dot{q})
*/
tVectorXd btGenNQRCalculator::GetR()
{
    return mModel->Getqdot() +
           mdt * mModel->GetInvMassMatrix() *
               (mModel->CalcGenGravity(mWorld->GetGravity()) -
                mModel->GetCoriolisMatrix() * mModel->Getqdot());
}

/**
 * \brief       calculate the residual of state equation, "h"
 * 
 * h =  [mdt * Rt + q_t]
 *      [Rt]
*/
tVectorXd btGenNQRCalculator::Geth()
{
    tVectorXd Rt = GetR();
    tVectorXd h = tVectorXd::Zero(mStateSize);
    h.segment(0, num_of_freedom).noalias() = mdt * Rt + mModel->Getq();
    h.segment(num_of_freedom, num_of_freedom).noalias() = Rt;
    return h;
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
void btGenNQRCalculator::GetdGdx(const tContactInfo &contact_info,
                                 tEigenArr<tMatrixXd> &dGdx)
{
    // 1. dGdq
    const int total_control_size =
        num_of_underactuated_freedom + 3 * contact_info.size();
    dGdx.resize(mStateSize, tMatrixXd::Zero(mStateSize, total_control_size));
    tEigenArr<tMatrixXd> dMinvdq;
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    {
        // 1.1 give the dMinv/dq = -M^{-1} * dMdq * M^{-1}
        mModel->ComputedMassMatrixdq(dMinvdq); // now it's dMdq
        for (int dof = 0; dof < num_of_freedom; dof++)
        {
            dMinvdq[dof] = -Minv * dMinvdq[dof] * Minv;
        }

        // 1.2 get the dJac/dq from before, then gives the final
        const tMatrixXd &control_jac = CalcTotalControlJacobian(contact_info);
        /*
            dGdq =  [ dt * dMinvdq * Jac + dt * Minv * dJac/dq ]
                    [ dt2 * dMinvdq * Jac + dt2 * Minv * dJac/dq ]

            dGdx = [dGdq(this part); dGdqdot]
        */
        tEigenArr<tMatrixXd> dControlJac_dq;
        CalcTotalControl_dJacobiandq(dControlJac_dq, contact_info);

        for (int dof = 0; dof < num_of_freedom; dof++)
        {
            // first part: [ dt * dMinvdq * Jac + dt * Minv * dJac/dq ]
            dGdx[dof].block(0, 0, num_of_freedom, total_control_size) =
                mdt * dMinvdq[dof] * control_jac +
                mdt * Minv * dControlJac_dq[dof];

            // second part: dt * first part
            dGdx[dof].block(num_of_freedom, 0, num_of_freedom,
                            total_control_size) =
                mdt * dGdx[dof].block(0, 0, num_of_freedom, total_control_size);
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
}

/**
 * \brief               Get dh/dx
 * given the state equation: x_{t+1} = G * u + h, h is the state transition residual
 * 
 * for multibody h =    [ dt * R_t + q_t ]
 *                      [ R_t  ]       \in R^{2n x 1}
 * 
 *                      R_t = \dot{q}_t + dt * Minv * (QG - C * qdot)
 * 
 * so dh/dx = [dh/dq, dh/dqdot] \in R^{2n x 2n}
 * 
 * dhdq =   [dt * dR_t/dq + I]
 *          [dR_t/dq] \in R^{2n x n}
 * dhdqdot =    [dt * dR_t/dqdot]
 *              [dR_t/dqdot]        \in R{2n x n}
 * 
 * The key point is the formula above is:
 *  1. dRdq 
 *  2. dRdqdot
*/
void btGenNQRCalculator::Getdhdx(tMatrixXd &dhdx)
{
    // std::cout << "dhdx hasn't been implemented\n";
    // 1. get dRdq
    tMatrixXd dRdq, dRdqdot;
    GetdRdq(dRdq);
    // VerifydRdq(dRdq);

    // 2. get dRdqdot
    GetdRdqdot(dRdqdot);
    // VerifydRdqdot(dRdqdot);
    // 3. combine and get the final dhdx
    dhdx.noalias() = tMatrixXd::Zero(mStateSize, mStateSize);
    dhdx.block(0, 0, num_of_freedom, num_of_freedom).noalias() =
        mdt * dRdq + tMatrixXd::Identity(num_of_freedom, num_of_freedom);
    dhdx.block(num_of_freedom, 0, num_of_freedom, num_of_freedom).noalias() =
        dRdq;

    dhdx.block(0, num_of_freedom, num_of_freedom, num_of_freedom).noalias() =
        mdt * dRdqdot;
    dhdx.block(num_of_freedom, num_of_freedom, num_of_freedom, num_of_freedom)
        .noalias() = dRdqdot;
}

// verify numerical gradient of CalcContactAndControlForce
void btGenNQRCalculator::VerifyContactAndControlJacobian(
    const tContactInfo &contact_info, const tNQRFrameInfo &nqr_info,
    const tEigenArr<tMatrixXd> &dJacdq_analytic)
{
    // 1. remember current control jacobian value, old dJacdq
    mModel->PushState("verify_contact_and_control_jac");

    tMatrixXd old_control_jac = CalcTotalControlJacobian(contact_info);
    int num_of_freedom = mModel->GetNumOfFreedom();

    // 2. forward computation
    tVectorXd q = mModel->Getq();
    double eps = 1e-8;
    for (int dof = 0; dof < num_of_freedom; dof++)
    {
        q[dof] += eps;
        mModel->SetqAndqdot(q, mModel->Getqdot());
        tMatrixXd new_control_jac = CalcTotalControlJacobian(contact_info);
        tMatrixXd dcontrol_jac_dqi = (new_control_jac - old_control_jac) / eps;
        tMatrixXd dcontrol_jac_dq_diff =
            dcontrol_jac_dqi - dJacdq_analytic[dof];
        std::cout << "dof " << dof << " diff norm "
                  << dcontrol_jac_dq_diff.norm() << std::endl;
        if (dcontrol_jac_dq_diff.norm() > eps * 10)
        {
            std::cout << "analytic dJacdq = \n"
                      << dJacdq_analytic[dof] << std::endl;
            std::cout << "numerical dJacdq = \n"
                      << dcontrol_jac_dqi << std::endl;
            std::cout << "diff = \n" << dcontrol_jac_dq_diff << std::endl;
            exit(0);
        }
        q[dof] -= eps;
    }
    std::cout << "[log] total control jacobian derivation verified succ\n";
    mModel->PopState("verify_contact_and_control_jac");
}

// Verify the numerical dGdx
void btGenNQRCalculator::VerifydGdx(const tContactInfo &contact_info,
                                    const tEigenArr<tMatrixXd> &analytic_dGdx)
{
    mModel->PushState("verify_dGdx");
    const tMatrixXd old_G = GetG(contact_info);

    tVectorXd q = mModel->Getq();
    double eps = 1e-8;
    for (int i = 0; i < num_of_freedom; i++)
    {
        q[i] += eps;
        mModel->SetqAndqdot(q, mModel->Getqdot());
        tMatrixXd new_G = GetG(contact_info);
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
    printf("[log] dGdx verified succ\n");
    mModel->PopState("verify_dGdx");
}

void btGenNQRCalculator::Verifydhdx(const tMatrixXd &analytic_dhdx)
{
    mModel->PushState("verify_dhdx");
    tVectorXd old_h = Geth();
    tVectorXd q_old = mModel->Getq(), qdot_old = mModel->Getqdot();
    double eps = 1e-7;
    for (int i = 0; i < num_of_freedom; i++)
    {
        q_old[i] += eps;
        mModel->SetqAndqdot(q_old, qdot_old);
        tVectorXd new_h = Geth();
        tVectorXd dhdq_num = (new_h - old_h) / eps;
        tVectorXd dhdq_diff = dhdq_num - analytic_dhdx.col(i);
        double norm = dhdq_diff.norm();
        if (norm > 10 * eps)
        {
            printf("[error] Verify dhdq%d failed, diff norm %.10f\n", i, norm);
            std::cout << "ana = " << analytic_dhdx.col(i).transpose();
            std::cout << "num = " << dhdq_num.transpose();
        }
        q_old[i] -= eps;
        printf("[log] NQR: dhdq%d verify succ, value norm = %.5f, diif norm "
               "%.10f\n",
               i, dhdq_num.norm(), norm);
    }
    for (int i = 0; i < num_of_freedom; i++)
    {
        qdot_old[i] += eps;
        mModel->SetqAndqdot(q_old, qdot_old);
        tVectorXd new_h = Geth();
        tVectorXd dhdqdot_num = (new_h - old_h) / eps;
        tVectorXd dhdqdot_diff =
            dhdqdot_num - analytic_dhdx.col(i + num_of_freedom);
        double norm = dhdqdot_diff.norm();
        if (norm > 10 * eps)
        {
            printf("[error] Verify dhdqdot%d failed, diff norm %.10f\n", i,
                   norm);
            std::cout << "ana = "
                      << analytic_dhdx.col(i + num_of_freedom).transpose();
            std::cout << "num = " << dhdqdot_num.transpose();
        }
        qdot_old[i] -= eps;
        printf("[log] NQR: dhdqdot%d verify succ, value norm = %.5f, diff norm "
               "%.10f\n",
               i, dhdqdot_num.norm(), norm);
    }
    printf("[log] dhdx verified succ\n");
    mModel->PopState("verify_dhdx");
}

/**
 * \brief           calculate the derivative of R w.r.t q
 * R = \dot{q}_t + dt * Minv * (QG - C * \dot{q}_t)
 * 
 * dRdq = dt * dMinvdq * (QG - C*qdot) + dt * Minv * (dQGdq - dCdq * qdot)
 *      = -dt * Minv * dMdq * Minv * (QG - C*qdot) + dt * Minv * (dQGdq - dCdq * qdot)
*/

void btGenNQRCalculator::GetdRdq(tMatrixXd &dRdq)
{
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &M = mModel->GetMassMatrix();
    EIGEN_V_MATXD dMdq;
    mModel->ComputedMassMatrixdq(dMdq);

    const tVectorXd &QG = mModel->CalcGenGravity(mWorld->GetGravity());
    const tMatrixXd &C = mModel->GetCoriolisMatrix();
    const tMatrixXd &dQGdq = mModel->CalcdGenGravitydq(mWorld->GetGravity());
    const tVectorXd &qdot = mModel->Getqdot();
    EIGEN_V_MATXD dCdq;
    mModel->ComputeDCoriolisMatrixDq(qdot, dCdq);

    const tVectorXd &Minv_QG_Cqdot = Minv * (QG - C * qdot);

    dRdq.noalias() = tMatrixXd::Zero(num_of_freedom, num_of_freedom);
    for (int dof = 0; dof < num_of_freedom; dof++)
    {
        dRdq.col(dof).noalias() =
            -mdt * Minv * dMdq[dof] * Minv_QG_Cqdot +
            mdt * Minv * (dQGdq.col(dof) - dCdq[dof] * qdot);
    }
}

/**
 * \brief       calculate dRdqdot
 * R = \dot{q}_t + dt * Minv * (QG - C * \dot{q}_t)
 * 
 * dRdqdot = I - dt * Minv (dCdqdot * qdot + C * I)
*/
void btGenNQRCalculator::GetdRdqdot(tMatrixXd &dRdqdot)
{
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    const tMatrixXd &C = mModel->GetCoriolisMatrix();
    EIGEN_V_MATXD dCdqdot;
    mModel->ComputeDCoriolisMatrixDqdot(dCdqdot);
    const tVectorXd &qdot = mModel->Getqdot();
    dRdqdot.noalias() = tMatrixXd::Identity(num_of_freedom, num_of_freedom);
    for (int i = 0; i < num_of_freedom; i++)
    {
        dRdqdot.col(i) -= mdt * Minv * (dCdqdot[i] * qdot + C.col(i));
    }
}

/**
 * \brief           compute the numerical gradient of R in order to make sure the analytic gradient is correct
*/
void btGenNQRCalculator::VerifydRdq(const tMatrixXd &dRdq_ana)
{
    mModel->PushState("verify_drdq");
    tVectorXd R_old = GetR();
    tVectorXd q_old = mModel->Getq();
    double eps = 1e-7;
    for (int i = 0; i < num_of_freedom; i++)
    {
        q_old[i] += eps;
        mModel->SetqAndqdot(q_old, mModel->Getqdot());
        tVectorXd R_new = GetR();
        tVectorXd dRdq_num = (R_new - R_old) / eps;
        tVectorXd dRdq_diff = dRdq_num - dRdq_ana.col(i);
        double norm = dRdq_diff.norm();
        if (norm > 10 * eps)
        {
            printf("[error] Verify dRdq%d failed, diff norm %.10f\n", i, norm);
            std::cout << "ana = " << dRdq_ana.col(i).transpose();
            std::cout << "num = " << dRdq_num.transpose();
        }
        q_old[i] -= eps;
        printf("[log] NQR: dRdq%d verify succ, value norm = %.5f\n", i,
               dRdq_num.norm());
    }
    std::cout << "dRdq verify succ\n";
    mModel->PopState("verify_drdq");
}

/**
 * \brief           compute the numerical gradient of R in order to make sure the analytic gradient is correct
*/
void btGenNQRCalculator::VerifydRdqdot(const tMatrixXd &dRdqdot_ana)
{
    mModel->PushState("verify_drdqdot");
    tVectorXd R_old = GetR();
    tVectorXd qdot_old = mModel->Getqdot();
    double eps = 1e-7;
    for (int i = 0; i < num_of_freedom; i++)
    {
        qdot_old[i] += eps;
        mModel->SetqAndqdot(mModel->Getq(), qdot_old);
        tVectorXd R_new = GetR();
        tVectorXd dRdqdot_num = (R_new - R_old) / eps;
        tVectorXd dRdqdot_diff = dRdqdot_num - dRdqdot_ana.col(i);
        double norm = dRdqdot_diff.norm();
        if (norm > 10 * eps)
        {
            printf("[error] Verify dRdqdot%d failed, diff norm %.10f\n", i,
                   norm);
            std::cout << "ana = " << dRdqdot_ana.col(i).transpose();
            std::cout << "num = " << dRdqdot_num.transpose();
        }
        qdot_old[i] -= eps;
        printf("[log] NQR: dRdqdot%d verify succ, value norm = %.5f\n", i,
               dRdqdot_num.norm());
    }
    std::cout << "dRdqdot verify succ\n";
    mModel->PopState("verify_drdqdot");
}

/**
 * \brief       Given control vector u =[contact_force, control_force], apply it to the model
*/
void btGenNQRCalculator::ApplyControlVector(const tContactInfo &contact_info,
                                            const tNQRFrameInfo &nqr_info,
                                            const tVectorXd &control_vector)
{
    assert(control_vector.size() == nqr_info.mTotalControlForceSize);

    // 1. get the contact force
    for (int i = 0; i < contact_info.size(); i++)
    {
        auto single_info = contact_info[i];
        tVector global_pos =
            mModel->GetLinkById(single_info->mLinkId)->GetGlobalTransform() *
            single_info->mLocalPos;
        tVector force = tVector::Zero();
        force.segment(0, 3) = control_vector.segment(i * 3, 3);
        mModel->ApplyForce(single_info->mLinkId, force, global_pos);
        // std::cout << "[nqr] apply contact force " << i << " = "
        //           << force.transpose() << std::endl;
    }

    // 2. get the joint force
    tVectorXd joint_force = control_vector.segment(
        nqr_info.mContactForceSize, nqr_info.mJointControlForceSize);
    assert(joint_force.size() == num_of_underactuated_freedom);
    // std::cout << "[nqr] apply joint force = " << joint_force.transpose()
    //           << std::endl;
    for (int i = 0; i < num_of_underactuated_freedom; i++)
    {
        mModel->ApplyGeneralizedForce(i + 6, joint_force[i]);
    }
}

/**
 * 
 * \brief           Calculate the control vector = [contact_force, control_force] 
 *  by precomputed NQR policy
 * uk = 
 *    (Pk + Rk + GT * Sk+1 * G)^{-1}
 *       * 
 *      (Rk * \bar{u}k 
 *          - GT * 
 *              (s_{k+1} +
 *                  S_{k+1} * (hk - \bar{x}_{k+1})
 *              )
 *      )
*/
tVectorXd
btGenNQRCalculator::CalcControlVector(const tContactInfo &contact_info)
{
    const auto &cur_info = mNQRFrameInfos[mRefFrameId];
    const auto &next_info = mNQRFrameInfos[mRefFrameId + 1];
    const tMatrixXd &Pk = cur_info.P_coef.asDiagonal();
    const tMatrixXd &Rk = cur_info.R_coef.asDiagonal();
    const tMatrixXd &G = GetG(contact_info);
    const tMatrixXd &S_mat_next = next_info.Sk_mat;
    const tVectorXd &bar_u_k = cur_info.mTotalControlVector_mocap;
    const tVectorXd &s_vec_next = next_info.sk_vec;
    const tVectorXd &hk = Geth();
    const tVectorXd &bar_x_next = next_info.mStateVector_mocap;

    tMatrixXd part1 = (Pk + Rk + G.transpose() * S_mat_next * G).inverse();
    tVectorXd part2 =
        Rk * bar_u_k -
        G.transpose() * (s_vec_next + S_mat_next * (hk - bar_x_next));
    tVectorXd uk = part1 * part2;
    std::cout << "[nqr] solve control vec = " << uk.transpose() << std::endl;
    std::cout << "[nqr] mocap control vec = " << bar_u_k.transpose()
              << std::endl;
    return uk;
}

/**
 * \brief       get the value of the transition euqation:
 *          x_{next} = G(x) * u + h(x) = f(x, u)
 * (used in the computation of numerical derivatives of dfdx and dfdu)
*/
tVectorXd btGenNQRCalculator::Getxnext(const tContactInfo &contact_info,
                                       const tVectorXd &x, const tVectorXd &u)
{
    mModel->PushState("get_x_next");
    // 1. apply current x and u
    assert(x.size() == mStateSize);
    assert(u.size() ==
           (num_of_underactuated_freedom + 3 * contact_info.size()));
    tVectorXd q_cur = x.segment(0, num_of_freedom),
              qdot_cur = x.segment(num_of_freedom, num_of_freedom);
    mModel->SetqAndqdot(q_cur, qdot_cur);

    // 2. get G in current configuration
    tMatrixXd G = GetG(contact_info);

    // 3. get h in current configuration
    tVectorXd h = Geth();

    // 4. get final result
    tVectorXd f_next = G * u + h;
    mModel->PopState("get_x_next");
    return f_next;
}

/**
 * \brief       calculate the derivation of total_control_jacobian:
 * 
 *  d(control_jacobian)/dq  =   [[dJvc^T/dq] : d(joint_torque_jacobian)/dq:]
 *                          =   [dJvc^T/dq : 0]
*/
void btGenNQRCalculator::CalcTotalControl_dJacobiandq(
    tEigenArr<tMatrixXd> &dJacdq, const tContactInfo &contact_info)
{
    /*
        4. calculate d(total_jacobian)/dq
        d(control_jacobian)/dq  =   [dJvc/dq : d(joint_torque_jacobian)/dq ]
                                =   [dJvc^T/dq : 0]
    */
    dJacdq.resize(num_of_freedom);

    tEigenArr<tMatrixXd> dContactJacdq;
    CalcdContactJacobiandq(contact_info, dContactJacdq);
    // mTraj->GetGen_dContactJacobian_dq_NoSet(frame_id, mModel, dContactJacdq);
    int total_control_size =
        3 * contact_info.size() + num_of_underactuated_freedom;
    for (int dof = 0; dof < num_of_freedom; dof++)
    {
        dJacdq[dof].noalias() =
            tMatrixXd::Zero(num_of_freedom, total_control_size);
        dJacdq[dof].block(0, 0, num_of_freedom, 3 * contact_info.size()) =
            dContactJacdq[dof].transpose();
    }
}

/**
 * \brief           Get the dJv/dq for contact points in current frame, Jv is the jacobian in some specified contact points
*/
void btGenNQRCalculator::CalcdContactJacobiandq(
    const tContactInfo &contact_info, tEigenArr<tMatrixXd> &dJacdq)
{
    // std::cout << "[warn] tmp hang out the check model state for dJacdq\n";
    int dof = mModel->GetNumOfFreedom();

    int contact_num = contact_info.size();
    dJacdq.resize(dof, tMatrixXd::Zero(3 * contact_num, dof));

    for (int i = 0; i < contact_num; i++)
    {
        auto &pt = contact_info[i];
        auto link_col = dynamic_cast<btGenRobotCollider *>(pt->mObj);
        assert(link_col != nullptr);
        int link_id = link_col->mLinkId;
        auto link = mModel->GetLinkById(link_id);

        // calculate at this contact point(local pos)
        tVector local_pos = pt->mLocalPos;
        btMathUtil::IsHomogeneousPos(local_pos);
        link->ComputeDJkvdq(local_pos.segment(0, 3));

        // fetch dJvdq
        for (int dof_id = 0; dof_id < dof; dof_id++)
        {
            dJacdq[dof_id].block(3 * i, 0, 1, dof) =
                link->GetdJKvdq_nxnversion(0).col(dof_id).transpose();
            dJacdq[dof_id].block(3 * i + 1, 0, 1, dof) =
                link->GetdJKvdq_nxnversion(1).col(dof_id).transpose();
            dJacdq[dof_id].block(3 * i + 2, 0, 1, dof) =
                link->GetdJKvdq_nxnversion(2).col(dof_id).transpose();
        }

        // restore dJvdq
        link->ComputeDJkvdq(tVector3d::Zero());
    }
}