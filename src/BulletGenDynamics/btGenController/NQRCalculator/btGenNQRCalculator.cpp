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
    mSupposedContactPt.clear();
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
    InitSupposdedContactPoints(sup_pt_conf);
}
void btGenNQRCalculator::SetTraj(btTraj *traj_)
{
    btGenTargetCalculator::SetTraj(traj_);

    // do precomputation for this traj
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
    DrawSupposedContactPoints();
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
    mContactForceSize = 3 * mSupposedContactPt.size();
    auto root = mModel->GetRoot();
    mJointControlForceSize = num_of_freedom - root->GetNumOfFreedom();
    mTotalControlForceSize = mJointControlForceSize + mContactForceSize;

    // 3. coef
    for (int i = 0; i < mTraj->mNumOfFrames; i++)
    {
        auto info = mNQRFrameInfos[i];
        info.Q_coef = tVectorXd::Ones(mStateSize);
        info.R_coef = tVectorXd::Ones(mTotalControlForceSize);
        info.P_coef = tVectorXd::Ones(mTotalControlForceSize);
    }

    // 4. begin the main iter
    for (int frame_id = mTraj->mNumOfFrames - 1; frame_id >= 0; frame_id--)
    {
        // set q and qdot
        std::cout << "--------------- frame " << frame_id << " -------------\n";

        const tVectorXd &q = mTraj->mq[frame_id],
                        &qdot = mTraj->mqdot[frame_id];

        mModel->SetqAndqdot(q, qdot);

        // calc the contact and control
        CalcNQRContactAndControlForce(frame_id);
    }

    // 3. solve the ARE (algebraic riccati equation) recursively, inversely
    CalcNQRRiccati();
    mModel->PopState("calc_nqr");
}

void btGenNQRCalculator::CalcNQRContactAndControlForce(int frame_id)
{
    int num_of_supposed_contact_pt = mSupposedContactPt.size();
    // 1. contorl force
    tVectorXd control_force = mTraj->GetGenControlForce(frame_id, mModel);

    // 2. contact force:
    // legacy Q_contact, current total Jacobian, generalized inverse solution
    tVectorXd legacy_contact_force =
        mTraj->GetGenContactForce(frame_id, mModel);
    tMatrixXd supposed_jacobian =
        tMatrixXd::Zero(num_of_supposed_contact_pt * 3, num_of_freedom);
    for (int c_id = 0; c_id < num_of_supposed_contact_pt; c_id++)
    {
        auto pt = mSupposedContactPt[c_id];
        tMatrixXd jac;
        mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
            pt->mLinkId, pt->GetGlobalPos().segment(0, 3), jac);
        supposed_jacobian.block(c_id * 3, 0, 3, num_of_freedom) = jac;
    }

    tMatrixXd supposed_jacobian_inv =
        (supposed_jacobian * supposed_jacobian.transpose()).inverse() *
        supposed_jacobian;

    tVectorXd solved_contact_force =
        supposed_jacobian_inv * legacy_contact_force;
    std::cout << "solved contact force = " << solved_contact_force.transpose()
              << std::endl;
    exit(0);
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

/**
 * \brief       calculate contact points
*/
void btGenNQRCalculator::DrawSupposedContactPoints()
{
    std::cout << "suppose contact pts num = " << mSupposedContactPt.size()
              << std::endl;
    if (mDrawPointsList.size() == 0)
    {
        for (auto &pt : this->mSupposedContactPt)
        {
            DrawPoint(pt->GetGlobalPos().segment(0, 3));
        }
    }
    else
    {
        for (int i = 0; i < mSupposedContactPt.size(); i++)
        {
            btTransform trans = mDrawPointsList[i]->getWorldTransform();
            trans.setOrigin(btBulletUtil::tVectorTobtVector(
                mSupposedContactPt[i]->GetGlobalPos()));
            std::cout << "pt " << i << " global pos = "
                      << mSupposedContactPt[i]->GetGlobalPos().transpose()
                      << std::endl;
            mDrawPointsList[i]->setWorldTransform(trans);
        }
    }
}

/**
 * \brief           Load the supposed contact points from file
*/
void btGenNQRCalculator::InitSupposdedContactPoints(
    const std::string &conf_path)
{
    mSupposedContactPt.clear();
    Json::Value root;
    btJsonUtil::LoadJson(conf_path, root);
    std::string skeleton_path =
        btJsonUtil::ParseAsString("skeleton_path", root);
    int num = btJsonUtil::ParseAsInt("num_of_supposed_contact_points", root);
    const Json::Value &lst =
        btJsonUtil::ParseAsValue("supposed_contact_point_lst", root);

    if (skeleton_path != this->mModel->GetCharFile())
    {
        std::cout << "[error] nqr supposed contact skeleton file inconsistent "
                  << skeleton_path << " != " << mModel->GetCharFile()
                  << std::endl;
        exit(0);
    }
    if (num != lst.size())
    {
        std::cout << "[error] the supposed contact points num is "
                     "consistent "
                  << num << " != " << lst.size() << std::endl;
        exit(1);
    }

    for (int i = 0; i < num; i++)
    {
        auto cur_pt = lst[i];
        std::string link_name = btJsonUtil::ParseAsString("link_name", cur_pt);
        int link_id = btJsonUtil::ParseAsInt("link_id", cur_pt);
        tVector3d local_pos = btJsonUtil::ReadVectorJson(
                                  btJsonUtil::ParseAsValue("local_pos", cur_pt))
                                  .segment(0, 3);

        // check link id and link name
        auto link = mModel->GetLinkById(link_id);
        if (link->GetName() != link_name)
        {
            std::cout << "[error] nqr supposed pt: the given link name "
                      << link_name << " != "
                      << " model link name " << link->GetName() << std::endl;
            exit(1);
        }

        mSupposedContactPt.push_back(new tSupposedContactPt(
            this->mModel, link_id, btMathUtil::Expand(local_pos, 1)));
    }
}

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
    // 1. validate the frame id

    // 2.
}

/**
 * \brief           
*/
void btGenNQRCalculator::VerifyNQRSystemLinearzation(int frame) {}