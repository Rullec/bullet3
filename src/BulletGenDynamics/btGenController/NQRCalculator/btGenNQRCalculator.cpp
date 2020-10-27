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
*/
void btGenNQRCalculator::CalcNQR()
{
    // 1. allocation
    mNQRFrameInfos.resize(mTraj->mNumOfFrames);

    // 2. given all of the weight coef
    CalcNQRCoef();

    // 3. solve the ARE (algebraic riccati equation) recursively, inversely
    CalcNQRRiccati();
}

/**
 * \brief       set the control weight coefficient in NQR Info, including
 * - Q_coef
 * - R_coef
 * - P_coef
*/
void btGenNQRCalculator::CalcNQRCoef()
{

    for (int i = 0; i < mTraj->mNumOfFrames; i++)
    {
    }
}

/**
 * \brief       calculate the nqr variables size or configuration, including :
 * - state size (in state equation), 2 * dof
 * - contact force size (2 points on each foot indepently, load from an external file)
 * - control force size (n-6 dof, we have proved it's equivalent to the cartesian torque)
*/
void btGenNQRCalculator::CalcNQRVarSize() {}
void btGenNQRCalculator::CalcNQRRiccati() {}

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
