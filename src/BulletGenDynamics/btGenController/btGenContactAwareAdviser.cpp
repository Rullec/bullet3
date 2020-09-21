#include "btGenContactAwareAdviser.h"
#include "BulletGenDynamics/btGenController/FBFOptimizer/btGenFrameByFrameOptimizer.h"
#include "BulletGenDynamics/btGenController/btGenFeature.h"
#include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenSolver/ContactSolver.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include <fstream>
#include <iostream>
tVectorXd ConvertPoseToq(const tVectorXd &pose, cRobotModelDynamics *model);

// btGenContactAwareAdviser::btGenContactAwareAdviser(btGeneralizeWorld* world,
// const std::string& path, double W, double Wm) : mW(W), mWm(Wm) std::string
// new_path = "new.out";
btGenContactAwareAdviser::btGenContactAwareAdviser(btGeneralizeWorld *world)
{
    mCurdt = 0;
    mHasRefTraj = false;
    mInternalFrameId = 0;

    mDrawReferenceTrajCharacter = false;
    mDrawTargetFBFCharacter = false;
    mMaxFrame = 1e4;
    mStartFrame = 1;
    mFeatureVectorFile = "";
    mResolveControlToruqe = false;
    // mFBFPosCoef = 0.5;
    // mFBFVelCoef = 1;
    // mFBFAccelCoef = 1;
    mEnableSyncTrajPeriodly = false;
    mSyncTrajPeriod = 100;

    mModel = nullptr;
    mWorld = world;
    mFeatureVector = new btGenFeatureArray();
    mFBFOptimizer = nullptr;
    mOutputTraj = nullptr;
    mRefTraj = nullptr;
    mRefTrajPath = "";
    mOutputTrajPath = "";
    mEnableOutput = false;

    mRefModel = nullptr;
    mOutputControlDiff = false;
}

btGenContactAwareAdviser::~btGenContactAwareAdviser()
{
    delete mRefModel;
    delete mFeatureVector;
}

int btGenContactAwareAdviser::GetInternalFrameId() const
{
    return this->mInternalFrameId;
}

/**
 * \brief           Set the wait-to-solve trajectory
 * \param ref_traj          set the reference trajectory
 * \param output_traj       set the output trajectory path
 * \param enable_output     output or not, default not (false)
 */
void btGenContactAwareAdviser::SetTraj(const std::string &ref_traj,
                                       const std::string &output_traj,
                                       bool enable_output /*= false*/)
{
    mEnableOutput = enable_output;
    mRefTrajPath = ref_traj;
    mOutputTrajPath = output_traj;
    LoadTraj(mRefTrajPath);

    if (mModel->GetNumOfFreedom() != mRefTraj->mq[mInternalFrameId].size())
    {
        std::cout << "btGenContactAwareAdviser::SetTraj the model is "
                     "inconsistent with the given motion\n";
        exit(0);
    }
    if (mInternalFrameId >= mRefTraj->mNumOfFrames - 2)
    {
        std::cout << "btGenContactAwareAdviser::SetTraj the traj has "
                  << mRefTraj->mNumOfFrames << " frames but the start frame is "
                  << mInternalFrameId << ", doesn't match\n";
        exit(0);
    }
    if (mInternalFrameId)
        mModel->SetqAndqdot(mRefTraj->mq[mInternalFrameId],
                            mRefTraj->mqdot[mInternalFrameId]);
    // std::cout << "[adviser] init q = "
    //           << mRefTraj->mq[mInternalFrameId].transpose() << std::endl;
    // std::cout << "[adviser] init qdot = "
    //           << mRefTraj->mqdot[mInternalFrameId].transpose() << std::endl;
    mFBFOptimizer->SetTraj(mRefTraj);
    mHasRefTraj = true;
}
void btGenContactAwareAdviser::Init(cRobotModelDynamics *model_,
                                    const std::string &contact_aware_config)
{
    ReadConfig(contact_aware_config);
    mModel = model_;

    num_of_freedom = mModel->GetNumOfFreedom();
    num_of_underactuated_freedom = num_of_freedom - 6;

    mFBFOptimizer = new btGenFrameByFrameOptimizer();

    // btGenFrameByFrameOptimizer::tParams FBFOptimizer_params;
    // FBFOptimizer_params.mWorld = mWorld;
    // FBFOptimizer_params.mFBFEnergyCoeffPos = mFBFPosCoef;
    // FBFOptimizer_params.mFBFEnergyCoeffVel = mFBFVelCoef;
    // FBFOptimizer_params.mFBFEnergyCoeffAccel = mFBFAccelCoef;

    mFBFOptimizer->Init(mWorld, mFrameByFrameConfig);
    mFeatureVector->Init(mFeatureVectorFile, mModel, mWorld->GetGravity());

    if (mDrawReferenceTrajCharacter || mDrawTargetFBFCharacter)
    {
        CreateRefChar();
    }
    PostProcess();
}

/**
 * \brief               update the adviser
 *  calculate and offer new action guidence for the character
 */
void btGenContactAwareAdviser::Update(double dt)
{
    std::cout << "---------------------frame " << mInternalFrameId << std::endl;
    mCurdt = dt;
    if (mRefTraj == nullptr)
        std::cout << "[error] adviser guide traj hasn't been set\n", exit(0);
    if (std::fabs(mRefTraj->mTimestep - dt) > 1e-8)
    {
        std::cout << "[error] btGenContactAwareAdviser ref motion dt "
                  << mRefTraj->mTimestep << " != sim dt " << dt << std::endl;
        exit(0);
    }
    if (IsEnd() == true)
    {
        std::cout << "[error] The contact-aware adviser has terminated, it "
                     "shouldn't be updated anymore\n";
        exit(0);
    }

    RecordTraj();
    // // check the current q difference
    // {
    // 	tVectorXd q_cur = mModel->Getq(),
    // 			  qdot_cur = mModel->Getqdot();

    // 	tVectorXd q_ref = mRefTraj->mq[mInternalFrameId],
    // 			  qdot_ref = mRefTraj->mqdot[mInternalFrameId];
    // 	tVectorXd q_diff = q_cur - q_ref,
    // 			  qdot_diff = qdot_cur - qdot_ref;
    // 	// std::cout << "[ref] qdot diff = " << qdot_diff.transpose() <<
    // std::endl;
    // 	// std::cout << "[ref] q diff = " << q_diff.transpose() << std::endl;
    // }
    GetTargetInfo(dt, mTargetAccel, mTargetVel, mTargetPos, mTargetTau);
    mFeatureVector->Eval(dt, mTargetAccel, mTargetVel, mTargetPos, mTargetTau,
                         mH, mE, mf);

    UpdateRefChar();
    // std::cout << "[debug] [new] frame " << mInternalFrameId << " N norm = "
    // << mN.norm() << std::endl; std::cout << "[debug] [new] frame " <<
    // mInternalFrameId << " H norm = " << mH.norm() << std::endl; std::cout <<
    // "[debug] [new] frame " << mInternalFrameId << " E norm = " << mE.norm()
    // << std::endl; std::cout << "[debug] [new] frame " << mInternalFrameId <<
    // " f norm = " << mf.norm() << std::endl;
    mInternalFrameId++;
    // if (mInternalFrameId == 3) exit(0);
}

/**
 * \brief               load the guidence trajectory
 */

void btGenContactAwareAdviser::LoadTraj(const std::string &path)
{
    if (mRefTraj != nullptr)
        delete mRefTraj, mRefTraj = nullptr;
    if (mOutputTraj != nullptr)
        delete mOutputTraj, mOutputTraj = nullptr;
    if (mRefTraj == nullptr)
        mRefTraj = new btTraj();
    if (mOutputTraj == nullptr)
        mOutputTraj = new btTraj();
    if (mModel == nullptr)
    {
        std::cout << "adviser LoadTraj but mModel is empty\n";
        exit(0);
    }
    mRefTraj->LoadTraj(path, mModel, mMaxFrame);
    // resolve the active force

    if (mResolveControlToruqe)
        ResolveActiveForce();

    // load the saved traj in order to shape that
    mOutputTraj->LoadTraj(path, mModel, mMaxFrame);
    mOutputTraj->Reshape(std::min(mMaxFrame, mRefTraj->mNumOfFrames) - 2);
}

/**
 * \brief					Resolve the active force by the
 * qddot and contact forces
 */
void btGenContactAwareAdviser::ResolveActiveForce()
{
    /*
                    Mqddot + Cqdot = QG + Qcontrol + Qcontact
                    Qcontrol = Mqddot + Cqdot - QG - Qcontact
            */
    int num_of_frame = mRefTraj->mq.size();
    int num_of_freedom = mModel->GetNumOfFreedom();
    for (int frame_id = 1; frame_id < num_of_frame - 1; frame_id++)
    {
        if (frame_id % 100 == 0)
            std::cout << "[debug] resolve frame " << frame_id << std::endl;
        const tVectorXd &q = mRefTraj->mq[frame_id],
                        &qdot = mRefTraj->mqdot[frame_id],
                        &qddot = mRefTraj->mqddot[frame_id];
        mModel->SetqAndqdot(q, qdot);

        const tMatrixXd &M = mModel->GetMassMatrix(),
                        &C = mModel->GetCoriolisMatrix();
        tVectorXd LHS = M * qddot + C * qdot;

        // contact gravity and contact forces
        tVectorXd G = mModel->CalcGenGravity(mWorld->GetGravity());

        // contact forces
        const auto &fcs = mRefTraj->mContactForce[frame_id];
        tMatrixXd jac;
        tVectorXd Qcontact = tVectorXd::Zero(num_of_freedom);
        for (const auto &fc : fcs)
        {
            int link_id = dynamic_cast<btGenRobotCollider *>(fc->mObj)->mLinkId;
            mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                link_id, fc->mWorldPos.segment(0, 3), jac);

            Qcontact += jac.transpose() * fc->mForce.segment(0, 3);
        }
        mRefTraj->mActiveForce[frame_id] = LHS - G - Qcontact;
    }
}

/**
 * \brief					Init the pose of guided
 * character by the input trajectory
 */
void btGenContactAwareAdviser::PostProcess()
{
    if (mStartFrame < 1)
    {
        std::cout << "[error] post process: illegal start frame " << mStartFrame
                  << std::endl;
        exit(0);
    }
    mInternalFrameId = mStartFrame;
    mModel->SetContactAwareAdviser(this);
    mModel->SetEnableContactAwareAdviser(true);

    // create the N matrix
    mN = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    mN.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
}

/**
 * \brief					input the contact forces
 * (generalized), calculate and apply the active control torque
 */
tVectorXd btGenContactAwareAdviser::CalcControlForce(const tVectorXd &Q_contact)
{
    // std::cout << "[adviser] contact force = " << Q_contact.norm() <<
    // std::endl; std::cout << "mH = \n"
    // 		  << mH << std::endl;
    tVectorXd Q_active = mH * Q_contact + mE.inverse() * mf;
    // std::cout << "[control] contact force norm = " << Q_contact.norm() <<
    // std::endl; std::cout << "[control] mH norm = " << mH.norm() << std::endl;
    // std::cout << "[control] mE norm = " << mE.norm() << std::endl;
    // std::cout << "[control] mf norm = " << mf.norm() << std::endl;
    // std::cout << "[control] mf = " << mf.transpose() << std::endl;
    tVectorXd ref_force =
        mRefTraj->mActiveForce[mInternalFrameId - 1].transpose().segment(
            6, mModel->GetNumOfFreedom() - 6);
    // std::cout << "q = " << mModel->Getq().segment(0, 3).transpose() <<
    // std::endl; std::cout << "[adviser] ref ctrl force = " <<
    // ref_force.transpose() << std::endl; std::cout << "[adviser] calced ctrl
    // force = " << Q_active.transpose() << std::endl;

    mOutputTraj->mActiveForce[mInternalFrameId - 1] =
        tVectorXd::Zero(num_of_freedom);
    mOutputTraj->mActiveForce[mInternalFrameId - 1].segment(
        6, num_of_underactuated_freedom) = Q_active;
    // if (Q_active.norm() > 1e8)
    // {
    // 	std::cout << "[warn] force explode, use the reference torque = " <<
    // ref_force.transpose() << std::endl; 	Q_active = ref_force;
    // }
    mCtrlForce = Q_active;
    return Q_active;
}

tVectorXd btGenContactAwareAdviser::GetPrevControlForce() { return mCtrlForce; }
/**
 * \brief					Load config
 */
void btGenContactAwareAdviser::ReadConfig(const std::string &config)
{
    std::cout << "[debug] read config of adviser from " << config << std::endl;
    Json::Value root;
    btJsonUtil::LoadJson(config, root);
    mResolveControlToruqe =
        btJsonUtil::ParseAsBool("resolve_control_tau", root);
    mDrawReferenceTrajCharacter =
        btJsonUtil::ParseAsBool("draw_reference_traj_character", root);
    mDrawTargetFBFCharacter =
        btJsonUtil::ParseAsBool("draw_target_FBF_character", root);
    mMaxFrame = btJsonUtil::ParseAsInt("max_frame", root);
    mStartFrame = btJsonUtil::ParseAsInt("start_frame", root);
    mOutputTraj = nullptr;
    mFeatureVectorFile = btJsonUtil::ParseAsString("feature_vector_file", root);
    mOutputControlDiff = btJsonUtil::ParseAsBool("output_diff", root);
    mEnableSyncTrajPeriodly =
        btJsonUtil::ParseAsBool("enable_sync_traj_periodly", root);
    mSyncTrajPeriod = btJsonUtil::ParseAsInt("sync_traj_period", root);

    mFrameByFrameConfig = btJsonUtil::ParseAsValue("ctrl_config", root);
    // mFBFPosCoef =
    //     btJsonUtil::ParseAsDouble("dynamic_pos_energy_coef", ctrl_config);
    // mFBFVelCoef =
    //     btJsonUtil::ParseAsDouble("dynamic_vel_energy_coef", ctrl_config);
    // mFBFAccelCoef =
    //     btJsonUtil::ParseAsDouble("dynamic_accel_energy_coef", ctrl_config);
    if (mDrawTargetFBFCharacter && mDrawReferenceTrajCharacter)
    {
        std::cout << "[error] ContactAwareAdviser::Readconfig: 2 draw options"
                     "are both turned on, exit\n";
        exit(0);
    }
}

void btGenContactAwareAdviser::UpdateMultibodyVelocityDebug(double dt)
{
    tVectorXd cur_qddot = mModel->Getqddot();
    tVectorXd ref_qddot = mRefTraj->mqddot[mInternalFrameId - 1];
    if (mOutputControlDiff == true)
    {
        // std::ofstream fout(mOutputControlDiffFile, std::ios::app);
        // fout << "[ref] qddot diff = " << (cur_qddot -
        // ref_qddot).transpose()
        // << std::endl; fout.close(); std::cout << "[ref] qddot = " <<
        // ref_qddot.transpose() << std::endl; std::cout << "[true] qddot = "
        // <<
        // cur_qddot.transpose() << std::endl;
        tVectorXd q_diff = mModel->Getq() - mRefTraj->mq[mInternalFrameId - 1],
                  qdot_diff =
                      mModel->Getqdot() - mRefTraj->mqdot[mInternalFrameId - 1];
        tVectorXd qddot_diff = cur_qddot - ref_qddot;
        std::cout << "[true] qddot diff = " << qddot_diff.norm()
                  << ", qdot diff = " << qdot_diff.norm()
                  << ", q diff = " << q_diff.norm() << std::endl;
    }

    mModel->UpdateVelocity(dt, false);
}
/**
 * \brief					Update the multibody velocity,
 * used in debug mode
 */
void btGenContactAwareAdviser::UpdateMultibodyVelocityAndTransformDebug(
    double dt)
{
    tVectorXd cur_qddot = mModel->Getqddot();
    tVectorXd ref_qddot = mRefTraj->mqddot[mInternalFrameId - 1];
    if (mOutputControlDiff == true)
    {
        // std::ofstream fout(mOutputControlDiffFile, std::ios::app);
        // fout << "[ref] qddot diff = " << (cur_qddot -
        // ref_qddot).transpose()
        // << std::endl; fout.close(); std::cout << "[ref] qddot = " <<
        // ref_qddot.transpose() << std::endl; std::cout << "[true] qddot = "
        // <<
        // cur_qddot.transpose() << std::endl;
        tVectorXd q_diff = mModel->Getq() - mRefTraj->mq[mInternalFrameId - 1],
                  qdot_diff =
                      mModel->Getqdot() - mRefTraj->mqdot[mInternalFrameId - 1];
        tVectorXd qddot_diff = cur_qddot - ref_qddot;
        std::cout << "[true] qddot diff = " << qddot_diff.norm()
                  << ", qdot diff = " << qdot_diff.norm()
                  << ", q diff = " << q_diff.norm() << std::endl;
    }

    mModel->UpdateVelocityAndTransform(dt);

    // std::cout << "mEnable sync traj per = " << mEnableSyncTrajPeriodly
    //           << ", internal frame id " << mInternalFrameId
    //           << ", sync period = " << mSyncTrajPeriod << std::endl;
    if (mEnableSyncTrajPeriodly == true &&
        mInternalFrameId % mSyncTrajPeriod == 0)
    {
        std::cout << "[sync] internal frame " << mInternalFrameId
                  << ", sync traj period " << mSyncTrajPeriod << std::endl;

        mModel->SetqAndqdot(mRefTraj->mq[mInternalFrameId],
                            mRefTraj->mqdot[mInternalFrameId]);
    }
}

tVectorXd btGenContactAwareAdviser::CalcLCPResidual(double dt) const
{
    // std::cout << "Adviser Residual hasn't been implemented\n", exit(0);
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    // mModel
    const tVectorXd &QG = mModel->CalcGenGravity(mWorld->GetGravity());
    const tVectorXd &qdot = mModel->Getqdot();
    tVectorXd residual =
        dt * Minv *
            (QG + mN * mE.inverse() * mf - mModel->GetCoriolisMatrix() * qdot) +
        qdot;
    // std::cout << "[debug] [new] frame " << mInternalFrameId << " residual = "
    // << residual.transpose() << std::endl;
    return residual;
}
tMatrixXd btGenContactAwareAdviser::CalcLCPPartBPrefix() const
{
    // std::cout << "Adviser PartB hasn't been implemented\n", exit(0);
    int num_of_freedom = mModel->GetNumOfFreedom();
    tMatrixXd partb =
        tMatrixXd::Identity(num_of_freedom, num_of_freedom) + mN * mH;
    // std::cout << "[debug] [new] frame " << mInternalFrameId << " PartB norm =
    // " << partb.norm() << std::endl; std::cout << "[debug] [new] frame " <<
    // mInternalFrameId << " PartB mN = " << mN.norm() << std::endl; std::cout
    // << "[debug] [new] frame " << mInternalFrameId << " PartB mE = " <<
    // mH.norm() << std::endl;
    return partb;
    // return tMatrixXd::Zero(0, 0);
}

bool btGenContactAwareAdviser::IsEnd()
{
    return mInternalFrameId >= (mRefTraj->mq.size() - 2);
}

/**
 * \brief					Get current reference gen accel
 * "qddot_target" and "tau_target"
 */
void btGenContactAwareAdviser::GetTargetInfo(double dt, tVectorXd &qddot_target,
                                             tVectorXd &qdot_target,
                                             tVectorXd &q_target,
                                             tVectorXd &tau_target)
{
    // if (false == mEnableFrameByFrameCtrl)
    // {
    //     int num_of_underactuated_freedom = mModel->GetNumOfFreedom() - 6;
    //     qddot_target = mRefTraj->mqddot[mInternalFrameId];
    //     tau_target = mRefTraj->mActiveForce[mInternalFrameId].segment(
    //         6, num_of_underactuated_freedom);
    // }
    // else
    // {

    // }
    mFBFOptimizer->CalcTarget(dt, mInternalFrameId, qddot_target, qdot_target,
                              q_target, tau_target);
}

/**
 * \brief					Create the reference character
 * in ref trajectory
 */
void btGenContactAwareAdviser::CreateRefChar()
{
    mRefModel = new cRobotModelDynamics();

    mRefModel->Init(mModel->GetCharFile().c_str(), mModel->GetScale(),
                    ModelType::JSON);
    mRefModel->InitSimVars(mWorld->GetInternalWorld(), true, true, false);
    // mRefModel->SetqAndqdot(mRefTraj->mq[mInternalFrameId],
    // mRefTraj->mqdot[mInternalFrameId]);
}

/**
 * \brief					update the reference character
 */
void btGenContactAwareAdviser::UpdateRefChar()
{
    if (mRefModel != nullptr)
    {
        // mRefModel->SetqAndqdot(mRefTraj->mq[mInternalFrameId],
        // mRefTraj->mqdot[mInternalFrameId]);
        if (mDrawReferenceTrajCharacter)
        {
            mRefModel->SetqAndqdot(mRefTraj->mq[mInternalFrameId],
                                   mRefTraj->mqdot[mInternalFrameId]);
        }
        if (mDrawTargetFBFCharacter)
        {
            tVectorXd qdot_next = mTargetAccel * mCurdt + mModel->Getqdot();
            tVectorXd q_next = qdot_next * mCurdt + mModel->Getq();
            mRefModel->SetqAndqdot(q_next, qdot_next);
        }
    }
}

/**
 * \brief					record the current contact info
 * and motion status into the mSaveTraj
 */
void btGenContactAwareAdviser::RecordTraj()
{
    // record motion
    mOutputTraj->mq[mInternalFrameId] = mModel->Getq();
    // std::cout << "record traj " << mInternalFrameId << " " <<
    // mSavedTraj->mq[mInternalFrameId].transpose() << std::endl; record
    // contacts

    {
        // const std::vector<btGenContactForce*>& cur_contacts =;
        auto &rec_contacts = mOutputTraj->mContactForce[mInternalFrameId - 1];
        for (auto &x : rec_contacts)
            delete x;
        rec_contacts.clear();
        int num_of_contact_forces = mWorld->GetContactForces().size();
        // std::cout << "[rec] contact num = " << num_of_contact_forces
        //           << std::endl;
        for (int id = 0; id < num_of_contact_forces; id++)
        {
            auto cur_contact = mWorld->GetContactForces()[id];
            if (cur_contact->mObj->GetType() == eColObjType::RobotCollder)
            {
                rec_contacts.push_back(new btGenContactForce(
                    cur_contact->mObj, cur_contact->mForce,
                    cur_contact->mWorldPos, cur_contact->mIsSelfCollision));
            }
        }
    }
    // exit(0);
}

/**
 * \brief               Reset the contact aware adviser, which means the current
 * trajectory should be deleted and cannot be used to do tracking
 */
void btGenContactAwareAdviser::Reset()
{
    std::cout << "[adviser] adviser reset!\n";
    if (mEnableOutput == true)
    {
        std::cout << "[adviser] Save traj v2 to " << mOutputTrajPath
                  << std::endl;
        mOutputTraj->SaveTraj(mOutputTrajPath, mModel);
    }
    else
    {
        std::cout << "[adviser] Save traj is diabled\n";
    }

    mInternalFrameId = mStartFrame;
    mHasRefTraj = false;
    mRefTrajPath.clear();
    mOutputTrajPath.clear();
    mFBFOptimizer->Reset();
}

btTraj *btGenContactAwareAdviser::GetRefTraj() { return this->mRefTraj; }