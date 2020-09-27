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
std::string debug_path = "numeric.log";
btGenContactAwareAdviser::btGenContactAwareAdviser(btGeneralizeWorld *world)
{
    mCurdt = 0;
    mHasRefTraj = false;
    mSimFrameId = 0;
    mRefFrameId = 0;

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

    mRefTrajModel = nullptr;
    mFBFTrajModel = nullptr;
    mOutputControlDiff = false;

    std::ofstream fout(debug_path);
    fout << "";
    fout.close();
}

btGenContactAwareAdviser::~btGenContactAwareAdviser()
{
    delete mRefTrajModel;
    delete mFBFTrajModel;
    delete mFeatureVector;
    delete mRefTraj;
}

// int btGenContactAwareAdviser::GetInternalFrameId() const
// {
//     return this->mInternalFrameId;
// }

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

    if (mModel->GetNumOfFreedom() != mRefTraj->mq[mRefFrameId].size())
    {
        std::cout << "btGenContactAwareAdviser::SetTraj the model is "
                     "inconsistent with the given motion\n";
        exit(0);
    }
    if (mRefFrameId >= mRefTraj->mNumOfFrames - 2)
    {
        std::cout << "btGenContactAwareAdviser::SetTraj the traj has "
                  << mRefTraj->mNumOfFrames << " frames but the start frame is "
                  << mRefFrameId << ", doesn't match\n";
        exit(0);
    }
    if (mRefFrameId)
        mModel->SetqAndqdot(mRefTraj->mq[mRefFrameId],
                            mRefTraj->mqdot[mRefFrameId]);
    // std::cout << "[adviser] init q = "
    //           << mRefTraj->mq[mInternalFrameId].transpose() << std::endl;
    // std::cout << "[adviser] init qdot = "
    //           << mRefTraj->mqdot[mInternalFrameId].transpose() << std::endl;
    mFBFOptimizer->SetTraj(mRefTraj);
    mHasRefTraj = true;

    if (mEnableInitStateLoad == true)
    {
        LoadInitState();
    }
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

    CreateRefChar();
    PostProcess();
}

/**
 * \brief               update the adviser
 *  calculate and offer new action guidence for the character
 */
void btGenContactAwareAdviser::Update(double dt)
{
    std::cout << "---------------------frame " << mSimFrameId << " ref "
              << mRefFrameId << std::endl;
    // std::cout << "[debug] model q = " << mModel->Getq().transpose()
    //           << std::endl;
    // std::cout << "[debug] model qdot = " << mModel->Getqdot().transpose()
    //           << std::endl;
    if (mEnableStateSave)
        SaveCurrentState();
    mCurdt = dt;
    if (mRefTraj == nullptr)
        std::cout << "[error] adviser guide traj hasn't been set\n", exit(0);
    // std::cout << "contact aware ref traj dt = " << mRefTraj->mTimestep << " "
    //           << dt << std::endl;
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
    if (mEnableOnlyFBFControl == true)
    {
        mFBFOptimizer->ControlByFBF();
    }
    else
    {
        mFeatureVector->Eval(dt, mTargetAccel, mTargetVel, mTargetPos,
                             mTargetTau, mH, mE, mf);
    }

    UpdateRefChar();
    // std::cout << "[debug] [new] frame " << mInternalFrameId << " N norm = "
    // << mN.norm() << std::endl; std::cout << "[debug] [new] frame " <<
    // mInternalFrameId << " H norm = " << mH.norm() << std::endl; std::cout <<
    // "[debug] [new] frame " << mInternalFrameId << " E norm = " << mE.norm()
    // << std::endl; std::cout << "[debug] [new] frame " << mInternalFrameId <<
    // " f norm = " << mf.norm() << std::endl;
    // mInternalFrameId++;
    UpdateReferenceTraj();
    mSimFrameId++;
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
    mSimFrameId = mStartFrame;
    mRefFrameId = mStartFrame;
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
    if (mEnableOnlyFBFControl == true)
    {
        std::cout << "[error] This function CalcControlForce should not be "
                     "called in only FBF model, exit\n";
        exit(0);
    }
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
        mRefTraj->mActiveForce[mRefFrameId - 1].transpose().segment(
            6, mModel->GetNumOfFreedom() - 6);
    // std::cout << "q = " << mModel->Getq().segment(0, 3).transpose() <<
    // std::endl; std::cout << "[adviser] ref ctrl force = " <<
    // ref_force.transpose() << std::endl; std::cout << "[adviser] calced ctrl
    // force = " << Q_active.transpose() << std::endl;

    mOutputTraj->mActiveForce[mRefFrameId - 1] =
        tVectorXd::Zero(num_of_freedom);
    mOutputTraj->mActiveForce[mRefFrameId - 1].segment(
        6, num_of_underactuated_freedom) = Q_active;
    // if (Q_active.norm() > 1e8)
    // {
    // 	std::cout << "[warn] force explode, use the reference torque = " <<
    // ref_force.transpose() << std::endl; 	Q_active = ref_force;
    // }
    mCtrlForce = Q_active;
    std::ofstream fout(debug_path, std::ios::app);

    fout << "[numeric] contact force = " << Q_contact.transpose() << std::endl;
    fout << "[numeric] control force = " << mCtrlForce.transpose() << std::endl;
    fout.close();
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
    mEnableStateSave = btJsonUtil::ParseAsBool("enable_state_save", root);
    mEnableInitStateLoad =
        btJsonUtil::ParseAsBool("enable_init_state_load", root);
    mInitStateFile = btJsonUtil::ParseAsString("init_state_file", root);
    mStateSaveDir = btJsonUtil::ParseAsString("save_dir", root);
    mEnableOnlyFBFControl =
        btJsonUtil::ParseAsBool("enable_only_FBF_control", root);
    mEnableRefTrajDelayedUpdate =
        btJsonUtil::ParseAsBool("enable_ref_traj_delayed_update", root);

    if (mEnableOnlyFBFControl == true)
    {
        // we must check, the contact mode of gen world is exactly "No"
        if (btGeneralizeWorld::eContactResponseMode::NoMode !=
            mWorld->GetContactResponseMode())
        {
            std::cout << "[error] The contact response mode of gen world is "
                         "not No. In this case, the only FBF control is "
                         "prohibited to be ture\n";
            exit(1);
        }
    }
    // validate the state save dir
    if (mEnableStateSave == true)
    {
        std::ofstream ftest(mStateSaveDir + "1.txt", std::ios::app);
        if (ftest.fail() == true)
        {
            std::cout << "[error] state save dir " << mStateSaveDir
                      << " doesn't exist\n";
            exit(0);
        }
    }
    // mFBFPosCoef =
    //     btJsonUtil::ParseAsDouble("dynamic_pos_energy_coef", ctrl_config);
    // mFBFVelCoef =
    //     btJsonUtil::ParseAsDouble("dynamic_vel_energy_coef", ctrl_config);
    // mFBFAccelCoef =
    //     btJsonUtil::ParseAsDouble("dynamic_accel_energy_coef", ctrl_config);
    // if (mDrawTargetFBFCharacter && mDrawReferenceTrajCharacter)
    // {
    //     std::cout << "[error] ContactAwareAdviser::Readconfig: 2 draw
    //     options"
    //                  "are both turned on, exit\n";
    //     exit(0);
    // }
}

// void btGenContactAwareAdviser::UpdateMultibodyVelocityDebug(double dt)
// {
//     // tVectorXd cur_qddot = mModel->Getqddot();
//     // tVectorXd ref_qddot = mRefTraj->mqddot[mInternalFrameId - 1];
//     if (mOutputControlDiff == true)
//     {
//         // std::ofstream fout(mOutputControlDiffFile, std::ios::app);
//         // fout << "[ref] qddot diff = " << (cur_qddot -
//         // ref_qddot).transpose()
//         // << std::endl; fout.close(); std::cout << "[ref] qddot = " <<
//         // ref_qddot.transpose() << std::endl; std::cout << "[true] qddot = "
//         // <<
//         // cur_qddot.transpose() << std::endl;
//         // tVectorXd q_diff = mModel->Getq() - mRefTraj->mq[mInternalFrameId
//         -
//         // 1],
//         //           qdot_diff =
//         //               mModel->Getqdot() - mRefTraj->mqdot[mInternalFrameId
//         -
//         //               1];
//         // tVectorXd qddot_diff = cur_qddot - ref_qddot;
//         // std::cout << "[true] qddot diff = " << qddot_diff.norm()
//         //           << ", qdot diff = " << qdot_diff.norm()
//         //           << ", q diff = " << q_diff.norm() << std::endl;

//     }

//     mModel->UpdateVelocity(dt, false);
// }
/**
 * \brief					Update the multibody velocity,
 * used in debug mode
 */
void btGenContactAwareAdviser::UpdateMultibodyVelocityAndTransformDebug(
    double dt)
{
    tVectorXd qddot = mModel->Getqddot();
    mModel->UpdateVelocityAndTransform(dt);
    tVectorXd q = mModel->Getq(), qdot = mModel->Getqdot();
    if (mOutputControlDiff == true)
    {
        // std::ofstream fout(mOutputControlDiffFile, std::ios::app);
        // fout << "[ref] qddot diff = " << (cur_qddot -
        // ref_qddot).transpose()
        // << std::endl; fout.close(); std::cout << "[ref] qddot = " <<
        // ref_qddot.transpose() << std::endl; std::cout << "[true] qddot = "
        // <<
        // cur_qddot.transpose() << std::endl;

        const tVectorXd &ref_traj_q = mRefTraj->mq[mRefFrameId],
                        ref_traj_qdot = mRefTraj->mqdot[mRefFrameId],
                        ref_traj_qddot = mRefTraj->mqddot[mRefFrameId - 1];
        std::ofstream fout(debug_path, std::ios::app);
        fout << "[numeric] ref q = " << ref_traj_q.transpose() << std::endl;
        fout << "[numeric] ref qdot = " << ref_traj_qdot.transpose()
             << std::endl;
        fout << "[numeric] ref qddot = " << ref_traj_qddot.transpose()
             << std::endl;
        fout << "[numeric] ctrl_res q = " << q.transpose() << std::endl;
        fout << "[numeric] ctrl_res qdot = " << qdot.transpose() << std::endl;
        fout << "[numeric] ctrl_res qddot = " << qddot.transpose() << std::endl;

        {
            tVectorXd q_diff = q - ref_traj_q, qdot_diff = qdot - ref_traj_qdot,
                      qddot_diff = qddot - ref_traj_qddot;

            std::cout << "[debug] ctrl_res and ref_traj q diff "
                      << q_diff.norm() << " qdot diff " << qdot_diff.norm()
                      << " qddot diff = " << qddot_diff.norm() << std::endl;
        }

        {
            tVectorXd q_diff = q - mTargetPos, qdot_diff = qdot - mTargetVel,
                      qddot_diff = qddot - mTargetAccel;

            std::cout << "[debug] ctrl_res and target_traj q diff "
                      << q_diff.norm() << " qdot diff " << qdot_diff.norm()
                      << " qddot diff = " << qddot_diff.norm() << std::endl;
        }

        {
            tVectorXd q_diff = ref_traj_q - mTargetPos,
                      qdot_diff = ref_traj_qdot - mTargetVel,
                      qddot_diff = ref_traj_qddot - mTargetAccel;

            std::cout << "[debug] ref_traj and target_traj q diff "
                      << q_diff.norm() << " qdot diff " << qdot_diff.norm()
                      << " qddot diff = " << qddot_diff.norm() << std::endl;
        }
    }
    // std::cout << "mEnable sync traj per = " << mEnableSyncTrajPeriodly
    //           << ", internal frame id " << mInternalFrameId
    //           << ", sync period = " << mSyncTrajPeriod << std::endl;

    if (mEnableSyncTrajPeriodly == true && mRefFrameId % mSyncTrajPeriod == 0)
    {
        std::cout << "[sync] ref frame " << mRefFrameId << ", sync traj period "
                  << mSyncTrajPeriod << std::endl;

        mModel->SetqAndqdot(mRefTraj->mq[mRefFrameId],
                            mRefTraj->mqdot[mRefFrameId]);
    }
}

tVectorXd btGenContactAwareAdviser::CalcLCPResidual(double dt) const
{
    if (mEnableOnlyFBFControl == true)
    {
        std::cout << "[error] This function CalcLCPResidual should not be "
                     "called in only FBF model, exit\n";
        exit(0);
    }
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
    if (mEnableOnlyFBFControl == true)
    {
        std::cout << "[error] This function CalcLCPPartBPrefix should not be "
                     "called in only FBF model, exit\n";
        exit(0);
    }
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
    return mRefFrameId >= (mRefTraj->mq.size() - 2);
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
    mFBFOptimizer->CalcTarget(dt, mRefFrameId, qddot_target, qdot_target,
                              q_target, tau_target);
    std::ofstream fout(debug_path, std::ios::app);
    fout << "[numeric] FBF q = " << q_target.transpose() << std::endl;
    fout << "[numeric] FBF qdot = " << qdot_target.transpose() << std::endl;
    fout << "[numeric] FBF qddot = " << qddot_target.transpose() << std::endl;
}

/**
 * \brief					Create the reference character
 * in ref trajectory
 */
void btGenContactAwareAdviser::CreateRefChar()
{
    if (mDrawReferenceTrajCharacter)
    {
        mRefTrajModel = new cRobotModelDynamics();

        mRefTrajModel->Init(mModel->GetCharFile().c_str(), mModel->GetScale(),
                            ModelType::JSON);
        mRefTrajModel->InitSimVars(mWorld->GetInternalWorld(), true, true,
                                   false);
    }
    if (mDrawTargetFBFCharacter)
    {
        mFBFTrajModel = new cRobotModelDynamics();
        mFBFTrajModel->Init(mModel->GetCharFile().c_str(), mModel->GetScale(),
                            ModelType::JSON);
        mFBFTrajModel->InitSimVars(mWorld->GetInternalWorld(), true, true,
                                   false);
    }
    // mRefModel->SetqAndqdot(mRefTraj->mq[mInternalFrameId],
    // mRefTraj->mqdot[mInternalFrameId]);
}

/**
 * \brief					update the reference character
 */
void btGenContactAwareAdviser::UpdateRefChar()
{
    if (mDrawReferenceTrajCharacter)
    {
        mRefTrajModel->SetqAndqdot(mRefTraj->mq[mRefFrameId],
                                   mRefTraj->mqdot[mRefFrameId]);
    }
    if (mDrawTargetFBFCharacter)
    {
        tVectorXd qdot_next = mTargetAccel * mCurdt + mModel->Getqdot();
        tVectorXd q_next = qdot_next * mCurdt + mModel->Getq();
        mFBFTrajModel->SetqAndqdot(q_next, qdot_next);
    }
}

/**
 * \brief					record the current contact info
 * and motion status into the mSaveTraj
 */
void btGenContactAwareAdviser::RecordTraj()
{
    // record motion
    mOutputTraj->mq[mSimFrameId] = mModel->Getq();
    // std::cout << "record traj " << mInternalFrameId << " " <<
    // mSavedTraj->mq[mInternalFrameId].transpose() << std::endl; record
    // contacts

    {
        // const std::vector<btGenContactForce*>& cur_contacts =;
        auto &rec_contacts = mOutputTraj->mContactForce[mSimFrameId - 1];
        for (auto &x : rec_contacts)
            delete x;
        rec_contacts.clear();
        int num_of_contact_forces = mWorld->GetContactForces().size();
        // std::cout << "[adviser] contact num = " << num_of_contact_forces
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
    // exit(0);
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

    mSimFrameId = mStartFrame;
    mRefFrameId = mStartFrame;
    mHasRefTraj = false;
    mRefTrajPath.clear();
    mOutputTrajPath.clear();
    mFBFOptimizer->Reset();
}

btGenFrameByFrameOptimizer *btGenContactAwareAdviser::GetFBFOptimizer()
{
    return this->mFBFOptimizer;
}

btTraj *btGenContactAwareAdviser::GetRefTraj() { return this->mRefTraj; }

/**
 * \brief               Save current state to the path speicifed by "SaveDir"
 */
void btGenContactAwareAdviser::SaveCurrentState()
{
    /**
     * 1. trajectory path
     * 2. model path
     * 3. model q, qdot
     * 4. current frame id
     */
    std::string filename = mStateSaveDir + "/" + std::to_string(mSimFrameId);
    Json::Value root;
    root["ref_traj_path"] = mRefTrajPath;
    root["model_path"] = mModel->GetCharFile();
    root["q"] = btJsonUtil::BuildVectorJsonValue(mModel->Getq());
    root["qdot"] = btJsonUtil::BuildVectorJsonValue(mModel->Getqdot());
    root["sim_frame_id"] = mSimFrameId;
    root["ref_frame_id"] = mRefFrameId;
    std::cout << "write adviser state to " << filename << std::endl;
    btJsonUtil::WriteJson(filename, root, true);
}

void btGenContactAwareAdviser::LoadInitState()
{
    std::cout << "[load] begin to load init state file " << mInitStateFile
              << std::endl;
    Json::Value root;
    if (false == btJsonUtil::LoadJson(mInitStateFile, root))
    {
        std::cout << "[error] load " << mInitStateFile << " failed\n";
        exit(0);
    }
    std::string ref_traj_path =
        btJsonUtil::ParseAsString("ref_traj_path", root);
    std::string model_path = btJsonUtil::ParseAsString("model_path", root);
    int sim_frame_id = btJsonUtil::ParseAsInt("sim_frame_id", root);
    int ref_frame_id = btJsonUtil::ParseAsInt("ref_frame_id", root);
    tVectorXd q, qdot;
    btJsonUtil::ReadVectorJson(btJsonUtil::ParseAsValue("q", root), q);
    btJsonUtil::ReadVectorJson(btJsonUtil::ParseAsValue("qdot", root), qdot);

    if (ref_traj_path != mRefTrajPath)
    {
        std::cout << "[error] the loaded ref traj " << ref_traj_path
                  << " != setting traj " << mRefTrajPath << std::endl;
        exit(1);
    }

    if (model_path != mModel->GetCharFile())
    {
        std::cout << "[error] the loaded model " << model_path
                  << " != setting model " << mModel->GetCharFile() << std::endl;
        exit(1);
    }

    // set q, qdot and start frame id
    mStartFrame = sim_frame_id;
    mSimFrameId = mStartFrame;
    mRefFrameId = ref_frame_id;
    std::cout << "model dof = " << mModel->GetNumOfFreedom() << std::endl;
    std::cout << "q size = " << q.size() << std::endl;
    std::cout << "qdot size = " << qdot.size() << std::endl;
    std::cout << "set init frame = " << mStartFrame << std::endl;

    mModel->SetqAndqdot(q, qdot);
}

/**
 * \brief                   Update the frame id of reference trajectory
 */
void btGenContactAwareAdviser::UpdateReferenceTraj()
{
    // std::cout << "[debug] update ref traj begin\n";
    if (mEnableRefTrajDelayedUpdate == true)
    {
        // std::cout << "[debug] delayed update ref traj\n";
        // get ref contact num
        // get current contact num
        int ref_contact_num = mRefTraj->mContactForce[mRefFrameId].size();
        int cur_contact_num = mFBFOptimizer->GetCalculatedNumOfContact();
        std::cout << "[adviser] ref contact num " << ref_contact_num
                  << " cur contact num " << cur_contact_num << std::endl;
        if (cur_contact_num >= ref_contact_num)
            mRefFrameId++;
        else
        {
            if (mSimFrameId - mRefFrameId >= 5)
            {
                mRefFrameId++;
            }
        }
    }
    else
    {
        std::cout << "[debug] no delayed update ref traj\n";
        mRefFrameId++;
    }
}