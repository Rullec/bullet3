#include "btGenContactAwareController.h"
#include "../examples/CommonInterfaces/CommonGUIHelperInterface.h"
#include "BulletGenDynamics/btGenController/ContactAdaptionFeature/btGenFeature.h"
#include "BulletGenDynamics/btGenController/FBFCalculator/btGenFrameByFrameCalculator.h"
#include "BulletGenDynamics/btGenController/NQRCalculator/btGenNQRCalculator.h"
#include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenController/btTrajContactMigrator.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenSolver/ContactSolver.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include <fstream>
#include <iostream>
tVectorXd ConvertPoseToq(const tVectorXd &pose, cRobotModelDynamics *model);

// btGenContactAwareController::btGenContactAwareController(btGeneralizeWorld* world,
// const std::string& path, double W, double Wm) : mW(W), mWm(Wm) std::string
// new_path = "new.out";
std::string debug_path = "numeric.log";
btGenContactAwareController::btGenContactAwareController(
    btGeneralizeWorld *world)
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
    mTargetCalculator = nullptr;
    mOutputTraj = nullptr;
    mRefTraj = nullptr;
    mRefTrajPath = "";
    mOutputTrajPath = "";
    mEnableOutput = false;

    mRefTrajModel = nullptr;
    mFBFTrajModel = nullptr;
    mOutputControlDiff = false;
    mBulletGUIHelper = nullptr;

    // std::ofstream fout(debug_path);
    // fout << "";
    // fout.close();
}

btGenContactAwareController::~btGenContactAwareController()
{
    delete mRefTrajModel;
    delete mFBFTrajModel;
    delete mFeatureVector;
    delete mRefTraj;
}

// int btGenContactAwareController::GetInternalFrameId() const
// {
//     return this->mInternalFrameId;
// }

/**
 * \brief           Set the wait-to-solve trajectory
 * \param ref_traj          set the reference trajectory
 * \param output_traj       set the output trajectory path
 * \param enable_output     output or not, default not (false)
 */
void btGenContactAwareController::SetTraj(const std::string &ref_traj,
                                          const std::string &output_traj,
                                          bool enable_output /*= false*/)
{
    mEnableOutput = enable_output;
    mRefTrajPath = ref_traj;
    mOutputTrajPath = output_traj;
    LoadTraj(mRefTrajPath);

    if (mModel->GetNumOfFreedom() != mRefTraj->mq[mRefFrameId].size())
    {
        std::cout << "btGenContactAwareController::SetTraj the model is "
                     "inconsistent with the given motion\n";
        exit(0);
    }
    if (mRefFrameId >= mRefTraj->mNumOfFrames - 1)
    {
        std::cout << "btGenContactAwareController::SetTraj the traj has "
                  << mRefTraj->mNumOfFrames << " frames but the start frame is "
                  << mRefFrameId << ", doesn't match\n";
        exit(0);
    }

    // set the start point
    {
        mModel->SetqAndqdot(mRefTraj->mq[mRefFrameId],
                            mRefTraj->mqdot[mRefFrameId]);
        if (mRefTrajModel)
            mRefTrajModel->SetqAndqdot(mRefTraj->mq[mRefFrameId],
                                       mRefTraj->mqdot[mRefFrameId]);
        if (mFBFTrajModel)
            mFBFTrajModel->SetqAndqdot(mRefTraj->mq[mRefFrameId],
                                       mRefTraj->mqdot[mRefFrameId]);
    }

    // std::cout << "[controller] init q = "
    //           << mRefTraj->mq[mRefFrameId].segment(0, 3).transpose()
    //           << std::endl;
    // std::cout << "[controller] init qdot = "
    //           << mRefTraj->mqdot[mRefFrameId].transpose() << std::endl;
    mTargetCalculator->SetTraj(mRefTraj);
    mHasRefTraj = true;

    if (mEnableInitStateLoad == true)
    {
        LoadInitState();
    }
    // std::cout << "[controller] final init q = "
    //           << mRefTraj->mq[mRefFrameId].segment(0, 3).transpose()
    //           << std::endl;
    // std::cout << "[controller] final init qdot = "
    //           << mRefTraj->mqdot[mRefFrameId].transpose() << std::endl;
}
void btGenContactAwareController::Init(cRobotModelDynamics *model_,
                                       const std::string &contact_aware_config)
{
    ReadConfig(contact_aware_config);
    mModel = model_;

    num_of_freedom = mModel->GetNumOfFreedom();
    num_of_underactuated_freedom = num_of_freedom - 6;

    mTargetCalculator = CreateTargetCalculator(mTargetCalculatorConfigFile);

    mFeatureVector->Init(mFeatureVectorFile, mModel, mWorld->GetGravity());

    CreateRefChar();
    PostProcess();
}

void btGenContactAwareController::PreUpdate(double dt)
{
    mCurdt = dt;

    // 1. save the current state
    if (mEnableStateSave)
        SaveCurrentState();

    // 2. draw the contact points
    if (mEnableDrawContactPointsInBulletGUIController == true ||
        mEnableDrawRefTrajContactPoints == true)
    {
        ClearDrawPoints();

        if (mEnableDrawContactPointsInBulletGUIController)
            DrawContactPoints();
        if (mEnableDrawRefTrajContactPoints)
            DrawRefTrajContactPoints();
        // exit(0);
    }

    // 3. judge the preliminary
    if (mRefTraj == nullptr)
        std::cout << "[error] controller guide traj hasn't been set\n", exit(0);

    // 4. protect the timestep
    if (std::fabs(mRefTraj->mTimestep - dt) > 1e-8)
    {
        std::cout << "[error] btGenContactAwareController ref motion dt "
                  << mRefTraj->mTimestep << " != sim dt " << dt << std::endl;
        exit(0);
    }

    // 5. judge the end
    if (IsEnd() == true)
    {
        std::cout << "[error] The contact-aware controller has terminated, it "
                     "shouldn't be updated anymore\n";
        exit(0);
    }

    // 6. record the current state into the save trajectory
    RecordTraj();
}

/**
 * \brief               update the controller
 *  calculate and offer new action guidence for the character
 */
void btGenContactAwareController::Update(double dt)
{
    std::cout << "---------------------frame " << mSimFrameId << " ref "
              << mRefFrameId << std::endl;

    // 1. pre update, check the preliminary
    PreUpdate(dt);

    // 2. calculate the control target from the FBF optimzier
    FetchControlTarget(dt, mTargetAccel, mTargetVel, mTargetPos, mTargetTau);
    if (mEnableOnlyTargetController == true)
        //      If we only use Frame by frame to do control (which means the LCP is disable), call the ControlByFBF method
        mTargetCalculator->ControlByAdaptionController();
    else
    {

        // 3. Calculate the convert matrix and residual used in the contact-aware jointed LCP

        mFeatureVector->Eval(dt, mTargetAccel, mTargetVel, mTargetPos,
                             mTargetTau, mH, mE, mf);
    }

    // 4. update
    PostUpdate();
}

void btGenContactAwareController::PostUpdate()
{
    UpdateDrawingChar();
    UpdateReferenceTraj();
    mSimFrameId++;
}
/**
 * \brief               load the guidence trajectory
 */

void btGenContactAwareController::LoadTraj(const std::string &path)
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
        std::cout << "controller LoadTraj but mModel is empty\n";
        exit(0);
    }
    mRefTraj->LoadTraj(path, mModel,
                       btGeneralizeWorld::GetIntegrationSchemeStr(
                           mWorld->GetIntegrationScheme()),
                       mMaxFrame);
    // resolve the active force
    if (mResolveControlToruqe)
        ResolveActiveForce();

    if (mEnableMigrateContactInfo)
        btTrajContactMigrator::MigrateTrajContactByGivenInfo(
            mModel, mRefTraj, mMigratecontactInfoPath);
    // load the saved traj in order to shape that
    mOutputTraj->LoadTraj(path, mModel,
                          btGeneralizeWorld::GetIntegrationSchemeStr(
                              mWorld->GetIntegrationScheme()),
                          mMaxFrame);
    mOutputTraj->Reshape(std::min(mMaxFrame, mRefTraj->mNumOfFrames));
}

/**
 * \brief					Resolve the active force by the
 * qddot and contact forces
 */
void btGenContactAwareController::ResolveActiveForce()
{
    std::cout << "[error] ResolveActiveForce should not be used anymore, it "
                 "will destroy the actuated force in mocap data\n";

    exit(0);
    /*
                    Mqddot + Cqdot = QG + Qcontrol + Qcontact
                    Qcontrol = Mqddot + Cqdot - QG - Qcontact
            */
    mModel->PushState("resolve");
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

            Qcontact =
                (Qcontact + jac.transpose() * fc->mForce.segment(0, 3)).eval();
        }
        mRefTraj->mTruthJointForceVec[frame_id] = LHS - G - Qcontact;
    }
    mModel->PopState("resolve");
}

/**
 * \brief					Init the pose of guided
 * character by the input trajectory
 */
void btGenContactAwareController::PostProcess()
{
    if (mStartFrame < 1)
    {
        std::cout << "[error] post process: illegal start frame " << mStartFrame
                  << std::endl;
        exit(0);
    }
    mSimFrameId = mStartFrame;
    mRefFrameId = mStartFrame;
    mModel->SetContactAwareController(this);
    mModel->SetEnableContactAwareController(true);

    // create the N matrix
    mN = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
    mN.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom)
        .setIdentity();
}

/**
 * \brief					input the contact forces
 * (generalized), calculate and apply the active control torque
 */
tVectorXd
btGenContactAwareController::CalcControlForce(const tVectorXd &Q_contact,
                                              bool verbose /* = true*/)
{
    if (mEnableOnlyTargetController == true)
    {
        std::cout << "[error] This function CalcControlForce should not be "
                     "called in only FBF model, exit\n";
        exit(0);
    }
    // std::cout << "[controller] contact force = " << Q_contact.norm() << std::endl;
    // std::cout << "mH = \n" << mH << std::endl;
    tVectorXd Q_active = mH * Q_contact + mE.inverse() * mf;
    // std::cout << "[control] contact force norm = " << Q_contact.norm() <<
    // std::endl; std::cout << "[control] mH norm = " << mH.norm() << std::endl;
    // std::cout << "[control] mE norm = " << mE.norm() << std::endl;
    // std::cout << "[control] mf norm = " << mf.norm() << std::endl;
    int ref_size = mRefTraj->mTruthJointForceVec[mRefFrameId - 1].size();
    if (ref_size != num_of_underactuated_freedom)
    {
        std::cout << "[error] the ref truth joint force vec size " << ref_size
                  << " != " << num_of_underactuated_freedom << std::endl;
        exit(1);
    }
    // std::cout << "[control] mf = " << mf.transpose() << std::endl;
    tVectorXd ref_force = mRefTraj->mTruthJointForceVec[mRefFrameId - 1];
    // std::cout << "q = " << mModel->Getq().segment(0, 3).transpose() <<
    // std::endl; std::cout << "[controller] ref ctrl force = " <<
    // ref_force.transpose() << std::endl; std::cout << "[controller] calced ctrl
    // force = " << Q_active.transpose() << std::endl;

    mOutputTraj->mTruthJointForceVec[mRefFrameId - 1] =
        tVectorXd::Zero(num_of_freedom);
    mOutputTraj->mTruthJointForceVec[mRefFrameId - 1].segment(
        6, num_of_underactuated_freedom) = Q_active;
    // if (Q_active.norm() > 1e8)
    // {
    // 	std::cout << "[warn] force explode, use the reference torque = " <<
    // ref_force.transpose() << std::endl; 	Q_active = ref_force;
    // }
    mCtrlForce = Q_active;
    // std::cout << "[numeric] contact force = " << Q_contact.transpose()
    //           << std::endl;
    // std::cout << "[numeric] control force = " << mCtrlForce.transpose()
    //           << std::endl;
    if (mCtrlForce.cwiseAbs().maxCoeff() > 1e6)
    {
        std::cout << "[error] control force abs max > 1e6\n";
        exit(0);
    }
    // std::ofstream fout(debug_path, std::ios::app);
    // std::cout << "[controller] contact force = " << Q_contact.transpose()
    //           << std::endl;
    // fout << "[numeric] contact force = " << Q_contact.transpose() << std::endl;

    // fout << "[numeric] control force = " << mCtrlForce.transpose() << std::endl;

    mFeatureVector->CalcEnergy(mCtrlForce, Q_contact);

    if (verbose == true)
    {
        // mFeatureVector->DebugFullMinimiumIsTheSameAsTheRefTraj(mCtrlForce, Q_contact);
        tVectorXd mocap_contact_force =
            mRefTraj->GetGenContactForce(mRefFrameId - 1, mModel);
        tVectorXd mocap_control_force =
            mRefTraj->GetGenControlForce(mRefFrameId - 1, mModel);
        // std::cout << "[mocap] contact force = " << mocap_contact_force.transpose()
        //           << std::endl;
        // std::cout << "[mocap] control force = " << mocap_control_force.transpose()
        //           << std::endl;

        tVectorXd contact_force_diff = mocap_contact_force - Q_contact;
        tVectorXd control_force_diff = mocap_control_force - Q_active;
        std::cout << "[log] contact_force diff_percent = "
                  << contact_force_diff.norm() /
                         (mocap_contact_force.norm() + 1e-8) * 100
                  << std::endl;
        std::cout << "[log] control_force diff_percent = "
                  << control_force_diff.norm() /
                         (mocap_control_force.norm() + 1e-8) * 100
                  << std::endl;
    }
    // check the contact force & control force that is calculated by the full minimium of optimization problem
    // it should be the same as the contact force in the ref traj
    // but, it will be different from the new LCP solved result
    std::cout << "[warn] begin to verify whether the dFdtau is zero\n";
    // mFeatureVector->DebugPosFeatureDFdtauIsZero(mCtrlForce, Q_contact);
    // mFeatureVector->DebugVelFeatureDFdtauIsZero(mCtrlForce, Q_contact);
    mFeatureVector->DebugAccelFeatureDFdtauIsZero(mCtrlForce, Q_contact);
    // mFeatureVector->DebugTauFeatureDFdtauIsZero(mCtrlForce, Q_contact);
    // when we get the control force, we can evaluate the energy term in btGenFeatureArray then evaulate the control result immediately
    std::cout
        << "[warn] begin to verify whether the solution is a local minimium\n";
    mFeatureVector->VerifyTheSolutionIsLocalMin(Q_active, Q_contact);

    return Q_active;
}

tVectorXd btGenContactAwareController::GetPrevControlForce()
{
    return mCtrlForce;
}
/**
 * \brief					Load config
 */
void btGenContactAwareController::ReadConfig(const std::string &config)
{
    std::cout << "[debug] read config of controller from " << config
              << std::endl;
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
    mTargetCalculatorConfigFile =
        btJsonUtil::ParseAsString("target_calculator_config", root);
    mEnableStateSave = btJsonUtil::ParseAsBool("enable_state_save", root);
    mEnableInitStateLoad =
        btJsonUtil::ParseAsBool("enable_init_state_load", root);
    mInitStateFile = btJsonUtil::ParseAsString("init_state_file", root);
    mStateSaveDir = btJsonUtil::ParseAsString("save_dir", root);
    mEnableOnlyTargetController =
        btJsonUtil::ParseAsBool("enable_only_target_calculator", root);
    mEnableRefTrajDelayedUpdate =
        btJsonUtil::ParseAsBool("enable_ref_traj_delayed_update", root);
    mEnableDrawContactPointsInBulletGUIController = btJsonUtil::ParseAsBool(
        "enable_draw_contact_points_in_bullet_GUI_controller", root);
    mEnableDrawRefTrajContactPoints =
        btJsonUtil::ParseAsBool("enable_draw_ref_traj_contact_point", root);

    mEnableMigrateContactInfo =
        btJsonUtil::ParseAsBool("enable_contact_migrator", root);
    mMigratecontactInfoPath =
        btJsonUtil::ParseAsString("contact_migrator_info", root);

    if (mEnableOnlyTargetController == true)
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
}

// void btGenContactAwareController::UpdateMultibodyVelocityDebug(double dt)
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
 * \brief           Update the multibody info in INVERSE semi-implicit scheme (which means only update velocity and do syncronization here)
*/
void btGenContactAwareController::UpdateMultibodyInverseSemiImplicit(double dt)
{
    mModel->UpdateVelocity(dt);
    CheckTheDiff(false);
    CheckAndSyncCharByRefTraj();
}

/**
 * \brief					Update the multibody in semi implicit scheme
 */
void btGenContactAwareController::UpdateMultibodySemiImplicit(double dt)
{
    mModel->UpdateVelocityAndTransform(dt);
    CheckTheDiff(false);
    CheckAndSyncCharByRefTraj();
}

/**
 * 
 */
tVectorXd btGenContactAwareController::CalcLCPResidual(double dt) const
{
    if (mEnableOnlyTargetController == true)
    {
        std::cout << "[error] This function CalcLCPResidual should not be "
                     "called in only FBF model, exit\n";
        exit(0);
    }
    // std::cout << "Controller Residual hasn't been implemented\n", exit(0);
    const tMatrixXd &Minv = mModel->GetInvMassMatrix();
    // mModel
    const tVectorXd &QG = mModel->CalcGenGravity(mWorld->GetGravity());
    const tVectorXd &qdot = mModel->Getqdot();
    tVectorXd residual =
        dt * Minv *
            (QG + mN * mE.inverse() * mf - mModel->GetCoriolisMatrix() * qdot) +
        qdot;
    tVectorXd Gen_force = mModel->GetGeneralizedForce();
    tVectorXd diff = QG - Gen_force;
    if (diff.norm() > 1e-8)
    {
        std::cout << "[error] btGenContactAwareController::CalcLCPResidual: "
                     "GenForce should be the same as QG, but now the diff "
                     "is too big "
                  << diff.transpose() << std::endl;
        std::cout << "[error] cur total gen force = " << Gen_force.transpose()
                  << std::endl;
        std::cout << "[error] cur QG = " << QG.transpose() << std::endl;
        std::cout << "[error] cur pure = "
                  << mModel->DebugGetGeneralizedForce().transpose()
                  << std::endl;
        exit(0);
    }
    // std::cout << "[debug] [new] frame " << mInternalFrameId << " residual = "
    // << residual.transpose() << std::endl;
    return residual;
}
tMatrixXd btGenContactAwareController::CalcLCPPartBPrefix() const
{
    if (mEnableOnlyTargetController == true)
    {
        std::cout << "[error] This function CalcLCPPartBPrefix should not be "
                     "called in only FBF model, exit\n";
        exit(0);
    }
    // std::cout << "Controller PartB hasn't been implemented\n", exit(0);
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

bool btGenContactAwareController::IsEnd()
{
    return mRefFrameId >= (mRefTraj->mq.size() - 1);
}

/**
 * \brief					Get current reference gen accel
 * "qddot_target" and "tau_target"
 */
void btGenContactAwareController::FetchControlTarget(double dt,
                                                     tVectorXd &qddot_target,
                                                     tVectorXd &qdot_target,
                                                     tVectorXd &q_target,
                                                     tVectorXd &tau_target)
{
    mTargetCalculator->CalcTarget(dt, mRefFrameId, qddot_target, qdot_target,
                                  q_target, tau_target);
    // std::ofstream fout(debug_path, std::ios::app);
    // fout << "[numeric] FBF q = " << q_target.transpose() << std::endl;
    // fout << "[numeric] FBF qdot = " << qdot_target.transpose() << std::endl;
    // fout << "[numeric] FBF qddot = " << qddot_target.transpose() << std::endl;
}

/**
 * \brief					Create the reference character
 * in ref trajectory
 */
void btGenContactAwareController::CreateRefChar()
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
void btGenContactAwareController::UpdateDrawingChar()
{
    if (mDrawReferenceTrajCharacter)
    {
        mRefTrajModel->SetqAndqdot(mRefTraj->mq[mRefFrameId],
                                   mRefTraj->mqdot[mRefFrameId]);
    }
    if (mDrawTargetFBFCharacter)
    {
        mFBFTrajModel->SetqAndqdot(mTargetPos, mTargetVel);
    }
}

/**
 * \brief					record the current contact info
 * and motion status into the mSaveTraj
 */
void btGenContactAwareController::RecordTraj()
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
        // std::cout << "[controller] contact num = " << num_of_contact_forces
        //           << std::endl;
        for (int id = 0; id < num_of_contact_forces; id++)
        {
            auto cur_contact = mWorld->GetContactForces()[id];
            if (cur_contact->mObj->GetType() == eColObjType::RobotCollder)
            {
                /*
                btGenRobotCollider *collider, const tVector &f,
                        const tVector &world_pos, const tVector &local_pos,
                        bool is_self_collision
                */
                auto link_collider =
                    dynamic_cast<btGenRobotCollider *>(cur_contact->mObj);
                auto link = mModel->GetLinkById(link_collider->mLinkId);
                tVector local_pos = link->GetGlobalTransform().inverse() *
                                    cur_contact->mWorldPos;
                rec_contacts.push_back(new btGenMBContactForce(
                    link_collider, cur_contact->mForce, cur_contact->mWorldPos,
                    local_pos, cur_contact->mIsSelfCollision));
            }
        }
    }
    // exit(0);
}

/**
 * \brief               Reset the contact aware controller, which means the current
 * trajectory should be deleted and cannot be used to do tracking
 */
void btGenContactAwareController::Reset()
{
    std::cout << "[controller] controller reset!\n";
    // exit(0);
    if (mEnableOutput == true)
    {
        std::cout << "[controller] Save traj v2 to " << mOutputTrajPath
                  << std::endl;
        mOutputTraj->mIntegrationScheme =
            btGeneralizeWorld::GetIntegrationSchemeStr(
                mWorld->GetIntegrationScheme());
        mOutputTraj->SaveTraj(mOutputTrajPath, mModel);
    }
    else
    {
        std::cout << "[controller] Save traj is diabled\n";
    }

    mSimFrameId = mStartFrame;
    mRefFrameId = mStartFrame;
    mHasRefTraj = false;
    mRefTrajPath.clear();
    mOutputTrajPath.clear();
    mTargetCalculator->Reset();
}

btGenTargetCalculator *btGenContactAwareController::GetTargetCalculator()
{
    return this->mTargetCalculator;
}

void btGenContactAwareController::SetBulletGUIHelperInterface(
    struct GUIHelperInterface *inter)
{
    mBulletGUIHelper = inter;
    mTargetCalculator->SetBulletGUIHelperInterface(inter);
}
const btTraj *btGenContactAwareController::GetRefTraj() const
{
    return this->mRefTraj;
}

/**
 * \brief               Save current state to the path speicifed by "SaveDir"
 */
void btGenContactAwareController::SaveCurrentState()
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
    std::cout << "write controller state to " << filename << std::endl;
    btJsonUtil::WriteJson(filename, root, true);
}

void btGenContactAwareController::LoadInitState()
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
void btGenContactAwareController::UpdateReferenceTraj()
{
    // std::cout << "[debug] update ref traj begin\n";
    if (mEnableRefTrajDelayedUpdate == true)
    {
        // std::cout << "[debug] delayed update ref traj\n";
        // get ref contact num
        // get current contact num
        int ref_contact_num = mRefTraj->mContactForce[mRefFrameId].size();
        int cur_contact_num = mTargetCalculator->GetCalculatedNumOfContact();
        std::cout << "[controller] ref contact num " << ref_contact_num
                  << " cur contact num " << cur_contact_num << std::endl;
        if (cur_contact_num >= ref_contact_num)
            mRefFrameId++;
        else
        {
            if (mSimFrameId - mRefFrameId >= 5)
            {
                mRefFrameId = mSimFrameId;
                mRefFrameId++;
            }
        }
    }
    else
    {
        // std::cout << "[debug] no delayed update ref traj\n";
        mRefFrameId++;
    }
}

// void btGeneralizeWorld::AddObj(int n, const std::string &obj_type,
//                                bool add_perturb /*=false*/)
// {

//     if (obj_type == "ball")
//     {

//         mCollisionShapeArray.push_back(compunde);
//     }
//     else if (obj_type == "cube")
//     {
//         colShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
//         mCollisionShapeArray.push_back(colShape);
//     }
//     else if (obj_type == "stick")
//     {
//         colShape = new btBoxShape(btVector3(0.001, 0.1, 2));
//         mCollisionShapeArray.push_back(colShape);
//     }
//     else
//     {
//         std::cout << "unsupported type " << obj_type << std::endl;
//         exit(0);
//     }

//     /// Create Dynamic Objects
//     btTransform startTransform;
//     startTransform.setIdentity();
//     startTransform.setOrigin(btVector3(0, -1, 0));
//     // startTransform.setOrigin(btVector3(0.680375, -1.18218, 0.566198));
//     // tQuaternion qua = tQuaternion(0.800701, 0.372043, 0.28516, -0.373023);
//     // startTransform.setRotation(btBulletUtil::tQuaternionTobtQuaternion(qua));

//     // startTransform.setRotation(btQuaternion(btVector3(1, 1, 0), SIMD_PI
//     // / 6.));

//     // for (int i = 0; i < n; i++)
//     // {
//     // 	startTransform.setOrigin(startTransform.getOrigin() + btVector3(0, 1.5,
//     // 0)); 	createRigidBody(1.f, startTransform, colShape, "ball" +
//     // std::to_string(i));
//     // }

//     for (int k = 0; k < n; k++)
//     {
//         for (int i = 0; i < 1; i++)
//         {
//             for (int j = 0; j < 1; j++)
//             {
//                 startTransform.setOrigin(btVector3(
//                     btScalar(1 * i), btScalar(-1 + 1.1 * k), btScalar(1 * j)));
//                 if (add_perturb)
//                     startTransform.setOrigin(startTransform.getOrigin() +
//                                              btBulletUtil::tVectorTobtVector(
//                                                  tVector::Random() * 0.1));
//                 createRigidBody(1.f, startTransform, colShape,
//                                 obj_type + std::to_string(k));
//             }
//         }
//     }
//     std::cout << "obj num = " << mSimObjs.size() << std::endl;
//     // mSimObjs[0]-(tVector(0, -0.607199, 0, 0));
//     // mSimObjs[0]->SetAngVel(tVector(2.14574, 0.00479028, -0.277455, 0));
//     // mSimObjs[0]->set(tVector(0, -0.607168, 0, 0));
// }

void btGenContactAwareController::ClearDrawPoints()
{
    if (mBulletGUIHelper == nullptr)
        return;
    auto inter_world = mWorld->GetInternalWorld();
    // std::cout << "clear points num = " << mDrawPointsList.size()
    //           << " now = " << inter_world->getCollisionObjectArray().size()
    //           << std::endl;

    for (auto &pt : this->mDrawPointsList)
    {
        // inter_world->getCollisionObjectArray().remove(pt);
        delete pt->getCollisionShape();
        mWorld->GetInternalWorld()->removeCollisionObject(pt);
        mBulletGUIHelper->removeGraphicsInstance(pt->getUserIndex());
        delete pt;
    }
    // std::cout << "[debug] clear points " << mDrawPointsList.size() << std::endl;
    mDrawPointsList.clear();
}

void btGenContactAwareController::DrawPoint(const tVector3d &pos,
                                            double radius /* = 0.05*/)
{
    if (mBulletGUIHelper == nullptr)
        return;
    btCollisionShape *colShape = nullptr;
    btCollisionObject *obj = new btCollisionObject();
    colShape = new btSphereShape(btScalar(radius));
    btTransform trans;
    // trans.setOrigin(btVector3(pos[0], pos[1], pos[2]));
    trans.setOrigin(btVector3(pos[0], pos[1], pos[2]));
    obj->setWorldTransform(trans);
    obj->setCollisionShape(colShape);
    obj->setCollisionFlags(0);
    mWorld->GetInternalWorld()->addCollisionObject(obj, 0, 0);
    mDrawPointsList.push_back(obj);

    auto inter_world = mWorld->GetInternalWorld();
}

/**
 * \brief               Draw all contact points
*/
void btGenContactAwareController::DrawContactPoints()
{
    auto manifolds = mWorld->GetContactManifolds();
    for (auto *mani : manifolds)
    {
        for (int i = 0; i < mani->getNumContacts(); i++)
        {

            tVector world_point = btBulletUtil::btVectorTotVector0(
                mani->getContactPoint(i).getPositionWorldOnA());
            DrawPoint(world_point.segment(0, 3));
        }
    }
}

/**
 * \brief               Draw contact points in ref traj
*/
void btGenContactAwareController::DrawRefTrajContactPoints()
{
    printf("begin to draw ref traj contact in ref frame %d num %d\n",
           mRefFrameId, mRefTraj->mContactForce[mRefFrameId].size());

    for (auto &x : mRefTraj->mContactForce[mRefFrameId])
    {

        tVectorXd global_pos =
            mModel->GetLinkById(x->mLinkId)->GetGlobalTransform() *
            x->mLocalPos;
        DrawPoint(global_pos.segment(0, 3));
        std::cout << "[debug] draw contact force in ref traj at "
                  << global_pos.transpose() << std::endl;
    }
}

btGenTargetCalculator *btGenContactAwareController::CreateTargetCalculator(
    const std::string conf_path) const
{
    btGenTargetCalculator *calc = nullptr;
    // 1. load the config, recognize the type
    Json::Value root;
    btJsonUtil::LoadJson(conf_path, root);
    std::string type =
        btJsonUtil::ParseAsString("target_calculator_type", root);

    if ("FBF" == type)
    {
        calc = new btGenFBFTargetCalculator();
    }
    else if ("NQR" == type)
    {
        calc = new btGenNQRCalculator();
    }
    else
    {
        std::cout << "unrecognized target calculator type = " << type
                  << std::endl;
        exit(1);
    }
    std::cout << "[log] build " << type << " target calculator\n";

    calc->Init(mWorld, conf_path);
    return calc;
}

std::string btGenContactAwareController::GetSupposedContactInfo()
{
    return this->mMigratecontactInfoPath;
}

/**
 * \brief       check the different between model and ref traj
*/
void btGenContactAwareController::CheckTheDiff(bool output /* = false*/)
{
    tVectorXd qddot = mModel->Getqddot();
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
        // std::ofstream fout(debug_path, std::ios::app);
        // fout << "[numeric] ref q = " << ref_traj_q.transpose() << std::endl;
        // fout << "[numeric] ref qdot = " << ref_traj_qdot.transpose()
        //      << std::endl;
        // fout << "[numeric] ref qddot = " << ref_traj_qddot.transpose()
        //      << std::endl;
        // fout << "[numeric] ctrl_res q = " << q.transpose() << std::endl;
        // fout << "[numeric] ctrl_res qdot = " << qdot.transpose() << std::endl;
        // fout << "[numeric] ctrl_res qddot = " << qddot.transpose() << std::endl;

        {
            tVectorXd q_diff = q - ref_traj_q, qdot_diff = qdot - ref_traj_qdot,
                      qddot_diff = qddot - ref_traj_qddot;

            // std::cout << "[debug] ctrl_res and ref_traj q diff "
            //           << q_diff.norm() << " qdot diff " << qdot_diff.norm()
            //           << " qddot diff = " << qddot_diff.norm() << " "
            //           << "total err" << std::endl;
        }

        {
            tVectorXd q_diff = q - mTargetPos, qdot_diff = qdot - mTargetVel,
                      qddot_diff = qddot - mTargetAccel;

            // std::cout << "[debug] ctrl_res and target_traj q diff "
            //           << q_diff.norm() << " qdot diff " << qdot_diff.norm()
            //           << " qddot diff = " << qddot_diff.norm() << " "
            //           << "ca err" << std::endl;
        }

        {
            tVectorXd q_diff = ref_traj_q - mTargetPos,
                      qdot_diff = ref_traj_qdot - mTargetVel,
                      qddot_diff = ref_traj_qddot - mTargetAccel;

            // std::cout << "[debug] ref_traj and target_traj q diff "
            //           << q_diff.norm() << " qdot diff " << qdot_diff.norm()
            //           << " qddot diff = " << qddot_diff.norm() << " "
            //           << "FBF err" << std::endl;
        }
    }
}

/**
 * \brief           Check the ref frame id and the synchronization frequency
*/
void btGenContactAwareController::CheckAndSyncCharByRefTraj()
{
    if (mEnableSyncTrajPeriodly == true && mRefFrameId % mSyncTrajPeriod == 0)
    {
        std::cout << "[sync] ref frame " << mRefFrameId << ", sync traj period "
                  << mSyncTrajPeriod << std::endl;

        mModel->SetqAndqdot(mRefTraj->mq[mRefFrameId],
                            mRefTraj->mqdot[mRefFrameId]);
    }
}