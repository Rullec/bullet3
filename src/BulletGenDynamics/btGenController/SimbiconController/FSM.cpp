#include "FSM.h"
#include "BulletGenDynamics/btGenController/Trajectory/btTraj.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include "FSMUtil.h"
btGenFSM::btGenFSM(btGeneralizeWorld *world, cRobotModelDynamics *model,
                   const Json::Value &config)
    : mWorld(world), mModel(model)
{
    mStateGraph.clear();
    mCurState = nullptr;

    std::string state_traj_path =
        btJsonUtil::ParseAsString("state_traj_path", config);
    mStateTraj = new btTraj();
    mStateTraj->LoadTraj(state_traj_path, mModel);

    int num_of_states = btJsonUtil::ParseAsInt("num_of_states", config);
    BTGEN_ASSERT(num_of_states == mStateTraj->mNumOfFrames);

    const Json::Value &states = btJsonUtil::ParseAsValue("states", config);

    BTGEN_ASSERT(states.size() == num_of_states);

    // parse each transition conditions
    for (int i = 0; i < states.size(); i++)
    {
        const Json::Value &cur_state = states[i];
        int state_id = btJsonUtil::ParseAsInt("state_id", cur_state);
        std::string default_stance_str =
            btJsonUtil::ParseAsString("default_stance", cur_state);
        std::string stance_update_mode =
            btJsonUtil::ParseAsString("stance_update_mode", cur_state);

        int default_stance = -1;
        if (default_stance_str == "left")
            default_stance = BTGEN_LEFT_STANCE;
        else if (default_stance_str == "right")
            default_stance = BTGEN_RIGHT_STANCE;
        else
        {
            BTGEN_ASSERT(false);
        }
        BTGEN_ASSERT(state_id == i);

        const Json::Value &conditions =
            btJsonUtil::ParseAsValue("conditions", cur_state);

        BTGEN_ASSERT(conditions.isArray() == true);
        tState *state =
            new tState(state_id, default_stance, stance_update_mode);
        for (auto &cond : conditions)
        {
            state->AddTransitionCondition(
                BuildTransitionCondition(state_id, world, model, cond));
        }
        mStateGraph.push_back(state);
        state->Print();
    }

    mCurState = mStateGraph[0];
}

btGenFSM::~btGenFSM()
{
    mCurState = nullptr;
    for (auto &x : mStateGraph)
        delete x;
    mStateGraph.clear();
}

/**
 * \brief               Update the target pose by FSM
 * \param dt            timestep
 * \param target_pose   ref to character's target pose
 * \param stance        ref to the stance situation
*/
void btGenFSM::Update(double dt, tVectorXd &target_pose, int &old_stance)
{
    mCurState->Update(dt);
    int target_state_id = mCurState->GetTargetId();
    if (target_state_id == -1)
    {
        printf("[FSM] target state id = -1, keep in the same state\n");
    }
    else
    {
        mCurState = mStateGraph[target_state_id];
        mCurState->Reset();
        int new_stance = mCurState->CalcNewStance(old_stance);
        printf("[FSM] transfer to target state id = %d, update stance from %d "
               "to %d",
               target_state_id, old_stance, new_stance);
        old_stance = new_stance;
        // when the trainsition of states occured, we needs to update the stance situation
    }

    target_pose = GetTargetPose();
    // std::cout << "[FSM] target pose = " << target_pose.transpose() << std::endl;
}

tVectorXd btGenFSM::GetTargetPose()
{
    return mStateTraj->mq[mCurState->GetStateId()];
}
/**
 * \brief               init the FSM
 * 1. set the default pose of character
 * 2. set the default state of FSM
 * 3. fetch the current stance 
*/
void btGenFSM::Init(int &stance)
{
    // tVectorXd q = mStateTraj->mq[mCurState->GetStateId()],
    //           qdot = tVectorXd::Zero(q.size());
    tVectorXd q = tVectorXd::Zero(mModel->GetNumOfFreedom()),
              qdot = tVectorXd::Zero(mModel->GetNumOfFreedom());

    switch (mModel->GetRoot()->GetJointType())
    {
    case JointType::BIPEDAL_NONE_JOINT:
        q[0] = 0.75;
        break;
    case JointType::NONE_JOINT:
        q[1] = 0.75;
        break;
    default:
        BTGEN_ASSERT(false);
        break;
    }
    stance = BTGEN_LEFT_STANCE;
    std::cout << "[FSM] init q = " << q.transpose() << " , stance = " << stance
              << std::endl;
    mModel->SetqAndqdot(q, qdot);
}

/**
 * \brief               Get the current state in FSM
*/
tState *btGenFSM::GetCurrentState() { return this->mCurState; }