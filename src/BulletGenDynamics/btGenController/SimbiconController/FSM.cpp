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
        std::string stance_update_mode =
            btJsonUtil::ParseAsString("stance_update_mode", cur_state);
        BTGEN_ASSERT(state_id == i);

        const Json::Value &conditions =
            btJsonUtil::ParseAsValue("conditions", cur_state);

        BTGEN_ASSERT(conditions.isArray() == true);
        tState *state = new tState(state_id, stance_update_mode);
        for (auto &cond : conditions)
        {
            state->AddTransitionCondition(
                BuildTransitionCondition(state_id, world, model, cond));
        }
        mStateGraph.push_back(state);
        state->Print();
    }

    mCurState = nullptr;
}

btGenFSM::~btGenFSM()
{
    mCurState = nullptr;
    for (auto &x : mStateGraph)
        delete x;
    mStateGraph.clear();
}

void btGenFSM::SetState(int state_id)
{
    BTGEN_ASSERT(state_id < mStateGraph.size());
    mCurState = mStateGraph[state_id];
}
/**
 * \brief               Update the target pose by FSM
 * \param dt            timestep
 * \param target_pose   ref to character's target pose
 * \param stance        ref to the stance situation
*/
void btGenFSM::Update(double dt, tVectorXd &target_pose, int &old_stance,
                      const int old_swing_foot_id, const int old_stance_foot_id)
{
    mCurState->Update(dt);
    int target_state_id =
        mCurState->GetTargetId(old_swing_foot_id, old_stance_foot_id);
    if (target_state_id == -1)
    {
        printf("[FSM] keep in the same state = %d\n", mCurState->GetStateId());
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
*/
void btGenFSM::Init() {}

/**
 * \brief               Get the current state in FSM
*/
tState *btGenFSM::GetCurrentState() { return this->mCurState; }