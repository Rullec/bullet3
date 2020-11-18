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
        BTGEN_ASSERT(state_id == i);

        const Json::Value &conditions =
            btJsonUtil::ParseAsValue("conditions", cur_state);

        BTGEN_ASSERT(conditions.isArray() == true);
        tState *state = new tState(state_id);
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
*/
void btGenFSM::Update(double dt, tVectorXd &target_pose)
{
    mCurState->Update(dt);
    int target_state_id = mCurState->GetTargetId();
    if (target_state_id == -1)
    {
        printf("[FSM] target state id = -1, keep in the same state\n");
    }
    else
    {
        printf("[FSM] target state id = %d, transfer\n", target_state_id);
        mCurState = mStateGraph[target_state_id];
        mCurState->Reset();
    }

    target_pose = GetTargetPose();
    // std::cout << "[FSM] target pose = " << target_pose.transpose() << std::endl;
}

tVectorXd btGenFSM::GetTargetPose()
{
    return mStateTraj->mq[mCurState->GetStateId()];
}
/**
 * \brief               Set the init pose by current state
*/
void btGenFSM::InitPose()
{
    // tVectorXd q = mStateTraj->mq[mCurState->GetStateId()],
    //           qdot = tVectorXd::Zero(q.size());
    // std::cout << "[FSM] init q = " << q.transpose() << std::endl;
    tVectorXd q = tVectorXd::Zero(mModel->GetNumOfFreedom()),
              qdot = tVectorXd::Zero(mModel->GetNumOfFreedom());
    q[0] = 0.75;
    mModel->SetqAndqdot(q, qdot);
}

/**
 * \brief               Get the current state in FSM
*/
tState *btGenFSM::GetCurrentState() { return this->mCurState; }