#include "FSMUtil.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenSolver/ContactManager.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include "FSM.h"
/**
 * \brief               constructor of transition condition
 * \param origin_id     original state id
 * \param target_id     target state id
 * \param type          enum type of this condition
*/
tTransitionCondition::tTransitionCondition(int origin_id, int tar_id,
                                           eTransitionCondition type)
    : mOriginStateId(origin_id), mTargetStateId(tar_id), mType(type)
{
}

// ====================================== elasped time condition begin ========================
tElapsedCondition::tElapsedCondition(int origin_state_id, int target_state_id,
                                     double elasped_time)
    : tTransitionCondition(origin_state_id, target_state_id,
                           eTransitionCondition::ElapsedTime),
      mThresholdElaspedTime(elasped_time)
{
    mCurTime = 0;
}

/**
 * \brief               Update the elasped 
*/
void tElapsedCondition::Update(double dt) { mCurTime += dt; }

/**
 * \brief               At this moment, judge whether the state should be transited to another one
 * \return              target state id if this condition is activated, -1 deactivied
*/
int tElapsedCondition::GetTransitionTargetId() const
{
    return mCurTime > mThresholdElaspedTime ? mTargetStateId : -1;
}

void tElapsedCondition::Reset() { mCurTime = 0; }
// ====================================== elasped time condition end ========================

// ====================================== contact condition begin ========================
/**
 * \brief               This "contact" condition will be activated 
 * when the specified link is contact with the ground
*/
tContactCondition::tContactCondition(int origin_state_id, int target_state_id,
                                     btGeneralizeWorld *world,
                                     cRobotModelDynamics *model, int link_id)
    : tTransitionCondition(origin_state_id, target_state_id,
                           eTransitionCondition::LinkContact),
      mWorld(world), mModel(model), mLinkId(link_id)
{
    mMinContactForceThreshold = 20;
    mMinElaspedTimeThreshold = 0.05;
    mCurTime = 0;
}

void tContactCondition::Update(double dt) { mCurTime += dt; }

/**
 * \brief           Judge wheter the specified link is contact with the ground, if so, we need to do transition
 * 1. if the elasped time is too short, return -1
 * 2. if the vertical contact force is too small (or hasn't contact), return -1
 * 3. otherwise, convert to another state, return the target state id
*/
int tContactCondition::GetTransitionTargetId() const
{
    // 1. elasped time threshold
    if (mCurTime < this->mMinElaspedTimeThreshold)
        return -1;

    // 2. get the contact force

    auto link_collider = mModel->GetLinkCollider(mLinkId);
    auto ground = mWorld->GetGround();
    auto manager = mWorld->GetContactManager();
    int num = manager->GetTwoObjsNumOfContact(ground, link_collider);
    if (num > 0)
    {
        double force = manager->GetVerticalTotalForceWithGround(link_collider);
        if (force > this->mMinContactForceThreshold)
            return this->mTargetStateId;
        else
        {
            printf("[error] state: vertical contact force %.4f is too small, "
                   "doesn't change!\n",
                   force);
            return -1;
        }
    }
    else
        return -1;
}

void tContactCondition::Reset() { mCurTime = 0; }

// ====================================== contact condition end ========================

// ====================================== state begin ========================

tState::tState(int state_id, int default_stance, std::string stance_update_mode)
    : mStateId(state_id), mDefaultStance(default_stance),
      mStanceUpdateMode(stance_update_mode)
{
    mTransitionConditions.clear();
    BTGEN_ASSERT(
        (stance_update_mode == "default" || stance_update_mode == "reverse") &&
        "unrecognized update mode");
}

tState::~tState()
{
    for (auto &x : mTransitionConditions)
        delete x;
    mTransitionConditions.clear();
}

/**
 * \brief           update this state (particularly, its transition conditions)
*/
void tState::Update(double dt)
{
    for (auto &x : mTransitionConditions)
        x->Update(dt);
}

void tState::AddTransitionCondition(tTransitionCondition *cond)
{
    mTransitionConditions.push_back(cond);
}
int tState::GetStateId() const { return mStateId; }
/**
 * \brief           judge and find the target transition state at this moment
 * \return          return the target state id we want to trainsite to. -1 means no transition
*/
int tState::GetTargetId() const
{
    int target_id = -1;
    for (auto &x : mTransitionConditions)
    {
        int cur_target = x->GetTransitionTargetId();
        if (target_id == -1)
        {
            if (cur_target == -1)
                continue;
            else
                target_id = cur_target;
        }
        else
        {
            if (cur_target != -1)
            {
                printf("[error] tState: two or more conditions are activated "
                       "in state %d, keep same\n",
                       this->mStateId);
            }
        }
    }
    return target_id;
}

/**
 * \brief           When the transition of states occured, this function will determine the new stance configuration
 * \param old_stance    old stance situation
*/
int tState::CalcNewStance(int old_stance) const
{
    // now simply opposite all stance
    if (mStanceUpdateMode == "default")
    {
        return mDefaultStance;
    }
    else if (mStanceUpdateMode == "reverse")
    {
        if (old_stance == BTGEN_RIGHT_STANCE)
            return BTGEN_LEFT_STANCE;
        else if (old_stance == BTGEN_LEFT_STANCE)
            return BTGEN_RIGHT_STANCE;
        else
        {
            BTGEN_ASSERT(false);
        }
    }
    else
    {
        // unknown mode
        BTGEN_ASSERT(false);
    }
    return -1;
}
/**
 * \brief       Show the state info (id and conds)
*/
void tState::Print() const
{
    printf("[log] for state %d, it has %d conditions\n", mStateId,
           this->mTransitionConditions.size());
}

void tState::Reset()
{
    for (auto &x : mTransitionConditions)
        x->Reset();
}
// ====================================== state end ========================

// ====================================== BuildCondition begin ========================
tTransitionCondition *BuildTransitionCondition(const int origin_state_id,
                                               btGeneralizeWorld *world,
                                               cRobotModelDynamics *model,
                                               const Json::Value &value)
{
    // 1. check the type
    std::string cond_type = btJsonUtil::ParseAsString("condition_type", value);

    eTransitionCondition type = eTransitionCondition::INVALID_CONDITION;
    for (uint i = 0; i < eTransitionCondition::NUM_OF_TRANSITION_CONDITION; i++)
    {
        if (gTransitionConditionStr[i] == cond_type)
        {
            type = static_cast<eTransitionCondition>(i);
        }
    }
    if (type == eTransitionCondition::INVALID_CONDITION)
    {
        printf("[error] BuildTransitionCondition failed for type %s\n",
               cond_type.c_str());
        exit(0);
    }

    tTransitionCondition *cond = nullptr;
    // 2. build the corresponding condition
    int target_state_id = btJsonUtil::ParseAsInt("target_state_id", value);
    switch (type)
    {
    case eTransitionCondition::ElapsedTime:
    {
        double time = btJsonUtil::ParseAsDouble("elapsed_time", value);
        cond = new tElapsedCondition(origin_state_id, target_state_id, time);
    }
    break;
    case eTransitionCondition::LinkContact:
    {
        std::string link_name =
            btJsonUtil::ParseAsString("contact_link_name", value);
        auto link_ptr = model->GetLink(link_name);
        BTGEN_ASSERT(link_ptr != nullptr);
        int link_id = link_ptr->GetId();
        cond = new tContactCondition(origin_state_id, target_state_id, world,
                                     model, link_id);
    }
    break;
    default:
        BTGEN_ASSERT(false && "unsupporteed type");
        break;
    }

    return cond;
}
// ====================================== BuildCondition end ========================
