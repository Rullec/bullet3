#pragma once
#include <string>
#include <vector>
class btGeneralizeWorld;
class cRobotModelDynamics;
enum eTransitionCondition
{
    INVALID_CONDITION,
    ElapsedTime,
    LinkContact,
    NUM_OF_TRANSITION_CONDITION
};

const std::string gTransitionConditionStr[NUM_OF_TRANSITION_CONDITION] = {
    "invalid", "ElapsedTime", "LinkContact"};
/**
 * \brief       the abstract base class of transition condition used in SIMBICON FSM
*/
struct tTransitionCondition
{
    tTransitionCondition(int origin_state_id, int target_state_id,
                         eTransitionCondition type);
    virtual void Update(double dt) = 0;
    virtual int GetTransitionTargetId() const = 0;
    virtual void Reset() = 0;

protected:
    eTransitionCondition mType;
    int mOriginStateId;
    int mTargetStateId;
};

/**
 * \brief       "elasped time" condition in FSM
*/
struct tElapsedCondition : public tTransitionCondition
{
    tElapsedCondition(int origin_state_id, int target_state_id,
                      double elasped_time);
    virtual void Update(double dt) override final;
    virtual int GetTransitionTargetId() const override final;
    virtual void Reset() override final;

protected:
    double mCurTime;              // current elasped time
    double mThresholdElaspedTime; // threshold time period
};

/**
 * \brief       "foot contact" condition in FSM
*/
struct tContactCondition : public tTransitionCondition
{
    tContactCondition(int origin_state_id, int target_state_id,
                      btGeneralizeWorld *world, cRobotModelDynamics *model,
                      int link_id);
    virtual void Update(double dt) override final;
    virtual int GetTransitionTargetId() const override final;
    virtual void Reset() override final;

protected:
    btGeneralizeWorld *mWorld;
    cRobotModelDynamics *mModel;
    int mLinkId;
};

/**
 * \brief       the state unit in SIMBICON FSM
*/
struct tState
{
    tState(int state_id, int default_swing_hip, int default_stance_hip);
    ~tState();

    void Update(double dt);
    void AddTransitionCondition(tTransitionCondition *cond);
    int GetStateId() const;
    int GetTargetId() const;
    void Print() const;
    int GetDefaultSwingHipId() const;
    int GetDefaultStanceHipId() const;

protected:
    int mStateId;
    int mDefaultSwingHipId, mDefaultStanceHipId;
    std::vector<tTransitionCondition *> mTransitionConditions;
};

namespace Json
{
class Value;
}
tTransitionCondition *BuildTransitionCondition(const int origin_state_id,
                                               btGeneralizeWorld *world,
                                               cRobotModelDynamics *model,
                                               const Json::Value &value);