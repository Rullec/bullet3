#include "SimbiconState.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

btGenSimbiconState::btGenSimbiconState(const Json::Value &conf, int state_id)
{
    mStateIndex = state_id;
    mNextStateIndex = btJsonUtil::ParseAsInt("next_state_id", conf);
    mStateTime = btJsonUtil::ParseAsDouble("time", conf);
    mTransitionOnFootContact =
        btJsonUtil::ParseAsBool("transition_on_foot_contact", conf);
    mMinPhiBeforeTransitionOnFootContact = 0.5;
    mMinSwingFootForceForContact = 20;
}

int btGenSimbiconState::GetStateStance(int old_stance) const
{
    if (old_stance == LEFT_STANCE)
    {
        return RIGHT_STANCE;
    }
    else if (old_stance == RIGHT_STANCE)
    {
        return LEFT_STANCE;
    }
    else
    {
        BTGEN_ASSERT(false);
        return -1;
    }
}

double btGenSimbiconState::GetStateTime() const { return mStateTime; }

int btGenSimbiconState::GetNextStateIndex() const { return mNextStateIndex; }

/**
 * \brief           Judge whether do we need to transite to another state
 * \param phi       motion reference percentage
*/
bool btGenSimbiconState::NeedTransition(double phi,
                                        double swing_foot_vertical_force,
                                        double stance_foot_vertical_force) const
{
    if (mTransitionOnFootContact == true)
    {
        if ((phi > mMinPhiBeforeTransitionOnFootContact &&
             swing_foot_vertical_force > mMinSwingFootForceForContact) ||
            phi >= 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        if (phi >= 1)
            return true;
        else
            return false;
    }
}
