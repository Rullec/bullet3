#pragma once
#define LEFT_STANCE 0
#define RIGHT_STANCE 1

#define LEFT_STANCE_STR ("left_stance")
#define RIGHT_STANCE_STR ("right_stance")

namespace Json
{
class Value;
};

class cRobotModelDynamics;

class btGenSimbiconState
{
public:
    btGenSimbiconState(const Json::Value &conf, int state_id);
    int GetStateStance(int old_stance) const;
    double GetStateTime() const;
    int GetNextStateIndex() const;
    bool NeedTransition(double phi, double swing_foot_vertical_force,
                        double stance_foot_vertical_force) const;

protected:
    int mStateIndex;               // state index
    int mNextStateIndex;           // next state index
    double mStateTime;             // state elapsed time
    bool mTransitionOnFootContact; // transite state on foot contact
    double mMinPhiBeforeTransitionOnFootContact; // least foot contact
    double
        mMinSwingFootForceForContact; // minimium swing foot contact force to begin transition
};