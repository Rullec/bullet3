#pragma once
#include "SimbiconTraj.h"

/**
 * \brief           Simbicon FSM state
*/
class btGenSimbiconState
{
public:
    btGenSimbiconState(const Json::Value &conf, int state_id,
                       cRobotModelDynamics *model);
    int GetStateStance(int old_stance) const;
    int GetTrajectoryCount() const;
    double GetStateTime() const;
    int GetNextStateIndex() const;
    bool NeedTransition(double phi, double swing_foot_vertical_force,
                        double stance_foot_vertical_force) const;

    std::vector<btGenSimbiconTraj *> mTrajs;

protected:
    int mStateIndex;               // state index
    int mNextStateIndex;           // next state index
    double mStateTime;             // state elapsed time
    bool mTransitionOnFootContact; // transite state on foot contact
    double mMinPhiBeforeTransitionOnFootContact; // least foot contact
    double
        mMinSwingFootForceForContact; // minimium swing foot contact force to begin transition
};