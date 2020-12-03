
#pragma once
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class btTraj;
class cRobotModelDynamics;
class btCollisionObject;
class btGeneralizeWorld;

/**
 * TargetCalculator works together with the ContactAware controller. 
 * It first receive the mocap data (traj) from outside by the "SetTraj" method
 * Then in each control frame, the "CalcTarget" will be called to calculate a control target for the outside 
 * ContactAwareController
*/
class btGenTargetCalculator
{
public:
    btGenTargetCalculator();
    virtual ~btGenTargetCalculator();
    virtual void Init(btGeneralizeWorld *mWorld, const std::string conf);
    virtual void SetTraj(btTraj *traj_);
    virtual void SetCoef(const Json::Value &conf) = 0;
    virtual void CalcTarget(double dt, int target_frame_id,
                            tVectorXd &tilde_qddot, tVectorXd &tilde_qdot,
                            tVectorXd &tilde_q, tVectorXd &tilde_tau) = 0;
    virtual int GetCalculatedNumOfContact() const = 0;
    // control directly by the adaption controller, mostly for debug purpose.
    // in this mode, the simulation will be only forward by the contact force & control force
    // calculated in this AdaptionController
    virtual void ControlByAdaptionController() = 0;
    virtual void Reset() = 0;
    virtual void SetBulletGUIHelperInterface(struct GUIHelperInterface *inter);

protected:
    cRobotModelDynamics *mModel;
    btTraj *mTraj;
    btGeneralizeWorld *mWorld;
    // model buffer vars
    int num_of_freedom;
    int num_of_underactuated_freedom;
    int mRefFrameId;
    double mdt;
    // draw contact points utils
    struct GUIHelperInterface *mBulletGUIHelper;
    std::vector<btCollisionObject *> mDrawPointsList;

    // helper functions
    virtual void PreCalcTarget(double dt, int target_id);
    void DrawPoint(const tVector3d &pos, double radius = 0.05);
    void ClearDrawPoints();
};