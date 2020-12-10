#ifndef CONTROLLER_BASW_H_
#define CONTROLLER_BASW_H_
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
enum ebtGenControllerType
{
    PDController,
    SimbiconSPDController,
    ContactAwareController,
    SimbiconController,
    ShowLimitController,
    BTGEN_NUM_CONTROLLER_TYPE
};

static const std::string
    gbtGenControllerTypeStr[ebtGenControllerType::BTGEN_NUM_CONTROLLER_TYPE] = {
        "PDController", "SimbiconSPDController", "ContactAwareController",
        "SimbiconController", "ShowLimitController"};
class btGeneralizeWorld;
class cRobotModelDynamics;
class btCollisionObject;
struct GUIHelperInterface;
class btCollisionShape;
class btGenControllerBase
{
public:
    btGenControllerBase(ebtGenControllerType type, btGeneralizeWorld *world);
    virtual ~btGenControllerBase();
    virtual void Init(cRobotModelDynamics *model, const std::string &conf);
    virtual void Update(double dt);
    virtual void Reset() = 0;
    ebtGenControllerType GetCtrlType() const;
    virtual void SetBulletGUIHelperInterface(struct GUIHelperInterface *inter);

protected:
    double mTime;
    cRobotModelDynamics *mModel;
    btGeneralizeWorld *mWorld;
    ebtGenControllerType mCtrlType;
    double mCurdt;

    // draw utils
    struct GUIHelperInterface *mBulletGUIHelper;
    std::vector<btCollisionObject *> mDrawPointsList; // draw points list
    std::vector<btCollisionObject *>
        mDrawFrame; // draw a reference 3d frame (x y z), vector = three axes

    std::vector<btCollisionObject *>
        mDrawLines; // draw contact force lines list
    std::vector<std::pair<int, btCollisionShape *>> mCollisionShapes;

    // methods
    void DrawPoint(const tVector3d &pos, double r = 0.05);
    void DrawFrame(const tMatrix &trans);
    void DrawLine(const tVector3d &st, const tVector3d &ed);
    void ClearDrawPoints();
    void ClearLines();

private:
    btCollisionObject *CreateLine(double length, double radius);
    // btCollisionShape *GetSphereCollsiionShape(double radius);
    // btCollisionShape *GetCapsuleCollsiionShape(double param1, double param2,
    //                                            double param3);
};
#endif /*CONTROLLER_BASW_H_*/