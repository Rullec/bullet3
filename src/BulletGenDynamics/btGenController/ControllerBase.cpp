#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "../examples/CommonInterfaces/CommonGUIHelperInterface.h"
#include "BulletGenDynamics/btGenWorld.h"
btGenControllerBase::btGenControllerBase(ebtGenControllerType type,
                                         btGeneralizeWorld *world)
    : mCtrlType(type), mWorld(world)
{
    mModel = nullptr;
    mCurdt = 0;
    mBulletGUIHelper = nullptr;
    mDrawFrame.clear();
    mDrawPointsList.clear();
    mDrawLines.clear();
    mCollisionShapes.clear();
}

btGenControllerBase::~btGenControllerBase()
{
    for (auto &x : mDrawPointsList)
        mWorld->GetInternalWorld()->removeCollisionObject(x);
    mDrawPointsList.clear();
    for (auto &x : mDrawFrame)
        mWorld->GetInternalWorld()->removeCollisionObject(x);
    mDrawFrame.clear();
}

void btGenControllerBase::Init(cRobotModelDynamics *model,
                               const std::string &conf)
{
    mTime = 0;
    mModel = model;
}

ebtGenControllerType btGenControllerBase::GetCtrlType() const
{
    return mCtrlType;
}

void btGenControllerBase::Update(double dt)
{
    mCurdt = dt;
    mTime += dt;
}

void btGenControllerBase::ClearDrawPoints()
{
    if (mBulletGUIHelper == nullptr)
        return;
    auto inter_world = mWorld->GetInternalWorld();
    // std::cout << "clear points num = " << mDrawPointsList.size()
    //           << " now = " << inter_world->getCollisionObjectArray().size()
    //           << std::endl;

    for (auto &pt : this->mDrawPointsList)
    {
        // inter_world->getCollisionObjectArray().remove(pt);
        // delete pt->getCollisionShape();
        mWorld->GetInternalWorld()->removeCollisionObject(pt);
        mBulletGUIHelper->removeGraphicsInstance(pt->getUserIndex());
        delete pt;
    }
    // std::cout << "[debug] clear points " << mDrawPointsList.size() << std::endl;
    mDrawPointsList.clear();
}

/**
 * \brief               Draw a point in specified position and radius
*/
void btGenControllerBase::DrawPoint(const tVector3d &pos,
                                    double radius /* = 0.05*/)
{
    BTGEN_ASSERT(mBulletGUIHelper != nullptr);
    btCollisionShape *colShape = mWorld->GetSphereCollisionShape(radius);
    btCollisionObject *obj = new btCollisionObject();
    btTransform trans;
    // trans.setOrigin(btVector3(pos[0], pos[1], pos[2]));
    trans.setOrigin(btVector3(pos[0], pos[1], pos[2]));
    obj->setWorldTransform(trans);
    obj->setCollisionShape(colShape);
    obj->setCollisionFlags(0);
    mWorld->GetInternalWorld()->addCollisionObject(obj, 0, 0);
    mDrawPointsList.push_back(obj);
}

/**
 * \brief               Set GUI Interface for drawing
*/
void btGenControllerBase::SetBulletGUIHelperInterface(
    struct GUIHelperInterface *inter)
{
    mBulletGUIHelper = inter;
}

/**
 * \brief               Draw reference frame, three axes
 * \param rot   frame world orientation
 * \param pos   frame origin world pos
*/
void btGenControllerBase::DrawFrame(const tMatrix &transform)
{
    // if it doesn't exist, create three lines, and put them into mDrawFrame
    double length = 0.5, // 0.5m
        radius = 0.01;   // 1cm
    if (mDrawFrame.size() == 0)
    {
        for (int i = 0; i < 3; i++)
            this->mDrawFrame.push_back(CreateLine(length, radius));
    }

    BTGEN_ASSERT(mDrawFrame.size() == 3);
    // 2. set the x, y, z object's position and rotation
    tVector3d axes[] = {tVector3d(1, 0, 0), tVector3d(0, 1, 0),
                        tVector3d(0, 0, 1)};
    for (int i = 0; i < 3; i++)
    {
        // calculate the transformation from rest pose to unit axes
        // and unit axes to current rot
        tVector3d axis = axes[i];
        tMatrix first_trans = tMatrix::Identity(); // from (1, 0, 0) to axes
        {
            tVector3d obj_rest_orient = tVector3d(0, 1, 0);
            first_trans.block(0, 0, 3, 3) =
                btMathUtil::DirToRotMat(btMathUtil::Expand(axis, 0),
                                        btMathUtil::Expand(obj_rest_orient, 0))
                    .block(0, 0, 3, 3);
            first_trans.block(0, 3, 3, 1) = length * axis / 2;
        }

        // multiple to another specified transform
        first_trans = transform * first_trans;
        mDrawFrame[i]->setWorldTransform(
            btBulletUtil::tMatrixTobtTransform(first_trans));
    }
}

/**
 * \brief               Create a capsule by given length, and radisu
*/
btCollisionObject *btGenControllerBase::CreateLine(double length, double radius)
{
    btCollisionShape *colShape = nullptr;
    btCollisionObject *obj = new btCollisionObject();
    colShape = mWorld->GetCapsuleCollisionShape(radius, length);
    btTransform trans;
    trans.setOrigin(btVector3(0, 0, 0));
    obj->setWorldTransform(trans);
    obj->setCollisionShape(colShape);
    obj->setCollisionFlags(0);
    mWorld->GetInternalWorld()->addCollisionObject(obj, 0, 0);
    return obj;
}

/**
 * \brief               Draw a line
*/
void btGenControllerBase::DrawLine(const tVector3d &st, const tVector3d &ed)
{
    tVector3d dir = (ed - st).normalized();
    double length = (ed - st).norm();
    tVector3d pos = (ed + st) / 2;
    btCollisionObject *obj = CreateLine(length, 0.01);
    mDrawLines.push_back(obj);
    // cur pos : (0, 0, 0), cur dir : (0, 1, 0)
    // target pose: (ed + st) / 2, target dir: (ed - st).noramlzied
    tMatrix rot_mat = btMathUtil::DirToRotMat(btMathUtil::Expand(dir, 0),
                                              tVector(0, 1, 0, 0));
    rot_mat.block(0, 3, 3, 1) = pos;
    obj->setWorldTransform(btBulletUtil::tMatrixTobtTransform(rot_mat));
}

void btGenControllerBase::ClearLines()
{
    if (mBulletGUIHelper == nullptr)
        return;
    auto inter_world = mWorld->GetInternalWorld();

    for (auto &pt : this->mDrawLines)
    {
        // inter_world->getCollisionObjectArray().remove(pt);
        // delete pt->getCollisionShape();
        mWorld->GetInternalWorld()->removeCollisionObject(pt);
        mBulletGUIHelper->removeGraphicsInstance(pt->getUserIndex());

        delete pt;
    }

    std::cout << "[debug] clear lines " << mDrawLines.size() << std::endl;
    mDrawLines.clear();
}
// btCollisionShape *btGenControllerBase::GetSphereCollsiionShape(double radius)
// {
//     for (auto &x : mCollisionShapes)
//     {
//         if (x.second->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
//         {
//             auto sphere = dynamic_cast<btSphereShape *>(x.second);
//             BTGEN_ASSERT(sphere != nullptr);
//             if (sphere->getRadius() == radius)
//             {
//                 x.first++;
//                 return x.second;
//             }
//         }
//     }

//     // no, create
//     btSphereShape *sphere_shape = new btSphereShape(radius);
//     mCollisionShapes.push_back(
//         std::make_pair<int, btCollisionShape *>(1, sphere_shape));
//     return sphere_shape;
// }

// btCollisionShape *btGenControllerBase::GetCapsuleCollsiionShape(double param1,
//                                                                 double param2,
//                                                                 double param3)
// {
//     BTGEN_ASSERT(false);
// }