
#include "btBulletDynamicsCommon.h"
#include <vector>

// create multibody by adding constraints between links
class cConstraintMultibody{
public:
    cConstraintMultibody(btDiscreteDynamicsWorld * world, btAlignedObjectArray<btCollisionShape*> & col_shapes, int num_links, bool floating_base, bool spherical);

    btVector3 GetLinkWorldPos(int id);
    btQuaternion GetLinkWorldRot(int id);
    btVector3 GetLinkWorldVel(int id);
    btVector3 GetLinkWorldOmega(int id);
    btVector3 GetCOM();
    double GetLinkMass(int id);
    double GetTotalMass();

    btRigidBody * GetLink(int id);
    btVector3 CalcLinearMomentum();
    btVector3 CalcAngMomentum();
    
protected:
    btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color = btVector4(1, 0, 0, 1));

    // simulation essentials
    btDiscreteDynamicsWorld * mWorld;

    // multibody info
    int mNumLinks;
    std::vector<btRigidBody * > mLinkLst;
};