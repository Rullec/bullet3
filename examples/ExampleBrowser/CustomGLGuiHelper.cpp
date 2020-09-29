#include "CustomGLGuiHelper.h"
#include "OpenGLGuiHelper.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "Bullet3Common/b3Scalar.h"
#include "CollisionShape2TriangleMesh.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "../OpenGLWindow/ShapeData.h"

#include "../OpenGLWindow/SimpleCamera.h"
#include "BulletGenDynamics/btGenModel/RobotCollider.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"

CustomGLGuiHelper::CustomGLGuiHelper(struct CommonGraphicsApp* glApp, bool useOpenGL2) : OpenGLGuiHelper(glApp, useOpenGL2)
{
}

extern bool shapePointerCompareFunc(const btCollisionObject* colA, const btCollisionObject* colB);
static btVector4 sColors[4] =
	{
		btVector4(60. / 256., 186. / 256., 84. / 256., 1),
		btVector4(244. / 256., 194. / 256., 13. / 256., 1),
		btVector4(219. / 256., 50. / 256., 54. / 256., 1),
		btVector4(72. / 256., 133. / 256., 237. / 256., 1),

		//btVector4(1,1,0,1),
};
void CustomGLGuiHelper::autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld)
{
	//sort the collision objects based on collision shape, the gfx library requires instances that re-use a shape to be added after eachother

	btAlignedObjectArray<btCollisionObject*> sortedObjects;
	sortedObjects.reserve(rbWorld->getNumCollisionObjects());
	
	for (int i = 0; i < rbWorld->getNumCollisionObjects(); i++)
	{
		btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
		sortedObjects.push_back(colObj);
	}
	// printf("[paint] sorted objects = %d\n", sortedObjects.size());
	sortedObjects.quickSort(shapePointerCompareFunc);
	for (int i = 0; i < sortedObjects.size(); i++)
	{
		btCollisionObject* colObj = sortedObjects[i];
		//btRigidBody* body = btRigidBody::upcast(colObj);
		//does this also work for btMultiBody/btMultiBodyLinkCollider?
		btSoftBody* sb = btSoftBody::upcast(colObj);
		if (sb)
		{
			colObj->getCollisionShape()->setUserPointer(sb);
		}
		createCollisionShapeGraphicsObject(colObj->getCollisionShape());
		int colorIndex = colObj->getBroadphaseHandle()->getUid() & 3;

		btVector4 color;
		color = sColors[colorIndex];
		if (colObj->getCollisionShape()->getShapeType() == STATIC_PLANE_PROXYTYPE)
		{
			color.setValue(1, 1, 1, 1);
		}
		GetColor(colObj, color);
		createCollisionObjectGraphicsObject(colObj, color);
	}
}

void CustomGLGuiHelper::GetColor(btCollisionObject* col, btVector4& color)
{
	auto collider = dynamic_cast<btGenRobotCollider*>(col);
	if (collider != nullptr)
	{
		if (collider->mModel->GetCollisionEnabled() == false)
		{
			color[3] = 0.7;
		}
	}
}