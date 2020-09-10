#include "OpenGLGuiHelper.h"
#include "LinearMath/btVector3.h"
struct CustomGLGuiHelper : public OpenGLGuiHelper
{
	CustomGLGuiHelper(struct CommonGraphicsApp* glApp, bool useOpenGL2);
	virtual ~CustomGLGuiHelper() = default;

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld);

protected:
	void GetColor(btCollisionObject*, btVector4& color);
};