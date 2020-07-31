#include "ColObjBase.h"

cCollisionObject::cCollisionObject(eColObjType type, const std::string& name)
{
	mType = type;
	mName = name;
}

eColObjType cCollisionObject::GetType()
{
	return mType;
}

const std::string& cCollisionObject::GetName()
{
	return mName;
}