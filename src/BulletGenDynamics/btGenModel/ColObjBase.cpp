#include "ColObjBase.h"

btGenCollisionObject::btGenCollisionObject(eColObjType type, const std::string& name)
{
	mType = type;
	mName = name;
}

eColObjType btGenCollisionObject::GetType()
{
	return mType;
}

const std::string& btGenCollisionObject::GetName()
{
	return mName;
}