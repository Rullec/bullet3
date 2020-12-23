#include "ContactManager.h"
#include "BulletGenDynamics/btGenCollision/btGenCollisionDispatcher.h"
#include "BulletGenDynamics/btGenWorld.h"

btGenContactManager::btGenContactManager(btGeneralizeWorld *world)

{
    mWorld = world;
}

/**
 * \brief       Get the contact number between two objectsI
*/
int btGenContactManager::GetTwoObjsNumOfContact(
    const btCollisionObject *b0, const btCollisionObject *b1) const
{
    auto m_dispatcher = mWorld->GetDispatcher();
    int num_of_manifold = m_dispatcher->getNumManifolds();
    for (int i = 0; i < num_of_manifold; i++)
    {
        auto mani = m_dispatcher->getManifoldByIndexInternal(i);

        if ((mani->getBody0() == b0 && mani->getBody1() == b1) ||
            (mani->getBody0() == b1 && mani->getBody1() == b0))
        {
            return mani->getNumContacts();
        }
    }
    return 0;
}

/**
 * \brief       Get the total, vertical contact force between the body and ground
*/
double btGenContactManager::GetVerticalTotalForceWithGround(
    const btCollisionObject *body) const

{
    const auto ground = mWorld->GetGround();
    auto contact_forces = mWorld->GetContactForces();

    double total_v_f = 0;
    for (auto &f : contact_forces)
    {
        if (f->mObj == body && f->mPassiveObj == ground)
        {
            total_v_f += f->mForce[1];
        }
    }
    return total_v_f;
}

tVector btGenContactManager::GetTotalForceWithGround(
    const btCollisionObject *body) const
{
    const auto ground = mWorld->GetGround();
    auto contact_forces = mWorld->GetContactForces();

    tVector total_f = tVector::Zero();
    for (auto &f : contact_forces)
    {
        if (f->mObj == body && f->mPassiveObj == ground)
        {
            total_f += f->mForce;
        }
    }
    return total_f;
}

int btGenContactManager::GetNumOfContactWithGround(
    const btCollisionObject *b1) const
{
    return this->GetTwoObjsNumOfContact(b1, mWorld->GetGround());
}