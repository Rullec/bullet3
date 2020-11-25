#include "ContactSolver.h"

/**
 * \brief           contact helper manager
*/
class btGeneralizeWorld;
class btGenContactManager
{
public:
    btGenContactManager(btGeneralizeWorld *world);
    int GetTwoObjsNumOfContact(const btCollisionObject *b1,
                               const btCollisionObject *b2) const;
    double GetVerticalTotalForceWithGround(const btCollisionObject *b1) const;

protected:
    btGeneralizeWorld *mWorld;
};