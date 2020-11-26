// #include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include <map>
class btTraj;
class cRobotModelDynamics;

/**
 * \brief           change the contact configuration in current ref traj to Given the local contact point (2 points on foot)
 * 
 * used to reproduce the paper "Contact-aware nonlinear character control"
 * 
 * The resulted traj will be saved to another location, which can be reused later
*/
class btTrajContactMigrator
{
public:
    struct tGivenContactPt
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        tGivenContactPt(int link_id,
                        const tVector &local_pos); // check the homogenous
        int mLinkId;                               // link id
        tVector mLocalPos;                         // local position
    };
    typedef std::map<int, tEigenArr<tGivenContactPt>> tGivenContactPtInfo;
    static void MigrateTrajContactByGivenInfo(cRobotModelDynamics *model,
                                              btTraj *traj,
                                              std::string given_contact_info);
    static tGivenContactPtInfo LoadGivenContactPts(
        cRobotModelDynamics *model,
        const std::string &conf_path); // load given contact info from the file
protected:
    static std::string
    GetAndCheckNewTrajPath(std::string old_path); // create new saved path

    static void MigrateTrajContact(cRobotModelDynamics *model,
                                   const tGivenContactPtInfo &given_contact,
                                   btTraj *traj); // migrate by LSQ Gen force
};
