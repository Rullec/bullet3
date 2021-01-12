#pragma once
#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include "ConstraintData.h"

class btGeneralizeWorld;
// class btDiscreteDynamicsWorld;
class btPersistentManifold;
class btGenRigidBody;
// class cNativeLemkeLCPSolver;
// class cQPSolver;
// class cMatlabQPSolver;
class cLCPSolverBase;
struct btGenRobotCollider;
struct btGenCollisionGroupData;
class cRobotModelDynamics;
class btGenContactForce
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenContactForce(btGenCollisionObject *mObj,
                      btGenCollisionObject *passive_object,
                      const tVector &mForce, const tVector &mWorldPos,
                      bool is_self_collision);
    btGenCollisionObject *mObj;        // object that force is applied
    btGenCollisionObject *mPassiveObj; // collision pair obj
    tVector mForce, mWorldPos; // position in world and force in world frame
    bool mIsMBSelfCollision;
};

class btGenMBContactForce : public btGenContactForce
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenMBContactForce(btGenRobotCollider *collider,
                        btGenCollisionObject *passive_obj, const tVector &f,
                        const tVector &world_pos, const tVector &local_pos,
                        bool is_self_collision);
    int mLinkId;
    tVector mLocalPos;
};

struct btGenConstraintGeneralizedForce
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenConstraintGeneralizedForce(cRobotModelDynamics *_model, int _dof_id,
                                    double _val)
        : model(_model), dof_id(_dof_id), value(_val)
    {
    }
    btGenConstraintGeneralizedForce(
        const btGenConstraintGeneralizedForce &old_obj)
    {
        model = old_obj.model;
        dof_id = old_obj.dof_id;
        value = old_obj.value;
    }
    cRobotModelDynamics *model;
    double dof_id;
    double value;
};

class btGenContactSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    btGenContactSolver();
    explicit btGenContactSolver(const std::string &config_path,
                                btGeneralizeWorld *world);
    virtual ~btGenContactSolver();

    void ConstraintProcess(float dt);
    void Reset();
    bool GetEnableConvertMatTest() { return mEnableConvertMatTest; }
    bool GetEnableGradientOverCtrlForce();

    std::vector<btGenContactForce *> GetContactForces();
    std::vector<btGenConstraintGeneralizedForce *>
    GetConstraintGeneralizedForces();
    tMatrixXd GetDGenConsForceDCtrlForce() const;

protected:
    int mNumFrictionDirs;
    double mMu;
    float cur_dt;
    bool mEnableMultibodySelfCol; // enable the self-collision of multibody
    bool mEnableConvertMatTest;
    bool mEnableContactPenetrationResolve; // resolve contact penetration or not
    bool
        mEnableJointLimitPenetrationResolve; // resolve joint limit penetration or not
    bool mEnableLCPProfiling;                // profiling the LCP solver
    double mContactErp; // erp value when resolving the contact penetration
    double
        mJointLimitErp; // erp value when resolving the joint limit penetration
    bool mEnableDebugOutput;

    // lcp config
    bool mEnableLCPCalc;
    bool mEnableContactLCP;
    bool mEnableFrictionalLCP;
    bool mEnableJointLimitLCP;
    bool mEnableGradientOverCtrlForce;
    bool mEnableTestGradientOverCtrlForce;
    bool mUseLCPResult;

    // SI config
    bool mEnableSICalc;
    bool mEnableFrictionalSI;
    bool mUseSIResult;
    int mMaxItersSI;
    double mConvergeThresholdSI;

    bool mEnableSILCPComparision; // compare the result of SI and LCP
    double mDiagonalEps;

    btGeneralizeWorld *mGenWorld;
    btDiscreteDynamicsWorld *mWorld;
    cLCPSolverBase *mLCPSolver;

    std::vector<int>
        map_colobjid_to_groupid; // which collision group does this colliion-object belong to?
    std::vector<btGenCollisionGroupData *> mColGroupData;
    std::vector<btGenContactPairData *> mContactPairConsData;
    std::vector<btGenJointLimitData *> mJointLimitConstraintData;
    std::vector<cRobotModelDynamics *> mMultibodyArray;
    int mNumContactPairs;
    int mNumConstraints;
    int mNumJointLimitConstraints;

    // 1. LCP buffer vars, rel vel buffer: cartesian force to cartesian rel
    // velocity
    tMatrixXd rel_vel_convert_mat;
    tVectorXd rel_vel_convert_vec;
    tMatrixXd
        rel_vel_convert_mat_of_residual; // the convert mat from multibody gen vel to rel vel, used in diff MBRL

    // 2. LCP buffer vars, normal and tangent convert mat buffer, cartesian
    // force to normal and tangent rel vel
    tMatrixXd normal_vel_convert_mat; // one contact point one line
    tVectorXd normal_vel_convert_vec; // one contact point one real number
    tMatrixXd
        normal_vel_convert_mat_of_residual; // the convert mat from multibody gen vel to normal rel vel, used in diff MBRL

    tMatrixXd tangent_vel_convert_mat; // one contact point N lines, N =
                                       // mNumFrictionDir
    tVectorXd tangent_vel_convert_vec; // one contact point one real number

    tMatrixXd
        tangent_vel_convert_mat_of_residual; // the convert mat from multibody gen vel to tangent rel vel, used in diff MBRL

    // 3. LCP buffer vars, from result vector to normal/tangent velcoity
    // (ultimate convert mat)
    tMatrixXd normal_vel_convert_result_based_mat;
    tVectorXd normal_vel_convert_result_based_vec;
    tMatrixXd
        normal_vel_convert_mat_result_based_of_residual; // the convert mat from multibody gen vel to normal rel vel, based on result vector, used in diff MBRL

    tMatrixXd tangent_vel_convert_result_based_mat;
    tVectorXd tangent_vel_convert_result_based_vec;
    tMatrixXd
        tangent_vel_convert_mat_result_based_of_residual; // the convert mat from multibody gen vel to tangent rel vel, based on result vector, used in diff MBRL
    tMatrixXd
        n_convert_mat_final; // convert matrix from robot collider residual to n (LCP residual)

    tMatrixXd
        DGenConsForceDGenCtrlForce; // the derivative of LCP constraint force w.r.t generalized control force of the char
    // LCP condition: x \perp (M * x + n)
    tMatrixXd M;
    tVectorXd x_lcp, n;
    tVectorXd f_lcp; // cartesian contact forces array for LCP problem

    // Sequential impulse solve result x_si
    tVectorXd x_si;

    // contact forces buffer
    std::vector<btGenContactForce *> contact_force_array;
    std::vector<btGenConstraintGeneralizedForce *> contact_torque_array;

    bool ConstraintSetup();
    void ConstraintSolve();
    void SolveByLCP();
    void SolveBySI();
    tVectorXd ConvertLCPResult(const tVectorXd &raw_lcp_result) const;
    void VerifySolution();
    double InternalIterationSI();
    void SetupDataForSI();
    void UpdateDataForSI();
    void CompareSILCPResult();

    void UpdateVelocity(float dt);
    void ClearAllConstraintForce();
    void ClearConstraintForceTorqueArrays(
        std::vector<btGenContactForce *> &contact_force_array,
        std::vector<btGenConstraintGeneralizedForce *> &contact_torque_array)
        const;
    void ConstraintFinished();
    void CalcCMats(tMatrixXd &C_lambda, tMatrixXd &C_mufn, tMatrixXd &C_c);
    cRobotModelDynamics *CollectMultibody();
    void RebuildCollisionGroup();
    void DeleteColObjData();
    void DeleteConstraintData();
    void AddManifold(btPersistentManifold *mani);
    void AddJointLimit();

    void CalcAbsvelConvertMat();     // cartesian force -> contact point abs vel
    void CalcRelvelConvertMat();     // cartesian force -> contact point rel vel
    void CalcDecomposedConvertMat(); // cartesian force -> contact point
                                     // normal/tangent rel vel
    void CalcResultVectorBasedConvertMat(); // result vector -> contact point
                                            // normal/tangent rel vel
    void CalcContactForceTorqueArrays(
        std::vector<btGenContactForce *> &contact_force_array,
        std::vector<btGenConstraintGeneralizedForce *> &contact_torque_array,
        const tVectorXd &) const;
    // Test cartesian force to cartesian velcoity convert mat
    void PushState(const std::string &name);
    void PopState(const std::string &name);
    void TestCartesianForceToCartesianVel();
    void TestCartesianForceToCartesianRelVel(const tMatrixXd &convert_mat,
                                             const tVectorXd &convert_vec);
    void TestCartesianForceToNormalAndTangetRelVel(const tMatrixXd &normal_mat,
                                                   const tVectorXd &normal_vec,
                                                   const tMatrixXd &tan_mat,
                                                   const tVectorXd &tan_vec);
    void TestCartesianForceToNormalAndTangetResultBasedRelVel(
        const tMatrixXd &normal_mat, const tVectorXd &normal_vec,
        const tMatrixXd &tan_mat, const tVectorXd &tan_vec);
    void TestSICartesianConvertMatAndVec();
    void
    TestAddContactAwareForceIfPossible(const tEigenArr<tVector> &contact_forces,
                                       const std::vector<int> &);
    bool IsMultibodyAndVelMax(btGenCollisionObject *body);

    // calculate jacobian d(gen_constraint_force)/d(x_lcp_vector)
    tMatrixXd CalcDGenConsForceDx() const;
    void TestDGenConsForceDx(); // test
    // void TestDxDn(const tVectorXd &M, const tVectorXd &n); // test
    // calculate jacobian d(n)/d(control_gen_force), n is the residual vector in LCP
    tMatrixXd CalcDnDCtrlForce() const;
    void TestDnDCtrlForce(); // test
    // calculate d(gen_constraint_force)/d(control_gen_force)
    tMatrixXd CalcDxDCtrlForce() const;
    void TestDxDCtrlForce();
    tMatrixXd CalcDGenConsForceDCtrlForce() const;
    void TestDGenConsForceDCtrlForce();

    int GetLCPSolutionSize() const;
    tVectorXd CalcConsGenForce(const tVectorXd &lcp_result_vector) const;
};