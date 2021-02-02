#include "ConstraintData.h"
#include "BulletGenDynamics/btGenController/ContactAwareController/btGenContactAwareController.h"
#include "BulletGenDynamics/btGenModel/RobotCollider.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenModel/SimObj.h"
#include <fstream>
#include <iostream>

extern btGenRigidBody *UpcastRigidBody(const btCollisionObject *col);
extern btGenRobotCollider *UpcastRobotCollider(const btCollisionObject *col);
extern btGenCollisionObject *UpcastColObj(const btCollisionObject *col);

btGenLCPConstraintData::btGenLCPConstraintData(int c_id,
                                               btGenLCPConstraintType type)
    : constraint_id(c_id), mConstraintType(type)
{
    jac0.resize(0, 0);
    jac1.resize(0, 0);
}

btGenJointLimitData::btGenJointLimitData(int constraint_id_,
                                         cRobotModelDynamics *multibody_,
                                         int dof_id_, bool is_upper_bound_,
                                         double violate_value_)
    : btGenLCPConstraintData(constraint_id_,
                             btGenLCPConstraintType::BTGEN_LCP_JOINT_LIMIT_CONS)
{
    multibody = multibody_;
    dof_id = dof_id_;
    is_upper_bound = is_upper_bound_;
    violate_value = violate_value_;
    joint_direction = multibody->GetFreedomDirectionWorldFrame(dof_id);
    // if (is_upper_bound) joint_direction *= -1;
}

void btGenJointLimitData::CalcJacobian()
{
    int n_dof = this->multibody->GetNumOfFreedom();
    jac0.noalias() = tMatrixXd::Zero(1, n_dof);
    jac0(0, this->dof_id) = 1;
    if (is_upper_bound)
    {
        jac0(0, this->dof_id) *= -1;
    }
}

btGenContactPairData::btGenContactPairData(int c_id)
    : btGenLCPConstraintData(
          c_id, btGenLCPConstraintType::BTGEN_LCP_CONTACT_PAIR_CONS)
{
    mbody0GroupId = -1;
    mbody1GroupId = -2;
    mIsMBSelfCollision = false;
}

/**
 * For non-self collision, the jacobian is simple Jv
 * For self collisin(2 links contacted with each other, the jacobian is
 * (Jva-Jvb))
*/
void btGenContactPairData::CalcJacobian()
{
    if (mBodyA->GetType() == eColObjType::RobotCollder)
    {
        UpcastRobotCollider(mBodyA)
            ->mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                UpcastRobotCollider(mBodyA)->mLinkId,
                mContactPosOnA.segment(0, 3), jac0);
    }
    if (mBodyB->GetType() == eColObjType::RobotCollder)
    {

        UpcastRobotCollider(mBodyB)
            ->mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                UpcastRobotCollider(mBodyB)->mLinkId,
                mContactPosOnB.segment(0, 3), jac1);
    }

    // for self collision, only reserve a diff at jac0
    if (mIsMBSelfCollision == true)
    {
        jac0 = jac0 - jac1;
        jac1.resize(0, 0);
    }
}

void btGenContactPairData::Init(double dt_, btPersistentManifold *manifold,
                                int contact_id_in_manifold)
{
    dt = dt_;
    mBodyA = UpcastColObj(manifold->getBody0());
    mBodyB = UpcastColObj(manifold->getBody1());
    mTypeA = mBodyA->GetType();
    mTypeB = mBodyB->GetType();

    if (mBodyA == nullptr || mBodyB == nullptr)
    {
        std::cout << "illegal body type!\n";
        exit(0);
    }

    const auto &pt = manifold->getContactPoint(contact_id_in_manifold);
    mNormalPointToA = btBulletUtil::btVectorTotVector0(pt.m_normalWorldOnB);
    mContactPosOnA = btBulletUtil::btVectorTotVector1(pt.getPositionWorldOnA());
    mContactPosOnB = btBulletUtil::btVectorTotVector1(pt.getPositionWorldOnB());
    distance = pt.getDistance();

    // judge if it is self collsiion (only for multibody)
    if (eColObjType::RobotCollder == mBodyA->GetType() &&
        eColObjType::RobotCollder == mBodyB->GetType())
    {
        auto mb_A = dynamic_cast<btGenRobotCollider *>(mBodyA);
        auto mb_B = dynamic_cast<btGenRobotCollider *>(mBodyB);
        if (mb_A->mModel == mb_B->mModel)
            mIsMBSelfCollision = true;
    }
}

void btGenJointLimitData::ApplyJointTorque(double val)
{
    tVector3d torque = joint_direction * val;

    // when this joint limit is the upper bound, the input of torque is -tau
    // which is positive and the output of it is -qdot which should be positive,
    // aka qdot should be negative, q should be descrese
    if (is_upper_bound)
    {
        // std::cout << "[joint limit] it is upperbound, so torque *=-1, raw val
        // = " << torque.transpose() << ", after val = " << torque.transpose() *
        // -1 << std::endl;

        torque *= -1;
    }

    // std::cout << "[joint limit] add torque " << torque.transpose() <<
    // std::endl;
    multibody->ApplyJointTorque(multibody->GetJointByDofId(dof_id)->GetId(),
                                btMathUtil::Expand(torque, 0));
}

void btGenJointLimitData::ApplyGeneralizedForce(double val)
{
    if (is_upper_bound)
        val *= -1;
    multibody->ApplyGeneralizedForce(dof_id, val);
}

/**
 * \brief			calculate the final convert matrix H and vector
 * h
 *
 */
void btGenContactPairData::Setup(int num_frictions)
{
    mNumOfFriction = num_frictions;

    // 4. calc body info
    CalcBodyInfo();

    // friction first
    CalcFrictionCone(mD);

    // then convert mat second (it relies on the result of CalcFrictionCone)
    CalcConvertMat(mS);

    // 3. Hn, hn and Ht, ht
    // this->CalcJacAndRes_n();
    // this->CalcJacAndRes_t(num_frictions);

    // std::cout << "----set up ----------\n";
    // std::cout << "mHn shape = " << mHn.rows() << " " << mHn.cols() <<
    // std::endl; std::cout << "mhn shape = " << mhn.rows() << " " << mhn.cols()
    // << std::endl; std::cout << "mHt shape = " << mHt.rows() << " " <<
    // mHt.cols() << std::endl; std::cout << "mht shape = " << mht.rows() << " "
    // << mht.cols() << std::endl;
}

void btGenContactPairData::ApplyForceResultVector(const tVectorXd &x0)
{
    ApplyForceCartersian(btMathUtil::Expand(mS * x0, 0));
}

void btGenContactPairData::ApplyForceCartersian(const tVector &contact_force)
{
    // std::cout <<"body A apply " << contact_force.transpose() << " on " <<
    // mContactPosOnA.transpose() << std::endl; std::cout <<"body B apply " <<
    // -contact_force.transpose() << " on " << mContactPosOnB.transpose() <<
    // std::endl;
    mBodyA->ApplyForce(contact_force, mContactPosOnA);
    mBodyB->ApplyForce(-contact_force, mContactPosOnB);
}

bool btGenContactPairData::CheckOverlap(btGenContactPairData *other_data)
{
    // bool overlap = false;
    // for (int i = 0; i < 2; i++)
    // 	for (int j = 0; j < 2; j++)
    // 	{
    // 		int my_self_id = i == 0 ? mId0 : mId1,
    // 			other_id = j == 0 ? other_data->mId0 : other_data->mId1;

    // 		if (my_self_id == other_id) overlap = true;
    // 	}
    return (other_data->mBodyId0 == mBodyId0) ||
           (other_data->mBodyId1 == mBodyId0) ||
           (other_data->mBodyId0 == mBodyId1) ||
           (other_data->mBodyId1 == mBodyId1);
}

tMatrixXd btGenContactPairData::GetConvertMatS(int world_col_id)
{
    if (world_col_id == mBodyId0)
        return mS;
    else if (world_col_id == mBodyId1)
        return -mS;
    else
    {
        std::cout << "get S for world id " << world_col_id
                  << " but here are only " << mBodyId0 << " and " << mBodyId1
                  << std::endl;
        exit(0);
    }
}

int btGenContactPairData::GetBody0Id() { return mBodyId0; }
int btGenContactPairData::GetBody1Id() { return mBodyId1; }

tVector btGenContactPairData::GetVelOnBody0()
{
    return btMathUtil::Expand(mBodyA->GetVelocityOnPoint(mContactPosOnA), 0);
}

tVector btGenContactPairData::GetVelOnBody1()
{
    return btMathUtil::Expand(mBodyB->GetVelocityOnPoint(mContactPosOnB), 0);
}

tVector btGenContactPairData::GetRelVel()
{
    tVector body0_vel = GetVelOnBody0(), body1_vel = GetVelOnBody1();

    // std::cout << "Get rel vel, vel0 = " << body0_vel.transpose() <<
    // std::endl; std::cout << "Get rel vel, vel1 = " << body1_vel.transpose()
    // << std::endl;
    return body0_vel - body1_vel;
}
void btGenContactPairData::UpdateVel(double dt)
{
    mBodyA->UpdateVelocity(dt);
    mBodyB->UpdateVelocity(dt);
}

void btGenContactPairData::CalcBodyInfo()
{
    // change type
    mBodyId0 = mBodyA->getWorldArrayIndex();
    mBodyId1 = mBodyB->getWorldArrayIndex();
}

// D is 3 x N. each col vector is a friction direction
void btGenContactPairData::CalcFrictionCone(tMatrixXd &D)
{
    D.resize(3, mNumOfFriction), D.setZero();
    D = btMathUtil::ExpandFrictionCone(mNumOfFriction, mNormalPointToA)
            .transpose()
            .block(0, 0, 3, mNumOfFriction);
    // std::cout <<"normal = " << mNormalPointToA.transpose() << std::endl;
    // std::cout <<"mD = \n" << mD << std::endl;
}

// mat S can convert a result vector x_0 (shape N+2) to a force in cartesian
// space f_0 3x1
void btGenContactPairData::CalcConvertMat(tMatrixXd &S)
{
    int x_single_size =
        mNumOfFriction +
        2; // (f_n + f_\parallel + \lambda).shape = NumOfFriction + 2
    S.resize(3, x_single_size), S.setZero();

    S.col(0) = mNormalPointToA.segment(0, 3);
    S.block(0, 1, 3, mNumOfFriction) = mD;
}

//---------------------------------------------------------------------------

btGenCollisionGroupData::btGenCollisionGroupData(
    int g_id, btGenCollisionObject *obj,
    bool record_cartesian_convert_mat_of_residual)
    : mEnableRecordCartesianConvertMatOfResidual(
          record_cartesian_convert_mat_of_residual),
      mGroupId(g_id)
{
    mBody = obj;
    Clear();
}

void btGenCollisionGroupData::Clear()
{
    mJointLimits.clear();
    mContactPairs.clear();
    mIsBody0.clear();
    mCharacterConvertMatOfResidualRecord.clear();

    num_contact_pairs_ingroup = 0;
    num_self_contact_pairs_ingroup = 0;
    num_nonself_contact_pairs_ingroup = 0;
    num_joint_constraint_ingroup = 0;
    // map_world_contact_id_to_local_contact_id.clear();
}

void btGenCollisionGroupData::AddContactPair(btGenContactPairData *data,
                                             bool isbody0)
{
    // std::cout << "contact " << data->contact_id << " add for body " <<
    // mBody->GetName() <<" isbody0 = " << isbody0 << std::endl;
    mContactPairs.push_back(data);
    mIsBody0.push_back(isbody0);
}

void btGenCollisionGroupData::AddJointLimitConstraint(btGenJointLimitData *data)
{
    mJointLimits.push_back(data);
}

/**
 * \brief			Setup all contact points for this collision
 * group. Note that, a collision group is the minimun group where contact
 * points' jacobian are calculated. A rigidbody = a collision group A multibody
 * = a collision group A link in multibody should belong to but != a colision
 * group
 */
// extern std::string gOutputLogPath;
void btGenCollisionGroupData::Setup(int num_world_contact_pairs_,
                                    int num_world_joint_limits_)
{
    // 1. get the world constraint info
    num_world_contact_pairs = num_world_contact_pairs_;
    num_world_joint_limits = num_world_joint_limits_;
    num_world_constraints = num_world_contact_pairs + num_world_joint_limits;

    int cartesian_sol_length =
        num_world_contact_pairs * 3 + num_world_joint_limits;

    // 2. allocate the convert mat & res in world size
    // TODO: optimize these mats to local storages in this group, for multiple characters case
    mConvertCartesianForceToVelocityMat.resize(cartesian_sol_length,
                                               cartesian_sol_length),
        mConvertCartesianForceToVelocityMat.setZero();
    mConvertCartesianForceToVelocityVec.resize(cartesian_sol_length),
        mConvertCartesianForceToVelocityVec.setZero();

    // 3. begin to setup these mat & vec
    if (mBody->IsStatic() == true)
        return;
    switch (mBody->GetType())
    {
    case eColObjType::Rigidbody:
        SetupRigidBody();
        break;
    case eColObjType::RobotCollder:
        SetupRobotCollider();
        break;
    default:
        std::cout << "wrong type\n";
        exit(0);
    }
    BTGEN_ASSERT(mConvertCartesianForceToVelocityMat.hasNaN() == false);
    BTGEN_ASSERT(mConvertCartesianForceToVelocityVec.hasNaN() == false);
    // std::ofstream fout(gOutputLogPath, std::ios::app);
    // fout << "abs convert mat = \n"
    // 	 << mConvertCartesianForceToVelocityMat << std::endl;
    // fout << "abs convert vec = "
    // 	 << mConvertCartesianForceToVelocityVec.transpose() << std::endl;
    // fout.close();
}

template <typename T1, typename T2, typename T3>
void Solve(const T1 &solver, const T2 &mat, const T3 &residual, T3 &result,
           double &error)
{
    result = solver.solve(residual);
    error = (mat * result - residual).norm();
}

template <typename T1, typename T2>
T2 AccurateSolve(const T1 &mat, const T2 &residual, double &min_error)
{
    // 1. FullPivLU
    T2 final_result, my_result;
    min_error = 1e4;
    double my_error;
    std::string name;
    {
        Solve(Eigen::FullPivLU<T1>(mat), mat, residual, my_result, my_error);
        // std::cout << "FullPivLU error = " << my_error << std::endl;
        if (min_error > my_error)
        {
            name = "FullPivLU";
            final_result = my_result;
            min_error = my_error;
        }
    }

    // 2. ComPivHouseHolderQR
    {
        Solve(Eigen::ColPivHouseholderQR<T1>(mat), mat, residual, my_result,
              my_error);
        // std::cout << "ColPivHouseholderQR error = " << my_error << std::endl;
        if (min_error > my_error)
        {
            name = "ColPivHouseholderQR";
            final_result = my_result;
            min_error = my_error;
        }
    }

    // 3. LDLT
    {
        Solve(Eigen::LDLT<T1>(mat), mat, residual, my_result, my_error);
        // std::cout << "LDLT error = " << my_error << std::endl;
        if (min_error > my_error)
        {
            name = "LDLT";
            final_result = my_result;
            min_error = my_error;
        }
    }
    // 4. LLT
    {
        Solve(Eigen::LLT<T1>(mat), mat, residual, my_result, my_error);
        // std::cout << "LLT error = " << my_error << std::endl;
        if (min_error > my_error)
        {
            name = "LLT";
            final_result = my_result;
            min_error = my_error;
        }
    }
    // 5. FullPivHouseholderQR
    {
        Solve(Eigen::FullPivHouseholderQR<T1>(mat), mat, residual, my_result,
              my_error);
        // std::cout << "FullPivHouseholderQR error = " << my_error <<
        // std::endl;
        if (min_error > my_error)
        {
            name = "FullPivHouseholderQR";
            final_result = my_result;
            min_error = my_error;
        }
    }
    // 6. BDCSVD
    {
        Solve(Eigen::BDCSVD<T1>(mat, Eigen::ComputeThinU | Eigen::ComputeThinV),
              mat, residual, my_result, my_error);
        // std::cout << "BDCSVD error = " << my_error << std::endl;
        if (min_error > my_error)
        {
            name = "BDCSVD";
            final_result = my_result;
            min_error = my_error;
        }
    }
    // 7. JacobiSVD
    {
        Solve(Eigen::JacobiSVD<T1>(mat,
                                   Eigen::ComputeThinU | Eigen::ComputeThinV),
              mat, residual, my_result, my_error);
        // std::cout << "JacobiSVD error = " << my_error << std::endl;
        if (min_error > my_error)
        {
            name = "JacobiSVD";
            final_result = my_result;
            min_error = my_error;
        }
    }

    std::cout << "here we use " << name << " method, error = " << min_error
              << std::endl;
    return final_result;
}

extern bool gEnablePauseWhenSolveError;
extern bool gEnableResolveWhenSolveError;

/**
 * \brief           Begin to calculate the convert matrix from contact force to the contact point's abs vel in next frame
 * 
 * Note that, in different integration scheme, the formula & performance of this function would be different
*/
void btGenCollisionGroupData::SetupRobotCollider()
{
    // std::cout << "----------set up robot collider begin----------\n";
    // 1. calculate some essential statistics
    SetupRobotColliderVars();
    // if (num_joint_constraint_ingroup)
    // {
    //     num_joint_constraint_ingroup = num_joint_constraint_ingroup;
    // }
    int num_constraints_ingroup =
        num_contact_pairs_ingroup + num_joint_constraint_ingroup;
    if (num_constraints_ingroup == 0)
        return;

    btGenRobotCollider *collider = UpcastRobotCollider(mBody);
    cRobotModelDynamics *model = collider->mModel;
    int n_dof = model->GetNumOfFreedom();
    double dt = 0;
    if (num_contact_pairs_ingroup)
        dt = mContactPairs[0]->dt;
    else
        dt = mJointLimits[0]->dt;

    // 1.1 calcualte the contact jacobian
    // tEigenArr<tMatrixXd> nonself_contact_pair_jac(
    //     num_nonself_contact_pairs_ingroup, tMatrixXd::Zero(3, n_dof));
    // tEigenArr<tMatrixXd> self_contact_pair_jac_diff(
    //     num_self_contact_pairs_ingroup, tMatrixXd::Zero(3, n_dof));

    // GetContactJacobianArrayRobotCollider(nonself_contact_pair_jac,
    //                                      self_contact_pair_jac_diff);

    // 1.2 calculate the joint limit jacobian
    // convert the torque magnitute(can be negative for upper bound) to the
    // generalized force

    // tEigenArr<tMatrixXd> joint_limit_selection_jac(num_joint_constraint_ingroup,
    //                                                tMatrixXd::Zero(1, n_dof));
    // GetJointLimitJaocibanArrayRobotCollider(joint_limit_selection_jac);

    M = model->GetMassMatrix();
    inv_M = model->GetInvMassMatrix();
    coriolis_mat = model->GetCoriolisMatrix();
    damping_mat = model->GetDampingMatrix();
    M_dt_C_D_inv = (M + dt * (coriolis_mat + damping_mat)).inverse();
    if (M_dt_C_D_inv.hasNaN() == true)
    {
        std::cout << "dt = " << dt << std::endl;
        std::cout << "M = \n" << M << std::endl;
        std::cout << "C = \n" << coriolis_mat << std::endl;
        std::cout << "D = \n" << damping_mat << std::endl;
        std::cout << "D+dtCd = \n"
                  << M + dt * (coriolis_mat + damping_mat) << std::endl;
        BTGEN_ASSERT(false);
    }
    BTGEN_ASSERT(inv_M.hasNaN() == false);
    const tMatrixXd &I = tMatrixXd::Identity(n_dof, n_dof);

    // residual part: the Q and qdot_next has a linear relationship, qdot_next = A * Q + b. b is the residual here
    tVectorXd residual_part = CalcRobotColliderResidual(dt);
    tMatrixXd A_middle_part = CalcRobotColliderMiddlePart(dt);
    // 2. begin to form the ultimate velocity convert mat, handle the abs vel convert mat
    tMatrixXd Jac_partA;
    int row_st = 0, row_size = 0;
    int col_st = 0, col_size = 0;

    // for each (row of) constraint data in this group
    // tEigenArr<tMatrixXd> Jac_partB_lst(num_constraints_ingroup);

    for (int row_cons_id_ingroup = 0;
         row_cons_id_ingroup < num_constraints_ingroup; row_cons_id_ingroup++)
    {
        // 2.1 select the constraint data for this row
        // and the target row "row_st", "row_size" in abs vel convert mat of this constraint
        GetDataInfoRobotCollider(row_cons_id_ingroup, row_st, row_size);

        /*
            2.1.3 
            get the jac_partA : convert qdot_delta -> rel_vel of this row
            get the residual_a: convert qdot_cur -> rel_vel of this row
        */
        Jac_partA =
            GetAbsVelConvertMat_JacPartA_RobotCollider(row_cons_id_ingroup);
        mConvertCartesianForceToVelocityVec.segment(row_st, row_size)
            .noalias() = Jac_partA * residual_part;
        // record residual_a (residual convert mat from gen to cartesian)
        if (mEnableRecordCartesianConvertMatOfResidual == true)
        {
            mCharacterConvertMatOfResidualRecord.push_back(Jac_partA);
        }
        // 2.2 Jac_partA = i_contact_jacobian * middle_part.
        /*
            middle_part = dt * Minv in old-semi-implicit 
                        = others in new-semi-implicit
        */
        Jac_partA *= A_middle_part;

        /*
            2.3
                calculate the EFFECT of all constraints in the current group 
                    on the current ROW constraint

                Note that the absolute velocity at ANY contact point
                 is affected by ALL contact forces in the group
        */

        for (int col_cons_id = 0; col_cons_id < num_constraints_ingroup;
             col_cons_id++)
        {
            // fetch the data by contact point id
            GetDataInfoRobotCollider(col_cons_id, col_st, col_size);

            // 2.3 final part = Jac_partA * Jac_partB
            mConvertCartesianForceToVelocityMat
                .block(row_st, col_st, row_size, col_size)
                .noalias() =
                Jac_partA * CalcRobotColliderJacPartBPrefix(dt) *
                GetAbsVelConvertMat_JacPartB_RobotCollider(col_cons_id);
            BTGEN_ASSERT(mConvertCartesianForceToVelocityMat.hasNaN() == false);
            // BTGEN_ASSERT(Jac_partA.hasNaN() == false);
            // BTGEN_ASSERT(Jac_partB.hasNaN() == false);
        }
    }
}

void btGenCollisionGroupData::SetupRigidBody()
{
    // 1. create buffer
    int n_my_contact = mContactPairs.size();
    if (n_my_contact == 0)
        return;
    tEigenArr<tMatrix> RelPosSkew(n_my_contact);
    btGenRigidBody *rigidbody = UpcastRigidBody(mBody);
    tVector angvel = rigidbody->GetAngVel();
    tVector world_pos = rigidbody->GetWorldPos();
    tMatrix inertia = rigidbody->GetInertia(),
            inv_inertia = rigidbody->GetInvInertia();
    double dt = mContactPairs[0]->dt;
    double invmass = rigidbody->GetInvMass();
    tVector total_force = rigidbody->GetTotalForce(),
            total_torque = rigidbody->GetTotalTorque();
    for (int i = 0; i < n_my_contact; i++)
    {
        btGenContactPairData *data = mContactPairs[i];
        int contact_id = mContactPairs[i]->constraint_id;

        tVector contact_pt =
            mIsBody0[i] == true ? data->mContactPosOnA : data->mContactPosOnB;
        RelPosSkew[i] = btMathUtil::VectorToSkewMat(contact_pt - world_pos);
        // std::cout << " body " << mBody->GetName() << " contact " << i << "
        // isbody0 = " << mIsBody0[i] << std::endl;
    }

    // 2. begin to form the ultimate mat
    for (int i = 0; i < n_my_contact; i++)
    {
        int i_id = mContactPairs[i]->constraint_id;
        // form the mat
        for (int j = 0; j < n_my_contact; j++)
        {
            int j_id = mContactPairs[j]->constraint_id;
            int symbol = mIsBody0[j] == false ? -1 : 1;
            mConvertCartesianForceToVelocityMat.block(i_id * 3, j_id * 3, 3,
                                                      3) =
                symbol * dt *
                (invmass * tMatrix3d::Identity() -
                 RelPosSkew[i].block(0, 0, 3, 3) *
                     inv_inertia.block(0, 0, 3, 3) *
                     RelPosSkew[j].block(0, 0, 3, 3));
        }

        // form the residual
        /*
            vt
            + 
            dt * inv_m * total_force
            -[r] * dt * Inertia_inv * (total_torque - [w]*(Inertia * w))
             - [r] * w
        */
        mConvertCartesianForceToVelocityVec.segment(i_id * 3, 3) =
            (rigidbody->GetLinVel() + dt * invmass * total_force -
             RelPosSkew[i] * dt * inv_inertia *
                 (total_torque -
                  btMathUtil::VectorToSkewMat(angvel) * (inertia * angvel)) -
             RelPosSkew[i] * angvel)
                .segment(0, 3);
    }
}

void btGenCollisionGroupData::SetupRobotColliderVars()
{
    num_contact_pairs_ingroup = mContactPairs.size();
    num_joint_constraint_ingroup = mJointLimits.size();
}

/**
 * \brief           Calculate the middle part matrix in the linear relationship between Q and abs_v_next of contact points
 * 
 * In old-semi-implicit integration, the middle part is 
 *          dt * Minv
 * In new-semi-implicit integration, the middle part is 
 *          dt * (M + dt * (C + D)).inverse()
*/
tMatrixXd btGenCollisionGroupData::CalcRobotColliderMiddlePart(double dt) const
{
    cRobotModelDynamics *model =
        static_cast<btGenRobotCollider *>(mBody)->mModel;
    BTGEN_ASSERT(model != nullptr);
    switch (model->GetIntegrationScheme())
    {
    case btGeneralizeWorld::eIntegrationScheme::OLD_SEMI_IMPLICIT_SCHEME:
    {
        return dt * inv_M;
        break;
    }
    case btGeneralizeWorld::eIntegrationScheme::NEW_SEMI_IMPLICIT_SCHEME:
    {
        return dt * M_dt_C_D_inv;
        break;
    }

    default:
        BTGEN_ASSERT(false);
        return tMatrixXd::Zero(0, 0);
        break;
    }
}
/**
 * \brief					Calculate the robot collider residual used in
 * the calculation of absolute velocity convert matrix
 * 
 * The residual in the formula means: qdot_{t+1} = Mat * Q + residual
 * It is the residual in the linear relationship between gen force Q and residual
 * 
 */
#include "BulletGenDynamics/btGenWorld.h"
tVectorXd btGenCollisionGroupData::CalcRobotColliderResidual(double dt) const
{
    tVectorXd residual;
    if (eColObjType::RobotCollder != mBody->GetType())
    {
        std::cout << "[error] CalcRobotColliderResidual: can only be used for "
                     "robot collider, exit\n";
        exit(0);
    }
    else
    {
        cRobotModelDynamics *model =
            static_cast<btGenRobotCollider *>(mBody)->mModel;
        BTGEN_ASSERT(model != nullptr);
        switch (model->GetIntegrationScheme())
        {
        case btGeneralizeWorld::eIntegrationScheme::OLD_SEMI_IMPLICIT_SCHEME:
        {
            residual = CalcRobotColliderResidualOldSemiImplicit(dt);
            break;
        }
        case btGeneralizeWorld::eIntegrationScheme::NEW_SEMI_IMPLICIT_SCHEME:
        {
            residual = CalcRobotColliderResidualNewSemiImplicit(dt);
            break;
        }
        default:
            BTGEN_ASSERT(false);
            break;
        }
    }
    return residual;
}

/**
 * 
 *  \brief          Calculate the convert formula residual from Q to qdot_next
 * 
 *  In old-semi-implicit integraion scheme
 *       If the contact-aware LCP is disabled, then the residual is simple
 *       		= dt * M^{-1} * (Q_applied - C\dot{q}_t)　+ \dot{q}_t
*
 *       But if the contact-aware LCP is enable, the residual is converted
 *       		= dt * M^{-1} *
 *       			(	\tilde{\tau} - C \tilde{B}\tilde{\chi}
 *       				+ Q_applied　- C\dot{q}_t)
 *       			+ \dot{q}_t
*/
tVectorXd btGenCollisionGroupData::CalcRobotColliderResidualOldSemiImplicit(
    double dt) const
{
    tVectorXd residual;
    cRobotModelDynamics *model =
        static_cast<btGenRobotCollider *>(mBody)->mModel;
    int n_dof = model->GetNumOfFreedom();
    const tMatrixXd I = tMatrixXd::Identity(n_dof, n_dof);

    if (false == model->GetEnableContactAwareController())
    {
        // std::cout << "contact aware controller is disabled\n";
        residual =
            (I - dt * inv_M * (coriolis_mat + damping_mat)) * model->Getqdot() +
            dt * inv_M * model->GetGeneralizedForce();
    }
    else
    {
        btGenContactAwareController *controller =
            model->GetContactAwareController();
        if (damping_mat.norm() > 0)
            std::cout << "[error] contact aware control didn't support "
                         "damping cuz the absence of formulas\n",
                exit(0);
        residual = controller->CalcLCPResidual(dt);
    }
    return residual;
}

/**
 *  \brief          Calculate the convert formula residual from Q to qdot_next
 * 
 *  In new-semi-implicit integraion scheme
 *      If the contact aware LCP is disabled, the residual is:
 *              = (M + dt * (C + D)).inv() * (M * qdot + dt * Q)
 *      If the contact awrae LCP is enabled, the reisudal is:P
 *              HASN'T BEEN IMPLEMENTED
*/
tVectorXd btGenCollisionGroupData::CalcRobotColliderResidualNewSemiImplicit(
    double dt) const
{
    tVectorXd residual;
    cRobotModelDynamics *model =
        static_cast<btGenRobotCollider *>(mBody)->mModel;
    int n_dof = model->GetNumOfFreedom();
    const tMatrixXd I = tMatrixXd::Identity(n_dof, n_dof);

    if (false == model->GetEnableContactAwareController())
    {
        residual.noalias() = M_dt_C_D_inv * (M * model->Getqdot() +
                                             dt * model->GetGeneralizedForce());
    }
    else
    {
        // new semi implicit in contact awrae controlelr hasn't been implemented
        BTGEN_ASSERT(false);
    }
    return residual;
}

tMatrixXd
btGenCollisionGroupData::CalcRobotColliderJacPartBPrefix(double dt) const
{
    tMatrixXd prefix;
    if (eColObjType::RobotCollder != mBody->GetType())
    {
        std::cout << "[error] CalcRobotColliderJacPartB: can only be used for "
                     "robot collider, exit\n";
        exit(0);
    }
    else
    {
        cRobotModelDynamics *model =
            static_cast<btGenRobotCollider *>(mBody)->mModel;
        if (model == nullptr)
            std::cout
                << "[error] CalcRobotColliderJacPartB: model is empty, exit",
                exit(0);
        int n_dof = model->GetNumOfFreedom();
        const tMatrixXd I = tMatrixXd::Identity(n_dof, n_dof);

        if (false == model->GetEnableContactAwareController())
        {
            // std::cout << "contact aware controller is disabled\n";
            prefix.noalias() = tMatrixXd::Identity(n_dof, n_dof);
        }
        else
        {
            btGenContactAwareController *controller =
                model->GetContactAwareController();
            // tMatrixXd E = controller->GetE(), N = controller->GetN();
            // std::cout << "[warn] contact aware controller is enabled, but the
            // code hasn't been revised\n";
            prefix.noalias() = controller->CalcLCPPartBPrefix();
        }
    }
    return prefix;
}

/**
 * \brief       get method
*/
void btGenCollisionGroupData::GetCharacterConvertMatOfResidualRecord(
    tEigenArr<tMatrixXd> &record) const
{
    BTGEN_ASSERT(mEnableRecordCartesianConvertMatOfResidual);
    record = mCharacterConvertMatOfResidualRecord;
}

/**
 * \brief       print debug info of this group
*/
void btGenCollisionGroupData::PrintDebugInfo() const
{
    std::cout << "----------\n";
    printf("[debug] group_id %d, contact pairs %d joint limits %d\n", mGroupId,
           mContactPairs.size(), mJointLimits.size());
    std::cout << "isbody0 = ";
    for (auto b : mIsBody0)
        std::cout << b << " ";
    std::cout << std::endl;
    std::cout << "abs mat norm = " << mConvertCartesianForceToVelocityMat.norm()
              << std::endl;
    std::cout << "abs mat norm = " << mConvertCartesianForceToVelocityVec.norm()
              << std::endl;
    for (int i = 0; i < mContactPairs.size(); i++)
    {
        {
            auto bodya = mContactPairs[i]->mBodyA;
            if (UpcastColObj(bodya) == nullptr)
            {
                std::cout << "[debug] bodyB cannot be recognized\n";
            }
            else
            {
                BTGEN_ASSERT(bodya != nullptr);
                printf("[debug] pair %d bodyA type %d\n", i, bodya->GetType());
                if (bodya->GetType() == eColObjType::RobotCollder)
                {
                    std::cout
                        << "a link id = " << UpcastRobotCollider(bodya)->mLinkId
                        << std::endl;
                }
                std::cout << "a world trans = \n"
                          << btBulletUtil::btTransformTotMatrix(
                                 bodya->getWorldTransform())
                          << std::endl;
            }
        }
        {
            auto bodyb = mContactPairs[i]->mBodyB;
            if (UpcastColObj(bodyb) == nullptr)
            {
                std::cout << "[debug] bodyB cannot be recognized\n";
            }
            else
            {
                BTGEN_ASSERT(bodyb != nullptr);
                printf("[debug] pair %d bodyB type %d\n", i, bodyb->GetType());
                if (bodyb->GetType() == eColObjType::RobotCollder)
                {
                    std::cout
                        << "b link id = " << UpcastRobotCollider(bodyb)->mLinkId
                        << std::endl;
                }
                std::cout << "b world trans = \n"
                          << btBulletUtil::btTransformTotMatrix(
                                 bodyb->getWorldTransform())
                          << std::endl;
            }
        }
    }
    std::cout << "----------\n";
}
/**
 * \brief           Given the constraint_id_ingroup, do
 *      calculate the occupied size & bias of this constraint, in world row convert mat
*/
void btGenCollisionGroupData::GetDataInfoRobotCollider(
    int constraint_id_ingroup, int &constraint_st_pos_world,
    int &constraint_size_world) const
{
    // it is a contact pair constraint
    if (constraint_id_ingroup < num_contact_pairs_ingroup)
    {
        // contact cartesian force, size = 3, bias = N * 3
        constraint_st_pos_world =
            mContactPairs[constraint_id_ingroup]->constraint_id * 3;
        constraint_size_world = 3;
    }
    else
    {
        // it is a joint limit constraint
        int joint_limit_id_in_group =
            constraint_id_ingroup - num_contact_pairs_ingroup;

        // all joint limit is behind the contact constraints
        // size = 1
        constraint_st_pos_world = num_world_contact_pairs * 3;

        constraint_st_pos_world +=
            mJointLimits[joint_limit_id_in_group]->constraint_id -
            num_world_contact_pairs;
        constraint_size_world = 1;
    }
}

/**
 * \brief           Given the constraint id (in group), the partA jacobain in abs vel
 * 
 *          the Jac_partA will do the convertion: 
 *              for non-self collision and joint_limit:  qdot_delta -> abs_cartesian_vel
 *              for self collsion : qdot_delta -> rel_cartesian_vel
 * \param cons_id_ingroup       constraint id (in group, not world)
*/
tMatrixXd btGenCollisionGroupData::GetAbsVelConvertMat_JacPartA_RobotCollider(
    int cons_id_ingroup) const
{
    tMatrixXd Jac_partA;
    btGenLCPConstraintData *cons_data = nullptr;

    // 1. get the constraint pointer from the given local id
    if (cons_id_ingroup < num_contact_pairs_ingroup)
        cons_data = mContactPairs[cons_id_ingroup];
    else
        cons_data = mJointLimits[cons_id_ingroup - num_contact_pairs_ingroup];

    // 2. if this is a contact constraint, calulate the jac
    if (cons_data->mConstraintType ==
        btGenLCPConstraintType::BTGEN_LCP_CONTACT_PAIR_CONS)
    {
        auto contact_data = static_cast<btGenContactPairData *>(cons_data);
        // if current cosntraint is CONTACT POINT
        if (contact_data->mIsMBSelfCollision == false)
        {
            // 2.1.3.1 for nonself contact pair constraint row

            // not self collsion, then at least one col_obj is not in this group
            if (mGroupId == contact_data->mbody0GroupId)
            {
                // body0 is in this group, so we may take jac0
                BTGEN_ASSERT(mGroupId != contact_data->mbody1GroupId);
                Jac_partA.noalias() = contact_data->jac0;
            }
            else
            {
                // body1 is in this group,
                Jac_partA.noalias() = contact_data->jac1;
            }
        }
        else
        {
            // 2.1.3.2 for self contact pair constraint row
            Jac_partA.noalias() = cons_data->jac0;
        }
    }
    else if (cons_data->mConstraintType ==
             btGenLCPConstraintType::BTGEN_LCP_JOINT_LIMIT_CONS)
    {
        // 2.1.3.3 for joint limit
        assert(cons_data != nullptr);

        // set Jac part A
        Jac_partA.noalias() = cons_data->jac0;
    }
    else
    {
        BTGEN_ASSERT(false);
    }
    return Jac_partA;
}

/**
 * \brief                   Given the constraint id, calculate the Jac partB
 * \param cons_id_ingroup   the local constraint id in group
 *          
 *  This Jac_partB mat will do the convertion:
 *          for non-self collision & joint limit: cartesian_contact_force_tobody0 -> Generalized force
 *          for self collision : cartesian_contact_force_tobody0 -> Generalized force diff 
*/
tMatrixXd btGenCollisionGroupData::GetAbsVelConvertMat_JacPartB_RobotCollider(
    int cons_id_ingroup) const
{
    tMatrixXd Jac_partB;
    btGenLCPConstraintData *cons_data = nullptr;

    // 1. get the constraint pointer from the given local id
    if (cons_id_ingroup < num_contact_pairs_ingroup)
        cons_data = mContactPairs[cons_id_ingroup];
    else
        cons_data = mJointLimits[cons_id_ingroup - num_contact_pairs_ingroup];

    if (cons_data->mConstraintType ==
        btGenLCPConstraintType::BTGEN_LCP_CONTACT_PAIR_CONS)
    {
        auto contact_point_data =
            static_cast<btGenContactPairData *>(cons_data);
        // jth constraint is a contact point constraint
        if (contact_point_data->mIsMBSelfCollision == false)
        {
            if (mGroupId == contact_point_data->mbody0GroupId)
            {
                // body0 is in group
                Jac_partB.noalias() = contact_point_data->jac0.transpose();
            }
            else
            {
                // body1 is in this group
                Jac_partB.noalias() = -contact_point_data->jac1.transpose();
            }
        }
        else
        {
            // for self collsiion

            Jac_partB.noalias() = contact_point_data->jac0.transpose();
        }
    }
    else if (cons_data->mConstraintType ==
             btGenLCPConstraintType::BTGEN_LCP_JOINT_LIMIT_CONS)
    {
        // jth constraint is a joint limit constraint
        Jac_partB.noalias() = cons_data->jac0.transpose();
    }
    else
    {
        BTGEN_ASSERT(false);
    }
    return Jac_partB;
}