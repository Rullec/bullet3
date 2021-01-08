#include "ConstraintData.h"
#include "BulletGenDynamics/btGenController/ContactAwareController/btGenContactAwareController.h"
#include "BulletGenDynamics/btGenModel/RobotCollider.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenModel/SimObj.h"
#include <iostream>

extern btGenRigidBody *UpcastRigidBody(const btCollisionObject *col);
extern btGenRobotCollider *UpcastRobotCollider(const btCollisionObject *col);
extern btGenCollisionObject *UpcastColObj(const btCollisionObject *col);

btGenJointLimitData::btGenJointLimitData(int constraint_id_,
                                         cRobotModelDynamics *multibody_,
                                         int dof_id_, bool is_upper_bound_,
                                         double violate_value_)
{
    constraint_id = constraint_id_;
    multibody = multibody_;
    dof_id = dof_id_;
    is_upper_bound = is_upper_bound_;
    violate_value = violate_value_;
    joint_direction = multibody->GetFreedomDirectionWorldFrame(dof_id);
    // if (is_upper_bound) joint_direction *= -1;
}

btGenContactPointData::btGenContactPointData(int c_id)
{
    contact_id = c_id;
    mbody0GroupId = -1;
    mbody1GroupId = -2;
    mIsSelfCollision = false;
}

void btGenContactPointData::Init(double dt_, btPersistentManifold *manifold,
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
    mContactPtOnA = btBulletUtil::btVectorTotVector1(pt.getPositionWorldOnA());
    mContactPtOnB = btBulletUtil::btVectorTotVector1(pt.getPositionWorldOnB());
    distance = pt.getDistance();
    // std::cout << "[debug] viodistance = " << distance << std::endl;
    // std::cout << "pt A = " << mContactPtOnA.transpose() << std::endl;
    // std::cout << "pt B = " << mContactPtOnB.transpose() << std::endl;
    // std::cout << "pt A-B = " << (mContactPtOnA - mContactPtOnB).norm()
    //           << std::endl;
    // double penetration = (mContactPtOnA - mContactPtOnB).dot(mNormalPointToA);
    // std::cout << "penetration = " << penetration << std::endl;
    // std::cout << "normal to A = " << mNormalPointToA.transpose() << std::endl;
    // judge if it is self collsiion (only for multibody)
    if (eColObjType::RobotCollder == mBodyA->GetType() &&
        eColObjType::RobotCollder == mBodyB->GetType())
    {
        auto mb_A = dynamic_cast<btGenRobotCollider *>(mBodyA);
        auto mb_B = dynamic_cast<btGenRobotCollider *>(mBodyB);
        if (mb_A->mModel == mb_B->mModel)
            mIsSelfCollision = true;
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
void btGenContactPointData::Setup(int num_frictions)
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

void btGenContactPointData::ApplyForceResultVector(const tVectorXd &x0)
{
    ApplyForceCartersian(btMathUtil::Expand(mS * x0, 0));
}

void btGenContactPointData::ApplyForceCartersian(const tVector &contact_force)
{
    // std::cout <<"body A apply " << contact_force.transpose() << " on " <<
    // mContactPtOnA.transpose() << std::endl; std::cout <<"body B apply " <<
    // -contact_force.transpose() << " on " << mContactPtOnB.transpose() <<
    // std::endl;
    mBodyA->ApplyForce(contact_force, mContactPtOnA);
    mBodyB->ApplyForce(-contact_force, mContactPtOnB);
}

bool btGenContactPointData::CheckOverlap(btGenContactPointData *other_data)
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

tMatrixXd btGenContactPointData::GetConvertMatS(int world_col_id)
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

int btGenContactPointData::GetBody0Id() { return mBodyId0; }
int btGenContactPointData::GetBody1Id() { return mBodyId1; }

tVector btGenContactPointData::GetVelOnBody0()
{
    return btMathUtil::Expand(mBodyA->GetVelocityOnPoint(mContactPtOnA), 0);
}

tVector btGenContactPointData::GetVelOnBody1()
{
    return btMathUtil::Expand(mBodyB->GetVelocityOnPoint(mContactPtOnB), 0);
}

tVector btGenContactPointData::GetRelVel()
{
    tVector body0_vel = GetVelOnBody0(), body1_vel = GetVelOnBody1();

    // std::cout << "Get rel vel, vel0 = " << body0_vel.transpose() <<
    // std::endl; std::cout << "Get rel vel, vel1 = " << body1_vel.transpose()
    // << std::endl;
    return body0_vel - body1_vel;
}
void btGenContactPointData::UpdateVel(double dt)
{
    mBodyA->UpdateVelocity(dt);
    mBodyB->UpdateVelocity(dt);
}

void btGenContactPointData::CalcBodyInfo()
{
    // change type
    mBodyId0 = mBodyA->getWorldArrayIndex();
    mBodyId1 = mBodyB->getWorldArrayIndex();
}

// D is 3 x N. each col vector is a friction direction
void btGenContactPointData::CalcFrictionCone(tMatrixXd &D)
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
void btGenContactPointData::CalcConvertMat(tMatrixXd &S)
{
    int x_single_size =
        mNumOfFriction +
        2; // (f_n + f_\parallel + \lambda).shape = NumOfFriction + 2
    S.resize(3, x_single_size), S.setZero();

    S.col(0) = mNormalPointToA.segment(0, 3);
    S.block(0, 1, 3, mNumOfFriction) = mD;
}

//---------------------------------------------------------------------------

btGenCollisionObjData::btGenCollisionObjData(
    btGenCollisionObject *obj, bool record_cartesian_convert_mat_of_residual)
    : mEnableRecordCartesianConvertMatOfResidual(
          record_cartesian_convert_mat_of_residual)
{
    mBody = obj;
    Clear();
}

void btGenCollisionObjData::Clear()
{
    mJointLimits.clear();
    mContactPts.clear();
    mIsBody0.clear();
    mCharacterConvertMatOfResidualRecord.clear();

    num_local_contacts = 0;
    num_self_contacts = 0;
    num_nonself_contacts = 0;
    num_joint_constraint = 0;
    map_world_contact_id_to_local_contact_id.clear();
}

void btGenCollisionObjData::AddContactPoint(btGenContactPointData *data,
                                            bool isbody0)
{
    // std::cout << "contact " << data->contact_id << " add for body " <<
    // mBody->GetName() <<" isbody0 = " << isbody0 << std::endl;
    mContactPts.push_back(data);
    mIsBody0.push_back(isbody0);
}

void btGenCollisionObjData::AddJointLimitConstraint(btGenJointLimitData *data)
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
#include <fstream>
// extern std::string gOutputLogPath;
void btGenCollisionObjData::Setup(int n_total_contact, int n_total_joint_limits)
{
    num_total_contacts = n_total_contact;
    num_total_joint_limits = n_total_joint_limits;
    num_total_constraints = num_total_contacts + num_total_joint_limits;

    int total_length = num_total_contacts * 3 + num_total_joint_limits;
    mConvertCartesianForceToVelocityMat.resize(total_length, total_length),
        mConvertCartesianForceToVelocityMat.setZero();
    mConvertCartesianForceToVelocityVec.resize(total_length),
        mConvertCartesianForceToVelocityVec.setZero();
    // std::cout << "my contact size = " << mContactPts.size() << std::endl;
    // for this collision obj, calculate the convert mat from full forces to
    // velcoity
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
void btGenCollisionObjData::SetupRobotCollider()
{
    // std::cout << "----------set up robot collider begin----------\n";
    // 1. calculate some essential statistics
    SetupRobotColliderVars();
    int num_constraints = num_local_contacts + num_joint_constraint;
    if (num_constraints == 0)
        return;

    btGenRobotCollider *collider = UpcastRobotCollider(mBody);
    cRobotModelDynamics *model = collider->mModel;
    int n_dof = model->GetNumOfFreedom();
    double dt = 0;
    if (num_local_contacts)
        dt = mContactPts[0]->dt;
    else
        dt = mJointLimits[0]->dt;

    // 1.1 calcualte the contact jacobian
    tEigenArr<tMatrixXd> nonself_contact_point_jac(
        num_nonself_contacts, tMatrixXd::Zero(3, model->GetNumOfFreedom()));
    tEigenArr<tMatrixXd> self_contact_point_jac(
        num_self_contacts, tMatrixXd::Zero(3, model->GetNumOfFreedom()));
    GetContactJacobianArrayRobotCollider(nonself_contact_point_jac,
                                         self_contact_point_jac);

    // 1.2 calculate the joint limit jacobian
    // convert the torque magnitute(can be negative for upper bound) to the
    // generalized force
    tEigenArr<tMatrixXd> joint_limit_selection_jac(num_joint_constraint,
                                                   tMatrixXd::Zero(1, n_dof));
    GetJointLimitJaocibanArrayRobotCollider(joint_limit_selection_jac);

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
    // 2. begin to form the ultimate velocity convert mat, handle the contact
    // point part
    tMatrixXd Jac_partA;
    tMatrixXd Jac_partB;
    int i_st = 0, i_size = 0;
    int j_st = 0, j_size = 0;
    for (int i_cons_id = 0; i_cons_id < num_constraints; i_cons_id++)
    {
        // 2.1 init the data and i global id
        btGenContactPointData *i_contact_point_data = nullptr;
        btGenJointLimitData *i_joint_limit_data = nullptr;
        int i_global_id = -1;

        // fetch the data by contact point id
        if (i_cons_id < num_local_contacts)
        {
            i_contact_point_data = mContactPts[i_cons_id];
            i_global_id = mContactPts[i_cons_id]->contact_id;
            i_st = i_global_id * 3;
            i_size = 3;
        }
        else
        {
            // determine the position of target matrix
            i_joint_limit_data = mJointLimits[i_cons_id - num_local_contacts];
            i_global_id =
                mJointLimits[i_cons_id - num_local_contacts]->constraint_id;
            i_st = num_total_contacts * 3 +
                   (i_joint_limit_data->constraint_id - num_total_contacts) * 1;
            i_size = 1;
        }

        tMatrixXd *residual_a = nullptr;
        if (i_contact_point_data != nullptr)
        {
            if (i_contact_point_data->mIsSelfCollision == false)
            {
                int nonself_local_id =
                    map_local_contact_id_to_nonself_contact_id[i_cons_id];
                Jac_partA = nonself_contact_point_jac[nonself_local_id];
                // std::cout << "[debug] for contact " << i_cons_id << ", res =
                // \n" << dt * inv_M * Jac_partA.transpose() << std::endl;
                residual_a = &(nonself_contact_point_jac[nonself_local_id]);
            }
            else
            {
                int self_local_id =
                    map_local_contact_id_to_self_contact_id[i_cons_id];
                Jac_partA = self_contact_point_jac[self_local_id];
                residual_a = &(self_contact_point_jac[self_local_id]);
            }
        }
        else
        {
            assert(i_joint_limit_data != nullptr);
            int i_joint_limit_id = i_cons_id - num_local_contacts;

            // set Jac part A
            Jac_partA = joint_limit_selection_jac[i_joint_limit_id];
            // if (i_joint_limit_data->is_upper_bound) Jac_partA *= -1;
            // std::cout << "Jac partA = " << Jac_partA << std::endl;
            // set residual a
            residual_a = &(joint_limit_selection_jac[i_joint_limit_id]);
            // std::cout << "residaul a before = " << residual_a << std::endl;
        }

        // 2.2 Jac_partA = i_contact_jacobian * middle_part.
        /*
            middle_part = dt * Minv in old-semi-implicit 
                        = others in new-semi-implicit
        */
        BTGEN_ASSERT(Jac_partA.hasNaN() == false);
        Jac_partA *= A_middle_part;
        BTGEN_ASSERT(Jac_partA.hasNaN() == false);
        // for each contact point that takes effect on this i-th contact point
        for (int j_cons_id = 0; j_cons_id < num_constraints; j_cons_id++)
        {
            // int j_global_id = mContactPts[j_local_id]->contact_id;
            btGenContactPointData *j_contact_point_data = nullptr;
            btGenJointLimitData *j_joint_limit_data = nullptr;
            int j_global_id = -1;

            // fetch the data by contact point id
            if (j_cons_id < num_local_contacts)
            {
                j_contact_point_data = mContactPts[j_cons_id];
                j_global_id = mContactPts[j_cons_id]->contact_id;

                j_st = j_global_id * 3;
                j_size = 3;
            }
            else
            {
                j_joint_limit_data =
                    mJointLimits[j_cons_id - num_local_contacts];
                j_global_id =
                    mJointLimits[j_cons_id - num_local_contacts]->constraint_id;
                j_st =
                    num_total_contacts * 3 +
                    (j_joint_limit_data->constraint_id - num_total_contacts) *
                        1;
                j_size = 1;
            }

            if (j_contact_point_data != nullptr)
            {
                // jth constraint is a contact point constraint
                if (mContactPts[j_cons_id]->mIsSelfCollision == false)
                {
                    int symbol = mIsBody0[j_cons_id] == false ? -1 : 1;
                    int nonself_local_id_j =
                        map_local_contact_id_to_nonself_contact_id[j_cons_id];
                    Jac_partB =
                        symbol * nonself_contact_point_jac[nonself_local_id_j]
                                     .transpose();
                }
                else
                {
                    // for self collsiion
                    int self_col_contact_id_j =
                        map_local_contact_id_to_self_contact_id[j_cons_id];
                    Jac_partB = self_contact_point_jac[self_col_contact_id_j]
                                    .transpose();
                }
            }
            else
            {
                // jth constraint is a joint limit constraint
                Jac_partB =
                    joint_limit_selection_jac[j_cons_id - num_local_contacts]
                        .transpose();
                // std::cout << "Jac partB = \n"
                // 		  << Jac_partB.transpose() << std::endl;
            }

            Jac_partB = CalcRobotColliderJacPartBPrefix(dt) * Jac_partB;
            // std::cout << "i " << i_cons_id << " j " << j_cons_id << " JacA
            // size = " << Jac_partA.rows() << " " << Jac_partA.cols() << ",
            // JacB size = " << Jac_partB.rows() << " " << Jac_partB.cols() <<
            // std::endl; 2.3 final part = Jac_partA * Jac_partB
            mConvertCartesianForceToVelocityMat
                .block(i_st, j_st, i_size, j_size)
                .noalias() = Jac_partA * Jac_partB;
            BTGEN_ASSERT(Jac_partA.hasNaN() == false);
            BTGEN_ASSERT(Jac_partB.hasNaN() == false);
        }
        // std::cout << "residaul a after = " << (*residual_a) << std::endl;
        // std::cout << "residual part = " << residual_part.transpose()
        //           << std::endl;
        mConvertCartesianForceToVelocityVec.segment(i_st, i_size).noalias() =
            (*residual_a) * residual_part;

        // record residual_a (residual convert mat from gen to cartesian)
        if (mEnableRecordCartesianConvertMatOfResidual == true)
        {
            mCharacterConvertMatOfResidualRecord.push_back(*residual_a);
        }
    }
    // std::cout << "convert mat = \n"
    //           << mConvertCartesianForceToVelocityMat << std::endl;
    // std::cout << "convert vec = "
    //           << mConvertCartesianForceToVelocityVec.transpose() << std::endl;

    // the real convert mat
    // Jv * (M + dt * (C + D )).inverse() * dt * Jv.T
    // {
    //     const tMatrixXd jac = nonself_contact_point_jac[0];
    //     tMatrix3d new_convert_mat =
    //         jac * (M + dt * (coriolis_mat + damping_mat)).inverse() * dt *
    //         jac.transpose();
    //     std::cout << "new convert mat = \n" << new_convert_mat << std::endl;

    //     tVectorXd res =
    //         jac * (M + dt * (coriolis_mat + damping_mat)).inverse() *
    //         (M * model->Getqdot() + dt * model->GetGeneralizedForce());
    //     std::cout << "new convert res = \n" << res.transpose() << std::endl;
    // }
    // std::cout << "----------set up robot collider end----------\n";
    // exit(0);
}

/**
 * \brief				Calculate the contact jacobian which can convert
 * the qdot to relative contact velocity
 *
 * For non-self collision, the jacobian is simple Jv
 * For self collisin(2 links contacted with each other, the jacobian is
 * (Jva-Jvb))
 */
void btGenCollisionObjData::GetContactJacobianArrayRobotCollider(
    tEigenArr<tMatrixXd> &nonself_contact_point_jac,
    tEigenArr<tMatrixXd> &self_contact_point_jac0_minus_jac1)
{
    btGenRobotCollider *collider = UpcastRobotCollider(mBody);
    cRobotModelDynamics *model = collider->mModel;
    int n_dof = model->GetNumOfFreedom();
    tMatrixXd Jac0_buf(3, n_dof), Jac1_buf(3, n_dof);
    for (int i_local_id = 0; i_local_id < num_local_contacts; i_local_id++)
    {
        // std::cout << "robot collider begin to set up contact " << i <<
        // std::endl;
        btGenContactPointData *data = mContactPts[i_local_id];
        int contact_id = data->contact_id;
        // std::cout <<"contact id = " << contact_id << std::endl;
        if (data->mIsSelfCollision == false)
        {
            int link_id = -1;
            if (mIsBody0[i_local_id] == true)
                link_id = UpcastRobotCollider(data->mBodyA)->mLinkId;
            else
                link_id = UpcastRobotCollider(data->mBodyB)->mLinkId;
            tVector contact_pt = mIsBody0[i_local_id] == true
                                     ? data->mContactPtOnA
                                     : data->mContactPtOnB;

            // std::cout << "contact point = " << contact_pt.transpose() <<
            // std::endl; for non self-collision
            int nonself_contact_local_id =
                map_local_contact_id_to_nonself_contact_id[i_local_id];
            model->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                link_id, contact_pt.segment(0, 3),
                nonself_contact_point_jac[nonself_contact_local_id]);
            // // std::cout << "[inner test] pred link id " << link_id << ",
            // contact pt = " << contact_pt.transpose() << std::endl;
        }
        else
        {
            // if this contact point is self-collision
            // calculate 2 jacobians "Jv" w.r.t contact point and put them into
            // "self contact point jacobian pairs"
            int link0_id = UpcastRobotCollider(data->mBodyA)->mLinkId,
                link1_id = UpcastRobotCollider(data->mBodyB)->mLinkId;
            model->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                link0_id, data->mContactPtOnA.segment(0, 3), Jac0_buf);
            // self_contact_point_jacobians_link0.push_back(jac_buffer);

            model->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                link1_id, data->mContactPtOnB.segment(0, 3), Jac1_buf);
            // self_contact_point_jacobians_link1.push_back(jac_buffer);
            int selfcontact_local_id =
                map_local_contact_id_to_self_contact_id[i_local_id];
            self_contact_point_jac0_minus_jac1[selfcontact_local_id].noalias() =
                Jac0_buf - Jac1_buf;
            // so that we can use new_id = map_contact_id_to_self_contact_id[i]
            // to access these 2 arrays above
        }
    }
}
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"

/**
 * \brief           Construct a selection matrix list 
 * which can convert the whole length of qdot to the specified dof of freedom's vel
 * 
*/
void btGenCollisionObjData::GetJointLimitJaocibanArrayRobotCollider(
    tEigenArr<tMatrixXd> &generalized_convert_list)
{
    for (int i = 0; i < num_joint_constraint; i++)
    {
        // 1. determine the joint id, its parent link and child link
        auto &joint_limit = mJointLimits[i];
        auto &mb = joint_limit->multibody;

        const auto &cur_joint = mb->GetJointByDofId(joint_limit->dof_id);
        const auto &child_link = cur_joint->GetFirstChild();
        const auto &parent_link = cur_joint->GetParent();
        generalized_convert_list[i](0, joint_limit->dof_id) = 1;
        if (joint_limit->is_upper_bound)
        {
            generalized_convert_list[i] *= -1;
        }
    }
}
void btGenCollisionObjData::SetupRigidBody()
{
    // 1. create buffer
    int n_my_contact = mContactPts.size();
    if (n_my_contact == 0)
        return;
    tEigenArr<tMatrix> RelPosSkew(n_my_contact);
    btGenRigidBody *rigidbody = UpcastRigidBody(mBody);
    tVector angvel = rigidbody->GetAngVel();
    tVector world_pos = rigidbody->GetWorldPos();
    tMatrix inertia = rigidbody->GetInertia(),
            inv_inertia = rigidbody->GetInvInertia();
    double dt = mContactPts[0]->dt;
    double invmass = rigidbody->GetInvMass();
    tVector total_force = rigidbody->GetTotalForce(),
            total_torque = rigidbody->GetTotalTorque();
    for (int i = 0; i < n_my_contact; i++)
    {
        btGenContactPointData *data = mContactPts[i];
        int contact_id = mContactPts[i]->contact_id;

        tVector contact_pt =
            mIsBody0[i] == true ? data->mContactPtOnA : data->mContactPtOnB;
        RelPosSkew[i] = btMathUtil::VectorToSkewMat(contact_pt - world_pos);
        // std::cout << " body " << mBody->GetName() << " contact " << i << "
        // isbody0 = " << mIsBody0[i] << std::endl;
    }

    // 2. begin to form the ultimate mat
    for (int i = 0; i < n_my_contact; i++)
    {
        int i_id = mContactPts[i]->contact_id;
        // form the mat
        for (int j = 0; j < n_my_contact; j++)
        {
            int j_id = mContactPts[j]->contact_id;
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

void btGenCollisionObjData::SetupRobotColliderVars()
{
    // num of world contacts
    // num of local contacts
    num_local_contacts = mContactPts.size();

    // num of self contacts
    for (int local_id = 0; local_id < num_local_contacts; local_id++)
    {
        auto &data = mContactPts[local_id];

        if (data->mIsSelfCollision == true)
        {
            map_local_contact_id_to_self_contact_id[local_id] =
                num_self_contacts;
            num_self_contacts++;
        }
        else
        {
            map_local_contact_id_to_nonself_contact_id[local_id] =
                num_nonself_contacts;
            num_nonself_contacts++;
        }

        map_world_contact_id_to_local_contact_id[data->contact_id] = local_id;
    }

    num_joint_constraint = mJointLimits.size();
}

/**
 * \brief           Calculate the middle part matrix in the linear relationship between Q and abs_v_next of contact points
 * 
 * In old-semi-implicit integration, the middle part is 
 *          dt * Minv
 * In new-semi-implicit integration, the middle part is 
 *          dt * (M + dt * (C + D)).inverse()
*/
tMatrixXd btGenCollisionObjData::CalcRobotColliderMiddlePart(double dt) const
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
tVectorXd btGenCollisionObjData::CalcRobotColliderResidual(double dt) const
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
tVectorXd
btGenCollisionObjData::CalcRobotColliderResidualOldSemiImplicit(double dt) const
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
tVectorXd
btGenCollisionObjData::CalcRobotColliderResidualNewSemiImplicit(double dt) const
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
btGenCollisionObjData::CalcRobotColliderJacPartBPrefix(double dt) const
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
void btGenCollisionObjData::GetCharacterConvertMatOfResidualRecord(
    tEigenArr<tMatrixXd> &record) const
{
    BTGEN_ASSERT(mEnableRecordCartesianConvertMatOfResidual);
    record = mCharacterConvertMatOfResidualRecord;
}