#include "ConstraintData.h"
#include "../SimObj.h"
#include "../Model/RobotCollider.h"
#include "../Model/RobotModel.h"
#include <iostream>

extern cSimRigidBody* UpcastRigidBody(const btCollisionObject* col);
extern cRobotCollider* UpcastRobotCollider(const btCollisionObject* col);
extern cCollisionObject* UpcastColObj(const btCollisionObject* col);

tContactPointData::tContactPointData(int c_id, double dt_)
{
	contact_id = c_id;
	dt = dt_;
	mbody0GroupId = -1;
	mbody1GroupId = -2;
	mIsSelfCollision = false;
}

/**
 * \brief			calculate the final convert matrix H and vector h
 * 
*/
void tContactPointData::Setup(int num_frictions)
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
	// std::cout << "mHn shape = " << mHn.rows() << " " << mHn.cols() << std::endl;
	// std::cout << "mhn shape = " << mhn.rows() << " " << mhn.cols() << std::endl;
	// std::cout << "mHt shape = " << mHt.rows() << " " << mHt.cols() << std::endl;
	// std::cout << "mht shape = " << mht.rows() << " " << mht.cols() << std::endl;
}

void tContactPointData::ApplyForceResultVector(const tVectorXd& x0)
{
	ApplyForceCartersian(cMathUtil::Expand(mS * x0, 0));
}

void tContactPointData::ApplyForceCartersian(const tVector& contact_force)
{
	// std::cout <<"body A apply " << contact_force.transpose() << " on " << mContactPtOnA.transpose() << std::endl;
	// std::cout <<"body B apply " << -contact_force.transpose() << " on " << mContactPtOnB.transpose() << std::endl;
	mBodyA->ApplyForce(contact_force, mContactPtOnA);
	mBodyB->ApplyForce(-contact_force, mContactPtOnB);
}

bool tContactPointData::CheckOverlap(tContactPointData* other_data)
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

tMatrixXd tContactPointData::GetConvertMatS(int world_col_id)
{
	if (world_col_id == mBodyId0)
		return mS;
	else if (world_col_id == mBodyId1)
		return -mS;
	else
	{
		std::cout << "get S for world id " << world_col_id << " but here are only " << mBodyId0 << " and " << mBodyId1 << std::endl;
		exit(1);
	}
}

int tContactPointData::GetBody0Id()
{
	return mBodyId0;
}
int tContactPointData::GetBody1Id()
{
	return mBodyId1;
}

tVector tContactPointData::GetVelOnBody0()
{
	return mBodyA->GetVelocityOnPoint(mContactPtOnA);
}

tVector tContactPointData::GetVelOnBody1()
{
	return mBodyB->GetVelocityOnPoint(mContactPtOnB);
}

tVector tContactPointData::GetRelVel()
{
	tVector body0_vel = GetVelOnBody0(),
			body1_vel = GetVelOnBody1();

	// std::cout << "Get rel vel, vel0 = " << body0_vel.transpose() << std::endl;
	// std::cout << "Get rel vel, vel1 = " << body1_vel.transpose() << std::endl;
	return body0_vel - body1_vel;
}
void tContactPointData::UpdateVel(double dt)
{
	mBodyA->UpdateVelocity(dt);
	mBodyB->UpdateVelocity(dt);
}

void tContactPointData::CalcBodyInfo()
{
	// change type
	mBodyId0 = mBodyA->getWorldArrayIndex();
	mBodyId1 = mBodyB->getWorldArrayIndex();
}

// D is 3 x N. each col vector is a friction direction
void tContactPointData::CalcFrictionCone(tMatrixXd& D)
{
	D.resize(3, mNumOfFriction), D.setZero();
	D = cMathUtil::ExpandFrictionCone(mNumOfFriction, mNormalPointToA).transpose().block(0, 0, 3, mNumOfFriction);
	// std::cout <<"normal = " << mNormalPointToA.transpose() << std::endl;
	// std::cout <<"mD = \n" << mD << std::endl;
}

// mat S can convert a result vector x_0 (shape N+2) to a force in cartesian space f_0 3x1
void tContactPointData::CalcConvertMat(tMatrixXd& S)
{
	int x_single_size = mNumOfFriction + 2;  // (f_n + f_\parallel + \lambda).shape = NumOfFriction + 2
	S.resize(3, x_single_size), S.setZero();

	S.col(0) = mNormalPointToA.segment(0, 3);
	S.block(0, 1, 3, mNumOfFriction) = mD;
}

//---------------------------------------------------------------------------

tCollisionObjData::tCollisionObjData(cCollisionObject* obj)
{
	mBody = obj;
	Clear();
}

void tCollisionObjData::Clear()
{
	mContactPts.clear();
	mIsBody0.clear();
}

void tCollisionObjData::AddContactPoint(tContactPointData* data, bool isbody0)
{
	mContactPts.push_back(data);
	mIsBody0.push_back(isbody0);
}

/**
 * \brief			Setup all contact points for this collision group.
 * 			Note that, a collision group is the minimun group where contact points' jacobian are calculated.
 * 			A rigidbody = a collision group	
 * 			A multibody = a collision group
 * 			A link in multibody should belong to but != a colision group
*/
void tCollisionObjData::Setup(int n_total_contacts)
{
	mConvertCartesianForceToVelocityMat.resize(3 * n_total_contacts, 3 * n_total_contacts), mConvertCartesianForceToVelocityMat.setZero();
	mConvertCartesianForceToVelocityVec.resize(3 * n_total_contacts), mConvertCartesianForceToVelocityVec.setZero();
	// std::cout << "my contact size = " << mContactPts.size() << std::endl;
	// for this collision obj, calculate the convert mat from full forces to velcoity
	if (mBody->IsStatic() == true) return;
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
			exit(1);
	}
}

void tCollisionObjData::SetupRobotCollider()
{
	int n_my_contact = mContactPts.size();
	if (n_my_contact == 0) return;
	cRobotCollider* collider = UpcastRobotCollider(mBody);
	cRobotModel* model = collider->mModel;
	// int link_id = mContactPts[];
	// auto link = model->GetLinkById(link_id);

	tEigenArr<tMatrixXd> non_self_contact_pt_jacobians(n_my_contact);      // jacobian for link affected (non self collision case)
	std::vector<int> map_contact_id_to_self_contact_id(n_my_contact, -1);  // idx: contact id from 0-n_my_contact-1, value: self contact id used to access self_contact_point_jacobians_link0
	tEigenArr<tMatrixXd> self_contact_point_jacobians_link0(0);            // jacobian arrays for body 0 in self collisoin case
	tEigenArr<tMatrixXd> self_contact_point_jacobians_link1(0);            // for body 1 in self collision case

	tMatrixXd inv_M = model->GetMassMatrix().inverse();
	tMatrixXd Coriolis_mat = model->GetCoriolisMatrix();
	tMatrixXd damping_mat = model->GetDampingMatrix();
	// std::cout << "[setup] M inv = \n"
	// 		  << inv_M << std::endl;
	// std::cout << "[setup] C = \n"
	// 		  << Coriolis_mat << std::endl;

	int n_dof = model->GetNumOfFreedom();
	double dt = mContactPts[0]->dt;
	int n_self_contact = 0;  // self collision number
	for (int i = 0; i < n_my_contact; i++)
	{
		tContactPointData* data = mContactPts[i];
		int contact_id = data->contact_id;

		if (data->mIsSelfCollision == false)
		{
			int link_id = -1;
			if (mIsBody0[i] == true)
				link_id = UpcastRobotCollider(data->mBodyA)->mLinkId;
			else
				link_id = UpcastRobotCollider(data->mBodyB)->mLinkId;
			tVector contact_pt = mIsBody0[i] == true ? data->mContactPtOnA : data->mContactPtOnB;

			// for non self-collision
			model->ComputeJacobiByGivenPointTotalDOFWorldFrame(link_id, contact_pt.segment(0, 3), non_self_contact_pt_jacobians[i]);
		}
		else
		{
			// if this contact point is self-collision
			// calculate 2 jacobians "Jv" w.r.t contact point and put them into "self contact point jacobian pairs"
			int link0_id = UpcastRobotCollider(data->mBodyA)->mLinkId,
				link1_id = UpcastRobotCollider(data->mBodyB)->mLinkId;
			tMatrixXd jac_buffer;
			model->ComputeJacobiByGivenPointTotalDOFWorldFrame(link0_id, data->mContactPtOnA.segment(0, 3), jac_buffer);
			self_contact_point_jacobians_link0.push_back(jac_buffer);

			model->ComputeJacobiByGivenPointTotalDOFWorldFrame(link1_id, data->mContactPtOnB.segment(0, 3), jac_buffer);
			self_contact_point_jacobians_link1.push_back(jac_buffer);

			// so that we can use new_id = map_contact_id_to_self_contact_id[i] to access these 2 arrays above
			map_contact_id_to_self_contact_id[i] = n_self_contact;
			n_self_contact++;
		}
	}
	// std::cout << "[setup] jac = \n"
	// 		  << contact_pt_jacobians[0] << std::endl;
	// std::cout << "[setup] Q = " << (contact_pt_jacobians[0].transpose() * tVector3d(0, 1, 0)).transpose() << std::endl;

	// 2. begin to form the ultimate velocity convert mat
	for (int i = 0; i < n_my_contact; i++)
	{
		auto& i_data = mContactPts[i];
		int i_id = mContactPts[i]->contact_id;
		if (i_data->mIsSelfCollision == false)
		{
			// if ith contact is not self collision, then the matrix which we want to calculate can
			// convert full size cartesian force into the abs velocity next frame in this contact point in myself body
			for (int j = 0; j < n_my_contact; j++)
			{
				int j_id = mContactPts[j]->contact_id;

				// here, ith contact is not a self contact, but the jth contact is a self contact
				// and the contact force in jth point, can aboselute affect the velocity in i th point
				if (mContactPts[j]->mIsSelfCollision == false)
				{
					int symbol = mIsBody0[j] == false ? -1 : 1;
					mConvertCartesianForceToVelocityMat.block(3 * i_id, 3 * j_id, 3, 3) =
						symbol * dt *
						non_self_contact_pt_jacobians[i] *
						inv_M *
						non_self_contact_pt_jacobians[j].transpose();
				}
				else
				{
					// for self collsiion
					int self_col_contact_id_j = map_contact_id_to_self_contact_id[j];
					mConvertCartesianForceToVelocityMat.block(3 * i_id, 3 * j_id, 3, 3) =
						dt *
						non_self_contact_pt_jacobians[i] *
						inv_M *
						(self_contact_point_jacobians_link0[self_col_contact_id_j] - self_contact_point_jacobians_link1[self_col_contact_id_j]).transpose();
				}
			}

			// the residual here is the the velocity of contact point at this moment (history)
			mConvertCartesianForceToVelocityVec.segment(i * 3, 3) =
				non_self_contact_pt_jacobians[i] *
				(tMatrixXd::Identity(n_dof, n_dof) - dt * inv_M * (Coriolis_mat + damping_mat)) * model->Getqdot();
		}
		else
		{
			// here if ith contact is self collision, we need to slightly violate the functionality of this method.
			// this method is originally use to calculate the ABOSULUTE CONTACT VELOCITY CONVERT MAT in this contact,
			// but in self-collision case, we must put the final RELATIVE CONTACT VELOCITY CONVERT MAT here...

			// so what we compute later is :
			// a mat which can convert full size carteisan contact force into the relative contact velocity next frame in this self-contact point
			// a vec as residual to represent relative contact velocity

			/*
				FORMULAE:
					(hint: o means other non self collsioin contact point)
				v_a = dt * Jvai * Minv * [0, ..., (Jvaj-Jvbj)^T, ..., Jvo^T, ...] [..., fa, ..., fo] + Jvai * (In - dt * Minv * C) qdot
				v_b = dt * Jvbi * Minv * [0, ..., (Jvaj-Jvbj)^T, ..., Jvo^T, ...] [..., fa, ..., fo] + Jvbi * (In - dt * Minv * C) qdot
				u 	= v_a - v_b
					= 	dt * (Jvai - Jvbi) * Min * [0, ..., (Jaj-Jbj)^T, ..., Jvo^T, ...] [..., fa, ..., fo] 
						+
						(Jvai - Jvbi) * (In = dt * Minv * C) qdot

				u = mat * F + vec
				mat = dt * (Jvai - Jvbi) * Min * [0, ..., (Jaj-Jbj)^T, ..., Jvo^T, ...] \in 3x(3m)
				vec = (Jvai - Jvbi) * (In = dt * Minv * C) qdot \in 3x1
				
			*/
			int i_self_col_id = map_contact_id_to_self_contact_id[i];
			tMatrixXd Jva_i_minus_Jvb_i = self_contact_point_jacobians_link0[i_self_col_id] - self_contact_point_jacobians_link1[i_self_col_id];

			for (int j = 0; j < n_my_contact; j++)
			{
				if (mContactPts[j]->mIsSelfCollision == false)
				{
					mConvertCartesianForceToVelocityMat.block(3 * i_id, 3 * j, 3, 3) =
						dt *
						Jva_i_minus_Jvb_i *
						inv_M *
						non_self_contact_pt_jacobians[j].transpose();
				}
				else
				{
					// self collision in here jth
					int j_self_col_id = map_contact_id_to_self_contact_id[j];
					tMatrixXd Jva_j_minus_Jvb_j = self_contact_point_jacobians_link0[j_self_col_id] - self_contact_point_jacobians_link1[j_self_col_id];
					mConvertCartesianForceToVelocityMat.block(3 * i_id, 3 * j, 3, 3) =
						dt *
						Jva_i_minus_Jvb_i *
						inv_M *
						Jva_j_minus_Jvb_j.transpose();
				}
			}

			// self collsiion relative velocity residual part
			mConvertCartesianForceToVelocityVec.segment(i * 3, 3) =
				Jva_i_minus_Jvb_i *
				(tMatrixXd::Identity(n_dof, n_dof) - dt * inv_M * (Coriolis_mat + damping_mat)) * model->Getqdot();
		}

		// std::cout <<"[abs convert] mat = \n" << mConvertCartesianForceToVelocityMat << std::endl;
		// std::cout <<"[abs convert] vec = " << mConvertCartesianForceToVelocityVec.transpose() << std::endl;
	}
}

void tCollisionObjData::SetupRigidBody()
{
	// 1. create buffer
	int n_my_contact = mContactPts.size();
	if (n_my_contact == 0) return;
	if (n_my_contact == 2)
	{
		n_my_contact = 2;
	}
	tEigenArr<tMatrix> RelPosSkew(n_my_contact);
	cSimRigidBody* rigidbody = UpcastRigidBody(mBody);
	tVector angvel = rigidbody->GetAngVel();
	tVector world_pos = rigidbody->GetWorldPos();
	tMatrix inertia = rigidbody->GetInertia(),
			inv_inertia = rigidbody->GetInvInertia();
	double dt = mContactPts[0]->dt;
	double invmass = rigidbody->GetInvMass();
	for (int i = 0; i < n_my_contact; i++)
	{
		tContactPointData* data = mContactPts[i];
		int contact_id = mContactPts[i]->contact_id;

		tVector contact_pt = mIsBody0[i] == true ? data->mContactPtOnA : data->mContactPtOnB;
		RelPosSkew[i] = cMathUtil::VectorToSkewMat(contact_pt - world_pos);
		// std::cout << " body " << mBody->GetName() << " contact " << i << " isbody0 = " << mIsBody0[i] << std::endl;
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
			mConvertCartesianForceToVelocityMat.block(i_id * 3, j_id * 3, 3, 3) =
				symbol * dt *
				(invmass * tMatrix3d::Identity() - RelPosSkew[i].block(0, 0, 3, 3) * inv_inertia.block(0, 0, 3, 3) * RelPosSkew[j].block(0, 0, 3, 3));
		}

		// form the residual
		mConvertCartesianForceToVelocityVec.segment(i_id * 3, 3) =
			(rigidbody->GetLinVel() - RelPosSkew[i] * angvel +
			 cMathUtil::VectorToSkewMat(dt * RelPosSkew[i] * inv_inertia * angvel) * (inertia * angvel))
				.segment(0, 3);
	}
}