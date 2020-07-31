#include "./ContactSolver.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include <set>

extern cSimRigidBody* UpcastRigidBody(const btCollisionObject* col);
extern cRobotCollider* UpcastRobotCollider(const btCollisionObject* col);
extern cCollisionObject* UpcastColObj(const btCollisionObject* col);

void cContactSolver::SolveBySI()
{
	if (mContactConstraintData.size() == 0) return;
	PushState("before_SI");

	// 1. set up SI data, calculate the convert mat from cartesian force to relative velocity, which will be used in the iterations
	SetupDataForSI();
	// do something, velocities are updated during SI, but transforms are kept the same.
	int iter = 0;
	// std::cout << "-----------outut contact info begin-----------\n";
	// std::cout << "contact size = " << mContactConstraintData.size() << std::endl;
	// if (mContactConstraintData.size() == 3)
	// {
	// 	std::cout << "3 contacts\n";
	// }

	// output contact info
	// {
	// 	for (int i = 0; i < mContactConstraintData.size(); i++)
	// 	{
	// 		auto& data = mContactConstraintData[i];
	// 		std::cout << "for contact " << i << ", bodyA = " << data->mBodyA->GetName() << ", bodyB = " << data->mBodyB->GetName() << std::endl;
	// 		std::cout << "contact normal = " << data->mNormalPointToA.transpose() << std::endl;
	// 	}
	// }
	// std::cout << "-----------outut contact info end-----------\n";
	for (iter = 0; iter < mMaxItersSI; iter++)
	{
		// std::cout << "----------iter " << iter << " begin-----------\n";
		double leaseSquare = InternalIterationSI();
		// std::cout << "----------iter " << iter << " end-----------\n";
		if (iter % 100 == 0 && iter && mEnableDebugOutput) std::cout << "iter " << iter << " lease square = " << leaseSquare << std::endl;
		if (leaseSquare < mConvergeThresholdSI)
		{
			if (mEnableDebugOutput)
			{
				std::cout << "[SI] convergence after " << iter << " iterations\n";
			}

			break;
		}
	}
	if (iter == mMaxItersSI) std::cout << "[SI] still didn't convergence after max " << mMaxItersSI << " iterations\n";
	PopState("before_SI");

	// fetch the result
	x_si.resize(mContactConstraintData.size() * 3);
	for (int i = 0; i < mContactConstraintData.size(); i++)
	{
		auto& data = mContactConstraintData[i];

		x_si.segment(i * 3, 3) =
			(data->mSI_normal_contact_value * data->mNormalPointToA +
			 data->mSI_tangent_contact_value[0] * data->mSI_tangent_contact_dir[0] +
			 data->mSI_tangent_contact_value[1] * data->mSI_tangent_contact_dir[1])
				.segment(0, 3);
	}
}

double cContactSolver::InternalIterationSI()
{
	// 1. iterate on each contact point
	double total_normal_changed = 0;
	double total_tangent_changed = 0;
	for (auto& data : mContactConstraintData)
	{
		int contact_id = data->contact_id;
		// 2. calculate the desired normal contact force
		double delta_normal_force =

			-data->mNormalPointToA.dot(data->mSI_h) /

			((data->mNormalPointToA.transpose() * data->mSI_Z).dot(data->mNormalPointToA));
		total_normal_changed += std::fabs(delta_normal_force);
		data->mSI_normal_contact_value += delta_normal_force;

		tVector total_force = delta_normal_force * data->mNormalPointToA;
		if (mEnableFrictionalSI)
		{
			double delta_tangent_force_1 =
				-data->mSI_tangent_contact_dir[0].dot(data->mSI_h) /

				((data->mSI_tangent_contact_dir[0].transpose() * data->mSI_Z).dot(data->mSI_tangent_contact_dir[0]));

			double delta_tangent_force_2 =
				-data->mSI_tangent_contact_dir[1].dot(data->mSI_h) /

				((data->mSI_tangent_contact_dir[1].transpose() * data->mSI_Z).dot(data->mSI_tangent_contact_dir[1]));

			double new_tangent_force_total_1 =
				data->mSI_tangent_contact_value[0] + delta_tangent_force_1;
			double new_tangent_force_total_2 =
				data->mSI_tangent_contact_value[1] + delta_tangent_force_2;

			double new_friction_norm = std::sqrt(std::pow(new_tangent_force_total_1, 2) + std::pow(new_tangent_force_total_2, 2));

			if (std::fabs(new_friction_norm) >= mMu * data->mSI_normal_contact_value)
			{
				// dynamics friction
				new_tangent_force_total_1 /= (std::fabs(new_friction_norm) / (mMu * data->mSI_normal_contact_value));
				new_tangent_force_total_2 /= (std::fabs(new_friction_norm) / (mMu * data->mSI_normal_contact_value));

				delta_tangent_force_1 = new_tangent_force_total_1 - data->mSI_tangent_contact_value[0];
				delta_tangent_force_2 = new_tangent_force_total_2 - data->mSI_tangent_contact_value[1];
			}
			// std::cout << "delta tan1 = " << delta_tangent_force_1 << ", 2 = " << delta_tangent_force_2 << std::endl;
			total_force += delta_tangent_force_1 * data->mSI_tangent_contact_dir[0] + delta_tangent_force_2 * data->mSI_tangent_contact_dir[1];

			total_tangent_changed += std::fabs(delta_tangent_force_1) + std::fabs(delta_tangent_force_2);
		}

		// dry friction
		// std::cout << "[inner iter] for contact " << contact_id << std::endl;
		// std::cout << "normal = " << data->mNormalPointToA.transpose() << std::endl;
		// std::cout << "SI_Z = " << data->mSI_Z << std::endl;
		// std::cout << "SI_h = " << data->mSI_h.transpose() << std::endl;
		// std::cout << "delta normal force = " << delta_normal_force << std::endl;
		data->ApplyForceCartersian(total_force);
		// std::cout << "apply " << (delta_normal_force * data->mNormalPointToA).transpose() << std::endl;
		{
			UpdateVelocity(cur_dt);
			ClearAllConstraintForce();
			CalcAbsvelConvertMat();
			UpdateDataForSI();
		}
		// std::cout << "delta normal force = " << delta_normal_force << std::endl;
		// 3. applied the contact force
	}

	return total_normal_changed + total_tangent_changed;
}

void cContactSolver::SetupDataForSI()
{
	// std::cout << "-----------setup data for SI begin----------\n";
	for (auto& data : mContactConstraintData)
	{
		int contact_id = data->contact_id;
		// 1. clear data
		data->mSI_normal_contact_value = 0;
		data->mSI_tangent_contact_dir[0].setZero();
		data->mSI_tangent_contact_dir[1].setZero();
		data->mSI_tangent_contact_value[0] = data->mSI_tangent_contact_value[1] = 0;
		data->mSI_Z.setZero();
		data->mSI_h.setZero();

		// 2. set up two perpendicular directions
		tMatrixXd FrictionDir4 = cMathUtil::ExpandFrictionCone(4, data->mNormalPointToA);
		// std::cout << FrictionDir4.rows() << " " << FrictionDir4.cols() << std::endl;
		tVector3d dir1 = FrictionDir4.block(0, 0, 1, 3).transpose(),
				  dir2 = FrictionDir4.block(1, 0, 1, 3).transpose();
		data->mSI_tangent_contact_dir[0] = cMathUtil::Expand(dir1, 0);
		data->mSI_tangent_contact_dir[1] = cMathUtil::Expand(dir2, 0);

		// 3. get
		int body0_id = data->GetBody0Id(),
			body1_id = data->GetBody1Id();
		int body0_groupid = map_colobjid_to_groupid[body0_id],
			body1_groupid = map_colobjid_to_groupid[body1_id];
		auto& body0_A_mat = mColGroupData[body0_groupid]->mConvertCartesianForceToVelocityMat;
		auto& body1_A_mat = mColGroupData[body1_groupid]->mConvertCartesianForceToVelocityMat;
		auto& body0_b_vec = mColGroupData[body0_groupid]->mConvertCartesianForceToVelocityVec;
		auto& body1_b_vec = mColGroupData[body1_groupid]->mConvertCartesianForceToVelocityVec;
		// std::cout << "for contact " << contact_id << std::endl;

		// shape the mSI_Z and mSI_h
		data->mSI_Z.block(0, 0, 3, 3) = body0_A_mat.block(contact_id * 3, contact_id * 3, 3, 3) - body1_A_mat.block(contact_id * 3, contact_id * 3, 3, 3);

		// std::cout << "bodyA vec_b = " << body0_b_vec.transpose() << std::endl;
		// std::cout << "bodyB vec_b = " << body1_b_vec.transpose() << std::endl;
		data->mSI_h.segment(0, 3) = body0_b_vec.segment(contact_id * 3, 3) - body1_b_vec.segment(contact_id * 3, 3);
		// std::cout << "mSI_h = " << data->mSI_h.transpose() << std::endl;
	}
	// std::cout << "-----------setup data for SI end----------\n";

	// TestSICartesianConvertMatAndVec();
	// std::cout << "test finished\n";
	// exit(1);
}

void cContactSolver::UpdateDataForSI()
{
	for (auto& data : mContactConstraintData)
	{
		int contact_id = data->contact_id;
		int body0_id = data->GetBody0Id(),
			body1_id = data->GetBody1Id();
		int body0_groupid = map_colobjid_to_groupid[body0_id],
			body1_groupid = map_colobjid_to_groupid[body1_id];
		auto& body0_b_vec = mColGroupData[body0_groupid]->mConvertCartesianForceToVelocityVec;
		auto& body1_b_vec = mColGroupData[body1_groupid]->mConvertCartesianForceToVelocityVec;
		data->mSI_h.segment(0, 3) = body0_b_vec.segment(contact_id * 3, 3) - body1_b_vec.segment(contact_id * 3, 3);
		// std::cout << "mSI_h = " << data->mSI_h.transpose() << std::endl;
	}
}

void cContactSolver::TestSICartesianConvertMatAndVec()
{
	for (auto& data : mContactConstraintData)
	{
		PushState("testSI");
		// for a single contact point
		tVector applied_force = tVector::Random();

		// 1. generate prediction value
		tVector pred_relvel = data->mSI_Z * applied_force + data->mSI_h;
		data->ApplyForceCartersian(applied_force);
		UpdateVelocity(cur_dt);
		// ClearAllConstraintForce();
		// CalcAbsvelConvertMat();
		// SetupDataForSI();
		tVector true_relvel = data->GetRelVel();
		int id = data->contact_id;

		if ((true_relvel - pred_relvel).norm() > 1e-6)
		{
			std::cout << "TestSi failed!\n";
			std::cout << "contact id " << id << "pred vel = " << pred_relvel.transpose() << std::endl;
			std::cout << "contact id " << id << "true vel = " << true_relvel.transpose() << std::endl;
			exit(1);
		}

		PopState("testSI");
	}
}

void cContactSolver::ClearAllConstraintForce()
{
	for (auto& i : mColGroupData) i->mBody->ClearForce();
}

/**
 * \brief		judge wheter this body is a multibody structure and whether it is working in a max vel (aka velocity )
*/
bool cContactSolver::IsMultibodyAndVelMax(cCollisionObject* body)
{
	if (body->GetType() == eColObjType::RobotCollder &&
		true == UpcastRobotCollider(body)->mModel->IsMaxVel())
	{
		std::cout << "[verify nt rel vel] bodyB now qdot is up to max_vel, so it's trivial to verify failed(caused by clamp)\n";
		std::cout << "qdot = " << UpcastRobotCollider(body)->mModel->Getqdot().transpose() << std::endl;
		return true;
	}
	else
		return false;
}

void cContactSolver::CompareSILCPResult()
{
	for (int i = 0; i < mContactConstraintData.size(); i++)
	{
		if (i == 0) std::cout << "----------begin to compare result between LCP and SI-----------\n";
		std::cout << "contact " << i << " lcp force = " << f_lcp.segment(i * 3, 3).transpose() << std::endl;
		std::cout << "contact " << i << " SI force = " << x_si.segment(i * 3, 3).transpose() << std::endl;
	}
}