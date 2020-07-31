#include "RobotModelDynamics.h"
#include "RobotCollider.h"
#include "btBulletDynamicsCommon.h"
#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#include "Link.h"
#include <iostream>
#include <fstream>

extern bool gEnablePauseWhenSolveError;
extern bool gEnableResolveWhenSolveError;

// std::string path = "debug_trans_bt.txt";
cRobotModelDynamics::cRobotModelDynamics(const char* model_file, double scale, int type) : cRobotModel(model_file, scale, type), damping_coef1(0), damping_coef2(0), eps_diagnoal_mass_mat(0), clamp_euler_angle(true)
{
	// std::ofstream fout(path);
	// fout << "";
	// fout.close();
}

void cRobotModelDynamics::ComputeDampingMatrix()
{
	damping_matrix = damping_coef1 * mass_matrix + damping_coef2 * coriolis_matrix;
}

void cRobotModelDynamics::ComputeMassMatrix()
{
	cRobotModel::ComputeMassMatrix();
	// std::cout << "[model] add eps " << eps_diagnoal_mass_mat << " on the diagnoal of mass matrix\n";
	mass_matrix += eps_diagnoal_mass_mat * tMatrixXd::Identity(mass_matrix.rows(), mass_matrix.cols());
}

/**
 * \brief					create Multibody colliders in bullet world
 * \param world				the ptr to btWorld
 * */
extern std::map<int, std::string> col_name;
void cRobotModelDynamics::InitSimVars(btDiscreteDynamicsWorld* world, bool zero_pose, bool zero_pose_vel)
{
	std::cout << "Init collider\n";
	multibody_colliders.clear();
	cRobotCollider* collider = nullptr;

	// 1. get link shape info  and add them into btworld
	int num_of_links = GetNumOfLinks();
	for (int i = 0; i < num_of_links; i++)
	{
		const auto& link = GetLinkById(i);
		double mass = link->GetMass();
		ShapeType shape_type = link->GetShapeType();
		btCollisionShape* shape = nullptr;
		const tVector3f& mesh_scale = link->GetMeshScale();
		std::cout << "for link " << i << " mesh scale = " << mesh_scale.transpose() << std::endl;
		switch (shape_type)
		{
			case ShapeType::SPHERE_SHAPE:
				shape = new btSphereShape(btScalar(mesh_scale[0]));
				break;
			case ShapeType::BOX_SHAPE:
				shape = new btBoxShape(cBulletUtil::tVectorTobtVector(cMathUtil::Expand(mesh_scale, 0)) / 2);
				break;
			default:
				std::cout << "unsupported type in init sim vars\n";
				exit(1);
				break;
		}
		// btCollisionShape* shape = new btBoxShape(btVector3(mesh_scale[0], mesh_scale[1], mesh_scale[2]) / 2);

		btVector3 localInertia(0, 0, 0);
		shape->calculateLocalInertia(mass, localInertia);

		collider = new cRobotCollider(this, i, GetLinkById(i)->GetName() + std::to_string(i));
		collider->setCollisionShape(shape);
		collider->setUserIndex(-1);
		collider->setUserPointer(collider);

		// world->addCollisionObject(collider, 2, 1 + 2);
		world->addCollisionObject(collider, 1, -1);
		col_name[collider->getWorldArrayIndex()] = "multibody" + std::to_string(collider->mLinkId);
		multibody_colliders.push_back(collider);
	}

	// 2. set up the parent collider
	for (int i = 0; i < num_of_links; i++)
	{
		const BaseObject* parent_joint = GetLinkById(i)->GetParent();
		int parent_link_id = parent_joint->GetParentId();
		if (-1 == parent_link_id)
		{
			multibody_colliders[i]->mParentCollider = nullptr;
		}
		else
		{
			multibody_colliders[i]->mParentCollider = multibody_colliders[parent_link_id];
		}
		std::cout << "for link " << i << " parent link id = " << parent_link_id << std::endl;
	}

	// 3. allocate sim vars and Init
	link_forces.resize(num_of_links, tVector::Zero());
	link_torques.resize(num_of_links, tVector::Zero());
	q.resize(GetNumOfFreedom()), q.setZero();
	qdot.resize(GetNumOfFreedom()), qdot.setZero();

	generalized_force.resize(GetNumOfFreedom());
	generalized_force.setZero();
	// const double q[] = {
	// 	-0.0289297, -1.82364, 0.196723, -2.5256, -1.27079, -0.473494, 3.89473, -0.461045, 0.0931059, 1.91737, 2.93732, -0.209514, 0.0629891};
	// const double qdot[] = {
	// 	-0.446217  -4.00862   1.62062  -10.2378  -21.9702  -25.5659        50  -5.44128  -4.33548  -16.1202   23.4421    -21.68       -50
	// }
	// set base init
	// {
	// 	qdot.segment(0, 3) = tVector3d(4, -1, 4);
	// 	qdot.segment(3, 3) = tVector3d(12, 01, -7);
	// q.segment(3, 3) = tVector3d(SIMD_PI / 6., 0, 0);
	// qdot[1] = -10;

	// q << 0.680375, -1.18218, 0.566198, 0.59947, 0.824512, -0.603272;
	// qdot << 0, -0.607199, 0, 2.59866, 1.22131, 1.63052;
	if (zero_pose == true)
		q.setZero();
	else
	{
		q.setRandom();
		// q << 0.692907, -1.89453, 1.00441, 2.60635, 0.0663348, 1.52721, 1.47363, -0.742139, -1.52918, -3.96639, 1.54574, -3.05681;
	}

	if (zero_pose_vel == true)
		qdot.setZero();
	else
	{
		qdot.setRandom();
		// qdot << 0.313725, -0.386997, -0.205985, 1.10284, -3.63833, 2.3115, 6.90362, -1.59941, -4.80563, -34.178, 0.49514, -34.3533;
		// qdot << -0.000843372, -2.79265, -0.128726, 12.5693, 0.0125792, -0.0564807, -16.9428, 0.0461597, -0.0708239, 8.18329, -6.70682, 0.0330331, -0.00263514, -16.986, 0.0207209, -0.14805, 8.19547, -6.57637, -0.238355, -0.231526;
	}

	// q << 1.15553, -1.88728, 1.27404, 1.55231, 1.10814, 0.000647896, 0.0180331, 0.00719906, -0.416765, 0.142436, -0.00265726, -0.290101, 0.101719, 0.100274, -0.306586, -1.36788, 0.0796965, 0.716776, 1.0514, -1.02883, 1.04332, -0.304633, 0.0190385, 0.240296, 0.139491, -0.115072, 0.112997, -0.990685, -0.0549583, -0.526088, 0.298854, 0.567608, -1.04001, -0.0234368, 0.0394043, 0.356135, -0.0214564, -0.258604, -1.5753, 1.61385, 0.0285423, -1.02547, 0.505478, -0.0890805, 1.57299, -1.57115, -0.124728, 0.0458547, 0.0520501, -0.204938;
	// q[6] = -0.1;
	SetPose(q, qdot);

	tVectorXd lower, upper;
	GetJointLimit(lower, upper);
	std::cout << "[debug] joint lower limit = " << lower.transpose() << std::endl;
	std::cout << "[debug] joint upper limit = " << upper.transpose() << std::endl;
	// this->TestRotationChar();
	// exit(1);
}

void cRobotModelDynamics::TestJacobian()
{
	// 1. save old state
	tVectorXd q_old = q, qdot_old = qdot;
	int n_dof = GetNumOfFreedom();

	// 2. test jacobian
	q = tVectorXd::Random(n_dof);
	qdot = tVectorXd::Random(n_dof);
	for (int link_id = 0; link_id < GetNumOfLinks(); link_id++)
	{
		Apply(q, true);
		tVector pt_local = cMathUtil::Expand(tVector3d::Random() * 10, 1);
		tMatrixXd jac_pred, jac_truth;
		auto link = GetLinkById(link_id);
		tVector pt_world_old = link->GetGlobalTransform() * pt_local;

		// 1. pred

		ComputeJacobiByGivenPointTotalDOFWorldFrame(link_id, pt_world_old.segment(0, 3), jac_pred);
		// ComputeJacobiByGivenPointTotalDOFLocalFrame(link_id, pt_local.segment(0, 3), jac_pred);

		// 2. true
		const double bias = 1e-7;
		jac_truth.resize(3, n_dof);
		tVector pt_world_new;
		for (int dof = 0; dof < n_dof; dof++)
		{
			// std::cout << "q size " << q.size() << " dof " << dof << std::endl;
			q[dof] += bias;
			Apply(q, true);
			pt_world_new = link->GetGlobalTransform() * pt_local;
			jac_truth.block(0, dof, 3, 1) = (pt_world_new - pt_world_old).segment(0, 3) / bias;
			q[dof] -= bias;
		}
		double err = (jac_truth - jac_pred).norm();
		std::cout << "test jacobian link " << link_id << ", residual = " << err << std::endl;

		if (err > 1e-5)
		{
			std::cout << "jac pred = \n"
					  << jac_pred << std::endl;
			std::cout << "jac truth = \n"
					  << jac_truth << std::endl;

			std::cout << "error test jacobian in link " << link_id << std::endl;
		}
	}
	// exit(0);
}

/**
 * \brief			test second order jacobian
 * d(Jv)/dq = 3xnxn = Jv'
 * d(Jw)/dq = 3xnxn = Jw'
*/
void cRobotModelDynamics::TestSecondJacobian()
{
	int num_of_links = GetNumOfLinks();
	tVectorXd q = Getq(), qdot = Getqdot();
	// q.setZero();
	// qdot.setZero();
	q.setRandom();
	qdot.setRandom();
	std::cout << "[test] q = " << q.transpose() << std::endl;
	std::cout << "[test] qdot = " << qdot.transpose() << std::endl;
	Apply(q, true);

	// check mWQQ

	{
		// auto link = GetLinkById(0);
		// for (int i = 3; i < 6; i++)
		// 	for (int j = i; j < 6; j++)
		// 	{
		// 		printf("dWdq%ddq%d = \n", i, j);
		// 		std::cout << link->GetParent()->GetMWQQ(i, j) << std::endl;
		// 	}
		// exit(1);
	}
	// for (int i = 0; i < 2; i++)
	// {
	// 	q[4] += 1.0;
	// 	std::cout << " q = " << q.transpose() << std::endl;
	// 	Apply(q, true);
	// 	for (int id = 0; id < num_of_links; id++)
	// 	{
	// 		auto link = GetLinkById(id);
	// 		// link->ComputeJKw();
	// 		std::cout << "Jkw = \n"
	// 				  << link->GetJKw() << std::endl;
	// 	}
	// }
	// exit(1);
	// 1. compute Jv'(dJv/dq) and Jw'(dJw/dq), current Jv and Jw

	tVector3d p = tVector3d(0, 0, 0);

	tEigenArr<tMatrixXd> Jkv_middle(0), Jkw_middle(0);  // Jv, Jw
	std::vector<tEigenArr<tMatrixXd>> dJkvdq_ana(num_of_links);
	std::vector<tEigenArr<tMatrixXd>> dJkwdq_ana(num_of_links);

	for (int i = 0; i < num_of_links; i++)
	{
		auto link = GetLinkById(i);
		link->ComputeDJkvdq(p);
		link->ComputeDJkwdq();

		Jkv_middle.push_back(link->GetJKv());
		Jkw_middle.push_back(link->GetJKw());

		dJkvdq_ana[i].resize(num_of_freedom, tMatrixXd::Zero(3, num_of_freedom));
		dJkwdq_ana[i].resize(num_of_freedom, tMatrixXd::Zero(3, num_of_freedom));
		tMatrixXd nxn_djvdq[3], nxn_djwdq[3];
		for (int sub_id = 0; sub_id < 3; sub_id++)
		{
			nxn_djvdq[sub_id] = link->GetJKv_dq(sub_id);
			nxn_djwdq[sub_id] = link->GetJKw_dq(sub_id);
			// std::cout << nxn_djvdq[sub_id] << std::endl;
			// std::cout << "nxn_djwdq " << sub_id << " = \n"
			// 		  << nxn_djwdq[sub_id] << std::endl;
		}
		// std::cout << "it should be all zero\n";
		// exit(1);
		for (int dof = 0; dof < num_of_freedom; dof++)
		{
			dJkvdq_ana[i][dof].resize(3, num_of_freedom);
			dJkvdq_ana[i][dof].block(0, 0, 1, num_of_freedom) = nxn_djvdq[0].block(dof, 0, 1, num_of_freedom);
			dJkvdq_ana[i][dof].block(1, 0, 1, num_of_freedom) = nxn_djvdq[1].block(dof, 0, 1, num_of_freedom);
			dJkvdq_ana[i][dof].block(2, 0, 1, num_of_freedom) = nxn_djvdq[2].block(dof, 0, 1, num_of_freedom);

			dJkwdq_ana[i][dof].resize(3, num_of_freedom);
			// dJkwdq_ana[i][dof].block(0, 0, 1, num_of_freedom) = nxn_djwdq[0].block(dof, 0, 1, num_of_freedom);
			// dJkwdq_ana[i][dof].block(1, 0, 1, num_of_freedom) = nxn_djwdq[1].block(dof, 0, 1, num_of_freedom);
			// dJkwdq_ana[i][dof].block(2, 0, 1, num_of_freedom) = nxn_djwdq[2].block(dof, 0, 1, num_of_freedom);
			dJkwdq_ana[i][dof].block(0, 0, 1, num_of_freedom) = nxn_djwdq[0].block(0, dof, num_of_freedom, 1).transpose();
			dJkwdq_ana[i][dof].block(1, 0, 1, num_of_freedom) = nxn_djwdq[1].block(0, dof, num_of_freedom, 1).transpose();
			dJkwdq_ana[i][dof].block(2, 0, 1, num_of_freedom) = nxn_djwdq[2].block(0, dof, num_of_freedom, 1).transpose();
		}
	}

	// 2. add bias to q and then calculat the numerical gradient
	// tEigenArr<tMatrixXd> dJkvdq_num(0), dJkwdq_num(0);
	double bias = 1e-6;
	for (int link_id = 0; link_id < num_of_links; link_id++)
	{
		auto link = GetLinkById(link_id);
		for (int dof = 0; dof < GetNumOfFreedom(); dof++)
		{
			// std::cout << "raw q = " << q.transpose() << std::endl;
			// q[dof] += bias;
			q[dof] = q[dof] + bias;
			// std::cout << "new q = " << q.transpose() << std::endl;
			Apply(q, true);
			link->ComputeJKv();
			link->ComputeJKw();
			tMatrixXd Jkv_cur = link->GetJKv();
			tMatrixXd dJkvdq_num = (Jkv_cur - Jkv_middle[link_id]) / bias;

			tMatrixXd dJkvdq_diff = dJkvdq_ana[link_id][dof] - dJkvdq_num;
			if (dJkvdq_diff.cwiseAbs().maxCoeff() > 1e-6)
			{
				std::cout << "dJvdq_ana link " << link_id << " dof " << dof << " = \n"
						  << dJkvdq_ana[link_id][dof] << std::endl;
				std::cout << "dJvdq_num link " << link_id << " dof " << dof << " = \n"
						  << dJkvdq_num << std::endl;
				std::cout << "error\n";
				exit(1);
			}
			else
				std::cout << "[log] link " << link_id << " dof " << dof << " dJvdq verified succ\n";
			tMatrixXd Jkw_cur = link->GetJKw();
			tMatrixXd dJkwdq_num = (Jkw_cur - Jkw_middle[link_id]) / bias;
			tMatrixXd dJkwdq_diff = dJkwdq_ana[link_id][dof] - dJkwdq_num;
			// std::cout << "link " << link_id << " dof " << dof << " Jw_cur = \n"
			// 		  << Jkw_cur << std::endl;
			// std::cout << "link " << link_id << " dof " << dof << " Jw_middle = \n"
			//   << Jkw_middle[link_id] << std::endl;
			if (dJkwdq_diff.cwiseAbs().maxCoeff() > 1e-6)
			{
				std::cout << "dJwdq_ana link " << link_id << " dof " << dof << " = \n"
						  << dJkwdq_ana[link_id][dof] << std::endl;
				std::cout << "dJwdq_num link " << link_id << " dof " << dof << " = \n"
						  << dJkwdq_num << std::endl;
				std::cout << "dJwdq_diff link " << link_id << " dof " << dof << " = \n"
						  << dJkwdq_diff << std::endl;
				std::cout << "error\n";
				exit(1);
			}
			else
				std::cout << "[log] link " << link_id << " dof " << dof << " dJwdq verified succ\n";

			q[dof] -= bias;
			Apply(q, true);
		}
	}

	// 3. compare
	// exit(1);
}

void cRobotModelDynamics::SetPose(const tVectorXd& q_, const tVectorXd& qdot_)
{
	q = q_;
	qdot = qdot_;
	Apply(q, true);
	ComputeMassMatrix();
	ComputeCoriolisMatrix(qdot);
	ComputeDampingMatrix();
	UpdateCartesianVelocity();
	SyncToBullet();
}

void cRobotModelDynamics::SetMassMatEps(double eps)
{
	eps_diagnoal_mass_mat = eps;
}

void cRobotModelDynamics::ApplyGravity(const tVector& g)  // apply force
{
	for (int i = 0; i < GetNumOfLinks(); i++)
	{
		ApplyForce(i, GetLinkById(i)->GetMass() * g, cMathUtil::Expand(GetLinkById(i)->GetWorldPos(), 1));
	}
	// std::cout << "after apply gravity, force num = " << link_force_id_lst.size() << std::endl;
}

void cRobotModelDynamics::ApplyForce(int link_id, const tVector& force, const tVector& applied_pos)
{
	if (link_id < 0 || link_id >= GetNumOfLinks())
	{
		std::cout << "ApplyForce: illegal link id " << link_id << std::endl;
		exit(1);
	}

	// 1. apply the force directly
	link_forces[link_id] += force;

	// 2. calculate the torque and apply it
	auto* link = GetLinkById(link_id);
	tVector rel_pos = applied_pos - cMathUtil::Expand(link->GetWorldPos(), 1);
	link_torques[link_id] += rel_pos.cross3(force);
}

void cRobotModelDynamics::ApplyGeneralizedForce(int dof_id, double value)
{
	if (dof_id < 0 || dof_id >= GetNumOfFreedom())
	{
		std::cout << "[error] add generalized force on dof " << dof_id << " illegal\n";
		exit(1);
	}
	generalized_force[dof_id] += value;
}

/**
 * \brief					add torques on multibody links
 * \param	link_id			target link id
 * \param	torque			applied torque in world frame
*/
void cRobotModelDynamics::ApplyLinkTorque(int link_id, const tVector& torque)
{
	if (link_id < 0 || link_id >= GetNumOfLinks())
	{
		std::cout << "ApplyLinkTorque: illegal link id " << link_id << std::endl;
		exit(1);
	}

	link_torques[link_id] += torque;
}

/**
 * \brief					add torque on multibody joints
 * \param	joint_id		target joint id
 * \param	torque			joint torque applied on the joint in world frame
*/
void cRobotModelDynamics::ApplyJointTorque(int joint_id, const tVector& torque)
{
	if (joint_id < 0 || joint_id >= GetNumOfJoint())
	{
		std::cout << "ApplyJointTorque: illegal joint id " << joint_id << std::endl;
		exit(1);
	}

	// one pair of opposite torque
	int parent_id = GetJointById(joint_id)->GetParentId(),
		child_id = joint_id;
	// std::cout << "[debug] add joint torque for parent " << parent_id << ": " << -torque.transpose() << std::endl;
	// std::cout << "[debug] add joint torque for child " << joint_id << ": " << torque.transpose() << std::endl;
	if (parent_id >= 0) link_torques[parent_id] -= torque;
	link_torques[joint_id] += torque;
}

tVectorXd cRobotModelDynamics::GetGeneralizedForce()
{
	tVectorXd Q = tVectorXd::Zero(GetNumOfFreedom());
	for (int i = 0; i < GetNumOfLinks(); i++)
	{
		const auto& link = GetLinkById(i);
		Q += link->GetJKv().transpose() * link_forces[i].segment(0, 3);
		Q += link->GetJKw().transpose() * link_torques[i].segment(0, 3);
		// std::cout << "[model] link " << i << " torque " << link_torques[i].transpose() << std::endl;
	}

	// add lefted generalized force
	Q += generalized_force;
	return Q;
}

cRobotCollider* cRobotModelDynamics::GetLinkCollider(int link_id)
{
	if (link_id < 0 || link_id >= multibody_colliders.size())
	{
		std::cout << "[error] GetLinkCollider invalid link id " << link_id << std::endl;
		exit(1);
	}
	return multibody_colliders[link_id];
}

void cRobotModelDynamics::ClearForce()
{
	for (int i = 0; i < GetNumOfLinks(); i++)
	{
		link_forces[i].setZero();
		link_torques[i].setZero();
	}
	generalized_force.setZero();
}

template <typename T1, typename T2, typename T3>
void Solve(const T1& solver, const T2& mat, const T3& residual, T3& result, double& error)
{
	result = solver.solve(residual);
	error = (mat * result - residual).norm();
}

template <typename T1, typename T2>
T2 AccurateSolve(const T1& mat, const T2& residual, double& min_error)
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
		Solve(Eigen::ColPivHouseholderQR<T1>(mat), mat, residual, my_result, my_error);
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
		Solve(Eigen::FullPivHouseholderQR<T1>(mat), mat, residual, my_result, my_error);
		// std::cout << "FullPivHouseholderQR error = " << my_error << std::endl;
		if (min_error > my_error)
		{
			name = "FullPivHouseholderQR";
			final_result = my_result;
			min_error = my_error;
		}
	}
	// 6. BDCSVD
	{
		Solve(Eigen::BDCSVD<T1>(mat, Eigen::ComputeThinU | Eigen::ComputeThinV), mat, residual, my_result, my_error);
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
		Solve(Eigen::JacobiSVD<T1>(mat, Eigen::ComputeThinU | Eigen::ComputeThinV), mat, residual, my_result, my_error);
		// std::cout << "JacobiSVD error = " << my_error << std::endl;
		if (min_error > my_error)
		{
			name = "JacobiSVD";
			final_result = my_result;
			min_error = my_error;
		}
	}

	std::cout << "here we use " << name << " method, error = " << min_error << std::endl;
	return final_result;
}

// time integrate
extern int global_frame_id;
extern bool gPauseSimulation;
void cRobotModelDynamics::UpdateVelocity(double dt, bool verbose /* = false*/)
{
	// std::cout << "----------------update vel begin-----------------\n";
	// 1. calculate generalized force
	int n = GetNumOfFreedom();
	tVectorXd Q = GetGeneralizedForce();

	// std::cout << "Q = " << Q.transpose() << std::endl;
	// 2. calculate qddot and new qdot
	// tMatrixXd mass_matrix_inv = (mass_matrix + 1e-5 * tMatrixXd::Identity(mass_matrix.rows(), mass_matrix.cols())).inverse();
	// tVectorXd residual = (Q - (coriolis_matrix + damping_matrix) * qdot);
	tVectorXd qddot = Getqddot();

	if (qddot.hasNaN())
	{
		std::cout << "RobotModel::UpdateVel: qddot hasNan\n";
		exit(0);
	}
	if (verbose)
	{
		// std::cout << "[model] qddot = " << qddot.transpose() << std::endl;
		// std::cout << "[model] Q = " << Q.transpose() << std::endl;
	}
	qdot += qddot * dt;
	qdot = qdot.cwiseMax(-max_vel);
	qdot = qdot.cwiseMin(max_vel);
	// 3. recalculate the coriolis force
	ComputeCoriolisMatrix(qdot);
	UpdateCartesianVelocity();
	// std::cout << "----------------update vel end-----------------\n";
}

// time integrate
void cRobotModelDynamics::UpdateTransform(double dt)
{
	// 1. calculate new q and update links and joints
	// std::ofstream fout(path, std::ios::app);
	// fout << "frame " << global_frame_id << " qdot[4] = " << qdot[4] << std::endl;
	// fout << "frame " << global_frame_id << " qold[4] = " << q[4] << std::endl;
	// fout << "frame " << global_frame_id << " dt = " << dt << std::endl;
	q = q + dt * qdot;
	// fout << "frame " << global_frame_id << " qnew[4] = " << q[4] << std::endl;

	if (clamp_euler_angle)
	{
		for (int i = 0; i < q.size(); i++)
		{
			while (q[i] < -M_PI) q[i] += 2 * M_PI;
			while (q[i] > M_PI) q[i] -= 2 * M_PI;
		}
	}

	Apply(q, true);

	// 2. recalc mats
	ComputeMassMatrix();
	ComputeCoriolisMatrix(qdot);
	ComputeDampingMatrix();
	UpdateCartesianVelocity();

	// 3. write to bullet
	SyncToBullet();
	// fout << "frame " << global_frame_id << " qfinal[4] = " << q[4] << std::endl;
	// fout << std::endl;
	// fout.close();
}

// only position is saved in bullet
void cRobotModelDynamics::SyncToBullet()
{
	btTransform trans;
	for (int i = 0; i < GetNumOfLinks(); i++)
	{
		auto link = GetLinkById(i);
		trans.setOrigin(btVector3(link->GetWorldPos()[0], link->GetWorldPos()[1], link->GetWorldPos()[2]));
		trans.setRotation(cBulletUtil::tQuaternionTobtQuaternion(cMathUtil::RotMatToQuaternion(link->GetGlobalTransform())));

		multibody_colliders[i]->setWorldTransform(trans);
	}
}

void cRobotModelDynamics::PushState(const std::string& tag, bool only_vel_and_force)
{
	if (mStateStack.size() >= mStackCapacity)
	{
		std::cout << "[error] cRobotModelDynamics::PushState exceed length " << mStackCapacity << " for tag " << tag;
		exit(1);
	}
	tStateRecord* state = new tStateRecord();
	state->only_vel_and_force_recorded = only_vel_and_force;
	state->qdot = qdot;
	state->link_forces = link_forces;
	state->link_torques = link_torques;

	state->damping_matrix = damping_matrix;
	state->coriolis_matrix = coriolis_matrix;
	state->generalized_force = generalized_force;

	if (only_vel_and_force == false)
	{
		state->q = q;
		state->mass_matrix = mass_matrix;
	}

	mStateStack.push_back(std::make_pair(tag, state));
}

void cRobotModelDynamics::PopState(const std::string& tag, bool only_vel_and_force)
{
	if (mStateStack.size() == 0)
	{
		std::cout << "[error] RobotModel stack is empty when you try to pop " << tag << std::endl;
		exit(1);
	}
	else if (mStateStack.back().first != tag)
	{
		std::cout << "[error] RobotModel stack trying to pop " << mStateStack[mStateStack.size() - 1].first << " but the user try to do " << tag << std::endl;
		exit(1);
	}

	auto state = mStateStack.back().second;
	if (state->only_vel_and_force_recorded != only_vel_and_force)
	{
		std::cout << "[error] RobotModel stack trying to pop " << state->only_vel_and_force_recorded << " but the user try to do " << only_vel_and_force << std::endl;
		exit(1);
	}

	qdot = state->qdot;
	link_forces = state->link_forces;
	link_torques = state->link_torques;
	generalized_force = state->generalized_force;

	coriolis_matrix = state->coriolis_matrix;
	damping_matrix = state->damping_matrix;

	if (only_vel_and_force == false)
	{
		state->q = q;
		state->mass_matrix = mass_matrix;
		Apply(q, true);
		ComputeMassMatrix();
	}

	ComputeCoriolisMatrix(qdot);
	ComputeDampingMatrix();
	UpdateCartesianVelocity();

	delete state;
	mStateStack.pop_back();
}

bool cRobotModelDynamics::IsMaxVel()
{
	return std::fabs((qdot.cwiseAbs().maxCoeff() - max_vel)) < 1;
}

void cRobotModelDynamics::SetDampingCoeff(double damp1, double damp2)
{
	damping_coef1 = damp1;
	damping_coef2 = damp2;
	// std::cout << "damping = " << damping_coef << std::endl;
	// exit(1);
}

void cRobotModelDynamics::SetAngleClamp(bool val)
{
	this->clamp_euler_angle = val;
}

void cRobotModelDynamics::SetMaxVel(double val)
{
	max_vel = val;
}

/**
 * \brief			Update generalized velocity by RK4
*/
void cRobotModelDynamics::UpdateRK4(double dt)
{
	// std::cout <<"update rk4";
	// exit(1);
	tVectorXd k1(num_of_freedom * 2), k2(num_of_freedom * 2), k3(num_of_freedom * 2), k4(num_of_freedom * 2);
	tVectorXd y_dot = qdot, y = q;
	tVectorXd Q;
	tVectorXd qddot;
	// k1: we need to recalculate M, recalculate Q, recalculate C by given
	{
		UpdateRK4InternalUpdate(y, y_dot, Q);
		qddot = mass_matrix.inverse() * (Q - coriolis_matrix * y_dot);

		k1.segment(0, num_of_freedom) = y_dot + dt * qddot;
		k1.segment(num_of_freedom, num_of_freedom) = qddot;

		k1 *= dt;
	}

	{
		y += 0.5 * k1.segment(0, num_of_freedom);
		y_dot += 0.5 * k1.segment(num_of_freedom, num_of_freedom);
		UpdateRK4InternalUpdate(y, y_dot, Q);

		qddot = mass_matrix.inverse() * (Q - coriolis_matrix * y_dot);

		k2.segment(0, num_of_freedom) = y_dot + dt * qddot;
		k2.segment(num_of_freedom, num_of_freedom) = qddot;

		k2 *= dt;
	}

	{
		y += 0.5 * k2.segment(0, num_of_freedom);
		y_dot += 0.5 * k2.segment(num_of_freedom, num_of_freedom);
		UpdateRK4InternalUpdate(y, y_dot, Q);
		qddot = mass_matrix.inverse() * (Q - coriolis_matrix * y_dot);

		k3.segment(0, num_of_freedom) = y_dot + dt * qddot;
		k3.segment(num_of_freedom, num_of_freedom) = qddot;

		k3 *= dt;
	}

	{
		y += k3.segment(0, num_of_freedom);
		y_dot += k3.segment(num_of_freedom, num_of_freedom);
		UpdateRK4InternalUpdate(y, y_dot, Q);

		qddot = mass_matrix.inverse() * (Q - coriolis_matrix * y_dot);

		k4.segment(0, num_of_freedom) = y_dot + dt * qddot;
		k4.segment(num_of_freedom, num_of_freedom) = qddot;
		k4 *= dt;
	}

	q = q + (k1.segment(0, num_of_freedom) + 2 * k2.segment(0, num_of_freedom) + 2 * k3.segment(0, num_of_freedom) + k4.segment(0, num_of_freedom)) / 6;

	qdot = qdot + (k1.segment(num_of_freedom, num_of_freedom) + 2 * k2.segment(num_of_freedom, num_of_freedom) + 2 * k3.segment(num_of_freedom, num_of_freedom) + k4.segment(num_of_freedom, num_of_freedom)) / 6;

	qdot = qdot.cwiseMax(-max_vel);
	qdot = qdot.cwiseMin(max_vel);

	// apply and clear all forces
	UpdateRK4InternalUpdate(q, qdot, Q);
	UpdateCartesianVelocity();
	ClearForce();
	SyncToBullet();
}

void cRobotModelDynamics::UpdateRK4InternalUpdate(tVectorXd& q, tVectorXd& qdot, tVectorXd& Q)
{
	Apply(q, true);
	ComputeMassMatrix();
	ComputeCoriolisMatrix(qdot);
	Q.resize(num_of_freedom);
	Q.setZero();

	int n = GetNumOfFreedom();
	for (int i = 0; i < GetNumOfLinks(); i++)
	{
		const auto& link = GetLinkById(i);
		Q += link->GetJKv().transpose() * link_forces[i].segment(0, 3);
		Q += link->GetJKw().transpose() * link_torques[i].segment(0, 3);
	}
}

void cRobotModelDynamics::GetEffectInfo(tEigenArr<tVector>& force_array, tEigenArr<tVector>& torque_array)
{
	force_array = this->link_forces;
	torque_array = this->link_torques;
}

tMatrixXd cRobotModelDynamics::GetInvMassMatrix()
{
	return inv_mass_matrix;
}

tVectorXd cRobotModelDynamics::Getqddot()
{
	tVectorXd residual = (GetGeneralizedForce() - (coriolis_matrix + damping_matrix) * qdot);
	tVectorXd qddot = inv_mass_matrix * residual;
	return qddot;
}

void cRobotModelDynamics::UpdateVelocityWithoutCoriolis(double dt)
{
	int n = GetNumOfFreedom();
	tVectorXd residual = GetGeneralizedForce();
	tVectorXd qddot = inv_mass_matrix * residual;

	if (qddot.hasNaN())
	{
		std::cout << "RobotModel::UpdateVel: qddot hasNan\n";
		exit(0);
	}
	qdot += qddot * dt;
	qdot = qdot.cwiseMax(-max_vel);
	qdot = qdot.cwiseMin(max_vel);

	// 3. recalculate the coriolis force
	ComputeCoriolisMatrix(qdot);

	// 4. update the cartesian velocity for each link
	UpdateCartesianVelocity();
}

/**
 * \brief				Update cartesian velocity (lin vel and ang vel) for each link by the generalized velocity qdot
 * 
 * 		v = Jv qdot
 * 		w = Jw qdot
*/
void cRobotModelDynamics::UpdateCartesianVelocity()
{
	int num_of_links = GetNumOfLinks();
	for (int i = 0; i < num_of_links; i++)
	{
		auto link = static_cast<Link*>(GetLinkById(i));
		link->SetLinkVel((link->GetJKv() * qdot).segment(0, 3));
		link->SetLinkOmega((link->GetJKw() * qdot).segment(0, 3));
	}
}

/**
 * \brief			test how to rotate / move the whole character
 * 
*/
#define expand(a) cMathUtil::Expand(a, 0)
void cRobotModelDynamics::TestRotationChar()
{
	std::cout << "begin to test rotate char\n";
	q.setRandom();
	qdot.setRandom();
	SetPose(q, qdot);

	// // get current root link rotation, vel and omega
	// 1. record new info
	int num_of_links = GetNumOfLinks();
	auto link = static_cast<Link*>(GetLinkById(0));
	tMatrix old_rotation = tMatrix::Zero();
	old_rotation.block(0, 0, 3, 3) = link->GetWorldOrientation();
	tVector old_link_vel = expand(link->GetLinkVel()),
			old_link_omega = expand(link->GetLinkOmega());

	// 2. set up new rotation, calculate new vel and new omega
	tVector random_rot_euler_angles = tVector::Random();
	tMatrix diff_rot = cMathUtil::EulerAnglesToRotMat(random_rot_euler_angles, eRotationOrder::XYZ);
	// std::cout << "root link old rot = \n"
	// 		  << old_rotation << std::endl;
	// std::cout << "root link diff rot = \n"
	// 		  << diff_rot << std::endl;
	// std::cout << "root link init rot = \n"
	// 		  << link->GetMeshRotation() << std::endl;

	tMatrix new_rotation = diff_rot * old_rotation * cMathUtil::ExpandMat(link->GetMeshRotation()).transpose();
	std::cout << "new rot = \n"
			  << new_rotation << std::endl;

	tVector new_euler_angles = cMathUtil::QuaternionToEulerAngles(cMathUtil::RotMatToQuaternion(new_rotation), eRotationOrder::XYZ);
	tVector new_link_vel = diff_rot * old_link_vel;  // diff rotation in world frame
	tVector new_link_omega = diff_rot * old_link_omega;

	std::vector<tVector3d> old_link_vel_array(num_of_links, tVector3d::Zero()), old_link_omega_array(num_of_links, tVector3d::Zero());
	std::vector<tMatrix3d> old_rotation_array(num_of_links, tMatrix3d::Zero());
	for (int i = 0; i < num_of_links; i++)
	{
		auto link = static_cast<Link*>(GetLinkById(i));
		old_rotation_array[i] = link->GetWorldOrientation();
		old_link_omega_array[i] = link->GetLinkOmega();
		old_link_vel_array[i] = link->GetLinkVel();
	}

	// 3. given new q into the result and update
	q.segment(3, 3) = new_euler_angles.segment(0, 3);
	Apply(q, true);

	// 4. given new link vel to qdot

	std::cout << "root link jkv = \n"
			  << link->GetJKv() << std::endl;
	qdot.segment(3, 3) = link->GetJKw().block(0, 3, 3, 3).inverse() * new_link_omega;
	qdot.segment(0, 3) = new_link_vel.segment(0, 3) - link->GetJKv().block(0, 3, 3, 3) * qdot.segment(3, 3);
	SetPose(q, qdot);

	// 5. begin to check
	{
		for (int i = 0; i < num_of_links; i++)
		{
			auto link = static_cast<Link*>(GetLinkById(i));
			tMatrix3d new_rot = link->GetWorldOrientation();
			tVector3d new_omega = link->GetLinkOmega();
			tVector3d new_vel = link->GetLinkVel();
			std::cout << "link " << i << " rot diff = \n"
					  << new_rot - diff_rot.block(0, 0, 3, 3) * old_rotation_array[i] << std::endl;
			std::cout << "link " << i << " vel diff = \n"
					  << new_vel - diff_rot.block(0, 0, 3, 3) * old_link_vel_array[i] << std::endl;
			std::cout << "link " << i << " omega diff = \n"
					  << new_omega - diff_rot.block(0, 0, 3, 3) * old_link_omega_array[i] << std::endl;
		}
	}
	exit(1);
}