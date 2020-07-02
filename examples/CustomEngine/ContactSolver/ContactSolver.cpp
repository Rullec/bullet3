#include "ContactSolver.h"
#include "LCPSolver/LCPSolver.h"
// #include "QPSolver/QPSolver.h"
#include "MatlabQPSolver/MatlabQPSolver.h"
#include "../SimObj.h"
#include "../Model/RobotCollider.h"
#include "../Model/RobotModel.h"
#include "../../ExampleBrowser/ID_test/FileUtil.h"
#include "btBulletDynamicsCommon.h"
#include <iostream>
// const std::string rigidbody_path = "rigidbody.txt";
// const std::string multibody_path = "multibody.txt";

// int contact_times = 0;

cSimRigidBody* UpcastRigidBody(const btCollisionObject* col)
{
	return const_cast<cSimRigidBody*>(dynamic_cast<const cSimRigidBody*>(col));
}

cRobotCollider* UpcastRobotCollider(const btCollisionObject* col)
{
	return const_cast<cRobotCollider*>(dynamic_cast<const cRobotCollider*>(col));
}
cCollisionObject* UpcastColObj(const btCollisionObject* col)
{
	return const_cast<cCollisionObject*>(dynamic_cast<const cCollisionObject*>(col));
}

cContactSolver::tParams::tParams()
{
	mNumFrictionConeDirs = 4;
	mMu = 0.5;
}

cContactSolver::cContactSolver(const cContactSolver::tParams& param)
{
	mNumFrictionDirs = param.mNumFrictionConeDirs;
	mWorld = param.mWorld;
	mMu = param.mMu;
	if (mMu < 0.1)
	{
		std::cout << "cContactSolver mMu should not be " << mMu << std::endl;
		exit(1);
	}
	dt = 1e-3;
	mEnableMultibodySelfCol = false;
	mEnableTest = false;
	mLCPSolver = nullptr;

	// test
	// mQPSolver = new cQPSolver();
}

cContactSolver::~cContactSolver()
{
	DeleteColObjData();
}

void cContactSolver::ConstraintProcess(float dt_)
{
	dt = dt_;
	ConstraintSetup();
	ConstraintSolve();
	ConstraintFinished();
}
/**
 * \brief        Set up the LCP contact constraint
 *          Calculate the LCP constraint tMatrixXd A and vec b, so that:
 *                  0 <= x \cdot (Ax + b) >= 0
*/
void cContactSolver::ConstraintSetup()
{
	// 1. build collision objects data
	RebuildColObjData();
	// for (int i = 0; i < mMap_collisionid_to_groupid.size(); i++)
	// {
	// 	std::cout << "col obj " << i << " belongs to group " << mMap_collisionid_to_groupid[i] << std::endl;
	// }
	// exit(0);

	// 2. get all contact points' info
	btDispatcher* dispatcher = mWorld->getDispatcher();
	int n_manifolds = dispatcher->getNumManifolds();
	for (int i = 0; i < n_manifolds; i++)
	{
		const auto& manifold = dispatcher->getManifoldByIndexInternal(i);
		AddManifold(manifold);
	}

	// if here is a contact
	int contact_size = mContactConstraintData.size();
	int self_contact_size = 0;
	for (int i = 0; i < contact_size; i++)
	{
		if (mContactConstraintData[i]->mIsSelfCollision) self_contact_size++;
	}
	if (contact_size != 0)
	{
		// 3. calculate velocity convert matrix for each contact point (united by collision groups)
		CalcAbsvelConvertMat();

		if (mEnableTest) TestCartesianForceToCartesianVel();

		// 4. calculate the relative velocity convert matrix
		CalcRelvelConvertMat();
		// if (contact_size > 5)
		if (mEnableTest) TestCartesianForceToCartesianRelVel(rel_vel_convert_mat, rel_vel_convert_vec);

		// 5. calculate the relative normal/tangent convert matrix
		CalcDecomposedConvertMat();
		// if (2 == contact_size)
		if (mEnableTest) TestCartesianForceToNormalAndTangetRelVel(normal_vel_convert_mat,
																   normal_vel_convert_vec,
																   tangent_vel_covert_mat,
																   tangent_vel_convert_vec);

		// 6. change base for decomposed convert matrix: the original base is cartesian force, now it is [fn, f_4dirs, \lambda]
		CalcResultVectorBasedConvertMat();
		// if (2 == contact_size)
		if (mEnableTest) TestCartesianForceToNormalAndTangetResultBasedRelVel(normal_vel_convert_result_based_mat,
																			  normal_vel_convert_result_based_vec,
																			  tangent_vel_covert_result_based_mat,
																			  tangent_vel_convert_result_based_vec);
	}
}
extern std::map<int, std::string> col_name;
void cContactSolver::ConstraintSolve()
{
	if (mNumFrictionDirs != 4)
	{
		std::cout << "error now friction dirs must be 4, otherwise the convert tMatrixXd S is illegal\n";
		exit(1);
	}

	if (mContactConstraintData.size() == 0) return;
	// construct a LCP problem: (x * Mx+n) = 0, x>=0, Mx+n >=0
	// 1. shape the vecs and mats:
	int contact_size = mContactConstraintData.size();
	int single_size = (mNumFrictionDirs + 2);
	int final_shape = contact_size * single_size;
	M.resize(final_shape, final_shape), M.setZero();
	x.resize(final_shape), x.setZero();
	n.resize(final_shape), n.setZero();

	// 2. fill in
	tMatrixXd unit_M(single_size, single_size);
	tVectorXd unit_n(single_size);
	tMatrixXd C_lambda(mNumFrictionDirs, single_size), C_mufn(1, single_size), C_c(1, single_size);
	CalcCMats(C_lambda, C_mufn, C_c);

	// 3. construct LCP problem
	tMatrixXd line_M(single_size, final_shape);
	tVectorXd line_n(single_size);
	for (int i = 0; i < contact_size; i++)
	{
		line_M.setZero();
		line_n.setZero();

		// 3.1 basic M and basic n
		line_M.block(0, 0, 1, final_shape) = normal_vel_convert_result_based_mat.block(i, 0, 1, final_shape);
		line_M.block(1, 0, mNumFrictionDirs, final_shape) = tangent_vel_covert_result_based_mat.block(i * mNumFrictionDirs, 0, mNumFrictionDirs, final_shape);

		line_n.segment(0, 1) = normal_vel_convert_result_based_vec.block(i, 0, 1, 1);
		line_n.segment(1, mNumFrictionDirs) = tangent_vel_convert_result_based_vec.segment(i * mNumFrictionDirs, mNumFrictionDirs);

		// 3.2 add C-related terms

		line_M.block(1, i * single_size, mNumFrictionDirs, single_size) += C_lambda;
		line_M.block(single_size - 1, i * single_size, 1, single_size) += C_mufn - C_c;

		// 3.3 give line M and line n to total M and n
		M.block(i * single_size, 0, single_size, final_shape) = line_M;
		n.segment(i * single_size, single_size) = line_n;
	}
	// std::cout << " M = \n " << M << std::endl;
	// std::cout << " n = \n " << n.transpose() << std::endl;
	// std::ofstream fout("lcp_record.txt", std::ios::app);
	// int ret = mLCPSolver->Solve(M, n, x);
	int ret = mLCPSolver->Solve(M, n, x, 0.5, 4, true);
	// fout << x.size() << std::endl;
	// fout << M << std::endl;
	// fout << n.transpose() << std::endl;
	// fout << x.transpose() << std::endl;
	// fout <<"----------------------\n";
	// fout.close();
	// ConstraintSolveByQP();
	// ConstraintSolveByMatlabQP();
	cMathUtil::RoundZero(M);
	cMathUtil::RoundZero(n);
	// std::cout << " M = \n " << M << std::endl;
	// std::cout << " n = \n " << n.transpose() << std::endl;
	if (ret != false)
	{
		// std::cout << "contact size = " << contact_size << std::endl;
		// std::cout << " M = \n " << M << std::endl;
		// std::cout << " n = \n " << n.transpose() << std::endl;
		std::cout << "solved failed\n";
		// exit(0);
	}
}
void cContactSolver::ConstraintSolveByMatlabQP()
{
	mMatlabQPSolver = new cMatlabQPSolver();
	int num_vars = x.size();
	tMatrixXd H, Aeq, Aineq;
	tVectorXd f, beq, bineq;
	tVectorXd x_qp = tVectorXd::Zero(num_vars);
	// 1. set H and f
	// func: x' * M * x + n
	H = 2 * M;
	f = n;

	// 2. set up inequality cons
	int num_ineq = 2 * num_vars;
	Aineq.resize(num_ineq, num_vars);
	Aineq.setZero();
	bineq.resize(num_ineq);
	bineq.setZero();
	Aineq.block(0, 0, num_vars, num_vars) = -M;
	bineq.segment(0, num_vars) = n;
	Aineq.block(num_vars, 0, num_vars, num_vars) = -1 * tMatrixXd::Identity(num_vars, num_vars);
	bineq.segment(num_vars, num_vars).setZero();

	// 3. set up equality cons
	int num_eq = 0;
	Aeq.resize(num_eq, num_vars);
	Aeq.setZero();
	beq.resize(num_eq);
	beq.setZero();

	// min 0.5*x'*H*x + f'*x   subject to:  Aineq*x <= bineq, Aeq * x = beq
	mMatlabQPSolver->Solve(num_vars, H, f, Aeq, beq, Aineq, bineq, 100, x_qp);
	std::cout << "x matlab qp = " << x_qp.transpose() << std::endl;
	std::cout << "x lcp = " << x.transpose() << std::endl;
	exit(1);
}
void cContactSolver::ConstraintSolveByQP()
{
	// tMatrixXd H, Aineq, Aeq;
	// tVectorXd f, bineq, beq;
	// int size = x.size();
	// x_qp = x;
	// /*
	// 	func: x.T * M * x + n.T * x
	// 	H * 2 * M
	// 	f = n
	// */
	// // 1. init H and f
	// H = 2 * M;
	// f = n;

	// // 2. equality cons
	// int num_eq = 0;
	// Aeq.resize(num_eq, size);
	// Aeq.setZero();
	// beq.resize(num_eq);
	// beq.setZero();
	// Aeq.transposeInPlace();

	// // 3. ineq cons
	// int num_ineq = 2 * size;
	// Aineq.resize(num_ineq, size);
	// Aineq.setZero();
	// bineq.resize(num_ineq);
	// bineq.setZero();

	// // 3.1 M * x + n \ge 0
	// Aineq.block(0, 0, size, size) = M;
	// bineq.segment(0, size) = n;

	// // 3.2 x \ge 0
	// Aineq.block(size, 0, size, size).setIdentity();
	// bineq.segment(size, size).setZero();
	// Aineq.transposeInPlace();

	// // 4. solve and compare
	// /*
	// 	func: 0.5 * x.T * H * x + f.T * x

	// 	Aeq.T * x + beq = 0
	// */
	// // mQPSolver->Solve(size, H, f, Aeq, beq, Aineq, bineq, x_qp);
	// std::cout << "x = " << x.transpose() << std::endl;
	// std::cout << "x_qp = " << x_qp.transpose() << std::endl;
	// exit(1);
}

#include <Eigen/Eigenvalues>
void cContactSolver::ConstraintFinished()
{
	int contact_size = mContactConstraintData.size();
	int single_size = (mNumFrictionDirs + 2);

	double total_contact_force_norm = 0;
	for (int i = 0; i < contact_size; i++)
	{
		if (0 == i)
		{
			std::cout << "--- num contact pt = " << contact_size << std::endl;
			// Eigen::EigenSolver<tMatrixXd> solver(M);

			// tVectorXd eigen_values = solver.eigenvalues().real();
			// std::cout << "[log] M eigen value = " << eigen_values.transpose() << std::endl;
			// std::cout << "[log] min eigen value = " << eigen_values.minCoeff() << std::endl;
			// std::cout << "[finish] x = " << x.transpose() << std::endl;
			// std::cout << "[finish] w = " << (M * x + n).transpose() << std::endl;
		}
		auto& data = mContactConstraintData[i];
		tVectorXd x_unit = x.segment(i * single_size, single_size);

		// tVector contact_force = (data.GetConvertMatS(data.mId0) * x_unit).segment(0, 4) / 4;
		tVector contact_force = cMathUtil::Expand((data->mS * x_unit), 0);
		// std::cout << "x unit " << i << " = " << x_unit.transpose() << std::endl;
		// tVector contact_force = cMathUtil::Expand((data->mS * x_unit), 0);
		std::cout << "contact " << i << " force = " << contact_force.transpose() << std::endl;
		total_contact_force_norm += contact_force.norm();
		data->ApplyForceCartersian(contact_force);
	}
	// std::cout << "LCP contact update vel begin\n";
	UpdateVelocity(dt);
	// std::cout << "LCP contact update vel end\n";

	// check data velocity
	bool output_contact_result = false;
	if (total_contact_force_norm < 1e-6 && contact_size)
	{
		output_contact_result = true;
		std::cout << "[warn] total contact force is zero\n";
	}
	for (int i = 0; i < contact_size; i++)
	{
		auto& data = mContactConstraintData[i];
		if (output_contact_result)
		{
			std::cout << "[post] contact " << i << " rel vel = " << data->GetRelVel().transpose() << std::endl;
		}
	}
	// if (contact_size == 2)
	// {
	// 	// std::cout << "-----------currrent contact = 2\n";
	// 	// std::cout << "x = " << x.transpose() << std::endl;
	// 	// std::cout << "M = \n " << M << std::endl;
	// 	// std::cout << "n = \n " << n.transpose() << std::endl;
	// 	// tVectorXd w = M * x + n;
	// 	// std::cout << "w = \n"
	// 	// 		  << w.transpose() << std::endl;
	// 	for (int i = 0; i < contact_size; i++)
	// 	{
	// 		auto& data = mContactConstraintData[i];
	// 		tVectorXd x_unit = x.segment(i * single_size, single_size);
	// 		// std::cout << "contact " << i << " between " << data->mBodyA->GetName() << " " << data->mBodyB->GetName() << std::endl;
	// 		// tVector contact_force = cMathUtil::Expand((data->GetConvertMatS(data->mBodyId0) * x_unit), 0);
	// 		tVector contact_force = cMathUtil::Expand((data->mS * x_unit), 0);
	// 		std::cout << "contact force " << i << " = " << contact_force.transpose() << std::endl;
	// 		// data->ApplyForceCartersian(contact_force);
	// 	}
	// 	// exit(1);
	// }
	DeleteContactData();
}

void cContactSolver::CalcCMats(tMatrixXd& C_lambda, tMatrixXd& C_mufn, tMatrixXd& C_c)
{
	int single_size = (mNumFrictionDirs + 2);
	C_lambda.resize(mNumFrictionDirs, single_size), C_mufn.resize(1, single_size), C_c.resize(1, single_size);
	C_lambda.setZero(), C_mufn.setZero(), C_c.setZero();

	for (int i = 0; i < mNumFrictionDirs; i++)
	{
		C_lambda(i, single_size - 1) = 1;
	}

	C_mufn(0, 0) = mMu;
	for (int i = 0; i < mNumFrictionDirs; i++) C_c(0, 1 + i) = 1;
}

void cContactSolver::RebuildColObjData()
{
	int n_objs = mWorld->getNumCollisionObjects();
	mColGroupData.clear();
	mMap_collisionid_to_groupid.resize(n_objs);

	int n_group = 0;
	std::map<cRobotModel*, int> model_set;
	for (int i = 0; i < n_objs; i++)
	{
		cCollisionObject* obj = UpcastColObj(mWorld->getCollisionObjectArray()[i]);
		bool add_entry = false;
		switch (obj->GetType())
		{
			case eColObjType::Rigidbody:
				add_entry = true;
				break;
			case eColObjType::RobotCollder:
			{
				cRobotModel* model = UpcastRobotCollider(obj)->mModel;
				std::map<cRobotModel*, int>::iterator it = model_set.find(model);
				if (it == model_set.end())
				{
					// cannot find the group, have a new group
					add_entry = true;
					model_set[model] = n_group;
				}
				else
				{
					mMap_collisionid_to_groupid[i] = it->second;
					continue;
				}
				break;
			}
			default:
				break;
		}

		if (add_entry)
		{
			mMap_collisionid_to_groupid[i] = n_group;
			mColGroupData.push_back(new tCollisionObjData(obj));
			n_group++;
		}
	}
}

void cContactSolver::DeleteColObjData()
{
	for (auto& i : mColGroupData) delete i;
	mColGroupData.clear();
}
void cContactSolver::DeleteContactData()
{
	for (auto& i : mContactConstraintData) delete i;
	mContactConstraintData.clear();
}

void cContactSolver::CalcAbsvelConvertMat()
{
	// for (auto& i : mColGroupData)
	for (int i = 0; i < mColGroupData.size(); i++)
	{
		auto& data = mColGroupData[i];
		data->Setup(mContactConstraintData.size());
		// std::cout << "[abs convert] outside convert mat for group " << i << ": \n"
		// 		  << data->mConvertCartesianForceToVelocityMat << std::endl;
		// std::cout << "[abs convert] outside convert vec for group " << i << ": \n"
		// 		  << data->mConvertCartesianForceToVelocityVec.transpose() << std::endl;
	}
}
void cContactSolver::CalcRelvelConvertMat()
{
	int n_contact = mContactConstraintData.size();
	rel_vel_convert_mat.resize(3 * n_contact, 3 * n_contact);
	rel_vel_convert_vec.resize(3 * n_contact);

	for (int i = 0; i < n_contact; i++)
	{
		auto& data = mContactConstraintData[i];
		if (data->mIsSelfCollision == false)
		{
			// get body0 and body1's collision group
			int body0_groupid = mMap_collisionid_to_groupid[data->mBodyId0],
				body1_groupid = mMap_collisionid_to_groupid[data->mBodyId1];

			// u = v_0 - v_1 = A_0f_0 + a_0 - (A_1f_0 + a_1)
			//  = (A_0 - A_1)f_0 + (a_0 - a_1)
			// here is "mat0 + mat1", because the pair of force applied is oppo to each other
			rel_vel_convert_mat.block(i * 3, 0, 3, 3 * n_contact) =
				mColGroupData[body0_groupid]->mConvertCartesianForceToVelocityMat.block(i * 3, 0, 3, 3 * n_contact);
			if (mColGroupData[body1_groupid]->mBody->IsStatic() == false)
				rel_vel_convert_mat.block(i * 3, 0, 3, 3 * n_contact) -= mColGroupData[body1_groupid]->mConvertCartesianForceToVelocityMat.block(i * 3, 0, 3, 3 * n_contact);

			// here vec is normally "minus", "vec0 - vec1"
			rel_vel_convert_vec.segment(3 * i, 3) = mColGroupData[body0_groupid]->mConvertCartesianForceToVelocityVec.segment(i * 3, 3);
			if (mColGroupData[body1_groupid]->mBody->IsStatic() == false)
				rel_vel_convert_vec.segment(3 * i, 3) -= mColGroupData[body1_groupid]->mConvertCartesianForceToVelocityVec.segment(i * 3, 3);
		}
		else
		{
			// get body0 and body1's collision group
			int single_groupid = mMap_collisionid_to_groupid[data->mBodyId0];

			// for self collision, the rel vel convert mat and vec has been calculated well.
			rel_vel_convert_mat.block(i * 3, 0, 3, 3 * n_contact) =
				mColGroupData[single_groupid]->mConvertCartesianForceToVelocityMat.block(i * 3, 0, 3, 3 * n_contact);
			rel_vel_convert_vec.segment(3 * i, 3) = mColGroupData[single_groupid]->mConvertCartesianForceToVelocityVec.segment(i * 3, 3);
		}
	}
	// std::cout << "rel vel convert mat = \n"
	// 		  << rel_vel_convert_mat << std::endl;
	// std::cout << "rel vel convert vec = " << rel_vel_convert_vec.transpose() << std::endl;
}
void cContactSolver::CalcDecomposedConvertMat()
{
	if (mNumFrictionDirs != 4)
	{
		std::cout << "unsupported friction dirs = " << mNumFrictionDirs << std::endl;
		exit(1);
	}
	int n_contact = mContactConstraintData.size();
	int x_size = 3 * n_contact;  // x is the collection of contact force

	normal_vel_convert_mat.resize(n_contact, x_size);                     // one contact point one line
	normal_vel_convert_vec.resize(n_contact);                             // one contact point one real number
	tangent_vel_covert_mat.resize(mNumFrictionDirs * n_contact, x_size);  // one contact point N lines, N = mNumFrictionDir
	tangent_vel_convert_vec.resize(mNumFrictionDirs * n_contact);         // one contact point one real number

	for (int i = 0; i < n_contact; i++)
	{
		tVector3d normal = mContactConstraintData[i]->mNormalPointToA.segment(0, 3);

		// normal_rel_vel_mat = n^T * rel_vel_mat
		normal_vel_convert_mat.block(i, 0, 1, x_size) =
			normal.transpose() *
			rel_vel_convert_mat.block(i * 3, 0, 3, x_size);
		normal_vel_convert_vec.segment(i, 1) = normal.transpose() * rel_vel_convert_vec.segment(i * 3, 3);

		// tangent_rel_vel_mat = D^T * (rel_vel_mat - normal_rel_vel_mat)
		tangent_vel_covert_mat.block(mNumFrictionDirs * i, 0, mNumFrictionDirs, x_size) =
			mContactConstraintData[i]->mD.transpose() *
			(rel_vel_convert_mat.block(3 * i, 0, 3, x_size) - normal * normal_vel_convert_mat.block(i, 0, 1, x_size));

		tangent_vel_convert_vec.segment(mNumFrictionDirs * i, mNumFrictionDirs) =
			mContactConstraintData[i]->mD.transpose() *
			(rel_vel_convert_vec.segment(3 * i, 3) - normal * normal_vel_convert_vec.segment(i, 1));
	}
}

void cContactSolver::AddManifold(btPersistentManifold* manifold)
{
	int n_contacts = manifold->getNumContacts();

	tContactPointData* data = nullptr;
	// determine collision object type

	// for each contact point
	for (int j = 0; j < n_contacts; j++)
	{
		data = new tContactPointData(mContactConstraintData.size(), dt);

		// 1. prepare contact point data
		{
			data->dt = this->dt;
			data->mBodyA = UpcastColObj(manifold->getBody0());
			data->mBodyB = UpcastColObj(manifold->getBody1());
			data->mTypeA = data->mBodyA->GetType();
			data->mTypeB = data->mBodyB->GetType();

			if (data->mBodyA == nullptr || data->mBodyB == nullptr)
			{
				std::cout << "illegal body type!\n";
				exit(1);
			}

			const auto& pt = manifold->getContactPoint(j);
			data->mNormalPointToA = cBulletUtil::btVectorTotVector0(pt.m_normalWorldOnB);
			data->mContactPtOnA = cBulletUtil::btVectorTotVector1(pt.getPositionWorldOnA());
			data->mContactPtOnB = cBulletUtil::btVectorTotVector1(pt.getPositionWorldOnB());
			data->distance = pt.getDistance();

			data->Setup(mNumFrictionDirs);
			data->mbody0GroupId = mMap_collisionid_to_groupid[data->GetBody0Id()];
			data->mbody1GroupId = mMap_collisionid_to_groupid[data->GetBody1Id()];

			// set self collision flag
			if (data->mbody1GroupId == data->mbody0GroupId)
			{
				data->mIsSelfCollision = true;
			}
			// std::cout << "data0 vel = " << data->GetVelOnBody0().transpose() << std::endl;
			// std::cout << "data1 vel = " << data->GetVelOnBody1().transpose() << std::endl;
		}

		// add it to body info
		// std::cout << "col obj data size " << mColObjData.size() << std::endl;
		// std::cout << "boidy 0 id " << data->GetBody0Id() << std::endl;
		int col_group0_data_main_group = mMap_collisionid_to_groupid[mColGroupData[data->mbody0GroupId]->mBody->getWorldArrayIndex()];
		int col_group1_data_main_group = mMap_collisionid_to_groupid[mColGroupData[data->mbody1GroupId]->mBody->getWorldArrayIndex()];
		if (col_group1_data_main_group != col_group0_data_main_group)
		{
			mColGroupData[data->mbody0GroupId]->AddContactPoint(data, col_group0_data_main_group == data->mbody0GroupId);
			mColGroupData[data->mbody1GroupId]->AddContactPoint(data, col_group1_data_main_group == data->mbody0GroupId);
		}
		else
		{
			if (mEnableMultibodySelfCol == true)
			{
				std::cout << "add self collision contact point into ColGroup" << col_group1_data_main_group << ", " << data->mBodyA->GetName() << " and " << data->mBodyB->GetName() << std::endl;
				// only add once for self-collisoin contact point
				mColGroupData[data->mbody0GroupId]->AddContactPoint(data, true);
			}
			else
			{
				delete data;
				continue;
			}
		}

		mContactConstraintData.push_back(data);
	}
}

#include <set>
void cContactSolver::TestCartesianForceToCartesianVel()
{
	PushState();

	int n_contacts = mContactConstraintData.size();
	if (0 == n_contacts) return;
	int n_group = mColGroupData.size();
	x.resize(3 * n_contacts);
	x.setRandom();

	// get prediction
	tEigenArr<tVectorXd> contact_point_cartesian_vel(n_group);
	for (int i = 0; i < n_group; i++)
	{
		cMathUtil::RoundZero(mColGroupData[i]->mConvertCartesianForceToVelocityMat);
		cMathUtil::RoundZero(mColGroupData[i]->mConvertCartesianForceToVelocityVec);
		// std::cout << "for body " << mColGroupData[i]->mBody->GetName() << " ";
		// std::cout << "convert mat = \n"
		// 		  << mColGroupData[i]->mConvertCartesianForceToVelocityMat << std::endl;
		// std::cout << "convert vec = " << mColGroupData[i]->mConvertCartesianForceToVelocityVec.transpose() << std::endl;
		contact_point_cartesian_vel[i] = mColGroupData[i]->mConvertCartesianForceToVelocityMat * x + mColGroupData[i]->mConvertCartesianForceToVelocityVec;
	}

	// apply active force
	for (int i = 0; i < n_contacts; i++)
	{
		auto& data = mContactConstraintData[i];
		data->ApplyForceCartersian(cMathUtil::Expand(x.segment(3 * i, 3), 0));
	}

	std::set<cRobotModel*> models_set;
	// update at the same time
	for (auto& i : mColGroupData)
	{
		switch (i->mBody->GetType())
		{
			case eColObjType::Rigidbody:
				i->mBody->UpdateVelocity(dt);
				break;
			case eColObjType::RobotCollder:
			{
				cRobotModel* model = UpcastRobotCollider(i->mBody)->mModel;
				if (models_set.find(model) == models_set.end())
				{
					model->UpdateVelocity(dt);
					models_set.insert(model);
				}
			}
			default:
				break;
		}
	}

	// get the true vel and update
	bool err = false;
	for (int i = 0; i < n_contacts; i++)
	{
		auto& data = mContactConstraintData[i];
		if (data->mIsSelfCollision) continue;  // ignore self collision in current logic

		tVector body0_true_vel = data->GetVelOnBody0();
		int body0_group = mMap_collisionid_to_groupid[data->GetBody0Id()];
		tVector body0_pred_vel = cMathUtil::Expand(contact_point_cartesian_vel[body0_group].segment(3 * i, 3), 0);

		tVector body1_true_vel = data->GetVelOnBody1();
		int body1_group = mMap_collisionid_to_groupid[data->GetBody1Id()];
		tVector body1_pred_vel = cMathUtil::Expand(contact_point_cartesian_vel[body1_group].segment(3 * i, 3), 0);

		double threshold = 1e-8;
		if ((body0_true_vel - body0_pred_vel).norm() > threshold ||
			(body1_true_vel - body1_pred_vel).norm() > threshold)
		{
			err = true;
			std::cout << "[error] convert cartesian abs vel: for contact " << i << "---------------\n";
			std::cout << "body0 is " << data->mBodyA->GetName() << " body1 is " << data->mBodyB->GetName() << std::endl;
			std::cout << "contact normal = " << data->mNormalPointToA.transpose() << std::endl;
			std::cout << "body 0 pred vel = " << body0_pred_vel.transpose() << std::endl;
			std::cout << "body 0 true vel = " << body0_true_vel.transpose() << std::endl;
			std::cout << "body 1 pred vel = " << body1_pred_vel.transpose() << std::endl;
			std::cout << "body 1 true vel = " << body1_true_vel.transpose() << std::endl;
			std::cout << "q = " << UpcastRobotCollider(data->mBodyA)->mModel->Getq().transpose() << std::endl;
			std::cout << "qdot = " << UpcastRobotCollider(data->mBodyA)->mModel->Getqdot().transpose() << std::endl;
			// exit(1);
		}
	}

	if (err)
	{
		exit(1);
	}
	PopState();
}

void cContactSolver::TestCartesianForceToCartesianRelVel(const tMatrixXd& convert_mat, const tVectorXd& convert_vec)
{
	int n_contacts = mContactConstraintData.size();
	if (0 == n_contacts) return;
	PushState();

	int n_group = mColGroupData.size();
	x.resize(3 * n_contacts);
	x.setRandom();
	// x.setZero();
	// x[1] = 1;

	// get prediction: relative velocity on each contact point
	// std::cout << "rel convert mat = \n"
	// 		  << convert_mat << std::endl;
	// std::cout << "rel convert vec = " << convert_vec.transpose() << std::endl;
	tVectorXd rel_vel_pred = convert_mat * x + convert_vec;

	// apply active force
	int n_self_collision = 0;
	for (int i = 0; i < n_contacts; i++)
	{
		auto& data = mContactConstraintData[i];
		data->ApplyForceCartersian(cMathUtil::Expand(x.segment(3 * i, 3), 0));
		if (data->mIsSelfCollision) n_self_collision++;
	}

	std::set<cRobotModel*> models_set;
	// update at the same time
	for (auto& i : mColGroupData)
	{
		switch (i->mBody->GetType())
		{
			case eColObjType::Rigidbody:
				i->mBody->UpdateVelocity(dt);
				break;
			case eColObjType::RobotCollder:
			{
				cRobotModel* model = UpcastRobotCollider(i->mBody)->mModel;
				if (models_set.find(model) == models_set.end())
				{
					model->UpdateVelocity(dt);
					models_set.insert(model);
				}
			}
			default:
				break;
		}
	}

	// get the true vel and update
	bool err = false;
	for (int i = 0; i < n_contacts; i++)
	{
		auto& data = mContactConstraintData[i];
		tVector true_relvel = data->GetVelOnBody0() - data->GetVelOnBody1();
		tVector pred_relvel = cMathUtil::Expand(rel_vel_pred.segment(i * 3, 3), 0);

		if ((true_relvel - pred_relvel).norm() > 1e-8)
		{
			err = true;
			std::cout << "error convert cartesian force to rel vel: for contact " << i << "---------------\n";
			std::cout << "pred rel vel = " << pred_relvel.transpose() << std::endl;
			std::cout << "true rel vel = " << true_relvel.transpose() << std::endl;

			std::cout << "total contact num = " << n_contacts << std::endl;
			std::cout << "self collision num = " << n_self_collision << std::endl;
			// exit(1);
			// exit(1);
		}
	}

	if (err) exit(1);
	PopState();
}

void cContactSolver::TestCartesianForceToNormalAndTangetRelVel(const tMatrixXd& normal_mat, const tVectorXd& normal_vec, const tMatrixXd& tan_mat, const tVectorXd& tan_vec)
{
	int n_contacts = mContactConstraintData.size();
	if (0 == n_contacts) return;
	PushState();

	int n_group = mColGroupData.size();
	x.resize(3 * n_contacts);
	x.setRandom();
	// x.setZero();
	// x[1] = 1;

	// get normal and tangent vel prediction on each point
	tVectorXd rel_vel_normal_pred = normal_mat * x + normal_vec,
			  rel_vel_tangent_pred = tan_mat * x + tan_vec;

	// apply active force
	for (int i = 0; i < n_contacts; i++)
	{
		auto& data = mContactConstraintData[i];
		data->ApplyForceCartersian(cMathUtil::Expand(x.segment(3 * i, 3), 0));
	}

	std::set<cRobotModel*> models_set;
	// update at the same time
	for (auto& i : mColGroupData)
	{
		switch (i->mBody->GetType())
		{
			case eColObjType::Rigidbody:
				i->mBody->UpdateVelocity(dt);
				break;
			case eColObjType::RobotCollder:
			{
				cRobotModel* model = UpcastRobotCollider(i->mBody)->mModel;
				if (models_set.find(model) == models_set.end())
				{
					model->UpdateVelocity(dt);
					models_set.insert(model);
				}
			}
			default:
				break;
		}
	}

	// get the true vel and update
	bool err = false;
	for (int i = 0; i < n_contacts; i++)
	{
		auto& data = mContactConstraintData[i];
		tVector true_rel_vel = data->GetVelOnBody0() - data->GetVelOnBody1();
		tVectorXd true_normal_relvel = data->mNormalPointToA.transpose() * true_rel_vel;
		tVectorXd true_tan_relvel = data->mD.transpose() * (true_rel_vel - data->mNormalPointToA * true_normal_relvel[0]).segment(0, 3);
		tVectorXd pred_normal_relvel = rel_vel_normal_pred.segment(i, 1);
		tVectorXd pred_tan_relvel = rel_vel_tangent_pred.segment(i * mNumFrictionDirs, mNumFrictionDirs);

		if (
			(pred_normal_relvel - pred_normal_relvel).norm() > 1e-8 ||
			(true_tan_relvel - pred_tan_relvel).norm() > 1e-8)
		{
			err = true;
			std::cout << "[error] cartesian force to rel vel for contact " << i << "---------------\n";
			std::cout << "pred normal rel vel = " << pred_normal_relvel.transpose() << std::endl;
			std::cout << "true normal rel vel = " << true_normal_relvel.transpose() << std::endl;
			std::cout << "pred tan rel vel = " << pred_tan_relvel.transpose() << std::endl;
			std::cout << "true tan rel vel = " << true_tan_relvel.transpose() << std::endl;
			// exit(1);
		}
	}
	if (err) exit(1);
	PopState();
	// std::cout << "TestCartesianForceToNormalAndTangetRelVel tested well" << std::endl;
	// exit(0);
}

void cContactSolver::CalcResultVectorBasedConvertMat()
{
	int n_contact = mContactConstraintData.size();
	int single_shape = (mNumFrictionDirs + 2);
	int final_shape = n_contact * single_shape;
	normal_vel_convert_result_based_mat.resize(n_contact, final_shape);
	normal_vel_convert_result_based_mat.setZero();
	normal_vel_convert_result_based_vec.resize(n_contact);
	tangent_vel_covert_result_based_mat.resize(n_contact * mNumFrictionDirs, final_shape);
	tangent_vel_covert_result_based_mat.setZero();
	tangent_vel_convert_result_based_vec.resize(n_contact * mNumFrictionDirs);

	for (int i = 0; i < n_contact; i++)
	{
		for (int j = 0; j < n_contact; j++)
		{
			auto& data = mContactConstraintData[j];
			normal_vel_convert_result_based_mat.block(i, j * single_shape, 1, single_shape) = normal_vel_convert_mat.block(i, j * 3, 1, 3) * data->mS;
			tangent_vel_covert_result_based_mat.block(i * mNumFrictionDirs, j * single_shape, mNumFrictionDirs, single_shape) =
				tangent_vel_covert_mat.block(i * mNumFrictionDirs, j * 3, mNumFrictionDirs, 3) * data->mS;
		}
	}

	// calculate in another way
	{
		tMatrixXd change_basis_mat;
		change_basis_mat.resize(3 * n_contact, single_shape * n_contact);
		change_basis_mat.setZero();
		for (int i = 0; i < n_contact; i++)
		{
			change_basis_mat.block(i * 3, i * single_shape, 3, single_shape) = mContactConstraintData[i]->mS;
		}
		tMatrixXd res_normal = normal_vel_convert_mat * change_basis_mat - normal_vel_convert_result_based_mat;
		tMatrixXd res_tan = tangent_vel_covert_mat * change_basis_mat - tangent_vel_covert_result_based_mat;
		double err = res_normal.norm() + res_tan.norm();
		// std::cout << "res normal = " << res_normal.norm() << ", res tan = " << res_tan.norm() << std::endl;
		if (err > 1e-6) exit(1);
	}
	tangent_vel_convert_result_based_vec = tangent_vel_convert_vec;
	normal_vel_convert_result_based_vec = normal_vel_convert_vec;
}

void cContactSolver::TestCartesianForceToNormalAndTangetResultBasedRelVel(const tMatrixXd& normal_mat, const tVectorXd& normal_vec, const tMatrixXd& tan_mat, const tVectorXd& tan_vec)
{
	int n_contacts = mContactConstraintData.size();
	if (0 == n_contacts) return;
	PushState();

	int n_group = mColGroupData.size();
	int single_size = mNumFrictionDirs + 2;
	int final_shape = single_size * n_contacts;
	tVectorXd applied_cartesian_force(n_contacts * 3);
	applied_cartesian_force.setZero();
	x.resize(final_shape);
	x.setRandom();

	// get normal and tangent vel prediction on each point
	tVectorXd rel_vel_normal_pred = normal_mat * x + normal_vec,
			  rel_vel_tangent_pred = tan_mat * x + tan_vec;

	// apply active force
	for (int i = 0; i < n_contacts; i++)
	{
		auto& data = mContactConstraintData[i];
		applied_cartesian_force.segment(3 * i, 3) = data->mS * x.segment(single_size * i, single_size);
		data->ApplyForceCartersian(cMathUtil::Expand(applied_cartesian_force.segment(3 * i, 3), 0));
	}

	std::set<cRobotModel*> models_set;
	// update at the same time
	for (auto& i : mColGroupData)
	{
		switch (i->mBody->GetType())
		{
			case eColObjType::Rigidbody:
				i->mBody->UpdateVelocity(dt);
				break;
			case eColObjType::RobotCollder:
			{
				cRobotModel* model = UpcastRobotCollider(i->mBody)->mModel;
				if (models_set.find(model) == models_set.end())
				{
					model->UpdateVelocity(dt);
					models_set.insert(model);
				}
			}
			default:
				break;
		}
	}

	// get the true vel and update
	bool err = false;
	for (int i = 0; i < n_contacts; i++)
	{
		// {
		// 	tVector true_rel_vel = data->GetVelOnBody0() - data->GetVelOnBody1();
		// 	tVectorXd true_normal_relvel = data->mNormalPointToA.transpose() * true_rel_vel;
		// 	tVectorXd true_tan_relvel = data->mD.transpose() * (true_rel_vel - data->mNormalPointToA * true_normal_relvel[0]).segment(0, 3);
		// 	tVectorXd pred_normal_relvel = rel_vel_normal_pred.segment(i, 1);
		// 	tVectorXd pred_tan_relvel = rel_vel_tangent_pred.segment(i * mNumFrictionDirs, mNumFrictionDirs);
		// }
		auto& data = mContactConstraintData[i];
		tVector true_rel_vel = data->GetVelOnBody0() - data->GetVelOnBody1();
		tVectorXd true_normal_relvel = data->mNormalPointToA.transpose() * true_rel_vel;
		// std::cout << "mD size " << data->mD.rows() << " " << data->mD.cols() << std::endl;
		// std::cout << "tmp 1 = " << data->mNormalPointToA * true_normal_relvel[0] << std::endl;
		// std::cout << "tmp 2 = " << true_rel_vel - data->mNormalPointToA * true_normal_relvel[0] << std::endl;
		tVectorXd true_tan_relvel = data->mD.transpose() * (true_rel_vel - data->mNormalPointToA * true_normal_relvel[0]).segment(0, 3);
		// std::cout <<"true 1 = " << true_tan_relvel << std::endl;
		tVectorXd pred_normal_relvel = rel_vel_normal_pred.segment(i, 1);
		tVectorXd pred_tan_relvel = rel_vel_tangent_pred.segment(i * mNumFrictionDirs, mNumFrictionDirs);
		if (
			(pred_normal_relvel - true_normal_relvel).norm() > 1e-8 ||
			(pred_tan_relvel - true_tan_relvel).norm() > 1e-8)
		{
			std::cout << "[error] convert x into decomposed vel, forr contact " << i << "---------------\n";
			std::cout << "total contact = " << n_contacts << std::endl;
			std::cout << "S = " << data->mS << std::endl;
			// std::cout << "self contact = " <<
			std::cout << "pred normal x-based rel vel = " << pred_normal_relvel.transpose() << std::endl;
			std::cout << "true normal x-based rel vel = " << true_normal_relvel.transpose() << std::endl;
			std::cout << "pred tan x-based rel vel = " << pred_tan_relvel.transpose() << std::endl;
			std::cout << "true tan x-based rel vel = " << true_tan_relvel.transpose() << std::endl;
			err = true;
		}
	}
	if (err)
	{
		std::cout << "applied cartesian force = " << applied_cartesian_force.transpose() << std::endl;
		tVectorXd normal_vel_another = normal_vel_convert_mat * applied_cartesian_force + normal_vel_convert_vec;
		tVectorXd tangent_vel_another = tangent_vel_covert_mat * applied_cartesian_force + tangent_vel_convert_vec;
		std::cout << "normal vel another = " << normal_vel_another.transpose() << std::endl;
		std::cout << "normal vel this = " << rel_vel_normal_pred.transpose() << std::endl;

		std::cout << "tangent vel another = " << tangent_vel_another.transpose() << std::endl;
		std::cout << "tangent vel this = " << rel_vel_tangent_pred.transpose() << std::endl;
	}
	if (err) exit(1);
	// exit(0);
	PopState();
}

void cContactSolver::PushState()
{
	std::set<cRobotModel*> models_set;
	for (auto& i : mColGroupData)
	{
		switch (i->mBody->GetType())
		{
			case eColObjType::Rigidbody:
				i->mBody->PushState();
				break;
			case eColObjType::RobotCollder:
			{
				cRobotModel* model = UpcastRobotCollider(i->mBody)->mModel;
				if (models_set.find(model) == models_set.end())
				{
					model->PushState();
					models_set.insert(model);
				}
			}
			default:
				break;
		}
	}
}
void cContactSolver::PopState()
{
	std::set<cRobotModel*> models_set;
	for (auto& i : mColGroupData)
	{
		switch (i->mBody->GetType())
		{
			case eColObjType::Rigidbody:
				i->mBody->PopState();
				break;
			case eColObjType::RobotCollder:
			{
				cRobotModel* model = UpcastRobotCollider(i->mBody)->mModel;
				if (models_set.find(model) == models_set.end())
				{
					model->PopState();
					models_set.insert(model);
				}
			}
			default:
				break;
		}
	}
}

void cContactSolver::UpdateVelocity(float dt)
{
	std::set<cRobotModel*> models_set;
	// update at the same time
	for (auto& i : mColGroupData)
	{
		switch (i->mBody->GetType())
		{
			case eColObjType::Rigidbody:
				i->mBody->UpdateVelocity(dt);
				break;
			case eColObjType::RobotCollder:
			{
				cRobotModel* model = UpcastRobotCollider(i->mBody)->mModel;
				if (models_set.find(model) == models_set.end())
				{
					model->UpdateVelocity(dt);
					models_set.insert(model);
				}
			}
			default:
				break;
		}
	}
}