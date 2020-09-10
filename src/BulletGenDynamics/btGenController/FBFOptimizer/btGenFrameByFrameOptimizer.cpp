#include "btGenFrameByFrameOptimizer.h"
#include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenWorld.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenController/QPSolver/QuadProgQPSolver.h"
#include "BulletGenDynamics/btGenSolver/ConstraintData.h"
#include "BulletGenDynamics/btGenController/FBFOptimizer/btGenFBFConstraint.h"
#include "BulletGenDynamics/btGenController/FBFOptimizer/btGenFBFEnergyTerm.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
static int mNumOfFrictionDirs = 4;

const std::string gContactStatusStr[] = {
	"INVALID_CONTACT_STATUS",
	"SLIDING",
	"STATIC",
	"BREAKAGE"};

eContactStatus
JudgeContactStatus(const tVector& vel, double breakage_threshold = 1e-3, double sliding_threshold = 5e-3)
{
	double vel_n = vel[1];
	double vel_t = std::sqrt(std::pow(vel[0], 2) + std::pow(vel[2], 2));
	if (vel_n > breakage_threshold)
	{
		return eContactStatus::BREAKAGE;
		// return eContactStatus::STATIC;
	}
	else
	{
		if (vel_t > sliding_threshold)
		{
			return eContactStatus::SLIDING;
		}
		else
			return eContactStatus::STATIC;
	}
}

struct btCharContactPt : public btGenContactPointData
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	btCharContactPt(int c_id) : btGenContactPointData(c_id)
	{
		mCollider = nullptr;
		mLocalPos = tVector::Zero();
		mWorldPos = tVector::Zero();
		mJac.resize(0, 0);
		mStatus = eContactStatus::INVALID_CONTACT_STATUS;
	}
	void Init(double dt, btPersistentManifold* manifold, int contact_id_in_manifold)
	{
		btGenContactPointData::Init(dt, manifold, contact_id_in_manifold);

		if (mIsSelfCollision == true)
		{
			std::cout << "[error] CalcContactStatus cannot handle self collsion at this moment\n";
			exit(1);
		}
	}
	bool IsMultibodyInvolved(cRobotModelDynamics* model)
	{
		bool involved = false;
		if (eColObjType::RobotCollder == mBodyA->GetType())
		{
			involved |= dynamic_cast<btGenRobotCollider*>(mBodyA)->mModel == model;
		}
		else if (eColObjType::RobotCollder == mBodyB->GetType())
		{
			involved |= dynamic_cast<btGenRobotCollider*>(mBodyB)->mModel == model;
		}
		return involved;
	}
	void CalcCharacterInfo()
	{
		if (eColObjType::RobotCollder == mBodyA->GetType())
		{
			mCollider = dynamic_cast<btGenRobotCollider*>(mBodyA);
			mWorldPos = mContactPtOnA;
		}

		else if (eColObjType::RobotCollder == mBodyB->GetType())
		{
			mCollider = dynamic_cast<btGenRobotCollider*>(mBodyB);
			mWorldPos = mContactPtOnB;
		}

		mWorldPos[3] = 1;
		auto link = mCollider->mModel->GetLinkById(mCollider->mLinkId);
		mLocalPos = btMathUtil::InverseTransform(link->GetGlobalTransform()) * mWorldPos;

		mCollider->mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(mCollider->mLinkId, mWorldPos.segment(0, 3), mJac);
		// std::cout << "world pos = " << mWorldPos.transpose() << std::endl;
		// std::cout << "local pos = " << mLocalPos.transpose() << std::endl;
	}

	btGenRobotCollider* mCollider;
	tVector mLocalPos;
	tVector mWorldPos;
	tMatrixXd mJac;
	eContactStatus mStatus;
};

typedef btGenFrameByFrameOptimizer Optimizer;

Optimizer::btGenFrameByFrameOptimizer()
{
	mCurFrameId = -1;
	mModel = nullptr;
	mTraj = nullptr;
	mWorld = nullptr;
	mQPSolver = nullptr;
	mContactPoints.clear();
	mContactSolOffset.clear();
	mContactSolSize.clear();

	mTotalSolutionSize = 0;
	mContactSolutionSize = 0;
	mCtrlSolutionSize = 0;

	A.resize(0, 0);
	Aeq.resize(0, 0);
	Aineq.resize(0, 0);

	b.resize(0);
	beq.resize(0);
	bineq.resize(0);
	mConstraint = nullptr;
	mEnergyTerm = nullptr;

	num_of_freedom = 0;
	num_of_underactuated_freedom = 0;
}

Optimizer::~btGenFrameByFrameOptimizer()
{
	if (mConstraint) delete mConstraint;
	if (mQPSolver) delete mQPSolver;
}
void Optimizer::Init(const Json::Value& conf, btTraj* ref_traj, btGeneralizeWorld* world)
{
	mTraj = ref_traj;
	mWorld = world;
	mModel = world->GetMultibody();
	ParseConfig(conf);
	InitModelInfo();
	InitQPSolver();
}

/**
 * \brief                   Calculate the reference qddot and the reference tau by frame-by-frame control
*/
void Optimizer::CalcTarget(double dt, int target_frame_id, tVectorXd& tilde_qddot, tVectorXd& tilde_tau)
{
	mCurFrameId = target_frame_id;
	mdt = dt;

	if (mUseNativeRefTarget == true)
	{
		std::cout << "[warn] use native ref target in FBF optimizer\n";
		tilde_qddot = mTraj->mqddot[mCurFrameId];
		tilde_tau = mTraj->mActiveForce[mCurFrameId].segment(6, num_of_underactuated_freedom);
	}
	else
	{
		// 1. judge the contact status
		CalcContactStatus();

		// 2. construct the optimization problem: energy terms + hard constraints for
		CalcSolutionVector();
		CalcEnergyTerms();
		CalcConstraints();

		// 3. solve this problem and get the ref contact force, ref control force
		// calculate the generalized coordinate accel qddot, and calculate the ref tau
		Solve(tilde_qddot, tilde_tau);
	}
}

/**
 * \brief                   Get the contact points, determine the contact constraint
*/
void Optimizer::CalcContactStatus()
{
	ClearContactPoints();
	// 1. get contact points & link id, calc point positions in local frame
	std::vector<btPersistentManifold*> manifolds = mWorld->GetContactManifolds();

	tEigenArr<tVector> ContactLocalPos(0);
	std::vector<int> ContactLinkId(0);
	for (auto& mani : manifolds)
	{
		for (int i = 0; i < mani->getNumContacts(); i++)
		{
			auto data = new btCharContactPt(mContactPoints.size());
			data->Init(0, mani, i);

			// judge whether it belongs to the multibody
			if (data->IsMultibodyInvolved(mModel) == true)
			{
				data->CalcCharacterInfo();
				mContactPoints.push_back(data);
			}

			else
				delete data;
		}
	}

	// 2. get the reference pos for these points in next frame, calculate their velocity
	// judge the contact status
	{
		tEigenArr<tVector> contact_pos_cur_ref(0), contact_pos_next_ref(0);  // save the position of contact point in current frame and next frame of ref motion
		tEigenArr<tVector> contact_vel_cur_ref(0);
		{
			mModel->PushState("fbf ctrl");
			tVectorXd q_cur_ref = mTraj->mq[mCurFrameId];
			mModel->Apply(q_cur_ref, false);
			for (auto& pt : mContactPoints)
			{
				auto link = mModel->GetLinkById(pt->mCollider->mLinkId);
				contact_pos_cur_ref.push_back(link->GetGlobalTransform() * pt->mLocalPos);
			}
			mModel->PopState("fbf ctrl");
		}
		{
			mModel->PushState("fbf ctrl");
			tVectorXd q_next_ref = mTraj->mq[mCurFrameId + 1];
			mModel->Apply(q_next_ref, false);
			for (auto& pt : mContactPoints)
			{
				auto link = mModel->GetLinkById(pt->mCollider->mLinkId);
				tVector cur_global_pos = link->GetGlobalTransform() * pt->mLocalPos;
				contact_pos_next_ref.push_back(cur_global_pos);

				{
					tMatrixXd jac;
					mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(link->GetId(), cur_global_pos.segment(0, 3), jac);
					tVectorXd qdot = mTraj->mqdot[mCurFrameId + 1];
					tVector vel = tVector::Zero();
					vel.segment(0, 3) = jac * qdot;
					contact_vel_cur_ref.push_back(vel);
				}
			}
			mModel->PopState("fbf ctrl");
		}

		int num_of_contacts = mContactPoints.size();
		for (int id = 0; id < num_of_contacts; id++)
		{
			auto& pt = mContactPoints[id];

			// 1. calculate rel vel
			tVector vel = (contact_pos_next_ref[id] - contact_pos_cur_ref[id]) / mdt;
			// 2. judge contact status
			pt->mStatus = JudgeContactStatus(vel);
			// pt->mStatus = eContactStatus::STATIC;

			// std::cout << "contact " << id << " car vel = " << vel.transpose() << " status " << gContactStatusStr[pt->mStatus] << std::endl;
			// std::cout << "contact " << id << " gen vel = " << contact_vel_cur_ref[id].transpose() << std::endl;
			// 3. calculate convert mat
			CalcContactConvertMat(pt, pt->mS);
		}
	}

	// std::cout
	// 	<< "calc contact status done " << mContactPoints.size() << std::endl;
}

/**
 * \brief                   Calculate the reference qddot and control forces
*/
void Optimizer::Solve(tVectorXd& tilde_qddot, tVectorXd& tilde_tau)
{
	// 1. solve the QP problem
	// std::cout << "[debug] begin to solve the QP problem\n";
	mEnergyTerm->GetEnergyTerm(A, b);
	mConstraint->GetEqJacobianAndResidual(Aeq, beq);        // Aeq * x + beq = 0
	mConstraint->GetIneqJacobianAndResidual(Aineq, bineq);  // Aineq * x + bineq >= 0
	tMatrixXd H = 2 * A.transpose() * A;
	tVectorXd f = 2 * b.transpose() * A;
	// bineq *= -1;
	// beq *= -1;
	tVectorXd solution = tVectorXd::Zero(mTotalSolutionSize);
	Aeq.transposeInPlace();
	Aineq.transposeInPlace();

	// {
	// 	std::cout << "H = \n"
	// 			  << H << std::endl;
	// 	std::cout << "f = " << f.transpose() << std::endl;
	// 	std::cout << "Aeq = \n"
	// 			  << Aeq << std::endl;
	// 	std::cout << "beq = " << beq.transpose() << std::endl;
	// 	std::cout << "Aieq = \n"
	// 			  << Aineq << std::endl;
	// 	std::cout << "bieq = " << bineq.transpose() << std::endl;
	// 	// exit(1);
	// }
	mQPSolver->Solve(mTotalSolutionSize, H, f, Aeq, beq, Aineq, bineq, 100,
					 solution);
	double energy = 0.5 * (solution.transpose() * H).dot(solution) + f.dot(solution) + b.dot(b);
	// std::cout << "[qp] energy = " << energy << std::endl;

	CalcTargetInternal(solution, tilde_qddot, tilde_tau);
	tVectorXd ref_tau = mTraj->mActiveForce[mCurFrameId].segment(6, num_of_underactuated_freedom);
	// std::cout << "[solved] tau = " << tilde_tau.transpose() << std::endl;
	// std::cout << "[ref] tau = " << ref_tau.transpose() << std::endl;
}

void Optimizer::CalcTargetInternal(const tVectorXd& solution, tVectorXd& qddot, tVectorXd& tau)
{
	if (solution.size() != mTotalSolutionSize)
	{
		std::cout << "[error] solution size doesn't match\n";
		exit(1);
	}
	tVectorXd contact_force = solution.segment(0, mContactSolutionSize);
	tVectorXd control_force = solution.segment(mContactSolutionSize, mCtrlSolutionSize);

	// 1. calculate the gen contact force and gen control force
	tVectorXd gen_contact_force = tVectorXd::Zero(num_of_freedom);
	tVectorXd gen_ctrl_force = tVectorXd::Zero(num_of_freedom);

	// std::cout << "contact point size = " << mContactPoints.size() << std::endl;
	// std::cout << "[solved] raw contact solution = " << contact_force.transpose() << std::endl;
	// std::cout << "[solved] raw tau solution = " << control_force.transpose() << std::endl;
	for (int i = 0; i < mContactPoints.size(); i++)
	{
		auto& pt = mContactPoints[i];
		int offset = mContactSolOffset[i];
		int size = mContactSolSize[i];
		tVector3d solved_force = pt->mS * contact_force.segment(offset, size);
		// std::cout << "[debug] contact force " << i << " solved " << solved_force.transpose() << std::endl;
		gen_contact_force += pt->mJac.transpose() * solved_force;
	}
	gen_ctrl_force.segment(6, num_of_underactuated_freedom) = control_force;
	tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
	tVectorXd RHS = gen_contact_force + gen_ctrl_force + QG - mModel->GetCoriolisMatrix() * mModel->Getqdot();
	// std::cout << gen_ctrl_force.size() << std::endl;
	// std::cout << gen_contact_force.size() << std::endl;
	// std::cout << RHS.size() << std::endl;
	// 2. calculate the generated accel
	qddot = mModel->GetInvMassMatrix() * RHS;
	tau = control_force;

	{
		tVectorXd ref_qddot = mTraj->mqddot[mCurFrameId];
		tVectorXd ref_qdot = mTraj->mqdot[mCurFrameId + 1];
		tVectorXd ref_q = mTraj->mqdot[mCurFrameId + 1];
		tVectorXd qdot_cur = mModel->Getqdot();
		tVectorXd q_cur = mModel->Getq();
		tVectorXd qddot_diff = ref_qddot - qddot;
		tVectorXd qdot_diff = qdot_cur + qddot * mdt - ref_qdot;
		tVectorXd q_diff = mdt * (qdot_cur + qddot * mdt) + q_cur - ref_q;
		// std::cout << "[FBF] qddot diff = " << qddot_diff.norm() << ", qdot diff = " << qdot_diff.norm() << ", q diff = " << q_diff.norm() << std::endl;
	}
}

void Optimizer::ParseConfig(const Json::Value& conf)
{
	mEnableHardConstraintForDynamics = btJsonUtil::ParseAsBool("enable_hard_dynamic_constraint", conf);
	mUseNativeRefTarget = btJsonUtil::ParseAsBool("use_native_reference_target", conf);
	mDynamicPosEnergyCoeff = btJsonUtil::ParseAsDouble("dynamic_pos_energy_coef", conf);
	mDynamicVelEnergyCoeff = btJsonUtil::ParseAsDouble("dynamic_vel_energy_coef", conf);
	mDynamicAccelEnergyCoeff = btJsonUtil::ParseAsDouble("dynamic_accel_energy_coef", conf);
}
void Optimizer::InitModelInfo()
{
	num_of_freedom = mModel->GetNumOfFreedom();
	num_of_underactuated_freedom = num_of_freedom - 6;
}
/**
 * \brief					Constrcut the Frame by frame optimization problem
*/
void Optimizer::InitQPSolver()
{
	mQPSolver = new QuadProgQPSolver();
	// std::cout << "init qp solve done\n";
	// exit(1);
}

/**
 * \brief					Construct the energy terms
 * 			Energy terms:
 * 	1. dynamics terms
 *  2. min control force 
 *  3. min contact forces norm
*/
void Optimizer::CalcEnergyTerms()
{
	if (mEnergyTerm != nullptr) delete mEnergyTerm;
	mEnergyTerm = new btGenFrameByFrameEnergyTerm(mTotalSolutionSize);

	if (mEnableHardConstraintForDynamics == false)
	{
		AddDynamicEnergyTerm();
	}
	AddMinTauEnergyTerm();
	AddMinContactForceEnergyTerm();
}

/**
 * \brief					Construct the constraints
*/
void Optimizer::CalcConstraints()
{
	if (nullptr != mConstraint) delete mConstraint;
	mConstraint = new btGenFrameByFrameConstraint(mTotalSolutionSize);

	if (mEnableHardConstraintForDynamics == true)
		AddDynamicConstraint();
	// for (int i = 0; i < mContactPoints.size(); i++)
	for (auto& pt : mContactPoints)
	{
		switch (pt->mStatus)
		{
			case eContactStatus::SLIDING:
				AddSlidingConstraint(pt);
				break;
			case eContactStatus::STATIC:
				AddStaticConstraint(pt);
				break;
			case eContactStatus::BREAKAGE:
				AddBreakageConstraint(pt);
				break;
			default:
				std::cout << "[error] illegal contact status " << pt->mStatus << std::endl;

				exit(1);
				break;
		}
	}
}

void Optimizer::ClearContactPoints()
{
	for (auto& data : mContactPoints) delete data;
	mContactPoints.clear();
	mContactSolOffset.clear();
	mContactSolSize.clear();
}

/**
 * \brief				Calculate solution size, the offset w.r.t each contact point
 * 
 * 		solution vector = [contact_foce_vector, active_ctrl_force]
*/
void Optimizer::CalcSolutionVector()
{
	mContactSolutionSize = 0;
	mCtrlSolutionSize = 0;
	mTotalSolutionSize = 0;
	int num_of_contacts = mContactPoints.size();
	// int num_of_freedom = mModel->GetNumOfFreedom();
	// int num_of_underactuated_freedom = num_of_freedom - 6;
	mContactSolOffset.resize(num_of_contacts, 0);
	mContactSolSize.resize(num_of_contacts, 0);
	// 1. init the result vector: contact forces + control forces (N-6)
	int offset = 0;

	for (int i = 0; i < num_of_contacts; i++)
	{
		auto pt = mContactPoints[i];

		int size = GetSolutionSizeByContactStatus(pt->mStatus);
		mContactSolSize[i] = size;
		mContactSolOffset[i] = offset;
		offset += size;
		mContactSolutionSize += size;
	}

	mCtrlSolutionSize += num_of_underactuated_freedom;

	mTotalSolutionSize = mCtrlSolutionSize + mContactSolutionSize;
	std::cout << "[debug] contact num " << num_of_contacts << ", total solution size " << mTotalSolutionSize << std::endl;
}

/**
 * \brief				Add sliding contact point constraint 
*/
void Optimizer::AddSlidingConstraint(btCharContactPt* pt)
{
	int contact_id = pt->contact_id;
	int offset = mContactSolOffset[contact_id];
	int size = mContactSolSize[contact_id];
	// 1. ineqaulty make fn, ft >=0
	{
		tMatrixXd jac = tMatrixXd::Identity(size, size);
		tVectorXd res = tVectorXd::Zero(size);
		mConstraint->AddIneqCon(jac, res, offset);
	}
	// 2. equalities make ft = mu * fn
	{
		tMatrixXd jac = tMatrixXd::Zero(1, size);
		jac(0, 0) = mu;
		jac.block(0, 1, 1, size - 1).fill(-1);
		mConstraint->AddEqCon(jac, tVectorXd::Zero(1), offset);
	}
}

void Optimizer::AddStaticConstraint(btCharContactPt* pt)
{
	int static_size = mContactSolSize[pt->contact_id];
	int offset = mContactSolOffset[pt->contact_id];
	tMatrixXd jac = tMatrixXd::Zero(static_size + 1, static_size);
	tVectorXd residual = tVectorXd::Zero(static_size + 1);

	jac.block(0, 0, static_size, static_size).setIdentity();

	jac(static_size, 0) = mu;
	jac.block(static_size, 1, 1, mNumOfFrictionDirs).fill(-1);
	mConstraint->AddIneqCon(jac, residual, offset);
}

void Optimizer::AddBreakageConstraint(btCharContactPt* pt)
{
	int offset = mContactSolOffset[pt->contact_id];
	tMatrixXd jac = tMatrix3d::Identity();
	mConstraint->AddEqCon(jac, tVector3d::Zero(), offset);
}

/**
 * \brief				Close to Origin energy term in this frame by frame control
*/
void Optimizer::AddDynamicEnergyTerm()
{
	AddDynamicEnergyTermPos();
	AddDynamicEnergyTermVel();
	AddDynamicEnergyTermAccel();
}

void Optimizer::AddDynamicConstraint()
{
	// int num_of_freedom = mModel->GetNumOfFreedom();
	// int num_of_underactuated_freedom = num_of_freedom - 6;
	tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
	tVectorXd b = tVectorXd::Zero(num_of_freedom);

	const tMatrixXd& Minv = mModel->GetInvMassMatrix();
	const tMatrixXd& C_d = mModel->GetCoriolisMatrix();
	const tVectorXd& qdot = mModel->Getqdot();
	const tVectorXd& q = mModel->Getq();
	const tVectorXd& q_next_ref = mTraj->mq[mCurFrameId + 1];
	double dt2 = mdt * mdt;
	tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
	b = dt2 * Minv * (QG - C_d * qdot) + q + mdt * qdot - q_next_ref;
	tMatrixXd A1 = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize), A2 = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
	for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
	{
		auto pt = mContactPoints[c_id];
		int size = mContactSolSize[pt->contact_id];
		int offset = mContactSolOffset[pt->contact_id];

		// (N * 3) * (3 * size) = N * size
		A1.block(0, offset, num_of_freedom, size).noalias() = dt2 * Minv * pt->mJac.transpose() * pt->mS;
	}

	tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
	N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom).setIdentity();
	A2.noalias() = dt2 * Minv * N;
	A.block(0, 0, num_of_freedom, mContactSolutionSize).noalias() = A1;
	A.block(0, mContactSolutionSize, num_of_freedom, num_of_underactuated_freedom).noalias() = A2;
	mConstraint->AddEquivalentEqCon(A, b, 0, 1e-12);
}
void Optimizer::AddMinTauEnergyTerm()
{
	// int num_of_underactuated_freedom = mModel->GetNumOfFreedom() - 6;
	tMatrixXd A = tMatrixXd::Identity(num_of_underactuated_freedom, num_of_underactuated_freedom);
	tVectorXd b = tVectorXd::Zero(num_of_underactuated_freedom);
	mEnergyTerm->AddEnergy(A, b, mContactSolutionSize);
}

void Optimizer::AddMinContactForceEnergyTerm()
{
	tMatrixXd A = tMatrixXd::Identity(mContactSolutionSize, mContactSolutionSize);
	tVectorXd b = tVectorXd::Zero(mContactSolutionSize);
	mEnergyTerm->AddEnergy(A, b, 0);
}

tMatrixXd CalcFrictionCone(int friction_num)
{
	assert(friction_num >= 3);
	tMatrixXd dir_mat = tMatrixXd::Zero(3, friction_num);
	double unit = 2 * M_PI / friction_num;
	for (int i = 0; i < friction_num; i++)
	{
		dir_mat(0, i) = std::cos(unit * i);
		dir_mat(2, i) = std::sin(unit * i);
	}

	// round to zero in order to get a clean output
	for (int i = 0; i < dir_mat.rows(); i++)
		for (int j = 0; j < dir_mat.cols(); j++)
		{
			if (std::fabs(dir_mat(i, j)) < 1e-10)
				dir_mat(i, j) = 0;
		}
	return dir_mat;
}

void Optimizer::CalcContactConvertMat(btCharContactPt* contact, tMatrixXd& convert_mat)
{
	int size = GetSolutionSizeByContactStatus(contact->mStatus);
	switch (contact->mStatus)
	{
		case eContactStatus::SLIDING:
		{
			convert_mat.resize(3, size);
			convert_mat.col(0) = tVector3d(0, 1, 0);
			convert_mat.block(0, 1, 3, mNumOfFrictionDirs) = CalcFrictionCone(mNumOfFrictionDirs);
		}
		break;
		case eContactStatus::STATIC:
		{
			convert_mat.resize(3, size);
			convert_mat.col(0) = tVector3d(0, 1, 0);
			convert_mat.block(0, 1, 3, mNumOfFrictionDirs) = CalcFrictionCone(mNumOfFrictionDirs);
		}
		break;
		case eContactStatus::BREAKAGE:
		{
			convert_mat.noalias() = tMatrix3d::Identity();
		}
		break;
		default:
			std::cout << "[error] contact status " << contact->mStatus << std::endl;
			break;
	}
}

int Optimizer::GetSolutionSizeByContactStatus(eContactStatus status)
{
	if (status == eContactStatus::SLIDING)
	{
		return mNumOfFrictionDirs + 1;
	}
	else if (status == eContactStatus::BREAKAGE)
	{
		return 3;
	}
	else if (status == eContactStatus::STATIC)
	{
		return mNumOfFrictionDirs + 1;
	}
	std::cout << "[error] Unsupported contact status " << status << std::endl;
	exit(1);
	return 0;
}

void Optimizer::AddDynamicEnergyTermPos()
{
	tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
	tVectorXd b = tVectorXd::Zero(num_of_freedom);

	const tMatrixXd& Minv = mModel->GetInvMassMatrix();
	const tMatrixXd& C_d = mModel->GetCoriolisMatrix();
	const tVectorXd& qdot = mModel->Getqdot();
	const tVectorXd& q = mModel->Getq();
	const tVectorXd& q_next_ref = mTraj->mq[mCurFrameId + 1];
	double dt2 = mdt * mdt;
	tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
	b = dt2 * Minv * (QG - C_d * qdot) + q + mdt * qdot - q_next_ref;
	tMatrixXd A1 = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize), A2 = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
	for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
	{
		auto pt = mContactPoints[c_id];
		int size = mContactSolSize[pt->contact_id];
		int offset = mContactSolOffset[pt->contact_id];

		// (N * 3) * (3 * size) = N * size
		A1.block(0, offset, num_of_freedom, size).noalias() = dt2 * Minv * pt->mJac.transpose() * pt->mS;
	}

	tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
	N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom).setIdentity();
	A2.noalias() = dt2 * Minv * N;
	A.block(0, 0, num_of_freedom, mContactSolutionSize).noalias() = A1;
	A.block(0, mContactSolutionSize, num_of_freedom, num_of_underactuated_freedom).noalias() = A2;
	A /= dt2;
	b /= dt2;
	A *= mDynamicPosEnergyCoeff;
	b *= mDynamicPosEnergyCoeff;
	mEnergyTerm->AddEnergy(A, b, 0);
}
void Optimizer::AddDynamicEnergyTermVel()
{
	tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
	tVectorXd b = tVectorXd::Zero(num_of_freedom);

	const tMatrixXd& Minv = mModel->GetInvMassMatrix();
	const tMatrixXd& C_d = mModel->GetCoriolisMatrix();
	const tVectorXd& qdot = mModel->Getqdot();
	const tVectorXd& q = mModel->Getq();
	const tVectorXd& qdot_next_ref = mTraj->mq[mCurFrameId + 1];
	// double dt2 = mdt * mdt;
	tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
	b = mdt * Minv * (QG - C_d * qdot) + qdot - qdot_next_ref;
	tMatrixXd A1 = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize), A2 = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
	for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
	{
		auto pt = mContactPoints[c_id];
		int size = mContactSolSize[pt->contact_id];
		int offset = mContactSolOffset[pt->contact_id];

		// (N * 3) * (3 * size) = N * size
		A1.block(0, offset, num_of_freedom, size).noalias() = mdt * Minv * pt->mJac.transpose() * pt->mS;
	}

	tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
	N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom).setIdentity();
	A2.noalias() = mdt * Minv * N;
	A.block(0, 0, num_of_freedom, mContactSolutionSize).noalias() = A1;
	A.block(0, mContactSolutionSize, num_of_freedom, num_of_underactuated_freedom).noalias() = A2;
	A /= mdt;
	b /= mdt;
	A *= mDynamicVelEnergyCoeff;
	b *= mDynamicVelEnergyCoeff;
	mEnergyTerm->AddEnergy(A, b, 0);
}
void Optimizer::AddDynamicEnergyTermAccel()
{
	tMatrixXd A = tMatrixXd::Zero(num_of_freedom, mTotalSolutionSize);
	tVectorXd b = tVectorXd::Zero(num_of_freedom);

	const tMatrixXd& Minv = mModel->GetInvMassMatrix();
	const tMatrixXd& C_d = mModel->GetCoriolisMatrix();
	const tVectorXd& qdot = mModel->Getqdot();
	const tVectorXd& q = mModel->Getq();
	const tVectorXd& qddot_next_ref = mTraj->mq[mCurFrameId];
	// double dt2 = mdt * mdt;
	tVectorXd QG = mModel->CalcGenGravity(mWorld->GetGravity());
	b = Minv * (QG - C_d * qdot) + qdot - qddot_next_ref;
	tMatrixXd A1 = tMatrixXd::Zero(num_of_freedom, mContactSolutionSize), A2 = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
	for (int c_id = 0; c_id < mContactPoints.size(); c_id++)
	{
		auto pt = mContactPoints[c_id];
		int size = mContactSolSize[pt->contact_id];
		int offset = mContactSolOffset[pt->contact_id];

		// (N * 3) * (3 * size) = N * size
		A1.block(0, offset, num_of_freedom, size).noalias() = Minv * pt->mJac.transpose() * pt->mS;
	}

	tMatrixXd N = tMatrixXd::Zero(num_of_freedom, num_of_underactuated_freedom);
	N.block(6, 0, num_of_underactuated_freedom, num_of_underactuated_freedom).setIdentity();
	A2.noalias() = Minv * N;
	A.block(0, 0, num_of_freedom, mContactSolutionSize).noalias() = A1;
	A.block(0, mContactSolutionSize, num_of_freedom, num_of_underactuated_freedom).noalias() = A2;
	A *= mDynamicAccelEnergyCoeff;
	b *= mDynamicAccelEnergyCoeff;
	mEnergyTerm->AddEnergy(A, b, 0);
}