#pragma once
#include "RobotModel.h"

class cRobotModelDynamics : public cRobotModel
{
public:
	cRobotModelDynamics(const char* model_file, double scale, int type);

	// ----------------------simulation APIs----------------------
	void InitSimVars(btDiscreteDynamicsWorld* world, bool zero_pose, bool zero_pose_vel);
	void SetPose(const tVectorXd& q, const tVectorXd& qdot);
	void SetMassMatEps(double eps);
	void ApplyGravity(const tVector& g);  // apply force
	void ApplyForce(int link_id, const tVector& f, const tVector& applied_pos);
	void ApplyGeneralizedForce(int dof_id, double value);
	void ApplyLinkTorque(int link_id, const tVector& torque);
	void ApplyJointTorque(int joint_id, const tVector& torque);
	tVectorXd GetGeneralizedForce();
	cRobotCollider* GetLinkCollider(int link_id);
	void ClearForce();
	tVectorXd Getqddot();
	void UpdateVelocity(double dt, bool verbose = false);
	void UpdateVelocityWithoutCoriolis(double dt);
	void UpdateTransform(double dt);
	void UpdateRK4(double dt);
	void SyncToBullet();
	bool IsMaxVel();
	void PushState(const std::string& tag, bool only_vel_and_force = false);
	void PopState(const std::string& tag, bool only_vel_and_force = false);
	void SetDampingCoeff(double, double);
	void SetAngleClamp(bool);
	void SetMaxVel(double);
	const tMatrixXd& GetDampingMatrix() const { return damping_matrix; }
	void TestJacobian();
	void TestSecondJacobian();
	tMatrixXd GetInvMassMatrix();
	void GetEffectInfo(tEigenArr<tVector>& force_array, tEigenArr<tVector>& torque_array);

protected:
	// -------------------------simulation vars-----------------------
	// Bullet collider vars
	std::vector<cRobotCollider*> multibody_colliders;  // used in bullet collision detection for multibody structure
	tEigenArr<tVector> link_forces, link_torques;      //
	tVectorXd generalized_force;
	double damping_coef1, damping_coef2;
	// bool calc_g_force_by_jv;
	bool clamp_euler_angle;
	double eps_diagnoal_mass_mat;
	double max_vel;
	tMatrixXd damping_matrix;
	struct tStateRecord
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		bool only_vel_and_force_recorded;
		tVectorXd q, qdot;
		tVectorXd generalized_force;
		tEigenArr<tVector> link_forces, link_torques;
		tMatrixXd mass_matrix;
		tMatrixXd coriolis_matrix;
		tMatrixXd damping_matrix;
	};

	const int mStackCapacity = 10;
	tEigenArr<std::pair<std::string, tStateRecord*> > mStateStack;

	void UpdateRK4InternalUpdate(tVectorXd& q, tVectorXd& qdot, tVectorXd& Q);
	void ComputeMassMatrix();
	void ComputeDampingMatrix();
};
