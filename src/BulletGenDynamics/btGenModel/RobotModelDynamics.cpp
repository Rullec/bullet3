#include "RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#include "Link.h"
#include "RobotCollider.h"
#include "btBulletDynamicsCommon.h"
#include <fstream>
#include <iostream>

cRobotModelDynamics ::tStateRecord::tStateRecord()
{
    only_vel_and_force_recorded = false;
    q.resize(0);
    qdot.resize(0);
    generalized_force.resize(0);
    link_forces.clear();
    link_torques.clear();
    mass_matrix.resize(0, 0);
    coriolis_matrix.resize(0, 0);
    damping_matrix.resize(0, 0);
    compute_2rd_derive = false;
    compute_3ed_derive = false;
}
extern bool gEnablePauseWhenSolveError;
extern bool gEnableResolveWhenSolveError;

// std::string path = "debug_trans_bt.txt";
cRobotModelDynamics::cRobotModelDynamics()
{
    // mBtWorld = nullptr;
    mColliders.clear();
    // mColShapes.clear();
    mLinkTorques.clear();
    mLinkForces.clear();
    mGenForce.resize(0);
    mDampingMatrix.resize(0, 0);
    mStateStack.clear();
    mController = nullptr;

    mEnableCollision = true;
    mDampingCoef1 = 0;
    mDampingCoef2 = 0;
    mEpsDiagnoal = 0;
    mEnableEulerAngleClamp = true;
    mMaxVel = 100.0;
    mEnableContactAwareController = false;
    // std::ofstream fout(path);
    // fout << "";
    // fout.close();
}

cRobotModelDynamics::~cRobotModelDynamics()
{
    // for (auto& x : mColShapes)
    // {
    // 	delete x;
    // }
    // mColShapes.clear();
    for (auto &x : mColliders)
    {
        if (x->getCollisionShape() != nullptr)
        {
            delete x->getCollisionShape();
            x->setCollisionShape(nullptr);
        }
        delete x;
    }
    mColliders.clear();

    for (auto &x : mStateStack)
        delete x.second;
    mStateStack.clear();
}
void cRobotModelDynamics::Init(const char *model_file, double scale, int type)
{
    cRobotModel::Init(model_file, scale, type);
}

void cRobotModelDynamics::ComputeDampingMatrix()
{
    mDampingMatrix =
        mDampingCoef1 * mass_matrix + mDampingCoef2 * coriolis_matrix;
}

void cRobotModelDynamics::ComputeMassMatrix()
{
    cRobotModel::ComputeMassMatrix();
    // std::cout << "[model] add eps " << eps_diagnoal_mass_mat << " on the
    // diagnoal of mass matrix\n";
    mass_matrix += mEpsDiagnoal *
                   tMatrixXd::Identity(mass_matrix.rows(), mass_matrix.cols());
}

/**
 * \brief					create Multibody colliders in
 * bullet world \param world				the ptr to btWorld
 * */
extern std::map<int, std::string> col_name;
#include "BulletGenDynamics/btGenModel/Joint.h"
void cRobotModelDynamics::InitSimVars(btDynamicsWorld *world, bool zero_pose,
                                      bool zero_pose_vel, bool enable_collision)
{
    // mBtWorld = world;
    // std::cout << "Init collider\n";
    mEnableCollision = enable_collision;
    mColliders.clear();
    btGenRobotCollider *collider = nullptr;

    // 1. get link shape info  and add them into btworld
    int num_of_links = GetNumOfLinks();
    for (int i = 0; i < num_of_links; i++)
    {
        const auto &link = static_cast<Link *>(GetLinkById(i));
        double mass = link->GetMass();
        ShapeType shape_type = link->GetShapeType();
        btCollisionShape *shape = nullptr;
        const tVector3f &mesh_scale = link->GetMeshScale();
        // std::cout << "for link " << i << " mesh scale = " <<
        // mesh_scale.transpose() << std::endl;
        switch (shape_type)
        {
        case ShapeType::SPHERE_SHAPE:
            shape = new btSphereShape(btScalar(mesh_scale[0]) / 2);
            // std::cout << "[btRobot] sphere radius = " <<
            // btScalar(mesh_scale[0]) / 2 << std::endl;
            break;
        case ShapeType::BOX_SHAPE:
            shape = new btBoxShape(btBulletUtil::tVectorTobtVector(
                                       btMathUtil::Expand(mesh_scale, 0)) /
                                   2);
            break;
        case ShapeType::CAPSULE_SHAPE:
        {
            double radius = 0.5 * mesh_scale[0];
            double height = mesh_scale[1];
            shape = new btCapsuleShape(radius, height);
        }
        break;
        default:
            std::cout << "unsupported type in init sim vars\n";
            exit(0);
            break;
        }
        // mColShapes.push_back(shape);
        // btCollisionShape* shape = new btBoxShape(btVector3(mesh_scale[0],
        // mesh_scale[1], mesh_scale[2]) / 2);

        btVector3 localInertia(0, 0, 0);
        shape->calculateLocalInertia(mass, localInertia);
        // std::cout << "local inertia = " <<
        // btBulletUtil::btVectorTotVector0(localInertia).transpose() <<
        // std::endl;
        collider = new btGenRobotCollider(
            this, i, link->GetName() + std::to_string(i), link->GetColGroup());
        collider->setCollisionShape(shape);
        collider->setUserIndex(-1);
        collider->setUserPointer(collider);

        // world->addCollisionObject(collider, 2, 1 + 2);
        if (enable_collision == true)
        {
            world->addCollisionObject(collider, 1, -1);
        }
        else
        {
            world->addCollisionObject(collider, 0, 0);
        }

        col_name[collider->getWorldArrayIndex()] =
            "multibody" + std::to_string(collider->mLinkId);
        mColliders.push_back(collider);
    }
    // 2. set up the parent collider
    for (int i = 0; i < num_of_links; i++)
    {
        const BaseObject *parent_joint = GetLinkById(i)->GetParent();
        int parent_link_id = parent_joint->GetParentId();
        if (-1 == parent_link_id)
        {
            mColliders[i]->mParentCollider = nullptr;
        }
        else
        {
            mColliders[i]->mParentCollider = mColliders[parent_link_id];
        }
        // std::cout << "for link " << i << " parent link id = " <<
        // parent_link_id << std::endl;
    }

    // 3. allocate sim vars and Init
    mLinkForces.resize(num_of_links, tVector::Zero());
    mLinkTorques.resize(num_of_links, tVector::Zero());
    int dof = GetNumOfFreedom();
    // std::cout << "dof = " << dof << std::endl;
    // assert(false);
    mq.resize(dof), mq.setZero();
    mqdot.resize(dof), mqdot.setZero();

    mGenForce.resize(dof);
    mGenForce.setZero();
    if (zero_pose == true)
        mq.setZero();
    else
    {
        mq.setRandom();
    }

    if (zero_pose_vel == true)
        mqdot.setZero();
    else
    {
        mqdot.setRandom();
    }
    mq[1] = 0.8;

    cRobotModelDynamics::SetqAndqdot(mq, mqdot);

    tVectorXd lower, upper;
    GetJointLimit(lower, upper);
    // std::cout << "[debug] joint lower limit = " << lower.transpose() <<
    // std::endl; std::cout << "[debug] joint upper limit = " <<
    // upper.transpose() << std::endl; this->TestRotationChar(); for (int i = 0;
    // i < GetNumOfJoint(); i++)
    // {
    // 	auto joint = static_cast<Joint*>(GetJointById(i));
    // 	std::cout << "joint " << i << " diff weight = " <<
    // joint->GetDiffWeight() << std::endl;
    // }
    // exit(0);
}

void cRobotModelDynamics::TestJacobian()
{
    // 1. save old state
    tVectorXd q_old = mq, qdot_old = mqdot;
    int n_dof = GetNumOfFreedom();

    // 2. test jacobian
    mq = tVectorXd::Random(n_dof);
    mqdot = tVectorXd::Random(n_dof);
    for (int link_id = 0; link_id < GetNumOfLinks(); link_id++)
    {
        Apply(mq, true);
        tVector pt_local = btMathUtil::Expand(tVector3d::Random() * 10, 1);
        tMatrixXd jac_pred, jac_truth;
        auto link = GetLinkById(link_id);
        tVector pt_world_old = link->GetGlobalTransform() * pt_local;

        // 1. pred

        ComputeJacobiByGivenPointTotalDOFWorldFrame(
            link_id, pt_world_old.segment(0, 3), jac_pred);
        // ComputeJacobiByGivenPointTotalDOFLocalFrame(link_id,
        // pt_local.segment(0, 3), jac_pred);

        // 2. true
        const double bias = 1e-7;
        jac_truth.resize(3, n_dof);
        tVector pt_world_new;
        for (int dof = 0; dof < n_dof; dof++)
        {
            // std::cout << "q size " << q.size() << " dof " << dof <<
            // std::endl;
            mq[dof] += bias;
            Apply(mq, true);
            pt_world_new = link->GetGlobalTransform() * pt_local;
            jac_truth.block(0, dof, 3, 1) =
                (pt_world_new - pt_world_old).segment(0, 3) / bias;
            mq[dof] -= bias;
        }
        double err = (jac_truth - jac_pred).norm();
        std::cout << "test jacobian link " << link_id << ", residual = " << err
                  << std::endl;

        if (err > 1e-5)
        {
            std::cout << "jac pred = \n" << jac_pred << std::endl;
            std::cout << "jac truth = \n" << jac_truth << std::endl;

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
    PushState("test_second_jacobian");
    int num_of_links = GetNumOfLinks();
    tVectorXd q = Getq(), qdot = Getqdot();
    // q.setZero();
    // qdot.setZero();
    q.setRandom();
    qdot.setRandom();
    std::cout << "[test] q = " << q.transpose() << std::endl;
    std::cout << "[test] qdot = " << qdot.transpose() << std::endl;
    Apply(q, true);

    // 1. compute Jv'(dJv/dq) and Jw'(dJw/dq), current Jv and Jw
    tVector3d p = tVector3d(0, 0, 0);

    tEigenArr<tMatrixXd> Jkv_middle(0), Jkw_middle(0); // Jv, Jw
    std::vector<tEigenArr<tMatrixXd>> dJkvdq_ana(num_of_links);
    std::vector<tEigenArr<tMatrixXd>> dJkwdq_ana(num_of_links);

    for (int i = 0; i < num_of_links; i++)
    {
        auto link = GetLinkById(i);
        link->ComputeDJkvdq(p);
        link->ComputeDJkwdq();

        Jkv_middle.push_back(link->GetJKv());
        Jkw_middle.push_back(link->GetJKw());

        dJkvdq_ana[i].resize(num_of_freedom,
                             tMatrixXd::Zero(3, num_of_freedom));
        dJkwdq_ana[i].resize(num_of_freedom,
                             tMatrixXd::Zero(3, num_of_freedom));
        tMatrixXd nxn_djvdq[3], nxn_djwdq[3];
        for (int sub_id = 0; sub_id < 3; sub_id++)
        {
            nxn_djvdq[sub_id] = link->GetdJKvdq_nxnversion(sub_id);
            nxn_djwdq[sub_id] = link->GetdJKwdq_nxnversion(sub_id);
            // std::cout << nxn_djvdq[sub_id] << std::endl;
            // std::cout << "nxn_djwdq " << sub_id << " = \n"
            // 		  << nxn_djwdq[sub_id] << std::endl;
        }
        // std::cout << "it should be all zero\n";
        // exit(0);
        for (int dof = 0; dof < num_of_freedom; dof++)
        {
            dJkvdq_ana[i][dof].resize(3, num_of_freedom);
            dJkvdq_ana[i][dof].block(0, 0, 1, num_of_freedom) =
                nxn_djvdq[0].block(dof, 0, 1, num_of_freedom);
            dJkvdq_ana[i][dof].block(1, 0, 1, num_of_freedom) =
                nxn_djvdq[1].block(dof, 0, 1, num_of_freedom);
            dJkvdq_ana[i][dof].block(2, 0, 1, num_of_freedom) =
                nxn_djvdq[2].block(dof, 0, 1, num_of_freedom);

            dJkwdq_ana[i][dof].resize(3, num_of_freedom);
            // dJkwdq_ana[i][dof].block(0, 0, 1, num_of_freedom) =
            // nxn_djwdq[0].block(dof, 0, 1, num_of_freedom);
            // dJkwdq_ana[i][dof].block(1, 0, 1, num_of_freedom) =
            // nxn_djwdq[1].block(dof, 0, 1, num_of_freedom);
            // dJkwdq_ana[i][dof].block(2, 0, 1, num_of_freedom) =
            // nxn_djwdq[2].block(dof, 0, 1, num_of_freedom);
            dJkwdq_ana[i][dof].block(0, 0, 1, num_of_freedom) =
                nxn_djwdq[0].block(0, dof, num_of_freedom, 1).transpose();
            dJkwdq_ana[i][dof].block(1, 0, 1, num_of_freedom) =
                nxn_djwdq[1].block(0, dof, num_of_freedom, 1).transpose();
            dJkwdq_ana[i][dof].block(2, 0, 1, num_of_freedom) =
                nxn_djwdq[2].block(0, dof, num_of_freedom, 1).transpose();
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
                std::cout << "dJvdq_ana link " << link_id << " dof " << dof
                          << " = \n"
                          << dJkvdq_ana[link_id][dof] << std::endl;
                std::cout << "dJvdq_num link " << link_id << " dof " << dof
                          << " = \n"
                          << dJkvdq_num << std::endl;
                std::cout << "error\n";
                exit(0);
            }
            // else
            //     std::cout << "[log] link " << link_id << " dof " << dof
            //               << " dJvdq verified succ\n";
            tMatrixXd Jkw_cur = link->GetJKw();
            tMatrixXd dJkwdq_num = (Jkw_cur - Jkw_middle[link_id]) / bias;
            tMatrixXd dJkwdq_diff = dJkwdq_ana[link_id][dof] - dJkwdq_num;
            // std::cout << "link " << link_id << " dof " << dof << " Jw_cur =
            // \n"
            // 		  << Jkw_cur << std::endl;
            // std::cout << "link " << link_id << " dof " << dof << " Jw_middle
            // = \n"
            //   << Jkw_middle[link_id] << std::endl;
            if (dJkwdq_diff.cwiseAbs().maxCoeff() > 1e-6)
            {
                std::cout << "dJwdq_ana link " << link_id << " dof " << dof
                          << " = \n"
                          << dJkwdq_ana[link_id][dof] << std::endl;
                std::cout << "dJwdq_num link " << link_id << " dof " << dof
                          << " = \n"
                          << dJkwdq_num << std::endl;
                std::cout << "dJwdq_diff link " << link_id << " dof " << dof
                          << " = \n"
                          << dJkwdq_diff << std::endl;
                std::cout << "error\n";
                exit(0);
            }
            // else
            //     std::cout << "[log] link " << link_id << " dof " << dof
            //               << " dJwdq verified succ\n";

            q[dof] -= bias;
            Apply(q, true);
        }
        std::cout << "[log] Test second jacobian for link " << link_id
                  << " succ\n";
    }

    // 3. compare
    // exit(0);
    PopState("test_second_jacobian");
}

void cRobotModelDynamics::SetqAndqdot(const tVectorXd &q_,
                                      const tVectorXd &qdot_)
{
    if (q_.size() != num_of_freedom || qdot_.size() != num_of_freedom)
    {
        std::cout << "[error] illegal q & qdot size : q " << q_.size()
                  << " qdot " << qdot_.size() << " dof " << num_of_freedom
                  << std::endl;
        assert(false);
        exit(0);
    }
    mq = q_;
    mqdot = qdot_;
    Apply(mq, true);
    ComputeMassMatrix();
    ComputeCoriolisMatrix(mqdot);
    ComputeDampingMatrix();
    UpdateCartesianVelocity();
    SyncToBullet();
}
void cRobotModelDynamics::Setqdot(const tVectorXd &qdot_)
{
    mqdot = qdot_;
    ComputeCoriolisMatrix(mqdot);
    ComputeDampingMatrix();
    UpdateCartesianVelocity();
    SyncToBullet();
}
void cRobotModelDynamics::SetMassMatEps(double eps) { mEpsDiagnoal = eps; }

void cRobotModelDynamics::ApplyGravity(const tVector &g) // apply force
{
    for (int i = 0; i < GetNumOfLinks(); i++)
    {
        ApplyForce(i, GetLinkById(i)->GetMass() * g,
                   btMathUtil::Expand(GetLinkById(i)->GetWorldPos(), 1));
    }
    // std::cout << "after apply gravity, force num = " <<
    // link_force_id_lst.size() << std::endl;
}

/**
 * \brief					given the gravity accel,
 * calcualte the generalized gravity \param g
 * gravity accel
 */
tVectorXd cRobotModelDynamics::CalcGenGravity(const tVector &g) const
{
    tVectorXd Q_gravity = tVectorXd::Zero(num_of_freedom);
    for (int link_id = 0; link_id < GetNumOfLinks(); link_id++)
    {
        auto link = GetLinkById(link_id);
        const tMatrixXd &jvT = link->GetJKv().transpose();
        tVector3d mg = link->GetMass() * g.segment(0, 3);
        // std::cout << "jvt = " << jvT << std::endl;
        // std::cout << "mg = " << mg.transpose() << std::endl;
        // std::cout << "Q size " << Q_gravity.size() << std::endl;
        // std::cout << "jvt size " << jvT.rows() << std::endl;
        Q_gravity += jvT * mg;
    }
    return Q_gravity;
}

void cRobotModelDynamics::ApplyForce3d(int link_id, const tVector3d &f,
                                       const tVector3d &applied_pos)
{
    tVector force = btMathUtil::Expand(f, 0);
    tVector pos = btMathUtil::Expand(applied_pos, 1);
    cRobotModelDynamics::ApplyForce(link_id, force, pos);
}
void cRobotModelDynamics::ApplyForce(int link_id, const tVector &force,
                                     const tVector &applied_pos)
{
    if (link_id < 0 || link_id >= GetNumOfLinks())
    {
        std::cout << "ApplyForce: illegal link id " << link_id << std::endl;
        exit(0);
    }

    // 1. apply the force directly
    mLinkForces[link_id] += force;

    // 2. calculate the torque and apply it
    auto *link = GetLinkById(link_id);
    tVector rel_pos = applied_pos - btMathUtil::Expand(link->GetWorldPos(), 1);
    mLinkTorques[link_id] += rel_pos.cross3(force);
}

void cRobotModelDynamics::ApplyGeneralizedForce(int dof_id, double value)
{
    if (dof_id < 0 || dof_id >= GetNumOfFreedom())
    {
        std::cout << "[error] add generalized force on dof " << dof_id
                  << " illegal\n";
        exit(0);
    }
    mGenForce[dof_id] += value;
}

/**
 * \brief					add torques on multibody links
 * \param	link_id			target link id
 * \param	torque			applied torque in world frame
 */
void cRobotModelDynamics::ApplyLinkTorque(int link_id, const tVector &torque)
{
    if (link_id < 0 || link_id >= GetNumOfLinks())
    {
        std::cout << "ApplyLinkTorque: illegal link id " << link_id
                  << std::endl;
        exit(0);
    }

    mLinkTorques[link_id] += torque;
}

/**
 * \brief					add torque on multibody joints
 * \param	joint_id		target joint id
 * \param	torque			joint torque applied on the joint in
 * world frame
 */
void cRobotModelDynamics::ApplyJointTorque(int joint_id, const tVector &torque)
{
    if (joint_id < 0 || joint_id >= GetNumOfJoint())
    {
        std::cout << "ApplyJointTorque: illegal joint id " << joint_id
                  << std::endl;
        exit(0);
    }

    // one pair of opposite torque
    int parent_id = GetJointById(joint_id)->GetParentId(), child_id = joint_id;
    // std::cout << "[debug] add joint torque for parent " << parent_id << ": "
    // << -torque.transpose() << std::endl; std::cout << "[debug] add joint
    // torque for child " << joint_id << ": " << torque.transpose() <<
    // std::endl;
    if (parent_id >= 0)
        mLinkTorques[parent_id] -= torque;
    mLinkTorques[joint_id] += torque;
}

tVectorXd cRobotModelDynamics::DebugGetGeneralizedForce() { return mGenForce; }
tVectorXd cRobotModelDynamics::GetGeneralizedForce()
{
    tVectorXd Q = tVectorXd::Zero(GetNumOfFreedom());
    for (int i = 0; i < GetNumOfLinks(); i++)
    {
        const auto &link = GetLinkById(i);
        Q += link->GetJKv().transpose() * mLinkForces[i].segment(0, 3);
        Q += link->GetJKw().transpose() * mLinkTorques[i].segment(0, 3);
        // std::cout << "[model] link " << i << " torque " <<
        // link_torques[i].transpose() << std::endl;
    }

    // add lefted generalized force
    Q += mGenForce;
    return Q;
}

btGenRobotCollider *cRobotModelDynamics::GetLinkCollider(int link_id)
{
    if (link_id < 0 || link_id >= mColliders.size())
    {
        std::cout << "[error] GetLinkCollider invalid link id " << link_id
                  << std::endl;
        exit(0);
    }
    return mColliders[link_id];
}

void cRobotModelDynamics::ClearForce()
{
    for (int i = 0; i < GetNumOfLinks(); i++)
    {
        mLinkForces[i].setZero();
        mLinkTorques[i].setZero();
    }
    mGenForce.setZero();
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

// time integrate
extern int global_frame_id;
// extern bool gPauseSimulation;
void cRobotModelDynamics::UpdateVelocity(double dt, bool verbose /* = false*/)
{
    // std::cout << "----------------update vel begin-----------------\n";
    // 1. calculate generalized force
    int n = GetNumOfFreedom();
    tVectorXd Q = GetGeneralizedForce();

    // std::cout << "Q = " << Q.transpose() << std::endl;
    // 2. calculate qddot and new qdot
    // tMatrixXd mass_matrix_inv = (mass_matrix + 1e-5 *
    // tMatrixXd::Identity(mass_matrix.rows(), mass_matrix.cols())).inverse();
    // tVectorXd residual = (Q - (coriolis_matrix + damping_matrix) * qdot);
    tVectorXd qddot = Getqddot();

    if (qddot.hasNaN())
    {
        std::cout << "UpdateVelocity: qddot hasNan\n";
        exit(0);
    }
    if (verbose)
    {
        std::cout << "[model] qddot = " << qddot.transpose() << std::endl;
        // std::cout << "[model] Q = " << Q.transpose() << std::endl;
    }
    mqdot += qddot * dt;
    mqdot = mqdot.cwiseMax(-mMaxVel);
    mqdot = mqdot.cwiseMin(mMaxVel);
    // 3. recalculate the coriolis force
    ComputeCoriolisMatrix(mqdot);
    UpdateCartesianVelocity();
    // std::cout << "----------------update vel end-----------------\n";
}

void cRobotModelDynamics::UpdateVelocityAndTransform(double dt)
{
    int n = GetNumOfFreedom();
    tVectorXd Q = GetGeneralizedForce();

    // std::cout << "Q = " << Q.transpose() << std::endl;
    // 2. calculate qddot and new qdot
    // tMatrixXd mass_matrix_inv = (mass_matrix + 1e-5 *
    // tMatrixXd::Identity(mass_matrix.rows(), mass_matrix.cols())).inverse();
    // tVectorXd residual = (Q - (coriolis_matrix + damping_matrix) * qdot);
    tVectorXd qddot = Getqddot();

    if (qddot.hasNaN())
    {
        std::cout << "UpdateVelocityAndTransform: qddot hasNan\n";
        assert(false);
        exit(0);
    }
    mqdot += qddot * dt;
    mqdot = mqdot.cwiseMax(-mMaxVel);
    mqdot = mqdot.cwiseMin(mMaxVel);

    // update transform
    mq = mq + dt * mqdot;

    if (mEnableEulerAngleClamp)
    {
        for (int i = 3; i < mq.size(); i++)
        {
            while (mq[i] < -M_PI)
                mq[i] += 2 * M_PI;
            while (mq[i] > M_PI)
                mq[i] -= 2 * M_PI;
        }
    }
    Apply(mq, true);

    // 2. recalc mats
    ComputeMassMatrix();
    ComputeCoriolisMatrix(mqdot);
    ComputeDampingMatrix();
    UpdateCartesianVelocity();

    // 3. write to bullet
    SyncToBullet();
}
// time integrate
void cRobotModelDynamics::UpdateTransform(double dt)
{
    // 1. calculate new q and update links and joints
    // std::ofstream fout(path, std::ios::app);
    // fout << "frame " << global_frame_id << " qdot[4] = " << qdot[4] <<
    // std::endl; fout << "frame " << global_frame_id << " qold[4] = " << q[4]
    // << std::endl; fout << "frame " << global_frame_id << " dt = " << dt <<
    // std::endl;
    mq = mq + dt * mqdot;
    // fout << "frame " << global_frame_id << " qnew[4] = " << q[4] <<
    // std::endl;

    if (mEnableEulerAngleClamp)
    {
        for (int i = 3; i < mq.size(); i++)
        {
            while (mq[i] < -M_PI)
                mq[i] += 2 * M_PI;
            while (mq[i] > M_PI)
                mq[i] -= 2 * M_PI;
        }
    }

    Apply(mq, true);

    // 2. recalc mats
    ComputeMassMatrix();
    ComputeCoriolisMatrix(mqdot);
    ComputeDampingMatrix();
    UpdateCartesianVelocity();

    // 3. write to bullet
    SyncToBullet();
    // fout << "frame " << global_frame_id << " qfinal[4] = " << q[4] <<
    // std::endl; fout << std::endl; fout.close();
}

// only position is saved in bullet
void cRobotModelDynamics::SyncToBullet()
{
    btTransform trans;
    for (int i = 0; i < GetNumOfLinks(); i++)
    {
        auto link = GetLinkById(i);
        trans.setOrigin(btVector3(link->GetWorldPos()[0],
                                  link->GetWorldPos()[1],
                                  link->GetWorldPos()[2]));
        trans.setRotation(btBulletUtil::tQuaternionTobtQuaternion(
            btMathUtil::RotMatToQuaternion(link->GetGlobalTransform())));

        mColliders[i]->setWorldTransform(trans);
    }
}

/**
 * \brief           push current model state into the stack
*/
void cRobotModelDynamics::PushState(const std::string &tag,
                                    bool only_vel_and_force)
{
    if (mStateStack.size() >= mStackCapacity)
    {
        std::cout << "[error] cRobotModelDynamics::PushState exceed length "
                  << mStackCapacity << " for tag " << tag;
        exit(0);
    }
    tStateRecord *state = new tStateRecord();
    state->only_vel_and_force_recorded = only_vel_and_force;
    state->qdot = mqdot;
    state->link_forces = mLinkForces;
    state->link_torques = mLinkTorques;

    state->damping_matrix = mDampingMatrix;
    state->coriolis_matrix = coriolis_matrix;
    state->generalized_force = mGenForce;
    state->compute_2rd_derive = compute_second_deriv;
    state->compute_3ed_derive = compute_third_deriv;
    if (only_vel_and_force == false)
    {
        state->q = mq;
        state->mass_matrix = mass_matrix;
    }

    mStateStack.push_back(std::make_pair(tag, state));
}

/**
 * \brief           pop current model state outof the stack
*/
void cRobotModelDynamics::PopState(const std::string &tag,
                                   bool only_vel_and_force)
{
    if (mStateStack.size() == 0)
    {
        std::cout << "[error] RobotModel stack is empty when you try to pop "
                  << tag << std::endl;
        exit(0);
    }
    else if (mStateStack.back().first != tag)
    {
        std::cout << "[error] RobotModel stack trying to pop "
                  << mStateStack[mStateStack.size() - 1].first
                  << " but the user try to do " << tag << std::endl;
        exit(0);
    }

    auto state = mStateStack.back().second;
    if (state->only_vel_and_force_recorded != only_vel_and_force)
    {
        std::cout << "[error] RobotModel stack trying to pop "
                  << state->only_vel_and_force_recorded
                  << " but the user try to do " << only_vel_and_force
                  << std::endl;
        exit(0);
    }

    mqdot = state->qdot;
    mLinkForces = state->link_forces;
    mLinkTorques = state->link_torques;
    mGenForce = state->generalized_force;
    coriolis_matrix = state->coriolis_matrix;
    mDampingMatrix = state->damping_matrix;
    SetComputeSecondDerive(state->compute_2rd_derive);
    SetComputeThirdDerive(state->compute_3ed_derive);

    if (only_vel_and_force == false)
    {
        mq = state->q;
        mass_matrix = state->mass_matrix;
        Apply(mq, true);
        ComputeMassMatrix();
    }

    ComputeCoriolisMatrix(mqdot);
    ComputeDampingMatrix();
    UpdateCartesianVelocity();
    SyncToBullet();
    delete state;
    mStateStack.pop_back();
}

bool cRobotModelDynamics::IsGeneralizedMaxVel(double max_vel /*= 100*/) const
{
    return std::fabs((mqdot.cwiseAbs().maxCoeff() - max_vel)) < 1e-10;
}

double cRobotModelDynamics::GetMaxVelThreshold() const { return mMaxVel; }

bool cRobotModelDynamics::IsCartesianMaxVel(double max_vel /*= 100*/) const
{
    for (int i = 0; i < GetNumOfLinks(); i++)
    {
        auto link = static_cast<Link *>(GetLinkById(i));
        if (link->GetLinkVel().cwiseAbs().maxCoeff() > max_vel ||
            link->GetLinkOmega().cwiseAbs().maxCoeff() > max_vel)
        {
            return true;
        }
    }
    return false;
}

double cRobotModelDynamics::GetMaxVel() const { return mMaxVel; }
void cRobotModelDynamics::SetDampingCoeff(double damp1, double damp2)
{
    mDampingCoef1 = damp1;
    mDampingCoef2 = damp2;
    // std::cout << "damping = " << damping_coef << std::endl;
    // exit(0);
}

// void cRobotModelDynamics::SetAngleClamp(bool val)
// {
// 	this->mEnableEulerAngleClamp = val;
// }

void cRobotModelDynamics::SetMaxVel(double val) { mMaxVel = val; }

// /**
//  * \brief			Update generalized velocity by RK4
// */
// void cRobotModelDynamics::UpdateRK4(double dt)
// {
// 	// std::cout <<"update rk4";
// 	// exit(0);
// 	tVectorXd k1(num_of_freedom * 2), k2(num_of_freedom * 2),
// k3(num_of_freedom * 2), k4(num_of_freedom * 2); 	tVectorXd y_dot = mqdot,
// y = mq; 	tVectorXd Q; 	tVectorXd qddot;
// 	// k1: we need to recalculate M, recalculate Q, recalculate C by given
// 	{
// 		UpdateRK4InternalUpdate(y, y_dot, Q);
// 		qddot = mass_matrix.inverse() * (Q - coriolis_matrix * y_dot);

// 		k1.segment(0, num_of_freedom) = y_dot + dt * qddot;
// 		k1.segment(num_of_freedom, num_of_freedom) = qddot;

// 		k1 *= dt;
// 	}

// 	{
// 		y += 0.5 * k1.segment(0, num_of_freedom);
// 		y_dot += 0.5 * k1.segment(num_of_freedom, num_of_freedom);
// 		UpdateRK4InternalUpdate(y, y_dot, Q);

// 		qddot = mass_matrix.inverse() * (Q - coriolis_matrix * y_dot);

// 		k2.segment(0, num_of_freedom) = y_dot + dt * qddot;
// 		k2.segment(num_of_freedom, num_of_freedom) = qddot;

// 		k2 *= dt;
// 	}

// 	{
// 		y += 0.5 * k2.segment(0, num_of_freedom);
// 		y_dot += 0.5 * k2.segment(num_of_freedom, num_of_freedom);
// 		UpdateRK4InternalUpdate(y, y_dot, Q);
// 		qddot = mass_matrix.inverse() * (Q - coriolis_matrix * y_dot);

// 		k3.segment(0, num_of_freedom) = y_dot + dt * qddot;
// 		k3.segment(num_of_freedom, num_of_freedom) = qddot;

// 		k3 *= dt;
// 	}

// 	{
// 		y += k3.segment(0, num_of_freedom);
// 		y_dot += k3.segment(num_of_freedom, num_of_freedom);
// 		UpdateRK4InternalUpdate(y, y_dot, Q);

// 		qddot = mass_matrix.inverse() * (Q - coriolis_matrix * y_dot);

// 		k4.segment(0, num_of_freedom) = y_dot + dt * qddot;
// 		k4.segment(num_of_freedom, num_of_freedom) = qddot;
// 		k4 *= dt;
// 	}

// 	mq = mq + (k1.segment(0, num_of_freedom) + 2 * k2.segment(0,
// num_of_freedom) + 2 * k3.segment(0, num_of_freedom) + k4.segment(0,
// num_of_freedom)) / 6;

// 	mqdot = mqdot + (k1.segment(num_of_freedom, num_of_freedom) + 2 *
// k2.segment(num_of_freedom, num_of_freedom) + 2 * k3.segment(num_of_freedom,
// num_of_freedom) + k4.segment(num_of_freedom, num_of_freedom)) / 6;

// 	mqdot = mqdot.cwiseMax(-mMaxVel);
// 	mqdot = mqdot.cwiseMin(mMaxVel);

// 	// apply and clear all forces
// 	UpdateRK4InternalUpdate(mq, mqdot, Q);
// 	UpdateCartesianVelocity();
// 	ClearForce();
// 	SyncToBullet();
// }

// void cRobotModelDynamics::UpdateRK4InternalUpdate(tVectorXd& q, tVectorXd&
// qdot, tVectorXd& Q)
// {
// 	Apply(q, true);
// 	ComputeMassMatrix();
// 	ComputeCoriolisMatrix(qdot);
// 	Q.resize(num_of_freedom);
// 	Q.setZero();

// 	int n = GetNumOfFreedom();
// 	for (int i = 0; i < GetNumOfLinks(); i++)
// 	{
// 		const auto& link = GetLinkById(i);
// 		Q += link->GetJKv().transpose() * mLinkForces[i].segment(0, 3);
// 		Q += link->GetJKw().transpose() * mLinkTorques[i].segment(0, 3);
// 	}
// }

void cRobotModelDynamics::GetEffectInfo(tEigenArr<tVector> &force_array,
                                        tEigenArr<tVector> &torque_array)
{
    force_array = this->mLinkForces;
    torque_array = this->mLinkTorques;
}

tMatrixXd cRobotModelDynamics::GetInvMassMatrix() { return inv_mass_matrix; }

tVectorXd cRobotModelDynamics::Getqddot()
{
    tVectorXd residual =
        (GetGeneralizedForce() - (coriolis_matrix + mDampingMatrix) * mqdot);
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
        std::cout << "UpdateVelocityWithoutCoriolis: qddot hasNan\n";
        exit(0);
    }
    mqdot += qddot * dt;
    mqdot = mqdot.cwiseMax(-mMaxVel);
    mqdot = mqdot.cwiseMin(mMaxVel);

    // 3. recalculate the coriolis force
    ComputeCoriolisMatrix(mqdot);

    // 4. update the cartesian velocity for each link
    UpdateCartesianVelocity();
}

/**
 * \brief				Update cartesian velocity (lin vel and
 * ang vel) for each link by the generalized velocity qdot
 *
 * 		v = Jv qdot
 * 		w = Jw qdot
 */
void cRobotModelDynamics::UpdateCartesianVelocity()
{
    int num_of_links = GetNumOfLinks();
    for (int i = 0; i < num_of_links; i++)
    {
        auto link = static_cast<Link *>(GetLinkById(i));
        link->SetLinkVel((link->GetJKv() * mqdot).segment(0, 3));
        link->SetLinkOmega((link->GetJKw() * mqdot).segment(0, 3));
    }
}

/**
 * \brief			test how to rotate / move the whole character
 *
 */
#define expand(a) btMathUtil::Expand(a, 0)
void cRobotModelDynamics::TestRotationChar()
{
    std::cout << "begin to test rotate char\n";
    mq.setRandom();
    mqdot.setRandom();
    SetqAndqdot(mq, mqdot);

    // // get current root link rotation, vel and omega
    // 1. record new info
    int num_of_links = GetNumOfLinks();
    auto link = static_cast<Link *>(GetLinkById(0));
    tMatrix old_rotation = tMatrix::Zero();
    old_rotation.block(0, 0, 3, 3) = link->GetWorldOrientation();
    tVector old_link_vel = expand(link->GetLinkVel()),
            old_link_omega = expand(link->GetLinkOmega());

    // 2. set up new rotation, calculate new vel and new omega
    tVector random_rot_euler_angles = tVector::Random();
    tMatrix diff_rot = btMathUtil::EulerAnglesToRotMat(random_rot_euler_angles,
                                                       btRotationOrder::bt_XYZ);
    // std::cout << "root link old rot = \n"
    // 		  << old_rotation << std::endl;
    // std::cout << "root link diff rot = \n"
    // 		  << diff_rot << std::endl;
    // std::cout << "root link init rot = \n"
    // 		  << link->GetMeshRotation() << std::endl;

    tMatrix new_rotation =
        diff_rot * old_rotation *
        btMathUtil::ExpandMat(link->GetMeshRotation()).transpose();
    std::cout << "new rot = \n" << new_rotation << std::endl;

    tVector new_euler_angles = btMathUtil::QuaternionToEulerAngles(
        btMathUtil::RotMatToQuaternion(new_rotation), btRotationOrder::bt_XYZ);
    tVector new_link_vel =
        diff_rot * old_link_vel; // diff rotation in world frame
    tVector new_link_omega = diff_rot * old_link_omega;

    std::vector<tVector3d> old_link_vel_array(num_of_links, tVector3d::Zero()),
        old_link_omega_array(num_of_links, tVector3d::Zero());
    std::vector<tMatrix3d> old_rotation_array(num_of_links, tMatrix3d::Zero());
    for (int i = 0; i < num_of_links; i++)
    {
        auto link = static_cast<Link *>(GetLinkById(i));
        old_rotation_array[i] = link->GetWorldOrientation();
        old_link_omega_array[i] = link->GetLinkOmega();
        old_link_vel_array[i] = link->GetLinkVel();
    }

    // 3. given new q into the result and update
    mq.segment(3, 3) = new_euler_angles.segment(0, 3);
    Apply(mq, true);

    // 4. given new link vel to qdot

    std::cout << "root link jkv = \n" << link->GetJKv() << std::endl;
    mqdot.segment(3, 3) =
        link->GetJKw().block(0, 3, 3, 3).inverse() * new_link_omega;
    mqdot.segment(0, 3) =
        new_link_vel.segment(0, 3) -
        link->GetJKv().block(0, 3, 3, 3) * mqdot.segment(3, 3);
    SetqAndqdot(mq, mqdot);

    // 5. begin to check
    {
        for (int i = 0; i < num_of_links; i++)
        {
            auto link = static_cast<Link *>(GetLinkById(i));
            tMatrix3d new_rot = link->GetWorldOrientation();
            tVector3d new_omega = link->GetLinkOmega();
            tVector3d new_vel = link->GetLinkVel();
            std::cout << "link " << i << " rot diff = \n"
                      << new_rot -
                             diff_rot.block(0, 0, 3, 3) * old_rotation_array[i]
                      << std::endl;
            std::cout << "link " << i << " vel diff = \n"
                      << new_vel -
                             diff_rot.block(0, 0, 3, 3) * old_link_vel_array[i]
                      << std::endl;
            std::cout << "link " << i << " omega diff = \n"
                      << new_omega - diff_rot.block(0, 0, 3, 3) *
                                         old_link_omega_array[i]
                      << std::endl;
        }
    }
    exit(0);
}

/**
 * \brief					Contact aware controller, used in
 * "contact-aware nonlienar dynamics controller". This pointer will be used when
 * the LCP constraints are handled in ConstraintData.cpp
 */
void cRobotModelDynamics::SetContactAwareController(
    btGenContactAwareController *ptr)
{
    mController = ptr;
}

btGenContactAwareController *
cRobotModelDynamics::GetContactAwareController() const
{
    return mController;
}

/***
 * \brief					Check whether the contact aware
 * controller are enabled
 */
bool cRobotModelDynamics::GetEnableContactAwareController() const
{
    return mEnableContactAwareController;
}

void cRobotModelDynamics::SetEnableContactAwareController(bool val)
{
    mEnableContactAwareController = val;
}

bool cRobotModelDynamics::GetCollisionEnabled() const
{
    return mEnableCollision;
}

/**
 * \brief           d QG dq, shape = n * n
*/
tMatrixXd cRobotModelDynamics::CalcdGenGravitydq(const tVector &g) const
{
    int global_dof = GetNumOfFreedom();
    tMatrixXd dQGdq = tMatrixXd::Zero(global_dof, global_dof);
    tVector3d g3d = g.segment(0, 3);
    for (int id = 0; id < GetNumOfLinks(); id++)
    {
        auto link = dynamic_cast<Link *>(GetLinkById(id));
        for (int dof = 0; dof < global_dof; dof++)
        {
            dQGdq.col(dof) += link->GetTotalDofdJKv_dq(dof).transpose() *
                              link->GetMass() * g3d;
        }
    }
    return dQGdq;
}

/**
 * \brief           Test dQGdq numerically
*/
void cRobotModelDynamics::TestdGenGravitydq(const tVector &g)
{
    int global_dof = GetNumOfFreedom();
    tVectorXd QG_old = CalcGenGravity(g);
    tMatrixXd dQGdq = CalcdGenGravitydq(g);
    double eps = 1e-6;
    tVectorXd q = mq;
    for (int dof = 0; dof < global_dof; dof++)
    {
        q[dof] += eps;
        this->SetqAndqdot(q, mqdot);

        tVectorXd QG_new = CalcGenGravity(g);
        tVectorXd dQGDq_num = (QG_new - QG_old) / eps;
        tVectorXd diff = dQGDq_num - dQGdq.col(dof);
        std::cout << "[log] TestdGenQGdq: dof " << dof
                  << " diff = " << diff.norm() << std::endl;
        if (diff.norm() > eps * 10)
        {
            std::cout << "[error] verified failed\n";
            exit(0);
        }
        q[dof] -= eps;
    };
    SetqAndqdot(q, mqdot);
    std::cout << "[log] verify dQGdq succ\n";
}

/**
 * \brief       Test ddJvdqq and ddJwdqq
*/

void cRobotModelDynamics::TestThirdJacobian()
{
    PushState("test_third_jacobian");
    tVectorXd q_old = mq;
    // 1. get the old dJvdq and ddJvdqq
    for (int i = 0; i < GetNumOfLinks(); i++)
    {
        TestLinkddJvddq(i);
        TestLinkddJwddq(i);
    }
    std::cout << "Test Thrid Jacobian succ\n";
    PopState("test_third_jacobian");
}

EIGEN_V_MATXD GetLinkdJxdq(Link *link, char type)
{
    int global_freedoms = link->GetGlobalFreedoms();
    EIGEN_V_MATXD dJxdq(global_freedoms);
    for (int i = 0; i < global_freedoms; i++)
    {
        switch (type)
        {
        case 'V':
            dJxdq[i] = link->GetTotalDofdJKv_dq(i);
            break;
        case 'W':
            dJxdq[i] = link->GetTotalDofdJKw_dq(i);
            break;

        default:
            std::cout << "[error] illegal type in GetLink dJxdq " << type
                      << std::endl;
            exit(1);
            break;
        }
    }

    return dJxdq;
}
EIGEN_V_MATXD GetLinkdJxdqDivision(const EIGEN_V_MATXD &left,
                                   const EIGEN_V_MATXD &right, double deno)
{
    assert(left.size() == right.size());
    for (int i = 0; i < left.size(); i++)
        assert(left[i].size() == right[i].size());

    EIGEN_V_MATXD result(left.size());
    for (int i = 0; i < left.size(); i++)
    {
        result[i] = (left[i] - right[i]) / deno;
    }
    return result;
}

EIGEN_VV_MATXD GetLinkddJxdqq(Link *link, char type)
{
    int global_freedoms = link->GetGlobalFreedoms();
    EIGEN_VV_MATXD ddJxddq(global_freedoms);
    for (auto &x : ddJxddq)
        x.resize(global_freedoms);
    for (int i = 0; i < global_freedoms; i++)
        for (int j = 0; j < global_freedoms; j++)
        {
            switch (type)
            {
            case 'V':
                ddJxddq[i][j] = link->GetTotalDofddJKv_dqq(i, j);
                // if (i == 3 && j == 3)
                // {
                //     printf("get link %d d^2Jkv/dq%dq%d = \n", link->GetId(), i, j);
                //     std::cout << ddJxddq[i][j] << std::endl;
                //     exit(1);
                // }

                break;
            case 'W':
                ddJxddq[i][j] = link->GetTotalDofddJKw_dqq(i, j);
                break;

            default:
                std::cout << "[error] illegal type in GetLink ddJxddq " << type
                          << std::endl;
                exit(1);
                break;
            }
        }

    return ddJxddq;
}

/**
 * \brief       Test the ddJvddq for link id
*/
void cRobotModelDynamics::TestLinkddJvddq(int id)
{
    PushState("test_link_ddjv_ddq");
    // 1. get the intermediate result
    auto link = dynamic_cast<Link *>(GetLinkById(id));
    EIGEN_V_MATXD dJvdq_old = GetLinkdJxdq(link, 'V');
    EIGEN_VV_MATXD ddJvddq_analytic = GetLinkddJxdqq(link, 'V');

    // 2. check the numerical derivatives
    tVectorXd q_old = mq;
    double eps = 1e-6;
    for (int i = 0; i < num_of_freedom; i++)
    {
        q_old[i] += eps;
        SetqAndqdot(q_old, mqdot);

        EIGEN_V_MATXD dJvdq_new = GetLinkdJxdq(link, 'V');
        EIGEN_V_MATXD num_i = GetLinkdJxdqDivision(dJvdq_new, dJvdq_old, eps);

        const EIGEN_V_MATXD &ana_i = ddJvddq_analytic[i];
        for (int j = 0; j < num_of_freedom; j++)
        {
            tMatrixXd diff = ana_i[j] - num_i[j];
            double diff_norm = diff.norm();

            if (diff_norm > eps)
            {
                printf("[error] link %d ddJvdq%dq%d diff norm %.10f\n", id, j,
                       i, diff_norm);
                std::cout << "ana = \n" << ana_i[j] << std::endl;
                std::cout << "num = \n" << num_i[j] << std::endl;
                std::cout << "dJvdq new = \n" << dJvdq_new[j] << std::endl;
                std::cout << "dJvdq old = \n" << dJvdq_old[j] << std::endl;
                exit(0);
            }
            // if (id == 0 && j == 3 && i == 0)
            // {
            //     std::cout << "ana = \n" << ana_i[j] << std::endl;
            //     std::cout << "num = \n" << num_i[j] << std::endl;
            //     std::cout << "dJvdq new = \n" << dJvdq_new[j] << std::endl;
            //     std::cout << "dJvdq old = \n" << dJvdq_old[j] << std::endl;
            //     exit(1);
            // }
        }
        q_old[i] -= eps;
    }
    PopState("test_link_ddjv_ddq");
    printf("Test Link %d ddJvddq succ\n", id);
}
void cRobotModelDynamics::TestLinkddJwddq(int id)
{
    PushState("test_link_ddjw_ddq");
    // 1. get the intermediate result
    auto link = dynamic_cast<Link *>(GetLinkById(id));
    EIGEN_V_MATXD dJwdq_old = GetLinkdJxdq(link, 'W');
    EIGEN_VV_MATXD ddJwddq_analytic = GetLinkddJxdqq(link, 'W');
    SetComputeThirdDerive(false);

    // 2. check the numerical derivatives
    tVectorXd q_old = mq;
    double eps = 1e-6;
    for (int i = 0; i < num_of_freedom; i++)
    {
        q_old[i] += eps;
        SetqAndqdot(q_old, mqdot);

        EIGEN_V_MATXD dJwdq_new = GetLinkdJxdq(link, 'W');
        EIGEN_V_MATXD num_i = GetLinkdJxdqDivision(dJwdq_new, dJwdq_old, eps);

        const EIGEN_V_MATXD &ana_i = ddJwddq_analytic[i];
        for (int j = 0; j < num_of_freedom; j++)
        {
            tMatrixXd diff = ana_i[j] - num_i[j];
            double diff_norm = diff.norm();
            if (diff_norm > eps * 10)
            {
                printf("[error] link %d ddJwdq%dq%d diff norm %.10f\n", id, j,
                       i, diff_norm);
                std::cout << "ana = \n" << ana_i[j] << std::endl;
                std::cout << "num = \n" << num_i[j] << std::endl;
                exit(0);
            }
        }
        q_old[i] -= eps;
    }
    PopState("test_link_ddjw_ddq");
    printf("Test Link %d ddJwddq succ\n", id);
}

void cRobotModelDynamics::TestDCoriolisMatrixDq()
{
    // test dCdq reduced and C reduced
    // {
    //     for (int i = 0; i < GetNumOfLinks(); i++)
    //     {
    //         // TestDCoriolisMatrixDq_part1(i);
    //         // TestDCoriolisMatrixDq_part2(i);
    //         TestDCoriolisMatrixDq_global_link(i);
    //     }
    //     std::cout << "[log] TestDcDq part1/2 for all links succ\n";
    //     exit(1);
    // }
    PushState("test_dcdq");
    ComputeCoriolisMatrix(mqdot);
    EIGEN_V_MATXD dCdq;
    ComputeDCoriolisMatrixDq(mqdot, dCdq);
    tMatrixXd C_old = coriolis_matrix;

    tVectorXd q_old = mq;
    double eps = 1e-7;
    for (int i = 0; i < GetNumOfFreedom(); i++)
    {

        q_old[i] += eps;
        SetqAndqdot(q_old, mqdot);

        tMatrixXd C_new = coriolis_matrix;
        tMatrixXd dCdq_num = (C_new - C_old) / eps;
        tMatrixXd dCdq_diff = dCdq_num - dCdq[i];
        double norm = dCdq_diff.norm();
        if (norm > 100 * eps)
        {
            std::cout << "[error] TestDCDq global dof " << i
                      << " dCdq diff norm = " << norm << std::endl;
            std::cout << "ana = \n" << dCdq[i] << std::endl;
            std::cout << "num = \n" << dCdq_num << std::endl;
            exit(1);
        }
        // std::cout << "[log] dof " << i << " dCdq succ\n";
        q_old[i] -= eps;
    }
    std::cout << "[log] Test reduced dCdq succ\n";
    PopState("test_dcdq");
}

void cRobotModelDynamics::TestDCoriolisMatrixDqdot()
{
    PushState("test_dCdqdot");
    tVectorXd qdot_old = mqdot;
    double eps = 1e-7;
    tMatrixXd C_old = GetCoriolisMatrix();
    EIGEN_V_MATXD dCdqdot_ana;
    ComputeDCoriolisMatrixDqdot(dCdqdot_ana);

    for (int i = 0; i < GetNumOfFreedom(); i++)
    {
        qdot_old[i] += eps;
        SetqAndqdot(mq, qdot_old);
        tMatrixXd C_new = GetCoriolisMatrix();

        tMatrixXd dCdqdot_num = (C_new - C_old) / eps;
        tMatrixXd dCdqdot_diff = dCdqdot_num - dCdqdot_ana[i];
        double norm = dCdqdot_diff.norm();
        if (norm > eps * 10)
        {
            printf("[error] TestDcDqdot for dof %d diff %.10f\n", i, norm);
            std::cout << "num = " << dCdqdot_num << std::endl;
            std::cout << "ana = " << dCdqdot_ana[i] << std::endl;
            exit(1);
        }
        printf("[log] TestDcDqdot dof %d succ, value norm = %.10f\n", i,
               dCdqdot_num.norm());
        qdot_old[i] -= eps;
    }
    printf("[log] TestDcDqdot succ\n");
    PopState("test_dCdqdot");
}

/**
 * \brief               Compute dCdq
 *      (dCdq has a very complex expression)
 * 
 * (NOT IMPLEMENTED)reduced version formula:
 * dCdq = \sum_k
 * {
 *      dJkTdq * (
 *              Mck * \dot{J}_k 
 *              + [\tilde{\omega_k} * Mck * Jk]
 *              )   // it's the first part d "JkT * Mck * \dot{J}_k"  w.r.t q
 *          +
 *      JkT * (dMckdq * \dot{J}_k
 *              + Mck * d\dot{J}dq
 *              + d[\tilde{\omega_k}]dq * Mck
 *              + [\tilde{\omega_k}] * dMckdq
 *          )       // it's the second part d "JkT * [\tilde{\omega_k}] * Mck * Jk" w.r.t q
 * }
 * 
 * (IMPLEMENTED) redundant version formula:
 * C = part1 + part2
 * part1 = JkT * Mck * \dot{J}k
 * part2 = JkT * [\tilde{\omega}_k] * Mck * Jk
 * 
 * dCdq = dpart1/dq + dpart2/dq
 * dpart1/dq = dJkTdq * Mck * \dot{J}_k + JkT * (dMckdq * \dot{J}_k + Mck * d\dot{J}k/dq)
 * dpart2/dq = dJkTdq * [\tilde{\omega_k}] * Mck * Jk
 *              + JkT * (d[\tilde{\omega}_k] * Mck * Jk + [\tilde{\omega}_k}] * 
 *                              (
 *                                   dMckdq * Jk + Mck * dJkdq   
 *                              )
 *                      )
*/
void cRobotModelDynamics::ComputeDCoriolisMatrixDq(const tVectorXd &qdot,
                                                   EIGEN_V_MATXD &dCdq)
{
    int global_dof = GetNumOfFreedom();
    dCdq.resize(global_dof);
    for (auto &x : dCdq)
        x = tMatrixXd::Zero(global_dof, global_dof);

    EIGEN_V_MATXD dCdq_reduced;
    for (int link_id = 0; link_id < GetNumOfLinks(); link_id++)
    {
        auto link = dynamic_cast<Link *>(GetLinkById(link_id));
        link->ComputedCoriolisMatrixdqReduced(qdot, dCdq_reduced);
        // map the current dCdq to the global dCdq
        auto dependent_freedom = link->GetPrevFreedomIds();
        int total_freedoms = link->GetNumTotalFreedoms();
        for (int local_dof = 0; local_dof < dependent_freedom.size();
             local_dof++)
        {
            for (int i = 0; i < total_freedoms; i++)
                for (int j = 0; j < total_freedoms; j++)
                {
                    dCdq[dependent_freedom[local_dof]](dependent_freedom[i],
                                                       dependent_freedom[j]) +=
                        dCdq_reduced[local_dof](i, j);
                }
        }
    }
}

// void cRobotModelDynamics::TestDCoriolisMatrixDq_part1(int link_id)
// {
//     PushState("test_dcdq_1");
//     auto link = dynamic_cast<Link *>(GetLinkById(link_id));
//     EIGEN_V_MATXD dCdq_part1;
//     tMatrixXd C_part1_old;
//     link->ComputeCoriolisMatrixReduced_part1(mqdot, C_part1_old);
//     link->ComputedCoriolisMatrixdqReduced_part1(mqdot, dCdq_part1);

//     int total_freedoms = link->GetNumTotalFreedoms();
//     tVectorXd q_old = mq;
//     double eps = 1e-7;
//     tMatrixXd C_part1_new;
//     for (int i = 0; i < total_freedoms; i++)
//     {
//         int global_dof = link->GetPrevFreedomIds()[i];
//         q_old[global_dof] += eps;
//         SetqAndqdot(q_old, mqdot);
//         link->ComputeCoriolisMatrixReduced_part1(mqdot, C_part1_new);
//         tMatrixXd dCpart1_dq_num = (C_part1_new - C_part1_old) / eps;
//         tMatrixXd diff = dCpart1_dq_num - dCdq_part1[i];
//         double diff_norm = diff.norm();
//         if (diff_norm > 10 * eps)
//         {
//             printf(
//                 "[error] Test dCpart1_dq for link %d dof %d diff norm %.10f\n",
//                 link_id, i, diff_norm);
//             std::cout << "num = \n" << dCpart1_dq_num << std::endl;
//             std::cout << "ana = \n" << dCdq_part1[i] << std::endl;
//             exit(1);
//         }
//         // std::cout << "dCdq part1 for link " << link_id << " d" << i << " = \n"
//         //           << dCpart1_dq_num << std::endl;
//         q_old[global_dof] -= eps;
//     }
//     PopState("test_dcdq_1");
//     std::cout << "[log] Test DC_part1dq link " << link_id << " succ\n";
// }
// void cRobotModelDynamics::TestDCoriolisMatrixDq_part2(int link_id)
// {
//     PushState("test_dcdq_2");
//     auto link = dynamic_cast<Link *>(GetLinkById(link_id));
//     EIGEN_V_MATXD dCdq_part2;
//     tMatrixXd C_part2_old;
//     link->ComputeCoriolisMatrixReduced_part2(mqdot, C_part2_old);
//     link->ComputedCoriolisMatrixdqReduced_part2(mqdot, dCdq_part2);

//     int total_freedoms = link->GetNumTotalFreedoms();
//     tVectorXd q_old = mq;
//     double eps = 1e-7;
//     tMatrixXd C_part2_new;
//     for (int i = 0; i < total_freedoms; i++)
//     {
//         int global_dof = link->GetPrevFreedomIds()[i];
//         q_old[global_dof] += eps;
//         SetqAndqdot(q_old, mqdot);
//         link->ComputeCoriolisMatrixReduced_part2(mqdot, C_part2_new);
//         tMatrixXd dCpart2_dq_num = (C_part2_new - C_part2_old) / eps;
//         tMatrixXd diff = dCpart2_dq_num - dCdq_part2[i];
//         double diff_norm = diff.norm();
//         if (diff_norm > 10 * eps)
//         {
//             printf(
//                 "[error] Test dCpart2_dq for link %d dof %d diff norm %.10f\n",
//                 link_id, i, diff_norm);
//             std::cout << "num = \n" << dCpart2_dq_num << std::endl;
//             std::cout << "ana = \n" << dCdq_part2[i] << std::endl;
//             exit(1);
//         }
//         // std::cout << "dCdq part2 for link " << link_id << " d" << i << " = \n"
//         //           << dCpart2_dq_num << std::endl;
//         q_old[global_dof] -= eps;
//     }
//     PopState("test_dcdq_2");
//     std::cout << "[log] Test DC_part2dq link " << link_id << " succ\n";
// }
/**
 * \brief               Compute dCdqdot
 * 
 * dCdqdot = JkT * (Mck * d\dot{Jk}dq + d[\tilde{wk}]dqdot * Mck * J)
*/
void cRobotModelDynamics::ComputeDCoriolisMatrixDqdot(EIGEN_V_MATXD &dCdqdot)
{
    // 1. allocate
    int num_of_freedom = GetNumOfFreedom();
    dCdqdot.resize(num_of_freedom);
    for (auto &x : dCdqdot)
        x.noalias() = tMatrixXd::Zero(num_of_freedom, num_of_freedom);

    // 2. get dCdqdot on each link
    for (int link_id = 0; link_id < GetNumOfLinks(); link_id++)
    {
        auto link = dynamic_cast<Link *>(GetLinkById(link_id));
        int total_freedom = link->GetNumTotalFreedoms();
        auto dependent_dof = link->GetPrevFreedomIds();
        EIGEN_V_MATXD dCdqdot_link_reduced;
        link->ComputedCoriolisMatrixdqdotReduced(dCdqdot_link_reduced);

        // 3. map the value into global dCdqdot
        for (int slice = 0; slice < total_freedom; slice++)
        {
            for (int i = 0; i < total_freedom; i++)
                for (int j = 0; j < total_freedom; j++)
                {
                    dCdqdot[dependent_dof[slice]](dependent_dof[i],
                                                  dependent_dof[j]) +=
                        dCdqdot_link_reduced[slice](i, j);
                }
            //
        }
    }
}

/**
 * \brief               Compute dJdotdq for all links
*/
void cRobotModelDynamics::TestdJdotdq()
{
    PushState("test_djdot_dq");
    for (int i = 0; i < GetNumOfLinks(); i++)
    {
        TestLinkdJdotdq(i);
    }
    PopState("test_djdot_dq");
}

void cRobotModelDynamics::TestLinkdJdotdq(int id)
{
    PushState("test_link_djdot_dq");

    // 1. get the middle value
    auto link = dynamic_cast<Link *>(GetLinkById(id));
    tMatrixXd jvdot_old = link->GetJKvdot_reduced();
    tMatrixXd jwdot_old = link->GetJKwdot_reduced();

    tVector3d local_p = tVector3d::Zero();
    link->ComputedJkvdot_dq(mqdot, local_p);
    link->ComputedJkwdot_dq(mqdot);
    int total_freedom = link->GetNumTotalFreedoms();
    EIGEN_V_MATXD dJkvdotdq(total_freedom), dJkwdotdq(total_freedom);
    for (int i = 0; i < total_freedom; i++)
    {
        dJkvdotdq[i].noalias() = link->GetdJkvdotdq(i);
        dJkwdotdq[i].noalias() = link->GetdJkwdotdq(i);
    }

    tVectorXd q_old = mq;
    double eps = 1e-7;
    for (int total_i = 0; total_i < total_freedom; total_i++)
    {
        int global_i = link->GetPrevFreedomIds()[total_i];
        q_old[global_i] += eps;
        SetqAndqdot(q_old, mqdot);
        tMatrixXd jvdot_new = link->GetJKvdot_reduced();
        tMatrixXd jwdot_new = link->GetJKwdot_reduced();
        tMatrixXd djvdotdq_num = (jvdot_new - jvdot_old) / eps,
                  djwdotdq_num = (jwdot_new - jwdot_old) / eps;

        tMatrixXd djvdotdq_diff = djvdotdq_num - dJkvdotdq[total_i];
        tMatrixXd djwdotdq_diff = djwdotdq_num - dJkwdotdq[total_i];
        double djvdotdq_diff_norm = djvdotdq_diff.norm();
        double djwdotdq_diff_norm = djwdotdq_diff.norm();
        if (djvdotdq_diff_norm > 1e-5)
        {
            printf("[error] link %d djvdotdq%d diff is too big %.10f\n", id,
                   total_i, djvdotdq_diff_norm);
            exit(0);
        }
        if (djwdotdq_diff_norm > 1e-5)
        {
            printf("[error] link %d djwdotdq%d diff is too big %.10f\n", id,
                   total_i, djwdotdq_diff_norm);
            std::cout << "num = \n" << djwdotdq_num << std::endl;
            std::cout << "ana = \n" << dJkwdotdq[total_i] << std::endl;
            exit(0);
        }
        q_old[global_i] -= eps;
    }
    std::cout << "[log] TestdJdotdq for link " << id << " succ\n";
    PopState("test_link_djdot_dq");
}

// // test JkT * Mck and its derivatives
// void cRobotModelDynamics::TestDCoriolisMatrixDq_part1_1(int link_id)
// {
//     PushState("test_dcdq_part11");
//     auto link = dynamic_cast<Link *>(GetLinkById(link_id));
//     EIGEN_V_MATXD derive;
//     link->ComputedCoriolisMatrixdq_part11(mqdot, derive);
//     tMatrixXd C11_old;
//     link->ComputeCoriolisMatrix_part11(mqdot, C11_old);

//     tVectorXd q_old = mq;
//     double eps = 1e-7;
//     int total_freedom = link->GetNumTotalFreedoms();
//     for (int total_i = 0; total_i < total_freedom; total_i++)
//     {
//         int global_i = link->GetPrevFreedomIds()[total_i];
//         q_old[global_i] += eps;
//         SetqAndqdot(q_old, mqdot);
//         tMatrixXd C11_new;
//         link->ComputeCoriolisMatrix_part11(mqdot, C11_new);
//         tMatrixXd dC11_num = (C11_new - C11_old) / eps;
//         tMatrixXd diff = dC11_num - derive[total_i];
//         double diff_norm = diff.norm();
//         std::cout << "link " << link_id << " dof " << total_i
//                   << " diff norm = \n"
//                   << diff_norm << std::endl;
//         if (diff_norm > 10 * eps)
//         {
//             std::cout << "ana = \n" << derive[total_i] << std::endl;
//             std::cout << "num = \n" << dC11_num << std::endl;
//             std::cout << "error\n";
//             exit(1);
//         }
//         q_old[global_i] -= eps;
//     }
//     PopState("test_dcdq_part11");
// }

void cRobotModelDynamics::TestReducedAPI()
{
    PushState("reduce_api");
    for (int link_id = 0; link_id < GetNumOfLinks(); link_id++)
    {
        PushState("reduce_api_i");
        auto link = dynamic_cast<Link *>(GetLinkById(link_id));
        int total_freedoms = link->GetNumTotalFreedoms();
        // 1. get old value
        tMatrixXd Jv_old = link->GetJKv_reduced();
        tMatrixXd Jw_old = link->GetJKw_reduced();
        EIGEN_V_MATXD dJkvdq(total_freedoms);
        EIGEN_V_MATXD dJkwdq(total_freedoms);
        for (int dof = 0; dof < total_freedoms; dof++)
        {
            dJkvdq[dof] = link->GetdJKvdq_3xnversion(dof);
            dJkwdq[dof] = link->GetdJKwdq_3xnversion(dof);
            // if (link_id == 2 && dof == 3)
            // {
            //     std::cout << "link2 dJvdq3 = " << dJkvdq[dof] << std::endl;
            //     std::cout << "link2 dJwdq3 = " << dJkwdq[dof] << std::endl;
            //     exit(1);
            // }
        }
        // 2. test new value
        tVectorXd q_old = mq;
        double eps = 1e-7;
        for (int dof = 0; dof < total_freedoms; dof++)
        {
            int global_dof = link->GetPrevFreedomIds()[dof];
            q_old[global_dof] += eps;
            SetqAndqdot(q_old, mqdot);
            tMatrixXd Jv_new = link->GetJKv_reduced();
            tMatrixXd Jw_new = link->GetJKw_reduced();
            tMatrixXd dJvdq_num = (Jv_new - Jv_old) / eps;
            tMatrixXd dJwdq_num = (Jw_new - Jw_old) / eps;
            tMatrixXd dJvdq_diff = dJvdq_num - dJkvdq[dof];
            tMatrixXd dJwdq_diff = dJwdq_num - dJkwdq[dof];
            double dJvdq_diff_norm = dJvdq_diff.norm();
            double dJwdq_diff_norm = dJwdq_diff.norm();
            if (dJvdq_diff_norm > 10 * eps)
            {
                std::cout << "[error] Test reduced API for link " << link_id
                          << " dof " << dof
                          << " dJvdq diff norm = " << dJvdq_diff_norm
                          << std::endl;
                std::cout << "ana = \n" << dJkvdq[dof] << std::endl;
                std::cout << "num = \n" << dJvdq_num << std::endl;
                exit(1);
            }
            if (dJwdq_diff_norm > 10 * eps)
            {
                std::cout << "[error] Test reduced API for link " << link_id
                          << " dof " << dof
                          << " dJwdq diff norm = " << dJwdq_diff_norm
                          << std::endl;
                exit(1);
            }
            q_old[global_dof] -= eps;
        }

        // link->GetdJKvdq_3xnversion()

        PopState("reduce_api_i");
    }
    PopState("reduce_api");
}

/**
 * \brief           
*/
void cRobotModelDynamics::TestDCoriolisMatrixDq_global_link(int id)
{
    auto link = dynamic_cast<Link *>(GetLinkById(id));
    EIGEN_V_MATXD dCdq_reduced;
    link->ComputedCoriolisMatrixdqReduced(mqdot, dCdq_reduced);
    tMatrixXd C_old = link->ComputeCoriolisMatrixReduced(mqdot);

    tVectorXd q_old = mq;
    double eps = 1e-7;
    int total_freedom = link->GetNumTotalFreedoms();
    for (int total_i = 0; total_i < total_freedom; total_i++)
    {
        int global_i = link->GetPrevFreedomIds()[total_i];
        q_old[global_i] += eps;
        SetqAndqdot(q_old, mqdot);

        tMatrixXd C_new = link->ComputeCoriolisMatrixReduced(mqdot);
        tMatrixXd dCdq_num = (C_new - C_old) / eps;
        tMatrixXd dCdq_diff = dCdq_num - dCdq_reduced[total_i];
        double norm = dCdq_diff.norm();
        if (norm > 10 * eps)
        {
            std::cout << "[error] TestDCDq global for link " << id << " dof "
                      << total_i << " dCdq diff norm = " << norm << std::endl;
            std::cout << "ana = \n" << dCdq_reduced[total_i] << std::endl;
            std::cout << "num = \n" << dCdq_num << std::endl;
            exit(1);
        }

        q_old[global_i] -= eps;
    }
    std::cout << "[log] Test reduced dCdq for link " << id << " succ\n";
}

/**
 * \brief           Verify the dJdotdqdot
 * 
 * dJdotdqdot = dJdq
*/
void cRobotModelDynamics::TestdJdotdqdot()
{

    for (int link_id = 0; link_id < GetNumOfLinks(); link_id++)
    {
        PushState("test_djdotdqdot");
        auto link = dynamic_cast<Link *>(GetLinkById(link_id));
        int total_freedoms = link->GetNumTotalFreedoms();
        // 1. get getcurrent Jdot, and dJdotdqdot
        tMatrixXd Jkdot_old = link->GetJkdot_recuded();
        EIGEN_V_MATXD dJkdotdqdot_ana(total_freedoms,
                                      tMatrixXd::Zero(6, total_freedoms));
        for (int i = 0; i < total_freedoms; i++)
        {
            dJkdotdqdot_ana[i] = link->GetdJkdotdqdot(i);
        }

        tVectorXd qdot_old = mqdot;
        double eps = 1e-7;
        for (int dof = 0; dof < total_freedoms; dof++)
        {
            int global_dof = link->GetPrevFreedomIds()[dof];
            qdot_old[global_dof] += eps;
            SetqAndqdot(mq, qdot_old);
            tMatrixXd Jkdot_new = link->GetJkdot_recuded();
            tMatrixXd dJkdotdqdot_num = (Jkdot_new - Jkdot_old) / eps;
            tMatrixXd dJkdotdqdot_diff = dJkdotdqdot_num - dJkdotdqdot_ana[dof];
            double norm = dJkdotdqdot_diff.norm();
            if (norm > eps * 10)
            {
                printf("[error] dJkdotdqdot link %d dof %d diff norm %.10f\n",
                       link_id, dof, norm);
                std::cout << "ana = \n" << dJkdotdqdot_ana[dof] << std::endl;
                std::cout << "num = \n" << dJkdotdqdot_num << std::endl;
                exit(1);
            }

            qdot_old[global_dof] -= eps;
        }
        std::cout << "[log] Test dJdotdqdot for link " << link_id << " succ\n";
        PopState("test_djdotdqdot");
    }
}

/**
 * \brief       Test the joint's get & set freedoms' APIs work fine
*/
void cRobotModelDynamics::TestSetFreedomValueAndDot()
{
    PushState("test_state");
    // 1. random a q and qdot, set them
    tVectorXd q = tVectorXd::Random(num_of_freedom),
              qdot = tVectorXd::Random(num_of_freedom);
    SetqAndqdot(q, qdot);
    // 2. check the model's freedoms v and vdot
    double eps = 1e-15;
    for (int i = 0; i < num_of_freedom; i++)
    {
        double v_diff = std::fabs(freedoms[i]->v - q[i]),
               vdot_diff = std::fabs(freedoms[i]->vdot - qdot[i]);
        if (v_diff > eps || vdot_diff > eps)
        {
            printf("[error] TestSetFreedomValueAndDot failed, for dof %d vdiff "
                   "%.5f, vdotdiff %.5f\n",
                   i, v_diff, vdot_diff);
            exit(0);
        }
    }

    // 3. check the v and vdot in each joint
    for (int i = 0; i < GetNumOfJoint(); i++)
    {
        auto joint = dynamic_cast<Joint *>(GetJointById(i));
        int local_dof = joint->GetNumOfFreedom();
        int offset = joint->GetFreedoms(0)->id;
        for (int j = 0; j < local_dof; j++)
        {
            double get_q, get_qdot;
            joint->GetFreedomValue(j, get_q);
            joint->GetFreedomValueDot(j, get_qdot);

            double target_q = q[offset + j], target_qdot = qdot[offset + j];
            double q_diff = std::fabs(get_q - target_q),
                   qdot_diff = std::fabs(get_qdot - target_qdot);
            if (q_diff > eps || qdot_diff > eps)
            {
                printf("[error] TestSetFreedomValueAndDot failed, for dof %d "
                       "qdiff "
                       "%.5f, qdotdiff %.5f\n",
                       i, q_diff, qdot_diff);
                exit(0);
            }
        }
    }

    printf("[log] TestSetFreedomValueAndDot succ\n");
    PopState("test_state");
}

/**
 * \brief           Test the Jkw_local for each joint
 * 
 * Jkw_local is the local jacobian of orientation w.r.t joint's own freedom
 * 
 * w_local = Jkw_local * qdot_local
*/
void cRobotModelDynamics::TestJointLocalJkw()
{
    PushState("test_joint_local_jkw");
    tVectorXd q_old = tVectorXd::Random(num_of_freedom);
    tVectorXd qdot = tVectorXd::Random(num_of_freedom);

    double eps = 1e-3;
    for (int i = 0; i < GetNumOfJoint(); i++)
    {
        auto joint = dynamic_cast<Joint *>(GetJointById(i));
        // 1. get its current orientation R_old, get its current jkw_local, calculate the local angular vel w_local_pred
        SetqAndqdot(q_old, qdot);
        tMatrix3d R_old = joint->GetRotations();
        tMatrixXd Jkw_local = joint->GetLocalJkw();
        tVector3d omega_local_pred =
            Jkw_local *
            qdot.segment(joint->GetOffset(), joint->GetNumOfFreedom());

        // 2. given eps, forward q_old by qdot, set new q
        tVectorXd q_new = q_old + eps * qdot;
        SetqAndqdot(q_new, qdot);
        // 3. get its current orientation R_new, calculate the angular velocity by w_local_true = R_new - R_old
        tMatrix3d R_new = joint->GetRotations();
        tVector omega_local_true =
            btMathUtil::QuaternionToAxisAngle(
                btMathUtil::RotMat3dToQuaternion(R_new * R_old.inverse())) /
            eps;

        // 4. compare, give the result

        tVector diff =
            omega_local_true - btMathUtil::Expand(omega_local_pred, 0);

        if (diff.norm() > eps)
        {
            std::cout << "[error] for joint " << i
                      << " local_omega_pred = " << omega_local_pred.transpose()
                      << " local_omega_true = " << omega_local_true.transpose()
                      << std::endl;
            std::cout << "diff = " << diff.transpose() << std::endl;
            exit(0);
        }
        printf("[log] test joint %d local Jkw succ\n", i);
    }
    printf("[log] test local Jkw succ\n");
    PopState("test_joint_local_jkw");
}