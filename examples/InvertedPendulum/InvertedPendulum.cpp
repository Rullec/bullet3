#include "InvertedPendulum.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
// #include "BulletGenDynamics/btGenController/btGenPDController.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include "Controller/CartPDController.h"
#include "Controller/LQRController.h"
#include <fstream>
#include <iostream>
#include <memory>

extern int global_frame_id;
struct InvertedPendulum : public CommonRigidBodyBase
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    struct tParams
    {
        double mDefaultTimestep;
        tParams(const std::string &path);
    };

    struct tParams *physics_param;
    InvertedPendulum(struct GUIHelperInterface *helper);
    virtual ~InvertedPendulum();

    virtual void stepSimulation(float deltaTime) override final;
    virtual bool keyboardCallback(int key, int state) override;
    virtual void initPhysics() override;
    virtual void exitPhysics() override;
    virtual void renderScene();
    void resetCamera()
    {
        float dist = 3;
        float pitch = -35;
        float yaw = 52;
        float targetPos[3] = {0, 0.46, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1],
                                 targetPos[2]);
    }

protected:
    btGeneralizeWorld *mGenWorld;
    cRobotModelDynamics *mModel;
    cInvertPendulumController *mCtrl;
    bool mEnableCtrl;
    std::map<int, tVector, std::less<int>,
             Eigen::aligned_allocator<std::pair<const int, tVector>>>
        mPerturb;
    double mTimestep;
    void RestrictCart();
    void Initmultibody();
    void TestMultibody();
    void InitController();
    void UpdatePerturb();
    void UpdateCtrl(double dt);
    void ToggleCtrl();
    void RemovePerturb(int key);
    void AddPerturb(int key);
    // btRigidBody* target_rigidbody;
    // float mTime;
};
extern bool gPauseSimulation;
#include "valgrind/callgrind.h"

void InvertedPendulum::stepSimulation(float dt)
{
    int num_substep = 0;
    while (dt > 0)
    {
        double elasped_time = physics_param->mDefaultTimestep;
        if (dt <= elasped_time)
            elasped_time = dt;
        dt -= elasped_time;

        RestrictCart();
        UpdatePerturb();
        UpdateCtrl(elasped_time);
        mGenWorld->StepSimulation(
            static_cast<float>(physics_param->mDefaultTimestep));
        mGenWorld->ClearForce();

        global_frame_id++;
        num_substep++;
    }
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void InvertedPendulum::exitPhysics() {}
void InvertedPendulum::initPhysics()
{
    srand(0);
    physics_param = new tParams("invalid.path");

    m_guiHelper->setUpAxis(1);

    mGenWorld = new btGeneralizeWorld();
    mGenWorld->Init("examples/CustomEngine/sim_config.json");
    // mGenWorld->SetGravity(tVector::Zero());
    // mGenWorld->AddGround(0);

    Initmultibody();
    InitController();
    m_dynamicsWorld = mGenWorld->GetInternalWorld();

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void InvertedPendulum::renderScene() { CommonRigidBodyBase::renderScene(); }

InvertedPendulum::tParams::tParams(const std::string &path)
{
    mDefaultTimestep = 1e-3;
}

CommonExampleInterface *
InvertedPendulumCreateFunc(CommonExampleOptions &options)
{
    return new InvertedPendulum(options.m_guiHelper);
}

InvertedPendulum::~InvertedPendulum()
{
    if (physics_param)
        delete physics_param;
    if (mGenWorld)
        delete mGenWorld;
}

InvertedPendulum::InvertedPendulum(struct GUIHelperInterface *helper)
    : CommonRigidBodyBase(helper)
{
    // mTime = 0;
    physics_param = nullptr;
    mGenWorld = nullptr;
    mPerturb.clear();
    mModel = nullptr;
    mCtrl = nullptr;
    mEnableCtrl = false;
    mTimestep = 0;
}

void InvertedPendulum::RestrictCart()
{
    // 1. make sure that root pos has only z freedom
    auto mb = mGenWorld->GetMultibody();
    int dof = mb->GetNumOfFreedom();
    tVectorXd q = mb->Getq(), qdot = mb->Getqdot();
    auto root_joint = mb->GetRoot();
    int root_dof = root_joint->GetNumOfFreedom();
    double max_z = 20;
    for (int dof = 0; dof < root_dof; dof++)
    {
        auto freedom = root_joint->GetFreedoms(dof);
        if (freedom->type == TRANSLATE &&
            std::fabs(freedom->axis[TRANSLATE_Z] - 1) < 1e-10)
        {
            // if it is the translate z axis
            if (q[dof] > max_z)
                q[dof] = -max_z;
            else if (q[dof] < -max_z)
                q[dof] = max_z;
            // q[dof] = std::min(std::max(q[dof], -max_z), max_z);
        }
    }
    double angle = q[dof - 1], angle_vel = qdot[dof - 1];
    {
        while (angle > M_PI)
            angle -= 2 * M_PI;
        while (angle < -M_PI)
            angle += 2 * M_PI;
        q[dof - 1] = angle;
    }
    // {
    //     while (angle > 2 * M_PI)
    //         angle -= 2 * M_PI;
    //     while (angle < -2 * M_PI)
    //         angle += 2 * M_PI;

    //     // if the lean angle > 45', and the qdot's sin
    //     double dist_angle = std::fabs(angle);
    //     {
    //         if (std::fabs(dist_angle) > M_PI)
    //             dist_angle = 2 * M_PI - dist_angle;
    //     }
    //     if (dist_angle > M_PI / 4)
    //     {
    //         // counter clockwise, take the negative one
    //         if (angle_vel > 0)
    //         {
    //             while (angle > 0)
    //                 angle -= 2 * M_PI;
    //         }

    //         // clockwise: take the positive one
    //         if (angle_vel < 0)
    //         {
    //             while (angle < 0)
    //                 angle += 2 * M_PI;
    //         }
    //     }

    //     q[dof - 1] = angle;
    // }
    mb->SetqAndqdot(q, qdot);
    std::cout << "[bias] phi = " << q[q.size() - 1] << std::endl;
    std::cout << "[bias] phi vel = " << qdot[q.size() - 1] << std::endl;

    // 2. add friction force
    // double mu = 0.1;
    // tVector mg = mModel->GetTotalMass() * mGenWorld->GetGravity();
    // double friction_force = mg.norm() * mu;
    // double root_vel = (mModel->GetLinkById(0)->GetJKv() * mModel->Getqdot())[2];
    // // std::cout << "root vel = " << root_vel << std::endl;
    // double root_vel_dir = root_vel / std::fabs(root_vel);
    // if (std::isnan(root_vel_dir))
    //     root_vel_dir = 0;
    // tVector force = tVector(0, 0, -root_vel_dir * friction_force, 0);
    // // std::cout << "friction = " << force.transpose() << std::endl;
    // mb->ApplyForce(0, force,
    //                btMathUtil::Expand(mb->GetRoot()->GetWorldPos(), 0));
}

void InvertedPendulum::Initmultibody()
{
    // mGenWorld->AddMultibody(
    //     "examples/InvertedPendulum/skeletons/invert_pendulum.json");
    mGenWorld->AddMultibody(
        "examples/InvertedPendulum/skeletons/limit_ip.json");
    // TestMultibody();
    auto mb = mGenWorld->GetMultibody();
    int dof = mb->GetNumOfFreedom();
    auto root = mb->GetRoot();
    tVectorXd q = mb->Getq(), qdot = mb->Getqdot();
    q.setZero();
    qdot.setZero();
    // q[q.size() - 1] = 0.1;
    mb->SetqAndqdot(q, qdot);

    mModel = mb;
}

bool InvertedPendulum::keyboardCallback(int key, int state)
{
    CommonRigidBodyBase::keyboardCallback(key, state);

    switch (key)
    {
    case 'a':
    case 'd':
        if (state == 1)
            AddPerturb(key);
        else
            RemovePerturb(key);
        break;
    case 'c':
        if (state == 1)
            ToggleCtrl();
        break;
    default:
        break;
    }
    std::cout << key << " " << state << std::endl;
    // std::cout << "a is " << int('a') << std::endl;
    // std::cout << "d is " << int('d') << std::endl;
    return true;
}

void InvertedPendulum::UpdatePerturb()
{
    tVector force = tVector::Zero();
    for (auto &iter : mPerturb)
    {
        force += iter.second;
    }

    if (force.norm() > 1e-8)
    {
        // std::cout << "apply perturb force = " << force.transpose() << std::endl;
        auto mb = mGenWorld->GetMultibody();
        mb->ApplyForce(0, force,
                       btMathUtil::Expand(mb->GetRoot()->GetWorldPos(), 0));
    }
}

void InvertedPendulum::RemovePerturb(int key)
{
    if (mPerturb.find(key) == mPerturb.end())
        return;
    else
    {
        mPerturb.erase(mPerturb.find(key));
    }
}
void InvertedPendulum::AddPerturb(int key)
{
    double f = 10;
    tVector force = tVector::Zero();
    if (key == 'a')
        force = tVector(0, 0, -f, 0);
    else if (key == 'd')
        force = tVector(0, 0, f, 0);
    else
        return;

    mPerturb[key] = force;
}

void InvertedPendulum::UpdateCtrl(double dt)
{
    if (mCtrl != nullptr && mEnableCtrl)
    {
        mCtrl->UpdateCtrl(dt);
    }
}

void InvertedPendulum::InitController()
{

    const std::string path =
        "examples/InvertedPendulum/invert_pendulum_pd_config.json";
    Json::Value root;
    btJsonUtil::LoadJson(path, root);
    std::string type = btJsonUtil::ParseAsString("control_type", root);
    if (type == "pd")
    {
        mCtrl = new cCartPDController(mModel);
    }
    else if (type == "lqr")
    {
        std::string lqr_type = btJsonUtil::ParseAsString(
            "lqr_type", btJsonUtil::ParseAsValue("lqr_conf", root));
        mCtrl = new cLQRController(mModel, lqr_type);
    }
    else
    {
        std::cout << "[error] unsupported control type " << type << std::endl;
        exit(0);
    }
}

void InvertedPendulum::ToggleCtrl() { mEnableCtrl = !mEnableCtrl; }

void InvertedPendulum::TestMultibody()
{
    auto mb = mGenWorld->GetMultibody();
    int num_of_freedom = mb->GetNumOfFreedom();
    int num_of_link = mb->GetNumOfLinks();
    int num_of_joint = mb->GetNumOfJoint();
    std::cout << "dof = " << num_of_freedom << " num links = " << num_of_link
              << " num of joint = " << num_of_joint << std::endl;
    for (int i = 0; i < num_of_joint; i++)
    {
        auto joint = mb->GetJointById(i);
        auto link = mb->GetLinkById(i);
        std::cout << "joint name " << joint->GetName()
                  << " id = " << joint->GetId()
                  << " type = " << joint->GetJointType()
                  << " freedom = " << joint->GetNumOfFreedom() << std::endl;

        tMatrixXd jv = link->GetJKv(), jw = link->GetJKw();
        tVector3d world_pos = link->GetWorldPos();
        tMatrix world_trans = link->GetGlobalTransform();

        std::cout << "jv = \n" << jv << std::endl;
        std::cout << "jw = \n" << jw << std::endl;
        std::cout << "world pos = " << world_pos.transpose() << std::endl;
        std::cout << "world trans = \n" << world_trans.transpose() << std::endl;
    }
    exit(0);
}