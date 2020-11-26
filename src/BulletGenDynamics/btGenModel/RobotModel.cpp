#include "RobotModel.h"
#include "BipedalRootJoint.h"
#include "FixedRootJoint.h"
#include "Joint.h"
#include "LimitRootJoint.h"
#include "Link.h"
#include "Printer.h"
#include "RootJoint.h"
#include "Tinyxml/tinyxml.h"
#include "json/json.h"
// #include "R
// #define DAMPING_COEF 0
std::map<std::string, int> cRobotModel::joint_type_map;
std::map<std::string, int> cRobotModel::shape_map;

static const std::string coriolis_info = "coriolis_record.log";
// extern std::string gOutputLogPath;
// cRobotModel::cRobotModel(const char* model_file) : root(nullptr), name(""),
// num_of_freedom(0), end_link(nullptr), com(0, 0, 0), total_mass(0),
// num_of_valid_joint(0), deep_mimic_motion_size(0)
// {
// 	InitJointTypeMap();
// 	InitShapeMap();
// 	LoadBaseMesh();
// 	Load(model_file);
// 	InitModel();
// 	Update(false);
// }

cRobotModel::~cRobotModel()
{
    for (auto itr = joints.begin(); itr != joints.end(); ++itr)
    {
        delete itr->second;
    }

    for (auto itr = links.begin(); itr != links.end(); ++itr)
    {
        delete itr->second;
    }
    joint_id_map.clear();
}

cRobotModel::cRobotModel()
    : root(nullptr), name(""), num_of_freedom(0), end_link(nullptr),
      com(0, 0, 0), total_mass(0), num_of_valid_joint(0),
      deep_mimic_motion_size(0), model_type(-1), scale(1)
{
    compute_second_deriv = false;
    compute_third_deriv = false;
}
void cRobotModel::Init(const char *model_file, double scale, int type)
{
    InitJointTypeMap();
    InitShapeMap();
    LoadBaseMesh();

    char_file = std::string(model_file);
    switch (type)
    {
    case ROM:
        Load(model_file);
        break;
    case ASF:
        LoadAsf(model_file);
        break;
    case JSON:
        LoadJsonModel(model_file, scale);
        break;
    default:
        break;
    }
    InitModel();
    Update();
}

void cRobotModel::PrintLocalTransform()
{
    for (auto &child : children_chain)
    {
        if (!child->IsJoint())
            continue;
        const tMatrix &m = child->GetLocalTransform();
        Printer::Print(m, child->GetName().data());
    }
}

EIGEN_V_tVector3d cRobotModel::GetJointPos()
{
    EIGEN_V_tVector3d pos;
    const size_t n_joints = joint_id_map.size();
    for (size_t i = 0; i < n_joints; ++i)
    {
        auto &j = joint_id_map.at(i);
        pos.push_back(j->GetWorldPos());
    }
    return pos;
}

EIGEN_V_MATXD cRobotModel::GetGlobalTransforms()
{
    EIGEN_V_MATXD ms;
    const size_t n_joints = joint_id_map.size();
    for (int i = 0; i < n_joints; ++i)
    {
        auto &j = joint_id_map.at(i);
        ms.push_back(j->GetGlobalTransform());
    }
    return ms;
}

std::vector<std::string> cRobotModel::GetJointNames()
{
    std::vector<std::string> names;
    const size_t n_joints = joint_id_map.size();
    for (int i = 0; i < n_joints; ++i)
    {
        auto &j = joint_id_map.at(i);
        names.push_back(j->GetName());
    }
    return names;
}

void cRobotModel::ResetPose()
{
    const std::vector<double> zeros(num_of_freedom, 0);
    this->Apply(zeros, 0, true);
}

void cRobotModel::GetAllLinksRotation(tEigenArr<tMatrix> &r)
{
    for (auto &child : children_chain)
    {
        if (child->IsJoint())
            continue;
        tMatrix m = child->GetGlobalTransform();
        r.emplace_back(m);
    }
}

void cRobotModel::GetAllRotation(tEigenArr<tMatrix> &r)
{
    for (auto &child : children_chain)
    {
        if (!child->IsJoint())
            continue;
        tMatrix m = child->GetGlobalTransform();
        r.emplace_back(m);
    }
}

void cRobotModel::GetAllLinksPos(EIGEN_V_tVector3d &r)
{
    for (auto &link : link_chain)
    {
        tVector p = link->GetGlobalTransform() * tVector(0, 0, 0, 1);
        r.push_back(Tools::GettVector3d(p));
    }
}

/*
        @Function: GetAllLinksPosTimesMass
        @params: x Type tMatrixXd &, buffer object
                This function try to calculate (m_k * pos_k).shape=(1, 3) for
   each link k so the resulted tMatrixXd x have a size [N, 3]
*/
void cRobotModel::GetAllLinksPosTimesMass(tMatrixXd &x)
{
    int r = 0;
    for (auto &link : link_chain)
    {
        x.block(r, 0, 1, 3) = link->GetWorldPos().transpose() * link->GetMass();
        r += 1;
    }
}

void cRobotModel::GetAllLinksPosTimesMass(tVector3d &p)
{
    p = tVector3d::Zero();
    for (auto &link : link_chain)
    {
        p += link->GetWorldPos().transpose() * link->GetMass();
    }
}

void cRobotModel::SetComputeSecondDerive(bool flag)
{
    compute_second_deriv = flag;
    for (auto &c : children_chain)
    {
        c->SetComputeSecondDerive(flag);
    }
}

void cRobotModel::SetComputeThirdDerive(bool flag)
{
    compute_third_deriv = flag;
    for (auto &c : children_chain)
    {
        c->SetComputeThirdDerive(flag);
    }
}

bool cRobotModel::GetComputeSecondDerive() { return compute_second_deriv; }
bool cRobotModel::GetComputeThirdDerive() { return compute_third_deriv; }

/**
 * \brief           Set the global transform for fixed root joint
*/
void cRobotModel::SetFixedRootTransform(const tMatrix &trans)
{
    BTGEN_ASSERT(GetRoot()->GetJointType() == JointType::FIXED_NONE_JOINT);
    dynamic_cast<FixedRootJoint *>(GetRoot())->SetFixedRootJointWorldTrans(
        trans);
}

tMatrix cRobotModel::GetFixedRootTransform() const
{
    BTGEN_ASSERT(GetRoot()->GetJointType() == JointType::FIXED_NONE_JOINT);
    return dynamic_cast<FixedRootJoint *>(GetRoot())
        ->GetFixedRootJointWorldTrans();
}
/**
 * \brif						����robot model
 * \param model_file			rom�ļ�·��
 */
void cRobotModel::Load(const char *model_file)
{
    TiXmlDocument doc;
    bool load_ok = doc.LoadFile(model_file);
    if (!load_ok)
    {
        std::cout << model_file << " " << doc.ErrorDesc() << std::endl;
    }

    TiXmlElement *doc_root = doc.RootElement();
    TiXmlElement *world = doc_root->FirstChildElement("world");
    TiXmlElement *robot_model = world->FirstChildElement("model");
    std::string type_name = robot_model->Attribute("type");

    if (type_name.find("global") != std::string::npos)
        model_type = ROM_GLOBAL;
    else
        model_type = ROM_LOCAL;

    name = robot_model->Attribute("name");

    //==================================����link==================================
    for (TiXmlElement *link = robot_model->FirstChildElement("link"); link;
         link = link->NextSiblingElement("link"))
    {
        TiXmlAttribute *link_name = link->FirstAttribute();

        TiXmlElement *link_pose = link->FirstChildElement("pose");
        std::stringstream link_pose_str;
        link_pose_str << link_pose->GetText();

        tVector3d local_coordinate;
        tVector3d local_rotation;

        link_pose_str >> local_coordinate[0] >> local_coordinate[1] >>
            local_coordinate[2];
        link_pose_str >> local_rotation[0] >> local_rotation[1] >>
            local_rotation[2];

        TiXmlElement *inertial = link->FirstChildElement("inertial");
        TiXmlElement *mass = inertial->FirstChildElement("mass");
        TiXmlElement *pose_of_mass = inertial->FirstChildElement("pose");
        TiXmlElement *inertia = inertial->FirstChildElement("inertia");
        std::stringstream ss_v_of_mass;
        double value_of_mass;

        ss_v_of_mass << mass->GetText();
        ss_v_of_mass >> value_of_mass;

        std::stringstream center_of_mass_str;
        center_of_mass_str << pose_of_mass->GetText();
        tVector3d center_of_mass;
        center_of_mass_str >> center_of_mass[0] >> center_of_mass[1] >>
            center_of_mass[2];

        double ixx = 0, ixy = 0, ixz = 0;
        double iyy = 0, iyz = 0;
        double izz = 0;

        std::stringstream ss_i;
        ss_i << inertia->FirstChildElement("ixx")->GetText();
        ss_i >> ixx;
        ss_i.clear();
        ss_i << inertia->FirstChildElement("ixy")->GetText();
        ss_i >> ixy;
        ss_i.clear();
        ss_i << inertia->FirstChildElement("ixz")->GetText();
        ss_i >> ixz;
        ss_i.clear();
        ss_i << inertia->FirstChildElement("iyy")->GetText();
        ss_i >> iyy;
        ss_i.clear();
        ss_i << inertia->FirstChildElement("iyz")->GetText();
        ss_i >> iyz;
        ss_i.clear();
        ss_i << inertia->FirstChildElement("izz")->GetText();
        ss_i >> izz;
        ss_i.clear();

        tMatrix3d I;
        I << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;

        TiXmlElement *visual = link->FirstChildElement("visual");
        TiXmlElement *visual_pose = visual->FirstChildElement("pose");
        TiXmlElement *visual_geometry_mesh =
            visual->FirstChildElement("geometry")->FirstChildElement("mesh");
        TiXmlElement *mesh_uri = visual_geometry_mesh->FirstChildElement("uri");
        TiXmlElement *mesh_scale =
            visual_geometry_mesh->FirstChildElement("scale");
        TiXmlElement *mesh_shape = visual->FirstChildElement("shape");

        std::stringstream visual_pose_str;
        visual_pose_str << visual_pose->GetText();
        tVector3f mesh_translation, mesh_rotation_vector;
        visual_pose_str >> mesh_translation[0] >> mesh_translation[1] >>
            mesh_translation[2];
        visual_pose_str >> mesh_rotation_vector[0] >> mesh_rotation_vector[1] >>
            mesh_rotation_vector[2];

        std::stringstream mesh_scale_str;
        mesh_scale_str << mesh_scale->GetText();
        tVector3f mesh_scale_vector;
        mesh_scale_str >> mesh_scale_vector[0] >> mesh_scale_vector[1] >>
            mesh_scale_vector[2];
        AddLink(link_name->Value(), local_coordinate, local_rotation,
                mesh_uri->GetText(), mesh_rotation_vector, mesh_scale_vector,
                value_of_mass);
        const tVector3d &local_pos = links[link_name->Value()]->GetLocalPos();
        links[link_name->Value()]->SetPos(local_pos);
        links[link_name->Value()]->SetInertiaTensorBody(I);

        if (mesh_shape != nullptr)
        {
            std::stringstream ss_shape;
            ss_shape << mesh_shape->GetText();
            auto itr = shape_map.find(ss_shape.str());
            if (itr != shape_map.end())
            {
                BaseObjectShapeParam param;
                param.shape_type = itr->second;
                param.top_scale = 1;
                param.bottom_scale = 1;
                links[link_name->Value()]->UpdateShape(param);
            }
        }
    }
    //============================================================================
    AddRootJoint("root", JointType::NONE_JOINT, 0);

    //==================================����joint=================================
    for (TiXmlElement *joint = robot_model->FirstChildElement("joint"); joint;
         joint = joint->NextSiblingElement("joint"))
    {
        TiXmlAttribute *joint_name = joint->FirstAttribute();
        TiXmlAttribute *joint_type = joint_name->Next();

        TiXmlElement *joint_pose = joint->FirstChildElement("pose");
        std::stringstream joint_pose_str;
        joint_pose_str << joint_pose->GetText();

        tVector3d joint_pose_coordinate;
        tVector3d joint_pose_rotation;

        joint_pose_str >> joint_pose_coordinate[0] >>
            joint_pose_coordinate[1] >> joint_pose_coordinate[2];
        joint_pose_str >> joint_pose_rotation[0] >> joint_pose_rotation[1] >>
            joint_pose_rotation[2];

        TiXmlElement *father = joint->FirstChildElement("parent");
        TiXmlElement *child = joint->FirstChildElement("child");
        AddJoint(joint_name->Value(), father->GetText(), child->GetText(),
                 joint_pose_coordinate, joint_pose_rotation);
        LoadFreedom(joint_name->Value(), joint, joint_pose_rotation);
    }
    //============================================================================

    for (auto itr = joints.begin(); itr != joints.end(); ++itr)
    {
        if (itr->second->GetName() == "Root")
            continue;
        InsertFreedomMap(itr->second);
    }
    std::string str("Load Robot Model: ");
    str.append(model_file);
    Printer::ScreenLog(str.c_str());
}

/**
 * \brif						���Ӷ�ȡ joint ���ɶ���Ϣ
 * \param doc_joint			xml�ĵ��е�jointָ��
 */
void cRobotModel::LoadFreedom(const char *name, TiXmlElement *doc_joint,
                              tVector3d &joint_pose)
{
    BaseObject *joint;
    auto itr = joints.find(name);
    if (itr != joints.end())
    {
        joint = itr->second;
    }
    else
    {
        std::cout << "error when add freedom to joint: " << name << std::endl;
        return;
    }
    for (auto *axis = doc_joint->FirstChildElement("axis"); axis != nullptr;
         axis = axis->NextSiblingElement("axis"))
    {
        TiXmlElement *xyz = axis->FirstChildElement("xyz");
        TiXmlElement *limit = axis->FirstChildElement("limit");
        TiXmlElement *lower = limit->FirstChildElement("lower");
        TiXmlElement *upper = limit->FirstChildElement("upper");
        std::stringstream xyz_str, lb_ss, ub_ss;
        xyz_str << xyz->GetText();
        Freedom f;
        xyz_str >> f.axis[0] >> f.axis[1] >> f.axis[2];
        lb_ss << lower->GetText();
        ub_ss << upper->GetText();
        lb_ss >> f.lb;
        ub_ss >> f.ub;

        f.id = num_of_freedom++;
        f.v = f.axis.dot(joint_pose);
        f.name = name;
        joint->AddFreedom(f);
    }
}

/**
 * \brief					�� joint �е� freedom ���ӵ� robot
 *�� freedom_map �� �����?? robot ��ֱ�ӻ�ȡ freedom �����м��� \param joint ������
 *freedom �� freedom_map �� joint
 */
void cRobotModel::InsertFreedomMap(BaseObject *joint)
{
    for (int i = 0; i < joint->GetNumOfFreedom(); i++)
    {
        Freedom *p = joint->GetFreedoms(i);
        BTGEN_ASSERT(p->id == freedoms.size());
        freedoms.insert(std::make_pair(p->id, p));
    }
}

/**
 * \brif						����Root Joint
 */
void cRobotModel::AddRootJoint(const char *root_name, JointType root_joint_type,
                               double torque_lim)
{
    BaseObjectParams param;
    // param.mesh = BaseRender::mesh_map.find("ball")->second;
    // assert(param.mesh != nullptr);

    param.name = "root";
    param.local_pos = tVector3d(0, 0, 0);
    param.local_rotation = tVector3d(0, 0, 0);
    param.mesh_rotation = tVector3f(0, 0, 0);
    param.mesh_scale = tVector3f(0.2f, 0.2f, 0.2f);
    param.mass = 0;
    if (root == nullptr)
    {
        if (root_joint_type == JointType::NONE_JOINT)
        {
            root = new RootJoint(param, num_of_freedom);
        }
        else if (root_joint_type == JointType::LIMIT_NONE_JOINT)
        {
            root = new LimitRootJoint(param, num_of_freedom);
        }
        else if (root_joint_type == JointType::BIPEDAL_NONE_JOINT)
        {
            root = new BipedalRootJoint(param, num_of_freedom);
        }
        else if (root_joint_type == JointType::FIXED_NONE_JOINT)
        {
            root = new FixedRootJoint(param, num_of_freedom);
        }
        else
        {
            std::cout << "[error] unsupported root joint type "
                      << root_joint_type << std::endl;
            exit(0);
        }
    }
    if (false == links.empty())
    {
        auto *p = links.begin()->second;
        while (p->GetParent())
            p = p->GetParent();
        root->AddChild(p);
        root->SetNumTotalFreedoms(root->GetNumOfFreedom());
        p->SetParent(root);
    }
    dynamic_cast<Joint *>(root)->SetTorqueLim(torque_lim);
    // std::cout << "root torque lim = " << torque_lim << std::endl;
    InsertFreedomMap(root);
    num_of_freedom += root->GetNumOfFreedom();
    joints.insert(std::make_pair(param.name, root));
}

/**
 * \brief					����Root Link
 *							ͨ�� rom �ļ�
 *��ȡ�Ļ�����ģ�ͣ�����Ҫ�����������?? ͨ�� asf �ļ� ��ȡ�Ļ�����ģ�ͣ���Ҫ�����������??
 * \param root_name			Root Link ������
 */
void cRobotModel::AddRootLink(const char *root_name)
{
    BaseObjectParams param;
    // param.mesh = BaseRender::mesh_map.find("ball")->second;
    // param.mesh = BaseRender::mesh_map.find("box")->second;
    // assert(param.mesh != nullptr);

    param.name = root_name;
    param.local_pos = tVector3d(0, 0, 0);
    param.local_rotation = tVector3d(0, 0, 0);
    param.mesh_scale = tVector3f(0.0f, 0.0f, 0.f);
    param.mesh_rotation = tVector3f(0, 0, 0);
    param.mass = 1; // todo fix here

    assert(root);
    BaseObject *link = new Link(param);
    root->ClearChildren();
    root->AddChild(link);
    link->SetParent(root);
    links.insert({"root", link});
}

/**
 * \brif						���� Link
 * \param link_name			link���ƣ���link������
 * \param local_pos			link�����ھֲ�����ϵ�µ�λ�ã��ֲ�����ϵԭ����
 * parent joint ������ \param local_rotation		link_mesh ����ת \param
 * mesh_path
 * link mesh ��·�� \param mesh_rotation		mesh ����ת \param mesh_scale
 * mesh �����ű��� \param mass				link ����
 */
void cRobotModel::AddLink(std::string link_name, tVector3d &local_pos,
                          tVector3d &local_rotation, const char *mesh_path,
                          tVector3f &mesh_rotation, tVector3f &mesh_scale,
                          double mass)
{
    // Mesh* mesh = nullptr;
    // if (mesh_path == "")
    // {
    // 	mesh = BaseRender::mesh_map["link"];
    // }
    // else
    // {
    // 	auto itr = BaseRender::mesh_map.find(std::string(mesh_path));
    // 	if (itr == BaseRender::mesh_map.end())
    // 	{
    // 		mesh = new Mesh(mesh_path);
    // 		BaseRender::mesh_map.insert(std::make_pair(mesh_path, mesh));
    // 	}
    // 	else
    // 	{
    // 		mesh = itr->second;
    // 	}
    // }
    BaseObjectParams param;
    param.name = link_name;
    param.local_pos = local_pos;
    param.local_rotation = local_rotation;
    // param.mesh = mesh;
    param.mesh_scale = mesh_scale;
    param.mesh_rotation = mesh_rotation;
    param.mass = mass;
    BaseObject *link = new Link(param);
    links.insert(std::make_pair(link_name, link));
}

/**
 * \brif						���� Joint
 * \param joint_name			joint���ƣ���joint������
 * \param local_pos			joint�����ھֲ�����ϵ�µ�λ�ã��ֲ�����ϵԭ����
 * parent link ������ \param local_rotation		joint�ھֲ�����ϵ�µ���ת
 * \param mesh_scale			mesh �����ű���
 */
void cRobotModel::AddJoint(std::string joint_name, std::string father_name,
                           std::string child_name, const tVector3d &local_pos,
                           const tVector3d &local_rotation)
{
    BaseObjectParams param;
    // param.mesh = BaseRender::mesh_map.find("ball")->second;
    // assert(param.mesh != nullptr);

    param.mesh_scale = tVector3f(0.8f, 0.8f, 0.8f);
    param.name = joint_name;
    param.local_pos = local_pos;
    param.local_rotation = local_rotation;
    param.mesh_rotation = local_rotation.cast<float>();
    param.mass = 0;
    BaseObject *joint = new Joint(param);
    joints.insert(std::make_pair(joint_name, joint));

    if (!father_name.empty())
    {
        auto it = links.find(father_name);
        if (it != links.end())
        {
            joint->SetParent(it->second);
            it->second->AddChild(joint);
        }
        else
        {
            std::cout << "cannot find parent link: " << father_name
                      << std::endl;
        }
    }

    auto it = links.find(child_name);
    if (it != links.end())
    {
        joint->AddChild(it->second);
        it->second->SetParent(joint);
    }
    else
    {
        std::cout << "cannot find child link: " << child_name << std::endl;
    }
}

void cRobotModel::AddSphericalFreedoms(BaseObject *joint,
                                       const tVector3d &lower_limit,
                                       const tVector3d &upper_limit)
{
    for (int freedom_order = 0; freedom_order < 3; ++freedom_order)
    {
        Freedom f;
        f.id = 0;
        f.name = joint->GetName();
        f.type = REVOLUTE;
        f.axis = tVector3d(0, 0, 0);
        f.axis[freedom_order] = 1;

        f.v = 0;
        f.lb = lower_limit[freedom_order];
        f.ub = upper_limit[freedom_order];

        joint->AddFreedom(f);
    }
}

void cRobotModel::AddRevoluteJointFreedoms(BaseObject *joint, double lb,
                                           double ub)
{
    Freedom f;
    f.id = 0;
    f.name = joint->GetName();
    f.type = REVOLUTE;
    f.axis = tVector3d(1, 0, 0);

    f.v = 0;
    f.lb = lb;
    f.ub = ub;

    joint->AddFreedom(f);
}

void cRobotModel::UpdateFreedomId()
{
    int freedom_order = root->GetNumOfFreedom();

    for (int id = 1; id <= joint_id_map.size(); ++id)
    {
        auto *joint = joint_id_map[id];
        for (int f = 0; f < joint->GetNumOfFreedom(); ++f)
        {
            auto *freedom = joint->GetFreedoms(f);
            freedom->id = f + freedom_order;
        }
        InsertFreedomMap(joint);
        freedom_order += joint->GetNumOfFreedom();
    }
    num_of_freedom = freedom_order;
}

/**
 * \brif						���� ball mesh, ������ joint,
 *target point ������ ���� joint ����ͬһ�� mesh�� ����һ��link mesh
 *ͨ�����źͱ任����ʾ����link
 */
void cRobotModel::LoadBaseMesh()
{
    // auto* ball_mesh = new Mesh("../Data/objs/ball.obj", "../Data/objs/");
    // mesh_map.insert(std::make_pair("ball", ball_mesh));

    // auto* link_mesh = new Mesh("../Data/objs/NewLink.obj", "../Data/objs/");
    // mesh_map.insert(std::make_pair("link", link_mesh));

    // auto* box_mesh = new Mesh("../Data/objs/cube.obj", "../Data/objs/");
    // mesh_map.insert(std::make_pair("box", box_mesh));
}

/**
 * \brief					��ȡ asf ��ʽ�Ļ�����
 * \param file_path			�ļ�·��
 */
void cRobotModel::LoadAsf(const char *file_path)
{
    std::string path = file_path;
    path = path.substr(path.find_last_of('/') + 1);
    name = path.substr(0, path.find_first_of('.'));
    std::ifstream fin;
    fin.open(file_path, std::ios::in);
    if (!fin.is_open())
    {
        std::string s("Opening ");
        s.append(file_path);
        s.append(" error");
        Printer::Error(s.c_str());
        return;
    }
    std::string s;

    // move fin pointer to bonedata
    while (getline(fin, s))
    {
        s.erase(0, s.find_first_not_of(' '));
        if (s[0] == '#')
            continue;
        if (s.find(":bonedata") != std::string::npos)
            break;
    }
    std::string joint("_joint");
    int freedom_id = 6;
    // read bone data
    bool done = false;
    while (!done)
    {
        int id;
        tVector3d link_world_vec_from_parent;
        tVector3d link_local_pos;
        tVector3d link_local_rotation(0, 0, 0);
        tVector3d joint_local_pos(0, 0, 0);
        tVector3d joint_local_rotation(0, 0, 0);
        tVector3d mesh_rotation(0, 0, 0);
        tVector3f mesh_rotation_f;
        tVector3f mesh_scale(0, 1, 0);
        double length = 1;
        std::string name, joint_name;
        std::vector<Freedom> fs;
        double mass = 1;
        while (getline(fin, s))
        {
            s.erase(0, s.find_first_not_of(' '));
            if (s[0] == '#')
                continue;
            std::string reg = " ";
            std::vector<std::string> ss = Tools::split(s, reg);
            if (ss[0].find("end") != std::string::npos)
                break;
            if (ss[0].find("hie") != std::string::npos)
            {
                done = true;
                break;
            }
            if (ss[0].find("id") != std::string::npos)
            {
                id = static_cast<int>(atof(ss[1].data()));
            }
            if (ss[0].find("name") != std::string::npos)
            {
                name = ss[1];
                joint_name = name + joint;
            }
            else if (ss[0].find("direction") != std::string::npos)
            {
                link_world_vec_from_parent =
                    tVector3d(atof(ss[1].c_str()), atof(ss[2].c_str()),
                              atof(ss[3].c_str())) /
                    2;
            }
            else if (ss[0].find("length") != std::string::npos)
            {
                length = Tools::ConvertUnit(atof(ss[1].c_str()));
                mesh_scale *= length / 0.1;
                mesh_scale(0) = 1.4f;
                mesh_scale(2) = 1.4f;
                mass *= Tools::ConvertUnit(atof(ss[1].c_str()));
            }
            else if (ss[0].find("axis") != std::string::npos)
            {
                joint_local_rotation =
                    tVector3d(Tools::DegToRad(atof(ss[1].c_str())),
                              Tools::DegToRad(atof(ss[2].c_str())),
                              Tools::DegToRad(atof(ss[3].c_str())));
            }
            else if (ss[0].find("dof") != std::string::npos)
            {
                for (size_t i = 1; i < ss.size(); i++)
                {
                    Freedom f;
                    if (ss[i].find("x") != std::string::npos)
                    {
                        f.axis = tVector3d::UnitX();
                    }
                    else if (ss[i].find("y") != std::string::npos)
                    {
                        f.axis = tVector3d::UnitY();
                    }
                    else if (ss[i].find("z") != std::string::npos)
                    {
                        f.axis = tVector3d::UnitZ();
                    }
                    f.id = freedom_id++;
                    f.name = joint_name;
                    f.type = REVOLUTE;
                    f.v = 0;
                    f.lb = -3.14; // todo fix this
                    f.ub = 3.14;
                    fs.push_back(f);
                }
            }
            else if (ss[0].find("limits") != std::string::npos)
            {
                ss[1].erase(ss[1].begin());
                ss[2].erase(ss[2].end() - 1);
                fs[0].lb = Tools::DegToRad(std::atof(ss[1].data()));
                fs[0].ub = Tools::DegToRad(std::atof(ss[2].data()));
                int bound_i = 1;
                while (bound_i < fs.size())
                {
                    getline(fin, s);
                    s.erase(0, s.find_first_not_of(' '));
                    if (s[0] == '#')
                        continue;
                    std::string reg = " ";
                    std::vector<std::string> ss = Tools::split(s, reg);
                    ss[0].erase(0, 1);
                    ss[1].erase(ss[1].end() - 1);
                    fs[bound_i].lb = Tools::DegToRad(std::atof(ss[0].data()));
                    fs[bound_i].ub = Tools::DegToRad(std::atof(ss[1].data()));
                    bound_i++;
                }
            }
        }
        if (!done)
        {
            tMatrix3d m;
            m.setIdentity();
            m = zconventionRotation(-joint_local_rotation(2)) * m;
            m = yconventionRotation(-joint_local_rotation(1)) * m;
            m = xconventionRotation(-joint_local_rotation(0)) * m;

            link_local_pos = m * link_world_vec_from_parent;
            GetMeshRotation(mesh_rotation, link_local_pos);
            link_local_pos *= length;

            mesh_rotation_f = mesh_rotation.cast<float>();
            AddLink(name, link_local_pos, link_local_rotation, "",
                    mesh_rotation_f, mesh_scale, mass);
            AddJoint(joint_name, "", name, joint_local_pos,
                     joint_local_rotation);
            for (auto &f : fs)
            {
                joints[joint_name]->AddFreedom(f);
            }
            switch (fs.size())
            {
            case 6:
                joints[joint_name]->SetJointType(NONE_JOINT);
                break;
            case 3:
                joints[joint_name]->SetJointType(SPHERICAL_JOINT);
                break;
            case 1:
                joints[joint_name]->SetJointType(REVOLUTE_JOINT);
                break;
            case 0:
                joints[joint_name]->SetJointType(FIXED_JOINT);
                break;
            default:
                Printer::Error("illegal joint type");
                break;
            }
            InsertFreedomMap(joints[joint_name]);
            fs.clear();
            joint_id_map.insert(std::make_pair(id, joints[joint_name]));
        }
    }

    // add root
    AddRootJoint("root", JointType::NONE_JOINT, 0);
    AddRootLink("root");
    num_of_freedom = static_cast<int>(freedoms.size());
    joint_id_map.insert(std::make_pair(0, root));

    getline(fin, s);
    if (s.find("begin") != std::string::npos)
    {
        while (getline(fin, s))
        {
            s.erase(0, s.find_first_not_of(' '));
            if (s[0] == '#')
                continue;
            if (s.find("end") != std::string::npos)
                break;
            std::string reg(" ");
            std::vector<std::string> ss = Tools::split(s, reg);
            auto *link = links[ss[0]];
            for (size_t i = 1; i < ss.size(); i++)
            {
                link->AddChild(links[ss[i]]->GetParent());
                links[ss[i]]->GetParent()->SetParent(link);
                links[ss[i]]->GetParent()->SetLocalPos(link->GetLocalPos());
            }
        }
    }
    fin.close();
    char log[1024];
    sprintf(log, "Load Asf : %s", file_path);
    Printer::ScreenLog(log);
}

void cRobotModel::ExportAsf(const char *file_path)
{
    std::fstream fout;
    fout.open(file_path, std::ios::out);
    if (!fout.is_open())
    {
        std::string s("Opening ");
        s.append(file_path);
        s.append(" error");
        Printer::Error(s.c_str());
        return;
    }

    fout << ":bonedata\n";

    for (auto *p : joint_chain)
    {
        if (p->GetParent() == nullptr)
            continue;

        auto *c = p->GetFirstChild();
        tVector3d global_dir = 2 * (c->GetWorldPos() - p->GetWorldPos());

        tVector3d axis = p->GetGlobalTransform()
                             .topLeftCorner<3, 3>()
                             .eulerAngles(2, 1, 0)
                             .reverse();
        double length = Tools::ConvertUnitInv(sqrt(global_dir.dot(global_dir)));
        global_dir.normalize();

        fout << "  begin\n";
        fout << "    id " << p->GetId() << "\n";
        fout << "    name " << p->GetName() << "\n";
        fout << "    direction " << global_dir[0] << " " << global_dir[1] << " "
             << global_dir[2] << "\n";
        fout << "    axis " << Tools::RadToDeg(axis[0]) << " "
             << Tools::RadToDeg(axis[1]) << " " << Tools::RadToDeg(axis[2])
             << "\n";
        fout << "    length " << length << "\n";
        fout << "    dof";
        for (int i = 0; i < p->GetNumOfFreedom(); ++i)
        {
            auto *f = p->GetFreedoms(i);
            if (int(f->axis[0]) != 0)
                fout << " rx";
            if (int(f->axis[1]) != 0)
                fout << " ry";
            if (int(f->axis[2]) != 0)
                fout << " rz";
        }
        fout << "\n";
        // fout << f->GetLimitsString();
        fout << "  end\n";
    }

    fout << ":hierarchy\n  begin\n";
    for (auto *p : joint_chain)
    {
        fout << "    " << p->GetName();
        for (int i = 0; i < p->GetNumOfChildren(); ++i)
        {
            if (p->GetChild(i)->GetFirstChild())
                fout << " " << p->GetChild(i)->GetFirstChild()->GetName();
        }
        fout << "\n";
    }
    fout << "  end";
}

/**
 * \brief					load json model (DeepMimic json
 * file) \param file_path			file path
 */
void cRobotModel::LoadJsonModel(const char *file_path, double model_scale)
{
    // std::ofstream fout(coriolis_info);
    // fout << "";
    // fout.close();

    // cFileUtil::ClearFile(model_info);
    scale = model_scale;
    Json::Value json_root;
    Json::CharReaderBuilder builder;
    std::ifstream fin(file_path, std::ifstream::binary);
    std::string errs;
    if (!fin.is_open())
    {
        char log[1024];
        sprintf(log, "Can not open file: %s", file_path);
        Printer::Error(log);
        return;
    }
    bool ok = Json::parseFromStream(builder, fin, &json_root, &errs);
    if (!ok)
    {
        std::cout << "error reading json";
        exit(-1);
    }
    auto joint_node = json_root["Skeleton"]["Joints"];
    auto mesh_node = json_root["DrawShapeDefs"];
    auto bodydefs_node = json_root["BodyDefs"];

    // load joint
    double root_diff_weight = 0;
    JointType root_joint_type = JointType::INVALID_JOINT;
    double root_torque_lim = 0;
    for (auto itr = joint_node.begin(); itr != joint_node.end(); ++itr)
    {
        BaseObjectJsonParam param;
        param.name = (*itr)["Name"].asString() + "_joint";
        // std::cout << "add " << param.name << std::endl;
        param.id = (*itr)["ID"].asInt();
        param.parent_id = (*itr)["Parent"].asInt();

        // param.mesh = BaseRender::mesh_map["ball"];
        param.mesh_scale = tVector3f(0.8f, 0.8f, 0.8f);
        param.mesh_rot = tVector3f(0.f, 0.f, 0.f);
        param.type = JOINT;
        param.inertia = tMatrix3d::Identity();

        tVector3d local_pos;
        tVector3d local_rot;
        local_pos.x() = (*itr)["AttachX"].asDouble() * model_scale;
        local_pos.y() = (*itr)["AttachY"].asDouble() * model_scale;
        local_pos.z() = (*itr)["AttachZ"].asDouble() * model_scale;
        local_rot.x() = (*itr)["AttachThetaX"].asDouble();
        local_rot.y() = (*itr)["AttachThetaY"].asDouble();
        local_rot.z() = (*itr)["AttachThetaZ"].asDouble();

        param.local_pos = local_pos;
        param.local_rot = local_rot;
        double torque_lim = (*itr)["TorqueLim"].asDouble();
        double diff_weight = (*itr)["DiffWeight"].asDouble();

        std::string type_str = (*itr)["Type"].asString();
        if (joint_type_map.find(type_str) == joint_type_map.end())
        {
            printf("[error] no type called %s\n", type_str.c_str());
            exit(0);
        }
        int joint_type = joint_type_map[type_str];

        if (param.name == "root_joint")
        {
            root_diff_weight = diff_weight;
            root_joint_type = static_cast<JointType>(joint_type);
            root_torque_lim = torque_lim;
            continue;
        }

        BaseObject *joint = new Joint(param);
        joint_id_map.insert({param.id, joint});
        joints.insert({param.name, joint});
        joint->SetJointType(static_cast<JointType>(joint_type));
        static_cast<Joint *>(joint)->SetTorqueLim(torque_lim);
        static_cast<Joint *>(joint)->SetDiffWeight(diff_weight);
        if (joint_type == SPHERICAL_JOINT)
        {
            tVector3d lower_limit, upper_limit;
            for (int i = 0; i < 3; i++)
            {
                lower_limit[i] =
                    (*itr)["LimLow" + std::to_string(i)].asDouble();
                upper_limit[i] =
                    (*itr)["LimHigh" + std::to_string(i)].asDouble();
            }
            AddSphericalFreedoms(joint, lower_limit, upper_limit);
        }
        else if (joint_type == REVOLUTE_JOINT)
        {
            double lower_limit = (*itr)["LimLow0"].asDouble(),
                   upper_limit = (*itr)["LimHigh0"].asDouble();
            AddRevoluteJointFreedoms(joint, lower_limit, upper_limit);
        }
    }
    // std::cout << "before add root joint, joint num = " << joint_node.size()
    // << std::endl;
    AddRootJoint("root", root_joint_type, root_torque_lim);
    AddRootLink("root");
    UpdateFreedomId();
    joint_id_map.insert({root->GetId(), root});
    link_id_map.insert({0, links["root"]});

    // set the diffweight for root again
    static_cast<Joint *>(GetRoot())->SetDiffWeight(root_diff_weight);

    // load link
    for (auto itr = bodydefs_node.begin(); itr != bodydefs_node.end(); ++itr)
    {
        BaseObjectJsonParam param;
        param.name = (*itr)["Name"].asString();
        param.id = (*itr)["ID"].asInt();
        param.parent_id =
            param.id; // link's parent is joint which has the same id
        // param.mesh = BaseRender::mesh_map["box"];
        param.mass = (*itr)["Mass"].asDouble();

        tVector3d local_pos;
        tVector3f mesh_rot;
        int col_group;
        local_pos.x() = (*itr)["AttachX"].asDouble() * model_scale;
        local_pos.y() = (*itr)["AttachY"].asDouble() * model_scale;
        local_pos.z() = (*itr)["AttachZ"].asDouble() * model_scale;
        mesh_rot.x() = (*itr)["AttachThetaX"].asFloat();
        mesh_rot.y() = (*itr)["AttachThetaY"].asFloat();
        mesh_rot.z() = (*itr)["AttachThetaZ"].asFloat();
        col_group = (*itr)["ColGroup"].asInt();
        std::string shape = (*itr)["Shape"].asString();
        auto shape_itr = shape_map.find(shape);
        if (shape_itr != shape_map.end())
        {
            param.shape_type = shape_itr->second;
        }
        else
        {
            Printer::Error("shape type error");
            exit(-1);
        }

        float width = (*itr)["Param0"].asFloat() * model_scale;
        float length = (*itr)["Param1"].asFloat() * model_scale;
        float height = (*itr)["Param2"].asFloat() * model_scale;
        param.mesh_scale = tVector3f(width, length, height);

        param.local_pos = local_pos;
        param.local_rot = tVector3d(0, 0, 0);
        param.mesh_rot = mesh_rot;
        param.type = LINK;
        param.inertia = tMatrix3d::Zero();

        // we already have a root link
        // here we do not need to create a new one
        // just modify the existing one
        if (param.name == "root")
        {
            links["root"]->UpdateInfo(param);
            continue;
        }
        BaseObject *link = new Link(param);
        static_cast<Link *>(link)->SetColGroup(col_group);
        links.insert({link->GetName(), link});
        link_id_map.insert({param.id, link});
    }

    // read all info of mesh
    // there could be some difference between bodydef and its mesh
    // so we have to read these info again
    for (auto itr = mesh_node.begin(); itr != mesh_node.end(); ++itr)
    {
        std::string name = (*itr)["Name"].asString();
        BaseObject *base_object = nullptr;
        int parent_joint_id = (*itr)["ParentJoint"].asInt();
        if (name.find("_joint") != std::string::npos)
        { // for joint info
            auto joint_itr = joints.find(name);
            if (joint_itr != joints.end())
            {
                base_object = joint_itr->second;
            }
            else
                continue;
        }
        else
        {
            auto link_itr = links.find(name); // for link info
            if (link_itr != links.end())
            {
                base_object = link_itr->second;
            }
        }
        int shape_type = shape_map[(*itr)["Shape"].asString()];

        BaseObjectShapeParam shape_param;
        shape_param.top_scale = (*itr)["TopScale"].asDouble();
        shape_param.bottom_scale = (*itr)["BottomScale"].asDouble();
        shape_param.shape_type = shape_type;
        shape_param.width = (*itr)["Param0"].asDouble() * model_scale;
        shape_param.length = (*itr)["Param2"].asDouble() * model_scale;
        shape_param.height = (*itr)["Param1"].asDouble() * model_scale;
        if (base_object == nullptr)
        {
            Printer::ScreenLog(
                "RobotModel::LoadJsonModel base object access illegal",
                name.c_str());
            exit(0);
        }
        base_object->UpdateShape(shape_param);
    }

    // convert joint local pos
    // in json file, joint local pos is defined in parent joint frame
    // we need to convert it to parent link frame
    for (auto &itr : joint_id_map)
    {
        try
        {
            if (itr.second->GetId() != 0)
            {
                auto *parent_joint = joint_id_map[itr.second->GetParentId()];
                std::string parent_joint_name = parent_joint->GetName();
                std::string link_name = parent_joint_name.substr(
                    0, parent_joint_name.find_first_of("_"));
                auto *link_on_parent_joint = links[link_name];
                if (parent_joint->GetNumOfChildren() == 0)
                    parent_joint->AddChild(link_on_parent_joint);
                link_on_parent_joint->SetParent(parent_joint);
                link_on_parent_joint->AddChild(itr.second);
                itr.second->SetParent(link_on_parent_joint);

                link_on_parent_joint->SetParentJoint(parent_joint);
                itr.second->SetParentJoint(parent_joint);

                const tMatrix &m1 = link_on_parent_joint->GetLocalTransform();
                const tMatrix &m2 =
                    itr.second->GetLocalTransform(); // ��ʱ joint �� local_pos
                // �������?? parent_joint
                tMatrix m = m1.inverse() * m2;
                tVector3d joint_local_pos;
                joint_local_pos[0] = m.data()[12];
                joint_local_pos[1] = m.data()[13];
                joint_local_pos[2] = m.data()[14];
                tVector3d joint_local_rot =
                    m.topLeftCorner<3, 3>().eulerAngles(2, 1, 0).reverse();

                itr.second->SetLocalPos(
                    joint_local_pos); //  ��ʱ joint_local_pos �������??
                                      //  parent_link ��
                itr.second->SetLocalRot(joint_local_rot);
            }
        }
        catch (const char *msg)
        {
            Printer::Error(msg);
        }
    }

    for (auto &itr : links)
    {
        if (itr.second->GetParent() == nullptr)
        {
            joints[itr.second->GetName() + "_joint"]->AddChild(itr.second);
            itr.second->SetParent(joints[itr.second->GetName() + "_joint"]);
        }
    }

    // std::cout << "final joint num = " << GetNumOfJoint() << std::endl;
}

void cRobotModel::GetMeshRotation(tVector3d &mesh_rotation,
                                  tVector3d &direction) const
{
    tQuaternion q = Eigen::Quaternion<double>::FromTwoVectors(
        tVector3d::UnitY(), direction);
    mesh_rotation = q.toRotationMatrix().eulerAngles(0, 1, 2);
}

void cRobotModel::InitJointTypeMap()
{
    if (joint_type_map.empty())
    {
        for (int i = 0; i < TOTAL_JOINT_TYPE; ++i)
        {
            joint_type_map.insert(std::make_pair(joint_type_keys[i], i));
        }
    }
}

void cRobotModel::InitShapeMap()
{
    if (shape_map.empty())
    {
        for (int i = 0; i < TOTAL_SHAPE_TYPE; ++i)
        {
            shape_map.insert(std::make_pair(shape_type_keys[i], i));
        }
    }
}

// btRigidBody* createRigidBody(float mass, const btTransform& startTransform,
// btCollisionShape* shape, const std::string& name, const btVector4& color)
// {
// 	{
// 		btAssert((!shape || shape->getShapeType() !=
// INVALID_SHAPE_PROXYTYPE));

// 		//rigidbody is dynamic if and only if mass is non zero,
// otherwise static 		bool isDynamic = (mass != 0.f);

// 		btVector3 localInertia(0, 0, 0);
// 		if (isDynamic)
// 			shape->calculateLocalInertia(mass, localInertia);

// 		btRigidBody* body = new btRigidBody(mass, 0, shape,
// localInertia); 		body->setWorldTransform(startTransform);
// 		body->setUserIndex(-1);
// 		world->addRigidBody(body);
// 		return body;
// 	}
// }

/**
 * \brief					�����������ɶ�ֵ
 * \param ang				�µ����ɶ�ֵ
 * \param using_joint		�Ƿ�ͨ�� joint_map ������ freedom��
 *							ͨ�� joint_map ��
 *freedom_map ������Ч����һ���ġ� \param compute_gradient	�Ƿ���㵼��??
 */
void cRobotModel::Apply(const std::vector<double> &ang, int st,
                        bool compute_gradient)
{
    for (int i = 0; i < num_of_freedom; i++)
    {
        auto fre = freedoms[i];
        fre->v = ang[st + i];
    }

    Update(compute_gradient);
}

/**
 * \brief					�����������ɶ�ֵ
 * \param ang				�µ����ɶȵ�ֵ
 * \param cycle_step		һ��cycle���˶�����
 * \param st				ang�е���ʼλ��
 * \param compute_gradient	�Ƿ���㵼��??
 */
void cRobotModel::Apply(const std::vector<double> &ang, tVector3d cycle_step,
                        int st, bool compute_gradient)
{
    for (int i = 0; i < num_of_freedom; i++)
    {
        if (freedoms[i]->type == TRANSLATE)
        {
            freedoms[i]->v = ang[st + i] + cycle_step(i);
        }
        else
            freedoms[i]->v = ang[st + i];
    }
    Update(compute_gradient);
}

/**
 * \brief					�����������ɶ�ֵ
 * \param ang				�µ����ɶȵ�ֵ Type const vec &
 * \param compute_gradient	�Ƿ���㵼��??
 */
void cRobotModel::Apply(const tVectorXd &ang,
                        bool compute_gradient /* = false*/)
{
    std::vector<double> ang_vec(0);
    for (size_t i = 0; i < ang.size(); i++)
        ang_vec.push_back(ang[i]);
    BTGEN_ASSERT(ang_vec.size() == num_of_freedom);
    Apply(ang_vec, 0, compute_gradient);
}

/**
 * \brief					��ֵ����
 */
void cRobotModel::ComputeGradientNumerically()
{
    // end_link = links["lfoot"];
    auto *end_joint = joints["lfoot_joint"];
    double h = 1e-5;
    EIGEN_V_tVector3d grad;
    for (int order = 0; order < num_of_freedom; order++)
    {
        ans[order] += h;
        Apply(ans, 0, false);
        tVector3d p1 = end_joint->GetWorldPos();
        ans[order] -= h;
        Apply(ans, 0, false);
        tVector3d p2 = end_joint->GetWorldPos();
        tVector3d g_n = (p1 - p2) / h;
        grad.push_back(g_n);
    }
    Printer::Log(grad, "grad_num", "../Logs/Grad/grad_num_070601.txt");
}

/**
 * \brief					ʹ�ü��η������㵼����
 *							g = axis.cross(end_eff_pos
 *- joint_pos)
 */
void cRobotModel::ComputeGradientByGeometry()
{
    end_link = links["Link_3"];

    auto itr = joints.begin();
    while (itr != joints.end())
    {
        auto *p = itr->second;
        p->CleanGradient();
        ++itr;
    }

    tMatrix m = tMatrix::Identity();
    for (auto *p : children_chain)
    {
        p->ComputeJacobian(end_link->GetWorldPos(), m, 1);
    }

    EIGEN_V_tVector3d grads;
    for (int i = 0; i < num_of_freedom; i++)
    {
        grads.push_back(freedoms[i]->grad);
    }
    // Printer::Log(grads, "grad_geo", "../Logs/Grad/grad_geo_061201.txt");
}

/**
 * \brief					�˶�����
 */
void cRobotModel::ComputeGradientAnalytically()
{
    auto *end_link = links["lfoot"];
    auto *end_joint = joints["lfoot_joint"];

    tMatrix tsfm2, tsfm3;
    end_link->GetLocalTransform(tsfm2);

    const tVector3d &link_local_pos = end_link->GetLocalPos();

    tsfm3.setIdentity();
    tsfm3.data()[12] = link_local_pos[0];
    tsfm3.data()[13] = link_local_pos[1];
    tsfm3.data()[14] = link_local_pos[2];

    tVector p(0, 0, 0, 1);
    // p = tsfm2 * tsfm3 * p;
    tMatrixXd j;
    end_joint->ComputeJacobiByGivenPoint(j, p);
    std::vector<int> freedom_id;
    for (auto &freedom : freedoms)
    {
        freedom_id.push_back(freedom.second->id);
    }
    // Printer::Log(j, "../Logs/Grad/grad_ana_070601.txt");
}

/**
//  * \brief					������ĳ��target
point��ȫ���������ɶȵĵ���
//  * \param tp				�����target point
//  * \param j					����õ���Jacobian ����
//  */
// void RobotModel::ComputeJacobiByTargetPoint(TargetPoint* tp, tMatrixXd& j)
// {
// 	auto* target_link = links[tp->GetLinkName()];
// 	auto* target_joint = target_link->GetParent();
// 	const tMatrix& m1 = target_link->GetLocalTransform();
// 	tMatrix m2;
// 	m2.setIdentity();
// 	m2.data()[12] = tp->GetLocalPos()[0];
// 	m2.data()[13] = tp->GetLocalPos()[1];
// 	m2.data()[14] = tp->GetLocalPos()[2];

// 	tVector p = m1 * m2 * tVector(0, 0, 0, 1);
// 	target_joint->ComputeJacobiByGivenPoint(j, p);
// }

/**
 * \brief					���ݵ�target
 * point��ȫ���������ɶȵĵ���
 * \param name				target point �� link name
 * \param point				����������link�ľֲ�����ϵ�µ�λ��
 * \param j					����õ���Jacobian ����
 */
void cRobotModel::ComputeJacobiByGivenPoint(std::string name,
                                            const tVector3d &point,
                                            tMatrixXd &j) const
{
    auto itr = links.find(name);
    if (itr == links.end())
    {
        std::cout << "[ERROR] There is no link: " << name << std::endl;
    }
    auto *target_link = itr->second;
    auto *target_joint = target_link->GetParent();
    const tMatrix &m1 = target_link->GetLocalTransform();
    tMatrix m2;
    m2.setIdentity();
    m2.data()[12] = point[0];
    m2.data()[13] = point[1];
    m2.data()[14] = point[2];

    // tVector p = m1 * m2 * tVector(0, 0, 0, 1);
    tVector p = m1 * Tools::GettVector(point);
    target_joint->ComputeJacobiByGivenPoint(j, p);
}

/**
 * \brief					Calculate Jacobian for this point with
 * respect to the whole DOF of this skeleton
 * \param	link_id			the attached link id of this point
 * \param	point			point position in world frame
 * \param	j				the reference of jacobian, it will be REVISED in this function
 */
void cRobotModel::ComputeJacobiByGivenPointTotalDOFWorldFrame(
    int link_id, const tVector3d &point_world, tMatrixXd &j) const
{
    auto *target_link = GetLinkById(link_id);
    auto *target_joint = target_link->GetParent();

    tVector3d rt = point_world - target_link->GetWorldPos();
    tVector p =
        target_link->GetLocalTransform() *
        btMathUtil::Expand(
            target_link->GetGlobalTransform().block(0, 0, 3, 3).transpose() *
                rt,
            1);
    target_joint->ComputeJacobiByGivenPointTotalDOF(j, p);
    // auto* target_link = GetLinkById(link_id);
    // auto* target_joint = target_link->GetParent();
    // tVector3d rt = point_world - target_link->GetWorldPos();
    // tVector3d p = target_link->GetLocalTransform().block(0, 0, 3, 3) *
    // target_link->GetGlobalTransform().block(0, 0, 3, 3).transpose() * rt;
    // target_joint->ComputeJacobiByGivenPointTotalDOF(j, cMathUtil::Expand(p,
    // 1));
}

/**
 * \brief					Calculate Jacobian for this point with
 * respect to the whole DOF of this skeleton
 * \param	link_id			the attached link id of this point
 * \param	point			point position in local frame of the LINK, NOT JOINT
 * \param	j				the reference of jacobian, it will be REVISED in this function
 */
void cRobotModel::ComputeJacobiByGivenPointTotalDOFLinkLocalFrame(
    int link_id, const tVector3d &point, tMatrixXd &j) const
{
    ComputeJacobiByGivenPointTotalDOFWorldFrame(
        link_id,
        (GetLinkById(link_id)->GetGlobalTransform() *
         btMathUtil::Expand(point, 1))
            .segment(0, 3),
        j);
}
// /**
//  * \param point			point in link local frame
// */
// void cRobotModel::ComputeJacobiByGivenPointTotalDOFLocalFrame(int link_id,
// const tVector3d& point, tMatrixXd& j) const
// {
// 	auto* target_link = GetLinkById(link_id);
// 	auto* target_joint = target_link->GetParent();
// 	const tMatrix& m1 = target_link->GetLocalTransform();

// 	tVector p = m1 * Tools::GettVector(point);
// 	target_joint->ComputeJacobiByGivenPointTotalDOF(j, p);
// }
/**
 * \brief					�������Ķ�ȫ�����ɶȵĵ���
 * \param j					����õ���Jacobian ����
 */
void cRobotModel::ComputeCoMJacobi(tMatrixXd &j)
{
    tVector3d p(0, 0, 0);
    j.resize(3, num_of_freedom);
    j.setZero();
    for (auto &link : links)
    {
        tMatrixXd jac;
        ComputeJacobiByGivenPoint(link.first, p, jac);
        auto &ids = link.second->GetParent()->GetPrevFreedomIds();
        int mass = link.second->GetMass();
        jac = jac * mass;
        for (size_t i = 0; i < ids.size(); i++)
        {
            j(ids[i] * 3) += jac(i * 3);
            j(ids[i] * 3 + 1) += jac(i * 3 + 1);
            j(ids[i] * 3 + 2) += jac(i * 3 + 2);
        }
    }
    j /= total_mass;
}

/**
 * \brief					�������Ķ�ȫ�����ɶȵ���ֵ����
 */
void cRobotModel::ComputeComJacobiNumerically()
{
    double h = 1e-5;
    EIGEN_V_tVector3d grad;
    for (int order = 0; order < num_of_freedom; order++)
    {
        ans[order] += h;
        Apply(ans, 0, false);
        tVector3d p1 = com;
        ans[order] -= h;
        Apply(ans, 0, false);
        tVector3d p2 = com;
        tVector3d g_n = (p1 - p2) / h;
        grad.push_back(g_n);
    }
    // Printer::Log(grad, "grad_num",
    // "../Logs/Grad/robot/com_jac_num_062401.txt");
}

/**
 * \brief
 *�����ٶȵ���ֵ����������ȷ��������׵�������ȷ��??
 *							deprecated
 */
void cRobotModel::ComputeSecondDeriveNumerically()
{
    auto *joint = joints["joint_1"];
    const double h = 1e-5;
    tVector p(0.1, 0.1, 0.1, 1);
    tMatrixXd j1, j2;
    Apply(ans, 0, true);
    joint->ComputeJacobiByGivenPoint(j1, p);
    int total_freedoms = joint->GetNumTotalFreedoms();
    EIGEN_V_MATXD grads(3, tMatrixXd::Zero(joint->GetNumTotalFreedoms(),
                                           joint->GetNumTotalFreedoms()));
    const auto &prev_freedoms = joint->GetPrevFreedomIds();
    for (size_t i = 0; i < prev_freedoms.size(); ++i)
    {
        for (size_t j = 0; j < prev_freedoms.size(); ++j)
        {
            ans[prev_freedoms[j]] += h;
            Apply(ans, 0, true);
            joint->ComputeJacobiByGivenPoint(j2, p);
            tVector3d grad = (j2.col(i) - j1.col(i)) / h;
            for (int k = 0; k < grad.size(); ++k)
            {
                grads[k].data()[i * total_freedoms + j] = grad[k];
            }
            ans[prev_freedoms[j]] -= h;
        }
    }
    Printer::Log(grads, "../Logs/Grad/0116/011601.txt");
}

void cRobotModel::ComputeSecondDeriveNumerically(tVectorXd &q_dot, int frame)
{
    auto *joint = end_link; // ->GetParent();
    const double h = 1e-5;
    int st_freedom = frame * num_of_freedom;
    Apply(ans, st_freedom, true);
    tMatrixXd jk_o = joint->GetJK();
    tMatrixXd jk_diff = tMatrixXd::Zero(jk_o.rows(), jk_o.cols());

    EIGEN_V_MATXD res;
    for (int i = 0; i < num_of_freedom; ++i)
    {
        ans[st_freedom + i] += h;

        Apply(ans, st_freedom, true);
        const tMatrixXd &jk = joint->GetJK();
        jk_diff += (jk - jk_o) / h * q_dot[i];

        ans[st_freedom + i] -= h;
    }
    Apply(ans, st_freedom, true);
    res.push_back(jk_diff);
    Printer::Log(res, "../Logs/Grad/0518/051801_num.txt");
}

void cRobotModel::ComputeSecondDerive(tVectorXd &q_dot)
{
    tVector3d p(0, 0, 0);

    for (auto &c : children_chain)
    {
        if (!c->IsJoint())
        {
            c->ComputeJKv_dot(q_dot, p);
            c->ComputeJKw_dot(q_dot);
            c->ComputeJK_dot();
        }
    }

    //============ for test ============
    // auto* end_joint = end_link;// ->GetParent();
    const tMatrixXd &jk_dot = GetEndLink()->GetJKDot();
    // tMatrixXd jk_dot = tMatrixXd::Zero(6, num_of_freedom);
    // for(int i = 0; i < num_of_freedom; ++i) {
    //	jk_dot.block(0, 0, 3, num_of_freedom) += end_link->GetJKv_dq(i) *
    // q_dot[i]; 	jk_dot.block(3, 0, 3, num_of_freedom) +=
    // end_link->GetJKw_dq(i)
    //* q_dot[i];
    //}

    EIGEN_V_MATXD res;
    res.push_back(jk_dot);
    Printer::Log(res, "../Logs/Grad/0518/051801.txt");
    //==================================
}

void cRobotModel::ComputeMassTimesLinkJvSum(int frame, EIGEN_V_MATXD &data)
{
    data[frame] = tMatrixXd::Zero(3, num_of_freedom);
    // both joints and links are in this 'children_chain' one by one
    // so we need to judge whether the iterated c is link or not.5
    for (auto &c : children_chain)
    {
        if (c->IsJoint())
            continue;

        c->ComputeJKv();
        data[frame] += c->GetMass() * c->GetJKv();
    }
}

void cRobotModel::ComputeMassTimesLinkJvSumNumerically(int frame,
                                                       EIGEN_V_MATXD &data)
{
    data[frame] = tMatrixXd::Zero(3, num_of_freedom);
    for (auto &c : children_chain)
    {
        if (c->IsJoint())
            continue;
        tMatrixXd m(3, num_of_freedom);
        ComputeLinkJvNumerically(c, frame, m);
        data[frame] += c->GetMass() * m;
    }
}

void cRobotModel::ComputeLinkJvNumerically()
{
    tVector p(0, 0, 0, 1);
    double h = 1e-4;
    EIGEN_V_tVector3d grad;
    BaseObject *end_link = nullptr;
    for (auto &c : link_chain)
        if (c->GetNumOfChildren() == 0)
            end_link = c;
    if (end_link == nullptr)
        return;
    for (int order = 0; order < num_of_freedom; ++order)
    {
        ans[order] += h;
        Apply(ans, 0, false);
        tVector3d p1 = end_link->GetWorldPos();
        ans[order] -= h;
        Apply(ans, 0, false);
        tVector3d p2 = end_link->GetWorldPos();
        tVector3d g_n = (p1 - p2) / h;
        grad.push_back(g_n);
    }
    Printer::Log(grad, "num", "../Logs/Grad/0324/032401.txt");
}

void cRobotModel::ComputeLinkJvNumerically(BaseObject *end_link, int frame,
                                           tMatrixXd &grad)
{
    if (end_link == nullptr)
        return;

    tVector p(0, 0, 0, 1);
    double h = 1e-4;
    int st_freedom = frame * num_of_freedom;
    for (int order = 0; order < num_of_freedom; ++order)
    {
        ans[st_freedom + order] += h;
        Apply(ans, st_freedom, false);
        tVector3d p1 = end_link->GetWorldPos();
        ans[st_freedom + order] -= h;
        Apply(ans, st_freedom, false);
        tVector3d p2 = end_link->GetWorldPos();
        tVector3d g_n = (p1 - p2) / h;
        grad.col(order) = g_n;
    }
}

void cRobotModel::ComputeJv()
{
    int st_col = 0;
    for (auto &c : children_chain)
    {
        if (c->IsJoint())
            continue;
        Jv.block(0, st_col, num_of_freedom, 3) = c->GetJKv().transpose();
        st_col += 3;
    }
}

/**
 * \brief					��ʼ��ģ�ͣ���ʼ��ÿһ��link��joint
 */
void cRobotModel::InitModel()
{
    InitMatrix();
    InitChildrenChain();
    InitLocalPos();
    for (auto *p : children_chain)
    {
        p->SetGlobalFreedoms(num_of_freedom);
        p->InitTerms();
    }
    for (auto itr : joints)
    {
        if (itr.second->GetNumOfFreedom() != 0)
        {
            num_of_valid_joint++;
        }
    }
    InitDofidToJointMap();
    InitDeepMimicMotionSize();
    UpdateMass();
}

/**
 * \brief           Init the whole chilren chain, root joint -> root link -> child joint -> child link
 */
void cRobotModel::InitChildrenChain()
{
    // stack: push & pop
    std::stack<BaseObject *> temp_stack;
    temp_stack.push(root);

    // the parent of links are joints
    // the parent of joints are links
    // all freedoms are over joints
    while (!temp_stack.empty())
    {
        auto *p = temp_stack.top();
        temp_stack.pop();
        children_chain.push_back(p);
        if (p->IsJoint())
            joint_chain.push_back(p);
        if (!p->IsJoint())
            link_chain.push_back(p);
        for (int i = 0; i < p->GetNumOfChildren(); i++)
        {
            temp_stack.push(p->GetChild(i));
        }
        if (p->GetNumOfChildren() == 0)
            end_link = p;
    }
}

/**
 * \brief				init the map from dof id to joint id
 */
void cRobotModel::InitDofidToJointMap()
{
    dof_joint_map.resize(GetNumOfFreedom(), -1);
    for (auto &joint : joints)
    {
        const auto &cur_joint = joint.second;
        for (int i = 0; i < cur_joint->GetNumOfFreedom(); i++)
        {
            dof_joint_map[cur_joint->GetFreedoms(i)->id] = cur_joint->GetId();
        }
    }
}

void cRobotModel::InitLocalPos()
{
    if (model_type != ROM_GLOBAL)
        return;
    for (auto *p : children_chain)
    {
        if (p == root)
            continue;
        tVector3d local_pos;
        if (p->IsJoint())
        {
            const tVector3d &pos_in_child = p->GetLocalPos();
            const tVector3d &parent_pos = p->GetParent()->GetWorldPos();
            const tVector3d &child_pos = p->GetFirstChild()->GetWorldPos();
            const tVector3d &world_pos = child_pos + pos_in_child;
            p->SetPos(world_pos);
            local_pos = world_pos - parent_pos;
        }
        else
        {
            const tVector3d &parent_pos = p->GetParent()->GetWorldPos();
            const tVector3d &pos = p->GetWorldPos();
            local_pos = pos - parent_pos;
            if (p->GetParent()->GetParent() == nullptr)
            {
                p->GetParent()->SetPos(pos);
                p->GetParent()->SetLocalPos(pos);
                local_pos = tVector3d::Zero(); // Ҫ��root�ƶ��� root link ��λ��
                p->SetLocalPos(local_pos); // root link �ֲ�λ��Ϊ0��
            }
        }
        p->SetLocalPos(local_pos);
    }
}

void cRobotModel::InitDeepMimicMotionSize()
{
    int size = 1;
    for (const auto itr : joints)
    {
        switch (itr.second->GetJointType())
        {
        case NONE_JOINT:
            size += 7;
            break;
        case SPHERICAL_JOINT:
            size += 4;
            break;
        case REVOLUTE_JOINT:
            size += 1;
            break;
        case FIXED_JOINT:
            break;
        default:
            break;
        }
    }
    deep_mimic_motion_size = size;
}

void cRobotModel::InitMatrix()
{
    Jv = tMatrixXd::Zero(num_of_freedom, 3 * joints.size());
    Jw = tMatrixXd::Zero(num_of_freedom, 3 * joints.size());
    mass_matrix = tMatrixXd::Zero(num_of_freedom, num_of_freedom);
    coriolis_matrix = tMatrixXd::Zero(num_of_freedom, num_of_freedom);
}

/**
 * \brief           Update the joint & links chain
 */
void cRobotModel::Update(bool compute_gradient)
{
    // for the whole children chain (joint-link), update the rotation & translation
    for (auto *p : children_chain)
    {
        p->UpdateState(compute_gradient);
        p->UpdateMeshMatrix();
    }
    UpdateCoM();
    if (compute_gradient)
    {
        ComputeJw();
        ComputeJv();
        ComputeMassMatrix();
        ComputeCoriolisMatrix(mqdot);
    }
}

/**
 * \brief					Update the COM position of robot
 */
void cRobotModel::UpdateCoM()
{
    com.setZero();
    for (auto *p : children_chain)
    {
        double mass = p->GetMass();
        const tVector3d &pos = p->GetWorldPos();
        com += mass * pos;
    }
    com /= total_mass;
}

/**
 * \brief                   Update the COM vel of robot
*/
void cRobotModel::UpdateCoMVel()
{
    com_vel.setZero();

    for (auto &link : link_chain)
    {
        tVector3d vel = link->GetJKv() * mqdot;
        com_vel += link->GetMass() * vel;
    }
    com_vel /= total_mass;
}

void cRobotModel::UpdateMass()
{
    total_mass = 0;
    for (auto *p : children_chain)
    {
        if (p->IsJoint())
            continue;
        total_mass += p->GetMass();
    }
}

// todo implement this function
void cRobotModel::SaveModel(const char *file)
{
    TiXmlDocument doc;
    auto *decl = new TiXmlDeclaration("1.0", "", "");
    auto *ans_value_element = new TiXmlElement("rom");
    ans_value_element->SetAttribute("version", "0.1");

    TiXmlElement *doc_root = doc.RootElement();
    TiXmlElement *world = new TiXmlElement("world");
    TiXmlElement *robot_model = new TiXmlElement("model");
    robot_model->SetAttribute("name", name.c_str());

    doc.LinkEndChild(decl);
    doc.LinkEndChild(ans_value_element);
    doc.SaveFile(file);
}

// /**
//  * \brief					��ĳ�� target point ��λ��
//  * \param tp				�����?? target point
//  * \param pos				���صĽ��??
//  */
// void RobotModel::GetTargetPosition(TargetPoint& tp, tVector3d& pos)
// {
// 	auto itr = links.find(tp.GetLinkName());
// 	if (itr == links.end())
// 	{
// 		std::cout << "There is no link " << tp.GetLinkName() <<
// std::endl; 		return;
// 	}
// 	auto* l = itr->second;
// 	const tVector& p = l->GetGlobalTransform() *
// tVector(tp.GetLocalPos()[0], tp.GetLocalPos()[1], tp.GetLocalPos()[2], 1);
// 	pos[0] = p[0];
// 	pos[1] = p[1];
// 	pos[2] = p[2];
// }

// void RobotModel::GetTargetPosition(TargetPoint* tp, tVector3d& pos)
// {
// 	GetTargetPosition(*tp, pos);
// }

// tVector3d RobotModel::GetTargetPosition(TargetPoint* tp)
// {
// 	tVector3d pos;
// 	GetTargetPosition(*tp, pos);
// 	return pos;
// }

// void RobotModel::GetCoMPosition(CoMPoint* tp, tVector3d& pos)
// {
// 	pos = com;
// }

tVector3d cRobotModel::GetObjectPos(std::string name, int type)
{
    if (type == LINK)
    {
        auto itr = links.find(name);
        if (itr != links.end())
        {
            return itr->second->GetWorldPos();
        }
    }
    else if (type == JOINT)
    {
        auto itr = joints.find(name);
        if (itr != joints.end())
        {
            return itr->second->GetWorldPos();
        }
    }
    return tVector3d(0, 0, 0);
}

void cRobotModel::GetObjectGradient(std::string name, int type, tMatrixXd jac)
{
    BaseObject *p;

    if (type == LINK)
    {
        auto itr = links.find(name);
        if (itr != links.end())
        {
            p = itr->second;
        }
        else
            return;
    }
    else if (type == JOINT)
    {
        auto itr = joints.find(name);
        if (itr != joints.end())
        {
            p = itr->second;
        }
        else
            return;
    }
    else
        return;
    assert(p);
    tVector point(0, 0, 0, 1);
    p->ComputeJacobiByGivenPoint(jac, point);
}

std::map<int, Freedom *> &cRobotModel::GetFreedoms() { return freedoms; }

const std::string &cRobotModel::GetName() const { return name; }

BaseObject *cRobotModel::GetRoot() const { return root; }

BaseObject *cRobotModel::GetJoint(std::string joint_name) const
{
    auto itr = joints.find(joint_name);
    if (itr != joints.end())
        return itr->second;
    return nullptr;
}

BaseObject *cRobotModel::GetLink(std::string name) const
{
    auto itr = links.find(name);
    if (itr != links.end())
        return itr->second;
    return nullptr;
}

int cRobotModel::GetNumOfFreedom() const { return num_of_freedom; }

/**
 * \brief			Get the lower & upper bound for each joint
 * \param lb
 * \param ub
 */
void cRobotModel::GetJointLimit(tVectorXd &lb, tVectorXd &ub) const
{
    lb.resize(num_of_freedom);
    ub.resize(num_of_freedom);

    for (int i = 0; i < num_of_freedom; i++)
    {
        const auto &dof = freedoms.find(i)->second;
        lb[i] = dof->lb;
        ub[i] = dof->ub;
    }
}

/**
 * \brief			Get the direction for the targeted dof
 */
tVector3d cRobotModel::GetFreedomDirectionWorldFrame(int dof_id) const
{
    // auto [] = CheckAxis(const tVector3d& axis)->int
    // {
    // 	int target = -1;
    // 	for (int i = 0; i < 3; i++)
    // 	{
    // 		if (std::fabs(axis[i] - 1.0) < 1e-10) return i;
    // 	}
    // 	return target;
    // };

    Joint *cur_joint = dynamic_cast<Joint *>(GetJointByDofId(dof_id));
    const tMatrix3d &joint_orientation =
        cur_joint->GetGlobalTransform().topLeftCorner<3, 3>();
    for (int i = 0; i < cur_joint->GetNumOfFreedom(); i++)
    {
        Freedom *freedom = cur_joint->GetFreedoms(i);
        if (freedom->id == dof_id)
        {
            return joint_orientation * freedom->axis;
        }
    }
    std::cout << "[error] cRobotModel::GetFreedomDirectionWorldFrame for dof "
              << dof_id << std::endl;
    exit(0);
    return tVector3d::Zero();
}

void cRobotModel::Travel() const
{
    for (auto *p : children_chain)
    {
        p->Tell();
    }
}

BaseObject *cRobotModel::GetLinkById(int id) const
{
    return GetBaseObjectById(id, LINK);
}

BaseObject *cRobotModel::GetJointById(int id) const
{
    return GetBaseObjectById(id, JOINT);
}

BaseObject *cRobotModel::GetJointByDofId(int dof_id) const
{
    return GetJointById(dof_joint_map[dof_id]);
}

BaseObject *cRobotModel::GetBaseObjectById(int id, int type) const
{
    const std::map<int, BaseObject *> *object_map = nullptr;
    if (type == LINK)
    {
        object_map = &(this->link_id_map);
    }
    else if (type == JOINT)
    {
        object_map = &(this->joint_id_map);
    }
    else
        return nullptr;

    const auto itr = object_map->find(id);
    if (itr != object_map->end())
        return itr->second;
    else
        return nullptr;
}

void cRobotModel::ComputeJw()
{
    Jw.setZero(num_of_freedom, 3 * joints.size());
    int st_col = 0;
    for (auto &child : children_chain)
    {
        if (child->IsJoint())
            continue;
        child->ComputeJKw();
        const tMatrixXd &jkw = child->GetJKw();
        Jw.block(0, st_col, num_of_freedom, 3) = jkw.transpose();
        st_col += 3;
    }
}

void cRobotModel::ComputeAngularVelocity(tMatrixXd &omega, tVectorXd &q_dot)
{
    omega = Jw.transpose() * q_dot;
}

void cRobotModel::ComputeMassMatrix()
{
    mass_matrix.setZero();
    // std::cout << "------begin to calc mass mat------\n";
    for (auto &link : link_chain)
    {
        link->ComputeMassMatrix();
        const tMatrixXd &mass_cartesian = link->GetMassMatrix();
        // std::cout << "mass cartesian = \n"
        //   << mass_cartesian << std::endl;
        const tMatrixXd &jk = link->GetJK();
        mass_matrix.noalias() += jk.transpose() * mass_cartesian * jk;

        // std::cout << "J = \n"
        // 		  << jk << std::endl;
    }
    inv_mass_matrix.noalias() = mass_matrix.inverse();
    // std::cout << "------end to calc mass mat------\n";
}

void cRobotModel::ComputeCoriolisMatrix(tVectorXd &q_dot)
{
    // std::ofstream fout(coriolis_info, std::ios::app);
    // std::cout << "qdot = " << qdot.transpose() << std::endl;
    BTGEN_ASSERT(q_dot.size() == num_of_freedom);
    coriolis_matrix.setZero();

    tVector3d p(0, 0, 0);
    tMatrixXd w_skew(6, 6);
    w_skew.setZero();
    for (auto &link : link_chain)
    {
        // fout << "---------link " << link->GetName()
        // 	 << " begin to calc coriolis matrix---------\n";
        // fout << "q = " << q.transpose() << std::endl;
        // fout << "qdot = " << qdot.transpose() << std::endl;
        link->ComputeJKw_dot(q_dot);
        link->ComputeJKv_dot(q_dot, p);
        link->ComputeJK_dot();

        const tMatrixXd &mass_cartesian = link->GetMassMatrix();
        const tMatrixXd &jk = link->GetJK();
        const tMatrixXd &jk_dot = link->GetJKDot();
        const tMatrixXd &jkw = link->GetJKw();

        tVector3d omega = jkw * q_dot;
        tMatrix3d omega_skew;
        Tools::SkewMatrix(omega, omega_skew);

        w_skew.block(3, 3, 3, 3) = omega_skew;

        // fout << "Jv = \n"
        // 	 << jk.block(0, 0, 3, jk.cols()) << std::endl;
        // fout << "Jvdot = \n"
        // 	 << jk_dot.block(0, 0, 3, jk_dot.cols()) << std::endl;
        // fout << "Jw = \n"
        // 	 << jk.block(3, 0, 3, jk.cols()) << std::endl;
        // fout << "Jwdot = \n"
        // 	 << jk_dot.block(3, 0, 3, jk_dot.cols()) << std::endl;
        // fout << "w = " << cMathUtil::SkewMatToVector(omega_skew).transpose()
        // << std::endl;

        // tMatrixXd res1 = jk.transpose() * mass_cartesian * jk_dot +
        // 				 jk.transpose() * w_skew * mass_cartesian
        // * jk; tMatrixXd res2 = ; tMatrixXd diff = res1 - res2; std::cout <<
        // "diff norm = " << diff.norm() << std::endl;
        coriolis_matrix.noalias() +=
            jk.transpose() *
            (mass_cartesian * jk_dot + w_skew * mass_cartesian * jk);

        // coriolis_matrix.noalias() += ;

        // fout << "---------link " << link->GetName()
        // 	 << " end to calc coriolis matrix---------\n";
        if (compute_third_deriv == true)
        {
            link->ComputedJkvdot_dq(q_dot, tVector3d::Zero());
            link->ComputedJkwdot_dq(q_dot);
        }
    }

    // set the qdot into the freedoms. the joint chain share the same freedoms point with this map below
    for (auto &f : freedoms)
    {
        f.second->vdot = q_dot[f.first];
    }
    // update COM vel
    UpdateCoMVel();
}

void cRobotModel::SetOmega(EIGEN_V_tVector &omega, int st_joint)
{
    int order = 0;
    for (auto &joint : joint_chain)
    {
        tVector w = omega[st_joint + order];
        tVector3d w3 = Tools::GettVector3d(w) * w.w();
        joint->SetOmega(w3);
    }
}

void cRobotModel::ComputeDCdqdot(tVectorXd &q_dot, tMatrixXd &dcdqdot)
{
    dcdqdot = tMatrixXd::Zero(num_of_freedom, num_of_freedom);
    tVector3d p(0, 0, 0);
    for (auto &link : link_chain)
    {
        link->ComputeMassMatrix();
        link->ComputeJKw_dot(q_dot);
        link->ComputeJKv_dot(q_dot, p);
        link->ComputeJK_dot();

        const tMatrixXd &mass_cartesian = link->GetMassMatrix();
        const tMatrixXd &jk = link->GetJK();
        const tMatrixXd &jk_dot = link->GetJKDot();

        dcdqdot += jk.transpose() * mass_cartesian * jk_dot;
    }
}

std::string cRobotModel::GetCharFile() const { return char_file; }

double cRobotModel::GetScale() const { return scale; }

/**
 * \brief               Calculate dM/dq, M is the mass matrix
 * 
 * M = \sum_i J_vi^T * link_i_massmat * J_vi
 * link_i_massmat = diag(I3 * mass, R * Ibody * R_T)
 * 
 * dMdq = \sum_i (dJ_{i}^Tdq Mi Ji + JiT dMidq Ji + JiT Mi dJidq)
*/
void cRobotModel::ComputedMassMatrixdq(EIGEN_V_MATXD &dMdq) const
{
    // 1. begin to test link's dMdq by numerical gradient
    int dof = GetNumOfFreedom();
    EIGEN_V_MATXD dJkvdq(dof);
    EIGEN_V_MATXD dJkwdq(dof);

    EIGEN_V_MATXD dMidq;
    dMdq.resize(dof, tMatrixXd::Zero(dof, dof));
    for (int id = 0; id < GetNumOfLinks(); id++)
    {
        auto link = static_cast<Link *>(GetLinkById(id));
        // 1. compuate dJdq
        for (int dof = 0; dof < GetNumOfFreedom(); dof++)
        {
            dJkvdq[dof] = link->GetTotalDofdJKv_dq(dof);
            dJkwdq[dof] = link->GetTotalDofdJKw_dq(dof);
        }

        // 2. compute dMdq
        link->ComputedMassMatrixdq_global_freedom(dMidq);

        // 3. compute final result
        // dJvi dq: 3xn3n
        // dMidq: 6xnxn
        const tMatrixXd link_massmat = link->GetMassMatrix();
        for (int i = 0; i < dof; i++)
        {
            tMatrixXd dJdqi = tMatrixXd::Zero(6, dof);
            dJdqi.block(0, 0, 3, dof) = dJkvdq[i];
            dJdqi.block(3, 0, 3, dof) = dJkwdq[i];

            tMatrixXd Ji = tMatrixXd::Zero(6, dof);
            Ji.block(0, 0, 3, dof) = link->GetJKv();
            Ji.block(3, 0, 3, dof) = link->GetJKw();
            dMdq[i] += dJdqi.transpose() * link_massmat * Ji;
            dMdq[i] += Ji.transpose() * dMidq[i] * Ji;
            dMdq[i] += Ji.transpose() * link_massmat * dJdqi;
            // if (i == 6)
            // {
            //     std::cout << "------------link " << id << " dof " << i
            //               << "-----------\n";
            //     std::cout << "Ji = \n" << Ji << std::endl;
            //     std::cout << "link_massmat = \n" << link_massmat << std::endl;
            //     std::cout << "dJdqi = \n" << dJdqi << std::endl;
            //     std::cout << "dMidqi = \n" << dMidq[i] << std::endl;
            // }
        }
    }

    // for (int i = 0; i < dof; i++)
    // {
    //     std::cout << "dMd" << i << " = \n" << dMdq[i] << std::endl;
    // }
}

/**
 * \brief               Calculate d(M)/dq numerically and verify
*/
void cRobotModel::TestLinkdMassMatrixdq()
{
    // 1. calculate the dMdq and M_old in old conf
    tVectorXd q_old = this->mq;

    EIGEN_VV_MATXD dMidq(GetNumOfLinks());
    EIGEN_V_MATXD Mi_old(GetNumOfLinks());
    for (int id = 0; id < GetNumOfLinks(); id++)
    {
        auto link = static_cast<Link *>(GetLinkById(id));
        // 1. analytic solution of dMidq
        link->ComputedMassMatrixdq_global_freedom(dMidq[id]);

        // 2. old dMdq
        Mi_old[id] = link->GetMassMatrix();
    }

    // 2. calculate the numerical dMdq, then compare
    tVectorXd q_new = q_old;
    double eps = 1e-5;
    for (int i = 0; i < GetNumOfFreedom(); i++)
    {
        // 2.1 apply new q
        q_new[i] += eps;
        Apply(q_new, true);

        // 2.2 get the new mass mat
        EIGEN_V_MATXD Mi_new(GetNumOfLinks());
        for (int id = 0; id < GetNumOfLinks(); id++)
        {
            auto link = static_cast<Link *>(GetLinkById(id));

            Mi_new[id] = link->GetMassMatrix();

            // 2.3 compare
            tMatrixXd dMdq_num = (Mi_new[id] - Mi_old[id]) / eps;
            tMatrixXd diff = dMidq[id][i] - dMdq_num;
            std::cout << "[log] TestLinkMassdq link " << id << " dof " << i
                      << " diff = " << diff.norm() << std::endl;
            if (diff.norm() > eps * 10)
            {
                std::cout << "dMdq num = \n" << dMdq_num << std::endl;
                std::cout << "dMdq analytic = \n" << dMidq[id][i] << std::endl;
                std::cout << "error! exit\n";
                exit(1);
            }
        }

        q_new[i] -= eps;
    }
    Apply(q_old, true);
    std::cout << "[log] dMdq verified succ\n";
}

/**
 * \brief               Test d(mass_matrix)/dq
 * 
 * 
*/

void cRobotModel::TestdMassMatrixdq()
{
    tVectorXd q = mq;
    int dof = this->GetNumOfFreedom();
    double eps = 1e-8;
    tMatrixXd old_mass_mat = GetMassMatrix();
    tEigenArr<tMatrixXd> dMassdq;
    ComputedMassMatrixdq(dMassdq);
    for (int i = 0; i < dof; i++)
    {
        q[i] += eps;
        Apply(q, true);
        tMatrixXd new_mass_mat = GetMassMatrix();
        tMatrixXd dMassdq_num = (new_mass_mat - old_mass_mat) / eps;
        tMatrixXd diff = dMassdq_num - dMassdq[i];
        std::cout << "[log] TestdMdq: dof " << i << " norm = " << diff.norm()
                  << std::endl;
        if (diff.norm() > 1e-6)
        {
            printf("[error] dMassdq%d failed diff norm %.10f\n", i,
                   diff.norm());
            std::cout << "num =\n" << dMassdq_num << std::endl;
            std::cout << "ana =\n" << dMassdq[i] << std::endl;
            std::cout << "diff =\n" << diff << std::endl;
            exit(0);
        }
        q[i] -= eps;
    }
    Apply(q, true);
    std::cout << "[log] test dmass dq succ\n";
}

/**
 * \brief       test the mWqqq 
*/
EIGEN_VV_tMatrixD GetJointXqq(Joint *joint, char type)
{
    EIGEN_VV_tMatrixD mWqq;
    int target_freedom = -1;
    if (type == 'T')
        target_freedom = joint->GetNumOfFreedom();
    else if (type == 'W')
        target_freedom = joint->GetNumTotalFreedoms();

    mWqq.resize(target_freedom);
    for (int i = 0; i < target_freedom; i++)
    {
        mWqq[i].resize(target_freedom);
        for (int j = 0; j < target_freedom; j++)
        {
            if (type == 'T')
                mWqq[i][j] = joint->GetMTQQ(i, j);
            else if (type == 'W')
                mWqq[i][j] = joint->GetMWQQ(i, j);
            else
            {
                std::cout << "[error] invalid type " << type << std::endl;
                exit(1);
            }
        }
    }
    return mWqq;
};

EIGEN_V_tMatrixD GetJointXq(Joint *joint, char type)
{
    EIGEN_V_tMatrixD mXq; // mTq or mWq
    int target_dof = -1;
    if (type == 'T')
        target_dof = joint->GetNumOfFreedom();
    else if (type == 'W')
        target_dof = joint->GetNumTotalFreedoms();
    else
    {
        std::cout << "[error] wrong type " << type << std::endl;
        exit(1);
    }
    mXq.resize(target_dof, tMatrix::Zero());
    for (int i = 0; i < target_dof; i++)
    {
        if (type == 'T')
        {
            mXq[i] = joint->GetMTQ(i);
        }
        else if (type == 'W')
        {
            mXq[i] = joint->GetMWQ(i);
        }
        else
        {
            std::cout << "[error] wrong type " << type << std::endl;
            exit(1);
        }
    }
    return mXq;
}
EIGEN_VV_tMatrixD GetJointXqqDiff(const EIGEN_VV_tMatrixD &left,
                                  const EIGEN_VV_tMatrixD &right,
                                  double division = 1)
{
    assert(left.size() == right.size());
    for (int i = 0; i < left.size(); i++)
        assert(left[i].size() == right[i].size());

    EIGEN_VV_tMatrixD diff = left;
    for (int i = 0; i < left.size(); i++)
    {
        for (int j = 0; j < left[i].size(); j++)
        {
            diff[i][j] = (left[i][j] - right[i][j]) / division;
        }
    }
    return diff;
}
EIGEN_V_tMatrixD GetJointXqDiff(const EIGEN_V_tMatrixD &left,
                                const EIGEN_V_tMatrixD &right,
                                double division = 1)
{
    assert(left.size() == right.size());

    EIGEN_V_tMatrixD diff = left;
    for (int i = 0; i < left.size(); i++)
    {
        diff[i] = (left[i] - right[i]) / division;
    }
    return diff;
}
EIGEN_VVV_tMatrixD GetJointXqqq(Joint *joint, char type)
{
    EIGEN_VVV_tMatrixD mWqqq;
    int target_freedom = -1;
    if (type == 'T')
        target_freedom = joint->GetNumOfFreedom();
    else if (type == 'W')
        target_freedom = joint->GetNumTotalFreedoms();
    mWqqq.resize(target_freedom);
    for (int i = 0; i < target_freedom; i++)
    {
        mWqqq[i].resize(target_freedom);
        for (int j = 0; j < target_freedom; j++)
        {
            mWqqq[i][j].resize(target_freedom);
            for (int k = 0; k < target_freedom; k++)
            {
                if (type == 'T')
                    mWqqq[i][j][k] = joint->GetMTQQQ(i, j, k);
                else if (type == 'W')
                    mWqqq[i][j][k] = joint->GetMWQQQ(i, j, k);
                else
                {
                    std::cout << "[error] invalid type " << type << std::endl;
                    exit(1);
                }
            }
        }
    }
    return mWqqq;
};

void cRobotModel::TestmWqqq()
{
    tVectorXd q_old = Getq();
    for (int i = 0; i < GetNumOfJoint(); i++)
    {
        TestJointmWqqq(i);
    }
    Apply(q_old, true);
    std::cout << "[log] TestmWqqq for all joints succ\n";
}

/**
 * \brief           Test the mTqqq for each joint
*/
void cRobotModel::TestmTqqq()
{
    tVectorXd q_old = Getq();
    for (int i = 0; i < GetNumOfJoint(); i++)
    {
        TestJointmTqqq(i);
    }
    Apply(q_old, true);
    std::cout << "[log] TestmTqqq for all joints succ\n";
}

void cRobotModel::TestJointmTqqq(int id)
{
    // 1. remember the old mTqq, and the analytic mTqqq
    auto joint = dynamic_cast<Joint *>(GetJointById(id));
    EIGEN_VV_tMatrixD Tqq_old = GetJointXqq(joint, 'T');
    EIGEN_VVV_tMatrixD Tqqq_ana = GetJointXqqq(joint, 'T');

    tVectorXd q = Getq();
    double eps = 1e-6;
    int local_freedom = joint->GetNumOfFreedom();
    for (int i = 0; i < local_freedom; i++)
    {
        int global_i = joint->GetFreedoms(i)->id;
        q[global_i] += eps;
        Apply(q, true);
        EIGEN_VV_tMatrixD Tqq_new = GetJointXqq(joint, 'T');
        EIGEN_VV_tMatrixD Tqqq_num = GetJointXqqDiff(Tqq_new, Tqq_old, eps);

        for (int dof1 = 0; dof1 < local_freedom; dof1++)
            for (int dof2 = 0; dof2 < local_freedom; dof2++)
            {
                const tMatrix &num = Tqqq_num[dof1][dof2],
                              &ana = Tqqq_ana[i][dof1][dof2];
                tMatrix diff = ana - num;
                double diff_norm = diff.norm();

                // std::cout << "joint " << id << " d" << i << " for d^2T/q"
                //           << dof1 << "q" << dof2 << " diff norm = " << diff_norm
                //           << std::endl;
                if (diff_norm > eps)
                {
                    printf("[error] TestTqqq joint %d d^3T/q%dq%dq%d diff norm "
                           "= %.10f\n",
                           id, i, dof1, dof2, diff_norm);
                    std::cout
                        << "[error] wrong result, num_norm = " << num.norm()
                        << " ana_norm = " << ana.norm() << std::endl;
                    std::cout << "num = \n" << num << std::endl;
                    std::cout << "ana = \n" << ana << std::endl;
                    exit(0);
                }
            }
        q[global_i] -= eps;
    }
    printf("[log] TestTqqq joint %d succ\n", id);
    Apply(q, true);
}

void cRobotModel::TestJointmWqqq(int id)
{
    // 1. remember the old mTqq, and the analytic mTqqq
    auto joint = dynamic_cast<Joint *>(GetJointById(id));
    EIGEN_VV_tMatrixD Wqq_old = GetJointXqq(joint, 'W');
    EIGEN_VVV_tMatrixD Wqqq_ana = GetJointXqqq(joint, 'W');

    tVectorXd q = Getq();
    double eps = 1e-6;
    int local_freedom = joint->GetNumOfFreedom();
    int total_freedom = joint->GetNumTotalFreedoms();
    for (int i = 0; i < total_freedom; i++)
    {
        int global_i = joint->GetPrevFreedomIds()[i];
        q[global_i] += eps;
        Apply(q, true);
        EIGEN_VV_tMatrixD Wqq_new = GetJointXqq(joint, 'W');
        EIGEN_VV_tMatrixD Wqqq_num = GetJointXqqDiff(Wqq_new, Wqq_old, eps);

        for (int dof1 = 0; dof1 < total_freedom; dof1++)
            for (int dof2 = 0; dof2 < total_freedom; dof2++)
            {
                const tMatrix &num = Wqqq_num[dof1][dof2],
                              &ana = Wqqq_ana[i][dof1][dof2];
                tMatrix diff = ana - num;
                double diff_norm = diff.norm();

                // std::cout << "joint " << id << " d" << i << " for d^2T/q"
                //           << dof1 << "q" << dof2 << " diff norm = " << diff_norm
                //           << std::endl;
                if (diff_norm > eps)
                {
                    printf("[error] TestMqqq joint %d d^3W/q%dq%dq%d diff norm "
                           "= %.10f\n",
                           id, i, dof1, dof2, diff_norm);
                    std::cout
                        << "[error] wrong result, num_norm = " << num.norm()
                        << " ana_norm = " << ana.norm() << std::endl;
                    std::cout << "num = \n" << num << std::endl;
                    std::cout << "ana = \n" << ana << std::endl;
                    exit(0);
                }
            }
        q[global_i] -= eps;
    }
    printf("[log] TestMWqqq for joint %d succ\n", id);
    Apply(q, true);
}

void cRobotModel::TestmTqq()
{
    tVectorXd q_old = mq;
    for (int i = 0; i < GetNumOfJoint(); i++)
    {
        TestJointmTqq(i);
    }
    Apply(q_old, true);
    std::cout << "[log] TestmTqq for all joints succ\n";
}
void cRobotModel::TestmWqq()
{
    tVectorXd q_old = mq;
    for (int i = 0; i < GetNumOfJoint(); i++)
    {
        TestJointmWqq(i);
    }
    Apply(q_old, true);
    std::cout << "[log] TestmWqq for all joints succ\n";
}

void cRobotModel::TestJointmTqq(int id)
{
    // 1. get mTq old, get mTqq
    auto joint = dynamic_cast<Joint *>(GetJointById(id));
    int local_dof = joint->GetNumOfFreedom();
    int prev_dof = joint->GetFreedoms(0)->id;
    EIGEN_V_tMatrixD mTq(local_dof, tMatrix::Zero()); // 4x4xn
    EIGEN_VV_tMatrixD mTqq(local_dof);                // 4x4xn xn
    for (auto &x : mTqq)
        x.resize(local_dof, tMatrix::Zero());

    for (int i = 0; i < local_dof; i++)
    {
        mTq[i].noalias() = joint->GetMTQ(i);
        for (int j = 0; j < local_dof; j++)
        {
            mTqq[i][j].noalias() = joint->GetMTQQ(i, j);
        }
    }

    // 2. apply freedom, get mTq new, calculate mTqq numerically
    tVectorXd q = mq;
    double eps = 1e-7;
    for (int i = 0; i < local_dof; i++)
    {
        int global_i = joint->GetFreedoms(i)->id;
        q[global_i] += eps;
        Apply(q, true);
        EIGEN_V_tMatrixD mTq_new(local_dof, tMatrix::Zero());
        EIGEN_V_tMatrixD mTqq_num(local_dof, tMatrix::Zero());
        for (int j = 0; j < local_dof; j++)
        {
            mTq_new[j] = joint->GetMTQ(j);
            mTqq_num[j] = (mTq_new[j] - mTq[j]) / eps;

            tMatrix diff = mTqq_num[j] - mTqq[i][j];
            double diff_norm = diff.norm();
            if (diff_norm > eps)
            {
                printf("[error] joint %d dT^2/dq%dq%d diff norm %.10f\n", id, i,
                       j, diff_norm);
                exit(0);
            }
        }

        // compare the mTqq
        q[global_i] -= eps;
    }
    Apply(q, true);

    printf("[log] TestmTqq for joint %d succ\n", id);
}
void cRobotModel::TestJointmWqq(int id)
{
    // 1. get mWq old, get mWqq
    auto joint = dynamic_cast<Joint *>(GetJointById(id));
    int local_dof = joint->GetNumOfFreedom();
    int total_dof = joint->GetNumTotalFreedoms();
    EIGEN_V_tMatrixD mWq_old = GetJointXq(joint, 'W'); // mWq_old[i] = dW/dqi
    EIGEN_VV_tMatrixD mWqq =
        GetJointXqq(joint, 'W'); // mWqq[i][j] = dW^2/dqidqj

    // 2. apply freedom, get mTq new, calculate mTqq numerically
    // if (id == 1)
    // {
    //     for (int i = 0; i < total_dof; i++)
    //     {
    //         printf("joint %d dWdq%d = \n", id, i);
    //         std::cout << mWq_old[i] << std::endl;
    //     }
    //     exit(1);
    // }

    tVectorXd q = mq;
    double eps = 1e-7;
    for (int i = 0; i < total_dof; i++)
    {
        int global_i = joint->GetPrevFreedomIds()[i];
        q[global_i] += eps;
        Apply(q, true);
        EIGEN_V_tMatrixD mWq_new =
            GetJointXq(joint, 'W'); // mWq_new[i] = dW/dqi

        EIGEN_V_tMatrixD mWqq_numi = GetJointXqDiff(mWq_new, mWq_old, eps);

        EIGEN_V_tMatrixD mWqq_analytici(total_dof);
        for (int tmp = 0; tmp < total_dof; tmp++)
        {
            mWqq_analytici[tmp] = mWqq[tmp][i];
        }

        for (int check_dof = 0; check_dof < total_dof; check_dof++)
        {
            tMatrix diff = mWqq_analytici[check_dof] - mWqq_numi[check_dof];
            double diff_norm = diff.norm();
            if (diff_norm > eps)
            {
                printf("[error] TestJointmWqq for joint %d dof %d check_dof %d "
                       "diff norm %.10f\n",
                       id, i, check_dof, diff_norm);
                std::cout << "numerical res = \n"
                          << mWqq_numi[check_dof] << std::endl;
                std::cout << "analytic res = \n"
                          << mWqq_analytici[check_dof] << std::endl;
                exit(0);
            }
        }
        // compare the mTqq
        q[global_i] -= eps;
    }
    Apply(q, true);
    // std::cout << "q = " << q.transpose() << std::endl;
    printf("[log] TestmWqq for joint %d succ\n", id);
}

void cRobotModel::TestmWq()
{
    tVectorXd q_old = mq;
    for (int i = 0; i < GetNumOfJoint(); i++)
    {
        TestJointmWq(i);
    }
    Apply(q_old, true);
    std::cout << "[log] TestmWq for all joints succ\n";
}

void cRobotModel::TestJointmWq(int id)
{
    // 1. get old mW, get current analytic mWq
    auto joint = GetJointById(id);
    tMatrix W_old = joint->GetGlobalTransform();
    int total_freedom = joint->GetNumTotalFreedoms();
    EIGEN_V_tMatrixD mWq(total_freedom, tMatrix::Zero());
    for (int i = 0; i < total_freedom; i++)
    {
        mWq[i] = joint->GetMWQ(i);
    }

    // 2. apply new value
    tVectorXd q = mq;
    double eps = 1e-7;
    int prev_freedom = total_freedom - joint->GetNumOfFreedom();
    for (int i = 0; i < total_freedom; i++)
    {
        int global_i = joint->GetPrevFreedomIds()[i];

        q[global_i] += eps;
        Apply(q, true);
        tMatrix W_new = joint->GetGlobalTransform();
        tMatrix mWq_num = (W_new - W_old) / eps;

        tMatrix diff = mWq[i] - mWq_num;
        double diff_norm = diff.norm();
        if (diff_norm > eps * 10)
        {
            printf("[error] joint %d dW/dq%d diff norm %.10f\n", id, i,
                   diff_norm);
            std::cout << "mWq analytic = \n" << mWq[i] << std::endl;
            std::cout << "mWq numerical = \n" << mWq_num << std::endl;
            exit(0);
        }

        // compare the mTqq
        q[global_i] -= eps;
    }
    Apply(q, true);
    printf("[log] TestmWq for joint %d succ\n", id);
}

tVector3d cRobotModel::GetCoMPosition() const { return com; }
tVector3d cRobotModel::GetComVelocity() const { return com_vel; }

/**
 * \brief               Get the heading of this robot
 * the heading is the rotation angle of root link along with Y axis. we assume the Y axis is the upright
 * \return angle double in [-pi, pi]
 * 
 * IMPORTANCT: the heading angle is different when the unit vec is different. 
 * for example, the heading angles are different when unit_vec = (1, 0, 0) or unit_vec = (0, 0, 1)
 * we choose the result of (1, 0, 0) here
*/
double cRobotModel::GetHeading() const
{
    tMatrix3d root_ori = GetLinkById(0)->GetWorldOrientation();
    tVector3d unit_vec = tVector3d(1, 0, 0);
    unit_vec = root_ori * unit_vec;
    unit_vec[1] = 0;
    unit_vec.normalize();

    double heading = std::atan2(-unit_vec[2], unit_vec[0]);
    return heading;
}