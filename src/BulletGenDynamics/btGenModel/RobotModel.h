#ifndef ROBOT_ROBOTMODEL_H
#define ROBOT_ROBOTMODEL_H
#pragma warning(disable : 26451)
#include "BaseObject.h"
#include "ModelEigenUtils.h"
#include "RobotCollider.h"
#include <map>
// #include "TargetProject.h"
#include "tools.h"

enum ModelType
{
    ROM = 0,
    ASF,
    JSON
};

enum ROM_TYPE
{
    ROM_GLOBAL = 0,
    ROM_LOCAL
};

class TiXmlElement;
typedef std::vector<BaseObject *> Chain;
class btDiscreteDynamicsWorld;
class cRobotModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // cRobotModel(const char* model_file);
    cRobotModel();
    ~cRobotModel();

    // cRobotModel(const char* model_file, double scale, int type);
    virtual void Init(const char *model_file, double scale, int type);
    const std::string &GetName() const;

    BaseObject *GetRoot() const;
    BaseObject *GetJoint(std::string joint_name) const;
    BaseObject *GetLink(std::string name) const;

    int GetNumOfFreedom() const;
    void Apply(const std::vector<double> &ang, int st,
               bool compute_gradient);
    void Apply(const std::vector<double> &ang, tVector3d cycle_step, int st,
               bool compute_gradient);
    void Apply(const tVectorXd &ang, bool compute_gradient);

    void GetJointLimit(tVectorXd &lb, tVectorXd &up) const;
    tVector3d GetFreedomDirectionWorldFrame(int dof_id) const;

    void ComputeGradientNumerically();
    void ComputeGradientByGeometry();
    void ComputeGradientAnalytically();
    // void ComputeJacobiByTargetPoint(TargetPoint* tp, tMatrixXd& j);
    void ComputeJacobiByGivenPoint(std::string name, const tVector3d &point,
                                   tMatrixXd &j) const;
    void ComputeJacobiByGivenPointTotalDOFWorldFrame(int link_id,
                                                     const tVector3d &point,
                                                     tMatrixXd &j) const;
    // void ComputeJacobiByGivenPointTotalDOFLocalFrame(int link_id, const
    // tVector3d& point, tMatrixXd& j) const;
    void ComputeCoMJacobi(tMatrixXd &j);
    void ComputeComJacobiNumerically();

    void ComputeSecondDeriveNumerically();
    void ComputeSecondDeriveNumerically(tVectorXd &q_dot, int frame = 0);
    void ComputeSecondDerive(tVectorXd &q_dot);

    void ComputeMassTimesLinkJvSum(int frame, EIGEN_V_MATXD &data);
    void ComputeMassTimesLinkJvSumNumerically(int frame, EIGEN_V_MATXD &data);

    void ComputeLinkJvNumerically();
    void ComputeLinkJvNumerically(BaseObject *end_link, int st_frame,
                                  tMatrixXd &grad);

    void ComputeJv(); // compute link jv
    void ComputeJw(); // compute link jw
    void ComputeAngularVelocity(tMatrixXd &omega, tVectorXd &q_dot);
    void SetOmega(EIGEN_V_tVector &omega, int st_joint);
    const tMatrixXd &GetMassMatrix() const { return mass_matrix; }
    const tMatrixXd &GetCoriolisMatrix() const { return coriolis_matrix; }
    void ComputedMassMatrixdq(EIGEN_V_MATXD &dMdq) const;
    void TestLinkdMassMatrixdq();
    void TestdMassMatrixdq();
    void Compute_dCoriolisMatrix_dqandqdot(EIGEN_V_MATXD &dCdq,
                                           EIGEN_V_MATXD &dCdqdot);
    void Test_dCoriolisMatrix_dqandqdot();
    void InitModel();
    void InitChildrenChain();
    void InitDofidToJointMap();
    void InitLocalPos();
    void InitDeepMimicMotionSize();
    void InitMatrix();
    const Chain &GetChains() const { return children_chain; }
    const Chain &GetJointChains() const { return joint_chain; }
    // void GetTargetPosition(TargetPoint& tp, tVector3d& pos);
    // void GetTargetPosition(TargetPoint* tp, tVector3d& pos);
    // tVector3d GetTargetPosition(TargetPoint* tp);
    // void GetCoMPosition(CoMPoint* tp, tVector3d& pos);
    tVector3d GetCoMPosition() const { return com; };
    tVector3d GetObjectPos(std::string name, int type);
    void GetObjectGradient(std::string name, int type, tMatrixXd jac);
    int GetNumOfLinks() const { return static_cast<int>(link_chain.size()); }
    int GetNumOfJoint() const { return static_cast<int>(joints.size()); }
    int GetNumOfValidJoint() const { return num_of_valid_joint; }
    int GetDeepMimicMotionSize() const { return deep_mimic_motion_size; };
    BaseObject *GetLinkById(int id) const;
    BaseObject *GetJointById(int id) const;
    BaseObject *GetJointByDofId(int dof_id) const;

    void PrintLocalTransform();
    EIGEN_V_tVector3d GetJointPos();
    EIGEN_V_MATXD GetGlobalTransforms();
    std::vector<std::string> GetJointNames();
    void ResetPose();

    void GetAllLinksRotation(tEigenArr<tMatrix> &r);
    void GetAllRotation(tEigenArr<tMatrix> &r);
    void GetAllLinksPos(EIGEN_V_tVector3d &r);

    void GetAllLinksPosTimesMass(tMatrixXd &x);
    void GetAllLinksPosTimesMass(tVector3d &p);

    tMatrixXd &GetJW() { return Jw; }
    tMatrixXd &GetJV() { return Jv; }

    double GetTotalMass() { return total_mass; }
    BaseObject *GetEndLink() { return end_link; }

    void SetComputeSecondDerive(bool flag);
    void SetComputeThirdDerive(bool flag);
    bool GetComputeSecondDerive();
    bool GetComputeThirdDerive();
    const tVectorXd &Getqdot() const { return mqdot; }
    const tVectorXd &Getq() const { return mq; }

    std::string GetCharFile() const;
    double GetScale() const;
    void TestmWqqq();
    void TestmTqqq();
    void TestmTqq();
    void TestmWqq();
    void TestmWq();

protected:
    void TestJointmTqqq(int joint);
    void TestJointmWqqq(int joint);
    void TestJointmTqq(int joint);
    void TestJointmWqq(int joint);
    void TestJointmWq(int joint);
    // compute methods is prohibited to be called outside of the class
    void ComputeMassMatrix();
    void ComputeCoriolisMatrix(tVectorXd &q_dot);
    void ComputeDampingMatrix();
    void ComputeDCdqdot(tVectorXd &q_dot, tMatrixXd &dcdqdot);

    void Travel() const;

    void Update(bool compute_gradient = false);

    void ExportAsf(const char *file_path);

    std::map<int, Freedom *> &GetFreedoms();
    const std::map<std::string, BaseObject *> &GetJointsMap() const
    {
        return joints;
    }
    const std::map<int, BaseObject *> &GetJointIdMap() const
    {
        return joint_id_map;
    }
    const std::map<int, BaseObject *> &GetLinkIdMap() const
    {
        return link_id_map;
    }

    void UpdateCoM();
    void UpdateMass();

    void SaveModel(const char *file);
    void SetAns(std::vector<double> &ans_q) { this->ans = ans_q; }

    void Load(const char *model_file);

    void LoadFreedom(const char *name, TiXmlElement *doc_joint,
                     tVector3d &joint_pose);

    void InsertFreedomMap(BaseObject *joint);

    void AddLink(std::string link_name, tVector3d &local_pos,
                 tVector3d &local_rotation, const char *mesh_path,
                 tVector3f &mesh_rotation, tVector3f &mesh_scale, double mass);

    void AddRootJoint(const char *root_name, JointType root_type);
    void AddRootLink(const char *root_name = "root");

    void AddJoint(std::string joint_name, std::string father_name,
                  std::string child_name, const tVector3d &local_pos,
                  const tVector3d &local_rotation);
    void AddSphericalFreedoms(BaseObject *joint, const tVector3d &lower_limit,
                              const tVector3d &upper_limit);
    void AddRevoluteJointFreedoms(BaseObject *joint, double lower_limit = -M_PI,
                                  double upper_limit = M_PI);
    void UpdateFreedomId();

    void LoadBaseMesh();

    void LoadAsf(const char *file_path);

    void LoadJsonModel(const char *file_path, double scale);

    void GetMeshRotation(tVector3d &mesh_rotation, tVector3d &direction) const;

    void InitJointTypeMap();
    void InitShapeMap();

    BaseObject *GetBaseObjectById(int id, int type = JOINT) const;

    BaseObject *root; // root joint
    std::map<std::string, BaseObject *> links;
    std::map<std::string, BaseObject *> joints;
    std::map<int, BaseObject *> joint_id_map;
    std::map<int, BaseObject *> link_id_map;
    std::map<int, Freedom *> freedoms;
    std::vector<int> dof_joint_map; // map the dof id to joint id

    // std::map<std::string, Mesh*>		mesh_map;
    tMatrix4f joint_mesh_mat;

    std::string name;
    int num_of_freedom;
    std::vector<double> ans;
    BaseObject *end_link;
    std::vector<BaseObject *> children_chain;
    std::vector<BaseObject *> joint_chain;
    std::vector<BaseObject *> link_chain;

    tVector3d com;
    tMatrix com_transform;
    double total_mass;

    int num_of_valid_joint; // used when load asf, to determine how many lines
                            // should be read in one frame

    static std::map<std::string, int> joint_type_map;
    static std::map<std::string, int> shape_map;

    int deep_mimic_motion_size;
    int model_type;
    bool compute_second_deriv;
    bool compute_third_deriv;

    tMatrixXd Jv; // (k * n_freedom, 3 * n_links)
    tMatrixXd Jw; // (k * n_freedom, 3 * n_joints)
    tMatrixXd mass_matrix;
    tMatrixXd inv_mass_matrix;
    tMatrixXd coriolis_matrix;
    std::vector<BaseObject *> ee;

    // sim vars
    tVectorXd mq, mqdot;
    std::string char_file;
    double scale;
};

#endif // ROBOT_ROBOTMODEL_H
