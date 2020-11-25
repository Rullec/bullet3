#include "BulletGenDynamics/btGenController/Trajectory/btTrajContactMigrator.h"
#include "BulletGenDynamics/btGenController/Trajectory/btTraj.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/FileUtil.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"

btTrajContactMigrator::tGivenContactPt::tGivenContactPt(
    int link_id, const tVector &local_pos)
{
    mLinkId = link_id;
    mLocalPos = local_pos;
    btMathUtil::IsHomogeneousPos(local_pos, true);
}

void btTrajContactMigrator::MigrateTrajContactByGivenInfo(
    cRobotModelDynamics *model, btTraj *traj, std::string given_contact_info)
{
    assert(traj != nullptr);
    /* 1. form the new filename, check existence
        if it exists, ask the user whether do we need to read it or resolve
        resolve: resolve
        read: read the old ref traj, then return 
    */
    std::string migrated_path = GetAndCheckNewTrajPath(traj->GetLoadPath());

    // 2. load the migrated local info and validate
    tGivenContactPtInfo given_contact_pts =
        LoadGivenContactPts(model, given_contact_info);

    // 2. imigrate contacts in each frame, note that we will modify the given traj directly
    btTrajContactMigrator::MigrateTrajContact(model, given_contact_pts, traj);

    // 3. write down, then return
    traj->SaveTraj(migrated_path, model);
    printf("[log] contact-migrated traj saved to %s", migrated_path.c_str());
    exit(0);
}

std::string btTrajContactMigrator::GetAndCheckNewTrajPath(std::string raw_path)
{
    std::string raw_dir = btFileUtil::GetDir(raw_path);
    std::string raw_filename = btFileUtil::GetFilename(raw_path);
    std::string raw_purename = btFileUtil::RemoveExtension(raw_filename);
    std::string raw_suffix = btFileUtil::GetExtension(raw_path);

    if (raw_purename.find("migrated") != -1)
    {
        printf("[log] GetAndCheckNewTrajPath: the origin traj %s has been "
               "migrated, it should be migrated anymore\n",
               raw_purename.c_str());
        exit(0);
    }
    std::string migrated_filename = raw_purename + "_migrated." + raw_suffix;
    std::string migrated_path =
        btFileUtil::ConcatFilename(raw_dir, migrated_filename);
    // printf("raw path %s\nraw dir %s\nraw filename %s\nraw name %s\nraw suffix "
    //        "%s\nmigrated "
    //        "name %s\nmigrated path %s\n",
    //        raw_path.c_str(), raw_dir.c_str(), raw_filename.c_str(),
    //        raw_purename.c_str(), raw_suffix.c_str(), migrated_filename.c_str(),
    //        migrated_path.c_str());

    if (btFileUtil::ExistsFile(migrated_path) == true)
    {
        printf("[warn] MigrateTrajContactByGivenInfo: target path %s exist, "
               "overwrite\n",
               migrated_path.c_str());
    }
    else
    {
        printf("[log] MigrateTrajContactByGivenInfo begin to write to %s\n",
               migrated_path.c_str());
    }
    return migrated_path;
}

btTrajContactMigrator::tGivenContactPtInfo
btTrajContactMigrator::LoadGivenContactPts(cRobotModelDynamics *model,
                                           const std::string &conf_path)
{
    // 1. load json
    btTrajContactMigrator::tGivenContactPtInfo mSupposedContactPt;
    // 2. get contact points' info and validate

    mSupposedContactPt.clear();
    Json::Value root;
    btJsonUtil::LoadJson(conf_path, root);
    std::string skeleton_path =
        btJsonUtil::ParseAsString("skeleton_path", root);
    int num = btJsonUtil::ParseAsInt("num_of_supposed_contact_points", root);
    const Json::Value &lst =
        btJsonUtil::ParseAsValue("supposed_contact_point_lst", root);

    if (skeleton_path != model->GetCharFile())
    {
        std::cout << "[error] btTrajContactMigrator: supposed contact skeleton "
                     "file inconsistent "
                  << skeleton_path << " != " << model->GetCharFile()
                  << std::endl;
        exit(0);
    }
    if (num != lst.size())
    {
        std::cout << "[error] the supposed contact points num is "
                     "consistent "
                  << num << " != " << lst.size() << std::endl;
        exit(1);
    }

    for (int i = 0; i < num; i++)
    {
        auto cur_pt = lst[i];
        std::string link_name = btJsonUtil::ParseAsString("link_name", cur_pt);
        int link_id = btJsonUtil::ParseAsInt("link_id", cur_pt);
        tVector3d local_pos = btJsonUtil::ReadVectorJson(
                                  btJsonUtil::ParseAsValue("local_pos", cur_pt))
                                  .segment(0, 3);

        // check link id and link name
        auto link = model->GetLinkById(link_id);
        if (link->GetName() != link_name)
        {
            std::cout << "[error] nqr supposed pt: the given link name "
                      << link_name << " != "
                      << " model link name " << link->GetName() << std::endl;
            exit(1);
        }

        if (mSupposedContactPt.find(link_id) == mSupposedContactPt.end())
        {
            mSupposedContactPt[link_id] = tEigenArr<tGivenContactPt>();
        }

        mSupposedContactPt[link_id].push_back(
            tGivenContactPt(link_id, btMathUtil::Expand(local_pos, 1)));
    }
    return mSupposedContactPt;
}

void btTrajContactMigrator::MigrateTrajContact(
    cRobotModelDynamics *model, const tGivenContactPtInfo &given_contact,
    btTraj *traj)
{
    // push state
    model->PushState("migrate_traj_contact");
    // model set 2nd and 3rd derivates to false
    model->SetComputeSecondDerive(false);
    model->SetComputeThirdDerive(false);

    /*
        2. frames iteration
    */
    int num_of_freedom = model->GetNumOfFreedom();
    for (int frame_id = 0; frame_id < traj->mNumOfFrames; frame_id++)
    {
        printf("---------Migrate contact for frame %d---------\n", frame_id);
        // 1. apply q, update gradient = True
        model->Apply(traj->mq[frame_id], true);
        int num_of_contacts = 0; // new contact points counting
        std::vector<int> link_id_array(0);
        tEigenArr<tVector3d> local_pos_array(0);
        tEigenArr<tVector3d> global_pos_array(0);
        tEigenArr<tVector3d> force_array(0);

        for (auto &link_pts : given_contact)
        {
            // for this link's contact points
            int link_id = link_pts.first; // get link id
            const tEigenArr<tGivenContactPt> &given_pts = link_pts.second;

            // get total, old contact gen force in this link
            tVectorXd old_gen_contact = tVectorXd::Zero(num_of_freedom);
            for (auto &f : traj->mContactForce[frame_id])
            {
                if (f->mLinkId == link_id)
                {
                    tMatrixXd jac;
                    model->ComputeJacobiByGivenPointTotalDOFLinkLocalFrame(
                        link_id, f->mLocalPos.segment(0, 3), jac);
                    std::cout
                        << "old: link " << link_id
                        << " force = " << f->mForce.segment(0, 3).transpose()
                        << std::endl;
                    old_gen_contact +=
                        jac.transpose() * f->mForce.segment(0, 3);
                }
            }
            if (old_gen_contact.norm() < 1e-10)
                continue;
            // the new cartesian force should generate the same gen contact force
            tMatrixXd new_contact_jac =
                tMatrixXd::Zero(3 * given_contact.size(), num_of_freedom);
            for (int i = 0; i < given_pts.size(); i++)
            {
                tMatrixXd jac;
                model->ComputeJacobiByGivenPointTotalDOFLinkLocalFrame(
                    link_id, given_pts[i].mLocalPos.segment(0, 3), jac);
                new_contact_jac.block(i * 3, 0, 3, num_of_freedom) = jac;
            }
            // calculate the cartesian force
            tVectorXd new_contact_force =
                (new_contact_jac * new_contact_jac.transpose()).inverse() *
                new_contact_jac * old_gen_contact;
            // check diff
            {
                tVectorXd new_gen_force =
                    new_contact_jac.transpose() * new_contact_force;
                tVectorXd gen_diff = new_gen_force - old_gen_contact;
                std::cout << "old gen contact f = "
                          << old_gen_contact.transpose() << std::endl;
                std::cout << "new gen contact f = " << new_gen_force.transpose()
                          << std::endl;
                std::cout << "diff f = " << gen_diff.transpose() << std::endl;
                std::cout << "diff percent = "
                          << gen_diff.norm() / old_gen_contact.norm() * 100
                          << " %\n";
            }
            // push the new contact forces into a set
            for (int i = 0; i < given_pts.size(); i++)
            {
                tVector3d contact_force = new_contact_force.segment(i * 3, 3);
                tVector3d local_pos = given_pts[i].mLocalPos.segment(0, 3);
                tVector3d global_pos = (model->GetLinkById(given_pts[i].mLinkId)
                                            ->GetGlobalTransform() *
                                        btMathUtil::Expand(local_pos, 1))
                                           .segment(0, 3);

                link_id_array.push_back(link_id);
                local_pos_array.push_back(local_pos);
                global_pos_array.push_back(global_pos);
                force_array.push_back(contact_force);
                num_of_contacts++;
            }
        }

        // 4. release old contact points
        auto &traj_contact = traj->mContactForce[frame_id];
        for (auto &x : traj_contact)
            delete x;
        traj_contact.clear();
        // 5. push; new contact points
        for (int i = 0; i < num_of_contacts; i++)
        {
            int link_id = link_id_array[i];
            btGenRobotCollider *collider = model->GetLinkCollider(link_id);
            std::cout << "[log] link " << link_id << " add contact force "
                      << force_array[i].transpose() << std::endl;
            btGenMBContactForce *new_contact_force = new btGenMBContactForce(
                collider, nullptr, btMathUtil::Expand(force_array[i], 0),
                btMathUtil::Expand(global_pos_array[i], 1),
                btMathUtil::Expand(local_pos_array[i], 1), false);
            traj_contact.push_back(new_contact_force);
        }
    }

    // pop state
    model->PopState("migrate_traj_contact");
}