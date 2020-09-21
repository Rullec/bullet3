#include "cIDSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#include <iostream>
#include <fstream>
#define DEBUG_LOG_VEL
extern btVector3 gGravity;

// the conservation of momentums
void cIDSolver::VerifyLinearMomentum()
{
    assert(mEnableVerifyMomentum);
    assert(this->mContactForces.size() ==0 );
    
	{
		// verify linear momentum
		double mass = 0, total_mass = 0;
		tVector old_momentum = tVector::Zero(), new_momentum = tVector::Zero();
        assert(mLinkVel[mFrameId].size() == mNumLinks);
        assert(mLinkVel[mFrameId - 1].size() == mNumLinks);
		for(int i=0; i<mNumLinks; i++)
		{
			if(i ==0) mass = mMultibody->getBaseMass();
			else mass = mMultibody->getLinkMass(i - 1);
			old_momentum += mLinkVel[mFrameId- 1][i] * mass;
			new_momentum += mLinkVel[mFrameId][i] * mass;
			total_mass += mass;
		}
		tVector impulse = btBulletUtil::btVectorTotVector0(gGravity) * total_mass;
		// add external forces
		for(int i=0; i<mNumLinks; i++) impulse += mExternalForces[i];
		impulse *= mCurTimestep;
		tVector momentum_changes = new_momentum - old_momentum;
        tVector error_vec = (impulse - momentum_changes);
#ifdef DEBUG_LOG_VEL
		// std::cout <<"----- frame " << mFrameId << std::endl;
		// std::cout <<"[linear mom] before momentum = " << old_momentum.transpose() << std::endl;
		// std::cout <<"cur momentum = " << new_momentum.transpose() << std::endl;
		std::cout <<"[linear mom] linear momentum changes = " << (momentum_changes).transpose() << std::endl;
		// std::cout <<"[linear mom] lienar impulse = " << (impulse).transpose() << std::endl;
		// // std::cout <<"error vector = " << (error_vec).transpose() << std::endl;
		std::cout <<"[linear mom] relative error = " << error_vec.norm() / impulse.norm() * 100 << "%" << std::endl;
#endif
		
		// std::ofstream fout("test_res.txt", std::ios::app);
		// fout <<"----- frame " << mFrameId << std::endl;
		// fout <<"cur momentum = " << new_momentum.norm() << std::endl;
		// fout <<"momentum changes = " << (momentum_changes).transpose() << std::endl;
		// fout <<"impulse = " << (impulse).transpose() << std::endl;
		// fout <<"relative error = " << (impulse - momentum_changes).norm() / impulse.norm() * 100 << "%" << std::endl;
	}

	// base_pos_new = btBulletUtil::btVectorTotVector0(mMultibody->getBasePos());
	// base_vel = btBulletUtil::btVectorTotVector0(mMultibody->getBaseVel());
	// tVector base_vel_pred = (base_pos_new - base_pos_old) / mCurTimestep;
	// std::cout <<"Base vel diff = " << (base_vel - base_vel_pred).transpose() << std::endl;
	

    // exit(0);
}

/*
    Function: VerifyAngMomentum
        This function will calculate
*/
void cIDSolver::VerifyAngMomentum()
{
    assert(mEnableVerifyMomentum);

    // std::cout <<" void cIDSolver::VerifyAngMomentum()\n";

    // calculate ang momentum of this multibody
    /*
        L_k = m_k*(x_k - p) x v_k + I0 \omega_k
        p is a selected point for calculating it
        I0 = R I_body R^T
        R rotate a vector in local frame to world frame
    */
   tVector old_ang_mom = CalcAngMomentum(mFrameId - 1),
            new_ang_mom = CalcAngMomentum(mFrameId);

    tVector mom_changes = new_ang_mom - old_ang_mom;
    // std::cout << "ang momentum changes = " << mom_changes.transpose() << std::endl;
    // std::cout <<"new ang momentum = " << new_ang_mom.transpose() << std::endl;    
    // tVector base_torque = mExternalTorques[0];
    // std::cout <<"base torque = " << base_torque.transpose() << std::endl;
    // tVector impulse = base_torque * mCurTimestep,
    //         mom_changes = new_ang_mom - old_ang_mom;
    // std::cout <<"impulse = " << impulse.transpose()<<"\nang mom changes = " << mom_changes.transpose() << std::endl;
    // std::cout <<"diff = " << (impulse - mom_changes).transpose() << std::endl;
    // double err = (impulse - mom_changes).norm() / impulse.norm() * 100 ;
    // std::cout <<"omega/err = " << mLinkOmega[mFrameId][0].norm() << " " << err << "\n";
    // std::cout <<"omega = " << mLinkOmega[mFrameId][0].transpose() << "\n";
    tVector ext_torque = mExternalTorques[0];
    tVector impulse = ext_torque * mCurTimestep;
    auto error_vec = mom_changes - impulse;
    std::cout <<"[ang mom] ang momentum changes = " << (mom_changes).transpose() << std::endl;
    std::cout <<"[ang mom] ang impulse = " << impulse.transpose() << std::endl;
    std::cout <<"[ang mom] old ang momentum = " << old_ang_mom.transpose() << std::endl;
    std::cout <<"[ang mom] abs error = " << (error_vec).transpose() << std::endl;
    if(impulse.norm() > 1e-10)
        std::cout <<"[ang mom] relative error = " << error_vec.norm() / impulse.norm() * 100 << "%" << std::endl;
    else
        std::cout <<"[ang mom] relative error = " << error_vec.norm() / old_ang_mom.norm() * 100 << "%" << std::endl;
}

void cIDSolver::VerifyMomentum()
{
    assert(mEnableVerifyMomentum);
    std::cout <<" void cIDSolver::VerifyMomentum()\n";
    VerifyLinearMomentum();
    exit(0);
}

/*
    @Function: VerifyLinkVel
    this function should be called in PostSim(), after the lastest frame info (linkpos, linkrot) has been recorded.
    it will compare the computed vel for each link COM and the fetched vel from btMultibody, so as to validate the simulation step
*/
void cIDSolver::VerifyLinkVel()
{
    // now calculate the vel of link COM by myself
    std::vector<tVector> compute_link_vel(mNumLinks), compute_link_omega(mNumLinks);
    {
        // set up for base link
        btVector3 base_omega_world = btVector3(mMultibody->getRealBuf()[0], mMultibody->getRealBuf()[1], mMultibody->getRealBuf()[2]),
            base_vel_world = btVector3(mMultibody->getRealBuf()[3], mMultibody->getRealBuf()[4], mMultibody->getRealBuf()[5]);
        tQuaternion world_to_base = btBulletUtil::btQuaternionTotQuaternion(mMultibody->getWorldToBaseRot());

        // the vel and omega of base link represented in base local frame
        compute_link_omega[0] = btMathUtil::QuatRotVec(world_to_base, btBulletUtil::btVectorTotVector0(base_omega_world));
        compute_link_vel[0] = btMathUtil::QuatRotVec(world_to_base, btBulletUtil::btVectorTotVector0(base_vel_world));

        // std::cout <<"myself joint " << 0 <<" local omega part 1 = " << compute_link_omega[0].transpose()\
            << ", vel = " << compute_link_vel[0].transpose() << std::endl;

        for(int ID_link_id = 1; ID_link_id < mNumLinks; ID_link_id++)
        {
            // formula: 
            // the vel of COM of link "idx" = w_local x r + v_rel
            // Hint:    w_local is the rotation speed of joint axis expressed in local frame
            //          r is a vector from rotatation center(aka joint) to the COM of this link, expressed in local frame
            //          v_rel is the relative velocity in joint position
            // the omega of this link "idx" = w_parent + R * w_myself, expressed in parent frame
            // Hint:    It's quite simple. Just sum all the omega_local together from root to this link
            // std::cout <<"verify 1 " << ID_link_id << std::endl;
            int cur_multibody_link_id = ID_link_id - 1;
            int parent_multibody_link_id = mMultibody->getParent(cur_multibody_link_id);
            int parent_ID_link_id = parent_multibody_link_id+1;
            auto & cur_link = mMultibody->getLink(cur_multibody_link_id);
            tQuaternion parent_to_local = btBulletUtil::btQuaternionTotQuaternion(mMultibody->getParentToLocalRot(cur_multibody_link_id));
            tVector parent_omega = compute_link_omega[parent_ID_link_id],
                    parent_vel = compute_link_vel[parent_ID_link_id];       // both these 2 values expressed in parent frame

            // std::cout <<"verify 2 " << ID_link_id << std::endl;
            // convert to this local frame and give it to link "idx"
            compute_link_omega[ID_link_id] = btMathUtil::QuatRotVec(parent_to_local, parent_omega);
            compute_link_vel[ID_link_id] = btMathUtil::QuatRotVec(parent_to_local, parent_vel) \
                + btBulletUtil::btVectorTotVector0((-cur_link.m_cachedRVector).cross(btBulletUtil::tVectorTobtVector(compute_link_omega[ID_link_id])));

            // std::cout <<"myself joint " << ID_link_id <<" local omega part1 = " << compute_link_omega[ID_link_id].transpose()\
            << ", vel = " << compute_link_vel[ID_link_id].transpose() << std::endl;
            // add w_local x r
            const btScalar* jointVel = mMultibody->getJointVelMultiDof(cur_multibody_link_id);
            
            // std::cout <<"verify 3 " << ID_link_id << std::endl;
            for (int dof = 0; dof < cur_link.m_dofCount; ++dof)
            {
                compute_link_omega[ID_link_id ] += btBulletUtil::btVectorTotVector0(jointVel[dof] * cur_link.getAxisTop(dof));
                compute_link_vel[ID_link_id] += btBulletUtil::btVectorTotVector0(jointVel[dof] * cur_link.getAxisBottom(dof));
            }
            // std::cout <<"myself joint " << ID_link_id <<" local omega final = " << compute_link_omega[ID_link_id].transpose()\
            << ", vel = " << compute_link_vel[ID_link_id].transpose() << std::endl;
        }

        // rotate all of them to world frame and compare
        for (int ID_link_id = 0; ID_link_id < mNumLinks; ID_link_id++)
        {
            // set up rot & pos
            tQuaternion rot;
            if (0 == ID_link_id)
            {
                rot = btBulletUtil::btQuaternionTotQuaternion(mMultibody->getWorldToBaseRot().inverse());
            }
            else
            {
                int multibody_link_id = ID_link_id - 1;
                rot = btBulletUtil::btQuaternionTotQuaternion(mMultibody->getLinkCollider(multibody_link_id)->getWorldTransform().getRotation());
            }
            compute_link_omega[ID_link_id] = btMathUtil::QuatRotVec(rot, compute_link_omega[ID_link_id]);
            compute_link_vel[ID_link_id] = btMathUtil::QuatRotVec(rot, compute_link_vel[ID_link_id]);

            double omega_err = (compute_link_omega[ID_link_id] - mLinkOmega[mFrameId][ID_link_id]).norm(),
                vel_err = (compute_link_vel[ID_link_id] - mLinkVel[mFrameId][ID_link_id]).norm();
            if(omega_err > 1e-10 || vel_err > 1e-10)
            {
                std::cout <<"[log] cIDSolver::VerifyLinkVel: diff for joint omega " << ID_link_id << \
                    (compute_link_omega[ID_link_id] - mLinkOmega[mFrameId][ID_link_id]).transpose() << std::endl;
                std::cout <<"[log] cIDSolver::VerifyLinkVel: diff for joint vel " << ID_link_id << \
                    (compute_link_vel[ID_link_id] - mLinkVel[mFrameId][ID_link_id]).transpose() << std::endl;
                exit(0);
            }
            // assert(omega_buffer < 1e-10);
            // assert(omega_buffer < 1e-10);

        }

    }
    // std::ofstream fout("verify_link_vel.log", std::ios::app);

    const std::string info []={"disabled", "enabled"};
    assert(mEnableVerifyVel);
#ifdef DEBUG_LOG_VEL
    std::cout <<"ext force = " << info[mEnableExternalForce] <<", ext torque = " << info[mEnableExternalTorque]\
        << ", applied joint torque " << info[mEnableAppliedJointTorque] << "\n";
#endif
    // verify vel = (pos_new - pos_old) / time
    tVector diff_vel = tVector::Zero();
    for(int i=0; i<mNumLinks; i++)
    {
        tVector pred_vel = (mLinkPos[mFrameId][i] - mLinkPos[mFrameId-1][i]) / this->mCurTimestep,
                true_vel = mLinkVel[mFrameId][i];
        
        double relative_error = (true_vel - pred_vel).norm() / true_vel.norm();
#ifdef DEBUG_LOG_VEL
        std::cout <<"[log] link " << i <<" discrete vel relative error = " << relative_error * 100 << "%";

        if(relative_error > 1e-10)
        {
            std::cout <<", pred vel = " << pred_vel.transpose() <<", true vel = " << true_vel.transpose();
        }
        std::cout << std::endl;
#endif
    }
    // exit(0);
}

void cIDSolver::VerifyLinkOmega()
{
    assert(mEnableVerifyVel);

    double err = 0;
    for(int i=0; i<mNumLinks; i++)
    {
        // tQuaternion new_rot = cMathUtil::RotMatToQuaternion(mLinkRot[mFrameId][i]);
        // tQuaternion old_rot = cMathUtil::RotMatToQuaternion(mLinkRot[mFrameId-1][i]);
        // tQuaternion diff_rot = new_rot * old_rot.inverse();
        tQuaternion diff_rot = btMathUtil::RotMatToQuaternion(mLinkRot[mFrameId][i] * mLinkRot[mFrameId-1][i].transpose());
        tVector aa_omega = btMathUtil::QuaternionToAxisAngle(diff_rot) / mCurTimestep;
#ifdef DEBUG_LOG_VEL
        // std::cout <<"link " << i <<" calculated omega = " << aa_omega.transpose() << std::endl;
        // std::cout <<"link " << i <<" true omega = " << mLinkOmega[mFrameId][i].transpose() << std::endl;
        

        tVector error_vector = aa_omega - mLinkOmega[mFrameId][i];
        // std::cout <<"link " << i <<" diff = " << error_vector.transpose() << std::endl;
        // std::cout <<"link " << i <<" omega relative error = " << error_vector.norm()/ (mLinkOmega[mFrameId][i].norm() + 1e-6) * 100 << "%\n";
#endif
        // if(error_vector.norm() > 1e-6)
        // {
        //     std::cout <<"rot mat = \n"<< mLinkRot[mFrameId][i] * mLinkRot[mFrameId-1][i].transpose() << std::endl;
        //     std::cout <<"diff quaternion = " << diff_rot.coeffs().transpose() <<", axis angle = " << aa_omega.transpose() * mCurTimestep << std::endl;;
        //     std::cout <<"axis angle to quater = " << cMathUtil::AxisAngleToQuaternion(aa_omega * mCurTimestep).coeffs().transpose() << std::endl;
        //     std::cout <<"from quater to rotmat = \n" << cMathUtil::RotMat(diff_rot) << std::endl;
        //     // exit(0);
        // }
    }
    // exit(0);
}

tVector cIDSolver::CalcCOM(int frame_id)
{
    assert(frame_id >=0 && frame_id <= mFrameId);
    tVector COM = tVector::Zero();
    for(int i=0; i<mNumLinks; i++)
    {
        COM += mLinkMass[i] * mLinkPos[frame_id][i];
    }
    COM /= mTotalMass;
    return COM;
}

tVector cIDSolver::CalcAngMomentum(int frame_id)
{
    assert(frame_id >=0 && frame_id <= mFrameId);
    tVector ang_m = tVector::Zero();
    for(int i=0; i<mNumLinks; i++)
    {
        double m_k = mLinkMass[i];
        tVector x_k = mLinkPos[frame_id][i];
        tVector p = CalcCOM(frame_id);
        // tVector p = tVector::Zero();
        tVector v_k = mLinkVel[frame_id][i];
        tVector w_k = mLinkOmega[frame_id][i];
        tMatrix R = mLinkRot[frame_id][i];
        tMatrix I_body;
        if(i == 0)
        {
            I_body = btBulletUtil::btVectorTotVector0(mMultibody->getBaseInertia()).asDiagonal();
        }
        else
        {
            I_body = btBulletUtil::btVectorTotVector0(mMultibody->getLinkInertia(i-1)).asDiagonal();
        }
        
        // tVector I0 = R * I_body * R.transpose();
        tMatrix I0 = R * I_body * R.transpose();
        tVector part1 = m_k * (x_k - p).cross3(v_k),
                part2 = I0 * w_k;
        ang_m += part2;
    }
    
    // std::cout <<" I body = " << I_body.diagonal().transpose() << std::endl;
    // std::cout <<" part1 = " << part1.transpose() << std::endl;
    return ang_m;
}