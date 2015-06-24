/*
 * FILE:
 *   multibody.hpp
 *
 * PURPOSE:
 *   Class representing an articulated rigid body. Stores the body's
 *   current state, allows forces and torques to be set, handles
 *   timestepping and implements Featherstone's algorithm.
 *   
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   31-July-2010
 *
 * COPYRIGHT:
 *   Copyright (C) Stephen Thompson, 2011.
 * 
 *   This file is part of Stephen Thompson's Car Physics Demo.
 *
 *   The Car Physics Demo is free software: you can redistribute it
 *   and/or modify it under the terms of the GNU General Public
 *   License as published by the Free Software Foundation, either
 *   version 3 of the License, or (at your option) any later version.
 *
 *   The Car Physics Demo is distributed in the hope that it will be
 *   useful, but WITHOUT ANY WARRANTY; without even the implied
 *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *   See the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with the Car Physics Demo. If not, see
 *   <http://www.gnu.org/licenses/>.
 *
 */

#ifndef MULTIBODY_HPP
#define MULTIBODY_HPP

#include "real.hpp"

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <vector>

class Multibody {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //
    // initialization
    //
    
    Multibody(int n_links,                // NOT including the base
              real_t mass,                // mass of base
              const Vector3r &inertia,    // inertia of base, in base frame; assumed diagonal
              bool fixed_base_,           // whether the base is fixed (true) or can move (false)
              bool can_sleep_);

    ~Multibody();
    
    void setupPrismatic(int i,             // 0 to num_links-1
                        real_t mass,
                        const Vector3r &inertia,       // in my frame; assumed diagonal
                        int parent,
                        const Quaternionr &rot_parent_to_this,  // rotate points in parent frame to my frame.
                        const Vector3r &joint_axis,             // in my frame
                        const Vector3r &r_vector_when_q_zero);  // vector from parent COM to my COM, in my frame, when q = 0.

    void setupRevolute(int i,            // 0 to num_links-1
                       real_t mass,
                       const Vector3r &inertia,
                       int parent,
                       const Quaternionr &zero_rot_parent_to_this,  // rotate points in parent frame to this frame, when q = 0
                       const Vector3r &joint_axis,    // in my frame
                       const Vector3r &parent_axis_position,    // vector from parent COM to joint axis, in PARENT frame
                       const Vector3r &my_axis_position);       // vector from joint axis to my COM, in MY frame

    //
    // get parent
    // input: link num from 0 to num_links-1
    // output: link num from 0 to num_links-1, OR -1 to mean the base.
    //
    int getParent(int link_num) const;
    
    
    //
    // get number of links, masses, moments of inertia
    //

    int getNumLinks() const { return num_links; }
    real_t getBaseMass() const { return base_mass; }
    const Vector3r & getBaseInertia() const { return base_inertia; }
    real_t getLinkMass(int i) const;
    const Vector3r & getLinkInertia(int i) const;
    

    //
    // change mass (incomplete: can only change base mass and inertia at present)
    //

    void setBaseMass(real_t mass) { base_mass = mass; }
    void setBaseInertia(const Vector3r &inertia) { base_inertia = inertia; }


    //
    // get/set pos/vel/rot/omega for the base link
    //

    const Vector3r & getBasePos() const { return base_pos; }    // in world frame
    Eigen::Map<Vector3r> getBaseVel() const { return Eigen::Map<Vector3r>(&real_buf[3]); }     // in world frame
    const Quaternionr & getWorldToBaseRot() const { return base_quat; }     // rotates world vectors into base frame
    Eigen::Map<Vector3r> getBaseOmega() const { return Eigen::Map<Vector3r>(&real_buf[0]); }   // in world frame

    void setBasePos(const Vector3r &pos) { base_pos = pos; }
    void setBaseVel(const Vector3r &vel) { real_buf[3]=vel[0]; real_buf[4]=vel[1]; real_buf[5]=vel[2]; }
    void setWorldToBaseRot(const Quaternionr &rot) { base_quat = rot; }
    void setBaseOmega(const Vector3r &omega) { real_buf[0]=omega[0]; real_buf[1]=omega[1]; real_buf[2]=omega[2]; }


    //
    // get/set pos/vel for child links (i = 0 to num_links-1)
    //

    real_t getJointPos(int i) const;
    real_t getJointVel(int i) const;

    void setJointPos(int i, real_t q);
    void setJointVel(int i, real_t qdot);

    //
    // direct access to velocities as a vector of 6 + num_links elements.
    // (omega first, then v, then joint velocities.)
    //
    const real_t * getVelocityVector() const { return &real_buf[0]; }
    real_t * getVelocityVector() { return &real_buf[0]; }
    

    //
    // get the frames of reference (positions and orientations) of the child links
    // (i = 0 to num_links-1)
    //

    const Vector3r & getRVector(int i) const;   // vector from COM(parent(i)) to COM(i), in frame i's coords
    const Quaternionr & getParentToLocalRot(int i) const;   // rotates vectors in frame parent(i) to vectors in frame i.


    //
    // transform vectors in local frame of link i to world frame (or vice versa)
    //
    Vector3r localPosToWorld(int i, const Vector3r &vec) const;
    Vector3r localDirToWorld(int i, const Vector3r &vec) const;
    Vector3r worldPosToLocal(int i, const Vector3r &vec) const;
    Vector3r worldDirToLocal(int i, const Vector3r &vec) const;
    

    //
    // calculate kinetic energy and angular momentum
    // useful for debugging.
    //

    real_t getKineticEnergy() const;
    Vector3r getAngularMomentum() const;
    

    //
    // set external forces and torques. Note all external forces/torques are given in the WORLD frame.
    //

    void clearForcesAndTorques();
    void addBaseForce(const Vector3r &f) { base_force += f; }
    void addBaseTorque(const Vector3r &t) { base_torque += t; }
    void addLinkForce(int i, const Vector3r &f);
    void addLinkTorque(int i, const Vector3r &t);
    void addJointTorque(int i, real_t Q);

    const Vector3r & getBaseForce() const { return base_force; }
    const Vector3r & getBaseTorque() const { return base_torque; }
    const Vector3r & getLinkForce(int i) const;
    const Vector3r & getLinkTorque(int i) const;
    real_t getJointTorque(int i) const;


    //
    // dynamics routines.
    //

    // timestep the velocities (given the external forces/torques set using addBaseForce etc).
    // also sets up caches for calcAccelerationDeltas.
    //
    // Note: the caller must provide three vectors which are used as
    // temporary scratch space. The idea here is to reduce dynamic
    // memory allocation: the same scratch vectors can be re-used
    // again and again for different Multibodies, instead of each
    // Multibody allocating (and then deallocating) their own
    // individual scratch buffers. This gives a considerable speed
    // improvement, at least on Windows (where dynamic memory
    // allocation appears to be fairly slow).
    //
    void stepVelocities(real_t dt,
                        std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v,
                        std::vector<Matrix3r> &scratch_m);

    // calcAccelerationDeltas
    // input: force vector (in same format as jacobian, i.e.:
    //                      3 torque values, 3 force values, num_links joint torque values)
    // output: 3 omegadot values, 3 vdot values, num_links q_double_dot values
    // (existing contents of output array are replaced)
    // stepVelocities must have been called first.
    void calcAccelerationDeltas(const real_t *force, real_t *output,
                                std::vector<real_t> &scratch_r,
                                std::vector<Vector3r> &scratch_v) const;

    // apply a delta-vee directly. used in sequential impulses code.
    void applyDeltaVee(const real_t * delta_vee) {
        for (int i = 0; i < 6 + num_links; ++i) real_buf[i] += delta_vee[i];
    }
    void applyDeltaVee(const real_t * delta_vee, real_t multiplier) {
        for (int i = 0; i < 6 + num_links; ++i) real_buf[i] += delta_vee[i] * multiplier;
    }

    // timestep the positions (given current velocities).
    void stepPositions(real_t dt);


    //
    // contacts
    //

    // This routine fills out a contact constraint jacobian for this body.
    // the 'normal' supplied must be -n for body1 or +n for body2 of the contact.
    // 'normal' & 'contact_point' are both given in world coordinates.
    void fillContactJacobian(int link,
                             const Vector3r &contact_point,
                             const Vector3r &normal,
                             real_t *jac,
                             std::vector<real_t> &scratch_r,
                             std::vector<Vector3r> &scratch_v,
                             std::vector<Matrix3r> &scratch_m) const;


    //
    // sleeping
    //

    bool isAwake() const { return awake; }
    void wakeUp();
    void goToSleep();
    void checkMotionAndSleepIfRequired(real_t timestep);
    
private:
    Multibody(const Multibody &);  // not implemented
    void operator=(const Multibody &);  // not implemented

    void compTreeLinkVelocities(Vector3r *omega, Vector3r *vel) const;
    void solveImatrix(const Eigen::Matrix<real_t, 6, 1> &rhs, Eigen::Matrix<real_t, 6, 1> &result) const;
    
private:
    int num_links;  // includes base.

    Vector3r base_pos;       // position of COM of base (world frame)
    Quaternionr base_quat;   // rotates world points into base frame

    real_t base_mass;         // mass of the base
    Vector3r base_inertia;   // inertia of the base (in local frame; diagonal)

    Vector3r base_force;     // external force applied to base. World frame.
    Vector3r base_torque;    // external torque applied to base. World frame.
    
    struct Link;
    Link * links;    // array of links, excluding the base. index from 0 to num_links-2.

    
    //
    // real_buf:
    //  offset         size            array
    //   0              6 + num_links   v (base_omega; base_vel; joint_vels)
    //   6+num_links    num_links       D
    //
    // vector_buf:
    //  offset         size         array
    //   0              num_links    h_top
    //   num_links      num_links    h_bottom
    //
    // matrix_buf:
    //  offset         size         array
    //   0              num_links+1  rot_from_parent
    //
    
    std::vector<real_t> real_buf;
    std::vector<Vector3r> vector_buf;
    std::vector<Matrix3r> matrix_buf;

    std::auto_ptr<Eigen::LU<Eigen::Matrix<real_t, 6, 6> > > cached_imatrix_lu;
    
    bool fixed_base;

    // Sleep parameters.
    bool awake;
    bool can_sleep;
    real_t sleep_timer;
};

#endif
