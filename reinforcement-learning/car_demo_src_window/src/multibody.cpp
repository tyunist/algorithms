/*
 * FILE:
 *   multibody.cpp
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

#include "multibody.hpp"

using namespace Eigen;

using std::vector;

// #define INCLUDE_GYRO_TERM 

namespace {
    const real_t SLEEP_EPSILON = real_t(0.01);  // this is a squared velocity (m^2 s^-2)
    const real_t SLEEP_TIMEOUT = real_t(5);     // in seconds
}


//
// Link struct
//

struct Multibody::Link {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    real_t joint_pos;    // qi

    real_t mass;         // mass of link
    Vector3r inertia;   // inertia of link (local frame; diagonal)

    int parent;         // index of the parent link (assumed to be < index of this link), or -1 if parent is the base link.

    Quaternionr zero_rot_parent_to_this;    // rotates vectors in parent-frame to vectors in local-frame (when q=0). constant.

    // "axis" = spatial joint axis (Mirtich Defn 9 p104). (expressed in local frame.) constant.
    // for prismatic: axis_top = zero;
    //                axis_bottom = unit vector along the joint axis.
    // for revolute: axis_top = unit vector along the rotation axis (u);
    //               axis_bottom = u cross d_vector.
    Vector3r axis_top;
    Vector3r axis_bottom;

    Vector3r d_vector;   // vector from the inboard joint pos to this link's COM. (local frame.) constant. set for revolute joints only.

    // e_vector is constant, but depends on the joint type
    // prismatic: vector from COM of parent to COM of this link, WHEN Q = 0. (local frame.)
    // revolute: vector from parent's COM to the pivot point, in PARENT's frame.
    Vector3r e_vector;

    bool is_revolute;   // true = revolute, false = prismatic

    Quaternionr cached_rot_parent_to_this;   // rotates vectors in parent frame to vectors in local frame
    Vector3r cached_r_vector;                // vector from COM of parent to COM of this link, in local frame.

    Vector3r applied_force;    // In WORLD frame
    Vector3r applied_torque;   // In WORLD frame
    real_t joint_torque;

    // ctor: set some sensible defaults
    Link();

    // routine to update cached_rot_parent_to_this and cached_r_vector
    void updateCache();
};

Multibody::Link::Link()
    : joint_pos(0),
      mass(1),
      parent(-1),
      zero_rot_parent_to_this(1, 0, 0, 0),
      is_revolute(false),
      cached_rot_parent_to_this(1, 0, 0, 0),
      joint_torque(0)
{
    inertia << 1, 1, 1;
    axis_top << 0, 0, 0;
    axis_bottom << 1, 0, 0;
    d_vector << 0, 0, 0;
    e_vector << 0, 0, 0;
    cached_r_vector << 0, 0, 0;
    applied_force << 0, 0, 0;
    applied_torque << 0, 0, 0;
}

void Multibody::Link::updateCache()
{
    if (is_revolute) {
        cached_rot_parent_to_this = Quaternionr(AngleAxisr(-joint_pos, axis_top)) * zero_rot_parent_to_this;
        cached_r_vector = cached_rot_parent_to_this * e_vector + d_vector;
    } else {
        // cached_rot_parent_to_this never changes, so no need to update
        cached_r_vector = e_vector + joint_pos * axis_bottom;
    }
}


//
// Various spatial helper functions
//

namespace {
    void SpatialTransform(const Matrix3r &rotation_matrix,  // rotates vectors in 'from' frame to vectors in 'to' frame
                          const Vector3r &displacement,     // vector from origin of 'from' frame to origin of 'to' frame, in 'to' coordinates
                          const Vector3r &top_in,       // top part of input vector
                          const Vector3r &bottom_in,    // bottom part of input vector
                          Vector3r &top_out,         // top part of output vector
                          Vector3r &bottom_out)      // bottom part of output vector
    {
        top_out = rotation_matrix * top_in;
        bottom_out = -displacement.cross(top_out) + rotation_matrix * bottom_in;
    }

    void InverseSpatialTransform(const Matrix3r &rotation_matrix,
                                 const Vector3r &displacement,
                                 const Vector3r &top_in,
                                 const Vector3r &bottom_in,
                                 Vector3r &top_out,
                                 Vector3r &bottom_out)
    {
        top_out = rotation_matrix.transpose() * top_in;
        bottom_out = rotation_matrix.transpose() * (bottom_in + displacement.cross(top_in));
    }

    real_t SpatialDotProduct(const Vector3r &a_top,
                            const Vector3r &a_bottom,
                            const Vector3r &b_top,
                            const Vector3r &b_bottom)
    {
        return a_bottom.dot(b_top) + a_top.dot(b_bottom);
    }
}


//
// Implementation of class Multibody
//

Multibody::Multibody(int n_links,
                     real_t mass,
                     const Vector3r &inertia,
                     bool fixed_base_,
                     bool can_sleep_)
    : num_links(n_links),
      base_quat(1, 0, 0, 0),
      base_mass(mass),
      base_inertia(inertia),
      links(n_links > 0 ? new Link[n_links] : 0),
      real_buf(6 + 2*n_links),
      vector_buf(2*n_links),
      matrix_buf(n_links + 1),
      fixed_base(fixed_base_),
      awake(true),
      can_sleep(can_sleep_),
      sleep_timer(0)
{
    base_pos << 0, 0, 0;
    base_force << 0, 0, 0;
    base_torque << 0, 0, 0;
}

Multibody::~Multibody()
{
    delete[] links;
}

void Multibody::setupPrismatic(int i,
                               real_t mass,
                               const Vector3r &inertia,
                               int parent,
                               const Quaternionr &rot_parent_to_this,
                               const Vector3r &joint_axis,
                               const Vector3r &r_vector_when_q_zero)
{
    links[i].mass = mass;
    links[i].inertia = inertia;
    links[i].parent = parent;
    links[i].zero_rot_parent_to_this = rot_parent_to_this;
    links[i].axis_top << 0, 0, 0;
    links[i].axis_bottom = joint_axis;
    links[i].e_vector = r_vector_when_q_zero;
    links[i].is_revolute = false;
    links[i].cached_rot_parent_to_this = rot_parent_to_this;
    links[i].updateCache();
}

void Multibody::setupRevolute(int i,
                              real_t mass,
                              const Vector3r &inertia,
                              int parent,
                              const Quaternionr &zero_rot_parent_to_this,
                              const Vector3r &joint_axis,
                              const Vector3r &parent_axis_position,
                              const Vector3r &my_axis_position)
{
    links[i].mass = mass;
    links[i].inertia = inertia;
    links[i].parent = parent;
    links[i].zero_rot_parent_to_this = zero_rot_parent_to_this;
    links[i].axis_top = joint_axis;
    links[i].axis_bottom = joint_axis.cross(my_axis_position);
    links[i].d_vector = my_axis_position;
    links[i].e_vector = parent_axis_position;
    links[i].is_revolute = true;
    links[i].updateCache();
}

int Multibody::getParent(int i) const
{
    return links[i].parent;
}

real_t Multibody::getLinkMass(int i) const
{
    return links[i].mass;
}

const Vector3r & Multibody::getLinkInertia(int i) const
{
    return links[i].inertia;
}

real_t Multibody::getJointPos(int i) const
{
    return links[i].joint_pos;
}

real_t Multibody::getJointVel(int i) const
{
    return real_buf[6 + i];
}

void Multibody::setJointPos(int i, real_t q)
{
    links[i].joint_pos = q;
    links[i].updateCache();
}

void Multibody::setJointVel(int i, real_t qdot)
{
    real_buf[6 + i] = qdot;
}

const Vector3r & Multibody::getRVector(int i) const
{
    return links[i].cached_r_vector;
}

const Quaternionr & Multibody::getParentToLocalRot(int i) const
{
    return links[i].cached_rot_parent_to_this;
}

Vector3r Multibody::localPosToWorld(int i, const Vector3r &local_pos) const
{
    Vector3r result = local_pos;
    while (i != -1) {
        // 'result' is in frame i. transform it to frame parent(i)
        result += getRVector(i);
        result = getParentToLocalRot(i).inverse() * result;
        i = getParent(i);
    }

    // 'result' is now in the base frame. transform it to world frame
    result = getWorldToBaseRot().inverse() * result;
    result += getBasePos();

    return result;
}

Vector3r Multibody::worldPosToLocal(int i, const Vector3r &world_pos) const
{
    if (i == -1) {
        // world to base
        return getWorldToBaseRot() * (world_pos - getBasePos());
    } else {
        // find position in parent frame, then transform to current frame
        return getParentToLocalRot(i) * worldPosToLocal(getParent(i), world_pos) - getRVector(i);
    }
}

Vector3r Multibody::localDirToWorld(int i, const Vector3r &local_dir) const
{
    Vector3r result = local_dir;
    while (i != -1) {
        result = getParentToLocalRot(i).inverse() * result;
        i = getParent(i);
    }
    result = getWorldToBaseRot().inverse() * result;
    return result;
}

Vector3r Multibody::worldDirToLocal(int i, const Vector3r &world_dir) const
{
    if (i == -1) {
        return getWorldToBaseRot() * world_dir;
    } else {
        return getParentToLocalRot(i) * worldDirToLocal(getParent(i), world_dir);
    }
}

void Multibody::compTreeLinkVelocities(Vector3r *omega, Vector3r *vel) const
{
    // Calculates the velocities of each link (and the base) in its local frame
    omega[0] = base_quat * getBaseOmega();
    vel[0] = base_quat * getBaseVel();
    
    for (int i = 0; i < num_links; ++i) {
        const int parent = links[i].parent;

        // transform parent vel into this frame, store in omega[i+1], vel[i+1]
        SpatialTransform(Matrix3r(links[i].cached_rot_parent_to_this), links[i].cached_r_vector,
                         omega[parent+1], vel[parent+1],
                         omega[i+1], vel[i+1]);

        // now add qidot * shat_i
        omega[i+1] += getJointVel(i) * links[i].axis_top;
        vel[i+1] += getJointVel(i) * links[i].axis_bottom;
    }
}

real_t Multibody::getKineticEnergy() const
{
    // TODO: would be better not to allocate memory here
    vector<Vector3r> omega(num_links+1), vel(num_links+1);
    compTreeLinkVelocities(&omega[0], &vel[0]);

    // we will do the factor of 0.5 at the end
    real_t result = base_mass * vel[0].dot(vel[0]);
    result += omega[0].dot(base_inertia.cwise() * omega[0]);
    
    for (int i = 0; i < num_links; ++i) {
        result += links[i].mass * vel[i+1].dot(vel[i+1]);
        result += omega[i+1].dot(links[i].inertia.cwise() * omega[i+1]);
    }

    return 0.5f * result;
}

Vector3r Multibody::getAngularMomentum() const
{
    // TODO: would be better not to allocate memory here
    vector<Vector3r> omega(num_links+1), vel(num_links+1);
    vector<Quaternionr> rot_from_world(num_links+1);
    compTreeLinkVelocities(&omega[0], &vel[0]);

    rot_from_world[0] = base_quat;
    Vector3r result = rot_from_world[0].inverse() * (base_inertia.cwise() * omega[0]);

    for (int i = 0; i < num_links; ++i) {
        rot_from_world[i+1] = links[i].cached_rot_parent_to_this * rot_from_world[links[i].parent+1];
        result += (rot_from_world[i+1].inverse() * (links[i].inertia.cwise() * omega[i+1]));
    }

    return result;
}


void Multibody::clearForcesAndTorques()
{
    base_force << 0, 0, 0;
    base_torque << 0, 0, 0;

    for (int i = 0; i < num_links; ++i) {
        links[i].applied_force << 0, 0, 0;
        links[i].applied_torque << 0, 0, 0;
        links[i].joint_torque = 0;
    }
}

void Multibody::addLinkForce(int i, const Vector3r &f)
{
    links[i].applied_force += f;
}

void Multibody::addLinkTorque(int i, const Vector3r &t)
{
    links[i].applied_torque += t;
}

void Multibody::addJointTorque(int i, real_t Q)
{
    links[i].joint_torque += Q;
}

const Vector3r & Multibody::getLinkForce(int i) const
{
    return links[i].applied_force;
}

const Vector3r & Multibody::getLinkTorque(int i) const
{
    return links[i].applied_torque;
}

real_t Multibody::getJointTorque(int i) const
{
    return links[i].joint_torque;
}

void Multibody::stepVelocities(real_t dt,
                               std::vector<real_t> &scratch_r,
                               std::vector<Vector3r> &scratch_v,
                               std::vector<Matrix3r> &scratch_m)
{
    // Implement Featherstone's algorithm to calculate joint accelerations (q_double_dot)
    // and the base linear & angular accelerations.

    // We apply damping forces in this routine as well as any external forces specified by the 
    // caller (via addBaseForce etc).

    // output should point to an array of 6 + num_links reals.
    // Format is: 3 angular accelerations (in world frame), 3 linear accelerations (in world frame),
    // num_links joint acceleration values.
    
    const real_t DAMPING_K1 = real_t(0.04);
    const real_t DAMPING_K2 = real_t(0);

    const Map<Vector3r> base_vel = getBaseVel();
    const Map<Vector3r> base_omega = getBaseOmega();

    // Temporary matrices/vectors -- use scratch space from caller
    // so that we don't have to keep reallocating every frame

    scratch_r.resize(2*num_links + 6);
    scratch_v.resize(8*num_links + 6);
    scratch_m.resize(4*num_links + 4);

    real_t * r_ptr = &scratch_r[0];
    real_t * output = &scratch_r[num_links];  // "output" holds the q_double_dot results
    Vector3r * v_ptr = &scratch_v[0];
    
    // vhat_i  (top = angular, bottom = linear part)
    Vector3r * vel_top = v_ptr; v_ptr += num_links + 1;
    Vector3r * vel_bottom = v_ptr; v_ptr += num_links + 1;

    // zhat_i^A
    Vector3r * zero_acc_top = v_ptr; v_ptr += num_links + 1;
    Vector3r * zero_acc_bottom = v_ptr; v_ptr += num_links + 1;

    // chat_i  (note NOT defined for the base)
    Vector3r * coriolis_top = v_ptr; v_ptr += num_links;
    Vector3r * coriolis_bottom = v_ptr; v_ptr += num_links;

    // top left, top right and bottom left blocks of Ihat_i^A.
    // bottom right block = transpose of top left block and is not stored.
    // Note: the top right and bottom left blocks are always symmetric matrices, but we don't make use of this fact currently.
    Matrix3r * inertia_top_left = &scratch_m[num_links + 1];
    Matrix3r * inertia_top_right = &scratch_m[2*num_links + 2];
    Matrix3r * inertia_bottom_left = &scratch_m[3*num_links + 3];

    // Cached 3x3 rotation matrices from parent frame to this frame.
    Matrix3r * rot_from_parent = &matrix_buf[0];
    Matrix3r * rot_from_world = &scratch_m[0];

    // hhat_i, ahat_i
    // hhat is NOT stored for the base (but ahat is)
    Vector3r * h_top = num_links > 0 ? &vector_buf[0] : 0;
    Vector3r * h_bottom = num_links > 0 ? &vector_buf[num_links] : 0;
    Vector3r * accel_top = v_ptr; v_ptr += num_links + 1;
    Vector3r * accel_bottom = v_ptr; v_ptr += num_links + 1;

    // Y_i, D_i
    real_t * Y = r_ptr; r_ptr += num_links;
    real_t * D = num_links > 0 ? &real_buf[6 + num_links] : 0;

    // ptr to the joint accel part of the output
    real_t * joint_accel = output + 6;


    // Start of the algorithm proper.
    
    // First 'upward' loop.
    // Combines CompTreeLinkVelocities and InitTreeLinks from Mirtich.

    rot_from_parent[0] = base_quat;

    vel_top[0] = rot_from_parent[0] * base_omega;
    vel_bottom[0] = rot_from_parent[0] * base_vel;

    if (fixed_base) {
        zero_acc_top[0] = zero_acc_bottom[0] = Vector3r::Zero();
    } else {
        zero_acc_top[0] = - (rot_from_parent[0] * (base_force 
                                                   - base_mass*(DAMPING_K1+DAMPING_K2*base_vel.norm())*base_vel));
        zero_acc_bottom[0] =
#ifdef INCLUDE_GYRO_TERM
            vel_top[0].cross( base_inertia.cwise() * vel_top[0] )
#endif
            - (rot_from_parent[0] * base_torque);
        zero_acc_bottom[0] += base_inertia.cwise() * vel_top[0] * (DAMPING_K1 + DAMPING_K2*vel_top[0].norm());
    }
    inertia_top_left[0] = Matrix3r::Zero();
    inertia_top_right[0] << base_mass, 0, 0,
                            0, base_mass, 0,
                            0, 0, base_mass;
    inertia_bottom_left[0] << base_inertia[0], 0, 0,
                              0, base_inertia[1], 0,
                              0, 0, base_inertia[2];

    rot_from_world[0] = rot_from_parent[0];

    for (int i = 0; i < num_links; ++i) {
        const int parent = links[i].parent;
        rot_from_parent[i+1] = links[i].cached_rot_parent_to_this;

        rot_from_world[i+1] = rot_from_parent[i+1] * rot_from_world[parent+1];
        
        // vhat_i = i_xhat_p(i) * vhat_p(i)
        SpatialTransform(rot_from_parent[i+1], links[i].cached_r_vector,
                         vel_top[parent+1], vel_bottom[parent+1],
                         vel_top[i+1], vel_bottom[i+1]);

        // we can now calculate chat_i
        // remember vhat_i is really vhat_p(i) (but in current frame) at this point
        coriolis_bottom[i] = vel_top[i+1].cross(vel_top[i+1].cross(links[i].cached_r_vector))
            + 2 * vel_top[i+1].cross(links[i].axis_bottom) * getJointVel(i);
        if (links[i].is_revolute) {
            coriolis_top[i] = vel_top[i+1].cross(links[i].axis_top) * getJointVel(i);
            coriolis_bottom[i] += (getJointVel(i) * getJointVel(i)) * links[i].axis_top.cross(links[i].axis_bottom);
        } else {
            coriolis_top[i] = Vector3r::Zero();
        }
        
        // now set vhat_i to its true value by doing
        // vhat_i += qidot * shat_i
        vel_top[i+1] += getJointVel(i) * links[i].axis_top;
        vel_bottom[i+1] += getJointVel(i) * links[i].axis_bottom;

        // calculate zhat_i^A
        zero_acc_top[i+1] = - (rot_from_world[i+1] * (links[i].applied_force));
        zero_acc_top[i+1] += links[i].mass * (DAMPING_K1 + DAMPING_K2*vel_bottom[i+1].norm()) * vel_bottom[i+1];

        zero_acc_bottom[i+1] =
#ifdef INCLUDE_GYRO_TERM
            vel_top[i+1].cross( links[i].inertia.cwise() * vel_top[i+1] )
#endif
            - (rot_from_world[i+1] * links[i].applied_torque);
        zero_acc_bottom[i+1] += links[i].inertia.cwise() * vel_top[i+1] * (DAMPING_K1 + DAMPING_K2*vel_top[i+1].norm());

        // calculate Ihat_i^A
        inertia_top_left[i+1] = Matrix3r::Zero();
        inertia_top_right[i+1] << links[i].mass, 0, 0,
                                  0, links[i].mass, 0,
                                  0, 0, links[i].mass;
        inertia_bottom_left[i+1] << links[i].inertia[0], 0, 0,
                                    0, links[i].inertia[1], 0,
                                    0, 0, links[i].inertia[2];
    }


    // 'Downward' loop.
    // (part of TreeForwardDynamics in Mirtich.)
    for (int i = num_links - 1; i >= 0; --i) {

        h_top[i] = inertia_top_left[i+1] * links[i].axis_top + inertia_top_right[i+1] * links[i].axis_bottom;
        h_bottom[i] = inertia_bottom_left[i+1] * links[i].axis_top + inertia_top_left[i+1].transpose() * links[i].axis_bottom;

        D[i] = SpatialDotProduct(links[i].axis_top, links[i].axis_bottom, h_top[i], h_bottom[i]);
        Y[i] = links[i].joint_torque
            - SpatialDotProduct(links[i].axis_top, links[i].axis_bottom, zero_acc_top[i+1], zero_acc_bottom[i+1])
            - SpatialDotProduct(h_top[i], h_bottom[i], coriolis_top[i], coriolis_bottom[i]);

        const int parent = links[i].parent;

        
        // Ip += pXi * (Ii - hi hi' / Di) * iXp
        const real_t one_over_di = 1.0f / D[i];
        const Matrix3r TL = inertia_top_left[i+1] - (one_over_di * h_top[i] * h_bottom[i].transpose());
        const Matrix3r TR = inertia_top_right[i+1] - (one_over_di * h_top[i] * h_top[i].transpose());
        const Matrix3r BL = inertia_bottom_left[i+1] - (one_over_di * h_bottom[i] * h_bottom[i].transpose());

        Matrix3r r_cross;
        r_cross <<
            0, -links[i].cached_r_vector[2], links[i].cached_r_vector[1],
            links[i].cached_r_vector[2], 0, -links[i].cached_r_vector[0],
            -links[i].cached_r_vector[1], links[i].cached_r_vector[0], 0;
        
        inertia_top_left[parent+1] += rot_from_parent[i+1].transpose() * ( TL - TR * r_cross ) * rot_from_parent[i+1];
        inertia_top_right[parent+1] += rot_from_parent[i+1].transpose() * TR * rot_from_parent[i+1];
        inertia_bottom_left[parent+1] += rot_from_parent[i+1].transpose() *
            (r_cross * (TL - TR * r_cross) + BL - TL.transpose() * r_cross) * rot_from_parent[i+1];
        
        
        // Zp += pXi * (Zi + Ii*ci + hi*Yi/Di)
        Vector3r in_top, in_bottom, out_top, out_bottom;
        const real_t Y_over_D = Y[i] * one_over_di;
        in_top = zero_acc_top[i+1]
            + inertia_top_left[i+1] * coriolis_top[i]
            + inertia_top_right[i+1] * coriolis_bottom[i]
            + Y_over_D * h_top[i];
        in_bottom = zero_acc_bottom[i+1]
            + inertia_bottom_left[i+1] * coriolis_top[i]
            + inertia_top_left[i+1].transpose() * coriolis_bottom[i]
            + Y_over_D * h_bottom[i];
        InverseSpatialTransform(rot_from_parent[i+1], links[i].cached_r_vector,
                                in_top, in_bottom, out_top, out_bottom);
        zero_acc_top[parent+1] += out_top;
        zero_acc_bottom[parent+1] += out_bottom;
    }


    // Second 'upward' loop
    // (part of TreeForwardDynamics in Mirtich)

    if (fixed_base) {
        accel_top[0] = accel_bottom[0] = Vector3r::Zero();
    } else {
        if (num_links > 0) {
            // Here we have to do a 6x6 matrix inversion. There might be faster ways of doing this
            // (taking advantage of the structure of Ihat_1^A) but here I just explicitly create the
            // 6x6 matrix then use the LU solver.
            Matrix<real_t, 6, 6> Imatrix;
            Imatrix.block<3,3>(0,0) = inertia_top_left[0];
            Imatrix.block<3,3>(3,0) = inertia_bottom_left[0];
            Imatrix.block<3,3>(0,3) = inertia_top_right[0];
            Imatrix.block<3,3>(3,3) = inertia_top_left[0].transpose();
            cached_imatrix_lu.reset(new Eigen::LU<Matrix<real_t, 6, 6> >(Imatrix));  // TODO: Avoid memory allocation here?
        }
        Matrix<real_t, 6, 1> rhs;
        rhs << zero_acc_top[0][0], zero_acc_top[0][1], zero_acc_top[0][2],
            zero_acc_bottom[0][0], zero_acc_bottom[0][1], zero_acc_bottom[0][2];
        Matrix<real_t, 6, 1> result;
        solveImatrix(rhs, result);
        for (int i = 0; i < 3; ++i) {
            accel_top[0][i] = -result[i];
            accel_bottom[0][i] = -result[i+3];
        }
    }

    // now do the loop over the links
    for (int i = 0; i < num_links; ++i) {
        const int parent = links[i].parent;
        SpatialTransform(rot_from_parent[i+1], links[i].cached_r_vector,
                         accel_top[parent+1], accel_bottom[parent+1],
                         accel_top[i+1], accel_bottom[i+1]);
        joint_accel[i] = (Y[i] - SpatialDotProduct(h_top[i], h_bottom[i], accel_top[i+1], accel_bottom[i+1])) / D[i];
        accel_top[i+1] += coriolis_top[i] + joint_accel[i] * links[i].axis_top;
        accel_bottom[i+1] += coriolis_bottom[i] + joint_accel[i] * links[i].axis_bottom;
    }

    // transform base accelerations back to the world frame.
    Map<Vector3r> omegadot_out(output);
    omegadot_out = rot_from_parent[0].transpose() * accel_top[0];

    Map<Vector3r> vdot_out(output + 3);
    vdot_out = rot_from_parent[0].transpose() * accel_bottom[0];

    // Final step: add the accelerations (times dt) to the velocities.
    applyDeltaVee(output, dt);
}

void Multibody::solveImatrix(const Matrix<real_t, 6, 1> &rhs, Matrix<real_t, 6, 1> &result) const
{
    // this does the equivalent of "cached_imatrix_lu->solve(rhs, &result)"
    // except that in the case of 0 links (i.e. a plain rigid body, not a multibody)
    // we can optimize -- don't have to use the LU solver.

    if (num_links == 0) {
        result[0] = rhs[3] / base_inertia[0];
        result[1] = rhs[4] / base_inertia[1];
        result[2] = rhs[5] / base_inertia[2];
        result[3] = rhs[0] / base_mass;
        result[4] = rhs[1] / base_mass;
        result[5] = rhs[2] / base_mass;
    } else {
        cached_imatrix_lu->solve(rhs, &result);
    }
}

void Multibody::calcAccelerationDeltas(const real_t *force, real_t *output,
                                       vector<real_t> &scratch_r, vector<Vector3r> &scratch_v) const
{
    // Temporary matrices/vectors -- use scratch space from caller
    // so that we don't have to keep reallocating every frame

    scratch_r.resize(num_links);
    scratch_v.resize(4*num_links + 4);

    real_t * r_ptr = num_links == 0 ? 0 : &scratch_r[0];
    Vector3r * v_ptr = &scratch_v[0];
    
    // zhat_i^A (scratch space)
    Vector3r * zero_acc_top = v_ptr; v_ptr += num_links + 1;
    Vector3r * zero_acc_bottom = v_ptr; v_ptr += num_links + 1;

    // rot_from_parent (cached from calcAccelerations)
    const Matrix3r * rot_from_parent = &matrix_buf[0];

    // hhat (cached), accel (scratch)
    const Vector3r * h_top = num_links > 0 ? &vector_buf[0] : 0;
    const Vector3r * h_bottom = num_links > 0 ? &vector_buf[num_links] : 0;
    Vector3r * accel_top = v_ptr; v_ptr += num_links + 1;
    Vector3r * accel_bottom = v_ptr; v_ptr += num_links + 1;

    // Y_i (scratch), D_i (cached)
    real_t * Y = r_ptr; r_ptr += num_links;
    const real_t * D = num_links > 0 ? &real_buf[6 + num_links] : 0;

    assert(num_links == 0 || r_ptr - &scratch_r[0] == scratch_r.size());
    assert(v_ptr - &scratch_v[0] == scratch_v.size());


    
    // First 'upward' loop.
    // Combines CompTreeLinkVelocities and InitTreeLinks from Mirtich.

    const Map<Vector3r> input_force(force + 3);
    const Map<Vector3r> input_torque(force);
    
    // Fill in zero_acc
    // -- set to force/torque on the base, zero otherwise
    if (fixed_base) {
        zero_acc_top[0] = zero_acc_bottom[0] = Vector3r::Zero();
    } else {
        zero_acc_top[0] = - (rot_from_parent[0] * input_force);
        zero_acc_bottom[0] =  - (rot_from_parent[0] * input_torque);
    }
    for (int i = 0; i < num_links; ++i) {
        zero_acc_top[i+1] = zero_acc_bottom[i+1] = Vector3r::Zero();
    }

    // 'Downward' loop.
    for (int i = num_links - 1; i >= 0; --i) {

        Y[i] = - SpatialDotProduct(links[i].axis_top, links[i].axis_bottom, zero_acc_top[i+1], zero_acc_bottom[i+1]);
        Y[i] += force[6 + i];  // add joint torque
        
        const int parent = links[i].parent;
        
        // Zp += pXi * (Zi + hi*Yi/Di)
        Vector3r in_top, in_bottom, out_top, out_bottom;
        const real_t Y_over_D = Y[i] / D[i];
        in_top = zero_acc_top[i+1] + Y_over_D * h_top[i];
        in_bottom = zero_acc_bottom[i+1] + Y_over_D * h_bottom[i];
        InverseSpatialTransform(rot_from_parent[i+1], links[i].cached_r_vector,
                                in_top, in_bottom, out_top, out_bottom);
        zero_acc_top[parent+1] += out_top;
        zero_acc_bottom[parent+1] += out_bottom;
    }

    // ptr to the joint accel part of the output
    real_t * joint_accel = output + 6;

    // Second 'upward' loop
    if (fixed_base) {
        accel_top[0] = accel_bottom[0] = Vector3r::Zero();
    } else {
        Matrix<real_t, 6, 1> rhs;
        rhs << zero_acc_top[0][0], zero_acc_top[0][1], zero_acc_top[0][2],
            zero_acc_bottom[0][0], zero_acc_bottom[0][1], zero_acc_bottom[0][2];
        Matrix<real_t, 6, 1> result;
        solveImatrix(rhs, result);
        for (int i = 0; i < 3; ++i) {
            accel_top[0][i] = -result[i];
            accel_bottom[0][i] = -result[i+3];
        }
    }
    
    // now do the loop over the links
    for (int i = 0; i < num_links; ++i) {
        const int parent = links[i].parent;
        SpatialTransform(rot_from_parent[i+1], links[i].cached_r_vector,
                         accel_top[parent+1], accel_bottom[parent+1],
                         accel_top[i+1], accel_bottom[i+1]);
        joint_accel[i] = (Y[i] - SpatialDotProduct(h_top[i], h_bottom[i], accel_top[i+1], accel_bottom[i+1])) / D[i];
        accel_top[i+1] += joint_accel[i] * links[i].axis_top;
        accel_bottom[i+1] += joint_accel[i] * links[i].axis_bottom;
    }

    // transform base accelerations back to the world frame.
    Map<Vector3r> omegadot_out(output);
    omegadot_out = rot_from_parent[0].transpose() * accel_top[0];

    Map<Vector3r> vdot_out(output + 3);
    vdot_out = rot_from_parent[0].transpose() * accel_bottom[0];

}

void Multibody::stepPositions(real_t dt)
{
    // step position by adding dt * velocity
    base_pos += dt * getBaseVel();

    // "exponential map" method for the rotation
    const Map<Vector3r> base_omega = getBaseOmega();
    const real_t omega_norm = base_omega.norm();
    const real_t omega_times_dt = omega_norm * dt;
    const real_t SMALL_ROTATION_ANGLE = 0.02f; // Theoretically this should be ~ pow(FLT_EPSILON,0.25) which is ~ 0.0156
    if (fabs(omega_times_dt) < SMALL_ROTATION_ANGLE) {
        const real_t xsq = omega_times_dt * omega_times_dt;     // |omega|^2 * dt^2
        const real_t sin_term = dt * (xsq / 48.0f - 0.5f);      // -sin(0.5*dt*|omega|) / |omega|
        const real_t cos_term = 1.0f - xsq / 8.0f;              // cos(0.5*dt*|omega|)
        base_quat = base_quat * Quaternionr(cos_term,
            sin_term * base_omega[0],
            sin_term * base_omega[1],
            sin_term * base_omega[2]);
    } else {
        base_quat = base_quat * Quaternionr(AngleAxisr(-omega_times_dt, base_omega / omega_norm));
    }

    // Make sure the quaternion represents a valid rotation.
    // (Not strictly necessary, but helps prevent any round-off errors from building up.)
    base_quat.normalize();

    // Finally we can update joint_pos for each of the links
    for (int i = 0; i < num_links; ++i) {
        links[i].joint_pos += dt * getJointVel(i);
        links[i].updateCache();
    }
}

void Multibody::fillContactJacobian(int link,
                                    const Vector3r &contact_point,
                                    const Vector3r &normal,
                                    real_t *jac,
                                    vector<real_t> &scratch_r,
                                    vector<Vector3r> &scratch_v,
                                    vector<Matrix3r> &scratch_m) const
{
    // temporary space
    scratch_v.resize(2*num_links + 2);
    scratch_m.resize(num_links + 1);

    Vector3r * v_ptr = &scratch_v[0];
    Vector3r * p_minus_com = v_ptr; v_ptr += num_links + 1;
    Vector3r * n_local = v_ptr; v_ptr += num_links + 1;
    assert(v_ptr - &scratch_v[0] == scratch_v.size());

    scratch_r.resize(num_links);
    real_t * results = num_links > 0 ? &scratch_r[0] : 0;

    Matrix3r * rot_from_world = &scratch_m[0];

    const Vector3r p_minus_com_world = contact_point - base_pos;

    rot_from_world[0] = Matrix3r(base_quat);

    p_minus_com[0] = rot_from_world[0] * p_minus_com_world;
    n_local[0] = rot_from_world[0] * normal;
    
    // omega coeffients first.
    Map<Vector3r> omega_coeffs(jac);
    omega_coeffs = p_minus_com_world.cross(normal);

    // then v coefficients
    jac[3] = normal[0];
    jac[4] = normal[1];
    jac[5] = normal[2];

    // Set remaining jac values to zero for now.
    for (int i = 6; i < 6 + num_links; ++i) {
        jac[i] = 0;
    }

    // Qdot coefficients, if necessary.
    if (num_links > 0 && link > -1) {

        // TODO: speed this up -- don't calculate for links we don't need.
        // (Also, we are making 3 separate calls to this function, for the normal & the 2 friction directions,
        // which is resulting in repeated work being done...)

        // calculate required normals & positions in the local frames.
        for (int i = 0; i < num_links; ++i) {

            // transform to local frame
            const int parent = links[i].parent;
            const Matrix3r mtx(links[i].cached_rot_parent_to_this);
            rot_from_world[i+1] = mtx * rot_from_world[parent+1];

            n_local[i+1] = mtx * n_local[parent+1];
            p_minus_com[i+1] = mtx * p_minus_com[parent+1] - links[i].cached_r_vector;

            // calculate the jacobian entry
            if (links[i].is_revolute) {
                results[i] = n_local[i+1].dot( links[i].axis_top.cross(p_minus_com[i+1]) + links[i].axis_bottom );
            } else {
                results[i] = n_local[i+1].dot( links[i].axis_bottom );
            }
        }

        // Now copy through to output.
        while (link != -1) {
            jac[6 + link] = results[link];
            link = links[link].parent;
        }
    }
}

void Multibody::wakeUp()
{
    awake = true;
}

void Multibody::goToSleep()
{
    awake = false;
}

void Multibody::checkMotionAndSleepIfRequired(real_t timestep)
{
    if (!can_sleep) return;

    // motion is computed as omega^2 + v^2 + (sum of squares of joint velocities)
    real_t motion = 0;
    for (int i = 0; i < 6 + num_links; ++i) {
        motion += real_buf[i] * real_buf[i];
    }

    if (motion < SLEEP_EPSILON) {
        sleep_timer += timestep;
        if (sleep_timer > SLEEP_TIMEOUT) {
            goToSleep();
        }
    } else {
        sleep_timer = 0;
    }
}
