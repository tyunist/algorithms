/*
 * FILE:
 *   bridge_constraint.cpp
 *
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   27-Jul-2011
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

#include "bridge_constraint.hpp"
#include "multibody.hpp"

BridgeConstraint::BridgeConstraint(Multibody *body,
                                   const Vector3r &con_pos,
                                   const Vector3r &pivot_offset,
                                   const Vector3r &endpoint_offset)
    : Constraint(false, false,
                 body, 0, body->getNumLinks()-1, -1,
                 2,    // number of rows
                 0,    // friction (N/A)
                 0),   // restitution (N/A)
      bridge_con_pos(con_pos),
      bridge_pivot_offset(pivot_offset),
      bridge_endpoint_offset(endpoint_offset)
{
}

void BridgeConstraint::update(std::vector<real_t> &,
                              std::vector<Vector3r> &link_com_pos,
                              std::vector<Matrix3r> &world_to_link_rot)
{
    Multibody *bod = getBody1();
    const int n_links = bod->getNumLinks();

    world_to_link_rot.resize(n_links + 1);   // rotation matrix: world to local frame
    link_com_pos.resize(n_links + 1);   // position of local frame COM in world coords

    // Compute rotation matrices and COM positions
    world_to_link_rot[0] = bod->getWorldToBaseRot();
    link_com_pos[0] = bod->getBasePos();

    for (int i = 0; i < n_links; ++i) {
        world_to_link_rot[i+1] = Matrix3r(bod->getParentToLocalRot(i)) * world_to_link_rot[i];
        link_com_pos[i+1] = link_com_pos[i] + world_to_link_rot[i+1].transpose() * bod->getRVector(i);
    }

    // Compute the endpoint of the final link (in world coords)
    const Vector3r endpoint = link_com_pos[n_links] + world_to_link_rot[n_links].transpose() * bridge_endpoint_offset;

    // For each link
    for (int i = 0; i < n_links; ++i) {

        // Calculate the pivot point (in world coords)
        const Vector3r pivot_point = link_com_pos[i+1] + world_to_link_rot[i+1].transpose() * bridge_pivot_offset;

        // Calculate the vector from the pivot point to the endpoint (in world coords)
        const Vector3r endpt_ofs = endpoint - pivot_point;

        // The Jacobian is then (y, -z)
        jacobian1(0)[6+i] = endpt_ofs[1];
        jacobian1(1)[6+i] = -endpt_ofs[2];
    }

    // Compute pos_error = Vector from where the endpoint SHOULD BE, to where it IS, in world coords
    const Vector3r pos_error = bridge_con_pos - endpoint;

    // Constraint position is just the (-z, -y) components of this
    setPosition(0, -pos_error[2]);
    setPosition(1, -pos_error[1]);
}
