/*
 * FILE:
 *   contact_constraint.cpp
 *
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   10-Aug-2010
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

#include "contact_constraint.hpp"
#include "multibody.hpp"

namespace {
    const real_t ALLOWED_PENETRATION = real_t(0.01);
    
    void MakeFrictionBasis(const Multibody &bod,
                           const Vector3r &normal,
                           Vector3r &fric1, 
                           Vector3r &fric2)
    {
        // use body z direction as the first friction direction.
        fric1 = bod.localDirToWorld(-1, Vector3r(0,0,1));

        // make it orthogonal to normal
        fric1 -= fric1.dot(normal) * normal;

        real_t len = fric1.norm();
        if (len < real_t(0.2)) {
            // Oops, vector is too parallel to the normal.
            // We'll use the y diretion instead
            fric1 = bod.localDirToWorld(-1, Vector3r(0,1,0));
            fric1 -= fric1.dot(normal) * normal;
            len = fric1.norm();
        }

        fric1 /= len;

        // now fric2 is just the cross product of the other two
        fric2 = fric1.cross(normal);
    }
}

ContactConstraint::ContactConstraint(Multibody *bod1, Multibody *bod2,
                                     int lnk1, int lnk2,
                                     const Vector3r &contact_point_1_,
                                     const Vector3r &contact_point_2_,
                                     const Vector3r &normal_,
                                     real_t friction_coeff,
                                     real_t restitution_coeff)
    : Constraint(true, true, bod1, bod2, lnk1, lnk2, 3, friction_coeff, restitution_coeff),
      contact_point_1(contact_point_1_),
      contact_point_2(contact_point_2_),
      normal(normal_)
{
}

void ContactConstraint::update(std::vector<real_t> &scratch_r,
                               std::vector<Vector3r> &scratch_v,
                               std::vector<Matrix3r> &scratch_m)
{
    Multibody *const bod1 = getBody1();
    Multibody *const bod2 = getBody2();
    const int link1 = getLink1();
    const int link2 = getLink2();
    
    // Convert local points back to world
    const Vector3r world_cp1 = bod1->localPosToWorld(link1, contact_point_1);

    Vector3r world_cp2, world_normal;
    if (bod2) {
        world_cp2 = bod2->localPosToWorld(link2, contact_point_2);
        world_normal = bod2->localDirToWorld(link2, normal);
    } else {
        world_cp2 = contact_point_2;
        world_normal = normal;
    }

    // Position
    setPosition(0, (world_cp2 - world_cp1).dot(world_normal) + ALLOWED_PENETRATION);

    // Jacobians for the normal reaction row
    // NOTE: We apply the force at contact point 1, on BOTH bodies.
    bod1->fillContactJacobian(link1, world_cp1, -world_normal, jacobian1(0), scratch_r, scratch_v, scratch_m);
    if (bod2) {
        bod2->fillContactJacobian(link2, world_cp1, world_normal, jacobian2(0), scratch_r, scratch_v, scratch_m);
    }
    
    // Make a friction basis
    // TODO: If there are two bodies then ideally we should pick one of the two by some rule,
    // instead of just choosing whichever one happens to be body 1.
    Vector3r world_fric1, world_fric2;
    MakeFrictionBasis(*bod1, world_normal, world_fric1, world_fric2);
    
    // Jacobians for friction rows
    bod1->fillContactJacobian(link1, world_cp1, -world_fric1,  jacobian1(1), scratch_r, scratch_v, scratch_m);
    bod1->fillContactJacobian(link1, world_cp1, -world_fric2,  jacobian1(2), scratch_r, scratch_v, scratch_m);
    if (bod2) {
        bod2->fillContactJacobian(link2, world_cp1, world_fric1,  jacobian2(1), scratch_r, scratch_v, scratch_m);
        bod2->fillContactJacobian(link2, world_cp1, world_fric2,  jacobian2(2), scratch_r, scratch_v, scratch_m);
    }
    
#ifdef CONTACT_FORCE_DEBUGGING
    gfx_contact_point = world_cp1;
    gfx_normal = world_normal;
#endif
}
