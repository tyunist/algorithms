/*
 * FILE:
 *   wheel_contact_constraint.cpp
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

#include "multibody.hpp"
#include "wheel_contact_constraint.hpp"

WheelContactConstraint::WheelContactConstraint(Multibody *bod1, Multibody *bod2,
                                               int lnk1, int lnk2,
                                               const Vector3r &cp1,
                                               const Vector3r &cp2,
                                               const Vector3r &n,
                                               real_t fric,
                                               real_t rest,
                                               const Vector3r &axis_,
                                               real_t radius_)
  : ContactConstraint(bod1, bod2, lnk1, lnk2,
                      cp1, cp2, n,
                      fric, rest),
    axis(axis_),
    radius(radius_)
{
}

void WheelContactConstraint::update(std::vector<real_t> &scratch_r,
                                    std::vector<Vector3r> &scratch_v,
                                    std::vector<Matrix3r> &scratch_m)
{
    const Multibody *body1 = getBody1();
    const int link1 = getLink1();
    const Multibody *body2 = getBody2();
    const int link2 = getLink2();

    // We assume the wheel is always the first body.
    Vector3r n1;
    if (body2) {
        n1 = body1->worldDirToLocal(link1, body2->localDirToWorld(link2, normal));
    } else {
        n1 = body1->worldDirToLocal(link1, normal);
    }
    
    // adjust the wheel contact point (cp1)
    contact_point_1 = 
        contact_point_1.dot(axis) * axis +
        (n1 - n1.dot(axis) * axis).normalized() * radius;

    // now call the parent class
    ContactConstraint::update(scratch_r, scratch_v, scratch_m);
}
