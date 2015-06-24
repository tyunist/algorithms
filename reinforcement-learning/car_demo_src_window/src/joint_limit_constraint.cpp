/*
 * FILE: 
 *   joint_limit_constraint.cpp
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

#include "joint_limit_constraint.hpp"
#include "multibody.hpp"

JointLimitConstraint::JointLimitConstraint(Multibody *body, int link,
                                           real_t lower_bound_,
                                           real_t upper_bound_)
    : Constraint(true, false, body, 0, link, -1,
                 2,  // number of rows
                 0,  // friction (irrelevant)
                 0),  // restitution (off by default -- caller can still call setRestitutionCoeff if they want)
      lower_bound(lower_bound_),
      upper_bound(upper_bound_)
{
    // the jacobians never change, so may as well
    // initialize them here
        
    // note: we rely on the fact that jacobians are
    // always initialized to zero by the Constraint ctor

    // row 0: the lower bound
    jacobian1(0)[6 + link] = 1;

    // row 1: the upper bound
    jacobian1(1)[6 + link] = -1;
}

void JointLimitConstraint::update(std::vector<real_t> &scratch_r,
                                  std::vector<Vector3r> &scratch_v,
                                  std::vector<Matrix3r> &scratch_m)
{
    Multibody *body = getBody1();
    const int link = getLink1();

    // only positions need to be updated -- jacobians and force
    // directions were set in the ctor and never change.
    
    // row 0: the lower bound
    setPosition(0, body->getJointPos(link) - lower_bound);

    // row 1: the upper bound
    setPosition(1, upper_bound - body->getJointPos(link));
}
