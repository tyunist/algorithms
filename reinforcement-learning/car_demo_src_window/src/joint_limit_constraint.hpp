/*
 * FILE: 
 *   joint_limit_constraint.hpp
 *
 * PURPOSE:
 *   Subclass of Constraint that constrains a given "joint angle" to
 *   be between given limits.
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

#ifndef JOINT_LIMIT_CONSTRAINT_HPP
#define JOINT_LIMIT_CONSTRAINT_HPP

#include "constraint.hpp"

// Constrain a joint angle to be between given limits.

// Note: currently, this is always set up as two unilateral
// constraints. If the upper and lower bounds are equal, this should
// be equivalent to a single bilateral constraint (albeit less
// efficient).

class JointLimitConstraint : public Constraint {
public:
    JointLimitConstraint(Multibody *body, int link,
                         real_t lower_bound_,
                         real_t upper_bound_);

    virtual void update(std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v,
                        std::vector<Matrix3r> &scratch_m);

    void setJointLimits(real_t lower, real_t upper) {
        lower_bound = lower;
        upper_bound = upper;
    }    
    
private:
    real_t lower_bound;
    real_t upper_bound;
};

#endif
