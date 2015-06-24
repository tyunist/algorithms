/*
 * FILE:
 *   wheel_contact_constraint.hpp
 *
 * PURPOSE:
 *   Version of ContactConstraint specially adapted for car wheels.
 *   Some adjustments -- some might say "hacks" -- are applied to
 *   ensure that the contact normal is always "smooth", and therefore
 *   no unexpected bumps or jumps occur while driving. At least, that
 *   is the intention :)
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

#ifndef WHEEL_CONTACT_CONSTRAINT_HPP
#define WHEEL_CONTACT_CONSTRAINT_HPP

#include "contact_constraint.hpp"

class WheelContactConstraint : public ContactConstraint {
public:
    WheelContactConstraint(Multibody *bod1, Multibody *bod2,
                           int lnk1, int lnk2,
                           const Vector3r &cp1,
                           const Vector3r &cp2,
                           const Vector3r &n,
                           real_t fric,
                           real_t rest,
                           const Vector3r &axis_,
                           real_t radius_);

    virtual void update(std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v,
                        std::vector<Matrix3r> &scratch_m);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
private:
    Vector3r axis;
    real_t radius;
};

#endif
