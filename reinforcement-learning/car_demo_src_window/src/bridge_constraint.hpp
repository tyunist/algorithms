/*
 * FILE:
 *   bridge_constraint.hpp
 *
 * PURPOSE:
 *   An ad hoc constraint to make sure the last "plank" of the bridge
 *   is attached to the far pillar.
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

#ifndef BRIDGE_CONSTRAINT_HPP
#define BRIDGE_CONSTRAINT_HPP

#include "constraint.hpp"

class BridgeConstraint : public Constraint {
public:
    BridgeConstraint(Multibody *bridge_body,
                     const Vector3r & con_pos,   // Where the bridge endpoint is constrained to be
                     const Vector3r & pivot_offset,      // Vector from COM of each plank to its parent pivot point (in local frame)
                     const Vector3r & endpoint_offset);  // Vector from COM of final plank to endpoint of final plank (in local frame)

    virtual void update(std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v,
                        std::vector<Matrix3r> &scratch_m);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
private:
    Vector3r bridge_con_pos;
    Vector3r bridge_pivot_offset;
    Vector3r bridge_endpoint_offset;
};

#endif
