/*
 * FILE:
 *   sequential_impulses.hpp
 *   
 * PURPOSE:
 *   The constraint solver. Given a set of constraints, we compute the
 *   impulses that are required to enforce the constraints, and apply
 *   them to the relevant bodies. The standard "sequential impulses"
 *   method is used.
 *   
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   05-Aug-2010
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

#ifndef SEQUENTIAL_IMPULSES_HPP
#define SEQUENTIAL_IMPULSES_HPP

#include "real.hpp"
#include <Eigen/StdVector>
#include <vector>

class Constraint;
class Multibody;

// Calculates "rhs" vector for Sequential Impulses algorithm.
// To reduce "vibration" this should be called BEFORE the velocity stepping.

void CalculateBVector(const Constraint * const *constraints,
                      int num_constraints,
                      real_t dt,
                      std::vector<real_t> &Bvector);

// Applies impulses to enforce J v >= 0 or J v = 0 as appropriate.
// Precondition: update() has been called on each constraint.

void SequentialImpulses(Constraint **constraints,
                        int num_constraints,
                        real_t dt,
                        const std::vector<real_t> &Bvector,
                        std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v);

#endif
