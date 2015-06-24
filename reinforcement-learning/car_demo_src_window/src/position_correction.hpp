/*
 * FILE:
 *   position_correction.hpp
 *
 * PURPOSE:
 *   Position correction. NOT USED CURRENTLY.
 *   
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   25-Jul-2011
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

#ifndef POSITION_CORRECTION_HPP
#define POSITION_CORRECTION_HPP

#include "real.hpp"
#include <Eigen/StdVector>
#include <vector>

class Constraint;
class Multibody;

// Applies delta-positions to enforce pos >= 0 or pos == 0 as
// appropriate.

// Does NOT assume constraints have been updated on entry, and
// constraints are NOT updated on exit.

void PositionCorrection(Constraint **constraints,
                        int num_constraints,
                        std::vector<real_t> &scratch_r,
                        std::vector<Vector3r> &scratch_v,
                        std::vector<Matrix3r> &scratch_m);

#endif
