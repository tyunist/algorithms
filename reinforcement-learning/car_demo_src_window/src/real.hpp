/*
 * FILE:
 *   real.hpp
 *
 * PURPOSE:
 *   Define "real_t" as typedef for either float or double. By default
 *   we use float, but you can uncomment "#define
 *   USE_DOUBLE_PRECISION" (below) to use double instead.
 *   
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   03-Sep-2010
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

#ifndef REAL_HPP
#define REAL_HPP

//#define USE_DOUBLE_PRECISION

#ifdef USE_DOUBLE_PRECISION
typedef double real_t;
#else
typedef float real_t;
#endif

#include <Eigen/Core>
typedef Eigen::Matrix<real_t, 3, 1> Vector3r;
typedef Eigen::Matrix<real_t, 3, 3> Matrix3r;
typedef Eigen::Quaternion<real_t> Quaternionr;
typedef Eigen::AngleAxis<real_t> AngleAxisr;

#endif
