/*
 * FILE:
 *   road_builder.hpp
 *
 * PURPOSE:
 *   This is an experimental class for generating road surface
 *   geometry. It ended up not being used in the final demo.
 *   
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   08-Oct-2010
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

#ifndef ROAD_BUILDER_HPP
#define ROAD_BUILDER_HPP

#include "real.hpp"

#include <Eigen/StdVector>
#include <vector>

class RoadBuilder {
public:
    // start_angle = initial heading of road
    // (0 = +z direction, PI/2 = +x direction)
    explicit RoadBuilder(const Vector3r & start_point, real_t start_angle);

    // Note: The input points correspond to the centre of the road object.
    // The upper road surface will be above this by ROAD_HEIGHT/2.
    void road(const Vector3r &pos, real_t turn_radius);

    void loop(real_t radius, real_t screw);

    const std::vector<Vector3r> & getLeftPoints() const { return left_points; }
    const std::vector<Vector3r> & getRightPoints() const { return right_points; }
    
private:
    std::vector<Vector3r> left_points, right_points;
    
    Vector3r curr_pos;
    real_t theta;
};

#endif
