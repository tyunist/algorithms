/*
 * FILE:
 *   road_builder.cpp
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

#include "game_world.hpp"
#include "road_builder.hpp"

namespace {
    const real_t PI = 4*atan(real_t(1));

    const real_t ROAD_WIDTH = 16;
    const real_t ROAD_HEIGHT = real_t(0.2);
    const real_t TURN_INCREMENT = (PI / 180) * 5;

    const real_t ROAD_CHUNK_LENGTH = 50;
}

namespace {
    int SolveForTurnAngle(real_t sigma_dx, real_t dz, real_t turn_radius)
    {
        // This works out the angle we have to turn through (at the given turn_radius)
        // so that we end up facing directly towards our destination.
        //
        // To do this we have to solve
        // (sigma*delta_x - turn_radius)*cos(angle) - delta_z*sin(angle) + turn_radius = 0.
        // 
        // Return value is angle/TURN_INCREMENT, rounded to an integer.
        
        // brute force method...
        real_t previous_value = sigma_dx;
        
        for (int i = 1; i < int(2*PI/TURN_INCREMENT); ++i) {
            const real_t angle = i*TURN_INCREMENT;
            const real_t this_value = (sigma_dx - turn_radius)*cos(angle) - dz*sin(angle) + turn_radius;
            if (this_value * previous_value <= 0) {
                // Sign changed
                if (fabs(this_value) < fabs(previous_value)) {
                    // This value is closer to zero
                    return i;
                } else {
                    // Previous value is closer
                    return i-1;
                }
            }
            previous_value = this_value;
        }

        // No solution was found. Return zero, this means we will just
        // plonk down a road going straight to the destination, without
        // putting in a "corner" first.
        return 0;
    }
}

RoadBuilder::RoadBuilder(const Vector3r &start_pos, real_t start_angle)
    : curr_pos(start_pos), theta(start_angle)
{
    Vector3r left = Vector3r(ROAD_WIDTH/2 * cos(theta), 0, -ROAD_WIDTH/2 * sin(theta)) + start_pos;
    left_points.push_back(left);
    Vector3r right = Vector3r(-ROAD_WIDTH/2 * cos(theta), 0, ROAD_WIDTH/2 * sin(theta)) + start_pos;
    right_points.push_back(right);
}

void RoadBuilder::road(const Vector3r &target_pos, real_t turn_radius)
{
    // First add a "corner" section so that the road points towards
    // the target. Note this is flat, i.e. we don't try to change the
    // height of the road at the same time as turning round the
    // corner, that would be quite complicated!
    
    const Quaternionr orig_rot(AngleAxisr(theta, Vector3r(0,1,0)));
    const Vector3r delta = orig_rot.inverse() * Vector3r(target_pos[0] - curr_pos[0], 0, target_pos[2] - curr_pos[2]);
    const real_t sigma = real_t(delta[0] > 0 ? 1 : -1);  // +1 for left turn, -1 for right turn

    const int num_increments = SolveForTurnAngle(sigma * delta[0], delta[2], turn_radius);

    if (num_increments > 0) {
        for (int i = 1; i <= num_increments; ++i) {
            const real_t angle = i*TURN_INCREMENT;

            Vector3r pos(sigma*turn_radius*(1-cos(angle)), 0, turn_radius*sin(angle));
            pos = orig_rot * pos;
            pos += curr_pos;

            const Vector3r across = Vector3r(ROAD_WIDTH*cos(sigma*angle+theta)/2, 0, -ROAD_WIDTH*sin(sigma*angle+theta)/2);
            
            const Vector3r left = pos + across;
            const Vector3r right = pos - across;

            left_points.push_back(left);
            right_points.push_back(right);
        }

        // Modify the current position to be at the end of the turn
        curr_pos += Vector3r(sigma*turn_radius*cos(theta), 0, -sigma*turn_radius*sin(theta));
        theta += sigma*num_increments*TURN_INCREMENT;
        curr_pos -= Vector3r(sigma*turn_radius*cos(theta), 0, -sigma*turn_radius*sin(theta));
    }

    // Now add a straight road section going directly to the target.
    // (including an up or down slope if necessary.)

    // Split into multiple chunks if necessary.

    Vector3r vector_to_target = target_pos - curr_pos;
    real_t length = vector_to_target.norm();

    if (length >= real_t(0.1)) {
        vector_to_target /= length;

        // Adjust theta to point directly at the target
        theta = atan2(target_pos[0] - curr_pos[0], target_pos[2] - curr_pos[2]);

        Quaternionr rot(AngleAxisr(asin((target_pos[1]-curr_pos[1])/length), Vector3r(-1,0,0))); // slope
        rot = rot * Quaternionr(AngleAxisr(theta, Vector3r(0,1,0)));  // facing direction    
        
        bool last_chunk = false;

        do {
            last_chunk = false;
            real_t chunk_len = ROAD_CHUNK_LENGTH;
            if (length < chunk_len) {
                last_chunk = true;
                chunk_len = length;
            }

            const Vector3r left = rot * Vector3r(ROAD_WIDTH/2,0,0) + curr_pos + vector_to_target*chunk_len;
            const Vector3r right = rot * Vector3r(-ROAD_WIDTH/2,0,0) + curr_pos + vector_to_target*chunk_len;

            left_points.push_back(left);
            right_points.push_back(right);
            
            curr_pos += vector_to_target * chunk_len;
            length -= chunk_len;

        } while (!last_chunk);
    }
}

void RoadBuilder::loop(real_t radius, real_t screw)
{
    const int NUM_SEGMENTS = 30;

    // rot * local_coord = world_coord
    const Quaternionr rot(AngleAxisr(theta, Vector3r(0,1,0)));

    const Vector3r centre = curr_pos + Vector3r(0, radius, 0);

    for (int i = 1; i <= NUM_SEGMENTS; ++i) {

        real_t theta = real_t(i) / real_t(NUM_SEGMENTS) * 2 * PI;

        Vector3r pos(screw * theta / (2*PI), - radius * cos(theta), radius * sin(theta));
        pos = rot * pos + centre;

        Vector3r across = rot * Vector3r(ROAD_WIDTH/2, 0, 0);
        Vector3r left = pos + across;
        Vector3r right = pos - across;

        left_points.push_back(left);
        right_points.push_back(right);
    }

    curr_pos += rot * Vector3r(screw, 0, 0);
}
