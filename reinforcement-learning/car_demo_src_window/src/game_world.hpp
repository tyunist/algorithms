/*
 * FILE:
 *   game_world.hpp
 *
 * PURPOSE:
 *   This class is effectively a Mediator that combines together the
 *   three main subsystems:
 *
 *   1) Physics ("featherstone" project)
 *   2) Collision detection (BulletCollision library)
 *   3) Graphics (OGRE library)
 *
 *   The three subsystems do not interact with one another directly;
 *   all interactions are controlled and orchestrated by the GameWorld
 *   class.
 *   
 * AUTHOR:
 *   Stephen Thompson <stephen@solarflare.org.uk>
 *
 * CREATED:
 *   08-Aug-2010
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

#ifndef GAME_WORLD_HPP
#define GAME_WORLD_HPP

#include "real.hpp"

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <OgreSceneManager.h>

#include <memory>

class btCollisionObject;
class btCollisionShape;

struct GameWorldImpl;
class Multibody;

#ifdef CONTACT_FORCE_DEBUGGING
#include "DebugGraphics.h"
#endif

class GameWorld {
public:

    enum Material { M_DEFAULT, M_CAR };

    explicit GameWorld(Ogre::SceneManager *scene_mgr);
    ~GameWorld();

    // Functions to add stuff to the game world.

    // For positioning objects, the convention is that the object starts in its standard configuration
    // (i.e. centred on the origin with principal axes aligned with x/y/z), then is translated by "pos",
    // then is rotated by "rotation" (about its own centre of mass).
    
    void beginObjectMultibody(const Vector3r &pos, const Quaternionr &rotation, bool fixed_base, bool can_sleep,
                              bool sleep_initially);
    void beginObjectFixed(const Vector3r &pos, const Quaternionr &rotation);

    void setShapeCuboid(real_t density, real_t xsize, real_t ysize, real_t zsize,   // "diameter" not "radius" given.
                        float red, float green, float blue);
    void setShapeCuboid2(real_t density, real_t com_shift,
                         real_t xsize, real_t ysize, real_t zsize,   // "diameter" not "radius" given.
                         float red, float green, float blue);
    void setShapeSphere(real_t density, real_t radius, float red, float green, float blue);
    void setShapeCylinder(real_t density, real_t radius, real_t height, float red, float green, float blue); // X-aligned
    void setShapeDummyCylinder(real_t density, real_t radius, real_t height, float red, float green, float blue);
    void setShapeGroundPlane();  // the plane y=0. (y<0 is "underground".)
    void setShapeRoad(const std::vector<Vector3r> & left_points, const std::vector<Vector3r> & right_points);
    
    void setInitialVelocity(const Vector3r& vel);
    
    void setMaterial(Material m);

    // for joints we give the pos/rotation of the child object (when q=0) RELATIVE TO THE PARENT FRAME.
    // the joint axis is given in the PARENT frame.
    // the joint axis position is relative to parent COM and in parent frame.
    void beginChildPrismatic(const Vector3r &pos, const Quaternionr &rotation,
                             const Vector3r &joint_axis,
                             real_t initial_joint_angle);
    void beginChildRevolute(const Vector3r &pos, const Quaternionr &rotation,
                            const Vector3r &joint_axis, const Vector3r &joint_axis_position,
                            real_t initial_joint_angle);

    void endChild();
    Multibody * endObject();


    // set the material properties (simple hard-coded system for now)
    void setFriction(real_t mu);
    void setTyreFriction(real_t mu);
    void setRestitution(real_t e);
    
    
    // setup constraint that a joint angle should be between given limits.
    void setupJointLimit(Multibody *bod, int link, real_t lower_bound, real_t upper_bound);

    // setup the bridge constraint. "endpoint" of bridge is tied to the "pos" (which is given in world coordinates).
    void setupBridgeConstraint(Multibody *bod, const Vector3r & pos,
        const Vector3r &pivot_ofs, const Vector3r &endpt_ofs);

    // Tell GameEngine about car wheels.
    // This does two things:
    // 1. Adjusts wheel contact points so they are consistent with the collision normal.
    // 2. Filters out collisions between wheels and the car body.
    void setupWheels(Multibody *car,
                     const std::vector<int> &wheel_links,
                     const Vector3r &axis,   // axle direction. unit length
                     real_t radius);         // wheel radius.

    // Add raycast collision detection
    // (intended to be used together with DummyCylinderShape)
    void addRaycast(Multibody *body,
                    int ref_link,
                    int force_link,
                    const Vector3r &start_point,  // in local coords of ref_link
                    const Vector3r &end_point,    // in local coords of ref_link
                    const Vector3r &axis,  // wheel axis (in ref_link frame)
                    const real_t radius);  // wheel radius
    
    // Attach an ogre camera to a multibody.
    void attachCamera(Multibody *bod, Ogre::Camera *cam) const;
    void detachCamera(Multibody *bod, Ogre::Camera *cam) const;
    bool isCameraAttached(Multibody *bod, Ogre::Camera *cam) const;


    

    // HACK: Update position of graphic and collision objects without changing the underlying multibody.
    // "pos" gives the vector FROM the centre of the collision/graphic shape, TO the centre of mass of the multibody.
    // Only works with CuboidShape2 (will CRASH if any other shape). 
    // Used for "car centre of mass height" feature.
    void setPositionAdjustment(Multibody *bod, const Vector3r &pos);


    // Clears all forces/torques then applies the given gravity acceleration to all objects.
    void applyGravity(const Vector3r & g);

    // Run a timestep. this will do collision detection, setup contact constraints as needed,
    // run sequential impulses and integrate the bodies.
    void doTimestep(real_t dt, 
                    std::vector<real_t> &scratch_r, std::vector<Vector3r> &scratch_v,
                    std::vector<Matrix3r> &scratch_m,
                    double &c_time, double &p_time, double &s_time, double &t_time, int &nconstr);

    real_t getGlobalTime() const;

#ifdef CONTACT_FORCE_DEBUGGING
    void setDebugGraphics(Ogre::DebugGraphics *d) { debug_graphics = d; }
    void setDrawContactForces(bool draw);
#endif
    
private:
    GameWorld(const GameWorld &); // not implemented
    void operator=(const GameWorld &); // not implemented

    std::auto_ptr<GameWorldImpl> pimpl;

#ifdef CONTACT_FORCE_DEBUGGING
    Ogre::DebugGraphics * debug_graphics;
#endif
};

#endif
