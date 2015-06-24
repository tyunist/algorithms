/*
 * FILE:
 *   game_world.cpp
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
 * NOTES:
 *   This file is fairly complex (~2000 lines) but:
 *
 *    1) A lot of the code is experimental code that didn't make it
 *       into the final demo; and
 *
 *    2) This is the code that represents the interactions between the
 *       three main subsystems of the demo, so it is going to be
 *       complex. I would rather have that complexity all in one place
 *       where I can see it, than scattered throughout several
 *       different classes in the system where the dependencies would
 *       be harder to understand. (In many ways this is the whole
 *       point of the Mediator pattern.)
 *
 *   That being said, there is scope for some refactoring and code
 *   cleanup here. For example the different "Shape" classes in this
 *   file should probably be factored out into their own files, and a
 *   public interface defined so that users can create their own
 *   Shapes (rather than GameWorld just providing a short list of
 *   predefined Shapes).
 *   
 */

#include <Eigen/StdVector>  // we use vectors of Eigen objects below
#include <Eigen/LU>

#include "bridge_constraint.hpp"
#include "contact_constraint.hpp"
#include "game_world.hpp"
#include "joint_limit_constraint.hpp"
#include "multibody.hpp"
#include "position_correction.hpp"
#include "sequential_impulses.hpp"
#include "wheel_contact_constraint.hpp"

#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btEmptyShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMesh.h"

#include <OgreEntity.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>

#include <map>
#include <vector>

#ifdef CONTACT_FORCE_DEBUGGING
#include "DebugGraphics.h"
#endif

#define WIN32_LEAN_AND_MEAN
#include <windows.h>  // For timing code (QueryPerformanceCounter).

#define USE_SLEEPING
//#define DUMP_CONSTRAINT_INFO


// Stores data about each multibody
struct MultibodyData {
    std::vector<btCollisionObject *> collision_objects;  // one per link. (owned by us)
    std::vector<Ogre::SceneNode *> scene_nodes;   // owned by OGRE. can be null.
    std::vector<std::pair<Multibody*, int> > obj_and_link_num;  // this is needed to map the btCollisionObjects back to a Multibody* & link num.
};


// MultibodyMap typedef.
// (Note: should probably change this to use hash_map, or tr1::unordered_map.)
typedef std::map<Multibody *, MultibodyData> MultibodyMap;

namespace {

    // Describes the shape of a body or link
    class Shape {
    public:
        virtual ~Shape() { }

        // Returns a new btCollisionShape for this shape
        virtual btCollisionShape * makeCollisionShape() const = 0;

        // Returns a new ogre movableobject (owned by OGRE)
        virtual Ogre::MovableObject * makeOgreMovableObject(Ogre::SceneManager *sm) const = 0;

        // Get mass and inertia
        virtual void getMassAndInertia(real_t &mass, Vector3r &inertia) const = 0;

        // HACK for "car centre of mass height" feature
        virtual bool wantExtraSceneNode() const { return false; }
    };


    // Debug drawing routines
    Ogre::ManualObject * CreateLine(Ogre::SceneManager * scene_manager, const Vector3r &from, const Vector3r &to)
    {
        static int obj_count = 0;
        ++obj_count;
        std::ostringstream str;
        str << "DebugLine" << obj_count;

        Ogre::ManualObject * myManualObject = scene_manager->createManualObject(str.str());

        Ogre::MaterialPtr myManualObjectMaterial = Ogre::MaterialManager::getSingleton().create("manualLineMaterial","General"); 
        myManualObjectMaterial->setReceiveShadows(false);
        myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,0,0);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,0);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(1, 1, 1);

        myManualObject->begin("manualLineMaterial", Ogre::RenderOperation::OT_LINE_LIST); 
        myManualObject->position(from.x(), from.y(), from.z());
        myManualObject->position(to.x(), to.y(), to.z());
        myManualObject->end();

        return myManualObject;
    }

    Ogre::ManualObject * CreatePoint(Ogre::SceneManager * scene_manager, const Vector3r &where)
    {
        static int obj_count = 0;
        ++obj_count;
        std::ostringstream str;
        str << "DebugPoint" << obj_count;

        Ogre::ManualObject * myManualObject = scene_manager->createManualObject(str.str());

        Ogre::MaterialPtr myManualObjectMaterial = Ogre::MaterialManager::getSingleton().create("manualPointMaterial","General"); 
        myManualObjectMaterial->setReceiveShadows(false);
        myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,0,0);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,0);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(1, 1, 1);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setPointSize(10);

        myManualObject->begin("manualPointMaterial", Ogre::RenderOperation::OT_POINT_LIST); 
        myManualObject->position(where.x(), where.y(), where.z());
        myManualObject->end();

        return myManualObject;
    }

    // ObjectSetup class
    // Used in GameWorld during object creation (fixed objs & multibodies)
    class ObjectSetup {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // minimal ctor
        ObjectSetup() : shape(0), parent(0), material(GameWorld::M_DEFAULT) { }

        // dtor. deletes the shape, and all child objects.
        ~ObjectSetup() {
            delete shape;
            for (std::vector<ObjectSetup*>::iterator it = children.begin(); it != children.end(); ++it) {
                delete *it;
            }
        }

        // function to help create child objects
        ObjectSetup * createChild() {
            ObjectSetup *p = new ObjectSetup;
            p->parent = this;
            children.push_back(p);
            return p;
        }

        int countLinks() const {
            int i = 1;
            for (std::vector<ObjectSetup*>::const_iterator it = children.begin(); it != children.end(); ++it) {
                i += (*it)->countLinks();
            }
            return i;
        }

        // function to setup a multibody, complete with scene nodes etc
        // link_num_counter always points to the lowest unallocated link num
        void setupMultibodyLinks(Multibody *bod, MultibodyData &dat, btCollisionWorld &collision_world,
                                 std::vector<btCollisionShape*> &collision_shapes, 
                                 Ogre::SceneManager *scene_manager,
                                 Ogre::SceneNode * parent_scene_node,
                                 bool draw_line, const Vector3r & line_pos,
                                 int this_link_num, int & link_num_counter) const
        {
            // create Bullet collision object
            dat.collision_objects.push_back(new btCollisionObject);
            dat.collision_objects.back()->setCollisionFlags(
                dat.collision_objects.back()->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
                    // Maybe overkill to set it for all objects, never mind

            // create collision shape and attach it to the collision object
            // NOTE: not attempting to share collision shapes between different collision objects at the moment.
            if (shape) {
                btCollisionShape * cs = shape->makeCollisionShape();
                collision_shapes.push_back(cs);
                dat.collision_objects.back()->setCollisionShape(cs);

                // now we can add it to the collision world.
                collision_world.addCollisionObject(dat.collision_objects.back());
            } else {
                collision_shapes.push_back(0);

                // (do not add anything to the collision world in this case.)
            }

            // create Ogre scene node
            // NOTE: phys_scene_node is positioned at the centre of mass.
            // gfx_scene_node is positioned at the centre of the graphics model.
            // Normally these are the same, but when the hack for CoM adjustment is in use,
            // they will be different.
            Ogre::SceneNode * phys_scene_node = parent_scene_node->createChildSceneNode();
            dat.scene_nodes.push_back(phys_scene_node);

            Ogre::SceneNode * gfx_scene_node = 0;

            if (shape && shape->wantExtraSceneNode() && this_link_num == -1) {
                // HACK for "car centre of mass height" feature...
                gfx_scene_node = phys_scene_node->createChildSceneNode("CAR_COM_HACK");
            } else {
                // Normal case
                gfx_scene_node = phys_scene_node;
            }

            // create the Ogre movable object and attach it to the scene node
            // note: this is owned by Ogre so we don't have to delete it afterwards.
            if (shape) {
                Ogre::MovableObject *mob = shape->makeOgreMovableObject(scene_manager);
                gfx_scene_node->attachObject(mob);
            }

            if (draw_line) {
                Ogre::MovableObject * line = CreateLine(scene_manager, Vector3r::Zero(), line_pos);
                gfx_scene_node->attachObject(line);
            }

            // create back-reference to multibody & link num, and store this in the Bullet collision object.
            // note: we are assuming collision_objects has been reserve()d to the correct size (otherwise ptr may become invalid!)
            dat.obj_and_link_num.push_back(std::make_pair(bod, this_link_num));
            dat.collision_objects.back()->setUserPointer(&dat.obj_and_link_num.back());

            // Now process each child link in turn
            for (std::vector<ObjectSetup*>::const_iterator it = children.begin(); it != children.end(); ++it) {
                const int child_link_num = link_num_counter++;

                real_t mass;
                Vector3r inertia;

                if ((*it)->shape) {
                    (*it)->shape->getMassAndInertia(mass, inertia);
                } else {
                    mass = 0;
                    inertia = Vector3r::Zero();
                }

                Quaternionr parent_to_child = (*it)->rotation.inverse();
                Vector3r joint_axis_child = parent_to_child * (*it)->joint_axis;
                
                // Debug line drawing
                bool child_do_line = false;
                Vector3r child_line_pos;

                // Setup the joint (prismatic or revolute)
                if ((*it)->is_prismatic) {
                    bod->setupPrismatic(child_link_num, mass, inertia, this_link_num,
                                        parent_to_child, joint_axis_child, parent_to_child * (*it)->position);
                } else {
                    bod->setupRevolute(child_link_num, mass, inertia, this_link_num,
                                       parent_to_child, joint_axis_child,
                                       (*it)->joint_axis_position,
                                       parent_to_child * ((*it)->position - (*it)->joint_axis_position));

                    // Debug line drawing for revolute joints
                    Ogre::MovableObject * line = CreateLine(scene_manager, Vector3r::Zero(), (*it)->joint_axis_position);
                    gfx_scene_node->attachObject(line);
                    Ogre::MovableObject * point = CreatePoint(scene_manager, (*it)->joint_axis_position);
                    gfx_scene_node->attachObject(point);
                    child_do_line = true;
                    child_line_pos = parent_to_child * ((*it)->joint_axis_position - (*it)->position);
                }
                bod->setJointPos(child_link_num, (*it)->initial_joint_angle);

                // Do the recursive call
                (*it)->setupMultibodyLinks(bod, dat, collision_world, collision_shapes, 
                                           scene_manager, phys_scene_node,
                                           child_do_line, child_line_pos,
                                           child_link_num, link_num_counter);
            }
        }
        
    public:
        // general info. these are always set
        Shape *shape;  // contains information about the collision shape and ogre object.
        ObjectSetup *parent;
        std::vector<ObjectSetup*> children;

        // pos, rotation info. always set
        Vector3r position;
        Quaternionr rotation;

        // initial velocity (of the base). always set
        Vector3r initial_vel;
        
        // joint info. not set on the root.
        bool is_prismatic;
        Vector3r joint_axis;
        Vector3r joint_axis_position;
        real_t initial_joint_angle;

        // is_multibody flag. set only on the root. set by beginObject functions.
        bool is_multibody;

        bool is_fixed_base;
        bool can_sleep;
        bool sleep_initially;

        GameWorld::Material material;

    private:
        void operator=(const ObjectSetup&);   // not defined
        ObjectSetup(const ObjectSetup&);   // not defined
    };
}


namespace {
    // Wheel info
    struct WheelInfo {
        Vector3r axis;
        real_t radius;
    };

    typedef std::map<std::pair<Multibody*, int>, WheelInfo> WheelInfoMap;


    // Raycast
    struct Raycast {
        Multibody *body;
        int ref_link;
        int force_link;
        Vector3r start_point;  // in local coords of ref_link
        Vector3r end_point;    // in local coords of ref_link
        Vector3r axis;
        real_t radius;

        WheelContactConstraint * constraint;
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}


// GameWorld implementation class
struct GameWorldImpl {
    
    explicit GameWorldImpl(Ogre::SceneManager *sm);
    ~GameWorldImpl();
    
    // map from multibody ptr -> multibody data
    MultibodyMap multibodies;

    // kinetic collision objects (owned by us)
    std::vector<btCollisionObject*> kinetic_objects;
    
    // bullet collision stuff (collision world, etc)
    btDefaultCollisionConfiguration collision_configuration;
    btCollisionDispatcher dispatcher;
    btAxisSweep3 broadphase;
    btCollisionWorld collision_world;

    // list of collision shapes (to be deleted in dtor)
    std::vector<btCollisionShape*> collision_shapes;

    // Ogre scene manager
    Ogre::SceneManager *scene_manager;
    
    // This is just used for initialization
    ObjectSetup *obj;

    // Joint limit constraints
    std::vector<JointLimitConstraint *> joint_limit_constraints;

    // Bridge constraint (there is only one of these)
    std::auto_ptr<Constraint> bridge_constraint;

    real_t global_time;

    std::map<const Multibody*, GameWorld::Material> materials;
    real_t mFriction, mTyreFriction, mRestitution;
    bool force_update_contacts;

    bool draw_contact_forces;

    // Wheel info
    WheelInfoMap wheel_info_map;

    // Raycasts
    std::vector<Raycast> raycasts;
    
    // Meshes
    struct CuboidProps { 
        float xs, ys, zs, r, g, b;
        bool operator<(const CuboidProps &other) const
        {
            return
                xs < other.xs ? true :
                xs > other.xs ? false :
                ys < other.ys ? true :
                ys > other.ys ? false :
                zs < other.zs ? true :
                zs > other.zs ? false :
                r < other.r ? true :
                r > other.r ? false :
                g < other.g ? true : 
                g > other.g ? false :
                b < other.b;
        }
    };
    std::map<CuboidProps, Ogre::String> cuboid_meshes;

    // Member functions
    GameWorld::Material lookupMaterial(const Multibody *body) const;
    void getMaterialProperties(const Multibody *bod1, const Multibody *bod2, 
                               real_t &friction, real_t &restitution, real_t &baumgarte) const;
    Ogre::String generateCuboidMesh(float xs, float ys, float zs, float r, float g, float b);
};

GameWorldImpl::GameWorldImpl(Ogre::SceneManager *sm)
    : dispatcher(&collision_configuration),
      broadphase(btVector3(-1000, -1000, -1000),   // world aabb min
                 btVector3(1000, 1000, 1000)),     // world aabb max
      collision_world(&dispatcher, &broadphase, &collision_configuration),
      scene_manager(sm),
      obj(0),
      global_time(0),
      mFriction(0.5f), mTyreFriction(1), mRestitution(0.02f),
      force_update_contacts(false), draw_contact_forces(false)
{
    // Create the CuboidMaterial (it is the same for all cuboid meshes)
    const std::string material_name = "CuboidMaterial";
    Ogre::MaterialPtr myManualObjectMaterial = Ogre::MaterialManager::getSingleton().create(material_name, "General"); 
    myManualObjectMaterial->setReceiveShadows(false);
    myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true);
    myManualObjectMaterial->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE);
    myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,0);
}
    

GameWorldImpl::~GameWorldImpl()
{
    // Delete the contact constraints
    const int num_manifolds = collision_world.getDispatcher()->getNumManifolds();
    for (int i = 0; i < num_manifolds; ++i) {
        btPersistentManifold * contact_manifold = collision_world.getDispatcher()->getManifoldByIndexInternal(i);
        const int num_contacts = contact_manifold->getNumContacts();
        for (int j = 0; j < num_contacts; ++j) {
            btManifoldPoint * manifold_point = & contact_manifold->getContactPoint(j);
            delete static_cast<Constraint*>(manifold_point->m_userPersistentData);
            manifold_point->m_userPersistentData = 0;
        }
    }

    // Delete the other constraints
    for (std::vector<JointLimitConstraint*>::iterator it = joint_limit_constraints.begin(); it != joint_limit_constraints.end(); ++it) {
        delete *it;
    }
    for (std::vector<Raycast>::iterator it = raycasts.begin(); it != raycasts.end(); ++it) {
        delete it->constraint;
    }
        
    
    // Delete the bullet collision objects and the multibodies
    for (MultibodyMap::iterator it = multibodies.begin(); it != multibodies.end(); ++it) {
        delete it->first;
        for (std::vector<btCollisionObject*>::iterator it2 = it->second.collision_objects.begin(); it2 != it->second.collision_objects.end(); ++it2) {
            collision_world.removeCollisionObject(*it2);
            delete (*it2);
        }
    }

    // Delete the bullet collision objects corresponding to "fixed objects"
    for (std::vector<btCollisionObject*>::iterator it = kinetic_objects.begin(); it != kinetic_objects.end(); ++it) {
        collision_world.removeCollisionObject(*it);
        delete (*it);
    }

    // Delete the collision shapes
    for (std::vector<btCollisionShape*>::iterator it = collision_shapes.begin(); it != collision_shapes.end(); ++it) {
        delete *it;
    }
    
    // Delete any half-formed ObjectSetup
    delete obj;

    // clear out the ogre scene, this will get rid of all the ogre objects we created.
    scene_manager->clearScene();
}

GameWorld::Material GameWorldImpl::lookupMaterial(const Multibody *body) const
{
    std::map<const Multibody*, GameWorld::Material>::const_iterator it = materials.find(body);
    if (it == materials.end()) return GameWorld::M_DEFAULT;
    else return it->second;
}

void GameWorldImpl::getMaterialProperties(const Multibody *bod1, const Multibody *bod2, 
                                          real_t &friction, real_t &restitution, real_t &baumgarte) const
{
    GameWorld::Material mat1 = lookupMaterial(bod1);
    GameWorld::Material mat2 = lookupMaterial(bod2);

    restitution = mRestitution;

    if (mat1 == GameWorld::M_CAR || mat2 == GameWorld::M_CAR) {
        friction = mTyreFriction;
        baumgarte = real_t(0.05);
    } else {
        friction = mFriction;
        baumgarte = real_t(0.02);
    }
}

Ogre::String GameWorldImpl::generateCuboidMesh(float xsize, float ysize, float zsize, float red, float green, float blue)
{
    CuboidProps props;
    props.xs = xsize; props.ys = ysize; props.zs = zsize;
    props.r = red; props.g = green; props.b = blue;

    std::map<CuboidProps, Ogre::String>::const_iterator it = cuboid_meshes.find(props);
    if (it != cuboid_meshes.end()) {
        return it->second;
    } else {
        
        // create a manual object for the cuboid

        static int obj_count = 0;
        ++obj_count;
        std::ostringstream str;
        str << "MObj_Cuboid" << obj_count;
        
        Ogre::ManualObject * box_object = scene_manager->createManualObject(str.str());
        box_object->begin("CuboidMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);

        Ogre::uint32 idx = 0;
        
        for (int axis = 0; axis < 3; ++axis) {
            for (int sign = -1; sign <= 1; sign += 2) {
                Ogre::Vector3 zdir(0,0,0);
                zdir[axis] = float(sign);
                Ogre::Vector3 xdir(0,0,0);
                xdir[(axis+1)%3] = float(sign);
                Ogre::Vector3 ydir(0,0,0);
                ydir[(axis+2)%3] = 1;

                xdir[0] *= xsize/2;
                xdir[1] *= ysize/2;
                xdir[2] *= zsize/2;

                ydir[0] *= xsize/2;
                ydir[1] *= ysize/2;
                ydir[2] *= zsize/2;

                zdir[0] *= xsize/2;
                zdir[1] *= ysize/2;
                zdir[2] *= zsize/2;

                Ogre::Vector3 normal(0,0,0);
                normal[axis] = float(sign);

                Ogre::ColourValue col(red, green, blue);
                /*
                const float cfactor = 1.0f - 0.1f * axis;
                col[0] *= cfactor;
                col[1] *= cfactor;
                col[2] *= cfactor;
                */
                
                box_object->position(zdir - xdir - ydir);
                box_object->normal(normal);
                box_object->colour(col);
                
                box_object->position(zdir + xdir - ydir);
                box_object->normal(normal);
                box_object->colour(col);

                box_object->position(zdir + xdir + ydir);
                box_object->normal(normal);
                box_object->colour(col);

                box_object->position(zdir - xdir + ydir);
                box_object->normal(normal);
                box_object->colour(col);

                box_object->quad(idx, idx+1, idx+2, idx+3);

                idx += 4;
            }
        }
        
        box_object->end();

        std::ostringstream str2;
        str2 << "Mesh_Cuboid" << obj_count;

        box_object->convertToMesh(str2.str());

        return str2.str();
    }
}
    

namespace {
    
    // Cuboid shape implementation
    
    class CuboidShape : public Shape {
    public:
        CuboidShape(GameWorldImpl *gw, real_t den, real_t xs, real_t ys, real_t zs,
                    float r, float g, float b)
            : gw_impl(gw), density(den), xsize(xs), ysize(ys), zsize(zs), red(r), green(g), blue(b) { }
        virtual btCollisionShape * makeCollisionShape() const;
        virtual Ogre::MovableObject * makeOgreMovableObject(Ogre::SceneManager *sm) const;
        virtual void getMassAndInertia(real_t &mass, Vector3r &inertia) const;
        
    protected:
        GameWorldImpl * gw_impl;
        real_t density;
        real_t xsize, ysize, zsize;
        float red, green, blue;
    };
    
    btCollisionShape * CuboidShape::makeCollisionShape() const
    {
        btVector3 box_half_extents(xsize/2, ysize/2, zsize/2);
        btCollisionShape * shape = new btBoxShape(box_half_extents);
        //shape->setMargin(0);
        return shape;
    }

    Ogre::MovableObject * CuboidShape::makeOgreMovableObject(Ogre::SceneManager *scene_manager) const
    {
        Ogre::String mesh_name = gw_impl->generateCuboidMesh(float(xsize), float(ysize), float(zsize), red, green, blue);

        static int obj_count = 0;
        ++obj_count;
        std::ostringstream str;
        str << "Cuboid" << obj_count;

        return scene_manager->createEntity(str.str(), mesh_name);
    }

    void CuboidShape::getMassAndInertia(real_t &mass, Vector3r &inertia) const
    {
        mass = density * xsize * ysize * zsize;
        const real_t mass_over_12 = mass * real_t(1.0/12.0);
        inertia = Vector3r(mass_over_12 * (ysize*ysize + zsize*zsize),
                           mass_over_12 * (xsize*xsize + zsize*zsize),
                           mass_over_12 * (xsize*xsize + ysize*ysize));
    }


    // Like cuboid but all mass is assumed to be in a thin layer at the given Y position.
    // TODO: Fix copy/pasted code

    class CuboidShape2 : public Shape {
    public:
        CuboidShape2(real_t den, real_t cpos, real_t xs, real_t ys, real_t zs, float r, float g, float b) 
            : density(den), com_shift(cpos), xsize(xs), ysize(ys), zsize(zs), red(r), green(g), blue(b) { }
        virtual btCollisionShape * makeCollisionShape() const;
        virtual Ogre::MovableObject * makeOgreMovableObject(Ogre::SceneManager *sm) const;
        virtual void getMassAndInertia(real_t &mass, Vector3r &inertia) const;
        virtual bool wantExtraSceneNode() const { return true; }

    protected:
        real_t density;
        real_t com_shift;
        real_t xsize, ysize, zsize;
        float red, green, blue;
    };
    
    btCollisionShape * CuboidShape2::makeCollisionShape() const
    {
        const btVector3 box_half_extents(xsize/2, ysize/2, zsize/2);

        btCompoundShape * result = new btCompoundShape;
        result->addChildShape(btTransform(btQuaternion(0,0,0,1), btVector3(0, -com_shift, 0)),
                              new btBoxShape(box_half_extents));  // TODO: memory leak?

        return result;
    }

    Ogre::MovableObject * CuboidShape2::makeOgreMovableObject(Ogre::SceneManager *scene_manager) const
    {
        static int obj_count = 0;
        ++obj_count;
        std::ostringstream str;
        str << "Cuboid_2_" << obj_count;

        // NOTE: We should probably be creating an Ogre::Mesh, so that we can re-use the same mesh for different objects.
        // For now though I am just re-creating a ManualObject for each new box ...
        
        Ogre::ManualObject * box_object = scene_manager->createManualObject(str.str());

        const std::string material_name = str.str() + "material";
        Ogre::MaterialPtr myManualObjectMaterial = Ogre::MaterialManager::getSingleton().create(material_name, "General"); 
        myManualObjectMaterial->setReceiveShadows(false);
        myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,0);
        
        box_object->begin(material_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);

        Ogre::uint32 idx = 0;
        
        for (int axis = 0; axis < 3; ++axis) {
            for (int sign = -1; sign <= 1; sign += 2) {
                Ogre::Vector3 zdir(0,0,0);
                zdir[axis] = float(sign);
                Ogre::Vector3 xdir(0,0,0);
                xdir[(axis+1)%3] = float(sign);
                Ogre::Vector3 ydir(0,0,0);
                ydir[(axis+2)%3] = 1;

                xdir[0] *= xsize/2;
                xdir[1] *= ysize/2;
                xdir[2] *= zsize/2;

                ydir[0] *= xsize/2;
                ydir[1] *= ysize/2;
                ydir[2] *= zsize/2;

                zdir[0] *= xsize/2;
                zdir[1] *= ysize/2;
                zdir[2] *= zsize/2;

                Ogre::Vector3 shift(0,-com_shift,0);

                Ogre::Vector3 normal(0,0,0);
                normal[axis] = float(sign);

                Ogre::ColourValue col(red, green, blue);
                /*
                const float cfactor = 1.0f - 0.1f * axis;
                col[0] *= cfactor;
                col[1] *= cfactor;
                col[2] *= cfactor;
                */
                
                box_object->position(zdir - xdir - ydir + shift);
                box_object->normal(normal);
                box_object->colour(col);
                
                box_object->position(zdir + xdir - ydir + shift);
                box_object->normal(normal);
                box_object->colour(col);

                box_object->position(zdir + xdir + ydir + shift);
                box_object->normal(normal);
                box_object->colour(col);

                box_object->position(zdir - xdir + ydir + shift);
                box_object->normal(normal);
                box_object->colour(col);

                box_object->quad(idx, idx+1, idx+2, idx+3);
                idx += 4;
            }
        }
        
        box_object->end();

        return box_object;
    }

    void CuboidShape2::getMassAndInertia(real_t &mass, Vector3r &inertia) const
    {
        mass = density * xsize * ysize * zsize;
        const real_t mass_over_12 = mass * real_t(1.0/12.0);
        inertia = Vector3r(mass_over_12 * (zsize*zsize),
                           mass_over_12 * (xsize*xsize + zsize*zsize),
                           mass_over_12 * (xsize*xsize));
    }


    // Sphere shape implementation

    const real_t PI = real_t(4) * atan(real_t(1));
    
    class SphereShape : public Shape {
    public:
        SphereShape(real_t den, real_t rad, float r, float g, float b)
            : density(den), radius(rad), red(r), green(g), blue(b) { }
        virtual btCollisionShape * makeCollisionShape() const;
        virtual Ogre::MovableObject * makeOgreMovableObject(Ogre::SceneManager *sm) const;
        virtual void getMassAndInertia(real_t &mass, Vector3r &inertia) const;

    private:
        real_t density;
        real_t radius;
        float red, green, blue;
    };

    btCollisionShape * SphereShape::makeCollisionShape() const
    {
        return new btSphereShape(radius);
    }

    Ogre::MovableObject * SphereShape::makeOgreMovableObject(Ogre::SceneManager *scene_manager) const
    {
        static int obj_count = 0;
        ++obj_count;
        std::ostringstream str;
        str << "Sphere" << obj_count;

        Ogre::ManualObject * sphere_object = scene_manager->createManualObject(str.str());
        
        const std::string material_name = str.str() + "material";
        Ogre::MaterialPtr myManualObjectMaterial = Ogre::MaterialManager::getSingleton().create(material_name, "General"); 
        myManualObjectMaterial->setReceiveShadows(false);
        myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_AMBIENT | Ogre::TVC_DIFFUSE);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,0);


        // ogre ManualObject sphere implementation taken from ogre wiki
        
        sphere_object->begin(material_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);

        const int nRings = 16;
        const int nSegments = 16;

        float fDeltaRingAngle = (PI / nRings);
        float fDeltaSegAngle = (2 * PI / nSegments);
        unsigned short wVerticeIndex = 0 ;
 
        // Generate the group of rings for the sphere
        for( int ring = 0; ring <= nRings; ring++ ) {
            float r0 = radius * sin (ring * fDeltaRingAngle);
            float y0 = radius * cos (ring * fDeltaRingAngle);
 
            // Generate the group of segments for the current ring
            for(int seg = 0; seg <= nSegments; seg++) {
                float x0 = r0 * sin(seg * fDeltaSegAngle);
                float z0 = r0 * cos(seg * fDeltaSegAngle);
 
                // Add one vertex to the strip which makes up the sphere
                sphere_object->position( x0, y0, z0);
                sphere_object->normal(Ogre::Vector3(x0, y0, z0).normalisedCopy());
                sphere_object->colour(red, green, blue);
                sphere_object->textureCoord((float) seg / (float) nSegments, (float) ring / (float) nRings);
 
                if (ring != nRings) {
                    // each vertex (except the last) has six indicies pointing to it
                    sphere_object->index(wVerticeIndex + nSegments + 1);
                    sphere_object->index(wVerticeIndex);               
                    sphere_object->index(wVerticeIndex + nSegments);
                    sphere_object->index(wVerticeIndex + nSegments + 1);
                    sphere_object->index(wVerticeIndex + 1);
                    sphere_object->index(wVerticeIndex);
                    wVerticeIndex ++;
                }
            }; // end for seg
        } // end for ring
        sphere_object->end();

        return sphere_object;
    }

    void SphereShape::getMassAndInertia(real_t &mass, Vector3r &inertia) const
    {
        mass = density * real_t(4) / real_t(3) * PI * (radius*radius*radius);
        inertia[0] = real_t(0.4) * mass * radius * radius;
        inertia[1] = inertia[2] = inertia[0];
    }


    // Cylinder shape implementation
    // The cylinder is aligned with the X axis.

    class CylinderShape : public Shape {
    public:
        CylinderShape(real_t den, real_t rad, real_t ht, float r, float g, float b)
            : density(den), radius(rad), height(ht), red(r), green(g), blue(b)
        { }
        virtual btCollisionShape * makeCollisionShape() const;
        virtual Ogre::MovableObject * makeOgreMovableObject(Ogre::SceneManager *sm) const;
        virtual void getMassAndInertia(real_t &mass, Vector3r &inertia) const;

    private:
        void drawCircle(Ogre::ManualObject * obj, float x, float xn, int &vertex_count) const;
        void drawRim(Ogre::ManualObject * obj, int &vertex_count) const;
        enum {NUM_SEGMENTS=16};
        
    private:
        real_t density;
        real_t radius, height;
        float red, green, blue;
    };

    btCollisionShape * CylinderShape::makeCollisionShape() const
    {
        return new btCylinderShapeX(btVector3(height/2, radius, radius));
    }

    Ogre::MovableObject * CylinderShape::makeOgreMovableObject(Ogre::SceneManager * scene_manager) const
    {
        static int obj_count = 0;
        ++obj_count;
        std::ostringstream str;
        str << "Cylinder" << obj_count;

        Ogre::ManualObject * cylinder_object = scene_manager->createManualObject(str.str());

        const std::string material_name = str.str() + "material";
        Ogre::MaterialPtr myManualObjectMaterial = Ogre::MaterialManager::getSingleton().create(material_name, "General"); 
        myManualObjectMaterial->setReceiveShadows(false);
        myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(red,green,blue,0);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(red,green,blue);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0,0,0);

        cylinder_object->begin(material_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);
        
        const int nSegments = 16;
        float fDeltaSegAngle = (2 * PI / nSegments);

        // Draw the two circles
        int vertex_count = 0;
        drawCircle(cylinder_object, height/2, 1, vertex_count);
        drawCircle(cylinder_object, -height/2, -1, vertex_count);

        // Draw the "rim"
        drawRim(cylinder_object, vertex_count);

        cylinder_object->end();
        return cylinder_object;
    }

    void CylinderShape::drawCircle(Ogre::ManualObject * obj, float x, float xn, int &vert) const
    {
        // Central vertex
        obj->position(x, 0, 0);
        obj->normal(xn, 0, 0);

        // Edge vertices
        for (int i = 0; i < NUM_SEGMENTS; ++i) {
            const float theta = i*2*PI/NUM_SEGMENTS;
            const float y = radius * cos(theta);
            const float z = radius * sin(theta);

            obj->position(x, y, z);
            obj->normal(xn, 0, 0);

            if (xn>0) {
                obj->index(vert);
                obj->index(vert + 1 + i);
                obj->index(vert + 1 + ((i+1)%NUM_SEGMENTS));
            } else {
                obj->index(vert);
                obj->index(vert + 1 + ((i+1)%NUM_SEGMENTS));
                obj->index(vert + 1 + i);
            }
        }

        vert += NUM_SEGMENTS + 1;
    }

    void CylinderShape::drawRim(Ogre::ManualObject *obj, int &vert) const
    {
        for (int i = 0; i < NUM_SEGMENTS; ++i) {
            const float theta = i*2*PI/NUM_SEGMENTS;
            const float y = radius * cos(theta);
            const float z = radius * sin(theta);

            const float ny = cos(theta);
            const float nz = sin(theta);
            
            obj->position(height/2, y, z);
            obj->normal(0, ny, nz);

            obj->position(-height/2, y, z);
            obj->normal(0, ny, nz);

            obj->index(vert + 2*i);
            obj->index(vert + 2*i + 1);
            obj->index(vert + 2*((i+1)%NUM_SEGMENTS));

            obj->index(vert + 2*((i+1)%NUM_SEGMENTS));
            obj->index(vert + 2*i + 1);
            obj->index(vert + 2*((i+1)%NUM_SEGMENTS) + 1);
        }

        vert += 2*NUM_SEGMENTS;
    }
    
    void CylinderShape::getMassAndInertia(real_t &mass, Vector3r &inertia) const
    {
        mass = density * PI * radius * radius * height;
        inertia[0] = mass * radius * radius / 2;
        inertia[1] = inertia[2] = mass * (3 * radius * radius + height * height) / 12;        
    }


    // DummyCylinderShape.
    // This is identical to CylinderShape except that the collision shape is removed.
    // (Intention is for the caller to add a Raycast instead.)
    class DummyCylinderShape : public CylinderShape {
    public:
        DummyCylinderShape(real_t den, real_t rad, real_t ht, float r, float g, float b)
            : CylinderShape(den, rad, ht, r, g, b) { }
        virtual btCollisionShape * makeCollisionShape() const { return new btEmptyShape; }
    };

    

    // Infinite plane shape implementation
    // (normal to Y axis)

    class GroundPlaneShape : public Shape {
    public:
        virtual btCollisionShape * makeCollisionShape() const;
        virtual Ogre::MovableObject * makeOgreMovableObject(Ogre::SceneManager *sm) const;
        virtual void getMassAndInertia(real_t &mass, Vector3r &inertia) const;
    };

    btCollisionShape * GroundPlaneShape::makeCollisionShape() const
    {
        return new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    }

    Ogre::MovableObject * GroundPlaneShape::makeOgreMovableObject(Ogre::SceneManager *scene_manager) const
    {
        static int obj_count = 0;
        ++obj_count;
        std::ostringstream str;
        str << "GroundPlane" << obj_count;

        Ogre::ManualObject * myManualObject = scene_manager->createManualObject(str.str());
     
        const std::string material_name = str.str() + "material";
        Ogre::MaterialPtr myManualObjectMaterial = Ogre::MaterialManager::getSingleton().create(material_name, "General"); 
        myManualObjectMaterial->setReceiveShadows(false); 
        myManualObjectMaterial->getTechnique(0)->setLightingEnabled(true); 
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setDiffuse(0,0,0,0);
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setAmbient(0,0,0); 
        myManualObjectMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0.7f, 0.7f, 0.7f);
         
        myManualObject->begin(material_name, Ogre::RenderOperation::OT_LINE_LIST); 
        const float LOW = -1000, HIGH = 1000, STEP = 10;
        for (float x = LOW; x <= HIGH; x += STEP) {
            myManualObject->position(x, 0, LOW);
            myManualObject->position(x, 0, HIGH);
        }
        for (float z = LOW; z <= HIGH; z += STEP) {
            myManualObject->position(LOW, 0, z);
            myManualObject->position(HIGH, 0, z);
        }
        myManualObject->end();

        return myManualObject;
    }

    void GroundPlaneShape::getMassAndInertia(real_t &mass, Vector3r &inertia) const
    {
        // dummy values since this will never be used as a dynamic object
        mass = 1;
        inertia = Vector3r::Ones();
    }


    // Road Shape
    class RoadShape : public Shape {
    public:
        RoadShape(const std::vector<Vector3r> &left, const std::vector<Vector3r> &right)
            : left_verts(left), right_verts(right) { }
        
        virtual btCollisionShape * makeCollisionShape() const;
        virtual Ogre::MovableObject * makeOgreMovableObject(Ogre::SceneManager *sm) const;
        virtual void getMassAndInertia(real_t & mass, Vector3r & inertia) const;

    private:
        const std::vector<Vector3r> & left_verts;
        const std::vector<Vector3r> & right_verts;
    };

    btVector3 ToBtVector(const Vector3r &v)
    {
        return btVector3(v[0], v[1], v[2]);
    }

    Ogre::Vector3 ToOgreVector(const Vector3r &v)
    {
        return Ogre::Vector3(v[0], v[1], v[2]);
    }
    
    btCollisionShape * RoadShape::makeCollisionShape() const
    {
        btTriangleMesh * mesh = new btTriangleMesh;  // TODO memory leak, bullet won't delete this when the bvh tri mesh shape is deleted (I think)

        for (size_t i = 1; i < left_verts.size(); ++i) {
            const btVector3 bot_left = ToBtVector(left_verts[i-1]);
            const btVector3 bot_right = ToBtVector(right_verts[i-1]);
            const btVector3 top_left = ToBtVector(left_verts[i]);
            const btVector3 top_right = ToBtVector(right_verts[i]);

            btVector3 normal = (bot_right - bot_left).cross(top_left - bot_left);
            normal.normalize();
            btVector3 across = bot_right - bot_left;
            across.normalize();

            const real_t KERB_HEIGHT = real_t(0.20);

            btVector3 bot_left_dn = bot_left - normal*KERB_HEIGHT - across*KERB_HEIGHT*5;
            btVector3 bot_right_dn = bot_right - normal*KERB_HEIGHT + across*KERB_HEIGHT*5;
            btVector3 top_left_dn = top_left - normal*KERB_HEIGHT - across*KERB_HEIGHT*5;
            btVector3 top_right_dn = top_right - normal*KERB_HEIGHT + across*KERB_HEIGHT*5;

            mesh->addTriangle(bot_left, bot_right, top_right, true);
            mesh->addTriangle(top_right, top_left, bot_left, true);
            mesh->addTriangle(bot_right, bot_right_dn, top_right_dn, true);
            mesh->addTriangle(top_right_dn, top_right, bot_right, true);
            mesh->addTriangle(top_left, top_left_dn, bot_left_dn, true);
            mesh->addTriangle(bot_left_dn, bot_left, top_left, true);
        }

        btBvhTriangleMeshShape * shape = new btBvhTriangleMeshShape(mesh, true);

        btTriangleInfoMap * tri_inf_map = new btTriangleInfoMap;  // TODO: Memory leak
        btGenerateInternalEdgeInfo(shape, tri_inf_map);

        return shape;
    }

    Ogre::MovableObject * RoadShape::makeOgreMovableObject(Ogre::SceneManager *sm) const
    {
        static int obj_count = 0;
        ++obj_count;
        std::ostringstream str;
        str << "Road" << obj_count;

        static bool material_created = false;

        if (!material_created) {
            const float r = 0.9f, g = 0.9f, b = 0.2f;
            
            Ogre::MaterialPtr myMaterial = Ogre::MaterialManager::getSingleton().create("roadMaterial", "General");
            myMaterial->setReceiveShadows(false);
            myMaterial->getTechnique(0)->getPass(0)->setDiffuse(r,g,b,0);
            myMaterial->getTechnique(0)->getPass(0)->setAmbient(r,g,b);
            myMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(0, 0, 0);
            myMaterial->setCullingMode(Ogre::CULL_NONE);
        }

        Ogre::ManualObject * obj = sm->createManualObject(str.str());
        // be lazy, just make a huge triangle list. not attempting to optimize into strips or index buffers etc.
        obj->begin("roadMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        for (size_t i = 1; i < left_verts.size(); ++i) {

            const Ogre::Vector3 bot_left = ToOgreVector(left_verts[i-1]);
            const Ogre::Vector3 bot_right = ToOgreVector(right_verts[i-1]);
            const Ogre::Vector3 top_left = ToOgreVector(left_verts[i]);
            const Ogre::Vector3 top_right = ToOgreVector(right_verts[i]);
            
            obj->position(bot_left);
            obj->position(bot_right);
            obj->position(top_right);

            obj->position(top_right);
            obj->position(top_left);
            obj->position(bot_left);
        }
        obj->end();

        return obj;
    }
    
    void RoadShape::getMassAndInertia(real_t & mass, Vector3r & inertia) const
    {
        // Dummy values
        mass = 1;
        inertia = Vector3r(1,1,1);
    }
}        


// GameWorld helper functions
namespace {
    
    void UpdateBulletAndOgreTransforms(MultibodyMap::iterator it,
        std::vector<Quaternionr> &world_to_local,
        std::vector<Vector3r> &local_origin)
    {
        // temporary storage
        world_to_local.resize(it->first->getNumLinks() + 1);  // rotate world vectors to local vectors
        local_origin.resize(it->first->getNumLinks() + 1);    // origin of local frame, in world coords
        
        // work out world-to-local transforms for each of the links (and the base)
        
        world_to_local[0] = it->first->getWorldToBaseRot();
        local_origin[0] = it->first->getBasePos();
        
        for (int i = 0; i < it->first->getNumLinks(); ++i) {
            const int parent = it->first->getParent(i);
            world_to_local[i+1] = it->first->getParentToLocalRot(i) * world_to_local[parent+1];
            local_origin[i+1] = local_origin[parent+1] + (world_to_local[i+1].inverse() * it->first->getRVector(i));
        }

        // set the bullet & ogre transforms.
        // note bullet uses local to world, rather than world to local, as its "world transform".
        // also ogre uses local to parent. (note: we assume the ogre scene nodes have
        // the same parent/child relationships as the links themselves.)
        for (int i = 0; i < it->first->getNumLinks()+1; ++i) {
            it->second.collision_objects[i]->setWorldTransform(
                btTransform(btQuaternion(-world_to_local[i].x(),
                                         -world_to_local[i].y(),
                                         -world_to_local[i].z(),
                                         world_to_local[i].w()),
                            btVector3(local_origin[i][0],
                                      local_origin[i][1],
                                      local_origin[i][2])));

            if (it->second.scene_nodes[i]) {
                const Quaternionr & q = i==0 ? it->first->getWorldToBaseRot() : it->first->getParentToLocalRot(i-1);
                it->second.scene_nodes[i]->setOrientation(q.w(), -q.x(), -q.y(), -q.z());

                if (i == 0) {
                    const Vector3r & x = it->first->getBasePos();
                    it->second.scene_nodes[i]->setPosition(x[0], x[1], x[2]);
                } else {
                    const Vector3r x = q.inverse() * it->first->getRVector(i-1);
                    it->second.scene_nodes[i]->setPosition(x[0], x[1], x[2]);
                }
            }
        }
    }

    bool DeleteConstraint(void *p)
    {
        delete static_cast<Constraint*>(p);
        return true;
    }
}


// GameWorld public member functions

GameWorld::GameWorld(Ogre::SceneManager *sm)
    : pimpl(new GameWorldImpl(sm))
{ }

GameWorld::~GameWorld()
{ }

void GameWorld::beginObjectMultibody(const Vector3r &pos, const Quaternionr &rotation, bool fixed_base, bool can_sleep,
                                     bool sleep_initially)
{
    if (!pimpl->obj) {
        pimpl->obj = new ObjectSetup;
        pimpl->obj->position = pos;
        pimpl->obj->rotation = rotation;
        pimpl->obj->initial_vel = Vector3r::Zero();
        pimpl->obj->is_multibody = true;
        pimpl->obj->is_fixed_base = fixed_base;
        pimpl->obj->can_sleep = can_sleep;
        pimpl->obj->sleep_initially = sleep_initially;
    }
}

void GameWorld::beginObjectFixed(const Vector3r &pos, const Quaternionr &rotation)
{
    if (!pimpl->obj) {
        pimpl->obj = new ObjectSetup;
        pimpl->obj->position = pos;
        pimpl->obj->rotation = rotation;
        pimpl->obj->is_multibody = false;
    }
}

void GameWorld::setShapeCuboid(real_t density, real_t xsize, real_t ysize, real_t zsize, float r, float g, float b)
{
    if (pimpl->obj) pimpl->obj->shape = new CuboidShape(pimpl.get(), density, xsize, ysize, zsize, r, g, b);
}

void GameWorld::setShapeCuboid2(real_t density, real_t com_shift,
                                real_t xsize, real_t ysize, real_t zsize, float r, float g, float b)
{
    if (pimpl->obj) pimpl->obj->shape = new CuboidShape2(density, com_shift, xsize, ysize, zsize, r, g, b);
}

void GameWorld::setShapeSphere(real_t density, real_t radius, float r, float g, float b)
{
    if (pimpl->obj) pimpl->obj->shape = new SphereShape(density, radius, r, g, b);
}

void GameWorld::setShapeCylinder(real_t density, real_t radius, real_t height, float r, float g, float b)
{
    if (pimpl->obj) pimpl->obj->shape = new CylinderShape(density, radius, height, r, g, b);
}

void GameWorld::setShapeDummyCylinder(real_t density, real_t radius, real_t height, float r, float g, float b)
{
    if (pimpl->obj) pimpl->obj->shape = new DummyCylinderShape(density, radius, height, r, g, b);
}

void GameWorld::setShapeGroundPlane()
{
    if (pimpl->obj) pimpl->obj->shape = new GroundPlaneShape;
}

void GameWorld::setShapeRoad(const std::vector<Vector3r> &left_points, const std::vector<Vector3r> &right_points)
{
    if (pimpl->obj) pimpl->obj->shape = new RoadShape(left_points, right_points);
}

void GameWorld::setInitialVelocity(const Vector3r &vel)
{
    if (pimpl->obj) pimpl->obj->initial_vel = vel;
}

void GameWorld::setMaterial(GameWorld::Material m)
{
    if (pimpl->obj) pimpl->obj->material = m;
}

void GameWorld::setFriction(real_t mu)
{
    pimpl->mFriction = mu;
    pimpl->force_update_contacts = true;
}

void GameWorld::setTyreFriction(real_t mu)
{
    pimpl->mTyreFriction = mu;
    pimpl->force_update_contacts = true;
}

void GameWorld::setRestitution(real_t e)
{
    pimpl->mRestitution = e;
    pimpl->force_update_contacts = true;
}

void GameWorld::beginChildPrismatic(const Vector3r &pos, const Quaternionr &rotation,
                                    const Vector3r &joint_axis,
                                    real_t initial_joint_angle)
{
    if (pimpl->obj) {
        pimpl->obj = pimpl->obj->createChild();

        pimpl->obj->position = pos;
        pimpl->obj->rotation = rotation;

        pimpl->obj->is_prismatic = true;
        pimpl->obj->joint_axis = joint_axis;
        // note: joint_axis_position is not used for prismatic joints

        pimpl->obj->initial_joint_angle = initial_joint_angle;
    }
}

void GameWorld::beginChildRevolute(const Vector3r &pos, const Quaternionr &rotation,
                                   const Vector3r &joint_axis, const Vector3r &joint_axis_position,
                                   real_t initial_joint_angle)
{
    if (pimpl->obj) {
        pimpl->obj = pimpl->obj->createChild();

        pimpl->obj->position = pos;
        pimpl->obj->rotation = rotation;

        pimpl->obj->is_prismatic = false;
        pimpl->obj->joint_axis = joint_axis;
        pimpl->obj->joint_axis_position = joint_axis_position;
        pimpl->obj->initial_joint_angle = initial_joint_angle;
    }
}

void GameWorld::endChild()
{
    // move up to the parent
    if (pimpl->obj && pimpl->obj->parent) {
        pimpl->obj = pimpl->obj->parent;
    }
}

Multibody * GameWorld::endObject()
{
    Multibody * result = 0;

    if (pimpl->obj) {
        // make sure we are at the parent
        while (pimpl->obj->parent) pimpl->obj = pimpl->obj->parent;

        if (pimpl->obj->is_multibody) {
            // create the multibody
            const int n_links = pimpl->obj->countLinks() - 1;  // -1 to subtract out the base

            real_t mass;
            Vector3r inertia;
            pimpl->obj->shape->getMassAndInertia(mass, inertia);
            
            Multibody * bod = new Multibody(n_links, mass, inertia, pimpl->obj->is_fixed_base, pimpl->obj->can_sleep);
            result = bod;
            bod->setBasePos(pimpl->obj->position);
            bod->setWorldToBaseRot(pimpl->obj->rotation.inverse());
            bod->setBaseVel(pimpl->obj->initial_vel);

            // insert it into our multibodies map
            std::pair<MultibodyMap::iterator, bool> p = pimpl->multibodies.insert(std::make_pair(bod, MultibodyData()));
            assert(p.second);  // it should always be inserted
            MultibodyData & dat = p.first->second;  // alias

            // reserve vectors to the correct size
            dat.collision_objects.reserve(n_links + 1);
            dat.scene_nodes.reserve(n_links + 1);
            dat.obj_and_link_num.reserve(n_links + 1);

            // call recursive function to set up each child link correctly
            int link_num_counter = 0;
            pimpl->obj->setupMultibodyLinks(bod, dat, pimpl->collision_world, pimpl->collision_shapes,
                                            pimpl->scene_manager, pimpl->scene_manager->getRootSceneNode(), 
                                            false, Vector3r::Zero(), -1, link_num_counter);

            // inform bullet & ogre of the correct transforms.
            std::vector<Quaternionr> q_temp;
            std::vector<Vector3r> v_temp;
            UpdateBulletAndOgreTransforms(p.first, q_temp, v_temp);

            // store the material
            pimpl->materials[bod] = pimpl->obj->material;

            // put it to sleep initially, if required
            if (pimpl->obj->sleep_initially) {
                bod->goToSleep();
            }

        } else {

            // create collision object & collision shape
            btCollisionObject * co = new btCollisionObject;
            btCollisionShape * cs = pimpl->obj->shape->makeCollisionShape();
            pimpl->collision_shapes.push_back(cs);
            co->setCollisionShape(cs);

            // set collision object transform
            co->setWorldTransform(btTransform(
                btQuaternion(pimpl->obj->rotation.x(),
                             pimpl->obj->rotation.y(),
                             pimpl->obj->rotation.z(),
                             pimpl->obj->rotation.w()),
                btVector3(pimpl->obj->position[0],
                          pimpl->obj->position[1],
                          pimpl->obj->position[2])));

            // add the collision shape to the world
            pimpl->kinetic_objects.push_back(co);
            pimpl->collision_world.addCollisionObject(co);

            // create Ogre scene node
            Ogre::SceneNode * sn = pimpl->scene_manager->getRootSceneNode()->createChildSceneNode();
            sn->setOrientation(pimpl->obj->rotation.w(),
                               pimpl->obj->rotation.x(),
                               pimpl->obj->rotation.y(),
                               pimpl->obj->rotation.z());
            sn->setPosition(pimpl->obj->position.x(),
                            pimpl->obj->position.y(),
                            pimpl->obj->position.z());
            Ogre::MovableObject *mob = pimpl->obj->shape->makeOgreMovableObject(pimpl->scene_manager);
            sn->attachObject(mob);
        }

        // can get rid of the setup object now.
        delete pimpl->obj;
        pimpl->obj = 0;
    }

    return result;
}

void GameWorld::setupJointLimit(Multibody *bod, int link, real_t lower_bound, real_t upper_bound)
{
    for (std::vector<JointLimitConstraint*>::iterator it = pimpl->joint_limit_constraints.begin();
    it != pimpl->joint_limit_constraints.end(); ++it) {
        if ((*it)->getBody1() == bod && (*it)->getLink1() == link) {
            (*it)->setJointLimits(lower_bound, upper_bound);
            return;
        }
    }

    pimpl->joint_limit_constraints.push_back(new JointLimitConstraint(bod, link, lower_bound, upper_bound));
    pimpl->joint_limit_constraints.back()->setBaumgarteParams(real_t(0.4), real_t(1));
}

void GameWorld::setupBridgeConstraint(Multibody *bod, const Vector3r &pos, const Vector3r &pivot_ofs, const Vector3r &endpt_ofs)
{
    pimpl->bridge_constraint.reset(new BridgeConstraint(bod, pos, pivot_ofs, endpt_ofs));
}

void GameWorld::setupWheels(Multibody *bod, const std::vector<int> &links, const Vector3r &axis, real_t radius)
{
    WheelInfo info;
    info.axis = axis;
    info.radius = radius;
    for (std::vector<int>::const_iterator it = links.begin(); it != links.end(); ++it) {
        std::pair<Multibody*, int> key(bod, *it);
        WheelInfoMap::value_type value(key, info);
        pimpl->wheel_info_map.insert(value);
    }
}

void GameWorld::addRaycast(Multibody *bod, int ref_link, int force_link, 
                           const Vector3r &start_point, const Vector3r &end_point,
                           const Vector3r &axis, real_t radius)
{
    Raycast r;
    r.body = bod;
    r.ref_link = ref_link;
    r.force_link = force_link;
    r.start_point = start_point;
    r.end_point = end_point;
    r.axis = axis;
    r.radius = radius;
    r.constraint = 0;
    pimpl->raycasts.push_back(r);
}

void GameWorld::attachCamera(Multibody *bod, Ogre::Camera *cam) const
{
    MultibodyMap::const_iterator it = pimpl->multibodies.find(bod);
    if (it != pimpl->multibodies.end()) {
        for (int i = 0; i < it->second.scene_nodes[0]->numAttachedObjects(); ++i) {
            if (it->second.scene_nodes[0]->getAttachedObject(i) == cam) {
                return;  // cam already attached!
            }
        }
        it->second.scene_nodes[0]->attachObject(cam);
    }
}

void GameWorld::detachCamera(Multibody *bod, Ogre::Camera *cam) const
{
    MultibodyMap::const_iterator it = pimpl->multibodies.find(bod);
    if (it != pimpl->multibodies.end()) {
        it->second.scene_nodes[0]->detachObject(cam);
    }
}

bool GameWorld::isCameraAttached(Multibody *bod, Ogre::Camera *cam) const
{
    MultibodyMap::const_iterator it = pimpl->multibodies.find(bod);
    if (it != pimpl->multibodies.end()) {
        for (int i = 0; i < it->second.scene_nodes[0]->numAttachedObjects(); ++i) {
            if (it->second.scene_nodes[0]->getAttachedObject(i) == cam) {
                return true;
            }
        }
    }
    return false;
}

namespace {        
    bool CustomMaterialCombinerCallback(btManifoldPoint &cp,
                                        const btCollisionObject *colObj0,
                                        int partId0, int index0,
                                        const btCollisionObject *colObj1,
                                        int partId1, int index1)
    {
        // Use bullet "internal edge utility", this is to prevent car wheels (and other objects)
        // colliding off of internal edges of triangle meshes.
        if (colObj1->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE) {
            btAdjustInternalEdgeContacts(cp, colObj1, colObj0, partId1, index1);
        } else if (colObj0->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE) {
            std::swap(cp.m_partId0, cp.m_partId1);
            std::swap(cp.m_index0, cp.m_index1);
            std::swap(cp.m_localPointA, cp.m_localPointB);
            std::swap(cp.m_positionWorldOnA, cp.m_positionWorldOnB);
            cp.m_normalWorldOnB = -cp.m_normalWorldOnB;
            
            btAdjustInternalEdgeContacts(cp, colObj0, colObj1, partId0, index0);
            
            std::swap(cp.m_partId0, cp.m_partId1);
            std::swap(cp.m_index0, cp.m_index1);
            std::swap(cp.m_localPointA, cp.m_localPointB);
            std::swap(cp.m_positionWorldOnA, cp.m_positionWorldOnB);
            cp.m_normalWorldOnB = -cp.m_normalWorldOnB;
        }

        return false; // We don't calculate friction
    }

    const std::pair<Multibody *, int> NULL_BODY(0, -1);
}

extern ContactAddedCallback gContactAddedCallback;

void GameWorld::applyGravity(const Vector3r &g)
{
    // Clears all forces and torques, then applies gravity to all bodies.
    for (MultibodyMap::const_iterator it = pimpl->multibodies.begin(); it != pimpl->multibodies.end(); ++it) {
        it->first->clearForcesAndTorques();
        it->first->addBaseForce(g * it->first->getBaseMass());

        for (int j = 0; j < it->first->getNumLinks(); ++j) {
            it->first->addLinkForce(j, g * it->first->getLinkMass(j));
        }
    }
}

void GameWorld::doTimestep(real_t dt, 
                           std::vector<real_t> &scratch_r, std::vector<Vector3r> &scratch_v,
                           std::vector<Matrix3r> &scratch_m,
                           double &c_time, double &p_time, double &s_time, double &t_time, int &nconstr)
{
    pimpl->global_time += dt;

    // Setup the collision callbacks. We will get notified whenever a manifold point is deleted. We can
    // then delete the corresponding Constraint object
    gContactDestroyedCallback = &DeleteConstraint;
    gContactAddedCallback = CustomMaterialCombinerCallback;
    
    LARGE_INTEGER Freq;
    QueryPerformanceFrequency(&Freq);

    // Do collision detection
    LARGE_INTEGER Count1, Count2;
    QueryPerformanceCounter(&Count1);

    pimpl->collision_world.performDiscreteCollisionDetection();
    const int num_manifolds = pimpl->collision_world.getDispatcher()->getNumManifolds();

    QueryPerformanceCounter(&Count2);

    c_time = double(Count2.QuadPart - Count1.QuadPart) / double(Freq.QuadPart);

    Count1 = Count2;

#ifdef USE_SLEEPING
    // Wake up any sleeping bodies that need to be woken.
    while (true) {
        bool woke_something = false;
        for (int i = 0; i < num_manifolds; ++i) {
            
            btPersistentManifold * contact_manifold = pimpl->collision_world.getDispatcher()->getManifoldByIndexInternal(i);
            btCollisionObject * obj_0 = static_cast<btCollisionObject*>(contact_manifold->getBody0());
            btCollisionObject * obj_1 = static_cast<btCollisionObject*>(contact_manifold->getBody1());
            
            const std::pair<Multibody *, int> * bod_0 = static_cast<std::pair<Multibody*, int> *>(obj_0->getUserPointer());
            const std::pair<Multibody *, int> * bod_1 = static_cast<std::pair<Multibody*, int> *>(obj_1->getUserPointer());

            const bool awake_0 = bod_0 && bod_0->first->isAwake();
            const bool awake_1 = bod_1 && bod_1->first->isAwake();

            if (awake_0 && !awake_1 && bod_1) {
                bod_1->first->wakeUp();
                woke_something = true;
            }
            if (awake_1 && !awake_0 && bod_0) {
                bod_0->first->wakeUp();
                woke_something = true;
            }
        }

        if (!woke_something) break;  // Iteration complete
    }
#endif

    // temporary vector of constraints, used below for the SequentialImpulses call
    // initially contains the non-contact constraints only (will add contact constraints below)
    std::vector<Constraint*> constraints;
    constraints.reserve(pimpl->joint_limit_constraints.size());
    std::copy(pimpl->joint_limit_constraints.begin(),
              pimpl->joint_limit_constraints.end(),
              std::back_inserter(constraints));
    if (pimpl->bridge_constraint.get()) {
        constraints.push_back(pimpl->bridge_constraint.get());
    }

    // At this point (following Bullet collision detection)
    // 'old' contact constraints have been deleted, but have to create 'new' ones and
    // update 'existing' ones.
    for (int i = 0; i < num_manifolds; ++i) {
        btPersistentManifold * contact_manifold = pimpl->collision_world.getDispatcher()->getManifoldByIndexInternal(i);
        btCollisionObject * obj_0 = static_cast<btCollisionObject*>(contact_manifold->getBody0());
        btCollisionObject * obj_1 = static_cast<btCollisionObject*>(contact_manifold->getBody1());

        const std::pair<Multibody *, int> * bod_0 = static_cast<std::pair<Multibody*, int> *>(obj_0->getUserPointer());
        const std::pair<Multibody *, int> * bod_1 = static_cast<std::pair<Multibody*, int> *>(obj_1->getUserPointer());
 
        if (!bod_0 && !bod_1) continue;  // Collision between two 'fixed' objects. Ignore.

        // Ignore collisions between sleeping objects.
        if ((!bod_0 || !bod_0->first->isAwake())
            && (!bod_1 || !bod_1->first->isAwake())) continue;

        bool swapped = false;
        if (!bod_0) {
            // The physics engine assumes that if one of the bodies is null, it is the second.
            // Therefore we must swap the bodies, if the first is null but the second isn't.
            std::swap(bod_0, bod_1);
            swapped = true;
        }

        // If bod_1 exists and is a wheel, then also need to swap, as WheelContactConstraint
        // assumes the wheel is body 0
        WheelInfoMap::const_iterator wheel_find;
        if (bod_1) {
            wheel_find = pimpl->wheel_info_map.find(*bod_1);
            if (wheel_find != pimpl->wheel_info_map.end()) {
                std::swap(bod_0, bod_1);
                swapped = true;
            } else {
                wheel_find = pimpl->wheel_info_map.find(*bod_0);
            }
        } else {
            wheel_find = pimpl->wheel_info_map.find(*bod_0);
        }

        // Filter out self collisions if one of the colliders is a wheel
        if (wheel_find != pimpl->wheel_info_map.end() && bod_0 && bod_1 && bod_0->first == bod_1->first) {
            continue;
        }
        
        if (!bod_0) bod_0 = &NULL_BODY;
        if (!bod_1) bod_1 = &NULL_BODY;

        const int num_contacts = contact_manifold->getNumContacts();

        for (int j = 0; j < num_contacts; ++j) {
        
            btManifoldPoint * manifold_point = & contact_manifold->getContactPoint(j);

            const btVector3 & point_a = manifold_point->getPositionWorldOnA();
            const btVector3 & point_b = manifold_point->getPositionWorldOnB();
            const btVector3 & bnormal = manifold_point->m_normalWorldOnB;

            Vector3r cp0, cp1, normal;
            if (swapped) {
                for (int i = 0; i < 3; ++i) {
                    cp0[i] = point_b[i];
                    cp1[i] = point_a[i];
                    normal[i] = bnormal[i];
                }
            } else {
                for (int i = 0; i < 3; ++i) {
                    cp0[i] = point_a[i];
                    cp1[i] = point_b[i];
                    normal[i] = -bnormal[i];
                }
            }
                
            // transform to local frames
            cp0 = bod_0->first->worldPosToLocal(bod_0->second, cp0);
            if (bod_1->first) {
                cp1 = bod_1->first->worldPosToLocal(bod_1->second, cp1);
                normal = bod_1->first->worldDirToLocal(bod_1->second, normal);
            }

            if (manifold_point->m_userPersistentData == 0) {
                // 'new' contact
                real_t friction_coeff, restitution_coeff, baumgarte;
                pimpl->getMaterialProperties(bod_0->first, bod_1->first, friction_coeff, restitution_coeff, baumgarte);

                // Create the contact point (either normal contact or wheel contact)
                Constraint *con;
                if (wheel_find != pimpl->wheel_info_map.end()) {
                    con = new WheelContactConstraint(bod_0->first, bod_1->first, 
                                                     bod_0->second, bod_1->second,
                                                     cp0,
                                                     cp1,
                                                     normal,
                                                     friction_coeff,
                                                     restitution_coeff,
                                                     wheel_find->second.axis,
                                                     wheel_find->second.radius);
                } else {
                    con = new ContactConstraint(bod_0->first, bod_1->first, 
                                                bod_0->second, bod_1->second,
                                                cp0,
                                                cp1,
                                                normal,
                                                friction_coeff,
                                                restitution_coeff);
                }
                con->setBaumgarteParams(baumgarte, con->getBaumgarteEnvelope());
                manifold_point->m_userPersistentData = con;
            } else {
                // 'existing' contact
                ContactConstraint * con = static_cast<ContactConstraint *>(manifold_point->m_userPersistentData);
                con->setContactPoints(cp0,
                                      cp1,
                                      normal);
                
                if (pimpl->force_update_contacts) { 
                    // material properties (friction, restitution) have changed, so need to update
                    real_t friction_coeff, restitution_coeff, baumgarte;
                    pimpl->getMaterialProperties(bod_0->first, bod_1->first, friction_coeff, restitution_coeff, baumgarte);
                    con->setFrictionCoeff(friction_coeff);
                    con->setRestitutionCoeff(restitution_coeff);
                    con->setBaumgarteParams(baumgarte, con->getBaumgarteEnvelope());
                }
            }

            Constraint * con = static_cast<Constraint *>(manifold_point->m_userPersistentData);
            constraints.push_back(con);
        }
    }

    // Raycasts
    for (std::vector<Raycast>::iterator it = pimpl->raycasts.begin(); it != pimpl->raycasts.end(); ++it) {

        Multibody *body = it->body;

        // convert ray start/end points to world frame
        const Vector3r start_point = body->localPosToWorld(it->ref_link, it->start_point);
        const Vector3r end_point = body->localPosToWorld(it->ref_link, it->end_point);

        const btVector3 b_start_point(start_point.x(), start_point.y(), start_point.z());
        const btVector3 b_end_point(end_point.x(), end_point.y(), end_point.z());
        
        // shoot the ray; find first collision with something
        btCollisionWorld::ClosestRayResultCallback result(b_start_point, b_end_point);
        pimpl->collision_world.rayTest(b_start_point, b_end_point, result);

        if (result.m_closestHitFraction < 1.f) {
            // Extract hit position and normal
            Vector3r hit_point(result.m_hitPointWorld.x(), result.m_hitPointWorld.y(), result.m_hitPointWorld.z());
            Vector3r hit_normal(-result.m_hitNormalWorld.x(), -result.m_hitNormalWorld.y(), -result.m_hitNormalWorld.z());

            Vector3r ray_axis = (end_point - start_point).normalized();

            // Extract the other body (will be body "B" of the constraint)
            const std::pair<Multibody*, int> * other_bod =
                static_cast<std::pair<Multibody*, int>*>(result.m_collisionObject->getUserPointer());
            if (!other_bod) other_bod = &NULL_BODY;

            // Convert contact points to local frames
            Vector3r cp1 = body->worldPosToLocal(it->force_link, end_point);
            Vector3r cp2, n;
            if (other_bod->first) {
                cp2 = other_bod->first->worldPosToLocal(other_bod->second, hit_point);
                n = other_bod->first->worldDirToLocal(other_bod->second, hit_normal);
            } else {
                cp2 = hit_point;
                n = hit_normal;
            }

            // Make the constraint
            if (it->constraint
            && it->constraint->getBody2() == other_bod->first
            && it->constraint->getLink2() == other_bod->second
            && !pimpl->force_update_contacts) {
                // It is the same constraint between the same bodies, so can just reset the contact points
                it->constraint->setContactPoints(cp1, cp2, n);
            } else {
                if (it->constraint) {
                    // Need to delete the old constraint because the other body/link has changed
                    delete it->constraint;
                }
                real_t friction_coeff, restitution_coeff, baumgarte;
                pimpl->getMaterialProperties(body, other_bod->first, friction_coeff, restitution_coeff, baumgarte);
                it->constraint = new WheelContactConstraint(body, other_bod->first, it->force_link, other_bod->second,
                                                            cp1, cp2, n,
                                                            friction_coeff, restitution_coeff,
                                                            it->axis,
                                                            it->radius);
                it->constraint->setBaumgarteParams(baumgarte, it->constraint->getBaumgarteEnvelope());
            }

            constraints.push_back(it->constraint);
        }
    }

    pimpl->force_update_contacts = false;


    // OK, the constraints are now set up correctly.

    nconstr = 0;
    for (std::vector<Constraint*>::const_iterator it = constraints.begin(); it != constraints.end(); ++it) {
        nconstr += (*it)->getNumRows();
    }

    // Create a bodies vector
    std::vector<Multibody*> bodies;
    bodies.reserve(pimpl->multibodies.size());
    for (MultibodyMap::const_iterator it = pimpl->multibodies.begin(); it != pimpl->multibodies.end(); ++it) {
        bodies.push_back(it->first);
    }


    QueryPerformanceCounter(&Count2);
    s_time = double(Count2.QuadPart - Count1.QuadPart) / double(Freq.QuadPart);

    Count1 = Count2;


    // Do the timestep

    if (!bodies.empty()) {

        // update all constraints
        for (std::vector<Constraint*>::iterator it = constraints.begin();
        it != constraints.end(); ++it) {
            (*it)->update(scratch_r, scratch_v, scratch_m);
        }

        // calculate Bvector for sequential impulses
        std::vector<real_t> Bvector;
        CalculateBVector(&constraints[0], constraints.size(), dt, Bvector);
        
        // do velocity step
        for (size_t i = 0; i < bodies.size(); ++i) {
            if (bodies[i]->isAwake()) {
                bodies[i]->stepVelocities(dt, scratch_r, scratch_v, scratch_m);
            }
        }

        // do sequential impulses
        SequentialImpulses(&constraints[0], constraints.size(), dt, Bvector, scratch_r, scratch_v);

        // do position step
        for (size_t i = 0; i < bodies.size(); ++i) {
            if (bodies[i]->isAwake()) {
                bodies[i]->stepPositions(dt);
            }
        }
    }

#ifdef DUMP_CONSTRAINT_INFO
    std::ofstream str("E:/constraint_info.txt", std::ios::out | std::ios::app);
    str.precision(3);
    const int LINK_NUM = 1;

    str << pimpl->global_time << "\t";

    for (std::vector<Constraint*>::const_iterator it = constraints.begin(); it != constraints.end(); ++it) {
        if ((*it)->getLink1() == LINK_NUM) {
            Vector3r contact_point = (*it)->contact_point;
            Vector3r force = Vector3r::Zero();
            for (int i = 0; i < (*it)->getNumRows(); ++i) {
                force += Eigen::Map<Vector3r>((*it)->force1(i)) * (*it)->getConstraintImpulse(i);
            }

            // transform to base frame
            contact_point -= (*it)->getBody1()->getBasePos();
            contact_point = (*it)->getBody1()->getWorldToBaseRot() * contact_point;
            force = (*it)->getBody1()->getWorldToBaseRot() * force;

            // transform to link frame
            for (int i = 0; i <= LINK_NUM; ++i) {
                contact_point = (*it)->getBody1()->getParentToLocalRot(i) * contact_point;
                force = (*it)->getBody1()->getParentToLocalRot(i) * force;
                contact_point -= (*it)->getBody1()->getRVector(i);
            }

            // Print the result
            str << contact_point[0] << "\t";  // longitudinal position.
            for (int i = 0; i < 3; ++i) {
                str << force[i] << "\t";  // total force (both NR and friction).
            }
        }
    }
    str << "\n";
#endif

#ifdef USE_SLEEPING
    // Put bodies to sleep if necessary
    for (MultibodyMap::iterator it = pimpl->multibodies.begin(); it != pimpl->multibodies.end(); ++it) {
        if (it->first->isAwake()) {
            it->first->checkMotionAndSleepIfRequired(dt);
        }
    }
#endif

    QueryPerformanceCounter(&Count2);
    p_time = double(Count2.QuadPart - Count1.QuadPart) / double(Freq.QuadPart);

    // Update all the bullet collision objects & ogre scene nodes to reflect the new body positions
    QueryPerformanceCounter(&Count1);

    std::vector<Quaternionr> q_temp;
    for (MultibodyMap::iterator it = pimpl->multibodies.begin(); it != pimpl->multibodies.end(); ++it) {
        UpdateBulletAndOgreTransforms(it, q_temp, scratch_v);
    }

    QueryPerformanceCounter(&Count2);
    t_time = double(Count2.QuadPart - Count1.QuadPart) / double(Freq.QuadPart);


#ifdef CONTACT_FORCE_DEBUGGING
    debug_graphics->clear();
    if (pimpl->draw_contact_forces) {
        for (std::vector<Constraint*>::const_iterator it = constraints.begin(); it != constraints.end(); ++it) {
            Vector3r contact_point, normal;
            if ((*it)->getDebugGfx(contact_point, normal)) {
                if (fabs((*it)->getConstraintImpulse(0)) > 1e-6) {
                    debug_graphics->setColour(Ogre::ColourValue(1, 0, 0));
                } else {
                    debug_graphics->setColour(Ogre::ColourValue(0, 1, 0));
                }
                Vector3r ep1(contact_point);
                Ogre::Vector3 p1(ep1.x(), ep1.y(), ep1.z());
                const real_t imp = (*it)->getConstraintImpulse(0);
                const real_t force = imp / dt;
                Vector3r ep2(contact_point - normal * log(std::max(force, real_t(1)))/10);
                Ogre::Vector3 p2(ep2.x(), ep2.y(), ep2.z());
                debug_graphics->addPoint(p1);
                debug_graphics->addLine(p1, p2);
            }
        }
    }
#endif
}

real_t GameWorld::getGlobalTime() const
{
    return pimpl->global_time;
}

void GameWorld::setPositionAdjustment(Multibody *bod, const Vector3r &pos)
{
    MultibodyMap::iterator it = pimpl->multibodies.find(bod);
    if (it != pimpl->multibodies.end()) {

        // update Ogre scene node position
        it->second.scene_nodes[0]->getChild("CAR_COM_HACK")->setPosition(-pos[0], -pos[1], -pos[2]);

        // update Bullet collision object position
        // (we assume it is a btCompoundShape, this is why it only works with CuboidShape2 at the moment)
        static_cast<btCompoundShape*>(it->second.collision_objects[0]->getCollisionShape())->updateChildTransform(
            0, btTransform(btQuaternion(0,0,0,1), btVector3(-pos[0], -pos[1], -pos[2])));
    }
}

#ifdef CONTACT_FORCE_DEBUGGING
void GameWorld::setDrawContactForces(bool draw)
{
    pimpl->draw_contact_forces = draw;
}
#endif
