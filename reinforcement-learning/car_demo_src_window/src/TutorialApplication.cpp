/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.cpp
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _ 
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/                              
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/

/*
  Modified by Stephen Thompson, August 2010
*/

#include "TutorialApplication.h"

#include "multibody.hpp"
#include "road_builder.hpp"

#ifdef CONTACT_FORCE_DEBUGGING
#include "DebugGraphics.h"
#endif

#define USE_CAR
#define USE_RAYCAST
//#define DISPLAY_STEERING_INFO

#ifdef USE_RAYCAST
#define RAYCAST_CYLINDER_SHAPE setShapeDummyCylinder
#else
#define RAYCAST_CYLINDER_SHAPE setShapeCylinder
#endif

namespace {
    const real_t PI = 4*atan(real_t(1));

    const real_t BALL_DENSITY = real_t(0.25);
    const real_t OBJECT_DENSITY = real_t(1.0);

    const real_t METRES_PER_SEC_TO_MPH = real_t(2.23693629);  // Source: Google
}

//-------------------------------------------------------------------------------------
TutorialApplication::TutorialApplication(void)
    : time(0), pause(false), single_step(false), mTimePanel(0), car(0), accelerator(false), brake(false),
    steer_left(false), steer_right(false), steering(0), 
    
    mPhysicsSettingsButton(0), 
    mResetSimulationButton(0),
    mHelpButton(0), mQuitButton(0),

    mGravitySlider(0), mFrictionSlider(0), mRestitutionSlider(0),
    mDriveTypeMenu(0), mEngineTorqueSlider(0), mBrakeTorqueSlider(0), mBrakeBalanceSlider(0),
    mTyreFrictionSlider(0),
    mCoMHeightSlider(0), mRideHeightSlider(0),
    mSuspensionTravelSlider(0),
    mSuspensionStiffnessSlider(0), mSuspensionDampingSlider(0),
    mRestorePhysicsButton(0), mClosePhysicsButton(0),
    mShowForcesCheckBox(0),
    mPhysSep1(0), mPhysSep2(0), mPhysSep3(0),
    mWireframeCheckBox(0),
    mHelpText(0), mCloseHelpButton(0),
    mPhysicsTab(0), mCarTab(0), mGraphicsTab(0),
    
    mGravity(0), mEngineTorque(0), mBrakeTorque(0), mBrakeBalance(0),
    mCoMHeight(0), mRideHeight(0), mSuspensionTravel(0), mSuspensionStiffness(0), mSuspensionDamping(0),

    mDriveType(DRIVE_4WD)
{   
}
//-------------------------------------------------------------------------------------
TutorialApplication::~TutorialApplication(void)
{
}


//-------------------------------------------------------------------------------------

namespace {

    // Some car parameters
    const real_t WHEEL_RADIUS = real_t(0.28);
    const real_t WHEEL_GAP = real_t(0.04);  // Minimum allowed clearance between top of wheel and bottom of car body

    const real_t CAR_HEIGHT = real_t(0.7);     // Dimensions of the car body itself (excluding wheels/springs)
    const real_t CAR_WIDTH = 2;
    const real_t CAR_LENGTH = 4;
    const real_t WHEELBASE = 3;

    // GUI slider min/max values
    const real_t CAR_MASS = real_t(3) * CAR_WIDTH * CAR_HEIGHT * CAR_LENGTH;

    const real_t MIN_CAR_COM = 0;
    const real_t MAX_CAR_COM = 2;
    const int CAR_COM_SNAPS = 101;

    const real_t MAX_ENGINE_TORQUE = 20;
    const int ENGINE_TORQUE_SNAPS = 41;
    const real_t MAX_BRAKE_TORQUE_SLIDER = MAX_ENGINE_TORQUE;
    const int BRAKE_TORQUE_SNAPS = ENGINE_TORQUE_SNAPS;
    
    const real_t MIN_SUSPENSION_STIFFNESS = 10.0f;
    const real_t MAX_SUSPENSION_STIFFNESS = 200.0f;
    const int SUSPENSION_STIFFNESS_SNAPS = 96;
    const real_t MAX_SUSPENSION_DAMPING = 1.2f;    // On a scale where 1 = critical. NB values > about 1.2 give instability at 100 Hz timestep.
    const int SUSPENSION_DAMPING_SNAPS = 61;


    // the following have to be kept synchronized with the car creation code
    const int STEERING_JOINT = 0;
    const int LEFT_FRONT_WHEEL = 2;
    const int RIGHT_FRONT_WHEEL = 4;
    const int LEFT_REAR_WHEEL = 6;
    const int RIGHT_REAR_WHEEL = 8;

    const int NUM_WHEELS = 4;
    const int WHEELS[] = { LEFT_FRONT_WHEEL, RIGHT_FRONT_WHEEL, LEFT_REAR_WHEEL, RIGHT_REAR_WHEEL };
    

    std::string FormatNumber(float num, int decimal_places)
    {
        std::ostringstream str;
        str.setf(std::ios::fixed); 
        str.precision(decimal_places);
        str << num;
        return str.str();
    }

    void SetupPyramid(GameWorld &gw,
                      int pyramid_size, real_t cube_density, real_t cube_size,
                      const Vector3r &pos,
                      float r1, float g1, float b1,
                      float r2, float g2, float b2)
    {   
        for (int row = 0; row < pyramid_size; ++row) {
            for (int col = 0; col < pyramid_size - row; ++col) {
                const real_t x = (2*col + row - pyramid_size) * cube_size * real_t(0.55) + pos[0];
                const real_t y = row * cube_size * real_t(1.1) + cube_size * real_t(0.55) + pos[1];
                const real_t z = pos[2];
                gw.beginObjectMultibody(Vector3r(x,y,z), Quaternionr::Identity(), false, true, false);
                const int i = row+col;
                const float c1 = (i%2==0) ? r1 : r2;
                const float c2 = (i%2==0) ? g1 : g2;
                const float c3 = (i%2==0) ? b1 : b2;
                gw.setShapeCuboid(cube_density, cube_size, cube_size, cube_size, c1, c2, c3);
                gw.endObject();
            }
        }
    }

    void SetupBridge(GameWorld *gw, const Vector3r &base_pos)
    {
        const int    num_planks = 10;
        
        const real_t bridge_width = 15;

        const real_t block_length = 30;
        const real_t block_height = 15;
        
        const real_t plank_length = 9;
        const real_t plank_height = real_t(0.1);

        const real_t gap = real_t(0.05);
        
        // set can_sleep = false for bridge, because it can gently sway for quite a long time and it looks
        // odd if that just stops suddenly due to sleeping.
        gw->beginObjectMultibody(base_pos + Vector3r(0,block_height/2,0), Quaternionr::Identity(), true, false, false);
        gw->setShapeCuboid(OBJECT_DENSITY, bridge_width, block_height, block_length, 1, 1, 1);

        for (int i = 0; i < num_planks; ++i) {
            real_t angle = real_t(0.3);
            if (i > 0) angle = real_t(-0.06);
            gw->beginChildRevolute(Vector3r(0, 
                                            (i==0 ? block_height/2-plank_height/2 : 0), 
                                            gap + plank_length + (i==0? block_length/2-plank_length/2 : 0)),
                                   Quaternionr::Identity(),
                                   Vector3r(1, 
                                            0, 
                                            0),
                                   Vector3r(0, 
                                            (i==0 ? block_height/2 - plank_height/2 : 0),
                                            (i==0 ? block_length/2 + gap/2 : plank_length/2 + gap/2)),
                                   angle);
            gw->setShapeCuboid(OBJECT_DENSITY, bridge_width, plank_height, plank_length, 0, 1, 1);
        }

        for (int i = 0; i < num_planks; ++i) gw->endChild();
        Multibody *bod = gw->endObject();

        const real_t endpoint_z = block_length/2 + (plank_length+gap)*(num_planks-real_t(0.15));

        gw->setupBridgeConstraint(bod,
                                  base_pos + Vector3r(0,
                                                      block_height,
                                                      endpoint_z - real_t(0.01)),
                                  Vector3r(0,
                                           0, 
                                           -plank_length/2 - gap/2), 
                                  Vector3r(0,
                                           0,
                                           plank_length/2));

        // ramp leading up to bridge
        const real_t slope_angle = real_t(0.2);
        gw->beginObjectFixed(base_pos + Vector3r(0,
                                                 block_height/2 - real_t(0.5), 
                                                 -block_length/2 - block_height/tan(slope_angle)/2),
                             Quaternionr(AngleAxisr(slope_angle, Vector3r(-1,0,0))));
        gw->setShapeCuboid(OBJECT_DENSITY, 
                           bridge_width,
                           1, 
                           block_height/sin(slope_angle),
                           1, 1, 0);
        gw->endObject();

        // ramp leading down other side
        gw->beginObjectFixed(base_pos + Vector3r(0,
                                                 block_height/2 - real_t(0.5),
                                                 endpoint_z + block_length + block_height/tan(slope_angle)/2),
                             Quaternionr(AngleAxisr(slope_angle, Vector3r(1,0,0))));
        gw->setShapeCuboid(OBJECT_DENSITY,
                           bridge_width,
                           1,
                           block_height/sin(slope_angle),
                           1, 1, 0);
        gw->endObject();

        gw->beginObjectFixed(base_pos + Vector3r(0,
                                                 block_height/2,
                                                 endpoint_z + block_length/2),
                             Quaternionr::Identity());
        gw->setShapeCuboid(OBJECT_DENSITY,
                           bridge_width, block_height, block_length,
                           1, 1, 1);
        gw->endObject();
    }

    void SetupJump(GameWorld &gw,
                   const Vector3r &pos,
                   real_t angle,
                   real_t width,
                   real_t height,
                   real_t gap)
    {
        const real_t x_disp = gap + height/tan(angle);

        gw.beginObjectFixed(pos + Vector3r(-x_disp/2, height/2 - real_t(0.5), 0),
                            Quaternionr(AngleAxisr(angle, Vector3r(0, 0, 1))));
        gw.setShapeCuboid(1, height/sin(angle), 1, width, 1, 1, 0);
        gw.endObject();

        gw.beginObjectFixed(pos + Vector3r(x_disp/2, height/2 - real_t(0.5), 0),
                            Quaternionr(AngleAxisr(angle, Vector3r(0, 0, -1))));
        gw.setShapeCuboid(1, height/sin(angle), 1, width, 1, 1, 0);
        gw.endObject();
    }

    void SetupCube(GameWorld &gw,
                   real_t x,
                   real_t z,
                   real_t rotation,
                   bool even)
    {
        const real_t CUBE_SIZE = real_t(1.2);

        real_t r = 1;
        real_t g = even ? 1 : real_t(0.3);
        real_t b = g;
        gw.beginObjectMultibody(Vector3r(x, CUBE_SIZE/2, z), 
                                Quaternionr(AngleAxisr(rotation, Vector3r(0, -1, 0))),
                                false, true, true);
        gw.setShapeCuboid(real_t(0.1), CUBE_SIZE, CUBE_SIZE, CUBE_SIZE, r, g, b);
        gw.endObject();
    }

    void SetupTurn(GameWorld &gw,
                   real_t x, real_t z, real_t r,
                   real_t bmin, real_t bmax)
    {
        typedef Eigen::Matrix<real_t, 2, 1> Vector2r;

        const real_t road_width = 20;
        const int NUM_CUBES = int((bmax - bmin) / 10 + real_t(0.5));

        const Vector2r centre_point(x, z);

        bool even = true;
        for (int i = 0; i < NUM_CUBES; ++i, even = !even) { 
            real_t bearing = bmin + real_t(i) / real_t(NUM_CUBES-1) * (bmax - bmin);
            bearing *= PI / 180;  // degrees to radians
            const Vector2r displacement(cos(bearing), sin(bearing));
            const Vector2r inner = (r + road_width/2) * displacement + centre_point;
            const Vector2r outer = (r - road_width/2) * displacement + centre_point;

            SetupCube(gw, inner.x(), inner.y(), bearing, even);
            SetupCube(gw, outer.x(), outer.y(), bearing, even);
        }
    }

    void SetupXStraight(GameWorld &gw, real_t xmin, real_t xmax, real_t z)
    {
        bool even = false;
        for (real_t x = xmin; x < xmax + real_t(0.01); x += 20, even = !even) {
            SetupCube(gw, x, z - 10, 0, even);
            SetupCube(gw, x, z + 10, 0, even);
        }
    }

    void SetupPillar(GameWorld &gw,
                        real_t x,
                        real_t z,
                        bool even)
    {
        const real_t CUBE_SIZE = real_t(1.2);

        for (int i = 0; i < 5; ++i, even = !even) {
            real_t r = real_t(0.3);
            real_t g = even ? 1 : real_t(0.3);
            real_t b = even ? real_t(0.3) : 1;

            gw.beginObjectMultibody(Vector3r(x + i * real_t(0.01), 
                                             CUBE_SIZE/2 + real_t(0.02) + i*(CUBE_SIZE + real_t(0.02)), 
                                             z + i * real_t(0.01)),
                                    Quaternionr::Identity(), 
                                    false, true, false);
            gw.setShapeCuboid(real_t(0.1), CUBE_SIZE, CUBE_SIZE, CUBE_SIZE, r, g, b);
            gw.endObject();
        }
    }

    void SetupLight(Ogre::SceneManager *sm, float x, float y, float z, float brightness)
    {
        static int light_num = 0;
        std::ostringstream name;
        name << "Light" << ++light_num;
        Ogre::Light * light = sm->createLight(name.str());
        light->setDiffuseColour(Ogre::ColourValue(brightness, brightness, brightness));
        light->setPosition(x, y, z);
    }

    void SetupRoadCircuit(GameWorld &gw)
    {
        // Create a road circuit using RoadBuilder. NOT USED currently.

        const real_t ht = real_t(0.15);

        const Vector3r
            A( -200, ht,    0),
            B(  200, ht,    0),
            C(  280, ht,  -80),
            D(  280, ht, -660),
            E(  200, ht, -740),
            F( -100, ht, -740),
            G( -100, ht, -720),
            H( -160, ht, -720),
            I( -160, ht, -280),
            J(  -80, ht, -280),
            K(    0, ht, -200),
            L(    0, ht, -148),
            M(    0, ht,  147),
            N(    0, ht,  450),
            O(  -23, ht,  507),
            P(  -46, ht,  564),
            Q(  -46, ht,  630),
            R( -148, ht,  672),
            S( -300, ht,  450),
            T( -400, ht,  200),
            U( -400, ht,  100),
            V( -300, ht,    0);

        {
            RoadBuilder rb(A, PI/2);
            rb.road(B, 0);
            rb.road(D, C.x() - B.x());
            rb.road(F, D.x() - E.x());
            rb.loop(15, G.z() - F.z());
            rb.road(H, 0);
            rb.road(J, (I.z() - H.z())/2);
            rb.road(L, K.z() - J.z());

            gw.beginObjectFixed(Vector3r::Zero(), Quaternionr::Identity());
            gw.setShapeRoad(rb.getLeftPoints(), rb.getRightPoints());
            gw.endObject();
        }

        {
            RoadBuilder rb(M, 0);
            rb.road(N, 0);
            rb.road(O, (P.z() - N.z()) / sqrt(real_t(2)));
            rb.road(Q, (P.z() - N.z()) / sqrt(real_t(2)));
            rb.road(S, (Q.x() - R.x()) / (1 + sqrt(real_t(2))));
            rb.road(T, 100);
            rb.road(U, 40);
            rb.road(V, 100);   // turn to face final straight
            rb.road(A, 10);  // straight

            gw.beginObjectFixed(Vector3r::Zero(), Quaternionr::Identity());
            gw.setShapeRoad(rb.getLeftPoints(), rb.getRightPoints());
            gw.endObject();
        }
    }
}


//-------------------------------------------------------------------------------------
void TutorialApplication::createScene()
{
    mCameraMan->setTopSpeed(30);

    gw.reset(new GameWorld(mSceneMgr));

#ifdef CONTACT_FORCE_DEBUGGING
    Ogre::DebugGraphics * debug_graphics = new Ogre::DebugGraphics(mSceneMgr);
    gw->setDebugGraphics(debug_graphics);
    mRoot->addFrameListener(debug_graphics);
#endif

    // Set up the game world

#ifdef USE_DOUBLE_PRECISION
    // make sure full FPU precision is being used when in double-precision mode
    _controlfp(_CW_DEFAULT, 0xfffff);
#endif


#if 0
    /*
    // Stable stacking (note needs fairly low restitution to be stable)
    const int NUM_CUBES = 10;
    for (int i = 0; i < NUM_CUBES; ++i) {
        gw->beginObjectMultibody(Vector3r(0.01f*i, 3.0f*i + 1.001f, 0.014f*i), Quaternionr::Identity(), false);
        const float c1 = (i%2==0) ? 1.0f : 1.0f;
        const float c2 = (i%2==0) ? 0.3f : 1.0f;
        gw->setShapeCuboid(OBJECT_DENSITY, 2, 2, 2, c1, c2, c2);
        gw->endObject();
    }
    */

    /*
    const int PYRAMID_SIZE = 10;
    for (int row = 0; row < PYRAMID_SIZE; ++row) {
        for (int col = 0; col < PYRAMID_SIZE - row; ++col) {
            const real_t x = (2*col + row - PYRAMID_SIZE) * real_t(0.55);
            const real_t y = row * real_t(1.05) + real_t(0.55);
            const real_t z = 0;
            gw->beginObjectMultibody(Vector3r(x,y,z), Quaternionr::Identity());
            const int i = row+col;
            const float c1 = (i%2==0) ? 1.0f : 1.0f;
            const float c2 = (i%2==0) ? 0.3f : 1.0f;
            gw->setShapeCuboid(OBJECT_DENSITY, 1, 1, 1, c1, c2, c2);
            gw->endObject();
        }
    }
    */

    /*
    // Newton's cradle
    const int NUM_BALLS = 5;
    for (int i = 0; i < NUM_BALLS; ++i) {
        const real_t angle = i<2 ? real_t(-0.7) : 0;
        
        gw->beginObjectMultibody(Vector3r(i * real_t(1.02), 10, 0), Quaternionr::Identity(), true);
        gw->setShapeCuboid(OBJECT_DENSITY, real_t(0.2), real_t(0.2), real_t(0.2), 1, 0, 0);
        gw->beginChildRevolute(Vector3r(0, -5, 0), Quaternionr::Identity(), Vector3r(0,0,1),
                               Vector3r::Zero(), angle);
        gw->setShapeSphere(OBJECT_DENSITY, 0.5, 1.0f, 1.0f, 0.0f);
        gw->endChild();
        gw->endObject();
    }
    */

    /*
    // Complex Pendulum
    gw->beginObjectMultibody(Vector3r(0, 20, 0), Quaternionr::Identity(), true);
    gw->setShapeSphere(OBJECT_DENSITY, real_t(0.4), 0, 1, 0);

    gw->beginChildRevolute(Vector3r(0, -8, 0), Quaternionr::Identity(), Vector3r(0,0,1),
                           Vector3r(0, -6, 0), real_t(0.3));
    gw->setShapeCuboid(OBJECT_DENSITY, 6, 1, 1, 1, 0, 0);
    //gw->setShapeSphere(OBJECT_DENSITY, real_t(0.5), 1, 0, 0);

    for (int i = 0; i < 2; ++i) {
        gw->beginChildRevolute(Vector3r(4*i-2, -3, 0), Quaternionr::Identity(), Vector3r(1,0,0),
                               Vector3r(4*i-2, 0, 0), real_t(0.2*i-0.1));
        gw->setShapeCuboid(OBJECT_DENSITY, 1, 1, 1, 1, 0, 0);

        gw->beginChildRevolute(Vector3r(0, real_t(-3), 0), Quaternionr::Identity(), Vector3r(1,0,0),
                               Vector3r(0, 0, 0), 0);
        gw->setShapeCuboid(OBJECT_DENSITY, 1, 1, 1, 1, 0, 0);

        gw->beginChildRevolute(Vector3r(0, real_t(-3), 0), Quaternionr::Identity(), Vector3r(1,0,0),
                               Vector3r(0,0,0), 0);
        gw->setShapeCuboid(OBJECT_DENSITY, 1, 1, 1, 1, 0, 0);
        gw->endChild();
        gw->endChild();
        gw->endChild();
    }
    
    gw->endChild();
    gw->endObject();
    */
    

    /*
    gw->beginObjectMultibody(Vector3r(0, 30, 0), Quaternionr::Identity());
    gw->setShapeCuboid(OBJECT_DENSITY, 2, 2, 2, 1, 0, 0);
    gw->beginChildRevolute(Vector3r(0, -5, 0), Quaternionr::Identity(),
                           Vector3r(0, 0, 1), Vector3r(0, -2, 0), real_t(0.1));
    //gw->beginChildPrismatic(Vector3r(0, -5, 0), Quaternionr::Identity(), Vector3r(0,1,0), 0);
    gw->setShapeCuboid(OBJECT_DENSITY, 2, 2, 2, 1, 1, 0);
    gw->endChild();
    gw->endObject();
    */
#endif

#if 1
    
    // bridge
    SetupBridge(gw.get(), Vector3r(0, 0, -60));

    SetupXStraight(*gw, -420, 420, 0);  // main straight

    // pillars (down the main straight)
    
    SetupPillar(*gw, -250, -6, true);
    SetupPillar(*gw, -250, 6,  true);

    SetupPillar(*gw, 0, -6, false);
    SetupPillar(*gw, 0, 6,  false);
    
    SetupPillar(*gw, 250, -6, true);
    SetupPillar(*gw, 250, 6,  true);


    SetupTurn(*gw, 450, -110, 110, 270, 90+360);   // turn 1 (left 180 degrees)
    SetupXStraight(*gw, 100, 420, -220);       // "jumps" straight

    // jumps
    const real_t JUMP_WIDTH = 10, JUMP_HEIGHT = 3, JUMP_GAP = 22;
    const real_t JUMP_ANGLE = real_t(0.25);
    SetupJump(*gw, Vector3r(300, 0, -220), JUMP_ANGLE, JUMP_WIDTH, JUMP_HEIGHT, JUMP_GAP);
    SetupJump(*gw, Vector3r(150, 0, -220), JUMP_ANGLE, JUMP_WIDTH, JUMP_HEIGHT, JUMP_GAP);

    SetupTurn(*gw, 70, -150, 70, 180, 270);    // turn 2 (left 90 degrees, into bridge)

    SetupTurn(*gw, -70, 150, 70, 0, 90);       // turn 3 (right 90 degrees, towards the loop)

    SetupXStraight(*gw, -250, -80, 220);   // straight leading into the loop

    // loop
    const real_t LOOP_RADIUS = 15;
    const Vector3r LOOP_POS = Vector3r(-300, LOOP_RADIUS - 1, 240);
    const real_t LOOP_SCREW = real_t(-3);
    const Vector3r LOOP_BLOCK_SIZE = Vector3r(6, 1, 20);
    const int LOOP_SEGMENTS = 32;

    for (int i = 0; i < LOOP_SEGMENTS; ++i) {
        const real_t theta = i * (8*atan(real_t(1))/LOOP_SEGMENTS);
        const Vector3r pos = LOOP_POS + Vector3r(LOOP_RADIUS*sin(theta), -LOOP_RADIUS*cos(theta), LOOP_SCREW*theta);
        gw->beginObjectFixed(pos, Quaternionr(AngleAxisr(theta, Vector3r(0, 0, 1))));
        gw->setShapeCuboid(1, LOOP_BLOCK_SIZE.x(), LOOP_BLOCK_SIZE.y(), LOOP_BLOCK_SIZE.z(), 0, 0, 
            (i%2==0) ? 1 : 0.75f);
        gw->endObject();
    }

    SetupXStraight(*gw, -420, -330, 240);     // straight leading out of the loop

    SetupTurn(*gw, -450, 120, 120, 90, 270);   // turn 4 (right 180 degrees, back towards final straight.)

#endif

#if 0
    SetupCube(*gw, 0, 30, 0, false);
#endif

#ifdef USE_CAR
    // Car
    const real_t WHEEL_DENSITY = OBJECT_DENSITY*5;
    const real_t WHEEL_THICKNESS = real_t(0.2);
    
    const real_t CAR_X = -100, CAR_Y = 3, CAR_Z = 0;

    gw->beginObjectMultibody(Vector3r(CAR_X, CAR_Y, CAR_Z), 
                             Quaternionr(AngleAxisr(PI/2, Vector3r(0,1,0))), 
                             false, false, false);  // The car should never go to sleep
    gw->setShapeCuboid2(OBJECT_DENSITY, 0, CAR_WIDTH, CAR_HEIGHT, CAR_LENGTH, 1, 0, 0);
    
    // steering joint
    gw->beginChildRevolute(Vector3r(0, 0, WHEELBASE/2),
                           Quaternionr::Identity(),
                           Vector3r(0,1,0),
                           Vector3r(0, 0, WHEELBASE/2), 
                           0);

    // left front wheel -- suspension
    gw->beginChildPrismatic(Vector3r::Zero(), Quaternionr::Identity(), Vector3r(0,-1,0), 0);

    // left front wheel -- rotation
    gw->beginChildRevolute(Vector3r(real_t(0.8), 0, 0), Quaternionr::Identity(),
                           Vector3r(1,0,0), Vector3r(real_t(0.8), 0, 0), 0);
    gw->RAYCAST_CYLINDER_SHAPE (WHEEL_DENSITY, WHEEL_RADIUS, WHEEL_THICKNESS, 0.9f, 0.9f, 0.9f);
    gw->endChild();
    
    gw->endChild();

    // right front wheel -- suspension
    gw->beginChildPrismatic(Vector3r::Zero(), Quaternionr::Identity(), Vector3r(0,-1,0), 0);

    // right front wheel -- rotation
    gw->beginChildRevolute(Vector3r(real_t(-0.8),  0, 0), Quaternionr::Identity(),
                           Vector3r(1,0,0), Vector3r(real_t(-0.8), 0, 0), 0);
    gw->RAYCAST_CYLINDER_SHAPE (WHEEL_DENSITY, WHEEL_RADIUS, WHEEL_THICKNESS, 0.9f, 0.9f, 0.9f);
    gw->endChild();

    gw->endChild();

    gw->endChild();  // end steering joint

    // left rear wheel -- suspension
    gw->beginChildPrismatic(Vector3r::Zero(), Quaternionr::Identity(), Vector3r(0,-1,0), 0);

    // left rear wheel -- rotation
    gw->beginChildRevolute(Vector3r(real_t(0.8),  0, -WHEELBASE/2),
                           Quaternionr::Identity(),
                           Vector3r(1,0,0),
                           Vector3r(0,  0, -WHEELBASE/2), 
                           0);
    gw->RAYCAST_CYLINDER_SHAPE (WHEEL_DENSITY, WHEEL_RADIUS, WHEEL_THICKNESS, 0.9f, 0.9f, 0.9f);
    gw->endChild();
    gw->endChild();

    // right rear wheel -- suspension
    gw->beginChildPrismatic(Vector3r::Zero(), Quaternionr::Identity(), Vector3r(0,-1,0), 0);

    // right rear wheel -- rotation
    gw->beginChildRevolute(Vector3r(real_t(-0.8), 0, -WHEELBASE/2),
                           Quaternionr::Identity(),
                           Vector3r(1,0,0),
                           Vector3r(0, 0, -WHEELBASE/2), 
                           0);
    gw->RAYCAST_CYLINDER_SHAPE (WHEEL_DENSITY, WHEEL_RADIUS, WHEEL_THICKNESS, 0.9f, 0.9f, 0.9f);
    gw->endChild();
    
    gw->endChild();

    gw->setMaterial(GameWorld::M_CAR);  // give the tyres higher-than-normal friction.

    car = gw->endObject();

#ifdef USE_RAYCAST
    {
        const real_t x = real_t(0.8);
        const real_t d = real_t(0.01);
        gw->addRaycast(car, LEFT_FRONT_WHEEL-1, LEFT_FRONT_WHEEL, Vector3r(x, WHEEL_RADIUS, 0), Vector3r(x, -WHEEL_RADIUS-d, 0), Vector3r::UnitX(), WHEEL_RADIUS);
        gw->addRaycast(car, RIGHT_FRONT_WHEEL-1, RIGHT_FRONT_WHEEL, Vector3r(-x, WHEEL_RADIUS, 0), Vector3r(-x, -WHEEL_RADIUS-d, 0), Vector3r::UnitX(), WHEEL_RADIUS);
        gw->addRaycast(car, LEFT_REAR_WHEEL-1, LEFT_REAR_WHEEL, Vector3r(x, WHEEL_RADIUS, -WHEELBASE/2), Vector3r(x, -WHEEL_RADIUS-d, -WHEELBASE/2), Vector3r::UnitX(), WHEEL_RADIUS);
        gw->addRaycast(car, RIGHT_REAR_WHEEL-1, RIGHT_REAR_WHEEL, Vector3r(-x, WHEEL_RADIUS, -WHEELBASE/2), Vector3r(-x, -WHEEL_RADIUS-d, -WHEELBASE/2), Vector3r::UnitX(), WHEEL_RADIUS);
    }
#endif
    
    // Tell physics engine about the wheels, because they need special handling during collision detection.
    std::vector<int> wheel_list;
    for (int i = 0; i < NUM_WHEELS; ++i) wheel_list.push_back(WHEELS[i]);
    gw->setupWheels(car, wheel_list, Vector3r(1,0,0), WHEEL_RADIUS);

    // Steering joint limits
    gw->setupJointLimit(car, STEERING_JOINT, real_t(0), real_t(0));
#endif


    // Ground plane
    gw->beginObjectFixed(Vector3r(0,0,0), Quaternionr::Identity());
    gw->setShapeGroundPlane();
    gw->endObject();

#if 0
    // Setup road circuit.
    SetupRoadCircuit(*gw);
#endif

    // Setup some ogre lighting
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));
    SetupLight(mSceneMgr, -100, 100, 25, 0.5f);
    SetupLight(mSceneMgr, 500, 100, -200, 0.2f);
    SetupLight(mSceneMgr, -500, 100, 200, 0.2f);
    

    // Make sure values from sliders have been put back into the simulation
    readValuesFromSliders();

    // Attach camera to the car.
    chaseCamera();
}

void TutorialApplication::readValuesFromSliders()
{
    if (mGravitySlider) mGravity = mGravitySlider->getValue();
    if (mFrictionSlider) gw->setFriction(mFrictionSlider->getValue());
    if (mRestitutionSlider) gw->setRestitution(mRestitutionSlider->getValue());

    if (mDriveTypeMenu) {
        if (mDriveTypeMenu->getSelectedItem() == "4 Wheel Drive") {
            mDriveType = DRIVE_4WD;
        } else if (mDriveTypeMenu->getSelectedItem() == "Rear Wheel Drive") {
            mDriveType = DRIVE_RWD;
        } else {
            mDriveType = DRIVE_FWD;
        }
    }
    if (mEngineTorqueSlider) mEngineTorque = mEngineTorqueSlider->getValue();
    if (mBrakeTorqueSlider) mBrakeTorque = mBrakeTorqueSlider->getValue();
    if (mBrakeBalanceSlider) mBrakeBalance = mBrakeBalanceSlider->getValue();
    if (mTyreFrictionSlider) gw->setTyreFriction(mTyreFrictionSlider->getValue());

    real_t old_cam_height = mRideHeight - mCoMHeight;
    if (mCoMHeightSlider) mCoMHeight = mCoMHeightSlider->getValue();
    if (mRideHeightSlider) mRideHeight = mRideHeightSlider->getValue();
    if (mSuspensionTravelSlider) mSuspensionTravel = mSuspensionTravelSlider->getValue();
    if (mSuspensionStiffnessSlider) mSuspensionStiffness = mSuspensionStiffnessSlider->getValue();
    if (mSuspensionDampingSlider) mSuspensionDamping = mSuspensionDampingSlider->getValue();

    if (car) {

        // adjust car centre of mass position
        gw->setPositionAdjustment(car, Vector3r(0, mCoMHeight - mRideHeight - CAR_HEIGHT/2, 0));

        // adjust camera position to match change in ride height (if applicable)
        real_t new_cam_height = mRideHeight - mCoMHeight;
        real_t cam_shift = new_cam_height - old_cam_height;
        if (cam_shift != 0 && gw->isCameraAttached(car, mCamera)) {
            mCamera->setPosition(mCamera->getPosition() + Ogre::Vector3(0, cam_shift, 0));
        }
    }

    // adjust suspension ranges
    if (car) {
        real_t q_ride = mCoMHeight - WHEEL_RADIUS;
        real_t q_max = q_ride + mSuspensionTravel / 2;
        real_t q_min = std::max(q_ride - mSuspensionTravel / 2, 
                                WHEEL_RADIUS + WHEEL_GAP + mCoMHeight - mRideHeight); // prevent wheel scraping against bottom of car body

        gw->setupJointLimit(car, LEFT_FRONT_WHEEL-1, q_min, q_max);
        gw->setupJointLimit(car, RIGHT_FRONT_WHEEL-1, q_min, q_max);
        gw->setupJointLimit(car, LEFT_REAR_WHEEL-1, q_min, q_max);
        gw->setupJointLimit(car, RIGHT_REAR_WHEEL-1, q_min, q_max);
    }

#ifdef CONTACT_FORCE_DEBUGGING
    if (mShowForcesCheckBox) gw->setDrawContactForces(mShowForcesCheckBox->isChecked());
#endif
}

void TutorialApplication::writeDefaultsToSliders()
{
    mGravitySlider->setValue(10.0f);
    mFrictionSlider->setValue(0.5f);
    mRestitutionSlider->setValue(0.0f);

    mDriveTypeMenu->selectItem("4 Wheel Drive");
    mEngineTorqueSlider->setValue(7.25f);
    mBrakeTorqueSlider->setValue(12.0f);
    mBrakeBalanceSlider->setValue(0.5f);
    mTyreFrictionSlider->setValue(1.0f);

    mCoMHeightSlider->setValue(0.38f);
    mRideHeightSlider->setValue(0.63f);
    mSuspensionTravelSlider->setValue(0.06f);
    mSuspensionStiffnessSlider->setValue(160.0f);
    mSuspensionDampingSlider->setValue(1.0f);

#ifdef CONTACT_FORCE_DEBUGGING
    mShowForcesCheckBox->setChecked(false);
#endif
}


void TutorialApplication::cockpitCamera()
{
    gw->attachCamera(car, mCamera);
    mCamera->setPosition(0, mRideHeight - mCoMHeight + CAR_HEIGHT + 0.5f, 0.5f);
    mCamera->setOrientation(Ogre::Quaternion(Ogre::Radian(PI), Ogre::Vector3(0,1,0)));
}

void TutorialApplication::chaseCamera()
{
    gw->attachCamera(car, mCamera);
    mCamera->setPosition(0, mRideHeight - mCoMHeight + CAR_HEIGHT + 2.5f, -10.0f);
    Ogre::Quaternion q(Ogre::Radian(PI), Ogre::Vector3(0,1,0));
    q = q * Ogre::Quaternion(Ogre::Degree(5), Ogre::Vector3(-1,0,0));
    mCamera->setOrientation(q);
}

void TutorialApplication::reverseCamera()
{
    gw->attachCamera(car, mCamera);
    mCamera->setPosition(0, mRideHeight - mCoMHeight + CAR_HEIGHT + 2.5f, 10.0f);
    mCamera->setOrientation(Ogre::Quaternion(Ogre::Degree(5), Ogre::Vector3(-1,0,0)));
}

void TutorialApplication::freeCamera()
{
    const Quaternionr & quat = car->getWorldToBaseRot();

    Vector3r cam_pos = car->getBasePos();
    cam_pos += quat.inverse() * Vector3r(0, 5, -15);
    
    Ogre::Quaternion ogre_quat = 
        Ogre::Quaternion(quat.w(), -quat.x(), -quat.y(), -quat.z())
        * Ogre::Quaternion(Ogre::Radian(PI), Ogre::Vector3(0,1,0));

    gw->detachCamera(car, mCamera);
    mCamera->setPosition(cam_pos.x(), cam_pos.y(), cam_pos.z());
    mCamera->setOrientation(ogre_quat);
}

bool TutorialApplication::frameRenderingQueued(const Ogre::FrameEvent &evt)
{
    static bool first_time = true;
    if (first_time) {

        // Hide the FPS panel by default
        mTrayMgr->toggleAdvancedFrameStats();

        // Setup timings panel
        Ogre::StringVector ParamNames;
        ParamNames.push_back("Time");
        ParamNames.push_back("Collision");
        ParamNames.push_back("Constr. Setup");
        ParamNames.push_back("Physics");
        ParamNames.push_back("Update");
        ParamNames.push_back("Constraints");
        ParamNames.push_back("Wheel speed");
        ParamNames.push_back("Car speed");
#ifdef DISPLAY_STEERING_INFO
        ParamNames.push_back("Steering angle");
        ParamNames.push_back("Front slip");
        ParamNames.push_back("Rear slip");
        ParamNames.push_back("Rear - front");
#endif
        mTimePanel = mTrayMgr->createParamsPanel(OgreBites::TL_BOTTOMLEFT, "Physics_Times", 190, ParamNames);

        // Setup menus
        mPhysicsSettingsButton = mTrayMgr->createButton(OgreBites::TL_TOPLEFT, "Settings", "Settings...");
        mResetSimulationButton = mTrayMgr->createButton(OgreBites::TL_TOPLEFT, "Reset_Simulation", "Reset Simulation");
        //mHelpButton = mTrayMgr->createButton(OgreBites::TL_TOPLEFT, "Help", "Help...");
        mQuitButton = mTrayMgr->createButton(OgreBites::TL_TOPLEFT, "Quit", "Quit");

        mPhysicsTab = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "Physics_Lab", "Physics Settings");
        mGravitySlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Gravity", "Gravity", 200, 50, 0, 30, 121);
        mFrictionSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Friction", "Friction", 200, 50, 0, 2, 101);
        mRestitutionSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Restitution", "Restitution", 200, 50, 0, 1, 101);
        mPhysSep1 = mTrayMgr->createSeparator(OgreBites::TL_TOPRIGHT, "Phys_Params_Separator");

        mCarTab = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "Car_Lab", "Car Settings");
        Ogre::StringVector drive_types;
        drive_types.push_back("4 Wheel Drive");
        drive_types.push_back("Front Wheel Drive");
        drive_types.push_back("Rear Wheel Drive");
        mDriveTypeMenu = mTrayMgr->createLongSelectMenu(OgreBites::TL_TOPRIGHT, "Drive_Type", "Drive Type", 280, 180, 10, drive_types);
        mEngineTorqueSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Engine_Torque", "Engine Torque", 200, 50, 0, MAX_ENGINE_TORQUE, ENGINE_TORQUE_SNAPS);
        mBrakeTorqueSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Brake_Torque", "Brake Torque", 200, 50, 0, MAX_BRAKE_TORQUE_SLIDER, BRAKE_TORQUE_SNAPS);
        mBrakeBalanceSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Brake_Balance", "Brake Balance", 200, 50, 0, 1, 21);
        mTyreFrictionSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Tyre_Friction", "Tyre Friction", 200, 50, 0, 2, 21);
        mCoMHeightSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "CoM_Height", "Centre of Mass Height", 200, 50, MIN_CAR_COM, MAX_CAR_COM, CAR_COM_SNAPS);
        mRideHeightSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Ride_Height", "Ride Height", 200, 50, WHEEL_RADIUS*2 + WHEEL_GAP, 1, 41);
        mSuspensionTravelSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Suspension_Travel", "Suspension Travel", 200, 50, 0, 0.2f, 41);
        mSuspensionStiffnessSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Suspension_Stiffness", "Suspension Stiffness", 200, 50, MIN_SUSPENSION_STIFFNESS, MAX_SUSPENSION_STIFFNESS, SUSPENSION_STIFFNESS_SNAPS);
        mSuspensionDampingSlider = mTrayMgr->createLongSlider(OgreBites::TL_TOPRIGHT, "Suspension_Damping", "Suspension Damping", 200, 50, 0, MAX_SUSPENSION_DAMPING, SUSPENSION_DAMPING_SNAPS);
        
        mPhysSep2 = mTrayMgr->createSeparator(OgreBites::TL_TOPRIGHT, "Phys_Params_Separator_2");
        mGraphicsTab = mTrayMgr->createLabel(OgreBites::TL_TOPRIGHT, "Graphics_Tab", "Graphics Settings");
        mWireframeCheckBox = mTrayMgr->createCheckBox(OgreBites::TL_TOPRIGHT, "Wireframe", "Wireframe Rendering");
#ifdef CONTACT_FORCE_DEBUGGING
        mShowForcesCheckBox = mTrayMgr->createCheckBox(OgreBites::TL_TOPRIGHT, "Show_Contacts", "Show Contact Points");
#endif

        mPhysSep3 = mTrayMgr->createSeparator(OgreBites::TL_TOPRIGHT, "Phys_Params_Separator_3");

        mRestorePhysicsButton = mTrayMgr->createButton(OgreBites::TL_TOPRIGHT, "Restore_Physics", "Restore Defaults");
        mClosePhysicsButton = mTrayMgr->createButton(OgreBites::TL_TOPRIGHT, "Close_Physics", "Close Window");


        mHelpText = 
            mTrayMgr->createTextBox(OgreBites::TL_CENTER, "HelpBox", "Physics Demo v1", 450, 200);
        mHelpText->setText(
            "Copyright (C) Stephen Thompson, 2011\n"
            "\n"
            "Mouse Controls:\n"
            "Left mouse button = Fire spheres\n"
            "Right mouse button = Rotate camera\n"
            "\n"
            "Keyboard Controls:\n"
            "IJKL = Drive car\n"
            "F1 - F4 = Select camera view\n"
            "Arrow keys = Move camera\n"
            "P = Pause simulation\n"
            "Space = Single step (while paused)"
            "\n"
        );
        mCloseHelpButton = mTrayMgr->createButton(OgreBites::TL_CENTER, "Close_Help", "Close Window");

        hidePhysicsSettings();
        hideHelp();

        writeDefaultsToSliders();
        readValuesFromSliders();
        
        chaseCamera();  // readValuesFromSliders() tries to adjust the camera, this puts it back again.
        
        first_time = false;
    }


    static float c_time = 0;
    static float p_time = 0;
    static float s_time = 0;
    static float t_time = 0;
    static int n_constraints = 0;
    static int update_counter = 0;

    if (!pause || single_step) {

        // need fairly small timestep
        // because we would miss collisions otherwise.
        // implementing continuous collision detection would fix this.
        const real_t TIMESTEP = 1 / real_t(100.0);
    
        // One timestep if single stepping, else real time
        time += (!pause ? evt.timeSinceLastFrame : TIMESTEP);
        
        // allow upto 2 updates per frame, this is in case they have set VSYNC,
        // our 100 Hz timestep would need two timesteps per frame in that case.
        const int MAX_UPDATES_PER_FRAME = 2;

        for (int update = 0; update < MAX_UPDATES_PER_FRAME; ++update) {
            if (time >= TIMESTEP) {
                double ct, pt, st, tt;
                int ncon;

#ifdef USE_DOUBLE_PRECISION
                // make sure full FPU precision is being used when in double-precision mode
                _controlfp(_CW_DEFAULT, 0xfffff);
#endif

                gw->applyGravity(Vector3r(0, -mGravity, 0));

                real_t car_damp_const = 0;   // set below
                
                if (car) {

                    real_t car_mass = car->getBaseMass();
                    for (int i = 0; i < car->getNumLinks(); ++i) car_mass += car->getLinkMass(i);

                    if (accelerator) {
                        
                        // NOTE: mEngineTorque is the total torque across all four wheels.

                        if (mDriveType == DRIVE_FWD) {
                            car->addJointTorque(LEFT_FRONT_WHEEL, mEngineTorque/2);
                            car->addJointTorque(RIGHT_FRONT_WHEEL, mEngineTorque/2);
                        } else if (mDriveType == DRIVE_RWD) {
                            car->addJointTorque(LEFT_REAR_WHEEL, mEngineTorque/2);
                            car->addJointTorque(RIGHT_REAR_WHEEL, mEngineTorque/2);
                        } else {
                            car->addJointTorque(LEFT_FRONT_WHEEL, mEngineTorque*real_t(0.2));
                            car->addJointTorque(RIGHT_FRONT_WHEEL, mEngineTorque*real_t(0.2));
                            car->addJointTorque(LEFT_REAR_WHEEL, mEngineTorque*real_t(0.3));
                            car->addJointTorque(RIGHT_REAR_WHEEL, mEngineTorque*real_t(0.3));
                        }
                    }
                    if (brake) {

                        real_t wheel_speed = 0;
                        for (int i = 0; i < NUM_WHEELS; ++i) {
                            wheel_speed += car->getJointVel(WHEELS[i]);
                        }
                        wheel_speed *= WHEEL_RADIUS;
                        if (wheel_speed < real_t(2)) {
                            // go into reverse (rear wheel drive, half normal torque)
                            car->addJointTorque(LEFT_REAR_WHEEL, -mEngineTorque/4);
                            car->addJointTorque(RIGHT_REAR_WHEEL, -mEngineTorque/4);
                        } else {
                            // apply brakes
                            // NOTE: mBrakeTorque acts as a MAX brake torque (across all four wheels)
                            // mBrakeBalance is interpreted as: 1=front, 0=rear, 0.5=equal.
                            const real_t BRAKE_COEFF = real_t(0.5);
                            for (int i = 0; i < NUM_WHEELS; ++i) {
                                real_t torque = -BRAKE_COEFF * car->getJointVel(WHEELS[i]);
                                real_t limit = mBrakeTorque;
                                if (i==LEFT_FRONT_WHEEL||i==RIGHT_FRONT_WHEEL) limit *= mBrakeBalance;
                                else limit *= (1-mBrakeBalance);
                                limit /= 2; // split between left and right equally
                                if (torque > limit) torque = limit;
                                if (torque < -limit) torque = -limit;
                                car->addJointTorque(WHEELS[i], torque);
                            }
                        }
                    }

                    const real_t STEER_RATE = TIMESTEP*real_t(0.3);
                    const real_t STEER_RATE_RETURN = TIMESTEP*real_t(0.6);

                    // Approximate calculation of max possible steering angle 
                    // based on available tyre grip.
                    // NB this is calculated for tyre friction = 1, gravity = 10.
                    const real_t car_speed = car->getBaseVel().norm();
                    const real_t min_turn_radius = car_speed * car_speed / 10;
                    const real_t max_safe_steer = 2 * std::asin((WHEELBASE / 2) / min_turn_radius);

                    // Fudge factor.
                    const real_t MAX_STEER_RATIO = 4;

                    // Never allow steering to go more than this, under any circumstances.
                    const real_t STEERING_LOCK = 15 * (PI/180);

                    real_t max_steer = STEERING_LOCK;
                    if (_finite(max_safe_steer)) {
                        max_steer = std::min(max_steer, max_safe_steer * MAX_STEER_RATIO);
                    }

                    if (steer_left) {
                        steering += STEER_RATE;
                        if (steering < 0) steering += (STEER_RATE_RETURN - STEER_RATE);
                        if (steering > max_steer) steering = max_steer;
                    } else if (steer_right) {
                        steering -= STEER_RATE;
                        if (steering > 0) steering -= (STEER_RATE_RETURN - STEER_RATE);
                        if (steering < -max_steer) steering = -max_steer;
                    } else if (steering > STEER_RATE_RETURN) {
                        steering -= STEER_RATE_RETURN;
                    } else if (steering < -STEER_RATE_RETURN) {
                        steering += STEER_RATE_RETURN;
                    } else {
                        steering = 0;
                    }

                    gw->setupJointLimit(car, STEERING_JOINT, steering, steering);

                    // Suspension spring forces
                    const real_t spring_const = mSuspensionStiffness;
                    const real_t sprung_mass = car->getBaseMass();
                    const real_t q_rest = sprung_mass * mGravity / (NUM_WHEELS * spring_const) + mCoMHeight - WHEEL_RADIUS;
                    
                    car_damp_const = sqrt(4*(sprung_mass/NUM_WHEELS)*spring_const);  // critical damping (or near enough)
                    car_damp_const *= mSuspensionDamping;  // ... multiplied by user's setting (default=1)
                    for (int i = 0; i < NUM_WHEELS; ++i) {
                        const real_t curr_pos = car->getJointPos(WHEELS[i]-1);
                        const real_t curr_vel = car->getJointVel(WHEELS[i]-1);
                        real_t force = -spring_const * (curr_pos - q_rest);
                        force -= car_damp_const * curr_vel;
                        car->addJointTorque(WHEELS[i]-1, force);
                    }
                }

                gw->doTimestep(TIMESTEP, scratch_r, scratch_v, scratch_m, ct, pt, st, tt, ncon);

#ifdef USE_CAR
                // reset wheel angles back to (-2pi,2pi) range. 
                // (Not strictly necessary but may help with floating point accuracy.)
                for (int i = 0; i < NUM_WHEELS; ++i) {
                    car->setJointPos(WHEELS[i], std::fmod(car->getJointPos(WHEELS[i]), 2*PI));
                }
#endif

                c_time += ct;
                p_time += pt;
                s_time += st;
                t_time += tt;
                n_constraints += ncon;

                time -= TIMESTEP;
                if (time > 5*TIMESTEP) time = 5*TIMESTEP;

                ++update_counter;
            }

            if (pause && single_step) single_step = false;
        }
    }
    
    if (c_time + p_time + s_time + t_time > 0.5f) {

        c_time /= update_counter;
        p_time /= update_counter;
        s_time /= update_counter;
        t_time /= update_counter;
        float avg_ncon = float(n_constraints) / float(update_counter);

        mTimePanel->setParamValue("Collision", FormatNumber(c_time * 1000, 2) + " ms");
        mTimePanel->setParamValue("Constr. Setup", FormatNumber(s_time * 1000, 2) + " ms");
        mTimePanel->setParamValue("Physics", FormatNumber(p_time * 1000, 2) + " ms");
        mTimePanel->setParamValue("Update", FormatNumber(t_time * 1000, 2) + " ms");
        mTimePanel->setParamValue("Constraints", FormatNumber(avg_ncon, 0));

        c_time = 0;
        p_time = 0;
        s_time = 0;
        t_time = 0;
        n_constraints = 0;
        update_counter = 0;
    }

    mTimePanel->setParamValue("Time", FormatNumber(gw->getGlobalTime(), 1) + " s");

    if (car) {
        real_t wheel_speed = 0;
        for (int i = 0; i < NUM_WHEELS; ++i) {
            wheel_speed += car->getJointVel(WHEELS[i]);
        }
        wheel_speed /= NUM_WHEELS;
        wheel_speed *= WHEEL_RADIUS;

        mTimePanel->setParamValue("Wheel speed", FormatNumber(wheel_speed * METRES_PER_SEC_TO_MPH, 1) + " mph");
        mTimePanel->setParamValue("Car speed", FormatNumber(car->getBaseVel().norm() * METRES_PER_SEC_TO_MPH, 1) + " mph");

#ifdef DISPLAY_STEERING_INFO
        // display steering direction
        mTimePanel->setParamValue("Steering angle", FormatNumber(car->getJointPos(0) * 180 / PI, 1));

        // display slip angles
        // TODO: Using car velocity is not quite correct. We should get the velocity of the centre of the wheel.        
        Vector3r car_vel = car->getBaseVel();
        if (car_vel.norm() > real_t(0.01)) {
            car_vel.normalize();

            const Vector3r front_facing = car->localDirToWorld(0, Vector3r(0, 0, 1));
            const Vector3r rear_facing = car->localDirToWorld(-1, Vector3r(0, 0, 1));

            const real_t front_slip = std::acos(front_facing.dot(car_vel)) * 180 / PI;
            const real_t rear_slip = std::acos(rear_facing.dot(car_vel)) * 180 / PI;

            mTimePanel->setParamValue("Front slip", FormatNumber(front_slip, 1));
            mTimePanel->setParamValue("Rear slip", FormatNumber(rear_slip, 1));
            mTimePanel->setParamValue("Rear - front", FormatNumber(rear_slip - front_slip, 1));
        } else {
            mTimePanel->setParamValue("Front slip", "#N/A");
            mTimePanel->setParamValue("Rear slip", "#N/A");
            mTimePanel->setParamValue("Rear - front", "#N/A");
        }
#endif
    }

    // call the base class frameRenderingQueued event
    bool ret = BaseApplication::frameRenderingQueued(evt);
    return ret;
}

bool TutorialApplication::keyPressed(const OIS::KeyEvent &arg)
{
    if (mTrayMgr->isDialogVisible()) return true;

    if (arg.key == OIS::KC_P) {
        pause = !pause;
    } else if (arg.key == OIS::KC_SPACE) {
        single_step = true;
    } else if (arg.key == OIS::KC_R) {
        // reset
        createScene();
    } else if (arg.key == OIS::KC_F1) {
        chaseCamera();
    } else if (arg.key == OIS::KC_F2) {
        cockpitCamera();
    } else if (arg.key == OIS::KC_F3) {
        freeCamera();
    } else if (arg.key == OIS::KC_F4) {
        reverseCamera();
    } else if (arg.key == OIS::KC_I || arg.key == OIS::KC_W) {
        accelerator = true;
        return true;
    } else if (arg.key == OIS::KC_K || arg.key == OIS::KC_S) {
        brake = true;
        return true;
    } else if (arg.key == OIS::KC_J || arg.key == OIS::KC_A) {
        steer_left = true;
        return true;
    } else if (arg.key == OIS::KC_L || arg.key == OIS::KC_D) {
        steer_right = true;
        return true;
    }

    return BaseApplication::keyPressed(arg);
}

bool TutorialApplication::keyReleased(const OIS::KeyEvent &arg)
{
    if (mTrayMgr->isDialogVisible()) return true;

    if (arg.key == OIS::KC_I || arg.key == OIS::KC_W) {
        accelerator = false;
        return true;
    } else if (arg.key == OIS::KC_K || arg.key == OIS::KC_S) {
        brake = false;
        return true;
    } else if (arg.key == OIS::KC_J || arg.key == OIS::KC_A) {
        steer_left = false;
        return true;
    } else if (arg.key == OIS::KC_L || arg.key == OIS::KC_D) {
        steer_right = false;
        return true;
    }

    return BaseApplication::keyReleased(arg);
}

bool TutorialApplication::mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
    if (mTrayMgr->injectMouseDown(arg, id)) return true;

    if (id == OIS::MB_Left) {
        Ogre::Ray mouseRay = mCamera->getCameraToViewportRay(float(arg.state.X.abs) / float(arg.state.width), 
                                                             float(arg.state.Y.abs) / float(arg.state.height));
        Ogre::Vector3 direction = mouseRay.getDirection();
        Ogre::Vector3 startPoint = mouseRay.getOrigin() + direction*3;
        const real_t speed = 70;
        
        gw->beginObjectMultibody(Vector3r(startPoint[0], startPoint[1], startPoint[2]),
                                 Quaternionr::Identity(), false, true, false);
        gw->setInitialVelocity(Vector3r(direction[0] * speed, direction[1] * speed, direction[2] * speed));
        gw->setShapeSphere(BALL_DENSITY, 0.5, 0.8f, 0.8f, 1.0f);
        gw->endObject();
    }

    return BaseApplication::mousePressed(arg, id);
}

void TutorialApplication::sliderMoved(OgreBites::Slider *slider)
{
    readValuesFromSliders();
}

void TutorialApplication::buttonHit(OgreBites::Button *b)
{
    if (b == mRestorePhysicsButton) {
        writeDefaultsToSliders();
        readValuesFromSliders();

        mWireframeCheckBox->setChecked(false);
#ifdef CONTACT_FORCE_DEBUGGING
        mShowForcesCheckBox->setChecked(false);
#endif

    } else if (b == mQuitButton) {
        mShutDown = true;
    } else if (b == mPhysicsSettingsButton) {
        showPhysicsSettings();
    } else if (b == mResetSimulationButton) {
        createScene();  // this will reset everything
    } else if (b == mClosePhysicsButton) {
        hidePhysicsSettings();
    } else if (b == mHelpButton) {
        showHelp();
    } else if (b == mCloseHelpButton) {
        hideHelp();
    }
}

void TutorialApplication::itemSelected(OgreBites::SelectMenu *menu)
{
}

void TutorialApplication::checkBoxToggled(OgreBites::CheckBox *cb)
{
#ifdef CONTACT_FORCE_DEBUGGING
    if (cb == mShowForcesCheckBox) {
        gw->setDrawContactForces(cb->isChecked());
    }
#endif
    if (cb == mWireframeCheckBox) {
        if (cb->isChecked()) {
            mCamera->setPolygonMode(Ogre::PM_WIREFRAME);
        } else {
            mCamera->setPolygonMode(Ogre::PM_SOLID);
        }
    }
}

void TutorialApplication::hidePhysicsSettings()
{
    mTrayMgr->removeWidgetFromTray(mPhysicsTab); mPhysicsTab->hide();
    mTrayMgr->removeWidgetFromTray(mGravitySlider); mGravitySlider->hide();
    mTrayMgr->removeWidgetFromTray(mFrictionSlider); mFrictionSlider->hide();
    mTrayMgr->removeWidgetFromTray(mRestitutionSlider); mRestitutionSlider->hide();

    mTrayMgr->removeWidgetFromTray(mCarTab); mCarTab->hide();
    mTrayMgr->removeWidgetFromTray(mPhysSep1); mPhysSep1->hide();
    mTrayMgr->removeWidgetFromTray(mDriveTypeMenu); mDriveTypeMenu->hide();
    mTrayMgr->removeWidgetFromTray(mEngineTorqueSlider); mEngineTorqueSlider->hide();
    mTrayMgr->removeWidgetFromTray(mBrakeTorqueSlider); mBrakeTorqueSlider->hide();
    mTrayMgr->removeWidgetFromTray(mBrakeBalanceSlider); mBrakeBalanceSlider->hide();
    mTrayMgr->removeWidgetFromTray(mTyreFrictionSlider); mTyreFrictionSlider->hide();
    mTrayMgr->removeWidgetFromTray(mCoMHeightSlider); mCoMHeightSlider->hide();
    mTrayMgr->removeWidgetFromTray(mRideHeightSlider); mRideHeightSlider->hide();
    mTrayMgr->removeWidgetFromTray(mSuspensionTravelSlider); mSuspensionTravelSlider->hide();
    mTrayMgr->removeWidgetFromTray(mSuspensionStiffnessSlider); mSuspensionStiffnessSlider->hide();
    mTrayMgr->removeWidgetFromTray(mSuspensionDampingSlider); mSuspensionDampingSlider->hide();

    mTrayMgr->removeWidgetFromTray(mGraphicsTab); mGraphicsTab->hide();
    mTrayMgr->removeWidgetFromTray(mWireframeCheckBox); mWireframeCheckBox->hide();
#ifdef CONTACT_FORCE_DEBUGGING
    mTrayMgr->removeWidgetFromTray(mShowForcesCheckBox); mShowForcesCheckBox->hide();
#endif

    mTrayMgr->removeWidgetFromTray(mPhysSep2); mPhysSep2->hide();
    mTrayMgr->removeWidgetFromTray(mPhysSep3); mPhysSep3->hide();
    mTrayMgr->removeWidgetFromTray(mRestorePhysicsButton); mRestorePhysicsButton->hide();
    mTrayMgr->removeWidgetFromTray(mClosePhysicsButton); mClosePhysicsButton->hide();
}

void TutorialApplication::showPhysicsSettings()
{
    mTrayMgr->moveWidgetToTray(mPhysicsTab, OgreBites::TL_TOPRIGHT); mPhysicsTab->show();
    mTrayMgr->moveWidgetToTray(mGravitySlider, OgreBites::TL_TOPRIGHT); mGravitySlider->show();
    mTrayMgr->moveWidgetToTray(mFrictionSlider, OgreBites::TL_TOPRIGHT); mFrictionSlider->show();
    mTrayMgr->moveWidgetToTray(mRestitutionSlider, OgreBites::TL_TOPRIGHT); mRestitutionSlider->show();
    
    mTrayMgr->moveWidgetToTray(mPhysSep1, OgreBites::TL_TOPRIGHT); mPhysSep1->show();
    mTrayMgr->moveWidgetToTray(mCarTab, OgreBites::TL_TOPRIGHT); mCarTab->show();
    //mTrayMgr->moveWidgetToTray(mDriveTypeMenu, OgreBites::TL_TOPRIGHT); mDriveTypeMenu->show();
    mTrayMgr->moveWidgetToTray(mEngineTorqueSlider, OgreBites::TL_TOPRIGHT); mEngineTorqueSlider->show();
    mTrayMgr->moveWidgetToTray(mBrakeTorqueSlider, OgreBites::TL_TOPRIGHT); mBrakeTorqueSlider->show();
    mTrayMgr->moveWidgetToTray(mBrakeBalanceSlider, OgreBites::TL_TOPRIGHT); mBrakeBalanceSlider->show();
    mTrayMgr->moveWidgetToTray(mTyreFrictionSlider, OgreBites::TL_TOPRIGHT); mTyreFrictionSlider->show();
    mTrayMgr->moveWidgetToTray(mCoMHeightSlider, OgreBites::TL_TOPRIGHT); mCoMHeightSlider->show();
    mTrayMgr->moveWidgetToTray(mRideHeightSlider, OgreBites::TL_TOPRIGHT); mRideHeightSlider->show();
    mTrayMgr->moveWidgetToTray(mSuspensionTravelSlider, OgreBites::TL_TOPRIGHT); mSuspensionTravelSlider->show();
    mTrayMgr->moveWidgetToTray(mSuspensionStiffnessSlider, OgreBites::TL_TOPRIGHT); mSuspensionStiffnessSlider->show();
    mTrayMgr->moveWidgetToTray(mSuspensionDampingSlider, OgreBites::TL_TOPRIGHT); mSuspensionDampingSlider->show();

    mTrayMgr->moveWidgetToTray(mPhysSep2, OgreBites::TL_TOPRIGHT); mPhysSep2->show();
    mTrayMgr->moveWidgetToTray(mGraphicsTab, OgreBites::TL_TOPRIGHT); mGraphicsTab->show();
    mTrayMgr->moveWidgetToTray(mWireframeCheckBox, OgreBites::TL_TOPRIGHT); mWireframeCheckBox->show();
#ifdef CONTACT_FORCE_DEBUGGING
    mTrayMgr->moveWidgetToTray(mShowForcesCheckBox, OgreBites::TL_TOPRIGHT); mShowForcesCheckBox->show();
#endif
    
    mTrayMgr->moveWidgetToTray(mPhysSep3, OgreBites::TL_TOPRIGHT); mPhysSep3->show();
    mTrayMgr->moveWidgetToTray(mRestorePhysicsButton, OgreBites::TL_TOPRIGHT); mRestorePhysicsButton->show();
    mTrayMgr->moveWidgetToTray(mClosePhysicsButton, OgreBites::TL_TOPRIGHT); mClosePhysicsButton->show();

}

void TutorialApplication::hideHelp()
{
    mTrayMgr->removeWidgetFromTray(mHelpText); mHelpText->hide();
    mTrayMgr->removeWidgetFromTray(mCloseHelpButton); mCloseHelpButton->hide();
}

void TutorialApplication::showHelp()
{
    mTrayMgr->moveWidgetToTray(mHelpText, OgreBites::TL_CENTER); mHelpText->show();
    mTrayMgr->moveWidgetToTray(mCloseHelpButton, OgreBites::TL_CENTER); mCloseHelpButton->show();
}

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        TutorialApplication app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBoxA( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
