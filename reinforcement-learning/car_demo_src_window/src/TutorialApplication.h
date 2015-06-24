/*
-----------------------------------------------------------------------------
Filename:    TutorialApplication.h
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

#ifndef __TutorialApplication_h_
#define __TutorialApplication_h_

#include "real.hpp"
#include <Eigen/StdVector>

#include "BaseApplication.h"
#include "game_world.hpp"

#include <memory>

class TutorialApplication : public BaseApplication
{
public:
    TutorialApplication(void);
    virtual ~TutorialApplication(void);

protected:
    virtual void createScene(void);
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    virtual bool keyPressed(const OIS::KeyEvent &arg);
    virtual bool keyReleased(const OIS::KeyEvent &arg);
    virtual bool mousePressed(const OIS::MouseEvent &arg, OIS::MouseButtonID id);
    
    void cockpitCamera();
    void chaseCamera();
    void reverseCamera();
    void freeCamera();

    virtual void sliderMoved(OgreBites::Slider *slider);
    virtual void buttonHit(OgreBites::Button *button);
    virtual void checkBoxToggled(OgreBites::CheckBox *checkbox);
    virtual void itemSelected(OgreBites::SelectMenu *menu);

private:
    void readValuesFromSliders();
    void writeDefaultsToSliders();

    void hidePhysicsSettings();
    void showPhysicsSettings();
    void hideHelp();
    void showHelp();

    enum DriveType { DRIVE_4WD, DRIVE_FWD, DRIVE_RWD };

private:
    std::auto_ptr<GameWorld> gw;
    float time;

    bool pause, single_step;

    OgreBites::ParamsPanel * mTimePanel;

    std::vector<real_t> scratch_r;
    std::vector<Vector3r> scratch_v;
    std::vector<Matrix3r> scratch_m;

    Multibody * car;
    bool accelerator, brake, steer_left, steer_right;
    real_t steering;

    OgreBites::SelectMenu *mDriveTypeMenu;
    OgreBites::Button *mPhysicsSettingsButton,
        *mResetSimulationButton,
        *mHelpButton, *mQuitButton;

    OgreBites::Slider 
        *mGravitySlider, *mFrictionSlider, *mRestitutionSlider,
        *mEngineTorqueSlider, *mBrakeTorqueSlider, *mBrakeBalanceSlider,
        *mTyreFrictionSlider,
        *mCoMHeightSlider, *mRideHeightSlider,
        *mSuspensionTravelSlider,
        *mSuspensionStiffnessSlider, *mSuspensionDampingSlider;
    OgreBites::Button *mRestorePhysicsButton, *mClosePhysicsButton, *mCloseHelpButton;
    OgreBites::CheckBox * mShowForcesCheckBox, *mWireframeCheckBox;
    OgreBites::Separator *mPhysSep1, *mPhysSep2, *mPhysSep3;
    OgreBites::TextBox *mHelpText;
    OgreBites::Label *mPhysicsTab, *mCarTab, *mGraphicsTab;

    real_t mGravity;
    real_t mEngineTorque, mBrakeTorque, mBrakeBalance;
    real_t mCoMHeight, mRideHeight, mSuspensionTravel;
    real_t mSuspensionStiffness, mSuspensionDamping;

    DriveType mDriveType;
};

#endif // #ifndef __TutorialApplication_h_
