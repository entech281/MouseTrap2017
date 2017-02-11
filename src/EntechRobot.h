#ifndef _ENTECH_ROBOT_H
#define _ENTECH_ROBOT_H

#include <WPILib.h>
#include <list>

#include "RobotSubsystem.h"
#include "RobotConstants.h"
#include "DriveSubsystem.h"
#include "ClimberSubsystem.h"
#include "DropperSubsystem.h"
#include "PickUpSubsystem.h"
#include "ShooterSubsystem.h"
#include "OperatorButton.h"

class EntechRobot : public frc::IterativeRobot {
public:
    EntechRobot();
    virtual ~EntechRobot();

    void RegisterSubsystem(RobotSubsystem*);

protected:
    DriveSubsystem* m_drive;
    ClimberSubsystem *m_climber;
    DropperSubsystem *m_dropper;
    ShooterSubsystem *m_shooter;
    PickUpSubsystem *m_pickup;
    Compressor *m_compressor;
    LiveWindow* m_lw;

    Joystick  *m_joystick;
    OperatorButton *m_climbButton;
    OperatorButton *m_decendButton;

    std::list<RobotSubsystem*> m_robotSubsystems;

    void UpdateDashboard();
#if USB_CAMERA
	static void VisionThread();
#endif

    virtual void RobotInit();
    virtual void DisabledInit();
    virtual void DisabledPeriodic();
    virtual void TeleopInit();
    virtual void TeleopPeriodic();
    virtual void AutonomousInit();
    virtual void AutonomousPeriodic();
    virtual void TestInit();
    virtual void TestPeriodic();
};

#endif
