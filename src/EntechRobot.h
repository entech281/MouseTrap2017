#pragma once

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
    bool IsGearDropTriggered(void);

protected:
    void UpdateDashboard();
    void OpenLog(void);
    void WriteLog(void);
    void CloseLog(void);

    virtual void RobotInit();
    virtual void DisabledInit();
    virtual void DisabledPeriodic();
    virtual void TeleopInit();
    virtual void TeleopPeriodic();
    virtual void AutonomousInit();
    virtual void AutonomousPeriodic();
    virtual void TestInit();
    virtual void TestPeriodic();

private:
    void DetermineAutonomousSetup(void);
    DriveSubsystem    *m_drive;
    ClimberSubsystem  *m_climber;
    ShooterSubsystem *m_shooter;
    DropperSubsystem *m_dropper;
    PickUpSubsystem *m_pickup;
    Compressor *m_compressor;
    LiveWindow *m_lw;

    FILE *m_logFP;

    Joystick  *m_gamepad;
    OperatorButton *m_gp_climbButton;
    OperatorButton *m_gp_descendButton;
    OperatorButton *m_gp_climbgrabButton;
    OperatorButton *m_gp_dropButton;
    OperatorButton *m_gp_pickupButton;
    OperatorButton *m_gp_autodropButton;
    
    Joystick *m_buttonpanel;
    OperatorButton *m_bp_climbButton;
    OperatorButton *m_bp_dropButton;
    OperatorButton *m_bp_autodropButton;
    OperatorButton *m_bp_shooterOnButton;
    OperatorButton *m_bp_fireButton;

    std::list<RobotSubsystem*> m_robotSubsystems;

    bool m_autonomousActive;
    frc::DigitalInput *m_autoSelectionD1;
    frc::DigitalInput *m_autoSelectionD2;
    frc::DigitalInput *m_autoSelectionD3;
    enum AutoState { kStart = 0,
                     kTurnOnShooter, kWaitForShooterToSpinup,
                     kShootFuelLoad, kWaitForShootFuelLoad,
                     kInitialDrive, kWaitForInitialDrive,
                     kInitialTurn, kWaitForInitialTurn,
                     kDriveToTarget, kWaitForDriveToTarget,
                     kDriveBackward, kWaitForDriveBackward,
                     kDriveLateral, kWaitForDriveLateral,
                     kDriveForward, kWaitForDriveForward,
                     kSetSideShotYaw, kWaitForSetSideShotYaw,
                     kClearAirship, kWaitForClearAirship,
                     kAlignToTarget, kWaitForAlignToTarget,
                     kBackupToWall, kWaitForBackupToWall,
                     kDone };
    AutoState m_autoState;
    enum BoilerDistance { kNear, kMiddle, kFar, kSiderail };
    BoilerDistance m_boilerDistance;
    enum InitialTurn { kRight60, kStraight, kLeft60 };
    InitialTurn m_initialTurn;
    bool m_boilerToLeft;
    frc::Timer *m_autoTimer;

    Preferences *m_prefs;
    double m_shooterSpeed;
};
