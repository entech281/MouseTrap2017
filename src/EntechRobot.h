#pragma once

#include <WPILib.h>
#include <list>

#include "RobotSubsystem.h"
#include "RobotConstants.h"
#include "DriveSubsystem.h"
#include "ClimberSubsystem.h"
#include "DropperSubsystem.h"

class EntechRobot : public frc::IterativeRobot {
public:
    EntechRobot();
    virtual ~EntechRobot();

    void RegisterSubsystem(RobotSubsystem*);
    bool IsGearDropTriggered(void);

protected:
    void UpdateDashboard();

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
    DriveSubsystem    *m_drive;
    ClimberSubsystem  *m_climber;
    DropperSubsystem *m_dropper;
    Compressor *m_compressor;
    LiveWindow *m_lw;

    OperatorButton *m_autoDriveButton;
    OperatorButton *m_geardropButton;
    OperatorButton *m_climbButton;
    
    std::list<RobotSubsystem*> m_robotSubsystems;

    frc::DigitalInput *m_autoSelectorLeft;
    frc::DigitalInput *m_autoSelectorMiddle;
    frc::DigitalInput *m_autoSelectorRight;
    int m_autoSelection;
    enum AutoState { kStart = 0,
                     kInitialDrive, kWaitForInitialDrive,
                     kDriveToTarget, kWaitForDriveToTarget,
                     kShootFuelLoad, kWaitForShootFuelLoad,
                     kDriveBackward, kWaitForDriveBackward,
                     kDriveLateral, kWaitForDriveLateral,
                     kDriveForward, kWaitForDriveForward,
                     kDone };
    AutoState m_autoState;
    frc::Timer *m_autoTimer;
};

