#ifndef _DRIVE_SUBSYSTEM_H
#define _DRIVE_SUBSYSTEM_H

#include <WPILib.h>
#include <CANTalon.h>
#include <AHRS.h>

#include "RobotSubsystem.h"
#include "OperatorButton.h"

class DriveSubsystem : public RobotSubsystem {
public:
    DriveSubsystem(EntechRobot *pRobot, std::string name = "drive");
    virtual ~DriveSubsystem();

    void ToggleFieldAbsoluteDriving(void);
    void SetFieldAbsoluteDriving(bool active);

    /********************************** Subsystem Routines **********************************/
    virtual void RobotInit();
    virtual void DisabledInit();
    virtual void TeleopInit();
    virtual void AutonomousInit();
    virtual void TestInit();
    virtual void DisabledPeriodic();
    virtual void AutonomousPeriodic();
    virtual void TeleopPeriodic();
    virtual void TestPeriodic();
    virtual void UpdateDashboard(void);

private:
    Joystick* m_joystick;
    CANTalon* m_frmotor;
    CANTalon* m_flmotor;
    CANTalon* m_rrmotor;
    CANTalon* m_rlmotor;
    frc::RobotDrive* m_robotDrive;
    AHRS *m_ahrs;
    bool m_fieldAbsolute;
    OperatorButton *m_toggleFieldAbsoluteButton;
};
#endif
