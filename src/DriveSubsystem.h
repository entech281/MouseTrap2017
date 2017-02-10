#ifndef _DRIVE_SUBSYSTEM_H
#define _DRIVE_SUBSYSTEM_H

#include <WPILib.h>
#include <CANTalon.h>
#include <AHRS.h>
#include <NetworkTable.h>

#include "RobotSubsystem.h"
#include "PIDInterface.h"
#include "OperatorButton.h"

class DriveSubsystem : public RobotSubsystem {
public:
    DriveSubsystem(EntechRobot *pRobot, std::string name = "drive");
    virtual ~DriveSubsystem();

    void ToggleFieldAbsoluteDriving(void);
    void SetFieldAbsoluteDriving(bool active);
    
    void DriveHeading(double angle, double speed, double time);
    void YawToHeading(double angle);
    void Done(void);

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
    void Periodic();
    Joystick* m_joystick;
    CANTalon* m_frmotor;
    CANTalon* m_flmotor;
    CANTalon* m_rrmotor;
    CANTalon* m_rlmotor;
    frc::RobotDrive* m_robotDrive;
    AHRS *m_ahrs;
    frc::Timer *m_timer;

    NetworkTable m_ntTable;
    bool   m_visionTargetsFound;
    double m_visionLateral;
    double m_visionDistance;
    
    double m_yawJStwist;
    double m_lateralJS;
    double m_forwardJS;
    PIDInterface  *m_yawPIDInterface;
    PIDInterface  *m_lateralPIDInterface;
    PIDInterface  *m_distancePIDInterface;
    PIDController *m_yawController;
    PIDController *m_lateralController;
    PIDController *m_distanceController;

    bool m_fieldAbsolute;
    OperatorButton *m_toggleFieldAbsoluteButton;
    OperatorButton *m_yawToP60Button;
    OperatorButton *m_yawToZeroButton;
    OperatorButton *m_yawToM60Button;
    OperatorButton *m_autoDriveButton;
};
#endif
