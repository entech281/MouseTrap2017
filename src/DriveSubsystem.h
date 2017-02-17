#ifndef _DRIVE_SUBSYSTEM_H
#define _DRIVE_SUBSYSTEM_H

#include <WPILib.h>
#include <CANTalon.h>
#include <AHRS.h>

#include "RobotSubsystem.h"
#include "PIDInterface.h"
#include "OperatorButton.h"

class DriveSubsystem : public RobotSubsystem {
public:
    DriveSubsystem(EntechRobot *pRobot, std::string name = "drive");
    virtual ~DriveSubsystem();

    void DriveHeading(double angle, double speed, double time);
    void DriveToVisionTarget(double speed = -1.0);
    void AbortDriveToVisionTarget(void);
    bool Done(void);
    void FieldAbsoluteDriving(bool active);
    void HoldYaw(bool active);
    void SetYawDirection(double angle);

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
    enum DriveMode { kManual, kAutomatic, kDeadRecon };
    void SetMode(DriveMode mode);
    void GetVisionData(void);
    void DriveAutomatic(void);
    void DriveManual(void);
    void DriveDeadRecon(void);

    EntechRobot *m_pRobot;
    DriveMode m_currMode;
    Joystick* m_joystick;
    CANTalon* m_frmotor;
    CANTalon* m_flmotor;
    CANTalon* m_rrmotor;
    CANTalon* m_rlmotor;
    frc::RobotDrive* m_robotDrive;

    AHRS *m_ahrs;
    std::shared_ptr<NetworkTable> m_ntTable;
    int    m_missingRPiCount;
    bool   m_visionTargetsFound;
    double m_visionLateral;
    double m_visionDistance;
    // Simulated JS outputs from PID controllers
    double m_yawJStwist;
    double m_lateralJS;
    double m_forwardJS;
    PidInterface  *m_yawPIDInterface;
    PidInterface  *m_lateralPIDInterface;
    PidInterface  *m_distancePIDInterface;
    PIDController *m_yawController;
    PIDController *m_lateralController;
    PIDController *m_distanceController;

    frc::Timer *m_timer;
    double m_time;
    double m_speed;
    double m_dir;
    double m_yawAngle;

    bool m_fieldAbsolute;
    bool m_holdYaw;
    
    OperatorButton *m_fieldAbsoluteToggleButton;
    OperatorButton *m_holdYawToggleButton;
    OperatorButton *m_yawToP60Button;
    OperatorButton *m_yawToZeroButton;
    OperatorButton *m_yawToM60Button;
    OperatorButton *m_autoDriveButton;
};
#endif
