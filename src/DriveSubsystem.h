#ifndef _DRIVE_SUBSYSTEM_H
#define _DRIVE_SUBSYSTEM_H

#define NAVX_USB 1
#define NAVX_MXP 0
#define  IMU_MXP 0
#if NAVX_USB || NAVX_MXP
#define NAVX 1
#endif

#include <WPILib.h>
#include <CANTalon.h>
#if NAVX
#include <AHRS.h>
#endif
#if IMU_MXP
#include <ADIS16448_IMU.h>
#endif

#include "RobotSubsystem.h"
#include "PIDInterface.h"
#include "OperatorButton.h"

class DriveSubsystem : public RobotSubsystem {
public:
    DriveSubsystem(EntechRobot *pRobot, std::string name = "drive");
    virtual ~DriveSubsystem();

    void DriveHeading(double angle, double speed, double time);
    void DriveToVisionTarget(double speed = 10.0, bool auto_yaw = true);
    void AlignWithTargetFacing(double yaw_angle, double lateral_speed);
    void AbortDriveToVisionTarget(void);
    bool Done(void);
    bool AreTargetsVisible(void);
    void FieldAbsoluteDriving(bool active);
    void HoldYaw(bool active);
    void SetYawDirection(double angle);
    bool IsYawCorrect(void);
    bool IsAlignmentCorrect(void);
    bool Stopped(void);

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
    virtual void LogHeader(FILE *fp);
    virtual void LogData(FILE *fp);

private:
    enum DriveMode { kManual, kAutomatic, kDeadRecon };
    void BackoffPin(void);
    void SetMode(DriveMode mode);
    void GetVisionData(void);
    void DriveAutomatic(void);
    void DriveManual(void);
    void DriveDeadRecon(void);
    double GetRobotYaw(void);
    double NormalizeYaw(double yaw);

    EntechRobot *m_pRobot;
    DriveMode m_currMode;
    Joystick* m_joystick;
    CANTalon* m_frmotor;
    CANTalon* m_flmotor;
    CANTalon* m_rrmotor;
    CANTalon* m_rlmotor;
    frc::RobotDrive* m_robotDrive;

#if NAVX
    AHRS *m_ahrs;
#endif
#if IMU_MXP
    ADIS16448_IMU *m_imu;
#endif

    std::shared_ptr<NetworkTable> m_ntTable;
    int    m_missingRPiCount;
    int    m_rpi_lastseq;
    int    m_rpi_seq;
    int    m_rpi_seq_lastTargetsFound;
    bool   m_visionTargetsFound;
    bool   m_targetsBelowMinDistance;
    double m_visionLateral;
    double m_lateralDecay;
    double m_visionDistance;
    double m_straffeSpeed;
    bool   m_allowStraffe;
    bool   m_inAutonomous;

    double m_yawWhenTargetsLastSeen;
    double m_lateralWhenTargetsLastSeen;

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
    OperatorButton *m_resetYawToZeroButton;
    OperatorButton *m_autoDriveButton;
    OperatorButton *m_autoYawButton;
};
#endif
