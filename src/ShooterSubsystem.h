#ifndef _SHOOTER_SUBSYSTEM_H
#define _SHOOTER_SUBSYSTEM_H

#include <WPILib.h>
#include <CANTalon.h>

#include "RobotSubsystem.h"

class EntechRobot;

class ShooterSubsystem : public RobotSubsystem {
public:
    ShooterSubsystem(EntechRobot *pRobot, std::string name = "Shooter");
    virtual ~ShooterSubsystem();

    void Forward(double speed = 1.0);
    void SetRPM(double rpm);
    bool IsAtTargetRPM(void);
    void TriggerOpen(void);
    void TriggerClose(void);

    virtual void UpdateDashboard(void);

    virtual void RobotInit();
    virtual void DisabledInit();
    virtual void TeleopInit();
    virtual void AutonomousInit();
    virtual void TestInit();
    virtual void DisabledPeriodic();
    virtual void AutonomousPeriodic();
    virtual void TeleopPeriodic();
    virtual void TestPeriodic();

private:
    CANTalon* m_ShooterMotor;
    Solenoid* m_solenoid1;
    Solenoid* m_solenoid2;
    frc::Timer m_timer;
    enum ShooterMode { kRPM, kVbus };
    ShooterMode m_mode;
    bool m_shoot;
    double m_speed;
    double m_rpm;
    bool m_pidActive;
};
#endif
