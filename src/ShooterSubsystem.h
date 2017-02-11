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


    void Off(void);
    void Forward(void);

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
    double m_speed;
};
#endif
