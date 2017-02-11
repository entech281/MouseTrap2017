#ifndef _CLIMBER_SUBSYSTEM_H
#define _CLIMBER_SUBSYSTEM_H

#include <WPILib.h>
#include <CANTalon.h>

#include "RobotSubsystem.h"

class EntechRobot;

class ClimberSubsystem : public RobotSubsystem {
public:
    ClimberSubsystem(EntechRobot *pRobot, std::string name = "Climber");
    virtual ~ClimberSubsystem();


    void Off(void);
    void Forward(void);
    void Backward(void);

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
    CANTalon* m_climberMotor;
    double m_speed;
};
#endif
