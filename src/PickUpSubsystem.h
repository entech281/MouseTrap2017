#ifndef _PickUp_SUBSYSTEM_H
#define _PickUp_SUBSYSTEM_H

#include <WPILib.h>
#include <Solenoid.h>

#include "RobotSubsystem.h"
#include "OperatorButton.h"

class PickUpSubsystem : public RobotSubsystem {
public:
	PickUpSubsystem(EntechRobot *pRobot, std::string name = "PickUp");
    virtual ~PickUpSubsystem();
    enum PickUpPosition {kUp, kDown};

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

    void SetPosition(PickUpPosition position);

private:
    Solenoid* m_pickupSolenoid1;
    Solenoid* m_pickupSolenoid2;
 //   Timer *m_timer;
    PickUpPosition m_position;
};

#endif
