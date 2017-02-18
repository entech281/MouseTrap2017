#ifndef _DROPPER_SUBSYSTEM_H
#define _DROPPER_SUBSYSTEM_H

#include <WPILib.h>
#include <Solenoid.h>

#include "RobotSubsystem.h"

class DropperSubsystem : public RobotSubsystem {
public:
    DropperSubsystem(EntechRobot *pRobot, std::string name = "Dropper");
    virtual ~DropperSubsystem();
    enum DropperPosition {kUp, kDown};
    enum DropperMode { kManual, kAutomatic };

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

    void SetPosition(DropperPosition position);
    void SetMode(DropperMode mode);
    bool IsGearDropped();

private:
    bool IsPinSensed(void);
    
    Solenoid* m_dropperSolenoid1;
    Solenoid* m_dropperSolenoid2;
    DigitalInput *m_limitSwitch;
    Timer *m_timer;
    DropperPosition m_position;
    DropperMode m_mode;
    bool m_autoTriggered;
    int m_pinSensedCounter;
};

#endif
