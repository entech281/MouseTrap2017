#include <WPILib.h>

#include "DropperSubsystem.h"
#include "RobotConstants.h"

DropperSubsystem::DropperSubsystem(EntechRobot *pRobot, std::string name)
    : RobotSubsystem(pRobot, name)
    , m_dropperSolenoid1(NULL)
    , m_dropperSolenoid2(NULL)
    , m_limitSwitch(NULL)
    , m_timer(NULL)
    , m_position(kUp)
    , m_mode(kManual)
    , m_lastLimitState(false)
{

}
DropperSubsystem::~DropperSubsystem(){}

/********************************** Init Routines **********************************/

void DropperSubsystem::RobotInit()
{
    m_dropperSolenoid1 = new Solenoid(c_compressorPCMid, c_dropperSolenoidChannel1);
    m_dropperSolenoid2 = new Solenoid(c_compressorPCMid, c_dropperSolenoidChannel2);
    m_limitSwitch = new DigitalInput(c_dropperSensor);
    m_timer = new Timer();
}

void DropperSubsystem::AutonomousInit()
{

}

void DropperSubsystem::TeleopInit()
{

}

void DropperSubsystem::DisabledInit()
{

}

void DropperSubsystem::TestInit()
{

}

void DropperSubsystem::UpdateDashboard()
{
    SmartDashboard::PutBoolean("Gear Drop Pin Sensed", IsPinSensed());
    if (m_position == kDown) {
        SmartDashboard::PutString("Gear Drop Position", "Down");
    } else {
        SmartDashboard::PutString("Gear Drop Position", "Up");
    }
}


/********************************** Periodic Routines **********************************/

void DropperSubsystem::AutonomousPeriodic()
{
	TeleopPeriodic();
}

void DropperSubsystem::TeleopPeriodic()
{
    if (m_mode == kManual) {
        if (m_position == kDown)
        {
            m_dropperSolenoid1->Set(true);
            m_dropperSolenoid2->Set(true);
        } else {
            m_dropperSolenoid1->Set(false);
            m_dropperSolenoid2->Set(false);
        }
    } else {
        // TODO: read sensor and set solenoids based on sensor
        bool currLimitState = IsPinSensed();
        if (currLimitState) {
            if (m_lastLimitState != currLimitState) {
                m_timer->Stop();
                m_timer->Reset();
                m_timer->Start();
            }
            m_dropperSolenoid1->Set(true);
            m_dropperSolenoid2->Set(true);
        } else {
            m_dropperSolenoid1->Set(false);
            m_dropperSolenoid2->Set(false);
        }
        m_lastLimitState = currLimitState;
    }
}

void DropperSubsystem::DisabledPeriodic()
{

}

void DropperSubsystem::TestPeriodic()
{
}

void DropperSubsystem::SetMode(DropperMode mode)
{
    m_mode = mode;
}

void DropperSubsystem::SetPosition(DropperPosition position)
{
    m_position = position;
}

bool DropperSubsystem::IsGearDropped()
{
    if (IsPinSensed() && (m_timer->Get() > 0.25))
        return true;
    return false;
}

// One liner necessary because RoboRio digital inputs are pulled high
// by default (reading true).  Current hardware switch does not fix this.
// Using a normally closed limit switch wiring can switch this
bool DropperSubsystem::IsPinSensed(void)
{
    return !m_limitSwitch->Get();
}
