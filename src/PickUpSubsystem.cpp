#include <WPILib.h>

#include "PickUpSubsystem.h"
#include "RobotConstants.h"

PickUpSubsystem::PickUpSubsystem(EntechRobot *pRobot, std::string name)
	: RobotSubsystem(pRobot, name)
	, m_pickupSolenoid1(NULL)
    , m_timer(NULL)
	, m_position(kUp)
{

}
PickUpSubsystem::~PickUpSubsystem(){}

void PickUpSubsystem::RobotInit()
{
	m_pickupSolenoid1 = new Solenoid(c_compressorPCMid, c_pickupSolenoidChannel1);
    m_timer = new Timer();
}

void PickUpSubsystem::AutonomousInit()
{

}

void PickUpSubsystem::TeleopInit()
{

}

void PickUpSubsystem::DisabledInit()
{

}

void PickUpSubsystem::TestInit()
{

}

void PickUpSubsystem::UpdateDashboard()
{

}

void PickUpSubsystem::AutonomousPeriodic()
{
	TeleopPeriodic();
}

void PickUpSubsystem::TeleopPeriodic()
{

}

void PickUpSubsystem::DisabledPeriodic()
{

}

void PickUpSubsystem::TestPeriodic()
{
}

void PickUpSubsystem::SetPosition(PickUpPosition position)
{
    m_position = position;
}
