#include <WPILib.h>

#include "ClimberSubsystem.h"
#include "RobotConstants.h"

ClimberSubsystem::ClimberSubsystem(EntechRobot *pRobot, std::string name)
  : RobotSubsystem(pRobot, name)
  , m_climberMotor(NULL)
  , m_speed(0.0)
{

}

ClimberSubsystem::~ClimberSubsystem()
{

}

void ClimberSubsystem::Off()
{
		m_speed = 0.;
}

void ClimberSubsystem::Forward()
{
		m_speed = -1.;
}

void ClimberSubsystem::RobotInit()
{
	m_climberMotor = new CANTalon(c_climberMotor_CANid);
	m_climberMotor->SetControlMode(CANSpeedController::kPercentVbus);

}

void ClimberSubsystem::UpdateDashboard()
{

}

void ClimberSubsystem::TeleopInit()
{

}

void ClimberSubsystem::AutonomousInit()
{

}

void ClimberSubsystem::TestInit()
{

}

void ClimberSubsystem::DisabledInit()
{

}

void ClimberSubsystem::DisabledPeriodic()
{
	m_climberMotor->Set(0.0);
}

void ClimberSubsystem::TeleopPeriodic()
{
	m_climberMotor->Set(m_speed);
}

void ClimberSubsystem::AutonomousPeriodic()
{

}

void ClimberSubsystem::TestPeriodic()
{

}
