#include <WPILib.h>

#include "ShooterSubsystem.h"
#include "RobotConstants.h"

ShooterSubsystem::ShooterSubsystem(EntechRobot *pRobot, std::string name)
  : RobotSubsystem(pRobot, name)
  , m_ShooterMotor(NULL)
  , m_speed(0.0)
{

}

ShooterSubsystem::~ShooterSubsystem()
{

}

void ShooterSubsystem::Off()
{
		m_speed = 0.;
}

void ShooterSubsystem::Forward()
{
		m_speed = -1.;
}

void ShooterSubsystem::RobotInit()
{
	m_ShooterMotor = new CANTalon(c_ShooterMotor_CANid);
	m_ShooterMotor->SetControlMode(CANSpeedController::kPercentVbus);

}

void ShooterSubsystem::UpdateDashboard()
{

}

void ShooterSubsystem::TeleopInit()
{

}

void ShooterSubsystem::AutonomousInit()
{

}

void ShooterSubsystem::TestInit()
{

}

void ShooterSubsystem::DisabledInit()
{

}

void ShooterSubsystem::DisabledPeriodic()
{
    m_ShooterMotor->Set(0.0);
}

void ShooterSubsystem::TeleopPeriodic()
{

}

void ShooterSubsystem::AutonomousPeriodic()
{
    m_ShooterMotor->Set(m_speed);
}

void ShooterSubsystem::TestPeriodic()
{


}
