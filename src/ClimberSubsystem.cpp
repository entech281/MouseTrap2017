#include <WPILib.h>

#include "ClimberSubsystem.h"
#include "RobotConstants.h"

const double c_grabSpeed = 0.5;
const double c_climbSpeed = 1.0;
const double c_currentThreshold = 10.0;
const double c_speedThreshold = 100.0;

ClimberSubsystem::ClimberSubsystem(EntechRobot *pRobot, std::string name)
  : RobotSubsystem(pRobot, name)
  , m_climberMotor(NULL)
  , m_speed(0.0)
{
}

ClimberSubsystem::~ClimberSubsystem()
{

}
// Sets the rope climb turn speed to off/0 when the "button" is not pressed.
void ClimberSubsystem::Off()
{
	m_speed = 0.;
}

void ClimberSubsystem::Climb()
{
	m_speed = c_grabSpeed;
}

void ClimberSubsystem::Backward()
{
    m_speed = -c_climbSpeed;
}

void ClimberSubsystem::RobotInit()
{
	m_climberMotor = new CANTalon(c_climberMotor_CANid);
	m_climberMotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_climberMotor->SetInverted(true);
}

void ClimberSubsystem::UpdateDashboard()
{
    SmartDashboard::PutNumber("Climber Speed", m_speed);
    SmartDashboard::PutNumber("Climb Actual Speed", m_climberMotor->GetSpeed());
    SmartDashboard::PutNumber("Climber Current:", m_climberMotor->GetOutputCurrent());
}

void ClimberSubsystem::TeleopInit()
{
	m_speed = 0.0;
}

void ClimberSubsystem::AutonomousInit()
{
	m_speed = 0.0;
}

void ClimberSubsystem::TestInit()
{
}

void ClimberSubsystem::DisabledInit()
{
	m_speed = 0.0;
}

void ClimberSubsystem::DisabledPeriodic()
{
	m_climberMotor->Set(0.0);
}
//declares that the robot should turn the rope climber when the button is pressed.
void ClimberSubsystem::TeleopPeriodic()
{
    if ((m_speed > c_grabSpeed) &&
        (fabs(m_climberMotor->GetSpeed()) < c_speedThreshold)) {
            m_speed = 0.0;
    }
    if (m_climberMotor->GetOutputCurrent() > c_currentThreshold) {
    	m_speed = c_climbSpeed;
    }
    m_climberMotor->Set(m_speed);
}

void ClimberSubsystem::AutonomousPeriodic()
{
}

void ClimberSubsystem::TestPeriodic()
{
}
