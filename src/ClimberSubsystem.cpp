#include <WPILib.h>

#include "ClimberSubsystem.h"
#include "RobotConstants.h"

ClimberSubsystem::ClimberSubsystem(EntechRobot *pRobot, std::string name)
  : RobotSubsystem(pRobot, name)
  , m_climberMotor(NULL)
  , m_speed(0.0)
  , m_timer(NULL)
{

}

ClimberSubsystem::~ClimberSubsystem()
{

}
// Sets the rope climb turn speed to off/0 when the "button" is not pressed.
void ClimberSubsystem::Off()
{
		m_speed = 0.;
		m_timer->Stop();
}
// Sets the rope climb turn speed to on/-1 when the "button" is pressed.
void ClimberSubsystem::Forward()
{
		m_speed = -1.0;
		m_timer->Stop();
		m_timer->Reset();
		m_timer->Start();
}

void ClimberSubsystem::Grab()
{
        m_speed = -0.5;
        m_timer->Stop();
        m_timer->Reset();
        m_timer->Start();
}

void ClimberSubsystem::Backward()
{
        m_speed = 1.0;
        m_timer->Stop();
        m_timer->Reset();
        m_timer->Start();
}

void ClimberSubsystem::RobotInit()
{
	m_climberMotor = new CANTalon(c_climberMotor_CANid);
	m_climberMotor->SetControlMode(CANSpeedController::kPercentVbus);
	m_timer = new Timer();
}

void ClimberSubsystem::UpdateDashboard()
{
    SmartDashboard::PutNumber("Climber Speed", m_speed);
    SmartDashboard::PutNumber("Climb Actual Speed", m_climberMotor->GetSpeed());
    SmartDashboard::PutNumber("Climber Current:", m_climberMotor->GetOutputCurrent());
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
    m_timer->Stop();
}
//declares that the robot should turn the rope climber when the button is pressed.
void ClimberSubsystem::TeleopPeriodic()
{
    if ((m_speed != 0.0) &&
            (m_timer->Get() > 0.5) &&
            (fabs(m_climberMotor->GetSpeed()) < 0.01)) {
                m_speed = 0.0;
            }
    m_climberMotor->Set(m_speed);
}

void ClimberSubsystem::AutonomousPeriodic()
{

}

void ClimberSubsystem::TestPeriodic()
{


}
