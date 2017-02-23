#include <WPILib.h>

#include "ShooterSubsystem.h"
#include "RobotConstants.h"

ShooterSubsystem::ShooterSubsystem(EntechRobot *pRobot, std::string name)
  : RobotSubsystem(pRobot, name)
  , m_ShooterMotor(NULL)
  , m_solenoid1(NULL)
  , m_solenoid2(NULL)
  , m_shoot(false)
  , m_speed(0.0)
{
}

ShooterSubsystem::~ShooterSubsystem()
{
}

void ShooterSubsystem::Forward(double speed)
{
    m_speed = speed;
}

void ShooterSubsystem::TriggerOpen(void)
{
    m_shoot = true;
}

void ShooterSubsystem::TriggerClose(void)
{
    m_shoot = false;
}

void ShooterSubsystem::RobotInit()
{
    m_ShooterMotor = new CANTalon(c_ShooterMotor_CANid);
    m_ShooterMotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_solenoid1 = new Solenoid(c_compressorPCMid, c_shooterSolenoidChannel1);
    m_solenoid2 = new Solenoid(c_compressorPCMid, c_shooterSolenoidChannel2);
}

void ShooterSubsystem::UpdateDashboard()
{
    SmartDashboard::PutNumber("Shooter Speed", m_speed);
}

void ShooterSubsystem::TeleopInit()
{
	m_shoot = false;
}

void ShooterSubsystem::AutonomousInit()
{
	m_shoot = false;
}

void ShooterSubsystem::TestInit()
{
}

void ShooterSubsystem::DisabledInit()
{
	m_shoot = false;
}

void ShooterSubsystem::DisabledPeriodic()
{
    m_ShooterMotor->Set(0.0);
}

void ShooterSubsystem::TeleopPeriodic()
{
	AutonomousPeriodic();
}

void ShooterSubsystem::AutonomousPeriodic()
{
	m_ShooterMotor->Set(m_speed);
    if (m_shoot) {
        m_solenoid1->Set(false);
        m_solenoid2->Set(true);
    } else {
        m_solenoid1->Set(true);
        m_solenoid2->Set(false);
    }
}

void ShooterSubsystem::TestPeriodic()
{
}
