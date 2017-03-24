#include <WPILib.h>
#include <math.h>

#include "ShooterSubsystem.h"
#include "RobotConstants.h"

const double c_rpmTolerance = 50.0;
const double c_rpmDeltaToActivatePID = 300.0;

ShooterSubsystem::ShooterSubsystem(EntechRobot *pRobot, std::string name)
    : RobotSubsystem(pRobot, name)
    , m_ShooterMotor(NULL)
    , m_solenoid1(NULL)
    , m_solenoid2(NULL)
    , m_mode(kVbus)
    , m_shoot(false)
    , m_speed(0.0)
    , m_rpm(0.0)
{
}

ShooterSubsystem::~ShooterSubsystem()
{
}

void ShooterSubsystem::Forward(double speed)
{
    m_speed = speed;
    m_mode = kVbus;
    m_ShooterMotor->SetControlMode(CANTalon::kPercentVbus);
    m_ShooterMotor->Set(m_speed);
}

void ShooterSubsystem::SetRPM(double rpm)
{
    m_rpm = rpm;
    m_mode = kRPM;
    m_ShooterMotor->SetTalonControlMode(CANTalon::kSpeedMode);
    m_ShooterMotor->Set(m_rpm);
}

bool ShooterSubsystem::IsAtTargetRPM(void)
{
    if (fabs(m_ShooterMotor->GetSpeed() - m_rpm) < c_rpmTolerance) {
        return true;
    }
    return false;
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
    m_ShooterMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
    m_ShooterMotor->ConfigNominalOutputVoltage(+0.0,-0.0);
    m_ShooterMotor->ConfigMaxOutputVoltage(12.0);
    m_ShooterMotor->ConfigEncoderCodesPerRev(20);
    // m_ShooterMotor->SetInverted(true);

    m_ShooterMotor->SelectProfileSlot(0);
    m_ShooterMotor->SetF(0.0);
    m_ShooterMotor->SetP(10.0);
    m_ShooterMotor->SetI(0.05);
    m_ShooterMotor->SetD(500.0);
    m_ShooterMotor->SetAllowableClosedLoopErr(0);

    m_ShooterMotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_solenoid1 = new Solenoid(c_compressorPCMid, c_shooterSolenoidChannel1);
    m_solenoid2 = new Solenoid(c_compressorPCMid, c_shooterSolenoidChannel2);
}

void ShooterSubsystem::UpdateDashboard()
{
    SmartDashboard::PutNumber("Shooter JS Speed", m_speed);
    SmartDashboard::PutNumber("Shooter Speed", m_ShooterMotor->GetSpeed());
    SmartDashboard::PutNumber("Shooter CAN Speed", m_ShooterMotor->GetEncVel());
    SmartDashboard::PutNumber("Shooter P", m_ShooterMotor->GetP());
}

void ShooterSubsystem::TeleopInit()
{
    m_ShooterMotor->SetControlMode(CANSpeedController::kPercentVbus);
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
    switch (m_mode) {
    case kVbus:
        SmartDashboard::PutString("Shooter Mode", "PercentVbus");
        m_ShooterMotor->SetControlMode(CANTalon::kPercentVbus);
        m_ShooterMotor->Set(m_speed);
        break;
    case kRPM:
        SmartDashboard::PutString("Shooter Mode", "SpeedMode");
        m_ShooterMotor->SetTalonControlMode(CANTalon::kSpeedMode);
        m_ShooterMotor->Set(m_rpm);
        break;
    }
    if (m_shoot) {
        m_solenoid1->Set(true);
        m_solenoid2->Set(false);
    } else {
        m_solenoid1->Set(false);
        m_solenoid2->Set(true);
    }
}

void ShooterSubsystem::TestPeriodic()
{
}
