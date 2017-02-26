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
    , m_pidActive(false)
{
}

ShooterSubsystem::~ShooterSubsystem()
{
}

void ShooterSubsystem::Forward(double speed)
{
    m_speed = speed;
    m_mode = kVbus;
    m_pidActive = false;
}

void ShooterSubsystem::SetRPM(double rpm)
{
    m_rpm = rpm;
    m_mode = kRPM;
    m_pidActive = false;
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

    m_ShooterMotor->SelectProfileSlot(0);
    // 20 pulse/rev, 4600 rpm, 10 measures/sec, 1023 max output
    // 1023 / (4600rpm * 20pulse/rev / (60sec/min * 10 measures/sec)
    // m_ShooterMotor->SetF(6.67);
    m_ShooterMotor->SetF(0.0);
    m_ShooterMotor->SetP(40.0);
    // (16.47,4) 261

    m_ShooterMotor->SetI(0.04);
    m_ShooterMotor->SetD(200.0);
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
    SmartDashboard::PutNumber("Shooter F", m_ShooterMotor->GetF());  // 462
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
    if (m_mode == kVbus) {
        SmartDashboard::PutString("Shooter Mode", "PercentVbus");
        m_ShooterMotor->SetControlMode(CANTalon::kPercentVbus);
        m_ShooterMotor->Set(m_speed);
    } else if (m_pidActive || (m_ShooterMotor->GetSpeed() > (m_rpm - c_rpmDeltaToActivatePID))) {
        SmartDashboard::PutString("Shooter Mode", "SpeedMode");
        m_pidActive = true;
        m_ShooterMotor->Set(m_rpm);
        m_ShooterMotor->SetTalonControlMode(CANTalon::kSpeedMode);
    } else {
        SmartDashboard::PutString("Shooter Mode", "PercentVbus");
        m_ShooterMotor->SetControlMode(CANTalon::kPercentVbus);
        m_ShooterMotor->Set(1.0);
    }
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
