#include <WPILib.h>
#include <math.h>

#include "ShooterSubsystem.h"
#include "RobotConstants.h"

const double c_rpmTolerance = 200.0;

ShooterSubsystem::ShooterSubsystem(EntechRobot *pRobot, std::string name)
    : RobotSubsystem(pRobot, name)
    , m_ShooterMotor(NULL)
    , m_solenoid1(NULL)
    , m_solenoid2(NULL)
    , m_mode(kVbus)
    , m_shoot(true)
    , m_speed(0.0)
    , m_rpm(0.0)
    , m_pidF(1.55)
    , m_pidP(22.0)
    , m_pidI(0.0)
    , m_pidD(800.0)
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
    // if (fabs(m_ShooterMotor->GetSpeed() - m_rpm) < c_rpmTolerance) {
    if (m_ShooterMotor->GetSpeed() > (m_rpm-c_rpmTolerance)) {
        return true;
    }
    return false;
}

void ShooterSubsystem::TriggerOpen(void)
{
    m_shoot = false;
}

void ShooterSubsystem::TriggerClose(void)
{
    m_shoot = true;
}

void ShooterSubsystem::RobotInit()
{
    m_ShooterMotor = new CANTalon(c_ShooterMotor_CANid);
    m_ShooterMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
    m_ShooterMotor->ConfigNominalOutputVoltage(+0.0,-0.0);
    m_ShooterMotor->ConfigMaxOutputVoltage(11.5);
    m_ShooterMotor->ConfigEncoderCodesPerRev(20);  // actual  encoder docs say 20 pulses per channel
    // functions to invert the motor or sensor
    // m_ShooterMotor->SetInverted(true);
    // m_ShooterMotor->SetSensorDirection(true);

    SetPIDController();

    //m_ShooterMotor->SetVelocityMeasurementPeriod(CANTalon::Period_10Ms);
    //m_ShooterMotor->SetVelocityMeasurementWindow(64);

    m_ShooterMotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_solenoid1 = new Solenoid(c_compressorPCMid, c_shooterSolenoidChannel1);
    m_solenoid2 = new Solenoid(c_compressorPCMid, c_shooterSolenoidChannel2);
}

void ShooterSubsystem::SetPIDController(void)
{
    Preferences *prefs = frc::Preferences::GetInstance();
    m_pidF = prefs->GetDouble("shooterF", 1.55);
    m_pidP = prefs->GetDouble("shooterP", 22.0);
    m_pidI = prefs->GetDouble("shooterI", 0.0);
    m_pidD = prefs->GetDouble("shooterD", 800.0);
    m_ShooterMotor->SelectProfileSlot(0);
    m_ShooterMotor->SetF(m_pidF);
    m_ShooterMotor->SetP(m_pidP);
    m_ShooterMotor->SetI(m_pidI);
    m_ShooterMotor->SetD(m_pidD);
    m_ShooterMotor->SetAllowableClosedLoopErr(0);
}

void ShooterSubsystem::UpdateDashboard()
{
    SmartDashboard::PutNumber("Shooter JS Speed", m_speed);
    SmartDashboard::PutNumber("Shooter Speed", m_ShooterMotor->GetSpeed());
    SmartDashboard::PutNumber("Shooter CAN Speed", m_ShooterMotor->GetEncVel());
}

void ShooterSubsystem::LogHeader(FILE *fp)
{
    fputs("ShooterRPM,Shooting,",fp);
}

void ShooterSubsystem::LogData(FILE *fp)
{
    fprintf(fp,"%lf,%d,",m_ShooterMotor->GetSpeed(),m_shoot);
}

void ShooterSubsystem::TeleopInit()
{
    m_ShooterMotor->SetControlMode(CANSpeedController::kPercentVbus);
    SetPIDController();
    m_shoot = true;
}

void ShooterSubsystem::AutonomousInit()
{
    m_shoot = true;
}

void ShooterSubsystem::TestInit()
{
}

void ShooterSubsystem::DisabledInit()
{
    m_shoot = true;
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
