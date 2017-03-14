#include <WPILib.h>
#include <math.h>

#include "ClimberSubsystem.h"
#include "RobotConstants.h"

const double c_grabSpeed = 0.5;
const double c_climbSpeed = 1.0;
const double c_currentThreshold = 10.0;
const double c_speedThreshold = 100.0;
const double c_minClimbTime = 1.0;

ClimberSubsystem::ClimberSubsystem(EntechRobot *pRobot, std::string name)
  : RobotSubsystem(pRobot, name)
  , m_climberMotor(NULL)
  , m_speed(0.0)
  , m_mode(kOff)
  , m_timer(NULL)
{
}

ClimberSubsystem::~ClimberSubsystem()
{
}

void ClimberSubsystem::RobotInit()
{
    m_climberMotor = new CANTalon(c_climberMotor_CANid);
    m_climberMotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_climberMotor->SetInverted(true);
    m_climberMotor->ConfigNeutralMode(frc::CANSpeedController::NeutralMode::kNeutralMode_Brake);
    // TODO Invert encoder direction too

    m_timer = new Timer();
    m_timer->Stop();
    m_timer->Reset();
}

void ClimberSubsystem::Off()
{
    m_mode = kOff;
}

void ClimberSubsystem::Climb()
{
    // if already climbing, don't slow motor down
    if (m_mode == kOff)
        m_mode = kGrab;
}

void ClimberSubsystem::Backward()
{
    m_mode = kLower;
}

void ClimberSubsystem::UpdateDashboard()
{
    SmartDashboard::PutNumber("Climber Speed", m_speed);
    SmartDashboard::PutNumber("Climb Actual Speed", m_climberMotor->GetSpeed());
    SmartDashboard::PutNumber("Climber Current:", m_climberMotor->GetOutputCurrent());
}

void ClimberSubsystem::TeleopInit()
{
    m_mode = kOff;
}

void ClimberSubsystem::AutonomousInit()
{
    m_mode = kOff;
}

void ClimberSubsystem::TestInit()
{
    m_mode = kOff;
}

void ClimberSubsystem::DisabledInit()
{
    m_mode = kOff;
}

void ClimberSubsystem::DisabledPeriodic()
{
    m_mode = kOff;
    m_climberMotor->Set(0.0);
}

//declares that the robot should turn the rope climber when the button is pressed.
void ClimberSubsystem::TeleopPeriodic()
{
    switch (m_mode) {
    case kOff:
        m_speed = 0.0;
        break;
    case kLower:
        m_speed = -c_climbSpeed;
        break;
    case kGrab:
        m_speed = c_grabSpeed;
        if (m_climberMotor->GetOutputCurrent() > c_currentThreshold) {
            m_mode = kClimb;
            m_speed = c_climbSpeed;
            m_timer->Stop();
            m_timer->Reset();
            m_timer->Start();
        }
        break;
    case kClimb:
        m_speed = c_climbSpeed;
        // In case the operator lets go of the button during climb
        // we make sure the climber runs for a min time period before the encoder
        // check stops the climb.  The allows the robot to get climbing again (and the motor turning)
        // before we declare the climb finished.  Alternatively, we could simply just not check the
        // encoder at all and use the operator lifting his finger off the button to stop the climb
        if ((fabs(m_climberMotor->GetSpeed()) < c_speedThreshold) && (m_timer->Get() > c_minClimbTime)) {
            m_mode = kAtTop;
            m_speed = 0.0;
        }
        break;
    case kAtTop:
        m_speed = 0.0;
        break;
    }
    m_climberMotor->Set(m_speed);
}

void ClimberSubsystem::AutonomousPeriodic()
{
}

void ClimberSubsystem::TestPeriodic()
{
}
