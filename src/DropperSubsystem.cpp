#include <WPILib.h>
#include <math.h>

#include "DropperSubsystem.h"
#include "RobotConstants.h"

const int c_pinSensesUntilDrop = 2;

DropperSubsystem::DropperSubsystem(EntechRobot *pRobot, std::string name)
    : RobotSubsystem(pRobot, name)
    , m_dropperSolenoid1(NULL)
    , m_dropperSolenoid2(NULL)
    , m_limitSwitch1(NULL)
    , m_limitSwitch2(NULL)
    , m_timer(NULL)
    , m_position(kUp)
    , m_mode(kManual)
    , m_autoTriggered(false)
    , m_pinSensedCounter(0)
    , m_triggerTime(0.0)
{

}
DropperSubsystem::~DropperSubsystem(){}

/********************************** Init Routines **********************************/

void DropperSubsystem::RobotInit()
{
    m_dropperSolenoid1 = new Solenoid(c_compressorPCMid, c_dropperSolenoidChannel1);
    m_dropperSolenoid2 = new Solenoid(c_compressorPCMid, c_dropperSolenoidChannel2);
    m_limitSwitch1 = new DigitalInput(c_dropperSensor1);
    m_limitSwitch2 = new DigitalInput(c_dropperSensor2);
    m_timer = new Timer();
}

void DropperSubsystem::AutonomousInit()
{
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

void DropperSubsystem::TeleopInit()
{
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
}

void DropperSubsystem::DisabledInit()
{

}

void DropperSubsystem::TestInit()
{

}

void DropperSubsystem::UpdateDashboard()
{
    SmartDashboard::PutBoolean("Gear Drop Pin Sensed", IsPinSensed());
    if (m_position == kDown) {
        SmartDashboard::PutString("Gear Drop Position", "Down");
    } else {
        SmartDashboard::PutString("Gear Drop Position", "Up");
    }
    if (m_mode == kManual) {
        SmartDashboard::PutString("Gear Drop Mode", "Manual");
    } else {
        SmartDashboard::PutString("Gear Drop Mode", "Automatic");
    }
    SmartDashboard::PutBoolean("Gear Drop Auto Triggered",m_autoTriggered);
    SmartDashboard::PutNumber("Gear Drop Timer", m_timer->Get());
}

void DropperSubsystem::LogHeader(FILE *fp)
{
    fputs("pin_sensed,drop_pos,gear_dropped,",fp);
}

void DropperSubsystem::LogData(FILE *fp)
{
    fprintf(fp,"%d,%d,%d,",IsPinSensed(),m_position,IsGearDropped());
}


/********************************** Periodic Routines **********************************/

void DropperSubsystem::AutonomousPeriodic()
{
    TeleopPeriodic();
}

void DropperSubsystem::TeleopPeriodic()
{
    if (m_mode == kManual) {
        m_autoTriggered = false;
        if (m_position == kDown) {
            m_dropperSolenoid1->Set(false);
            m_dropperSolenoid2->Set(true);
        } else {
            m_dropperSolenoid1->Set(true);
            m_dropperSolenoid2->Set(false);
        }
    } else {
        if (IsPinSensed()) {
            ++m_pinSensedCounter;
        } else {
            // make the counter detect consecutive presses
            m_pinSensedCounter = 0;
        }
        if ((!m_autoTriggered) && (m_pinSensedCounter > c_pinSensesUntilDrop)) {
            m_autoTriggered = true;
            m_triggerTime = m_timer->Get();
        }
        if (m_autoTriggered) {
            m_dropperSolenoid1->Set(false);
            m_dropperSolenoid2->Set(true);
            m_position = kDown;
        } else {
            m_dropperSolenoid1->Set(true);
            m_dropperSolenoid2->Set(false);
            m_position = kUp;
        }
    }
}

void DropperSubsystem::DisabledPeriodic()
{

}

void DropperSubsystem::TestPeriodic()
{
}

void DropperSubsystem::SetMode(DropperMode mode)
{
    if (mode != m_mode) {
       m_mode = mode;
       m_pinSensedCounter = 0;
       m_autoTriggered = false;
    }
}

void DropperSubsystem::SetPosition(DropperPosition position)
{
    m_position = position;
}

bool DropperSubsystem::IsGearDropped()
{
    if ((m_mode == kAutomatic) && m_autoTriggered && (fabs(m_triggerTime - m_timer->Get()) > 0.40)) {
        return true;
    }
    if ((m_mode == kManual) && (m_position == kDown)) {
        return true;
    }
    return false;
}

// One liner necessary because RoboRio digital inputs are pulled high
// by default (reading true).  Current hardware switch does not fix this.
// Using a normally closed limit switch wiring can switch this
bool DropperSubsystem::IsPinSensed(void)
{
   return !m_limitSwitch1->Get();
}
