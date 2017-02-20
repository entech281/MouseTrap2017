#include <WPILib.h>

#include "RobotSubsystem.h"
#include "EntechRobot.h"

RobotSubsystem::RobotSubsystem(EntechRobot *pRobot, std::string name)
    : m_name(name)
{
    pRobot->RegisterSubsystem(this);
}

RobotSubsystem::~RobotSubsystem() {}

const char* RobotSubsystem::GetName()
{
    return m_name.c_str();
}

void RobotSubsystem::UpdateDashboard(void)
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::UpdateDashboard() being used for " + m_name);
    warning_issued = true;
  }
}
 void RobotSubsystem::LogHeader(FILE *fp)
 {
 }

 void RobotSubsystem::LogData(FILE *fp)
 {
 }

/********************************** Init Routines **********************************/

void RobotSubsystem::RobotInit()
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::RobotInit() " + m_name);
    warning_issued = true;
  }
}

void RobotSubsystem::DisabledInit()
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::DisabledInit() " + m_name);
    warning_issued = true;
  }
}

void RobotSubsystem::TeleopInit()
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::TeleopInit() " + m_name);
    warning_issued = true;
  }
}

void RobotSubsystem::AutonomousInit()
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::AutonomousInit() " + m_name);
    warning_issued = true;
  }
}

void RobotSubsystem::TestInit()
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::TestInit() " + m_name);
    warning_issued = true;
  }
}

/********************************** Periodic Routines **********************************/

void RobotSubsystem::DisabledPeriodic()
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::DisabledPeriodic() " + m_name);
    warning_issued = true;
  }
}

void RobotSubsystem::TeleopPeriodic()
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::TeleopPeriodic() " + m_name);
    warning_issued = true;
  }
}

void RobotSubsystem::AutonomousPeriodic()
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::AutonomousPeriodic() " + m_name);
    warning_issued = true;
  }
}

void RobotSubsystem::TestPeriodic()
{
  static bool warning_issued = false;
  if (!warning_issued) {
    frc::DriverStation::ReportWarning("RobotSubsystem::TestPeriodic() " + m_name);
    warning_issued = true;
  }
}
