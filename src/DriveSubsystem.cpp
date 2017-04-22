#include <WPILib.h>
#include <math.h>

#include "DriveSubsystem.h"
#include "RobotConstants.h"
#include "EntechRobot.h"

#ifndef M_PI
#define M_PI  3.1415926
#endif

#define POSITION_TABLE "position"
#define RIO_ALIVE_KEY "rio_alive"
#define RPI_ALIVE_KEY "rpi_alive"
#define RPI_SEQUENCE  "rpi_sequence"
#define FOUND_KEY "found"
#define DIRECTION_KEY "direction"
#define DISTANCE_KEY "distance"
#define TEAM_281 281
#define UPDATE_RATE_MS 30

const int c_countUntilIgnoreRPi = 60;
const int c_countUntilIgnoreRpiPermanently = 1000;
const double c_minVisionDistance = 12.;
const double c_slowVisionDistance = 24.0;
const double c_slowVisionSpeed = -0.15;
const double c_yawTolerance = 3.0;
const double c_lateralTolerence = 5.0;
const double c_stoppedVelocityTolerance = 0.001;

const double c_nudgeSpeed = 0.5;
const double c_nudgeTime = 0.1;

const static double kYaw_P = 0.03;
const static double kYaw_I = 0.0001;
const static double kYaw_D = 0.01;
const static double kYaw_ToleranceDegrees = 2.0;

const static double kLateral_P = -0.1;
const static double kLateral_I =  0.0;
const static double kLateral_D = -0.2;
const static double kLateral_Tolerance = 2.0;

DriveSubsystem::DriveSubsystem(EntechRobot *pRobot, std::string name)
    : RobotSubsystem(pRobot, name)
    , m_pRobot(pRobot)
    , m_currMode(kManual)
    , m_joystick(NULL)
    , m_frmotor(NULL)
    , m_flmotor(NULL)
    , m_rrmotor(NULL)
    , m_rlmotor(NULL)
    , m_robotDrive(NULL)

#if NAVX
    , m_ahrs(NULL)
    , m_navxOk(true)
#endif
#if IMU_MXP
    , m_imu(NULL)
#endif
    , m_missingRPiCount(0)
    , m_rpi_lastseq(-1)
    , m_rpi_seq(0)
    , m_rpi_seq_lastTargetsFound(-1)
    , m_visionTargetsFound(false)
    , m_targetsBelowMinDistance(false)
    , m_visionLateral(0.0)
    , m_lateralDecay(0.0)
    , m_visionDistance(100.0)
    , m_straffeSpeed(0.0)
    , m_allowStraffe(false)
    , m_pinSeenInAutoAlign(false)
    , m_inAutonomous(false)
    , m_yawJStwist(0.0)
    , m_lateralJS(0.0)
    , m_forwardJS(0.0)
    , m_lastPOV(-1)
    , m_yawPIDInterface(NULL)
    , m_lateralPIDInterface(NULL)
    , m_yawController(NULL)
    , m_lateralController(NULL)

    , m_timer(NULL)
    , m_vTimer(NULL)
    , m_vTimerRunning(false)
    , m_time(0.0)
    , m_speed(0.0)
    , m_dir(0.0)
    , m_yawAngle(0.0)
    , m_currentYawAngle(0.0)

    , m_fieldAbsolute(true)
    , m_useFieldAbsForDeadRec(true)

    , m_fieldAbsoluteToggleButton(NULL)
    , m_holdYawToggleButton(NULL)
    , m_yawToP60Button(NULL)
    , m_yawToZeroButton(NULL)
    , m_yawToM60Button(NULL)
    , m_resetYawToZeroButton(NULL)
    , m_autoDriveButton(NULL)
    , m_autoYawButton(NULL)
    , m_nudgeLeftButton(NULL)
    , m_nudgeRightButton(NULL)
{
}

DriveSubsystem::~DriveSubsystem() {}

/********************************** Init Routines **********************************/

void DriveSubsystem::RobotInit()
{
    // Try creating the NavX first, to give it time to calibrate
#if NAVX_USB
    try {
        m_ahrs = new AHRS(SerialPort::Port::kUSB);
        DriverStation::ReportWarning("NavX USB found");
        m_ahrs->Reset();
        m_navxOk = true;
    } catch (std::exception& ex) {
        DriverStation::ReportError("NavX USB MISSING");
        m_navxOk = false;
        m_ahrs = NULL;
    }
#endif
#if NAVX_MXP
    try {
        m_ahrs = new AHRS(SPI::kMXP);
        DriverStation::ReportWarning("NavX MXP found");
        m_ahrs->Reset();
        m_navxOk = true;
    } catch (std::exception& ex) {
        DriverStation::ReportError("NavX MXP MISSING");
        m_navxOk = false;
        m_ahrs = NULL;
    }
#endif
#if IMU_MXP
    try {
        m_imu = new ADIS16448_IMU();
        DriverStation::ReportWarning("IMU MXP found");
    	m_imu->Calibrate();
    	m_imu->Reset();
    } catch (std::exception & ex) {
        DriverStation::ReportWarning("IMU MXP MISSING");
        m_imu = NULL;
    }
#endif

    m_flmotor = new CANTalon(c_flmotor_CANid);
    m_frmotor = new CANTalon(c_frmotor_CANid);
    m_rlmotor = new CANTalon(c_rlmotor_CANid);
    m_rrmotor = new CANTalon(c_rrmotor_CANid);

    m_robotDrive = new frc::RobotDrive(m_flmotor, m_rlmotor, m_frmotor, m_rrmotor);
    m_robotDrive->SetSafetyEnabled(false);

//    m_flmotor->SetControlMode(CANSpeedController::kPercentVbus);
//    m_frmotor->SetControlMode(CANSpeedController::kPercentVbus);
//    m_rlmotor->SetControlMode(CANSpeedController::kPercentVbus);
//    m_rrmotor->SetControlMode(CANSpeedController::kPercentVbus);

    m_robotDrive->SetInvertedMotor(frc::RobotDrive::kFrontLeftMotor , c_kflmotor_inverted);
    m_robotDrive->SetInvertedMotor(frc::RobotDrive::kRearLeftMotor  , c_krlmotor_inverted);
    m_robotDrive->SetInvertedMotor(frc::RobotDrive::kFrontRightMotor, c_kfrmotor_inverted);
    m_robotDrive->SetInvertedMotor(frc::RobotDrive::kRearRightMotor , c_krrmotor_inverted);

    // PID Controllers
#if NAVX
    m_yawPIDInterface = new PidInterface(&m_currentYawAngle, &m_yawJStwist);
#endif
#if IMU_MXP
    m_yawPIDInterface = new PidInterface(m_imu, &m_yawJStwist);
#endif
    m_lateralPIDInterface = new PidInterface(&m_visionLateral, &m_lateralJS);

#if NAVX || IMU_MXP
    m_yawController = new frc::PIDController(kYaw_P, kYaw_I, kYaw_D, m_yawPIDInterface, m_yawPIDInterface);
    m_yawController->SetAbsoluteTolerance(kYaw_ToleranceDegrees);
    m_yawController->SetInputRange(-180.0, 180.0);
    m_yawController->SetContinuous(true);
    m_yawController->SetOutputRange(-1.0, 1.0);
    m_yawController->Disable();
#endif

    m_lateralController = new frc::PIDController(kLateral_P, kLateral_I, kLateral_D, m_lateralPIDInterface, m_lateralPIDInterface);
    m_lateralController->SetAbsoluteTolerance(kLateral_Tolerance);
    m_lateralController->SetInputRange(-100.0, 100.0);
    m_lateralController->SetContinuous(false);
    m_lateralController->SetOutputRange(-1.0, 1.0);
    m_lateralController->Disable();

    m_timer = new frc::Timer();
    m_vTimer = new frc::Timer();

    // Driver interface on Driver joystick buttons
    m_joystick = new Joystick(c_driverJSid);
    m_fieldAbsoluteToggleButton = new OperatorButton(m_joystick, c_jsfieldAbs_BTNid);
    m_holdYawToggleButton = new OperatorButton(m_joystick, c_jsHoldYaw_BTNid);
    m_yawToP60Button  = new OperatorButton(m_joystick, c_jsYawToP60_BTNid);
    m_yawToZeroButton = new OperatorButton(m_joystick, c_jsYawToZero_BTNid);
    m_yawToM60Button  = new OperatorButton(m_joystick, c_jsYawToM60_BTNid);
    m_resetYawToZeroButton  = new OperatorButton(m_joystick, c_jsYawReset_BTNid);
    m_autoDriveButton = new OperatorButton(m_joystick, c_jsautodrive_BTNid);
    m_autoYawButton = new OperatorButton(m_joystick, c_jsAutoYaw_BTNid);
    m_nudgeLeftButton = new OperatorButton(m_joystick, c_jsNudgeLeft_BTNid);
    m_nudgeRightButton = new OperatorButton(m_joystick, c_jsNudgeRight_BTNid);

    // OK make sure the NavX has finished calibrating
#if NAVX
    if (m_ahrs) {
        while (m_ahrs->IsCalibrating()) {
            Wait(0.05);
        }
        m_ahrs->ZeroYaw();
    }
    m_currentYawAngle = NormalizeYaw(m_ahrs->GetYaw() - m_pRobot->InitialYaw());
#endif
}

void DriveSubsystem::DisabledInit()
{
    m_inAutonomous = false;
    m_currMode = kManual;
    m_targetsBelowMinDistance = false;
}

void DriveSubsystem::TeleopInit()
{
    m_inAutonomous = false;
    m_currMode = kManual;
    m_targetsBelowMinDistance = false;
    HoldYaw(false);
    m_lateralController->Disable();
}

void DriveSubsystem::AutonomousInit()
{
    m_inAutonomous = true;
    m_currMode = kManual;
    m_targetsBelowMinDistance = false;
#if NAVX
    m_currentYawAngle = 0.0;
    if (m_ahrs && m_navxOk) {
        m_ahrs->ZeroYaw();
        m_currentYawAngle = NormalizeYaw(m_ahrs->GetYaw() - m_pRobot->InitialYaw());
    }
#endif
    SmartDashboard::PutBoolean("Pi seen in Autonomous",false);
}

void DriveSubsystem::TestInit()
{
    m_inAutonomous = false;
    m_currMode = kManual;
    m_targetsBelowMinDistance = false;
}

//======================= Public Methods ==========================================
void DriveSubsystem::DriveHeading(double angle, double speed, double time)
{
    m_dir = angle*M_PI/180.0;
    m_speed = speed;
    m_time = time;
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
    m_currMode = kDeadRecon;
    m_allowStraffe = false;
    m_useFieldAbsForDeadRec = true;
}

void DriveSubsystem::DriveRobotHeading(double angle, double speed, double time)
{
    DriveHeading(angle, speed, time);
    m_useFieldAbsForDeadRec = false;
}
void DriveSubsystem::NudgeLeft(int count)
{
    m_lateralController->Disable();
    DriveRobotHeading(-90.0, c_nudgeSpeed, count*c_nudgeTime);
}

void DriveSubsystem::NudgeRight(int count)
{
    m_lateralController->Disable();
    DriveRobotHeading(+90.0, c_nudgeSpeed, count*c_nudgeTime);
}

void DriveSubsystem::NudgeForward(int count)
{
    m_lateralController->Disable();
    DriveRobotHeading(0.0, 0.75*c_nudgeSpeed, 2.0*count*c_nudgeTime);
}

void DriveSubsystem::NudgeBackward(int count)
{
    m_lateralController->Disable();
    DriveRobotHeading(180.0, 0.75*c_nudgeSpeed, 2.0*count*c_nudgeTime);
}

void DriveSubsystem::BackoffPin(void)
{
    double yaw = GetRobotYaw();
    if (yaw < -30.0) {
        yaw = -60.0;
    } else if (yaw > 30.0) {
        yaw = 60.0;
    } else {
        yaw = 0.0;
    }
    m_dir = yaw*M_PI/180.0;
    m_speed = -0.25;
    m_time = 0.5;
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
    m_currMode = kDeadRecon;
    m_allowStraffe = false;
    SetYawDirection(GetRobotYaw());
    HoldYaw(true);
}

void DriveSubsystem::DriveToVisionTarget(double speed, bool auto_yaw)
{
#if NAVX || IMU_MXP
    if (auto_yaw) {
        double yaw = GetRobotYaw();
        if (yaw < -30.0) {
            SetYawDirection(-60.0);
        } else if (yaw > 30.0) {
            SetYawDirection(60.0);
        } else {
            SetYawDirection(0.0);
        }
        HoldYaw(true);
    }
#endif

    m_lateralController->SetSetpoint(0.0);
    m_lateralController->Enable();
    m_forwardJS = speed;

    m_allowStraffe = false;
    m_pinSeenInAutoAlign = false;
    m_currMode = kAutomatic;

    // If RPi is not found, we are going to try anyway for a max number of seconds.
    if (m_inAutonomous && (m_missingRPiCount > c_countUntilIgnoreRpiPermanently)) {
        DriveHeading(GetRobotYaw(),-speed,3.0);
    }
}
void DriveSubsystem::AlignWithTargetFacing(double yaw_angle, double lateral_speed)
{
    SetYawDirection(yaw_angle);
    HoldYaw(true);

    m_straffeSpeed = lateral_speed;
    m_allowStraffe = true;
    m_currMode = kAutomatic;
}

bool DriveSubsystem::AreTargetsVisible()
{
    // This code just returns the current frame target information
    // return m_visionTargetsFound;
    // This requires see vision targets in view for 1 second
    if (m_visionTargetsFound) {
        if (!m_vTimerRunning) {
            m_vTimerRunning = true;
            m_vTimer->Stop();
            m_vTimer->Reset();
            m_vTimer->Start();
        }
        if (m_vTimerRunning && (m_vTimer->Get() > 0.75)) {
            m_vTimer->Stop();
            m_vTimerRunning = false;
            return true;
        }
    } else {
        m_vTimer->Stop();
        m_vTimerRunning = false;
    }
    return false;
}

void DriveSubsystem::AbortDriveToVisionTarget(void)
{
    if (m_yawController)
        m_yawController->Disable();
    if (m_lateralController)
        m_lateralController->Disable();
    m_currMode = kManual;
    m_straffeSpeed = 0.0;
    m_targetsBelowMinDistance = false;
}

bool DriveSubsystem::Done(void)
{
    if (m_currMode == kManual)
        return true;
    return false;
}

bool DriveSubsystem::Stopped(void)
{
#if NAVX
    if (m_ahrs && m_navxOk) {
        if (fabs(m_ahrs->GetVelocityX()) < c_stoppedVelocityTolerance) {
            return true;
        }
        return false;
    }
#endif
    return true;
}

void DriveSubsystem::FieldAbsoluteDriving(bool active)
{
    m_fieldAbsolute = active;
}

void DriveSubsystem::HoldYaw(bool active)
{
    if (m_yawController) {
        if (active) {
            m_yawController->Enable();
        } else {
            m_yawController->Disable();
        }
    }
}

void DriveSubsystem::SetYawDirection(double angle)
{
    m_yawAngle = angle;
#if NAVX || IMU_MXP
    m_yawController->Disable();
    m_yawController->SetSetpoint(m_yawAngle);
#endif
    if (m_inAutonomous)
        m_currMode = kAutomatic;
}

bool DriveSubsystem::IsYawCorrect(void)
{
#if NAVX || IMU_MXP
    if (fabs(GetRobotYaw() - m_yawAngle) < c_yawTolerance) {
        return true;
    }
    return false;
#else
    return true;
#endif
}

bool DriveSubsystem::IsAlignmentCorrect(void)
{
    if (IsYawCorrect() && m_visionTargetsFound && (fabs(m_visionLateral) < c_lateralTolerence)) {
        return true;
    }
    return false;
}

/********************************** Periodic Routines **********************************/

void DriveSubsystem::GetVisionData()
{
    //static Timer rpi_timer;
	bool last_visionTargetsFound;

    m_ntTable = NetworkTable::GetTable(POSITION_TABLE);
    m_ntTable->PutBoolean(RIO_ALIVE_KEY,true);
    m_rpi_seq = m_ntTable->GetNumber(RPI_SEQUENCE,m_rpi_lastseq);
    if (m_rpi_seq != m_rpi_lastseq) {
        //SmartDashboard::PutNumber("RPi Timer", rpi_timer.Get());
        //SmartDashboard::PutNumber("RPi Seq", m_rpi_seq);
        //rpi_timer.Stop();
        //rpi_timer.Reset();
        //rpi_timer.Start();
        m_missingRPiCount = 0;
        last_visionTargetsFound = m_visionTargetsFound;
        m_visionTargetsFound = m_ntTable->GetBoolean(FOUND_KEY,false);
        m_visionLateral = m_ntTable->GetNumber(DIRECTION_KEY,0.0);
        m_visionDistance = m_ntTable->GetNumber(DISTANCE_KEY,100.0);
        //  Convert to real units based on camera 62.2deg field of view
        //  LateralDist = Distance * sin(62.2/2) * (m_visionLateral/100)
        // m_visionLateral = 0.01 * m_visionLateral * 0.516 * m_visionDistance;
        m_lateralDecay = m_visionLateral/20.0;
        // we check for min distance BEFORE we lose it off the bottom
        if (last_visionTargetsFound && m_visionTargetsFound) {
        	if (m_visionDistance < c_minVisionDistance) {
                m_targetsBelowMinDistance = true;
            } else {
                m_targetsBelowMinDistance = false;
            }
        }
        if (m_targetsBelowMinDistance) {
            m_lateralController->Disable();
            m_lateralJS = 0.0;
        } else {
            if (m_visionTargetsFound) {
                m_yawWhenTargetsLastSeen = GetRobotYaw();
                m_lateralWhenTargetsLastSeen = m_visionLateral;
                m_lateralController->SetSetpoint(0.0);
                m_lateralController->Enable();
            } else {
                if (m_allowStraffe) {
                    m_lateralJS = m_straffeSpeed;
                } else {
                    m_lateralController->Disable();
                    double deltaYaw = m_yawWhenTargetsLastSeen - GetRobotYaw();
                    // Yaw checks get priority over lateral positions for deciding which way the targets went
                    if (deltaYaw < -5.0) {
                        m_lateralJS = -0.15;
                    } else if (deltaYaw > 5.0) {
                        m_lateralJS = 0.15;
                    } else if (m_lateralWhenTargetsLastSeen < 0.0) {
                        m_lateralJS = -0.15;
                    } else if (m_lateralWhenTargetsLastSeen > 0.0) {
                        m_lateralJS = 0.15;
                    } else {
                        m_lateralJS = 0.0;
                    }
                }
            }
        }
    } else {
        ++m_missingRPiCount;
        if (fabs(m_visionLateral) > fabs(m_lateralDecay)) {
            m_visionLateral -= m_lateralDecay;
        } else {
            m_visionLateral = 0.0;
        }
    }

    m_rpi_lastseq = m_rpi_seq;
}

void DriveSubsystem::DisabledPeriodic()
{
    GetVisionData();
    if (m_ahrs && m_navxOk)
        m_currentYawAngle = NormalizeYaw(m_ahrs->GetYaw() - m_pRobot->InitialYaw());
    SmartDashboard::PutString("Drive Routine", "DisabledPeriodic");
    SmartDashboard::PutNumber("jsX", 0.0);
    SmartDashboard::PutNumber("jsY", 0.0);
    SmartDashboard::PutNumber("jsT", 0.0);
    m_robotDrive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
}

double DriveSubsystem::GetRobotYaw(void)
{
#if NAVX
    if (m_ahrs && m_navxOk)
        return NormalizeYaw(m_ahrs->GetYaw() - m_pRobot->InitialYaw());
#endif
#if IMU_MXP
    if (m_imu)
        return NormalizeYaw(m_imu->GetYaw() - m_pRobot->InitialYaw());
#endif
    return 0.0;
}

// Return a yaw angle between -180 and +180
double DriveSubsystem::NormalizeYaw(double yaw)
{
	while (yaw > 180.0)
		yaw -= 360.0;
	while (yaw < -180.0)
		yaw += 360.0;
	return yaw;
}

void DriveSubsystem::TeleopPeriodic()
{
    GetVisionData();

#ifdef NAVX
    if (m_ahrs->IsConnected()) {
        m_currentYawAngle = NormalizeYaw(m_ahrs->GetYaw() - m_pRobot->InitialYaw());
        m_navxOk = true;
    } else {
        m_currentYawAngle = 0.0;
        m_navxOk = false;
        m_yawController->Disable();
    }
#endif

    // This is teleop, so manage driver inputs here
    if (m_fieldAbsoluteToggleButton->Get() == OperatorButton::kJustPressed) {
        FieldAbsoluteDriving(!m_fieldAbsolute);
    }
    if (m_holdYawToggleButton->Get() == OperatorButton::kJustPressed) {
        SetYawDirection(GetRobotYaw());
        if (m_yawController)
            HoldYaw(!m_yawController->IsEnabled());
    }
    if (m_yawToP60Button->Get() == OperatorButton::kJustPressed) {
        SetYawDirection(60.0);
        HoldYaw(true);
    }
    if (m_yawToZeroButton->Get() == OperatorButton::kJustPressed) {
        SetYawDirection(0.0);
        HoldYaw(true);
    }
    if (m_yawToM60Button->Get() == OperatorButton::kJustPressed) {
        SetYawDirection(-60.0);
        HoldYaw(true);
    }
    if (m_resetYawToZeroButton->Get() == OperatorButton::kJustPressed) {
        m_ahrs->ZeroYaw();
        m_pRobot->ZeroInitialYaw();
    }
    int pov = -1;
    if (m_joystick) {
        pov = m_joystick->GetPOV();
    }
    if ((m_nudgeLeftButton->Get() == OperatorButton::kJustPressed) ||
        ((m_lastPOV != 270) && (pov == 270))                         )  {
        NudgeLeft();
    }
    if ((m_nudgeRightButton->Get() == OperatorButton::kJustPressed) ||
        ((m_lastPOV != 90) && (pov == 90))                            )  {
        NudgeRight();
    }
    if ((m_lastPOV != 0) && (pov == 0)) {
        NudgeForward();
    }
    if ((m_lastPOV != 180) && (pov == 180)) {
        NudgeBackward();
    }
    m_lastPOV = pov;
    if (m_autoDriveButton->GetBool() && (m_visionTargetsFound || m_targetsBelowMinDistance)) {
        if (m_pRobot->IsInAutoDropMode() && m_pRobot->IsGearDropped()) {
            m_pinSeenInAutoAlign = true;
            AbortDriveToVisionTarget();
            m_targetsBelowMinDistance = false;
            m_lateralController->Disable();
            m_currMode = kManual;
        }
        if ((m_currMode != kAutomatic) && (!m_pinSeenInAutoAlign)) {
            DriveToVisionTarget();
            m_currMode = kAutomatic;
        }
    } else {
        m_pinSeenInAutoAlign = false;
        if (m_currMode == kAutomatic) {
            AbortDriveToVisionTarget();
            m_targetsBelowMinDistance = false;
            m_lateralController->Disable();
            m_currMode = kManual;
        }
    }

    // If operator is in autodrop mode and the gear has been dropped backup until operator lets go
    switch (m_currMode) {
    case kDeadRecon:
        DoDriveDeadRecon();
        // If timer has expired --  back to manual driving
        if (m_timer->Get() > m_time) {
            m_allowStraffe = false;
            m_currMode = kManual;
        }
        break;
    case kManual:
        DoDriveManual();
        break;
    case kAutomatic:
        DoDriveAutomatic();
        // If trying automatic alignment with missing RPi -- back to manual driving
        if (m_missingRPiCount > c_countUntilIgnoreRPi) {
            m_allowStraffe = false;
            m_currMode = kManual;
        }
        break;
    }
}

void DriveSubsystem::AutonomousPeriodic()
{
    GetVisionData();

    m_currentYawAngle = 0.0;
    if (m_ahrs && m_navxOk)
        m_currentYawAngle = NormalizeYaw(m_ahrs->GetYaw() - m_pRobot->InitialYaw());
    if (m_missingRPiCount == 0) {
        SmartDashboard::PutBoolean("Pi seen in Autonomous",true);
    }

    switch (m_currMode) {
    case kDeadRecon:
        DoDriveDeadRecon();
        // If timer has expired, back to manual driving
        if (m_timer->Get() > m_time) {
            m_allowStraffe = false;
            m_currMode = kManual;
        }
        break;
    case kManual:
        m_allowStraffe = false;
        SmartDashboard::PutString("Drive Routine", "AutoPeriodic_kManual");
        SmartDashboard::PutNumber("jsX", 0.0);
        SmartDashboard::PutNumber("jsY", 0.0);
        SmartDashboard::PutNumber("jsT", 0.0);
        m_robotDrive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
        break;
    case kAutomatic:
        DoDriveAutomatic();
        break;
    }
}

void DriveSubsystem::DoDriveAutomatic()
{
    double jsX, jsY, jsT;

    if (m_pRobot->IsGearDropped() || (m_missingRPiCount > c_countUntilIgnoreRPi)) {
        m_currMode = kManual;
        SmartDashboard::PutString("Drive Routine", "DoDriveAutomatic1");
        SmartDashboard::PutNumber("jsX", 0.0);
        SmartDashboard::PutNumber("jsY", 0.0);
        SmartDashboard::PutNumber("jsT", 0.0);
        m_robotDrive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
    } else {
        // Either use joystick for speed from driver or what autonomous wants
        if (m_forwardJS > 1.0) {
            jsY = m_joystick->GetY();
        } else {
            jsY = m_forwardJS;
            if ((m_visionDistance < c_slowVisionDistance) && (m_forwardJS < c_slowVisionSpeed)) {
                jsY = c_slowVisionSpeed;
            }
        }
        if (m_pRobot->IsPinSensed() && (jsY < 0.0)) {
            jsY = 0.0;
        }
        jsX = m_lateralJS;
        if (m_targetsBelowMinDistance) {
            jsX = 0.0;
        }
        jsT = 0.0;
        if (m_yawController && m_yawController->IsEnabled()) {
            jsT = m_yawJStwist;
        }
        SmartDashboard::PutString("Drive Routine", "DoDriveAutomatic2");
        SmartDashboard::PutNumber("jsX", jsX);
        SmartDashboard::PutNumber("jsY", jsY);
        SmartDashboard::PutNumber("jsT", jsT);
        m_robotDrive->MecanumDrive_Cartesian(jsX, jsY, jsT, 0.0);
    }
}

void DriveSubsystem::DoDriveDeadRecon()
{
    double jsX, jsY, jsT, gyroAngle;

    jsX = m_speed*sin(m_dir);
    jsY = -m_speed*cos(m_dir);

    /* Rotate the robot if the trigger being held or yaw is being maintained */
    jsT = 0.0;
    if (m_yawController && m_yawController->IsEnabled()) {
        jsT = m_yawJStwist;
    }
    gyroAngle = 0.0;
#if NAVX
    if (m_useFieldAbsForDeadRec && m_ahrs && m_navxOk) {
        gyroAngle = GetRobotYaw();
    }
#endif
#if IMU_MXP
    if (m_useFieldAbsForDeadRec && m_imu) {
        gyroAngle = m_imu->GetAngle();
    }
#endif

    /* Move the robot */
    SmartDashboard::PutString("Drive Routine", "DoDriveDeadRecon");
    SmartDashboard::PutNumber("jsX", jsX);
    SmartDashboard::PutNumber("jsY", jsY);
    SmartDashboard::PutNumber("jsT", jsT);
    m_robotDrive->MecanumDrive_Cartesian(jsX, jsY, jsT, gyroAngle);
}

void DriveSubsystem::DoDriveManual()
{
    double jsX, jsY, jsT, jsAngle, gyroAngle, deltaAngle;

    jsX = 0.0;
    jsY = 0.0;
    jsAngle = 360.0;
    if (m_joystick) {
        jsX = m_joystick->GetX();
        jsY = m_joystick->GetY();
        if ((fabs(jsX) > 0.1) || (fabs(jsY) > 0.1)) {
            jsAngle = 180.0*atan2(jsX,-jsY)/M_PI;
        }
    }
    // If auto drive had dropped out because RPi not found, we still enforce no forward motion
    if (m_autoDriveButton->GetBool() && m_pRobot->IsInAutoDropMode() && m_pRobot->IsPinSensed() && (jsY < 0.0)) {
        jsY = 0.0;
    }

    /* Rotate the robot if the trigger being held or yaw is being maintained */
    jsT = 0.0;
    if (m_yawController && m_yawController->IsEnabled()) {
        jsT = m_yawJStwist;
    }
    if (m_joystick->GetTrigger()) {
        HoldYaw(false);
        jsT = m_joystick->GetTwist();
    }

    gyroAngle = 0.0;
#if NAVX
    if (m_fieldAbsolute && m_ahrs && m_navxOk) {
        gyroAngle = GetRobotYaw();
    }
#endif
#if IMU_MXP
    if (m_fieldAbsolute && m_imu) {
        gyroAngle = m_imu->GetAngle();
    }
#endif
    if (m_autoYawButton && m_autoYawButton->GetBool() && jsAngle < 360.0) {
        deltaAngle = fabs(GetRobotYaw() - jsAngle);
        // If robot is facing wrong direction,
        // don't pivot robot all the way around, drive "backwards"
        if ((deltaAngle > 120.0) && (deltaAngle < 240.0)) {
            jsAngle = NormalizeYaw(jsAngle + 180.0);
            SetYawDirection(jsAngle);
        } else {
            SetYawDirection(jsAngle);
        }
        HoldYaw(true);
    }

    /* Move the robot */
    SmartDashboard::PutString("Drive Routine", "DoDriveManual");
    SmartDashboard::PutNumber("jsX", jsX);
    SmartDashboard::PutNumber("jsY", jsY);
    SmartDashboard::PutNumber("jsT", jsT);
    m_robotDrive->MecanumDrive_Cartesian(jsX, jsY, jsT, gyroAngle);
}

void DriveSubsystem::TestPeriodic()
{
    GetVisionData();
}

void DriveSubsystem::LogHeader(FILE *fp)
{
    fputs("rpi_seq,missingRPiCount,vTargetsFound,TargetsBelowMin,vLateral,vDist,rawFwdJS,rawLatJS,rawYawJS,yaw_angle,gyroYaw,",fp);
}

void DriveSubsystem::LogData(FILE *fp)
{
    double yaw;
    yaw = 0.0;
    if (m_ahrs && m_navxOk)
        yaw = m_ahrs->GetYaw();
    fprintf(fp,"%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",m_rpi_seq,m_missingRPiCount,
            m_visionTargetsFound,m_targetsBelowMinDistance,m_visionLateral,m_visionDistance,
            m_forwardJS,m_lateralJS,m_yawJStwist,m_yawAngle,yaw);
}

void DriveSubsystem::UpdateDashboard(void)
{
#if NAVX || IMU_MXP
#if NAVX
    if (m_ahrs && m_navxOk) {
        SmartDashboard::PutData("NavX", m_ahrs);
        SmartDashboard::PutString("Gyro Type", "NAVX");
        SmartDashboard::PutNumber("GetAngle()", m_ahrs->GetAngle()); //total accumulated yaw angle, 360+
        SmartDashboard::PutNumber("GetYaw()", m_ahrs->GetYaw()); //-180 to 180 degrees
    } else {
        SmartDashboard::PutString("Gyro Type", "NAVX Missing");
    }
#endif
#if IMU_MXP
    if (m_imu) {
        SmartDashboard::PutData("IMU", m_imu);
        SmartDashboard::PutString("Gyro Type", "IMU");
        SmartDashboard::PutNumber("GetAngle()", m_imu->GetAngle()); //total accumulated yaw angle, 360+
        SmartDashboard::PutNumber("GetYaw()", m_imu->GetYaw()); //-180 to 180 degrees
    } else {
        SmartDashboard::PutString("Gyro Type", "IMU Missing");
    }
#endif
#else
    SmartDashboard::PutString("Gyro Type", "COMPILED OUT");
#endif
    SmartDashboard::PutNumber("Drive FieldAbsolute", m_fieldAbsolute);
    SmartDashboard::PutBoolean("Vision Targets Found",m_visionTargetsFound);
    SmartDashboard::PutBoolean("Vision Targets Below Min",m_targetsBelowMinDistance);
    SmartDashboard::PutNumber("Vision Lateral", m_visionLateral);
    SmartDashboard::PutNumber("Vision Distance", m_visionDistance);
    SmartDashboard::PutNumber("Missing RPi Count", m_missingRPiCount);
    SmartDashboard::PutNumber("JoystickLateral", m_lateralJS);
    SmartDashboard::PutNumber("JoystickX", m_joystick->GetX());
    SmartDashboard::PutNumber("JoystickY", m_joystick->GetY());
    SmartDashboard::PutBoolean("Yaw Controller Enabled", m_yawController->IsEnabled());
    SmartDashboard::PutNumber("Robot Yaw2", GetRobotYaw());
    SmartDashboard::PutNumber("currentYawAngle (PID in)", m_currentYawAngle);
    SmartDashboard::PutBoolean("Lateral Controller Enabled", m_lateralController->IsEnabled());
    SmartDashboard::PutNumber("Drive HoldYaw Angle", m_yawAngle);
}
