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
const double c_minVisionDistance = 30.;
const double c_yawTolerance = 3.0;
const double c_lateralTolerence = 5.0;
const double c_velocityTolerance = 0.001;

#if NAVX || IMU_MXP
const static double kYaw_P = 0.03;
const static double kYaw_I = 0.0;
const static double kYaw_D = 0.0;
const static double kYaw_ToleranceDegrees = 2.0;
#endif

const static double kLateral_P = -0.008;
const static double kLateral_I = 0.0;
const static double kLateral_D = 0.0;
const static double kLateral_TolerancePixels = 2.0;

const static double kDistance_P = 0.03;
const static double kDistance_I = 0.0;
const static double kDistance_D = 0.0;
const static double kDistance_TolerancePixels = 2.0;

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
#endif
#if IMU_MXP
    , m_imu(NULL)
#endif
    , m_missingRPiCount(0)
    , m_rpi_lastseq(-1)
    , m_rpi_seq(0)
    , m_visionTargetsFound(false)
    , m_targetsBelowMinDistance(false)
    , m_visionLateral(0.0)
    , m_lateralDecay(0.0)
    , m_visionDistance(100.0)
    , m_straffeSpeed(0.0)
    , m_allowStraffe(false)
    , m_inAutonomous(false)
    , m_yawJStwist(0.0)
    , m_lateralJS(0.0)
    , m_forwardJS(0.0)
    , m_yawPIDInterface(NULL)
    , m_lateralPIDInterface(NULL)
    , m_distancePIDInterface(NULL)
    , m_yawController(NULL)
    , m_lateralController(NULL)
    , m_distanceController(NULL)

    , m_timer(NULL)
    , m_time(0.0)
    , m_speed(0.0)
    , m_dir(0.0)
    , m_yawAngle(0.0)

    , m_fieldAbsolute(true)
    , m_holdYaw(false)

    , m_fieldAbsoluteToggleButton(NULL)
    , m_holdYawToggleButton(NULL)
    , m_yawToP60Button(NULL)
    , m_yawToZeroButton(NULL)
    , m_yawToM60Button(NULL)
    , m_resetYawToZeroButton(NULL)
    , m_autoDriveButton(NULL)
    , m_autoYawButton(NULL)
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
    } catch (std::exception& ex) {
        DriverStation::ReportError("NavX USB MISSING");
        m_ahrs = NULL;
    }
#endif
#if NAVX_MXP
    try {
        m_ahrs = new AHRS(SPI::kMXP);
        DriverStation::ReportWarning("NavX MXP found");
        m_ahrs->Reset();
    } catch (std::exception& ex) {
        DriverStation::ReportError("NavX MXP MISSING");
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

    m_flmotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_frmotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_rlmotor->SetControlMode(CANSpeedController::kPercentVbus);
    m_rrmotor->SetControlMode(CANSpeedController::kPercentVbus);

    m_robotDrive->SetInvertedMotor(frc::RobotDrive::kFrontLeftMotor , c_kflmotor_inverted);
    m_robotDrive->SetInvertedMotor(frc::RobotDrive::kRearLeftMotor  , c_krlmotor_inverted);
    m_robotDrive->SetInvertedMotor(frc::RobotDrive::kFrontRightMotor, c_kfrmotor_inverted);
    m_robotDrive->SetInvertedMotor(frc::RobotDrive::kRearRightMotor , c_krrmotor_inverted);

    // PID Controllers
#if NAVX
    m_yawPIDInterface = new PidInterface(m_ahrs, &m_yawJStwist);
#endif
#if IMU_MXP
    m_yawPIDInterface = new PidInterface(m_imu, &m_yawJStwist);
#endif
    m_lateralPIDInterface = new PidInterface(&m_visionLateral, &m_lateralJS);
    m_distancePIDInterface = new PidInterface(&m_visionDistance, &m_forwardJS);

#if NAVX || IMU_MXP
    m_yawController = new frc::PIDController(kYaw_P, kYaw_I, kYaw_D, m_yawPIDInterface, m_yawPIDInterface);
    m_yawController->SetAbsoluteTolerance(kYaw_ToleranceDegrees);
    m_yawController->SetInputRange(-180.0, 180.0);
    m_yawController->SetContinuous(true);
    m_yawController->SetOutputRange(-1.0, 1.0);
    m_yawController->Disable();
#endif

    m_lateralController = new frc::PIDController(kLateral_P, kLateral_I, kLateral_D, m_lateralPIDInterface, m_lateralPIDInterface);
    m_lateralController->SetAbsoluteTolerance(kLateral_TolerancePixels);
    m_lateralController->SetInputRange(-100.0, 100.0);
    m_lateralController->SetContinuous(false);
    m_lateralController->SetOutputRange(-1.0, 1.0);
    m_lateralController->Disable();

    m_distanceController = new frc::PIDController(kDistance_P, kDistance_I, kDistance_D, m_distancePIDInterface, m_distancePIDInterface);
    m_distanceController->SetAbsoluteTolerance(kDistance_TolerancePixels);
    m_distanceController->SetInputRange(0.0, 100.0);
    m_distanceController->SetContinuous(false);
    m_distanceController->SetOutputRange(0.25, 0.5);
    m_distanceController->Disable();

    m_timer = new frc::Timer();

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

    // OK make sure the NavX has finished calibrating
#if NAVX
    if (m_ahrs) {
        while (m_ahrs->IsCalibrating()) {
            Wait(0.05);
        }
        m_ahrs->ZeroYaw();
    }
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
}

void DriveSubsystem::AutonomousInit()
{
    m_inAutonomous = true;
    m_currMode = kManual;
    m_targetsBelowMinDistance = false;
#if NAVX
    m_ahrs->ZeroYaw();
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
}

void DriveSubsystem::BackoffPin(void)
{
    m_dir = GetRobotYaw()*M_PI/180.0;
    m_speed = -0.25;
    m_time = 0.20;
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
    m_currMode = kDeadRecon;
    m_allowStraffe = false;
#if NAVX || IMU_MXP
    SetYawDirection(GetRobotYaw());
    HoldYaw(true);
#endif
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
    // m_distanceController->SetSetpoint(0.0);
    // m_distanceController->Enable();

    m_allowStraffe = false;
    m_currMode = kAutomatic;

    // If RPi is not found, we are going to try anyway for a max number of seconds.
    if (m_inAutonomous && (m_missingRPiCount > c_countUntilIgnoreRpiPermanently)) {
        DriveHeading(GetRobotYaw(),-speed,2.0);
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

void DriveSubsystem::AbortDriveToVisionTarget(void)
{
#if NAVX || IMU_MXP
    if (!m_holdYaw)
        m_yawController->Disable();
#endif
    m_lateralController->Disable();
    // m_distanceController->Disable();
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
    if (m_ahrs) {
        if (fabs(m_ahrs->GetVelocityX()) < c_velocityTolerance) {
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
    m_holdYaw = active;
#if NAVX || IMU_MXP
    if (m_yawController) {
        if (m_holdYaw) {
            m_yawController->Enable();
        } else {
            m_yawController->Disable();
        }
    }
#endif
}

void DriveSubsystem::SetYawDirection(double angle)
{
    m_yawAngle = angle;
#if NAVX || IMU_MXP
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
    m_ntTable = NetworkTable::GetTable(POSITION_TABLE);
    m_ntTable->PutBoolean(RIO_ALIVE_KEY,true);
    m_rpi_seq = m_ntTable->GetNumber(RPI_SEQUENCE,m_rpi_lastseq);
    if (m_rpi_seq != m_rpi_lastseq) {
        m_missingRPiCount = 0;
        m_visionTargetsFound = m_ntTable->GetBoolean(FOUND_KEY,false);
        m_visionLateral = m_ntTable->GetNumber(DIRECTION_KEY,0.0);
        m_lateralDecay = m_visionLateral/20.0;
        m_visionDistance = m_ntTable->GetNumber(DISTANCE_KEY,100.0);
        if (m_targetsBelowMinDistance || (m_visionDistance < c_minVisionDistance)) {
            m_targetsBelowMinDistance = true;
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
    m_robotDrive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
}

double DriveSubsystem::GetRobotYaw(void)
{
#if NAVX
    if (m_ahrs)
        return m_ahrs->GetYaw();
#endif
#if IMU_MXP
    if (m_imu)
        return m_imu->GetYaw();
#endif
    return 0.0;
}

void DriveSubsystem::TeleopPeriodic()
{
    GetVisionData();

    // This is teleop, so manage driver inputs here
    if (m_fieldAbsoluteToggleButton->Get() == OperatorButton::kJustPressed) {
        FieldAbsoluteDriving(!m_fieldAbsolute);
    }
#if NAVX || IMU_MXP
    if (m_holdYawToggleButton->Get() == OperatorButton::kJustPressed) {
        SetYawDirection(GetRobotYaw());
        HoldYaw(!m_holdYaw);
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
    }
#endif
    if ((m_visionTargetsFound || m_targetsBelowMinDistance) && m_autoDriveButton->GetBool()) {
        if (m_currMode != kAutomatic) {
            DriveToVisionTarget();
        }
        m_currMode = kAutomatic;
    } else {
        if (m_currMode != kManual) {
            AbortDriveToVisionTarget();
        }
        m_targetsBelowMinDistance = false;
        m_currMode = kManual;
    }
    switch (m_currMode) {
    case kManual:
        DriveManual();
        break;
    case kDeadRecon:
        if (m_timer->Get() < m_time) {
            DriveDeadRecon();
        } else {
            m_allowStraffe = false;
            m_currMode = kManual;
            DriveManual();
        }
        break;
    case kAutomatic:
        DriveAutomatic();
        break;
    }
}

void DriveSubsystem::AutonomousPeriodic()
{
    GetVisionData();

    // This is autonomous -- no driver inputs
    switch (m_currMode) {
    case kDeadRecon:
        if (m_timer->Get() < m_time) {
            DriveDeadRecon();
        } else {
            m_allowStraffe = false;
            m_currMode = kManual;
        }
        break;
    case kManual:
        m_allowStraffe = false;
        m_robotDrive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
        break;
    case kAutomatic:
        DriveAutomatic();
        break;
    }
    if (m_missingRPiCount == 0) {
        SmartDashboard::PutBoolean("Pi seen in Autonomous",true);
    }
}

void DriveSubsystem::DriveAutomatic()
{
    double jsX, jsY, jsT;

    // In teleop trying automatic alignment with missing RPi -- back to manual
    if (!m_inAutonomous && (m_missingRPiCount > c_countUntilIgnoreRPi)) {
        m_currMode = kManual;
        DriveManual();
        return;
    }

    if (m_pRobot->IsGearDropped() || (m_missingRPiCount > c_countUntilIgnoreRPi)) {
        m_currMode = kManual;
        m_robotDrive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
    } else {
        // Either use joystick for speed from driver or what autonomous wants
        if (m_forwardJS > 1.0) {
            jsY = m_joystick->GetY();
        } else {
            jsY = m_forwardJS;
            if ((m_visionDistance < 65.0) && (m_forwardJS < -0.2)) {
                jsY = -0.2;
            } else if ((m_visionDistance < 50.0) && (m_forwardJS < -0.1)) {
                jsY = -0.15;
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
        if (m_holdYaw) {
            jsT = m_yawJStwist;
        }
        m_robotDrive->MecanumDrive_Cartesian(jsX, jsY, jsT, 0.0);
    }
}

void DriveSubsystem::DriveDeadRecon()
{
    double jsX, jsY, jsT, gyroAngle;

    jsX = m_speed*sin(m_dir);
    jsY = -m_speed*cos(m_dir);

    /* Rotate the robot if the trigger being held or yaw is being maintained */
    jsT = 0.0;
    if (m_holdYaw) {
        jsT = m_yawJStwist;
    }
    gyroAngle = 0.0;
#if NAVX
    if (m_fieldAbsolute && m_ahrs) {
        gyroAngle = m_ahrs->GetAngle();
    }
#endif
#if IMU_MXP
    if (m_fieldAbsolute && m_imu) {
        gyroAngle = m_imu->GetAngle();
    }
#endif

    /* Move the robot */
    m_robotDrive->MecanumDrive_Cartesian(jsX, jsY, jsT, gyroAngle);
}

void DriveSubsystem::DriveManual()
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
    //if (m_inAutonomous && m_pRobot->IsPinSensed() && (jsY < 0.0)) {
    //    jsY = 0.0;
    //}
    if (m_pRobot->IsInAutoDropMode() && m_pRobot->IsPinSensed() && m_autoDriveButton->GetBool() && (jsY < 0.0)) {
        jsY = 0.0;
    }
    // If not in autonomous, operator is in autodrop mode, robot is still touching pin, and the gear has been dropped
    if (!m_inAutonomous && m_pRobot->IsInAutoDropMode() && m_pRobot->IsGearDropped()) {
        BackoffPin();
    }

    /* Rotate the robot if the trigger being held or yaw is being maintained */
    jsT = 0.0;
    if (m_holdYaw) {
        jsT = m_yawJStwist;
    }
    if (m_joystick->GetTrigger()) {
        HoldYaw(false);
        jsT = m_joystick->GetTwist();
    }

    gyroAngle = 0.0;
#if NAVX
    if (m_fieldAbsolute && m_ahrs) {
        gyroAngle = m_ahrs->GetAngle();
    }
#endif
#if IMU_MXP
    if (m_fieldAbsolute && m_imu) {
        gyroAngle = m_imu->GetAngle();
    }
#endif
#if NAVX || IMU_MXP
    if (m_autoYawButton && m_autoYawButton->GetBool() && jsAngle < 360.0) {
        deltaAngle = fabs(GetRobotYaw() - jsAngle);
        // If robot is facing wrong direction,
        // don't pivot robot all the way around, drive "backwards"
        if ((deltaAngle > 90.0) && (deltaAngle < 270.0)) {
            jsAngle = jsAngle + 180.0;
            if (jsAngle > 180.0) jsAngle -= 360.0;
            SetYawDirection(jsAngle);
        } else {
            SetYawDirection(jsAngle);
        }
        HoldYaw(true);
    }
#endif

    /* Move the robot */
    m_robotDrive->MecanumDrive_Cartesian(jsX, jsY, jsT, gyroAngle);
}

void DriveSubsystem::TestPeriodic()
{
    GetVisionData();
}

void DriveSubsystem::LogHeader(FILE *fp)
{
    fputs("rpi_seq,missingRPiCount,vTargetsFound,TargetsBelowMin,vLateral,vDist,rawFwdJS,rawLatJS,rawYawJS,",fp);
}

void DriveSubsystem::LogData(FILE *fp)
{
    fprintf(fp,"%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,",m_rpi_seq,m_missingRPiCount,
            m_visionTargetsFound,m_targetsBelowMinDistance,m_visionLateral,m_visionDistance,
            m_forwardJS,m_lateralJS,m_yawJStwist);
}

void DriveSubsystem::UpdateDashboard(void)
{
#if NAVX || IMU_MXP
#if NAVX
    if (m_ahrs) {
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
    SmartDashboard::PutBoolean("Drive HoldYaw", m_holdYaw);
    SmartDashboard::PutNumber("Drive HoldYaw Angle", m_yawAngle);
}
