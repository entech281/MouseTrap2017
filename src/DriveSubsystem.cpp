#include <WPILib.h>
#include <math.h>

#include "DriveSubsystem.h"
#include "RobotConstants.h"
#include "EntechRobot.h"

#ifndef M_PI
#define M_PI  3.1415926
#endif

const static double kYaw_P = 0.03;
const static double kYaw_I = 0.0;
const static double kYaw_D = 0.0;
const static double kYaw_ToleranceDegrees = 2.0;

const static double kLateral_P = 0.03;
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

    , m_ahrs(NULL)
    , m_visionTargetsFound(false)
    , m_visionLateral(0.0)
    , m_visionDistance(100.0)
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

    , m_fieldAbsolute(false)
    , m_holdYaw(false)

    , m_fieldAbsoluteToggleButton(NULL)
    , m_holdYawToggleButton(NULL)
    , m_yawToP60Button(NULL)
    , m_yawToZeroButton(NULL)
    , m_yawToM60Button(NULL)
    , m_autoDriveButton(NULL)
{
}

DriveSubsystem::~DriveSubsystem() {}

void DriveSubsystem::DriveHeading(double angle, double speed, double time)
{
    m_dir = angle*M_PI/180.0;
    m_speed = speed;
    m_time = time;
    m_currMode = kDeadRecon;
    m_timer->Stop();
    m_timer->Reset();
    m_timer->Start();
    m_currMode = kDeadRecon;
}

void DriveSubsystem::DriveToVisionTarget(void)
{
    if (m_ahrs) {
        double yaw = m_ahrs->GetYaw();
        if (yaw < -30.0) {
            SetYawDirection(-60.0);
        } else if (yaw > 30.0) {
            SetYawDirection(60.0);
        } else {
            SetYawDirection(0.0);
        }
        HoldYaw(true);
    }

    m_lateralController->SetSetpoint(0.0);
    m_lateralController->Enable();
    m_distanceController->SetSetpoint(0.0);
    m_distanceController->Enable();

    m_currMode = kAutomatic;
}

bool DriveSubsystem::Done(void)
{
    if (m_currMode == kManual) 
        return true;
    return false;
}

void DriveSubsystem::FieldAbsoluteDriving(bool active)
{
    m_fieldAbsolute = active;
}

void DriveSubsystem::HoldYaw(bool active)
{
    m_holdYaw = active;
    if (m_holdYaw) {
        m_yawController->Enable();
    } else {
        m_yawController->Disable();
    }
}

void DriveSubsystem::SetYawDirection(double angle)
{
    m_yawAngle = angle;
    m_yawController->SetSetpoint(m_yawAngle);
}

/********************************** Init Routines **********************************/

void DriveSubsystem::RobotInit()
{
    // Try creating the NavX first, to give it time to calibrate
    try {
    	m_ahrs = new AHRS(SerialPort::Port::kUSB);
        DriverStation::ReportWarning("NavX found");
        m_ahrs->Reset();
    } catch (std::exception& ex) {
        DriverStation::ReportError("NavX MISSING");
        m_ahrs = NULL;
    }

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
    m_ntTable = NetworkTable::GetTable("Vision");
    m_yawPIDInterface = new PIDInterface(m_ahrs, &m_yawJStwist);
    m_lateralPIDInterface = new PIDInterface(&m_visionLateral, &m_lateralJS);
    m_distancePIDInterface = new PIDInterface(&m_visionDistance, &m_forwardJS);

    m_yawController = new frc::PIDController(kYaw_P, kYaw_I, kYaw_D, m_yawPIDInterface, m_yawPIDInterface);
    m_yawController->SetAbsoluteTolerance(kYaw_ToleranceDegrees);
    m_yawController->SetInputRange(-180.0, 180.0);
    m_yawController->SetContinuous(true);
    m_yawController->SetOutputRange(-1.0, 1.0);
    m_yawController->Disable();

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
    m_joystick = new Joystick(0);
    m_fieldAbsoluteToggleButton = new OperatorButton(m_joystick, c_jsfieldAbs_BTNid);
    m_holdYawToggleButton = new OperatorButton(m_joystick, c_jsHoldYaw_BTNid);
    m_yawToP60Button  = new OperatorButton(m_joystick, c_jsYawToP60_BTNid);
    m_yawToZeroButton = new OperatorButton(m_joystick, c_jsYawToZero_BTNid);
    m_yawToM60Button  = new OperatorButton(m_joystick, c_jsYawToM60_BTNid);
    m_autoDriveButton = new OperatorButton(m_joystick, c_jsthumb_BTNid);

    // OK make sure the NavX has finished calibrating
    if (m_ahrs) {
        while (m_ahrs->IsCalibrating()) {
            Wait(0.05);
        }
        m_ahrs->ZeroYaw();
    }
}

void DriveSubsystem::DisabledInit()
{
    m_currMode = kManual;
}

void DriveSubsystem::TeleopInit()
{
    m_currMode = kManual;
}

void DriveSubsystem::AutonomousInit()
{
    m_currMode = kManual;
}

void DriveSubsystem::TestInit()
{
    m_currMode = kManual;
}

/********************************** Periodic Routines **********************************/

void DriveSubsystem::GetVisionData()
{
    if (m_ntTable->GetBoolean("RPi_alive", false)) {
        m_visionTargetsFound = m_ntTable->GetBoolean("targets",false);
        m_visionLateral = m_ntTable->GetNumber("lateral",0.0);
        m_visionDistance = m_ntTable->GetNumber("distance",100.0);
    }
}

void DriveSubsystem::DisabledPeriodic()
{
    GetVisionData();
    m_robotDrive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
}

void DriveSubsystem::TeleopPeriodic()
{
    double yaw;
    
    GetVisionData();

    // This is teleop, so manage driver inputs here
    if (m_fieldAbsoluteToggleButton->Get() == OperatorButton::kJustPressed) {
        FieldAbsoluteDriving(!m_fieldAbsolute);
    }
    if (m_holdYawToggleButton->Get() == OperatorButton::kJustPressed) {
        SetYawDirection(m_ahrs->GetYaw());
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
    if (m_visionTargetsFound && m_autoDriveButton->GetBool()) {
        if (m_currMode != kAutomatic) {
            if (m_ahrs) {
                yaw = m_ahrs->GetYaw();
                if (yaw < -30.0) {
                    SetYawDirection(-60.0);
                } else if (yaw > 30.0) {
                    SetYawDirection(60.0);
                } else {
                    SetYawDirection(0.0);
                }
                HoldYaw(true);
            }
            m_lateralController->SetSetpoint(0.0);
            m_lateralController->Enable();
            m_distanceController->SetSetpoint(0.0);
            m_distanceController->Enable();
        }
        m_currMode = kAutomatic;
    } else {
        if (m_currMode != kManual) {
            if (!m_holdYaw)
                m_yawController->Disable();
            m_lateralController->Disable();
            m_distanceController->Disable();
        }
        m_currMode = kManual;
    }
    switch (m_currMode) {
    case kManual:
    case kDeadRecon:
        DriveManual();
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
            m_currMode = kManual;
        }
        break;
    case kManual:
        m_robotDrive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
        break;
    case kAutomatic:
        DriveAutomatic();
        break;
    }
}

void DriveSubsystem::DriveAutomatic()
{
    m_robotDrive->MecanumDrive_Cartesian(m_lateralJS, m_forwardJS, m_yawJStwist, 0.0);
    if (m_pRobot->IsGearDropTriggered()) {
        m_currMode = kManual;
    }
}

void DriveSubsystem::DriveDeadRecon()
{
    double jsX, jsY, jsT, gyroAngle;

    jsX = m_speed*sin(m_dir);
    jsY = m_speed*cos(m_dir);

    /* Rotate the robot if the trigger being held or yaw is being maintained */
    jsT = 0.0;
    if (m_holdYaw) {
        jsT = m_yawJStwist;
    }
    gyroAngle = 0.0;
    if (m_fieldAbsolute && m_ahrs) {
    	gyroAngle = m_ahrs->GetAngle();
    }

    /* Move the robot */
    m_robotDrive->MecanumDrive_Cartesian(jsX, jsY, jsT, gyroAngle);
}

void DriveSubsystem::DriveManual()
{
    double jsX, jsY, jsT, gyroAngle;

    jsX = 0.0;
    jsY = 0.0;
    if (m_joystick) {
        jsX = m_joystick->GetX();
        jsY = m_joystick->GetY();
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
    if (m_fieldAbsolute && m_ahrs) {
    	gyroAngle = m_ahrs->GetAngle();
    }

    /* Move the robot */
    m_robotDrive->MecanumDrive_Cartesian(jsX, jsY, jsT, gyroAngle);
}

void DriveSubsystem::TestPeriodic()
{
    GetVisionData();
}

void DriveSubsystem::UpdateDashboard(void)
{
    if (m_ahrs) {
        SmartDashboard::PutData("NavX", m_ahrs);
        SmartDashboard::PutString("NavX Exists", "YES");
        SmartDashboard::PutNumber("NavX GetAngle()", m_ahrs->GetAngle()); //total accumulated yaw angle, 360+
        SmartDashboard::PutNumber("NavX GetYaw()", m_ahrs->GetYaw()); //-180 to 180 degrees
    } else {
        SmartDashboard::PutString("NavX Exists", "NO");
    }
    SmartDashboard::PutNumber("Drive FieldAbsolute", m_fieldAbsolute);
    SmartDashboard::PutBoolean("Vision Targets Found",m_visionTargetsFound);
    SmartDashboard::PutNumber("Vision Lateral", m_visionLateral);
    SmartDashboard::PutNumber("Vision Distance", m_visionDistance);
}
