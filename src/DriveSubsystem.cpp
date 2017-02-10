#include <WPILib.h>

#include "DriveSubsystem.h"
#include "RobotConstants.h"

const static double kYaw_P = 0.03;
const static double kYaw_I = 0.0;
const static double kYaw_D = 0.0;
const static double kYaw_F = 0.0;
const static double kYaw_ToleranceDegrees = 2.0;

const static double kLateral_P = 0.03;
const static double kLateral_I = 0.0;
const static double kLateral_D = 0.0;
const static double kLateral_F = 0.0;
const static double kLateral_TolerancePixels = 2.0;

const static double kDistance_P = 0.03;
const static double kDistance_I = 0.0;
const static double kDistance_D = 0.0;
const static double kDistance_F = 0.0;
const static double kDistance_TolerancePixels = 2.0;

DriveSubsystem::DriveSubsystem(EntechRobot *pRobot, std::string name)
  : RobotSubsystem(pRobot, name)
  , m_joystick(NULL)
  , m_frmotor(NULL)
  , m_flmotor(NULL)
  , m_rrmotor(NULL)
  , m_rlmotor(NULL)
  , m_robotDrive(NULL)
  , m_ahrs(NULL)
  , m_yawController(NULL)
  , m_lateralController(NULL)
  , m_distanceController(NULL)
  , m_fieldAbsolute(false)
  , m_toggleFieldAbsoluteButton(NULL)
{
}

DriveSubsystem::~DriveSubsystem() {}

void DriveSubsystem::ToggleFieldAbsoluteDriving(void)
{
	SetFieldAbsoluteDriving(!m_fieldAbsolute);
}

void DriveSubsystem::SetFieldAbsoluteDriving(bool active)
{
	m_fieldAbsolute = active;
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
    m_ntTable = NetworkTable.GetTable("Vision");
    m_yawPIDInterface = new PIDInterface(m_ahrs, &m_yawJStwist);
    m_lateralPIDInterface = new PIDInterface(&m_visionLateral, &m_lateralJS);
    m_distancePIDInterface = new PIDInterface(&m_visionDistance, &m_forwardJS);

    m_yawController = new frc::PIDController(kYaw_P, kYaw_I, kYaw_F, m_yawPIDInterface, m_yawPIDInterface);
    m_yawController->SetAbsoluteTolerance(kYaw_ToleranceDegrees);
    m_yawController->SetInputRange(-180.0, 180.0);
    m_yawController->SetContinuous(true);
    m_yawController->SetOutputRange(-1.0, 1.0);
    m_yawController->Disable();

    m_lateralController = new frc::PIDController(kLateral_P, kLateral_I, kLateral_F, m_lateralPIDInterface, m_lateralPIDInterface);
    m_lateralController->SetAbsoluteTolerance(kLateral_TolerancePixels);
    m_lateralController->SetInputRange(-100.0, 100.0);
    m_lateralController->SetContinuous(false);
    m_lateralController->SetOutputRange(-1.0, 1.0);
    m_lateralController->Disable();

    m_distanceController = new frc::PIDController(kDistance_P, kDistance_I, kDistance_F, m_distancePIDInterface, m_distancePIDInterface);
    m_distanceController->SetAbsoluteTolerance(kDistance_TolerancePixels);
    m_distanceController->SetInputRange(-100.0, 100.0);
    m_distanceController->SetContinuous(false);
    m_distanceController->SetOutputRange(-1.0, 1.0);
    m_distanceController->Disable();

    // Driver interface on the buttons
    m_joystick = new Joystick(0);
    m_toggleFieldAbsoluteButton = new OperatorButton(m_joystick, c_jsfieldAbs_BTNid);
    m_yawToP60Button  = new OperatorButton(m_joystick, c_jsYawToP60_BTNid);
    m_yawToZeroButton = new OperatorButton(m_joystick, c_jsYawToZero_BTNid);
    m_yawToM60Button  = new OperatorButton(m_joystick, c_jsYawToM60_BTNid);
    m_autoDriveButton = new OperatorButton(m_joystick, c_jsthumb_BTNid);

    // OK make sure the NavX has finished calibrating
    if (m_ahrs) {
        while (m_ahrs->IsCalibrating()) {
            Wait(0.1);
        }
        m_ahrs->ZeroYaw();
    }
}

void DriveSubsystem::DisabledInit() {}

void DriveSubsystem::TeleopInit() {}

void DriveSubsystem::AutonomousInit() {}

void DriveSubsystem::TestInit() {}

/********************************** Periodic Routines **********************************/

void DriveSubsystem::GetVisionData()
{
    m_visionTargetsFound = m_ntTable.GetBoolean("targets",false);
    m_visionLateral = m_ntTable.GetNumber("lateral",0.0);
    m_visionDistance = m_ntTable.GetNumber("distance",100.0);
}

void DriveSubsystem::DisabledPeriodic()
{
    GetVisionData();
    m_robotDrive->MecanumDrive_Cartesian(0.0, 0.0, 0.0, 0.0);
}

void DriveSubsystem::TeleopPeriodic()
{
    GetVisionData();
    if (m_visionTargetsFound && m_autoDriveButton->GetBoolean()) {
        DriveAutomatic();
    } else {
        DriveManual();
    }
}

void DriveSubsystem::DriveAutomatic()
{
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

    /* Rotate the robot only if the trigger being held. */
    jsT = 0.0;
    if (m_joystick->GetTrigger()) {
        jsT = m_joystick->GetTwist();
    }

    if (m_toggleFieldAbsoluteButton->Get() == OperatorButton::kJustPressed) {
    	ToggleFieldAbsoluteDriving();
    }

    gyroAngle = 0.0;
    if (m_fieldAbsolute && m_ahrs) {
    	gyroAngle = m_ahrs->GetAngle();
    }

    /* Move the robot */
    m_robotDrive->MecanumDrive_Cartesian(jsX, jsY, jsT, gyroAngle);
}

void DriveSubsystem::AutonomousPeriodic()
{
    GetVisionData();
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

#if 0
// Example code from navex -- to be intergrated into DriveAuto
    void OperatorControl()
    {
        robotDrive.SetSafetyEnabled(false);
        while (IsOperatorControl() && IsEnabled())
        {
            bool reset_yaw_button_pressed = stick.GetRawButton(1);
            if ( reset_yaw_button_pressed ) {
                ahrs->ZeroYaw();
            }
            bool rotateToAngle = false;
            if ( stick.GetRawButton(2)) {
                turnController->SetSetpoint(0.0f);
                rotateToAngle = true;
            } else if ( stick.GetRawButton(3)) {
                turnController->SetSetpoint(90.0f);
                rotateToAngle = true;
            } else if ( stick.GetRawButton(4)) {
                turnController->SetSetpoint(179.9f);
                rotateToAngle = true;
            } else if ( stick.GetRawButton(5)) {
                turnController->SetSetpoint(-90.0f);
                rotateToAngle = true;
            }
            double currentRotationRate;
            if ( rotateToAngle ) {
                turnController->Enable();
                currentRotationRate = rotateToAngleRate;
            } else {
                turnController->Disable();
                currentRotationRate = stick.GetTwist();
            }
            try {
                /* Use the joystick X axis for lateral movement,          */
                /* Y axis for forward movement, and the current           */
                /* calculated rotation rate (or joystick Z axis),         */
                /* depending upon whether "rotate to angle" is active.    */
                robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(),
                                                  currentRotationRate ,ahrs->GetAngle());
            } catch (std::exception ex ) {
                std::string err_string = "Error communicating with Drive System:  ";
                err_string += ex.what();
                DriverStation::ReportError(err_string.c_str());
            }
            Wait(0.005); // wait 5ms to avoid hogging CPU cycles
        }
    }
#endif
