#include <WPILib.h>

#include "EntechRobot.h"
#include "RobotConstants.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

// Sample do nothing change

EntechRobot::EntechRobot()
    : m_drive(NULL)
    , m_gearDrop(NULL)
    , m_shooter(NULL)
    , m_climber(NULL)
    , m_lw(NULL)
    , m_autoSelectorLeft(NULL)
    , m_autoSelectorMiddle(NULL)
    , m_autoSelectorRight(NULL)
    , m_autoTimer(NULL)
{
    m_robotSubsystems.clear();
}

EntechRobot::~EntechRobot() {}

void EntechRobot::RobotInit()
{
    m_lw = frc::LiveWindow::GetInstance();
    m_drive = new DriveSubsystem(this,"drive");

    m_autoState = kStart;
    m_autoTimer = new Timer();
    m_autoSelectorLeft   = new frc::DigitalInput(c_autoSelectorLeftChannel);
    m_autoSelectorMiddle = new frc::DigitalInput(c_autoSelectorMiddleChannel);
    m_autoSelectorRight  = new frc::DigitalInput(c_autoSelectorRightChannel);
    m_autoSection = 0;
    if (m_autoSelectorLeft->Get())
        m_autoSelection += 1;
    if (m_autoSelectorLeft->Get())
        m_autoSelection += 2;
    if (m_autoSelectorRight->Get())
        m_autoSelection += 4;

#if USB_CAMERA
    std::thread t_visionThread(VisionThread);
    t_visionThread.detach();
#endif

    /* 
     * Iterate through each sub-system and run the
     * appropriate function for the current mode.
     * Descriptions for each mode can be found in
     * RobotSubsystem.h
     */

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->RobotInit();
    }

    UpdateDashboard();
}

#if USB_CAMERA
void EntechRobot::VisionThread()
{
	cs::UsbCamera t_camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
	t_camera.SetResolution(640,480);
	cs::CvSink t_cvSink = frc::CameraServer::GetInstance()->GetVideo();
	cs::CvSource t_outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
	cv::Mat t_source;
	cv::Mat t_output;
	while (true) {
		t_cvSink.GrabFrame(t_source);
		cvtColor(t_source, t_output, cv::COLOR_BGR2GRAY);
		t_outputStreamStd.PutFrame(t_output);
	}
}
#endif

void EntechRobot::DisabledInit()
{
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->DisabledInit();
    }

    UpdateDashboard();
}

void EntechRobot::DisabledPeriodic()
{
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->DisabledPeriodic();
    }

    UpdateDashboard();
}

void EntechRobot::TeleopInit()
{
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TeleopInit();
    }

    UpdateDashboard();
}

void EntechRobot::TeleopPeriodic()
{
    if (m_autoDriveButton->Get() == OperatorButton::kPressed) {
        m_drive->DriveToTarget();
        m_gearDrop->SetMode(GearDropSubsystem::kAutomatic);
    } else {
        m_gearDrop->SetMode(GearDropSubsystem::kManual);
        m_gearDrop->Drop(m_geardropButton->GetBoolean());
    }
    if (m_climbButton->Get() == OperatorButton::kPressed) {
        m_climber->Climb();
    } else {
        m_climber->Stop();
    }
    
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TeleopPeriodic();
    }

    UpdateDashboard();
}

void EntechRobot::AutonomousInit()
{
    m_autoState = kStart;
    m_autoTimer->Stop();
    m_autoTimer->Reset();

    m_autoSection = 0;
    if (m_autoSelectorLeft->Get())
        m_autoSelection += 1;
    if (m_autoSelectorLeft->Get())
        m_autoSelection += 2;
    if (m_autoSelectorRight->Get())
        m_autoSelection += 4;
    
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->AutonomousInit();
    }

    UpdateDashboard();
}

void EntechRobot::AutonomousPeriodic()
{
    // If no autonomous selection is available -- abort
    if (m_autoSection == 0)
        m_autoState = kDone;
    
    switch(m_autoState){
    case kStart:
        if ((m_autoSelection == 1) || (m_autoSelection == 4)) {
            m_autoState = kInitialDrive;
        } else {
            m_autoState = kDriveToTarget;
        }
        break;
    case kInitialDrive:
        m_drive->SetFieldAbsoluteDriving(true);
        m_drive->DriveHeading(0.0, 0.60, 1.0); // need values!
        if (m_autoSelection == 1) {
            m_drive->YawToHeading(60.0);
        } else if (m_autoSelection == 4) {
            m_drive->YawToHeading(-60.0);
        }
        m_autoState = kWaitForInitialDrive;
        break;
    case kWaitForInitialDrive:
        if (m_drive->Done()){
            m_autoState = kDriveToTarget;
        }
        break;
    case kDriveToTarget:
        m_gearDrop->SetMode(GearDropSubsystem::kAutomatic);
        m_drive->AutoToTarget();
        m_autoState = kWaitForDriveToTarget;
        break;
    case kWaitForDriveToTarget:
        if (m_gearDrop->DropMade()) {
            m_drive->Stop();
            m_autoState = kShootFuelLoad;
        }
        break;
    case kShootFuelLoad:
        m_shooter->ShootAll();
        m_autoState = kWaitForShootFuelLoad;
        break;
    case kWaitForShootFuelLoad:
        if (m_shooter->Done()) {
            m_autoState = kDriveBackward;
        }
        break;
    case kDriveBackward:
        if (m_autoSelection == 1) {
            m_drive->DriveHeading(120.0, 0.60, 0.5); // need values!
        } else if (m_autoSelection == 2) {
            m_drive->DriveHeading(-120.0, 0.60, 0.5); // need values!
        } else if (m_autoSelection == 4) {
            m_drive->DriveHeading(0.0, 0.60, 0.5); // need values!
        }
        m_autoState = kWaitForDriveBackward;
        break;
    case kWaitForDriveBackward:
        if (m_drive->Done()) {
            m_autoState = kDriveLateral;
        }
        break;
    case kDriveLateral:
        m_gearDrop->Hold();
        if (m_autoSelection == 1) {
            m_drive->DriveHeading(90.0, 0.60, 1.0); // need values!
        } else if (m_autoSelection == 2) {
            m_drive->DriveHeading(90.0, 0.60, 2.0); // need values!
        } else if (m_autoSelection == 4) {
            m_drive->DriveHeading(-90.0, 0.60, 1.0); // need values!
        }
        m_autoState = kWaitForDriveLateral;
        break;
    case kWaitForDriveLateral:
        if (m_drive->Done()) {
            m_autoState = kDriveForward;
        }
        break;
    case kDriveLateral:
        if (m_autoSelection == 1) {
            m_drive->DriveHeading(0.0, 0.60, 2.0); // need values!
        } else if (m_autoSelection == 2) {
            m_drive->DriveHeading(0.0, 0.60, 3.0); // need values!
        } else if (m_autoSelection == 4) {
            m_drive->DriveHeading(0.0, 0.60, 2.0); // need values!
        }
        m_autoState = kWaitForDriveForward;
        break;
    case kWaitForDriveForward:
        if (m_drive->Done()) {
            m_autoState = kDone;
        }
        break;
    case kDone:
        break;
    default:
        break;
    }

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->AutonomousPeriodic();
    }

    UpdateDashboard();
}

void EntechRobot::TestInit()
{
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TestInit();
    }

    UpdateDashboard();
}

void EntechRobot::TestPeriodic()
{
    /* Update Live Window */
    m_lw->Run();

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TestPeriodic();
    }

    UpdateDashboard();
}

void EntechRobot::UpdateDashboard()
{
    SmartDashboard::PutNumber("Autonomous Selection (L1,M2,R4)", m_autoSection);
    SmartDashboard::PutNumber("Autonomous State", m_autoState);
    
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
       (*it)->UpdateDashboard();
    }
}

void EntechRobot::RegisterSubsystem(RobotSubsystem* subsys)
{
    m_robotSubsystems.push_back(subsys);
}

START_ROBOT_CLASS(EntechRobot);
