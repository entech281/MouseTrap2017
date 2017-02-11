#include <WPILib.h>

#include "EntechRobot.h"
#include "RobotConstants.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

// Sample do nothing change

EntechRobot::EntechRobot()
    : m_drive(NULL)
    , m_climber(NULL)
    , m_shooter(NULL)
    , m_lw(NULL)
{
    m_robotSubsystems.clear();
}

EntechRobot::~EntechRobot() {}

void EntechRobot::RobotInit()
{
    m_lw = frc::LiveWindow::GetInstance();
    m_drive = new DriveSubsystem(this,"drive");
    m_climber = new ClimberSubsystem(this, "climber");
    m_shooter = new ShooterSubsystem(this, "shooter");

    m_compressor = new Compressor(c_compressorPCMid);
    if (m_compressor) {
        m_compressor->SetClosedLoopControl(true);
        m_compressor->Start();
    }

    m_joystick = new Joystick(1);
    m_climbButton = new OperatorButton(m_joystick,c_opclimb_BTNid);
    m_descendButton = new OperatorButton(m_joystick,c_opdescend_BTNid);
    m_pickupButton = new OperatorButton(m_joystick,c_oppickup_BTNid);
    m_autodropButton = new OperatorButton(m_joystick,c_opautodrop_BTNid);
    m_dropButton = new OperatorButton(m_joystick,c_opdrop_BTNid);

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
    m_climber->Off();
    if (m_climbButton->GetBool()) {
        m_climber->Forward();
    }
    if (m_descendButton->GetBool()) {
        m_climber->Backward();
    }
    if (m_pickupButton ->GetBool()) {
        m_pickup->SetPosition(PickUpSubsystem::kDown);
    }else{
        m_pickup->SetPosition(PickUpSubsystem::kUp);
    }
    if (m_autodropButton ->GetBool()) {
        m_dropper->SetMode(DropperSubsystem::kAutomatic);
    } else {
        m_dropper->SetMode(DropperSubsystem::kManual);
        if (m_dropButton ->GetBool()) {
            m_dropper->SetPosition(DropperSubsystem::kDown);
        } else {
            m_dropper->SetPosition(DropperSubsystem::kUp);
        }
    }

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TeleopPeriodic();
    }

    UpdateDashboard();
}

void EntechRobot::AutonomousInit()
{
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->AutonomousInit();
    }

    UpdateDashboard();
}

void EntechRobot::AutonomousPeriodic()
{
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
