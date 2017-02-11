#include <WPILib.h>

#include "EntechRobot.h"
#include "RobotConstants.h"

// Sample do nothing change

EntechRobot::EntechRobot()
    : m_drive(NULL)
    , m_climber(NULL)
    , m_dropper(NULL)
    , m_compressor(NULL)
    , m_lw(NULL)
    , m_autoDriveButton(NULL)
    , m_geardropButton(NULL)
    , m_climbButton(NULL)

    , m_autoSelectorLeft(NULL)
    , m_autoSelectorMiddle(NULL)
    , m_autoSelectorRight(NULL)
    , m_autoSelection(0)
    , m_autoState(kStart)
    , m_autoTimer(NULL)
{
    m_robotSubsystems.clear();
}

EntechRobot::~EntechRobot() {}

void EntechRobot::RobotInit()
{
    m_lw = frc::LiveWindow::GetInstance();
    m_drive = new DriveSubsystem(this,"drive");
    m_climber = new ClimberSubsystem(this, "climber");

    m_compressor = new frc::Compressor(c_compressorPCMid);
    if (m_compressor) {
        m_compressor->SetClosedLoopControl(true);
        m_compressor->Start();
    }

    m_autoState = kStart;
    m_autoTimer = new frc::Timer();
    m_autoSelectorLeft   = new frc::DigitalInput(c_autoSelectorLeftChannel);
    m_autoSelectorMiddle = new frc::DigitalInput(c_autoSelectorMiddleChannel);
    m_autoSelectorRight  = new frc::DigitalInput(c_autoSelectorRightChannel);
    m_autoSelection = 0;
    if (m_autoSelectorLeft->Get())
        m_autoSelection += 1;
    if (m_autoSelectorLeft->Get())
        m_autoSelection += 2;
    if (m_autoSelectorRight->Get())
        m_autoSelection += 4;

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

bool EntechRobot::IsGearDropTriggered(void)
{
    // TODO  actual implementation
    return true;
}

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
        m_drive->DriveToVisionTarget();
        m_dropper->SetMode(DropperSubsystem::kAutomatic);
    } else {
        m_dropper->SetMode(DropperSubsystem::kManual);
        if (m_geardropButton->GetBool()) {
            m_dropper->SetPosition(DropperSubsystem::kDown);
        } else {
            m_dropper->SetPosition(DropperSubsystem::kUp);
        }
    }
    if (m_climbButton->Get() == OperatorButton::kPressed) {
        m_climber->Forward();
    } else {
        m_climber->Off();
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

    m_autoSelection = 0;
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
    if (m_autoSelection == 0)
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
        m_drive->FieldAbsoluteDriving(true);
        m_drive->DriveHeading(0.0, 0.60, 1.0); // need values!
        if (m_autoSelection == 1) {
            m_drive->SetYawDirection(60.0);
        } else if (m_autoSelection == 4) {
            m_drive->SetYawDirection(-60.0);
        }
        m_autoState = kWaitForInitialDrive;
        break;
    case kWaitForInitialDrive:
        if (m_drive->Done()){
            m_autoState = kDriveToTarget;
        }
        break;
    case kDriveToTarget:
        m_dropper->SetMode(DropperSubsystem::kAutomatic);
        m_drive->DriveToVisionTarget();
        m_autoState = kWaitForDriveToTarget;
        break;
    case kWaitForDriveToTarget:
        if (m_dropper->IsGearDropped()) {
            m_autoState = kShootFuelLoad;
        }
        break;
    case kShootFuelLoad:
//        m_shooter->ShootAll();
        m_autoState = kWaitForShootFuelLoad;
        break;
    case kWaitForShootFuelLoad:
//        if (m_shooter->Done()) {
            m_autoState = kDriveBackward;
//        }
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
        m_dropper->SetPosition(DropperSubsystem::kUp);
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
    case kDriveForward:
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
    SmartDashboard::PutNumber("Autonomous Selection (L1,M2,R4)", m_autoSelection);
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
