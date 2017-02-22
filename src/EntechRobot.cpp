#include <WPILib.h>

#include "EntechRobot.h"
#include "RobotConstants.h"

#define LOG_FILE "EntechRobotLog.csv"

// Sample do nothing change

EntechRobot::EntechRobot()
    : m_drive(NULL)
    , m_climber(NULL)
    , m_shooter(NULL)
    , m_dropper(NULL)
    , m_pickup(NULL)
    , m_compressor(NULL)
    , m_lw(NULL)
    , m_logFP(NULL)
    , m_joystick(NULL)
    , m_climbButton(NULL)
    , m_descendButton(NULL)
    , m_climbgrabButton(NULL)
    , m_dropButton(NULL)
    , m_pickupButton(NULL)
    , m_autodropButton(NULL)

    , m_autonomousActive(true)
    , m_autoSelectionD1(NULL)
    , m_autoSelectionD2(NULL)
    , m_autoSelectionD3(NULL)
    , m_autoState(kStart)
    , m_autoTimer(NULL)

    , m_prefs(NULL)
    , m_shooterSpeed(0.0)
{
    m_robotSubsystems.clear();
}

EntechRobot::~EntechRobot() {}

void EntechRobot::OpenLog(void)
{
    m_logFP = fopen(LOG_FILE, "w");
    if (m_logFP) {
        fputs("autoState,",m_logFP);
        
        for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
             it != m_robotSubsystems.end(); ++it) {
            (*it)->LogHeader(m_logFP);
        }
        fputs("\n",m_logFP);
    }
}

void EntechRobot::WriteLog(void)
{
    if (m_logFP) {
        fprintf(m_logFP,"%d,",m_autoState);
        for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
             it != m_robotSubsystems.end(); ++it) {
            (*it)->LogData(m_logFP);
        }
        fputs("\n",m_logFP);
    }
}

void EntechRobot::CloseLog(void)
{
    if (m_logFP)
        fclose(m_logFP);
    m_logFP = NULL;
}

void EntechRobot::RobotInit()
{
    NetworkTable::SetServerMode();
    NetworkTable::SetUpdateRate(0.050);
    m_lw = frc::LiveWindow::GetInstance();
    m_drive = new DriveSubsystem(this,"drive");
    m_climber = new ClimberSubsystem(this, "climber");
    m_shooter = new ShooterSubsystem(this, "shooter");
    m_dropper = new DropperSubsystem(this, "dropper");
    m_pickup = new PickUpSubsystem(this, "pickup");

    m_compressor = new frc::Compressor(c_compressorPCMid);
    if (m_compressor) {
        m_compressor->SetClosedLoopControl(true);
        m_compressor->Start();
    }
    
    m_joystick = new Joystick(c_operatorJSid);
    if (m_joystick) {
        m_climbgrabButton = new OperatorButton(m_joystick,c_opclimbgrab_BTNid);
        m_climbButton = new OperatorButton(m_joystick,c_opclimb_BTNid);
        m_descendButton = new OperatorButton(m_joystick,c_opdescend_BTNid);
        m_pickupButton = new OperatorButton(m_joystick,c_oppickup_BTNid);
        m_autodropButton = new OperatorButton(m_joystick,c_opautodrop_BTNid);
        m_dropButton = new OperatorButton(m_joystick,c_opdrop_BTNid);
    }

    m_autoState = kStart;
    m_autoTimer = new frc::Timer();
    m_autoSelectionD1 = new frc::DigitalInput(c_autoSelectorD1Channel);
    m_autoSelectionD2 = new frc::DigitalInput(c_autoSelectorD2Channel);
    m_autoSelectionD3 = new frc::DigitalInput(c_autoSelectorD3Channel);

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

    m_prefs = frc::Preferences::GetInstance();

    UpdateDashboard();
}

void EntechRobot::DetermineAutonomousSetup(void)
{
    bool yaw_left;
    bool yaw_right;
    bool boiler_to_left;

    // jumpered = False, unjumpered = True
    // D1: jumpered = Boiler to robot left
    //     unjumpered = Boiler to robot right
    // D2: jumpered = Yaw left
    // D3: jumpered = Yaw right
    // D2 & D3 both jumpered -- no Autonomous
    // D2 & D3 both unjumpered -- straight

    boiler_to_left = m_autoSelectionD1->Get();
    yaw_left = !m_autoSelectionD2->Get();
    yaw_right = !m_autoSelectionD3->Get();

    m_autonomousActive = true;
    if (yaw_left && yaw_right) {
        // Both jumpered -- No autonomous!!
        m_autonomousActive = false;
    } else if ((!yaw_left) && (!yaw_right)) {
        m_boilerDistance = kMiddle;
        m_initialTurn = kStraight;
    } else if (yaw_left) {
        m_boilerDistance = kNear;
        if (boiler_to_left)
            m_boilerDistance = kFar;
        m_initialTurn = kLeft60;
    } else if (yaw_right) {
        m_boilerDistance = kFar;
        if (boiler_to_left)
            m_boilerDistance = kNear;
        m_initialTurn = kRight60;
    } else {
    	// impossible
    }

    // Set shooter speed based on boiler distance
    m_shooterSpeed = 0.0;
    switch (m_boilerDistance) {
    case kNear:
        m_shooterSpeed = m_prefs->GetDouble("shooterSpeedNear", -0.75);
        break;
    case kMiddle:
    	m_shooterSpeed = m_prefs->GetDouble("shooterSpeedMiddle", -0.9);
        break;
    case kFar:
        m_shooterSpeed = m_prefs->GetDouble("shooterSpeedFar", -1.0);
        break;
    }
}

bool EntechRobot::IsGearDropTriggered(void)
{
    return m_dropper->IsGearDropped();
}

void EntechRobot::DisabledInit()
{
    CloseLog();
    
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->DisabledInit();
    }

    UpdateDashboard();
}

void EntechRobot::DisabledPeriodic()
{
    DetermineAutonomousSetup();
    
    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->DisabledPeriodic();
    }

    UpdateDashboard();
}

void EntechRobot::TeleopInit()
{
    CloseLog();

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->TeleopInit();
    }

    UpdateDashboard();
}

void EntechRobot::TeleopPeriodic()
{
    if (m_joystick) {
        m_climber->Off();
        if (m_climbButton->GetBool()) {
            m_climber->Forward();
        }
        if (m_descendButton->GetBool()) {
            m_climber->Backward();
        }
        if (m_climbgrabButton->GetBool()) {
            m_climber->Grab();
        }
        if (m_pickupButton ->GetBool()) {
            m_pickup->SetPosition(PickUpSubsystem::kDown);
        }else{
            m_pickup->SetPosition(PickUpSubsystem::kUp);
        }
        if (m_autodropButton->GetBool()) {
            m_dropper->SetMode(DropperSubsystem::kAutomatic);
        } else {
            m_dropper->SetMode(DropperSubsystem::kManual);
            if (m_dropButton ->GetBool()) {
                m_dropper->SetPosition(DropperSubsystem::kDown);
            } else {
                m_dropper->SetPosition(DropperSubsystem::kUp);
            }
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
    m_autonomousActive = true;
    DetermineAutonomousSetup();

    if (m_autonomousActive) {
        m_autoState = kStart;
        OpenLog();
    } else {
        m_autoState = kDone;
    }
    m_autoTimer->Stop();
    m_autoTimer->Reset();

    for (std::list<RobotSubsystem*>::iterator it = m_robotSubsystems.begin();
         it != m_robotSubsystems.end(); ++it) {
        (*it)->AutonomousInit();
    }

    UpdateDashboard();
}

void EntechRobot::AutonomousPeriodic()
{
    switch(m_autoState) {
    case kStart:
        m_autoState = kTurnOnShooter;
        break;
    case kTurnOnShooter:
    	m_shooter->Forward(m_shooterSpeed);
        m_autoTimer->Start();
        m_autoState = kWaitForShooterToSpinup;
        break;
    case kWaitForShooterToSpinup:
        if (m_autoTimer->Get() > 0.5) {
            m_autoState = kShootFuelLoad;
        }
        break;
    case kShootFuelLoad:
        m_shooter->ShootAll();
        m_autoState = kWaitForShootFuelLoad;
        break;
    case kWaitForShootFuelLoad:
        if (m_autoTimer->Get() > 2.5) {
            m_shooter->Off();
            if (m_initialTurn == kStraight) {
                m_drive->SetYawDirection(0.0);
                m_autoState = kDriveToTarget;
            } else {
                m_autoState = kInitialDrive;
            }
        }
        break;
    case kInitialDrive:
        m_drive->FieldAbsoluteDriving(true);
        m_drive->HoldYaw(true);
        m_drive->SetYawDirection(0.0);
        m_drive->DriveHeading(0.0, 0.75, 0.9); 
        m_autoState = kWaitForInitialDrive;
        break;
    case kWaitForInitialDrive:
        if (m_drive->Done()) {
            m_autoState = kInitialTurn;
        }
        break;
    case kInitialTurn:
        if (m_initialTurn == kRight60) {
            m_drive->HoldYaw(true);
            m_drive->SetYawDirection(60.0);
        } else if (m_initialTurn == kLeft60) {
            m_drive->HoldYaw(true);
            m_drive->SetYawDirection(-60.0);
        }
        m_drive->DriveHeading(0.0, 0.75, 0.50); 
        m_autoState = kWaitForInitialTurn;
        break;
    case kWaitForInitialTurn:
        if (m_drive->Done()) {
            m_autoState = kDriveToTarget;
        }
        break;
    case kDriveToTarget:
        m_dropper->SetMode(DropperSubsystem::kAutomatic);
        m_drive->DriveToVisionTarget(-0.15);
        m_autoState = kWaitForDriveToTarget;
        break;
    case kWaitForDriveToTarget:
        if (m_dropper->IsGearDropped()) {
            m_drive->DriveHeading(0.0,0.0,0.0);
            m_autoState = kDriveBackward;
        }
        break;
    case kDriveBackward:
        switch (m_initialTurn) {
        case kLeft60:
            m_drive->DriveHeading(120.0, 0.45, 0.75);
            break;
        case kRight60:
            m_drive->DriveHeading(-120.0, 0.45, 0.75);
            break;
        case kStraight:
            m_drive->DriveHeading(180.0, 0.30, 1.8);
            break;
        }
        m_autoState = kWaitForDriveBackward;
        break;
    case kWaitForDriveBackward:
        if (m_drive->Done()) {
            m_drive->DriveHeading(0.0,0.0,0.0);
            // skip lateral drive for initial turn cases
            switch (m_initialTurn) {
            case kLeft60:
            case kRight60:
                m_autoState = kDriveForward;
                break;
            case kStraight:
                m_autoState = kDriveLateral;
                break;
            }                
        }
        break;
    case kDriveLateral:
        // only occurs for drive straight
        m_drive->DriveHeading(90.0, 0.80, 2.0);
        m_autoState = kWaitForDriveLateral;
        break;
    case kWaitForDriveLateral:
        if (m_drive->Done()) {
            m_autoState = kDriveForward;
        }
        break;
    case kDriveForward:
        m_dropper->SetMode(DropperSubsystem::kManual);
        m_dropper->SetPosition(DropperSubsystem::kUp);
        m_drive->SetYawDirection(0.0);
        switch (m_initialTurn) {
        case kLeft60:
        case kRight60:
            m_drive->DriveHeading(0.0, 0.70, 3.0);
            break;
        case kStraight:
            m_drive->DriveHeading(0.0, 0.70, 3.5);
            break;
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
    
    WriteLog();
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
    SmartDashboard::PutBoolean("Autonomous Active", m_autonomousActive);
    SmartDashboard::PutNumber("Autonomous State", m_autoState);
    switch (m_boilerDistance) {
    case kNear:
        SmartDashboard::PutString("Boiler Distance", "Near");
        break;
    case kMiddle:
        SmartDashboard::PutString("Boiler Distance", "Middle");
        break;
    case kFar:
        SmartDashboard::PutString("Boiler Distance", "Far");
        break;
    }
    SmartDashboard::PutNumber("Shooter Speed",m_shooterSpeed);

    switch (m_initialTurn) {
    case kRight60:
        SmartDashboard::PutString("Initial Turn", "Right60");
        break;
    case kStraight:
        SmartDashboard::PutString("Initial Turn", "None");
        break;
    case kLeft60:
        SmartDashboard::PutString("Initial Turn", "Left60");
        break;
    }
        
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
