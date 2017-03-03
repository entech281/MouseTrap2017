#include <WPILib.h>

#include "EntechRobot.h"
#include "RobotConstants.h"

#define LOG_FILE "/tmp/EntechRobotLog.csv"

const double c_shooterSpeedNear = 1500.0;
const double c_shooterSpeedMiddle = 3360.0;
const double c_shooterSpeedFar = 4350.0;
const double c_shooterSpeedSide = 3000.0;

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
    , m_gamepad(NULL)
    , m_gp_useShooterPID(NULL)
    , m_gp_climbButton(NULL)
    , m_gp_descendButton(NULL)
    , m_gp_dropButton(NULL)
    , m_gp_pickupButton(NULL)
    , m_gp_autodropButton(NULL)
    , m_buttonpanel(NULL)
    , m_bp_climbButton(NULL)
    , m_bp_dropButton(NULL)
    , m_bp_autodropButton(NULL)
    , m_bp_shooterOnButton(NULL)
    , m_bp_fireButton(NULL)

    , m_autonomousActive(true)
    , m_autoSelectionD1(NULL)
    , m_autoSelectionD2(NULL)
    , m_autoSelectionD3(NULL)
    , m_autoState(kStart)
	, m_boilerDistance(kMiddle)
    , m_initialTurn(kStraight)
    , m_boilerToLeft(false)
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
    NetworkTable::SetUpdateRate(0.02);
    m_lw = frc::LiveWindow::GetInstance();
    m_drive = new DriveSubsystem(this,"drive");
    m_climber = new ClimberSubsystem(this, "climber");
    m_shooter = new ShooterSubsystem(this, "shooter");
    m_dropper = new DropperSubsystem(this, "dropper");
#if PICKUP
    m_pickup = new PickUpSubsystem(this, "pickup");
#endif

    m_compressor = new frc::Compressor(c_compressorPCMid);
    if (m_compressor) {
        m_compressor->SetClosedLoopControl(true);
        m_compressor->Start();
    }
    
    m_gamepad = new Joystick(c_operatorGPid);
    if (m_gamepad) {
    	m_gp_useShooterPID = new OperatorButton(m_gamepad,4);
        m_gp_climbButton = new OperatorButton(m_gamepad,c_gpclimb_BTNid);
        m_gp_descendButton = new OperatorButton(m_gamepad,c_gpdescend_BTNid);
        m_gp_pickupButton = new OperatorButton(m_gamepad,c_gppickup_BTNid);
        m_gp_autodropButton = new OperatorButton(m_gamepad,c_gpautodrop_BTNid);
        m_gp_dropButton = new OperatorButton(m_gamepad,c_gpdrop_BTNid);
    }
    m_buttonpanel = new Joystick(c_operatorBPid);
    if (m_buttonpanel) {
        m_bp_climbButton = new OperatorButton(m_buttonpanel,c_opclimb_BTNid);
        m_bp_dropButton = new OperatorButton(m_buttonpanel,c_opdrop_BTNid);
        m_bp_autodropButton = new OperatorButton(m_buttonpanel,c_opautodrop_BTNid);
        m_bp_shooterOnButton = new OperatorButton(m_buttonpanel,c_opshooterOn_BTNid);
        m_bp_fireButton = new OperatorButton(m_buttonpanel,c_opfire_BTNid);
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

    // jumpered = False, unjumpered = True
    // D1: jumpered = Boiler to robot left
    //     unjumpered = Boiler to robot right
    // D2: jumpered = Yaw left
    // D3: jumpered = Yaw right
    // D2 & D3 both jumpered -- no Autonomous
    // D2 & D3 both unjumpered -- straight

    m_boilerToLeft = m_autoSelectionD1->Get();
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
        if (m_boilerToLeft)
            m_boilerDistance = kFar;
        m_initialTurn = kLeft60;
    } else if (yaw_right) {
        m_boilerDistance = kFar;
        if (m_boilerToLeft)
            m_boilerDistance = kNear;
        m_initialTurn = kRight60;
    } else {
    	// impossible
    }
    if (m_boilerToLeft)
    	m_boilerDistance = kSiderail;

    // Set shooter speed based on boiler distance
    m_shooterSpeed = 0.0;
    switch (m_boilerDistance) {
    case kNear:
        m_shooterSpeed = m_prefs->GetDouble("shooterSpeedNear", c_shooterSpeedNear);
        break;
    case kMiddle:
    	m_shooterSpeed = m_prefs->GetDouble("shooterSpeedMiddle", c_shooterSpeedMiddle);
        break;
    case kFar:
    	m_shooterSpeed = m_prefs->GetDouble("shooterSpeedFar", c_shooterSpeedFar);
        break;
    case kSiderail:
    	m_shooterSpeed = m_prefs->GetDouble("shooterSpeedSide", c_shooterSpeedSide);
        break;
    }
}

bool EntechRobot::IsGearDropTriggered(void)
{
    return m_dropper->IsGearDropped();
}

bool EntechRobot::IsPinSensed(void)
{
    return m_dropper->IsPinSensed();
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
    if ((m_gp_climbButton && m_gp_climbButton->GetBool()) ||
        (m_bp_climbButton && m_bp_climbButton->GetBool())    ) {
        m_climber->Climb();
    } else if (m_gp_descendButton && m_gp_descendButton->GetBool()) {
        m_climber->Backward();
    } else {
        m_climber->Off();
    }

    if (m_pickup) {
      	if (m_gp_pickupButton && m_gp_pickupButton->GetBool()) {
       		m_pickup->SetPosition(PickUpSubsystem::kDown);
       	} else {
       		m_pickup->SetPosition(PickUpSubsystem::kUp);
       	}
    }

    if ((m_gp_autodropButton && m_gp_autodropButton->GetBool()) ||
        (m_bp_autodropButton && m_bp_autodropButton->GetBool())    ) {
        m_dropper->SetMode(DropperSubsystem::kAutomatic);
    } else {
        m_dropper->SetMode(DropperSubsystem::kManual);
        if ((m_gp_dropButton && m_gp_dropButton->GetBool()) ||
            (m_bp_dropButton && m_bp_dropButton->GetBool())    ) {
            m_dropper->SetPosition(DropperSubsystem::kDown);
        } else {
            m_dropper->SetPosition(DropperSubsystem::kUp);
        }
    }

    if (m_gp_useShooterPID && m_gp_useShooterPID->GetBool()) {
    	m_shooter->SetRPM(m_shooterSpeed);
    } else if (m_bp_shooterOnButton && m_bp_shooterOnButton->GetBool()) {
        m_shooter->Forward(0.5*(1.0-m_buttonpanel->GetX()));
    } else {
        m_shooter->Forward(0.0);
    }
    if (m_bp_fireButton && m_bp_fireButton->GetBool()) {
        m_shooter->TriggerOpen();
    } else {
        m_shooter->TriggerClose();
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
        if (m_boilerToLeft) {
            if (m_initialTurn == kStraight) {
                m_drive->SetYawDirection(0.0);
                m_drive->HoldYaw(true);
                m_autoState = kDriveToTarget;
            } else {
                m_autoState = kInitialDrive;
            }
        }
        break;
    case kTurnOnShooter:
    	m_shooter->SetRPM(m_shooterSpeed);
        m_autoState = kWaitForShooterToSpinup;
        break;
    case kWaitForShooterToSpinup:
        if (m_shooter->IsAtTargetRPM()) {
            m_autoState = kShootFuelLoad;
        }
        break;
    case kShootFuelLoad:
        m_shooter->TriggerOpen();
        m_autoTimer->Stop();
        m_autoTimer->Reset();
        m_autoTimer->Start();
        m_autoState = kWaitForShootFuelLoad;
        break;
    case kWaitForShootFuelLoad:
        if (m_autoTimer->Get() > 4.5) {
            m_shooter->Forward(0.0);
            m_shooter->TriggerClose();
            if (m_initialTurn == kStraight) {
                m_drive->SetYawDirection(0.0);
                m_drive->HoldYaw(true);
                m_autoState = kDriveToTarget;
            } else if (m_boilerDistance == kSiderail) {
            	m_autoState = kDone;
            } else {
                m_autoState = kInitialDrive;
            }
        }
        break;
    case kInitialDrive:
        m_drive->FieldAbsoluteDriving(true);
        m_drive->HoldYaw(true);
        m_drive->SetYawDirection(0.0);
        m_drive->DriveHeading(0.0, 0.75, 0.75);
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
        m_drive->DriveToVisionTarget(-0.25);
        m_autoState = kWaitForDriveToTarget;
        break;
    case kWaitForDriveToTarget:
        if (m_dropper->IsPinSensed()) {
            m_drive->DriveHeading(0.0,0.0,0.0);
        }
        if (m_dropper->IsGearDropped()) {
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
            m_drive->DriveHeading(180.0, 0.40, 1.5);
            break;
        }
        m_autoState = kWaitForDriveBackward;
        break;
    case kWaitForDriveBackward:
        if (m_drive->Done()) {
            m_drive->DriveHeading(0.0,0.0,0.0);
            m_dropper->SetMode(DropperSubsystem::kManual);
            m_dropper->SetPosition(DropperSubsystem::kUp);
            // skip lateral drive for initial turn cases
            switch (m_initialTurn) {
            case kLeft60:
                m_autoState = kDriveForward;
                break;
            case kRight60:
                if (m_boilerToLeft) {
                    m_autoState = kSetSideShotYaw;
                } else {
                    m_autoState = kDriveForward;
                }
                break;
            case kStraight:
                if (m_boilerToLeft) {
                    m_autoState = kSetSideShotYaw;
                } else {
                    m_autoState = kDriveLateral;
                }
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
    case kSetSideShotYaw:
        m_dropper->SetMode(DropperSubsystem::kManual);
        m_dropper->SetPosition(DropperSubsystem::kUp);
        m_drive->SetYawDirection(90.0);
        m_drive->HoldYaw(true);
        m_autoState = kWaitForSetSideShotYaw;
        break;
    case kWaitForSetSideShotYaw:
        if (m_initialTurn == kStraight) {
            m_autoState = kClearAirship;
        } else {
            m_autoState = kAlignToTarget;
        }
        break;
    case kClearAirship:
        m_drive->DriveHeading(-90.0,0.8,1.75);
        m_autoState = kWaitForClearAirship;
        break;
    case kWaitForClearAirship:
        if (m_drive->Done()) {
            m_autoState = kAlignToTarget;
        }
        break;
    case kAlignToTarget:
        m_drive->AlignWithTargetFacing(90.0,-0.3);
        m_autoState = kWaitForAlignToTarget;
        break;
    case kWaitForAlignToTarget:
        if (m_drive->IsAlignmentCorrect()) {
            m_autoState = kBackupToWall;
        }
        break;
    case kBackupToWall:
        m_drive->DriveToVisionTarget(0.2,false);
        m_autoTimer->Stop();
        m_autoTimer->Reset();
        m_autoTimer->Start();
        m_autoState = kWaitForBackupToWall;
        break;
    case kWaitForBackupToWall:
        if ((m_autoTimer->Get() > 0.5) && m_drive->Stopped()) {
            m_autoState = kTurnOnShooter;
        }
        break;
    case kDone:
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
    case kSiderail:
        SmartDashboard::PutString("Boiler Distance", "Siderail");
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
