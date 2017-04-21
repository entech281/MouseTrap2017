#include <WPILib.h>

#include "EntechRobot.h"
#include "RobotConstants.h"

#define LOG_FILE "/home/lvuser/EntechRobotLog.csv"

const double c_shooterSpeedNear = 1500.0;
const double c_shooterSpeedMiddle = 3100.0;
const double c_shooterSpeedFar = 4350.0;
const double c_shooterSpeedSide = 3000.0;
const double c_shooterTimeMax = 4.5;

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
    , m_gp_fireButton(NULL)
    , m_buttonpanel(NULL)
    , m_bp_climbButton(NULL)
    , m_bp_dropButton(NULL)
    , m_bp_autodropButton(NULL)
    , m_bp_shooterOnButton(NULL)
    , m_bp_fireButton(NULL)
    , m_bp_yawLeftButton(NULL)
    , m_bp_yawRightButton(NULL)
    , m_bp_yawZeroButton(NULL)

    , m_dropState(kDropDone)
    , m_dropTimer(NULL)

    , m_autonomousActive(true)
    , m_autoSelectionD1(NULL)
    , m_autoSelectionD2(NULL)
    , m_autoSelectionD3(NULL)
    , m_autoState(kStart)
    , m_autoSecondTry(false)
    , m_autoNudgeCount(0)
    , m_autoNudgeDir(kLeft)
    , m_boilerDistance(kMiddle)
    , m_initialTurn(kStraight)
    , m_boilerToLeft(false)
    , m_yawZeroed(false)
    , m_autoTimer(NULL)


    , m_prefs(NULL)
    , m_shooterSpeed(0.0)
    , m_shooterTime(4.5)
{
    m_robotSubsystems.clear();
    NetworkTable::SetServerMode();
    NetworkTable::SetUpdateRate(0.01);
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
        m_gp_useShooterPID = new OperatorButton(m_gamepad,c_gpshooterSpd_BTNid);
        m_gp_climbButton = new OperatorButton(m_gamepad,c_gpclimb_BTNid);
        m_gp_descendButton = new OperatorButton(m_gamepad,c_gpdescend_BTNid);
        m_gp_pickupButton = new OperatorButton(m_gamepad,c_gppickup_BTNid);
        m_gp_autodropButton = new OperatorButton(m_gamepad,c_gpautodrop_BTNid);
        m_gp_dropButton = new OperatorButton(m_gamepad,c_gpdrop_BTNid);
        m_gp_fireButton = new OperatorButton(m_gamepad,c_gpfire_BTNid);
    }
    m_buttonpanel = new Joystick(c_operatorBPid);
    if (m_buttonpanel) {
        m_bp_climbButton = new OperatorButton(m_buttonpanel,c_opclimb_BTNid);
        m_bp_dropButton = new OperatorButton(m_buttonpanel,c_opdrop_BTNid);
        m_bp_autodropButton = new OperatorButton(m_buttonpanel,c_opautodrop_BTNid);
        m_bp_shooterOnButton = new OperatorButton(m_buttonpanel,c_opshooterOn_BTNid);
        m_bp_fireButton = new OperatorButton(m_buttonpanel,c_opfire_BTNid);
        m_bp_yawLeftButton = new OperatorButton(m_buttonpanel,c_opyawleft_BTNid);
        m_bp_yawRightButton = new OperatorButton(m_buttonpanel,c_opyawright_BTNid);
        m_bp_yawZeroButton = new OperatorButton(m_buttonpanel,c_opyawzero_BTNid);
    }

    m_dropState = kDropDone;
    m_dropTimer = new frc::Timer();

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
    //if (m_boilerToLeft)
    //    m_boilerDistance = kSiderail;

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
    m_shooterTime = m_prefs->GetDouble("shooterTime",c_shooterTimeMax);
    if (m_shooterTime > c_shooterTimeMax)
        m_shooterTime = c_shooterTimeMax;
}

bool EntechRobot::IsGearDropped(void)
{
    return m_dropper->IsGearDropped();
}

bool EntechRobot::IsPinSensed(void)
{
    return m_dropper->IsPinSensed();
}

bool EntechRobot::IsInAutoDropMode(void)
{
    if ((m_gp_autodropButton && m_gp_autodropButton->GetBool()) ||
        (m_bp_autodropButton && m_bp_autodropButton->GetBool())    ) {
        return true;
    }
    return false;
}

double EntechRobot::InitialYaw(void)
{
    if (m_yawZeroed)
        return 0.0;
    if ((m_boilerDistance == kFar) && m_boilerToLeft)
    	return 180.0;
	return 0.0;
}

void EntechRobot::ZeroInitialYaw(void)
{
    m_yawZeroed = true;
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

    if (IsInAutoDropMode()) {
        if ((m_dropState == kDropStart) || (m_dropState == kDropWaitForGearRelease)) {
            m_dropper->SetMode(DropperSubsystem::kAutomatic);
        }
        StartGearDropAction();
        DoGearDropAction();
    } else {
        ResetGearDropAction();
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
    if ((m_bp_fireButton && m_bp_fireButton->GetBool()) ||
        (m_gp_fireButton && m_gp_fireButton->GetBool())    ){
        m_shooter->TriggerOpen();
    } else {
        m_shooter->TriggerClose();
    }

    if (m_bp_yawLeftButton && m_bp_yawLeftButton->GetBool()) {
        m_drive->SetYawDirection(-60.0);
        m_drive->HoldYaw(true);
    } else if (m_bp_yawRightButton && m_bp_yawRightButton->GetBool()) {
        m_drive->SetYawDirection(60.0);
        m_drive->HoldYaw(true);
    } else if (m_bp_yawZeroButton && m_bp_yawZeroButton->GetBool()) {
        m_drive->SetYawDirection(0.0);
        m_drive->HoldYaw(true);
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
    m_autoSecondTry = false;
    m_autoNudgeDir = kLeft;
    m_autoNudgeCount = 0;
    
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

void EntechRobot::ResetGearDropAction(void)
{
    m_dropState = kDropStart;
}

void EntechRobot::StartGearDropAction(void)
{
    if (m_dropState == kDropStart)
        m_dropState = kDropWaitForGearRelease;
}

void EntechRobot::DoGearDropAction(void)
{
    if (m_dropState == kDropDone)
        return;

    switch (m_dropState) {
    case kDropStart:
    case kDropDone:
        break;
    case kDropWaitForGearRelease:
        if (m_dropper->IsGearDropped()) {
            m_dropState = kDropBackoff;
        }
        break;
    case kDropBackoff:
        m_drive->DriveHeading(m_drive->GetRobotYaw(),-0.55,0.18);
        m_dropState = kWaitForDropBackoff;
        break;
    case kWaitForDropBackoff:
        if (m_drive->Done()) {
            m_dropState = kDropRaise;
        }
        break;
    case kDropRaise:
        m_dropper->SetMode(DropperSubsystem::kManual);
        m_dropper->SetPosition(DropperSubsystem::kUp);
        m_dropTimer->Stop();
        m_dropTimer->Reset();
        m_dropTimer->Start();
        m_dropState = kWaitForDropRaise;
        break;
    case kWaitForDropRaise:
        if (m_dropTimer->Get() > 0.25) {
            m_dropState = kDropForward;
            m_dropTimer->Stop();
            m_dropTimer->Reset();
        }
        break;
    case kDropForward:
        m_drive->DriveHeading(m_drive->GetRobotYaw(),0.5,0.30);
        m_dropState = kWaitForDropForward;
        break;
    case kWaitForDropForward:
        if (m_drive->Done() || m_dropper->IsPinSensed()) {
            m_dropState = kDropBackup;
        }
        break;
    case kDropBackup:
        m_drive->DriveHeading(m_drive->GetRobotYaw(),-0.65,0.3);
        m_dropState = kWaitForDropBackup;
        break;
    case kWaitForDropBackup:
        if (m_drive->Done()) {
            m_dropState = kDropDone;
        }
        break;
    }
}

void EntechRobot::AutonomousPeriodic()
{
    double dropTimeout;

    dropTimeout = 4.5;
    if (m_initialTurn == kStraight) {
        dropTimeout = 6.0;
    }
    if (m_autoSecondTry) {
        dropTimeout = 2.0;
    }
    switch(m_autoState) {
    case kStart:
        // gear first -- if in middle
        if (m_initialTurn == kStraight) {
            m_drive->SetYawDirection(0.0);
            m_drive->HoldYaw(true);
            m_autoState = kDriveToTarget;
        } else if (m_boilerDistance == kFar) {
            m_autoState = kTurnOnShooter;
        } else  {
            m_autoState = kInitialDrive;
        }
        break;
    case kInitialDrive:
        m_drive->FieldAbsoluteDriving(true);
        m_drive->HoldYaw(true);
        m_drive->SetYawDirection(InitialYaw());
        m_drive->DriveHeading(0.0, 0.35, 0.75);
        m_autoState = kWaitForInitialDrive;
        break;
    case kWaitForInitialDrive:
        if (m_drive->Done()) {
            m_autoState = kInitialTurn;
        }
        break;
    case kInitialTurn:
        if (m_initialTurn == kRight60) {
            m_drive->SetYawDirection(60.0);
            m_drive->DriveHeading(-25.0, 0.35, 8.0);
            m_drive->HoldYaw(true);
        } else if (m_initialTurn == kLeft60) {
            m_drive->SetYawDirection(-60.0);
            m_drive->DriveHeading(25.0, 0.35, 8.0);
            m_drive->HoldYaw(true);
        }
        m_autoState = kWaitForInitialTurn;
        break;
    case kWaitForInitialTurn:
        if (m_drive->AreTargetsVisible()) {
            m_drive->DriveHeading(0.0,0.0,0.0);
            m_autoState = kDriveToTarget;
        }
        break;
    case kDriveToTarget:
        ResetGearDropAction();
        StartGearDropAction();
        m_dropper->SetMode(DropperSubsystem::kAutomatic);
        m_drive->DriveToVisionTarget(-0.22,true);
        m_autoState = kWaitForDriveToTarget;
        m_autoTimer->Stop();
        m_autoTimer->Reset();
        m_autoTimer->Start();
        break;
    case kWaitForDriveToTarget:
        if (m_dropper->IsPinSensed() && !m_dropper->IsGearDropped()) {
            m_drive->DriveHeading(0.0,0.0,0.0);
            m_autoTimer->Stop();
            m_autoTimer->Reset();
        }
        DoGearDropAction();
        if (m_dropState == kDropDone) {
            m_autoState = kDriveBackward;
            m_autoTimer->Stop();
            m_autoTimer->Reset();
        }
//        if (m_dropper->IsGearDropped()) {
//            m_autoState = kDriveBackward;
//            m_autoTimer->Stop();
//            m_autoTimer->Reset();
//        }
        if (m_autoTimer->Get() > dropTimeout) {
            m_autoSecondTry = true;
            m_autoState = kNudgeBack;
        }
        break;
    case kDriveBackward:
        switch (m_initialTurn) {
        case kLeft60:
            m_drive->DriveHeading(120.0, 0.35, 0.5);
            break;
        case kRight60:
            m_drive->DriveHeading(-120.0, 0.35, 0.5);
            break;
        case kStraight:
            m_drive->DriveHeading(180.0, 0.30, 0.1);
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
                m_autoState = kDriveForward;
                break;
            case kStraight:
                m_autoState = kBackupToEndWall;
                break;
            }
/*  Do Nudge Instead
            if (m_autoSecondTry) {
                m_autoSecondTry = false;
                m_autoState = kDriveToTarget;
            }
*/
            m_autoSecondTry = false;
        }
        break;
    case kBackupToEndWall:
        if (m_boilerToLeft) {
            m_drive->SetYawDirection(180.0);
            m_drive->HoldYaw(true);
        } else {
            m_drive->SetYawDirection(0.0);
            m_drive->HoldYaw(true);
        }
        m_shooter->SetRPM(m_shooterSpeed);
        m_drive->DriveHeading(180.0,0.2,3.0);
        m_autoTimer->Stop();
        m_autoTimer->Reset();
        m_autoTimer->Start();
        m_autoState = kWaitForBackupToEndWall;
        break;
    case kWaitForBackupToEndWall:
        if ((m_autoTimer->Get() > 0.5) && (m_drive->Stopped() || m_drive->Done())) {
            m_autoState = kWaitForShooterToSpinup;
        }
        break;
    case kTurnOnShooter:
        m_shooter->SetRPM(m_shooterSpeed);
        m_autoState = kWaitForShooterToSpinup;
        m_autoTimer->Stop();
        m_autoTimer->Reset();
        m_autoTimer->Start();
        break;
    case kWaitForShooterToSpinup:
        // Give shooter a max of 2 seconds to spin up
        if ((m_shooter->IsAtTargetRPM()) || (m_autoTimer->Get() > 2.0)) {
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
        if (m_autoTimer->Get() > m_shooterTime) {
            m_shooter->Forward(0.0);
            m_shooter->TriggerClose();
            if (m_initialTurn == kStraight) {
                m_drive->SetYawDirection(0.0);
                m_drive->HoldYaw(true);
                m_drive->DriveHeading(0.0,0.4,0.5);
                m_autoState = kWaitForDriveForward;
            } else if (m_boilerDistance == kSiderail) {
                m_autoState = kDone;
            } else {
                m_autoState = kInitialDrive;
            }
        }
        break;
    case kDriveLateral:
        // only occurs for drive straight
        if (m_boilerToLeft) {
            m_drive->DriveHeading(90.0, 0.80, 2.0);
        } else {
            m_drive->DriveHeading(-90.0, 0.80, 2.0);
        }
        m_autoState = kWaitForDriveLateral;
        break;
    case kWaitForDriveLateral:
        if (m_drive->Done()) {
            m_autoState = kDriveForward;
        }
        break;
    case kDriveForward:
        m_drive->SetYawDirection(0.0);
        m_drive->HoldYaw(true);
        switch (m_initialTurn) {
        case kLeft60:
        case kRight60:
            m_drive->DriveHeading(0.0, 0.60, 3.0);
            break;
        case kStraight:
            m_drive->DriveHeading(0.0, 0.60, 3.5);
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
    case kNudgeBack:
        m_autoSecondTry = true;
        m_drive->NudgeBackward();
        m_autoState = kWaitForNudgeBack;
        break;
    case kWaitForNudgeBack:
        if (m_drive->Done()) {
            m_autoState = kNudgeSide;
        }
        break;
    case kNudgeSide:
        ++m_autoNudgeCount;
        if (m_autoNudgeDir == kLeft) {
            m_drive->NudgeLeft(m_autoNudgeCount);
            m_autoNudgeDir = kRight;
        } else {
            m_drive->NudgeRight(m_autoNudgeCount);
            m_autoNudgeDir = kLeft;
        }
        m_autoState = kWaitForNudgeSide;
        break;
    case kWaitForNudgeSide:
        if (m_drive->Done()) {
            m_drive->DriveRobotHeading(0.0,0.25,2.0);
            m_autoTimer->Stop();
            m_autoTimer->Reset();
            m_autoTimer->Start();
            m_autoState = kWaitForDriveToTarget;
        }
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
