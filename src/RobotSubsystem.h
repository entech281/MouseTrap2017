#ifndef ROBOT_SUBSYSTEM_H
#define ROBOT_SUBSYSTEM_H

#include <stdio.h>

class EntechRobot;

class RobotSubsystem {
private:
    std::string m_name;

public:
    RobotSubsystem(EntechRobot *pRobot, std::string name = "unknown");
    virtual ~RobotSubsystem();

    const char* GetName(void);
    virtual void UpdateDashboard(void);
    virtual void LogHeader(FILE *fp);
    virtual void LogData(FILE *fp);

    /********************************** Init Routines **********************************/

    /*
     * Robot-wide initialization code which will
     * be called when the robot is first powered on.
     * It will be called exactly once.
     */
    virtual void RobotInit();

    /*
     * Initialization code which will be called
     * each time the robot enters disabled mode.
     */
    virtual void DisabledInit();

    /*
     * Initialization code which will be called
     * each time the robot enters teleop mode.
     */
    virtual void TeleopInit();

    /*
     * Initialization code which will be called
     * each time the robot enters autonomous mode.
     */
    virtual void AutonomousInit();

    /*
     * Initialization code which will be called
     * each time the robot enters test mode.
     */
    virtual void TestInit();

    /********************************** Periodic Routines **********************************/

    /*
     * Code which will be called periodically
     * at a regular rate while the robot is in
     * disabled mode.
     *
     * Disabled is the mode where the robot
     * should be disabled.
     */
    virtual void DisabledPeriodic();

    /*
     * Code which will be called periodically
     * at a regular rate while the robot is in
     * autonomous mode.
     *
     * Autonomous is the mode where user input
     * is disallowed for a short time at the
     * beginning of a match.
     */
    virtual void AutonomousPeriodic();

    /*
     * Code which will be called periodically
     * at a regular rate while the robot is in
     * teleop mode.
     *
     * Teleoperation is the mode where the user
     * gives input to the robot in order to complete
     * the challenge. This is the core of any challenge.
     */
    virtual void TeleopPeriodic();

    /*
     * Code which will be called periodically
     * at a regular rate while the robot is in
     * test mode.
     *
     * Test is the mode where the user can
     * pull up a live window to monitor the
     * robot and try out different teleop code.
     */
    virtual void TestPeriodic();
};
#endif
