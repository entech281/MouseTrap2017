#ifndef _ROBOTCONSTANTS_H
#define _ROBOTCONSTANTS_H

// Joystick Ports
const int c_joystickLeftPort = 0;

// Drive motor ports
const int c_flmotor_CANid = 1;
const int c_rlmotor_CANid = 2;
const int c_frmotor_CANid = 16;
const int c_rrmotor_CANid = 14;
const int c_climberMotor_CANid = 3;
const int c_compressorPCMid = 10;
const int c_ShooterMotor_CANid = 4;

// Drive motor inversion states
const bool c_kflmotor_inversed = false;
const bool c_krlmotor_inversed = false;
const bool c_kfrmotor_inversed = true;
const bool c_krrmotor_inversed = true;

// Driver Joystick buttons
const int c_jsthumb_BTNid = 2;
const int c_jsfieldAbs_BTNid = 6;

// Operator joystick buttons
const int c_opclimb_BTNid = 1;
const int c_opautodrop_BTNid = 2;
const int c_opdescend_BTNid = 3;
const int c_oppickup_BTNid = 7;
const int c_opdrop_BTNid = 8;


//Pneumatic Solenoid
const int c_dropperSolenoidChannel1		= 1;
const int c_dropperSolenoidChannel2		= 0;
const int c_pickupSolenoidChannel1     = 2;

//sensors - Digital Inputs
const int c_dropperSensor = 1;


#endif
