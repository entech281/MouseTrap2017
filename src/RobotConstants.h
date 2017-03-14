#ifndef _ROBOTCONSTANTS_H
#define _ROBOTCONSTANTS_H

#define PICKUP  0

// Joystick Ports
const int c_driverJSid = 0;
const int c_operatorBPid = 1;
const int c_operatorGPid = 2;

// Drive motor ports
const int c_flmotor_CANid = 1;
const int c_rlmotor_CANid = 2;
const int c_frmotor_CANid = 16;
const int c_rrmotor_CANid = 14;
const int c_climberMotor_CANid = 3;
const int c_ShooterMotor_CANid = 4;
const int c_compressorPCMid = 10;

// Drive motor inversion states
const bool c_kflmotor_inverted = false;
const bool c_krlmotor_inverted = false;
const bool c_kfrmotor_inverted = true;
const bool c_krrmotor_inverted = true;

// Driver Joystick buttons
const int c_jsthumb_BTNid     = 2;
const int c_climbMode_BTNid   = 3;
const int c_jsYawReset_BTNid  = 5;
const int c_jsfieldAbs_BTNid  = 6;
const int c_jsYawToP60_BTNid  = 7;
const int c_jsYawToM60_BTNid  = 8;
const int c_jsYawToZero_BTNid = 9;
const int c_jsHoldYaw_BTNid   = 10;
const int c_jsAutoYaw_BTNid   = 11;

// Operator Gamepad buttons
const int c_gpclimb_BTNid = 1;
const int c_gpautodrop_BTNid = 2;
const int c_gpdescend_BTNid = 3;
const int c_gppickup_BTNid = 7;
const int c_gpdrop_BTNid = 8;

// Operator Panel buttons
const int c_opclimb_BTNid = 5;
const int c_opdrop_BTNid = 7;
const int c_opautodrop_BTNid = 6;
const int c_opshooterOn_BTNid = 3;
const int c_opfire_BTNid = 4;
const int c_opyawleft_BTNid = 8;
const int c_opyawzero_BTNid = 9;
const int c_opyawright_BTNid = 10;

// Digital Inputs/Outputs
const int c_dropperSensor = 0;
const int c_autoSelectorD1Channel = 7;
const int c_autoSelectorD2Channel = 8;
const int c_autoSelectorD3Channel = 9;

// Pneumatic Solenoids
const int c_dropperSolenoidChannel2  = 2;
const int c_dropperSolenoidChannel1  = 3;
const int c_pickupSolenoidChannel1   = 0;
const int c_pickupSolenoidChannel2   = 1;
const int c_shooterSolenoidChannel1   = 0;
const int c_shooterSolenoidChannel2   = 1;

#endif
