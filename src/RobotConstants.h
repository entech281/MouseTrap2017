#ifndef _ROBOTCONSTANTS_H
#define _ROBOTCONSTANTS_H

// Joystick Ports
const int c_joystickLeftPort = 0;

// Drive motor ports
const int c_flmotor_CANid = 2;
const int c_rlmotor_CANid = 4;
const int c_frmotor_CANid = 1;
const int c_rrmotor_CANid = 3;

// Drive motor inversion states
const bool c_kflmotor_inverted = false;
const bool c_krlmotor_inverted = false;
const bool c_kfrmotor_inverted = true;
const bool c_krrmotor_inverted = true;

// Joystick buttons
const int c_jsthumb_BTNid = 2;
const int c_jsfieldAbs_BTNid = 6;

// Digital Inputs/Outputs
const int c_autoSelectorLeftChannel   = 1;
const int c_autoSelectorMiddleChannel = 2;
const int c_autoSelectorRightChannel  = 3;

#endif
