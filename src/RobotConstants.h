#ifndef _ROBOTCONSTANTS_H
#define _ROBOTCONSTANTS_H

// Camera definitions
#define USB_CAMERA  1
#define AXIS_CAMERA 0

// Joystick Ports
const int c_joystickLeftPort = 0;

// Drive motor ports
const int c_flmotor_PWMid = 0;
const int c_rlmotor_PWMid = 2;
const int c_frmotor_PWMid = 1;
const int c_rrmotor_PWMid = 4;

// Drive motor inversion states
const bool c_kflmotor_inversed = false;
const bool c_krlmotor_inversed = false;
const bool c_kfrmotor_inversed = true;
const bool c_krrmotor_inversed = true;

// Feeder subsystem motors
const int c_feedermotor_PWMid = 7;
const int c_loadermotor_PWMid = 5;
const int c_shootermotor1_PWMid = 6;
const int c_shootermotor2_PWMid = 3;

// Joystick buttons
const int c_jsthumb_BTNid = 2;
const int c_jsfieldAbs_BTNid = 6;

#endif
