#ifndef _OPERATOR_BUTTON_H
#define _OPERATOR_BUTTON_H

#include <WPILib.h>

class OperatorButton {
public:
        enum ButtonState { kReleased=0, kPressed=1, kJustPressed, kJustReleased };

        OperatorButton(Joystick *js, int number);
        OperatorButton(int js, int number);
        OperatorButton(int number);
        ~OperatorButton();
        ButtonState Get(void);
        bool GetBool(void);

private:
        frc::Joystick *m_js;
        int m_buttonNum;
        bool m_lastState;

        ButtonState DetermineState(bool current, bool previous);
};

#endif
