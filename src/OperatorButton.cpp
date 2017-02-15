#include "OperatorButton.h"
#include "RobotConstants.h"

// Constructor for Joystick Button
OperatorButton::OperatorButton(frc::Joystick *js, int number)
	: m_js(js)
    , m_buttonNum(number)
    , m_lastState(false)
{
}

OperatorButton::OperatorButton(int js, int number)
	: m_buttonNum(number)
    , m_lastState(false)
{
	m_js = frc::Joystick::GetStickForPort(js);
}

OperatorButton::OperatorButton(int number)
	: m_buttonNum(number)
    , m_lastState(false)
{
	m_js = frc::Joystick::GetStickForPort(0);
}

OperatorButton::~OperatorButton()
{
}

OperatorButton::ButtonState OperatorButton::Get(void)
{
        bool curr_state;
        ButtonState state;

        if (m_js) {
        	curr_state = m_js->GetRawButton(m_buttonNum);
        } else {
            curr_state = false;
        }
        state = DetermineState(curr_state, m_lastState);
        m_lastState = curr_state;
        return state;
}

bool OperatorButton::GetBool(void)
{
    if (m_js) {
    	return m_js->GetRawButton(m_buttonNum);
    }
    return false;
}

OperatorButton::ButtonState OperatorButton::DetermineState(bool current, bool previous)
{
	if (current) {
		if (previous) {
			return kPressed;
        } else {
        	return kJustPressed;
        }
	} else {
		if (previous) {
			return kJustReleased;
        } else {
        	return kReleased;
        }
    }
}
