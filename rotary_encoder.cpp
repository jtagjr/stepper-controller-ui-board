#include "rotary_encoder.h"

void debounce_delay(int delayMilliseconds)
{
    Timer timer;
    timer.start();
    while (timer.elapsed_time().count() < delayMilliseconds);
}

void rotary_switch_init(RotarySwitchState& currentState)
{
    currentState = ROTARY_STATE_INIT;
}

void rotary_switch_task(RotarySwitchState& currentState, 
                        DigitalIn& low_first_cw, 
                        DigitalIn& low_second_cw,
                        ButtonDownCallback cw_event, 
                        ButtonDownCallback ccw_event)
{
    do
    {
        debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
        
        RotarySwitchState newState = rotary_switch_state(currentState, low_first_cw, low_second_cw);

        if (currentState == ROTARY_STATE_CW_ENDING && newState == ROTARY_STATE_INIT)
        {
            cw_event();
        }
        else if (currentState == ROTARY_STATE_CCW_ENDING && newState == ROTARY_STATE_INIT)
        {
            ccw_event();
        }

        currentState = newState;
    } while(currentState != ROTARY_STATE_INIT);
}

RotarySwitchState rotary_switch_state(const RotarySwitchState switchState, DigitalIn& rotary_line_low_1st_cw, DigitalIn& rotary_line_low_2nd_cw)
{
    switch (switchState)
    {
        case ROTARY_STATE_INIT:
            if (rotary_line_low_1st_cw.read() == 0)
            {
                if (rotary_line_low_2nd_cw.read() == 1)
                {
                    return ROTARY_STATE_WAITING_CW;
                }
            }
            else if (rotary_line_low_2nd_cw.read() == 0)
            {
                if (rotary_line_low_1st_cw.read() == 1)
                {
                    return ROTARY_STATE_WAITING_CCW;
                }
            }
            break;

        case ROTARY_STATE_WAITING_CW:
            if (rotary_line_low_1st_cw.read() == 0)
            {
                if (rotary_line_low_2nd_cw.read() == 0)
                {
                    return ROTARY_STATE_CW;
                }
                else 
                {
                    return ROTARY_STATE_WAITING_CW;
                }
            }
            break;

        case ROTARY_STATE_CW:
            if (rotary_line_low_1st_cw.read() == 1)
            {
                if (rotary_line_low_2nd_cw.read() == 0)
                {
                    return ROTARY_STATE_CW_ENDING;
                }
                else 
                {
                    return ROTARY_STATE_CW;
                }
            }
            else if (rotary_line_low_1st_cw.read() == 0 && rotary_line_low_2nd_cw.read() == 0)
            {
                return ROTARY_STATE_CW;
            }
            break;

        case ROTARY_STATE_CW_ENDING:
            if (rotary_line_low_1st_cw.read() == 1)
            {
                if (rotary_line_low_2nd_cw.read() == 1)
                {
                    return ROTARY_STATE_INIT;
                }
                else 
                {
                    return ROTARY_STATE_CW_ENDING;
                }
            }
            break;

        case ROTARY_STATE_WAITING_CCW:
            if (rotary_line_low_2nd_cw.read() == 0)
            {
                if (rotary_line_low_1st_cw.read() == 0)
                {
                    return ROTARY_STATE_CCW;
                }
                else 
                {
                    return ROTARY_STATE_WAITING_CCW;
                }
            }
            break;

        case ROTARY_STATE_CCW:
            if (rotary_line_low_2nd_cw.read() == 1)
            {
                if (rotary_line_low_1st_cw.read() == 0)
                {
                    return ROTARY_STATE_CCW_ENDING;
                }
                else 
                {
                    return ROTARY_STATE_CCW;
                }
            }
            else if (rotary_line_low_1st_cw.read() == 0 && rotary_line_low_2nd_cw.read() == 0)
            {
                return ROTARY_STATE_CCW;
            }
            break;

        case ROTARY_STATE_CCW_ENDING:
            if (rotary_line_low_2nd_cw.read() == 1)
            {
                if (rotary_line_low_1st_cw.read() == 1)
                {
                    return ROTARY_STATE_INIT;
                }
                else 
                {
                    return ROTARY_STATE_CCW_ENDING;
                }
            }
            break;

        default:
            break;
    }

    return ROTARY_STATE_INIT;
}

void init_button(DigitalIn& pin)
{
    pin.mode(PinMode::PullUp);
}

void button_task(DigitalIn& pin, ButtonState& state, ButtonDownCallback callback)
{
    if (pin == 0)
    {
        if (state == ButtonState_UP)
        {
            state = ButtonState_DOWN;
            if (callback)
            {
                callback();
            }
            debounce_delay(BUTTON_DEBOUNCE_MILLISECONDS*1000);
        }
    }
    else
    {
        state = ButtonState_UP;
    }
}