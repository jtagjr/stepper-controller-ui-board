
#ifndef __ROTARY_ENCODER_H__
#define __ROTARY_ENCODER_H__

#include "mbed.h"

enum RotarySwitchState
{
    ROTARY_STATE_INIT = 0,
    ROTARY_STATE_WAITING_CW,
    ROTARY_STATE_CW,
    ROTARY_STATE_CW_ENDING,
    ROTARY_STATE_WAITING_CCW,
    ROTARY_STATE_CCW,
    ROTARY_STATE_CCW_ENDING
};

enum ButtonState
{
    ButtonState_DOWN = 0,
    ButtonState_UP
};

#define BUTTON_DEBOUNCE_MILLISECONDS 3

typedef void (*ButtonDownCallback)();
void init_button(DigitalIn& pin);
void button_task(DigitalIn& pin, ButtonState& state, ButtonDownCallback callback);
void debounce_delay(int delayMilliseconds);
void rotary_switch_init(RotarySwitchState& currentState);
void rotary_switch_task(RotarySwitchState& currentState, DigitalIn& low_first_cw, DigitalIn& low_second_cw, ButtonDownCallback cw_event, ButtonDownCallback ccw_event);
RotarySwitchState rotary_switch_state(const RotarySwitchState switchState, DigitalIn& rotary_line_low_1st_cw, DigitalIn& rotary_line_low_2nd_cw);

#endif