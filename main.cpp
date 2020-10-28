/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "rotary_encoder.h"
#include <analogin_api.h>
#include <cstdint>
#include <limits>
#include <cmath>
#include <Ticker.h>
#include <gpio_api.h>

#define WAIT_TIME_MS 50 
// max 5 revolutions per second to go from bottom to top in about 60 seconds
#define MAX_PULSES_PER_SECOND 3000*5
#define PULSES_PER_REVOLUTION 2000
#define INCREMENT_RATE_IN_MICROSECONDS 300
#define PULSES_PER_HALF_TEN_THOUSANDTH 2
#define PULSES_PER_ONE_THOUSANDTH 40

DigitalOut led1(LED1);

uint32_t steps_per_second_speed_curve(uint32_t desired_speed, int time_increment);
void update_pwm();

static volatile uint32_t s_desired_steps_per_second = 0;
static volatile uint32_t s_current_steps_per_second = 0;
static unsigned int s_last_speed_percent = 0;
static volatile int s_speed_increment = -20;
static volatile bool s_speed_ticker_running = false;
static int32_t s_last_voltage = 0;
static Ticker s_speed_ticker;  
static const unsigned int one_percent = 418;

// Bottom rotary switch
static DigitalIn s_rotary_line_bottom_low_1st_cw(PA_0); //CLK
static DigitalIn s_rotary_line_bottom_low_2nd_cw(PA_1); //DT

// Top rotary switch
static DigitalIn s_rotary_line_top_low_1st_cw(PA_3); //CLK
static DigitalIn s_rotary_line_top_low_2nd_cw(PA_4); //DT

static ButtonState s_up_button_state = ButtonState_UP;
static DigitalIn s_up_button(PA_5); 
static ButtonState s_down_button_state = ButtonState_UP;
static DigitalIn s_down_button(PA_6); 

static DigitalOut s_head_enable_pin(PA_2);
static DigitalOut s_head_direction_pin(PA_9);
static DigitalOut s_head_motor_rpm_pin(PWM_OUT);
static PwmOut s_pwm(PWM_OUT);

static RotarySwitchState s_top_rotary_encoder_state = ROTARY_STATE_INIT;
static RotarySwitchState s_bottom_rotary_encoder_state = ROTARY_STATE_INIT;

void move_grinder_head_task(analogin_t& speed_control);
void move_motor_head(analogin_t& speed_control);
void pulse_motor(bool direction_up, unsigned int number_of_pulses, DigitalOut&& pin);
void set_pwm_high();
void init_pwm();
void stop_motor_head();
void set_motor_head_up();
void set_motor_head_down();
void enable_motor_head();
void disable_motor_head();

int main()
{
    printf("Lathe head up down controller MBed OS version %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);

    analogin_t speed_control;
    analogin_init(&speed_control, PA_7);
    init_pwm();

    enable_motor_head();

    while (true)
    {
        move_grinder_head_task(speed_control);
        
        rotary_switch_task(s_bottom_rotary_encoder_state, 
                           s_rotary_line_bottom_low_1st_cw, 
                           s_rotary_line_bottom_low_2nd_cw, 
                           [](){ pulse_motor(false, PULSES_PER_HALF_TEN_THOUSANDTH, DigitalOut(PWM_OUT)); }, 
                           [](){ pulse_motor(true, PULSES_PER_HALF_TEN_THOUSANDTH, DigitalOut(PWM_OUT)); });

        rotary_switch_task(s_top_rotary_encoder_state, 
                           s_rotary_line_top_low_1st_cw, 
                           s_rotary_line_top_low_2nd_cw, 
                           [](){ pulse_motor(false, PULSES_PER_ONE_THOUSANDTH, DigitalOut(PWM_OUT)); }, 
                           [](){ pulse_motor(true, PULSES_PER_ONE_THOUSANDTH, DigitalOut(PWM_OUT)); });
    }
}

void init_pwm()
{
    s_pwm.write(0.50);
    s_pwm.suspend();
    set_pwm_high();
}

void set_pwm_high()
{
    gpio_t pin;
    gpio_init_out(&pin, PWM_OUT);
    gpio_write(&pin, 1);
}

void pulse_motor(bool direction_up, unsigned int number_of_pulses, DigitalOut&& pin)
{
    //printf("pulse motor\n");
    // Set direction to move 
    if (direction_up)
    {
        set_motor_head_up();
    }
    else 
    {
        set_motor_head_down();
    }

    for(uint32_t i=0; i<number_of_pulses; ++i)
    {
        pin.write(0);
        wait_us(500000/PULSES_PER_REVOLUTION);
        pin.write(1);
        wait_us(500000/PULSES_PER_REVOLUTION);
    }
}

void move_grinder_head_task(analogin_t& speed_control)
{
    // first check up and down buttons
    // If one is down set the direction pin, enable servo control and run below algorithm
    if (0 == s_up_button)
    {
        // Set direction pin to up
        set_motor_head_up();
        move_motor_head(speed_control);
        s_up_button_state = ButtonState_DOWN;
    }
    else if (0 == s_down_button)
    {
        // Set direction pin to down
        set_motor_head_down();
        move_motor_head(speed_control);
        s_down_button_state = ButtonState_DOWN;
    }
    else if (1 == s_up_button && ButtonState_DOWN == s_up_button_state)
    {
        stop_motor_head();
        s_up_button_state = ButtonState_UP;
    }
    else if (1 == s_down_button && ButtonState_DOWN == s_down_button_state)
    {
        stop_motor_head();
        s_down_button_state = ButtonState_UP;
    }
}

void set_motor_head_up()
{
    s_head_direction_pin = 1;
}

void set_motor_head_down()
{
    s_head_direction_pin = 0;
}

void enable_motor_head()
{
    s_head_enable_pin = 1;
}

void disable_motor_head()
{
    s_head_enable_pin = 0;
}

void move_motor_head(analogin_t& speed_control)
{
    auto voltage = analogin_read_u16(&speed_control);
    auto speed_percent = voltage / one_percent;

    if (speed_percent > 100)
    {
        speed_percent = 100;
    }
    
    int32_t large_voltage = voltage;
    auto diff = abs(large_voltage - s_last_voltage);
    
    if (speed_percent != s_last_speed_percent && diff > 400)
    {
        s_last_voltage = large_voltage;
        core_util_critical_section_enter();
        if (s_speed_ticker_running)
        {
            s_speed_ticker.detach();
        }
        core_util_critical_section_exit();

        s_desired_steps_per_second = (MAX_PULSES_PER_SECOND * speed_percent)/100;
        unsigned int timer_delay = 0;
        if (s_desired_steps_per_second < s_current_steps_per_second)
        {
            timer_delay = (s_current_steps_per_second - s_desired_steps_per_second) * INCREMENT_RATE_IN_MICROSECONDS / MAX_PULSES_PER_SECOND;
        }
        else 
        {
            timer_delay = (s_desired_steps_per_second - s_current_steps_per_second) * INCREMENT_RATE_IN_MICROSECONDS / MAX_PULSES_PER_SECOND;
        } 

        printf("voltage value=%u last-speed-percent=%u speed-percent=%u time-delay=%u current-steps-sec=%u desired-steps-sec/sec=%u\n", 
            voltage, 
            s_last_speed_percent, 
            speed_percent, 
            timer_delay,
            s_desired_steps_per_second,
            s_desired_steps_per_second); 
        
        s_last_speed_percent = speed_percent;
        s_speed_increment = -20;
        s_speed_ticker.attach(&update_pwm, std::chrono::microseconds(timer_delay));
    }    
}

void stop_motor_head()
{
    core_util_critical_section_enter();
    if (s_speed_ticker_running)
    {
        s_speed_ticker.detach();
    }
    core_util_critical_section_exit();
    s_speed_ticker_running = false;
    s_last_speed_percent = 0;
    s_last_voltage = 0;
    s_pwm.suspend();
    set_pwm_high();
}

uint32_t steps_per_second_speed_curve(uint32_t desired_speed, int time_increment)
{
    return desired_speed / (1.0f + exp(-0.2*time_increment));
}

void update_pwm()
{
    if (0 == s_desired_steps_per_second)
    {
        s_pwm.suspend();
        set_pwm_high();
        s_current_steps_per_second = s_desired_steps_per_second;
        s_speed_ticker_running = false;
        s_last_speed_percent = 0;
        s_last_voltage = 0;
        s_speed_ticker.detach();
    }
    else
    {
        s_current_steps_per_second = steps_per_second_speed_curve(s_desired_steps_per_second, s_speed_increment++);

        if (!s_speed_ticker_running)
        {
            s_pwm.resume();
            s_speed_ticker_running = true;
        }

        s_pwm.period_us(1000000.0/s_current_steps_per_second);
        
        if (s_speed_increment > 20)
        {
            s_speed_ticker_running = false;
            s_speed_ticker.detach();
        }
    }
}