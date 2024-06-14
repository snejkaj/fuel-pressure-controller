#include "pico/stdlib.h"
#include "hardware/pwm.h"
/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;

void compute();
void SetTunings(double Kp, double Ki, double Kd);
uint32_t pwm_set_freq_duty(uint slice_num,
       uint chan,uint32_t f, int d);

int main(){
    gpio_set_function(22, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(22);
    uint chan = pwm_gpio_to_channel(22);
    pwm_set_freq_duty(slice_num, chan, 50, 75);
    pwm_set_enabled(slice_num, true);
    return 0;
}

uint32_t pwm_set_freq_duty(uint slice_num,
       uint chan,uint32_t f, int d)
{
 uint32_t clock = 125000000;
 uint32_t divider16 = clock / f / 4096 + 
                           (clock % (f * 4096) != 0);
 if (divider16 / 16 == 0)
 divider16 = 16;
 uint32_t wrap = clock * 16 / divider16 / f - 1;
 pwm_set_clkdiv_int_frac(slice_num, divider16/16,
                                     divider16 & 0xF);
 pwm_set_wrap(slice_num, wrap);
 pwm_set_chan_level(slice_num, chan, wrap * d / 100);
 return wrap;
}


void Compute()
{
/*How long since we last calculated*/
unsigned long now = millis();
double timeChange = (double)(now - lastTime);
 
/*Compute all the working error variables*/
double error = Setpoint - Input;
errSum += (error * timeChange);
double dErr = (error - lastErr) / timeChange;
 
/*Compute PID Output*/
Output = kp * error + ki * errSum + kd * dErr;
 
/*Remember some variables for next time*/
lastErr = error;
lastTime = now;
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
kp = Kp;
ki = Ki;
kd = Kd;
}
