#include "PID.h"

PID::PID() 
{
        Serror = 0;
        lasterr = 0;
        lasttime = millis();        
}

PID::~PID() {}

float PID::getKp()
{
        return Kp;
}

float PID::getKi()
{      
        return Ki;
}

float PID::getKd()
{
        return Kd;
}

void PID::setKpid(float kp, float ki, float kd)
{
        if (kp < 0 || ki < 0 || kd < 0)
                return;
        
        Kp = kp;
        Ki = ki;
        Kd = kd;
}

void PID::setBounds(float minv, float maxv)
{
        minval = minv;
        maxval = maxv;
}

float PID::compute(float real, float expected)
{
        unsigned long now = millis(), ms = now - lasttime;
        // dt in seconds
        float dt = (float)(ms) / 1000.0;

        // Error
        float error = expected - real; 
        // Integral of the error
        float Sedt = Serror + (error * dt); 
        // Derivative
        float dedt = (error - lasterr) / dt;

        float output = Kp * error + Ki * Sedt + Kd * dedt;
        
        if (output > maxval)
                output = maxval;
        if (output < minval)
                output = minval;

        lasterr = error;
        lasttime = now;

        return output;
}

