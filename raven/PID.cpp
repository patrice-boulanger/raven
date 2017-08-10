#include "PID.h"

PID::PID() 
{
        Serror = 0;
        lasterr = 0;
        lasttime = millis();
        minval = maxval = 0;        
}

PID::~PID() {}

float PID::get_kp()
{
        return Kp;
}

float PID::get_ki()
{      
        return Ki;
}

float PID::get_kd()
{
        return Kd;
}

void PID::set_kpid(float kp, float ki, float kd)
{
        if (kp < 0 || ki < 0 || kd < 0)
                return;
        
        Kp = kp;
        Ki = ki;
        Kd = kd;
}

void PID::set_minmax(int minv, int maxv)
{
        minval = minv;
        maxval = maxv;
}

int PID::compute(float real, float expected)
{
        unsigned long now = millis(), ms = now - lasttime;
        // dt in seconds
        float dt = (float)(ms) / 1000.0;

        // Error
        float error = expected - real; 
        // Sum of the error (integral)
        float Sedt = Serror + (error * dt); 
        // Derivative
        float dedt = (error - lasterr) / dt;

        int output = (int)(Kp * error + Ki * Sedt + Kd * dedt);
        
        if (output > maxval)
                output = maxval;
        if (output < minval)
                output = minval;

        lasterr = error;
        lasttime = now;

        return output;
}

