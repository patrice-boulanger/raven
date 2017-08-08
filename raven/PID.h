#ifndef _RAVEN_PID_H
#define _RAVEN_PID_H

#include "Arduino.h"

class PID
{
public:
        PID();
        ~PID();

        // Returns Kp
        float getKp();
        // Returns Ki
        float getKi();
        // Returns Kd
        float getKd();

        // Set Kp/Ki/Kd
        void setKpid(float kp, float ki, float kd);

        // Set bounds 
        void setBounds(float minv, float maxv);

        // Compute the correction factor based on input (real) & target (expected) values
        float compute(float real, float expected);
        
protected:
        // PID coefficients
        float Kp, Kd, Ki;

        // Bounds
        float minval, maxval;

        // Last error 
        float lasterr;
        
        // Last time
        unsigned long lasttime;
};

#endif // _RAVEN_PID_H
