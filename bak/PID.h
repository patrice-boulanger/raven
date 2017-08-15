#ifndef _RAVEN_PID_H
#define _RAVEN_PID_H

#include "Arduino.h"

class PID
{
public:
        PID();
        ~PID();

        // Returns Kp
        float get_kp();
        // Returns Ki
        float get_ki();
        // Returns Kd
        float get_kd();

        // Set Kp/Ki/Kd
        void set_kpid(float kp, float ki, float kd);

        // Set bounds 
        void set_minmax(int minv, int maxv);

        // Compute the pulse correction based on input (real) & target (expected) values
        int compute(float real, float expected);
        
protected:
        // PID coefficients
        float Kp, Kd, Ki;

        // Bounds
        int minval, maxval;

        // Sum of previous error
        float Serror;

        // Last error 
        float lasterr;
        
        // Last time
        unsigned long lasttime;
};

#endif // _RAVEN_PID_H
