#ifndef _RAVEN_UTIL_H
#define _RAVEN_UTIL_H

// Some useful definitions

#define DEG2RAD 0.017453278
#define RAD2DEG 57.2957786

// avoid floating point comparison to 0.0f
#define EPSILON 0.00001
#define is_close_to_zero(x) (abs(x) < EPSILON)

#endif // _RAVEN_UTIL_H
