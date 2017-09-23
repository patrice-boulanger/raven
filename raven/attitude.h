#ifndef _RAVEN_ATTITUDE_H
#define _RAVEN_ATTITUDE_H

// Buffer size for Simple Moving Average of MPU & compass raw values
#define SMA_BUFSZ_BITS 		2
#define SMA_BUFSZ 		(1<<SMA_BUFSZ_BITS)

/*
 * Buffer to store intermediate values for simple moving average
 */
typedef struct {
	// Current position in the buffers
	uint16_t pos;
	// Buffers for raw values to average
	int16_t ax[SMA_BUFSZ], ay[SMA_BUFSZ], az[SMA_BUFSZ]; 
	int16_t gx[SMA_BUFSZ], gy[SMA_BUFSZ], gz[SMA_BUFSZ];
	int16_t mx[SMA_BUFSZ], my[SMA_BUFSZ], mz[SMA_BUFSZ];
} sma_buffer_t;

/* 
 * Drone attitude based on MPU & compass 
 */
typedef struct {
	double pitch, roll;			// pitch/roll angles (deg)
	double roll_rate, pitch_rate, yaw_rate;	// angular speeds on axis (deg.s-1)

	double heading;				// heading (deg)
	double heading_rate;			// heading angular speed (deg.s-1)
} attitude_t;

#endif // _RAVEN_ATTITUDE_H
