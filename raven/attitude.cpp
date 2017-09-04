#include "raven.h"
#include "state.h"

void get_raw_value(state_t *state)
{
	// Read raw values & add them to the corresponding buffer
	state->mpu.getAcceleration(&state->sma.ax[state->sma.pos], &state->sma.ay[state->sma.pos], &state->sma.az[state->sma.pos]);
	state->mpu.getRotation(&state->sma.gx[state->sma.pos], &state->sma.gy[state->sma.pos], &state->sma.gz[state->sma.pos]);

	while(!state->compass.getReadyStatus());
	state->compass.getHeading(&state->sma.mx[state->sma.pos], &state->sma.my[state->sma.pos], &state->sma.mz[state->sma.pos]);

	// Move to the next slot in the buffer	
	state->sma.pos = (state->sma.pos + 1) & (SMA_BUFSZ_BITS - 1);
}

void update_attitude(state_t *state, unsigned long micro)
{
	// Delta time in seconds
	float dt = micro / 1000000.0;

	// Averaged values from SMA buffer
	int32_t ax = 0, ay = 0, az = 0;
	int32_t gx = 0, gy = 0, gz = 0;
	int32_t mx = 0, my = 0, mz = 0;

	// Check for compass overflow
	if (state->sma.mx[state->sma.pos] == -4096 || state->sma.my[state->sma.pos] == -4096 || state->sma.mz[state->sma.pos] == -4096) {
		state->sma.mx[state->sma.pos] = 0;
		state->sma.my[state->sma.pos] = 0;
		state->sma.mz[state->sma.pos] = 0;
	} 	

	// Compute SMA from MPU & compass raw values
	for(int i = 0; i < SMA_BUFSZ; i ++) {

                // Datasheet: divided by 16384 to have the value in g
		ax += state->sma.ax[i];
		ay += state->sma.ay[i];
		az += state->sma.az[i];
		
		gx += state->sma.gx[i];
		gy += state->sma.gy[i];
		gz += state->sma.gz[i];
		
		mx += state->sma.mx[i];
		my += state->sma.my[i];
		mz += state->sma.mz[i]; 
	}

	ax = (ax >> SMA_BUFSZ_BITS);
	ay = (ay >> SMA_BUFSZ_BITS);
	az = (az >> SMA_BUFSZ_BITS);

	gx = (gx >> SMA_BUFSZ_BITS);
	gy = (gy >> SMA_BUFSZ_BITS);
	gz = (gz >> SMA_BUFSZ_BITS);

	mx = (mx >> SMA_BUFSZ_BITS);
	my = (my >> SMA_BUFSZ_BITS);
	mz = (mz >> SMA_BUFSZ_BITS);
	
	// Angular speeds from gyro (deg.s-1)
	state->attitude.roll_rate = gx / 131.0;
        state->attitude.pitch_rate = gy / 131.0;
        state->attitude.yaw_rate = gz / 131.0;

        // Orientation of the accelerometer relative to the earth (deg.)
        float x_angle = RAD2DEG(atan2(ay, az));
        float y_angle = RAD2DEG(atan2(-ax, az));

        
        // Gyroscope provides very accurate measurements but it tends to drift. The accelerometer is unstable, but does not drift.
        // Compute pitch and roll angles (deg) corrected w/ complementary filter.
        //state->attitude.roll = state->config.alpha * (state->attitude.roll + state->attitude.roll_rate * dt) + (1 - state->config.alpha) * x_angle; 
        //state->attitude.pitch = state->config.alpha * (state->attitude.pitch + state->attitude.pitch_rate * dt) + (1 - state->config.alpha) * y_angle; 
        state->attitude.roll = x_angle + state->config.alpha * ((state->attitude.roll + state->attitude.roll_rate * dt) - x_angle); 
        state->attitude.pitch = y_angle + state->config.alpha * ((state->attitude.pitch + state->attitude.pitch_rate * dt) - y_angle); 

	// YMFC: yaw transfer ????
	//0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      	//angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the roll angle to the pitch angel.
      	//angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the pitch angle to the roll angel.

	// Backup previous heading
	double p_head = state->attitude.heading;

	float cosRoll = cos(DEG2RAD(state->attitude.roll));
  	float sinRoll = sin(DEG2RAD(state->attitude.roll));  
  	float cosPitch = cos(DEG2RAD(state->attitude.pitch));
  	float sinPitch = sin(DEG2RAD(state->attitude.pitch));
  
  	// Tilt compensation for heading (rads)
  	// mx becomes -my & my becomes mx to match MPU disposal on the board

  	//float Xh = mx * cosPitch + mz * sinPitch;
  	//float Yh = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;
  	float Xh = -my * cosPitch + mz * sinPitch;
  	float Yh = -my * sinRoll * sinPitch + mx * cosRoll - mz * sinRoll * cosPitch;

  	float hrads = atan2(Yh, Xh) + DEG2RAD(state->config.off_decl);

	// Correct angle if necessary
	if (hrads < 0) 
		hrads += 2 * M_PI;
        else if (hrads > 2 * M_PI) 
  		hrads -= 2 * M_PI;  

	state->attitude.heading = RAD2DEG(hrads);

  	// Angular speed (deg.s-1)
  	state->attitude.heading_rate = (state->attitude.heading - p_head) / dt;
}


/* ----- Get drone attitude ----- */
/*void get_attitude(unsigned long ms)
{
	// Delta time in seconds
	float dt = ms / 1000.0;

        // Raw values from MPU
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	mpu.getAcceleration(&ax, &ay, &az);
	mpu.getRotation(&gx, &gy, &gz);

	// Accelerations (m.s-2)
	state->attitude.ax = iir_lpf(state->attitude.ax, ax);
	state->attitude.ay = iir_lpf(state->attitude.ay, ay);
	state->attitude.az = iir_lpf(state->attitude.az, az);
	
	state->attitude.x_acc = (9.81 * state->attitude.ax) / 16384.0;
	state->attitude.y_acc = (9.81 * state->attitude.ay) / 16384.0;
	state->attitude.z_acc = (9.81 * state->attitude.az) / 16384.0;
	
	// Angular speeds (deg.s-1)
	// The sensor returns deg.s-1
	state->attitude.gx = iir_lpf(state->attitude.gx, gx);
	state->attitude.gy = iir_lpf(state->attitude.gy, gy);
	state->attitude.gz = iir_lpf(state->attitude.gz, gz);	
	
        state->attitude.roll_rate = state->attitude.gx / 131.0; 
        state->attitude.pitch_rate = state->attitude.gy / 131.0;
        state->attitude.yaw_rate = state->attitude.gz / 131.0;

        // Orientation of the accelerometer relative to the earth (convert to deg.)
        float x_angle = RAD2DEG(atan2(state->attitude.ay, state->attitude.az));
        float y_angle = RAD2DEG(atan2(-state->attitude.ax, state->attitude.az));

        // Pitch and roll angles (deg) corrected w/ complementary filter
        double nr = ALPHA * (state->attitude.roll + state->attitude.roll_rate * dt) + (1 - ALPHA) * x_angle; 
        double np = ALPHA * (state->attitude.pitch + state->attitude.pitch_rate * dt) + (1 - ALPHA) * y_angle; 

	state->attitude.roll = ROUND1(iir_lpf(state->attitude.roll, nr));	
	state->attitude.pitch = ROUND1(iir_lpf(state->attitude.pitch, np));
	
	// Backup previous heading
	state->attitude.heading_p = state->attitude.heading;

	int16_t mx, my, mz, temp;

	while(!compass.getReadyStatus());
	compass.getHeading(&mx, &my, &mz);

	if (mx == -4096 || mx == -4096 || mz == -4096) {
		Serial.println("Compass overflow");
	} else {	
		// Swap X/Y to match MPU disposal on the board
		temp = mx;
		mx = -my;
		my = temp;

		state->attitude.mx = iir_lpf(state->attitude.mx, mx);
		state->attitude.my = iir_lpf(state->attitude.my, my);
		state->attitude.mz = iir_lpf(state->attitude.mz, mz);
		
		float cosRoll = cos(state->attitude.roll);
	  	float sinRoll = sin(state->attitude.roll);  
	  	float cosPitch = cos(state->attitude.pitch);
	  	float sinPitch = sin(state->attitude.pitch);
	  
	  	// Tilt compensation (rads)
	  	float Xh = state->attitude.mx * cosPitch + state->attitude.mz * sinPitch;
	  	float Yh = state->attitude.mx * sinRoll * sinPitch + state->attitude.my * cosRoll - state->attitude.mz * sinRoll * cosPitch;
	  	float tilt = atan2(Yh, Xh) + off_decl;
	
		// Correct angle if necessary
		if (state->attitude.heading < 0) 
			tilt += 2 * PI;
                      
	  	if (state->attitude.heading > 2 * PI) 
	  		tilt -= 2 * PI;  

		state->attitude.heading = ROUND1(RAD2DEG(tilt));
	
	  	// Angular speed (deg.s-1)
	  	state->attitude.heading_rate = (state->attitude.heading - state->attitude.heading_p) / dt;
	}

  	// Request temperature
    	barometer.setControl(BMP085_MODE_TEMPERATURE);
    
    	// Wait appropriate time for conversion (4.5ms delay)
    	unsigned long lastMicros = micros();
    	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    	// Read calibrated temperature value (deg. Celsius)
    	state->attitude.temperature = barometer.getTemperatureC();

	// Request pressure (3x oversampling mode, high detail, 23.5ms delay)
    	barometer.setControl(BMP085_MODE_PRESSURE_0);
    	while (micros() - lastMicros < barometer.getMeasureDelayMicroseconds());

    	// Read calibrated pressure value in Pascals (Pa)
    	state->attitude.pressure = barometer.getPressure();

    	// Calculate absolute altitude (m) based on known pressure
    	state->attitude.altitude_p = state->attitude.altitude;  
	
	state->attitude.alt_samples[state->attitude.alt_idx] = barometer.getAltitude(state->attitude.pressure);
	state->attitude.alt_idx = (state->attitude.alt_idx + 1) % ALT_SAMPLES;

	state->attitude.altitude = 0;
	for(int i = 0; i < ALT_SAMPLES; i ++) 
		state->attitude.altitude += state->attitude.alt_samples[i];

	state->attitude.altitude /= ALT_SAMPLES;
	
	// Vertical speed (m.s-1)
    	state->attitude.vspeed = (state->attitude.altitude - state->attitude.altitude_p) / dt;
}*/	
