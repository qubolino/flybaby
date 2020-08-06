#ifndef CONFIG_H_
#define CONFIG_H_

#define MPU9250_2G_SENSITIVITY 		16384.0f 	// lsb per g
#define MPU9250_500DPS_SENSITIVITY	65.5f 		// lsb per deg/sec
#define MPU6050_2G_SENSITIVITY 		16384.0f 	// lsb per g
#define MPU6050_500DPS_SENSITIVITY	65.5f 		// lsb per deg/sec

// Kalman filter configuration

#define KF_ZACCEL_VARIANCE		1200.0f
#define KF_ZPRESS_VARIANCE		1700.0f
#define KF_ACCELBIAS_VARIANCE   10.0f

// Power-down timeout. Here we power down if the
// vario does not see any climb or sink rate more than
// 50cm/sec, for 20 minutes.
#define SLEEP_TIMEOUT_SECONDS   1200 // 20 minutes
#define SLEEP_THRESHOLD_CPS		50

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet. netvario is the integration time is secs. max 15secs default 4secs
#define  CLIMB_THRESHOLD 30
#define	 ZERO_THRESHOLD -20
#define  SINK_THRESHOLD -250
#define  NET_VARIO_MAX 15

// change these parameters based on the frequency bandwidth of the speaker

#define VARIO_MAX_FREQHZ      4000
#define VARIO_XOVER_FREQHZ    2000
#define VARIO_MIN_FREQHZ      200

#define VARIO_SINK_FREQHZ     400
#define VARIO_TICK_FREQHZ     200

// audio feedback tones
#define BATTERY_TONE_FREQHZ			400
#define CALIB_TONE_FREQHZ			800
#define MPU9250_ERROR_TONE_FREQHZ	200
#define MS56XX_ERROR_TONE_FREQHZ	2500

// print useful information to the serial port for 
// verifying correct operation. Comment out to prevent
// data being spewed out continuously.
// #define IMU_DEBUG


#endif