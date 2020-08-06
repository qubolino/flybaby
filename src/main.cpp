
//
//Iain Frew - 8/30/17
//
//IDE is Visual Studio using Vmicro add-in for Arduino development
//
//This is a port of Hari Nair project to esp32 and using the DMP functionaility of
//the 9250 IMU.
//
//The majortity of this code was some great work by Jeff Rowberg for the
//mpu6050 library and the i2cdevlib interface, and Hari Nair for the Vario design. 
//The calibration came from  Luis R�denas.

//I integrated Jeff's MPU6050 library to take advantage of the 6050 dmp processing which works fine
//on esp8266 but a bit tempremental on esp32 as there are some issues with timing using Jeff's library 
//and the I2C bus. You need to be able to read in chunks using a buffer size of about 16.
//So for my chips and setup I played around with the buffer_length set in i2cdevlib.cpp and eventually chose 16.
//Anything larger tends to screw up the data being read. Also found that the readings are noisy when
//reading from the DMP at 200Hz as jeff found. So reduced ODR to 100Hz and modified the code accordingly to take
//that into account.

//I have a define to enable/disable DMP processing rather than calculate manually the rotations that Hari used.
//Seems to be accurate and reduces the loop elapsed time by a factor of 3. 

//Correct calibration I have found is key but on two gy-86, values of 1700 and 1200 for kfzVariance an kfazVariance
//seem to work best. Configuration in the config.h file of zero theshold 20 and climb threshold 20
//also produce good results.
//
//To build, you only need the esp32 core library installed as this sketch uses the preference, wire and wifi libraries
//
//
//This code uses the gy-86 IMU and the reference system is different than the CJMCU-117 that Hari used. If you use the 
//cjmcu-117 module then just turn the board upside down and you get the same reults as the gy-86 which assumes the chips are up facing you
//when used.

// gy-86 interface
// 3.3V  3.3v
// GND  GND
// SCL  GPIO21
// SDA  GPIO22
// INT  GPIO12
// audio GPIO13
// calibration button GPIO15

#include <Arduino.h>

//uncomment to use the dmp processor rather than the manual calculations of quaternions
#define dmp
// #define IMU_DEBUG

#include <Preferences.h>
#include <esp32-hal-ledc.h>
#include <Wire.h>


#include "VarioAudio.h"
#include "util.h"
#include "pztbl.h"
#include "MS56XX.h"
#include "KalmanVario.h"
#include "config.h"
#include "cct.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU9250.h";
#include "MahonyAHRS.h";

#include <HardwareSerial.h>

HardwareSerial SerialGPS(2);

// for power management
#include "WiFi.h" 
#include <esp_wifi.h>
#include <esp_bt.h>


extern "C" {

}

uint32_t timePreviousUs;

uint32_t timeNowUs;
float imuTimeDeltaSecs;
float baroTimeDeltaSecs;

float accel[3];
float gyro[3];
float kfAltitudeCm = 0.0f;
float kfClimbrateCps = 0.0f;
float avgeCps = 0.0f;
float zAccelAccumulator = 0.0f;
float kfzVariance = 1700;//This gets calulated during in baro.test calibration but my mpu6050 it as always around 1700 so setting as default;
float kazVariance = 1200;//This gets measured during calibration of accelrometer but my mpu6050 its is always around 1200;

//keep track of average integrated vario cps over period of time up to 15s max.
// 20ms( 10ms temp + 10ms pressure) sample rate so 50 sample pairs  per second;
int variosamplecnt = 0;
float oldvarioavge = 0.0;
int net_vario = 25; //default to 1/2s integrated vario;

float refAltitudeAtInitCm = 0.0f;

int32_t audioCps;
int audiochannel = 0;


int pinSDA = 4;
int pinSCL = 5;
int pinDRDYInt = 15;
int pinAudio = 17;
int pinCalibBtn = 12;


volatile int drdyCounter;
volatile int drdyFlag;
int baroCounter;
int timeoutSeconds;

const int buffersize = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;			 //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;           //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

Preferences preferences;
MPU6050 mpu;  //used for mpu6050 dmp functionality
MPU9250 imu; //6050 compatible. used for non dmp functionality. original code
MS56XX baro;
KalmanVario kf;
VarioAudio audio;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount = 0;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

						// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

						// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


int maximum(int a, int b) {
	return (a > b ? a : b);
}

static uint8_t btmsg_nmeaChecksum(const char *szNMEA){
    const char* sz = &szNMEA[1]; // skip leading '$'
    uint8_t cksum = 0;

    while ((*sz) != 0 && (*sz != '*')) {
        cksum ^= (uint8_t) *sz;
        sz++;
        }
    return cksum;
}

int batteryVoltage(void) {
	int adcSample = 0;
	for (int inx = 0; inx < 4; inx++) {
		adcSample += analogRead(A0);
		delay(1);
	}
	adcSample /= 4;
	// voltage divider with 120K and 33K to scale 4.2V down to < 1.0V for the ESP8266 ADC
	// actual measurement 0.854V with battery voltage = 4.0V => actual scale up from resistive divider = 4.0/0.854 = 4.6838
	// adc isn't very accurate either, experimental correction factor ~ 1.04, so effective scale up is 1.04 * 4.6838
	return (int)((adcSample*1.04f*4.6838f*10.0f) / 1023.0f + 0.5f); //  voltage x 10
}

void audio_indicateBatteryVoltage(int bv) {
	int numBeeps;
	if (bv >= 40) numBeeps = 5;
	else
		if (bv >= 39) numBeeps = 4;
		else
			if (bv >= 37) numBeeps = 3;
			else
				if (bv >= 36) numBeeps = 2;
				else  numBeeps = 1;
				while (numBeeps--) {
					audio.GenerateTone(BATTERY_TONE_FREQHZ, 300);
					delay(300);
				}
}

void DRDYInterruptHandler() {
	drdyFlag = 1; // indicate new MPU6050 data is available
	drdyCounter++;
}

void initTime() {
	timeNowUs = timePreviousUs = micros();
}

void updateTime() {
	timeNowUs = micros();
	imuTimeDeltaSecs = ((timeNowUs - timePreviousUs) / 1000000.0f);
	timePreviousUs = timeNowUs;
}

void powerDown() {
	// residual current draw after power down is the sum of ESP8266 deep sleep mode current,
	// MPU9250 sleep mode current, MS56XX standby current, quiescent current of voltage
	// regulators, and miscellaneous current through resistive paths e.g. the
	// ADC voltage divider.
	audio.SetFrequency(0); // switch off pwm audio 
	mpu.setSleepEnabled(true); // put MPU9250 in sleep mode
	ESP.deepSleep(0); // ESP8266 in sleep can only recover with a reset/power cycle
}


// if imu calibration data in flash is corrupted, the accel and gyro biases are 
// set to 0, and this uncalibrated state is indicated with a sequence of alternating 
// low and high beeps.
void indicateUncalibratedAccelerometer() {
	for (int cnt = 0; cnt < 5; cnt++) {
		audio.GenerateTone(200, 500);
		audio.GenerateTone(2000, 500);
	}
}




// "no-activity" power down is indicated with a series of descending
// tones. If you hear this, switch off the vario as there is still
// residual current draw from the circuit components	
void indicatePowerDown() {
	audio.GenerateTone(2000, 1000);
	audio.GenerateTone(1000, 1000);
	audio.GenerateTone(500, 1000);
	audio.GenerateTone(250, 1000);
}

// problem with MS56XX calibration CRC, assume communication 
// error or bad device. Indicate with series of 10 high pitched beeps.
void indicateFaultMS56XX() {
	for (int cnt = 0; cnt < 10; cnt++) {
		audio.GenerateTone(MS56XX_ERROR_TONE_FREQHZ, 1000);
		delay(100);
	}
}

// problem reading MPU9250 ID, assume communication 
// error or bad device. Indicate with series of 10 low pitched beeps.
void indicateFaultMPU9250() {
	for (int cnt = 0; cnt < 10; cnt++) {
		audio.GenerateTone(MPU9250_ERROR_TONE_FREQHZ, 1000);
		delay(100);
	}
}


void meansensors() {
	long i, buff_ax, buff_ay, buff_az, buff_gx, buff_gy, buff_gz;
	int16_t az_values[buffersize];

	kazVariance = 0.0;
	buff_ax = 0; buff_ay = 0; buff_az = 0; buff_gx = 0; buff_gy = 0; buff_gz = 0;
	i = 0;

	while (i < (buffersize)) {
		// read raw accel/gyro measurements from device
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//Serial.printf("az raw value %d\r\n", az);
		buff_ax += ax;
		buff_ay += ay;
		buff_az += az;
		az_values[i] = az;
		buff_gx += gx;
		buff_gy += gy;
		buff_gz += gz;

		i++;
		delay(2); //Needed so we don't get repeated measures
	}
	mean_ax = buff_ax / buffersize;
	mean_ay = buff_ay / buffersize;
	mean_az = buff_az / buffersize;
	mean_gx = buff_gx / buffersize;
	mean_gy = buff_gy / buffersize;
	mean_gz = buff_gz / buffersize;

	for (int n = 0; n < buffersize; n++) {
		kazVariance += (az_values[n] - mean_az)*(az_values[n] - mean_az);
		//Serial.printf("%d %d\r\n",(int)pa[n],(int)z[n]);
	}
	kazVariance = kazVariance / (buffersize - 1);
	Serial.print("kazVariance = "); Serial.println(kazVariance);
	Serial.println(mean_ax);
	Serial.println(mean_ay);
	Serial.println(mean_az);
	Serial.println(mean_gx);
	Serial.println(mean_gx);
	Serial.println(mean_gx);
}

bool calibration() {

	int numtries = 0;
	mpu.setXAccelOffset(0);
	mpu.setYAccelOffset(0);
	mpu.setZAccelOffset(0);
	mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);

	//First 100 measures are discarded
	for (int j = 0; j < 100; j++) {
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		delay(2);
	}

	//The mpu6050 yaw figues drift and takes a little time to stablize ~15 secs with reading settling after ~30s

	meansensors();
	delay(1000);

	ax_offset = -mean_ax / 8;
	ay_offset = -mean_ay / 8;
	az_offset = (16384 - mean_az) / 8;

	gx_offset = -mean_gx / 4;
	gy_offset = -mean_gy / 4;
	gz_offset = -mean_gz / 4;

	while (1) {
		int ready = 0;
		mpu.setXAccelOffset(ax_offset);
		mpu.setYAccelOffset(ay_offset);
		mpu.setZAccelOffset(az_offset);

		mpu.setXGyroOffset(gx_offset);
		mpu.setYGyroOffset(gy_offset);
		mpu.setZGyroOffset(gz_offset);

		meansensors();
		Serial.printf("...\r\n");

		if (abs(mean_ax) <= acel_deadzone) ready++;
		else ax_offset = ax_offset - mean_ax / acel_deadzone;

		if (abs(mean_ay) <= acel_deadzone) ready++;
		else ay_offset = ay_offset - mean_ay / acel_deadzone;

		if (abs(16384 - mean_az) <= acel_deadzone) ready++;
		else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

		if (abs(mean_gx) <= giro_deadzone) ready++;
		else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

		if (abs(mean_gy) <= giro_deadzone) ready++;
		else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

		if (abs(mean_gz) <= giro_deadzone) ready++;
		else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

		numtries++;

		if (ready == 6) {
			return true;
		}

		if (numtries > 10) return false;
	}

}

float runningAverageofCps(float cps)
{
	//Integrated vario calculation. Average the climbrate over last NET_VARIO secs
	//We sample at 20ms(10ms temp, 10ms pressure), so 200 samples is 4secs.
	
	double avgecps;

	if (net_vario > NET_VARIO_MAX) {
		net_vario = NET_VARIO_MAX;
	}
	variosamplecnt = (variosamplecnt + 1);
	if (variosamplecnt > (net_vario * 50)/50) variosamplecnt = (net_vario * 50/50);
	
	avgecps = oldvarioavge*(float(variosamplecnt - 1) / variosamplecnt) + cps / variosamplecnt;	
	//Serial.printf("cps %d average cps %d old average %d vario samplecnt %d\r\n", int(cps),int(avgecps),int(oldvarioavge), variosamplecnt);
	oldvarioavge = avgecps;

	
	return float(avgecps);

}




void setup() {

	
	delay(10);

  	SerialGPS.begin(9600, SERIAL_8N1, 13, 12);

	delay(100); // delay(1) is required, additional delay so battery voltage can settle 
	Serial.begin(115200);
	Serial.printf("\r\nESP32 MPU6050 MS56XX VARIO w/ GPS compiled on %s at %s\r\n", __DATE__, __TIME__);

	Serial.println("Performing power management ops");
	Serial.println("** Stopping WiFi+BT");
	WiFi.mode(WIFI_OFF);
	btStop();
	// WiFi.disconnect(true);
	// WiFi.mode(WIFI_OFF);
	// esp_wifi_stop();
	// btStop();
	esp_bt_controller_disable();
	Serial.print("** Setting CPU freq ");
	Serial.print(getCpuFrequencyMhz());
	setCpuFrequencyMhz(160);
	Serial.print("Mhz --> ");
	Serial.print(getCpuFrequencyMhz());
	Serial.println("Mhz");


	Wire.begin(pinSDA, pinSCL);
	Wire.setClock(100000); // set clock frequency AFTER Wire.begin()

	ledcAttachPin(pinAudio, audiochannel);
	ledcSetup(audiochannel, 4000, 14);

	audio.Config(pinAudio, audiochannel);

	int bv = batteryVoltage();
	Serial.printf("battery voltage = %d.%dV\r\n", bv / 10, bv % 10);
	audio_indicateBatteryVoltage(bv);
	delay(1000);

	if (!baro.ReadPROM()) {
		Serial.printf("Bad CRC read from MS56XX calibration PROM\r\n");
		Serial.flush();
		indicateFaultMS56XX(); // 10 high pitched beeps
		powerDown();   // try power-cycling to fix this
	}

	if (!mpu.testConnection()) {
		Serial.printf("Error reading mpu6050 WHO_AM_I register\r\n");
		Serial.flush();
		indicateFaultMPU9250(); // 10 low pitched beeps
		powerDown();   // try power-cycling to fix this
	}

	//read stored calibration parameters

	preferences.begin("fastvario", false);
	ax_offset = preferences.getInt("axBias", 0);
	ay_offset = preferences.getInt("ayBias", 0);
	az_offset = preferences.getInt("azBias", 0);
	gx_offset = preferences.getInt("gxBias", 0);
	gy_offset = preferences.getInt("gyBias", 0);
	gz_offset = preferences.getInt("gzBias", 0);
	kfzVariance = preferences.getInt("kfzVar", KF_ZPRESS_VARIANCE);
	kazVariance = preferences.getInt("kfazVar", KF_ZACCEL_VARIANCE);

#ifndef dmp
	imu.axBias_ = ax_offset;
	imu.ayBias_ = ay_offset;
	imu.azBias_ = az_offset;
	imu.gxBias_ = gx_offset;
	imu.gyBias_ = gy_offset;
	imu.gzBias_ = gz_offset;
#endif

	if ((ax_offset == 0) && (ay_offset == 0) && (az_offset == 0)) {
		indicateUncalibratedAccelerometer(); // series of alternating low/high tones
	}

	Serial.printf("Using saved calibration parameters :\r\n");
	Serial.printf("Accel : axBias %d, ayBias %d, azBias %d\r\n", ax_offset, ay_offset, az_offset);
	Serial.printf("Gyro : gxBias %d, gyBias %d, gzBias %d\r\n", gx_offset, gy_offset, gz_offset);
	Serial.printf("KfzVariance : %f, KfazVariance : %f \r\n", kfzVariance, kazVariance);


	drdyCounter = 0;
	drdyFlag = 0;
	// INT output of MPU6050 is configured as push-pull, active high pulse. 
	
	pinMode(pinDRDYInt, INPUT);
	attachInterrupt(digitalPinToInterrupt(pinDRDYInt), DRDYInterruptHandler, RISING);

	// configure MPU6050 to start generating gyro and accel data at 200Hz ODR	
	
#ifdef dmp
	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
#else
	imu.ConfigAccelGyro();
	devStatus = 0;
#endif



	// Try to calibrate gyro each time on power up. if the unit is not at rest, give up
	// and use the last saved gyro biases.
	// Allow a 30 seconds for unit to be left undisturbed so gyro can be calibrated.
	// This delay is indicated with a series of 10 short beeps. During this time if you press and hold the
	// calibration button (GPIO0), the unit will calibrate both accelerometer and gyro.
	// As soon as you hear the long confirmation tone, release the calibration
	// button and put the unit in accelerometer calibration position resting undisturbed on a horizontal surface 
	// You will have some time 
	// to do this, indicated by a series of beeps. After calibration, the unit will generate another 
	// tone, save the calibration parameters to flash, and continue with normal vario operation

	
	pinMode(pinCalibBtn, INPUT_PULLUP); digitalWrite(pinCalibBtn, HIGH);
	int bCalibrateAccelerometer = 0;
	// short beeps for ~5 seconds
	for (int inx = 0; inx < 10; inx++) {
		delay(500);
		audio.GenerateTone(CALIB_TONE_FREQHZ, 50);
		if (digitalRead(pinCalibBtn) == 0) {
			delay(100); // debounce the button
			if (digitalRead(pinCalibBtn) == 0) {
				bCalibrateAccelerometer = 1;
				break;
			}
		}
	}
	//*****
	//*****
	//uncomment next line to force calibration each power up until i get a button to hardwire calibration selection during powerup
	//******
	//******
	//bCalibrateAccelerometer = 1;
	if (bCalibrateAccelerometer) {
		// acknowledge calibration button press with long tone
		audio.GenerateTone(CALIB_TONE_FREQHZ, 3000);
		// allow 5 seconds for the unit to be placed in calibration position with the 
		// accelerometer +z pointing downwards. Indicate this delay with a series of short beeps
		Serial.printf("Place Vario on flat surface and do not move until calibration complete..you have 5 seconds before calibration\r\n");
		for (int inx = 0; inx < 20; inx++) {
			delay(200);
			Serial.printf("...\r\n");
			audio.GenerateTone(CALIB_TONE_FREQHZ, 50);
		}
		Serial.printf("Calibrating accel & gyro\r\n");
		calibration();
		Serial.printf("End Calibration accel & gyro\r\n");

		Serial.printf("Saving Calibration data to non volatile store\r\n");
		preferences.putInt("axBias", ax_offset);
		preferences.putInt("ayBias", ay_offset);
		preferences.putInt("azBias", az_offset);
		preferences.putInt("gxBias", gx_offset);
		preferences.putInt("gyBias", gy_offset);
		preferences.putInt("gzBias", gz_offset);
		Serial.printf("Calibrated data saved\r\n");


	}
	// normal operation flow, attempt to calibrate gyro. If calibration isn't possible because the unit is disturbed,
	// use the last saved gyro biases

	


#ifdef dmp
	mpu.setXGyroOffset(gx_offset);
	mpu.setYGyroOffset(gy_offset);
	mpu.setZGyroOffset(gz_offset);
	mpu.setXAccelOffset(ax_offset);
	mpu.setYAccelOffset(ay_offset);
	mpu.setZAccelOffset(az_offset);
#endif



	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {

#ifdef dmp
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison		
		packetSize = mpu.dmpGetFIFOPacketSize();
#endif

	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

	delay(1000);
	baro.Reset();
	baro.GetCalibrationCoefficients();
	baro.Test(200, &kfzVariance);
	baro.AveragedSample(4);
	baro.InitializeSampleStateMachine();

	refAltitudeAtInitCm = baro.zCmAvg_;

	// //save the calculated variances
	// preferences.putInt("kfzVar", int(kfzVariance));
	// preferences.putInt("kfazVar", int(kazVariance));
	Serial.printf("kfzVariance initialized to %d\r\n", int(kfzVariance));
	Serial.printf("kfazVariance initialized to %d\r\n", int(kazVariance));

	// indicate calibration complete
	audio.GenerateTone(CALIB_TONE_FREQHZ, 1000);

	// initialize kalman filter with barometer estimated altitude, measured kfzvariance
	//and kfazvariance, the accelerometer noise variance. The value for kfazvariance should be
	//the actual environmental noise, ie from thermals, sinky air etc. But I've found 
	//that the value chosen based on the noise measured during calibration seems to work well.
	//
	
	kf.Config(kfzVariance, kazVariance, KF_ACCELBIAS_VARIANCE, baro.zCmAvg_, 0.0f, 0.0f);

	initTime();
	zAccelAccumulator = 0.0f;
	baroTimeDeltaSecs = 0.0f;
	baroCounter = 0;
	timeoutSeconds = 0;
	Serial.printf("finshed setup\r\n");


}


void loop() {
	
#ifdef dmp

	// wait for MPU interrupt or extra packet(s) available
	while (!drdyFlag && fifoCount < packetSize) {
		// other program behavior stuff here
		// .
		// .
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		// .
		// .
		// .

		while (SerialGPS.available()) {
			char c = char(SerialGPS.read());
			Serial.print(c);

			if (drdyFlag){
				Serial.println();
				break;
			}
		}


	}

	// get INT_STATUS byte
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow! resetting.."));
		DEBUG_PRINTLN(F("Enabling FIFO..."));
		mpu.setFIFOEnabled(true);
		DEBUG_PRINTLN(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);
	}
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO

		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;


		// display initial world-frame acceleration, adjusted to remove gravity
		// and rotated based on known orientation from quaternion
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		/*Serial.print("aworld\t");
		Serial.print(aaWorld.x);
		Serial.print("\t");
		Serial.print(aaWorld.y);
		Serial.print("\t");
		Serial.println(aaWorld.z);*/


	}
#endif


	if (drdyFlag) { // new MPU6050 data ready, 100Hz ODR => ~10mS sample interval

		drdyFlag = 0;
		updateTime();
#ifdef IMU_DEBUG		
		cct_SetMarker(); // set origin for estimating the time taken to read and process the data
#endif		

#ifdef dmp
		float gravityCompensatedAccel = float(-aaWorld.z*(1.0f / MPU6050_2G_SENSITIVITY));
		//Serial.print("gravity compensated accelration: "); Serial.println(gravityCompensatedAccel);
#else
		// accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in deg/second
		imu.GetAccelGyroData(accel, gyro);
		imu_MahonyAHRSupdateIMU(imuTimeDeltaSecs, gyro[0] * DEG_TO_RAD, gyro[1] * DEG_TO_RAD, gyro[2] * DEG_TO_RAD, accel[0], -accel[1], -accel[2]);
		float gravityCompensatedAccel = imu_GravityCompensatedAccel(accel[0], -accel[1], -accel[2], q0, q1, q2, q3);
		//Serial.print("gravity compensated accelration: "); Serial.println(gravityCompensatedAccel);
		//Serial.print("Quaternio vector w x y z"); Serial.print(q0); Serial.print(" "); Serial.print(q1); Serial.print(" "); Serial.print(q2); Serial.print(" "); Serial.println(q3);
		//Serial.print("ax "); Serial.print(accel[0], 4); Serial.print(" ay "); Serial.print(accel[1], 4); Serial.print(" az "); Serial.println(accel[2], 4);
		//Serial.print("gx "); Serial.print(gyro[0], 4); Serial.print(" gy "); Serial.print(gyro[1], 4); Serial.print(" gz "); Serial.println(gyro[2], 4);
		//Serial.print("q0 "); Serial.print(q0, 4); Serial.print(" q1 "); Serial.print(q1, 4); Serial.print(" q2 "); Serial.print(q2, 4); Serial.print(" q3 "); Serial.println(q3, 4);

#endif


		zAccelAccumulator += gravityCompensatedAccel; // one earth-z acceleration value computed every 10mS, accumulate

		baroCounter++;
		baroTimeDeltaSecs += imuTimeDeltaSecs;
		//Serial.print("mpu time delta secs"); Serial.println(imuTimeDeltaSecs,3); //this should be 10ms
		if (baroCounter >= 1) { // ~10mS elapsed, this is the sampling period for MS56XX, 
			baroCounter = 0;    // alternating between pressure and temperature samples

			int zMeasurementAvailable = baro.SampleStateMachine(); // one z (altitude) sample calculated for every new pair of pressure & temperature samples
			if (zMeasurementAvailable) {
				//Serial.print("baro time delta secs"); Serial.println(baroTimeDeltaSecs);
				float zAccelAverage = zAccelAccumulator / 2.0f; // average earth-z acceleration over the 20mS interval between z samples
				kf.Update(baro.zCmSample_, zAccelAverage, baroTimeDeltaSecs, &kfAltitudeCm, &kfClimbrateCps);
				zAccelAccumulator = 0.0f;
				baroTimeDeltaSecs = 0.0f;

				//calculate the integrated cps over period of net_vario secs default 4s - represents 200 samples at 20ms sample rate
				avgeCps = runningAverageofCps(kfClimbrateCps);
				//Serial.printf("4s average cps %d\r\n", int(avgeCps));

				audioCps = avgeCps >= 0.0f ? (int32_t)(avgeCps + 0.5f) : (int32_t)(avgeCps - 0.5f);
				if (ABS(audioCps) > SLEEP_THRESHOLD_CPS) { // reset sleep timeout watchdog if there is significant vertical motion
					timeoutSeconds = 0;
				}
				if (timeoutSeconds >= SLEEP_TIMEOUT_SECONDS) {
					Serial.print("Timed out, put MPU6050 and ES32 to sleep to minimize current draw when using battery\r\n");
					Serial.flush();
					indicatePowerDown();
					powerDown();
				}
				CLAMP(audioCps, -1000, 1000); // clamp climbrate to +/- 10m/sec
				audio.VarioBeep(audioCps);
				

			}
		}

		if (drdyCounter >= 100) {
				drdyCounter = 0;
				timeoutSeconds++; // 100 * 10mS = 1 second
#ifdef IMU_DEBUG
				uint32_t elapsedUs = cct_ElapsedTimeUs(); // calculate time  taken in reading and processing the data, should be less than 5mS worst case
				Serial.printf("Elapsed %dus\r\n", (int)elapsedUs);
				Serial.printf("bAlt = %d kfAlt = %d kfVario = %d\r\n", (int)baro.zCmSample_, (int)kfAltitudeCm, (int)kfClimbrateCps);

#endif	
				// OPENVARIO-NMEA OUTPUT
				// https://www.openvario.org/doku.php?id=projects:series_00:software:nmea
				char szmsg[100];
				char szcksum[5];
				sprintf(szmsg, "$POV,P,%.2f,T,%.2f,E,%.2f*", 
					baro.paSample_/100.0f,   // P: static pressure in hPa
					baro.tempCx100_/100.0f,  // T: temperature in deg C
					avgeCps/100.0f           // E: TE vario in m/s
					);
				// TODO?
				// S: true airspeed in km/h
				// Q: dynamic pressure in Pa
				// R: total pressure in hPa
				// V: battery voltage in V
				// 
				uint8_t cksum = btmsg_nmeaChecksum(szmsg);
				sprintf(szcksum,"%02X\r\n", cksum);
				strcat(szmsg, szcksum);
				Serial.println();
				Serial.printf(szmsg);

		}		
	}
}

