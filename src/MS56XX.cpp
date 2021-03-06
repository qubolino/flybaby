#include <Arduino.h>
#include "Wire.h"
#include "MS56XX.h"

#define MS5607

MS56XX::MS56XX() {
	paSample_ = 0.0f;
	zCmSample_ = 0.0f;
	celsiusSample_ = 0;
	zCmAvg_ = 0.0f;
	}


#if 1


#define MAX_TEST_SAMPLES    200
extern char gszBuf[];
static float pa[MAX_TEST_SAMPLES];
static float z[MAX_TEST_SAMPLES];

void MS56XX::Test(int nSamples, float *kfzVariance) {
	int n;
    float paMean, zMean, zVariance, paVariance;
    paMean = 0.0f;
    zMean = 0.0f;
    paVariance = 0.0f;
	zVariance = 0.0;

	Serial.println("Calibrating pressure sensor");
	//ignore first 10 samples;
	for (int i = 0; i < 10; i++) {
		TriggerPressureSample();
		D2_ = ReadSample();
		delay(MS56XX_SAMPLE_PERIOD_MS);
	}
    for (n = 0; n < nSamples; n++) {
	    TriggerTemperatureSample();
	    delay(MS56XX_SAMPLE_PERIOD_MS);
	    D2_ = ReadSample();
	    CalculateTemperatureCx10();
		TriggerPressureSample();
		delay(MS56XX_SAMPLE_PERIOD_MS);
		D1_ = ReadSample();
		pa[n] = CalculatePressurePa();
        z[n] =  Pa2Cm(pa[n]);
        paMean += pa[n];
        zMean += z[n];
        }
    paMean /= nSamples;
    zMean /= nSamples;
    for (n = 0; n < nSamples; n++) {
        paVariance += (pa[n]-paMean)*(pa[n]-paMean);
        zVariance += (z[n]-zMean)*(z[n]-zMean);
       }
    paVariance /= (nSamples-1);
    zVariance /=  (nSamples-1);
	*kfzVariance = zVariance;
    Serial.printf("paMean = %d Pa (+/- %.2f@95%%), zMean = %dcm (+/- %.2f@95%%)\r\n",(int)paMean, 1.96*sqrt(paVariance)/sqrt(nSamples), (int)zMean, 1.96*sqrt(zVariance)/sqrt(nSamples));
    Serial.printf("\r\npaVariance %d  zVariance %d\r\n",(int)paVariance, (int)zVariance);
	}
#endif

void MS56XX::AveragedSample(int nSamples) {
	int32_t tc,tAccum,n;
    float pa,pAccum;
	pAccum = 0.0f;
    tAccum = 0;
	n = 2;
    while (n--) {
    	TriggerTemperatureSample();
    	delay(MS56XX_SAMPLE_PERIOD_MS);
		D2_ = ReadSample();
		TriggerPressureSample();
		delay(MS56XX_SAMPLE_PERIOD_MS);
		D1_ = ReadSample();
		}
	
	n = nSamples;
    while (n--) {
    	TriggerTemperatureSample();
    	delay(MS56XX_SAMPLE_PERIOD_MS);
		D2_ = ReadSample();
		CalculateTemperatureCx10();
		TriggerPressureSample();
		delay(MS56XX_SAMPLE_PERIOD_MS);
		D1_ = ReadSample();
		pa = CalculatePressurePa();
		pAccum += pa;
		tAccum += tempCx100_;
		}
	tc = tAccum/nSamples;
	celsiusSample_ = (tc >= 0 ?  (tc+50)/100 : (tc-50)/100);
	paSample_ = (pAccum+nSamples/2)/nSamples;
	zCmAvg_ = zCmSample_ = Pa2Cm(paSample_);
#if 1
   Serial.printf("Tavg : %dC\r\n",celsiusSample_);
   Serial.printf("Pavg : %dPa\r\n",(int)paSample_);
   Serial.printf("Zavg : %dcm\r\n",(int)zCmAvg_);
#endif

	}
	
	

/// Fast Lookup+Interpolation method for converting pressure readings to altitude readings.
#include "pztbl.h"

float MS56XX::Pa2Cm(float paf)  {
   	int32_t pa,inx,pa1,z1,z2;
    float zf;
    pa = (int32_t)(paf);

   	if (pa > PA_INIT) {
      	zf = (float)(gPZTbl[0]);
      	}
   	else {
      	inx = (PA_INIT - pa)>>10;
      	if (inx >= PZLUT_ENTRIES-1) {
         	zf = (float)(gPZTbl[PZLUT_ENTRIES-1]);
         	}
      	else {
         	pa1 = PA_INIT - (inx<<10);
         	z1 = gPZTbl[inx];
         	z2 = gPZTbl[inx+1];
         	zf = (float)(z1) + ( ((float)pa1-paf)*(float)(z2-z1))/1024.0f;
         	}
      	}
   	return zf;
   	}

void MS56XX::CalculateTemperatureCx10(void) {
	dT_ = (int64_t)D2_ - tref_;
	tempCx100_ = 2000 + ((dT_*((int32_t)cal_[5]))>>23);
	}


float MS56XX::CalculatePressurePa(void) {
	float pa;
    int64_t offset, sens,offset2,sens2,t2;
#ifdef MS5607
	offset = offT1_ + ((((int64_t)cal_[3])*dT_)>>6);
	sens = sensT1_ + ((((int64_t)cal_[2])*dT_)>>7);
#endif
#ifdef MS5611
	offset = offT1_ + ((((int64_t)cal_[3])*dT_)>>7);
	sens = sensT1_ + ((((int64_t)cal_[2])*dT_)>>8);
#endif
    if (tempCx100_ < 2000) { // correction for temperature < 20C
        t2 = ((dT_*dT_)>>31); 
#ifdef MS5607
        offset2 = 61  * (tempCx100_-2000) * (tempCx100_-2000) / 16;
        sens2 = (tempCx100_-2000) * (tempCx100_-2000) / 2.0f;
#endif
#ifdef MS5611
        offset2 = 5  * (tempCx100_-2000) * (tempCx100_-2000) / 2;
        sens2 = offset2 / 2;
#endif
		if (tempCx100_ < -15000) { //correction for temperature < -15 C
#ifdef MS5607
			offset2 += 15 * (tempCx100_ + 1500) * (tempCx100_ + 1500);
			sens2 += 8 * (tempCx100_ + 1500) * (tempCx100_ + 1500);
#endif
#ifdef MS5611
			offset2 += 7 * (tempCx100_ + 1500) * (tempCx100_ + 1500);
			sens2 += 11 * (tempCx100_ + 1500) * (tempCx100_ + 1500) / 2;
#endif
		}
	} 
    else {
        t2 = 0;
        sens2 = 0;
        offset2 = 0;
        }
    tempCx100_ -= t2;
    offset -= offset2;
    sens -= sens2;
	pa = (((float)((int64_t)D1_ * sens))/2097152.0f - (float)offset) / 32768.0f;
	return pa;
}


/// Trigger a pressure sample with max oversampling rate
void MS56XX::TriggerPressureSample(void) {
	Wire.beginTransmission(MS56XX_I2C_ADDRESS);  
	Wire.write(MS56XX_CONVERT_D1 | MS56XX_ADC_4096);  //  pressure conversion, max oversampling
	Wire.endTransmission();  
   }

/// Trigger a temperature sample with max oversampling rate
void MS56XX::TriggerTemperatureSample(void) {
	Wire.beginTransmission(MS56XX_I2C_ADDRESS);  
	Wire.write(MS56XX_CONVERT_D2 | MS56XX_ADC_4096);   //  temperature conversion, max oversampling
	Wire.endTransmission();  
   }

uint32_t MS56XX::ReadSample(void)	{
	Wire.beginTransmission(MS56XX_I2C_ADDRESS);  // Initialize the Tx buffer
	Wire.write(0x00);                        // Put ADC read command in Tx buffer
	Wire.endTransmission(false);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(MS56XX_I2C_ADDRESS, 3);     // Read three bytes from slave PROM address 
	int inx = 0;
	uint8_t buf[3];
	while (Wire.available()) {
        buf[inx++] = Wire.read(); 
		}          
	uint32_t w = (((uint32_t)buf[0])<<16) | (((uint32_t)buf[1])<<8) | (uint32_t)buf[2];
	return w;
   }


void MS56XX::InitializeSampleStateMachine(void) {
   TriggerTemperatureSample();
   sensorState = MS56XX_READ_TEMPERATURE;
   }

int MS56XX::SampleStateMachine(void) {
   if (sensorState == MS56XX_READ_TEMPERATURE) {
      D2_ = ReadSample();
      TriggerPressureSample();
      //DBG_1(); // turn on the debug pulse for timing the critical computation
      CalculateTemperatureCx10();
      //celsiusSample_ = (tempCx100_ >= 0? (tempCx100_+50)/100 : (tempCx100_-50)/100);
      paSample_ = CalculatePressurePa();
	  zCmSample_ = Pa2Cm(paSample_);
      //DBG_0();
	  sensorState = MS56XX_READ_PRESSURE;
      return 1;  // 1 => new altitude sample is available
      }
   else
   if (sensorState == MS56XX_READ_PRESSURE) { 
      D1_ = ReadSample();
      TriggerTemperatureSample();
      sensorState = MS56XX_READ_TEMPERATURE;
      return 0; // 0 => intermediate state
      }
   return 0;
   }

void MS56XX::Reset() {
	Wire.beginTransmission(MS56XX_I2C_ADDRESS);  
	Wire.write(MS56XX_RESET);                
	Wire.endTransmission();       
	delay(5); // 3mS as per app note AN520	
    }
   
	
void MS56XX::GetCalibrationCoefficients(void)  {
    for (int inx = 0; inx < 6; inx++) {
		int promIndex = 2 + inx*2; 
		cal_[inx] = (((uint16_t)prom_[promIndex])<<8) | (uint16_t)prom_[promIndex+1];
		}
    //Serial.printf("\r\nCalib Coeffs : %d %d %d %d %d %d\r\n",cal_[0],cal_[1],cal_[2],cal_[3],cal_[4],cal_[5]);
    tref_ = ((int64_t)cal_[4])<<8;
#ifdef MS5607
    offT1_ = ((int64_t)cal_[1])<<17;
    sensT1_ = ((int64_t)cal_[0])<<16;	
#endif
#ifdef MS5611
    offT1_ = ((int64_t)cal_[1])<<16;
    sensT1_ = ((int64_t)cal_[0])<<15;
#endif
    }

int MS56XX::ReadPROM(void)    {
    for (int inx = 0; inx < 8; inx++) {
		Wire.beginTransmission(MS56XX_I2C_ADDRESS); 
		Wire.write(0xA0 + inx*2); 
		Wire.endTransmission(false); // restart
		Wire.requestFrom(MS56XX_I2C_ADDRESS, 2); 
		int cnt = 0;
		while (Wire.available()) {
			prom_[inx*2 + cnt] = Wire.read(); 
			cnt++;
			}
		}			
	//Serial.printf("\r\nProm : ");
	//for (int inx = 0; inx < 16; inx++) {
	//	Serial.printf("0x%02x ", prom_[inx]);
	//	}
	//Serial.printf("\r\n");
	uint8_t crcPROM = prom_[15] & 0x0F;
	uint8_t crcCalculated = CRC4(prom_);
	return (crcCalculated == crcPROM ? 1 : 0);
	}
	
	
uint8_t MS56XX::CRC4(uint8_t prom[] ) {
	 int cnt, nbit; 
	 uint16_t crcRemainder; 
	 uint8_t crcSave = prom[15]; // crc byte in PROM
	 Serial.printf("PROM CRC = 0x%x\r\n", prom[15] & 0x0F);
	 crcRemainder = 0x0000;
	 prom[15] = 0; //CRC byte is replaced by 0
	 
	 for (cnt = 0; cnt < 16; cnt++)  {
		crcRemainder ^= (uint16_t) prom[cnt];
		for (nbit = 8; nbit > 0; nbit--) {
			if (crcRemainder & (0x8000)) {
				crcRemainder = (crcRemainder << 1) ^ 0x3000; 
				}
			else {
				crcRemainder = (crcRemainder << 1);
				}
			}
		}
	 crcRemainder= (0x000F & (crcRemainder >> 12)); // final 4-bit reminder is CRC code
	 prom[15] = crcSave; // restore the crc byte
	 Serial.printf("Calculated CRC = 0x%x\r\n",  crcRemainder ^ 0x0);
	 return (uint8_t)(crcRemainder ^ 0x0);
	} 