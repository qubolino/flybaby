#ifndef MS56XX_H_
#define MS56XX_H_

//#define MS56XX_TEST

// max conversion time with OSR=4096 is  9.04mS
#define MS56XX_SAMPLE_PERIOD_MS         10

#define MS56XX_READ_TEMPERATURE 		11
#define MS56XX_READ_PRESSURE			22


#define MS56XX_I2C_ADDRESS 0x77

#define MS56XX_RESET      	0x1E
#define MS56XX_CONVERT_D1 	0x40
#define MS56XX_CONVERT_D2 	0x50
#define MS56XX_ADC_READ   	0x00

#define MS56XX_ADC_4096 	0x08

class MS56XX {
	
public :
MS56XX();
void TriggerPressureSample(void);
void TriggerTemperatureSample(void);
uint32_t  ReadSample(void);
void AveragedSample(int nSamples);
void CalculateTemperatureCx10(void);
float CalculatePressurePa(void);
void CalculateSensorNoisePa(void);
void Reset(void);

void GetCalibrationCoefficients(void);
float Pa2Cm(float pa);
void Test(int nSamples, float *kfzVariance);
int  ReadPROM(void);
uint8_t CRC4(uint8_t prom[] );
int  SampleStateMachine(void);
void InitializeSampleStateMachine(void);

volatile float paSample_;
volatile float zCmSample_;
float zCmAvg_;
int  celsiusSample_;
volatile int sensorState;
int32_t tempCx100_;

private :
uint8_t prom_[16];
uint16_t cal_[6];
int64_t tref_;
int64_t offT1_;
int64_t sensT1_;
uint32_t D1_;
uint32_t D2_;
int64_t dT_;
};

#endif // MS56XX_H_
