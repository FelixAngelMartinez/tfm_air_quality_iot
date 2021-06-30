#ifndef SENSORS_H /* include guards */
#define SENSORS_H
#include <stdint.h>

float *getBme280Values(bool printValues);                                        // BME280
float getBme280Temperature();                                    // BME280
float getBme280Humidity();                                       // BME280
float getBme280Pressure();                                       // BME280
int *getCO2();                                                   //MHZ19
void calibrateMHZ19();                                           //MHZ19
void modifyAutoCalibrationMHZ19(bool flag);                      //MHZ19
void wakeupSDS011();                                             //SDS011
void goToSleepSDS011();                                          //SDS011
float *getPM();                                                  //SDS011
void autoCalibrationSGP30(bool flag);                            //SGP30
int *getSGP30();                                                 //SGP30
void calibrateSGP30();                                           //SGP30
uint32_t getAbsoluteHumidity(float temperature, float humidity); //SGP30
float *getVEML7700();                                            //VEML7700
float getVEML7700Lux();                                          //VEML7700
float getVEML7700White();                                        //VEML7700
int getVEML7700ALS();                                            //VEML7700
float getVEML7700_();                                            //VEML7700
float getVoltageBatteryValue();                                  //VEML7700
void setVEML7700PowerSave(bool powerSaveEnabled);                //VEML7700
float getVoltageBatteryValue();                                  //Battery
void sensors_init();                                             //Global

#endif /* SENSORS_H */