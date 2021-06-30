#include "sensors.h"
#include <Arduino.h>
#include <Wire.h>              //BME280 & OLED & SGP30
#include <Adafruit_Sensor.h>   //BME280
#include <Adafruit_BME280.h>   //BME280
#include "MHZ19.h"             //MHZ19
#include <SoftwareSerial.h>    //MHZ19 & SDS011
#include "SdsDustSensor.h"     //SDS011
#include "Adafruit_SGP30.h"    //SGP30
#include "Adafruit_VEML7700.h" //VEML7700
#include <EEPROM.h>

// Definitions
#define I2C_SDA SDA                    // BME280
#define I2C_SCL SCL                    // BME280
#define SEALEVELPRESSURE_HPA (1013.25) // BME280
#define RX_PIN_MHZ19 T4                //MHZ19
#define TX_PIN_MHZ19 T3                //MHZ19
#define BAUDRATE 9600                  //MHZ19
#define RX1 T6                         //SDS011
#define TX1 T5                         //SDS011
#define BATTERY A7                     //Battery

// Instances
Adafruit_BME280 bme;                                 // BME280
TwoWire I2CBME = TwoWire(1);                         // BME280
MHZ19 myMHZ19;                                       //MHZ19
SoftwareSerial mySerial(RX_PIN_MHZ19, TX_PIN_MHZ19); //MHZ19
SdsDustSensor sds(Serial1);                          //SDS011
Adafruit_SGP30 sgp;                                  //SGP30
Adafruit_VEML7700 veml = Adafruit_VEML7700();        //VEML7700

//Variables
float bme280_values[5] = {0, 0, 0, 0, 0}; // BME280
float bmeTemperature = 0;                 // BME280
float bmeHumidity = 0;                    // BME280
float bmePressure = 0;                    // BME280
int co2[2] = {0, 0};                      //MHZ19
float pm[2] = {0, 0};                     //SDS011
int sgp30[4] = {0, 0, 0, 0};              //SGP30
int counterSGP = 0;                       //SGP30
const int EEpromWrite = 24;               //SGP30 intervall in which to write new baselines into EEPROM in hours
unsigned long previousMillis = 0;         //SGP30  Millis at which the intervall started
uint16_t TVOC_base;                       //SGP30
uint16_t eCO2_base;                       //SGP30
float veml7700[4] = {0};                  //VEML7700
float veml7700Lux = 0;                    //VEML7700
float veml7700White = 0;                  //VEML7700
uint16_t veml7700ALS = 0;                 //VEML7700
float vBattery = 0;                       //Battery
bool autoCalibrationCO2 = true;           //SGP30

//BME280
float *getBme280Values(bool printValues)
{
    bme280_values[0] = bme.readTemperature();
    bme280_values[1] = bme.readTemperature();
    bme280_values[2] = bme.readHumidity();
    bme280_values[3] = bme.readPressure() / 100.0F;
    bme280_values[4] = bme.readAltitude(SEALEVELPRESSURE_HPA);

    if (printValues)
    {
        Serial.print("BME280 Temperature = ");
        Serial.print(bme280_values[1]);
        Serial.println(" *C");
        Serial.print("BME280 Humidity = ");
        Serial.print(bme280_values[2]);
        Serial.println(" %");
        Serial.print("BME280 Pressure = ");
        Serial.print(bme280_values[3] / 100.0F);
        Serial.println(" hPa");
        Serial.print("BME280 Approx. Altitude = ");
        Serial.print(bme280_values[4]);
        Serial.println(" m");
    }
    bme280_values[0] = roundf(bme280_values[0] * 100) / 100; //BME280
    bme280_values[1] = roundf(bme280_values[1] * 100) / 100; //BME280
    bme280_values[2] = roundf(bme280_values[2] * 100) / 100; //BME280
    bme280_values[3] = roundf(bme280_values[3] * 100) / 100; //BME280
    bme280_values[4] = roundf(bme280_values[4] * 100) / 100; //BME280
    return bme280_values;
}
//BME280
float getBme280Temperature()
{
    bmeTemperature = bme.readTemperature();
    Serial.print("BME280 Temperature = ");
    Serial.print(bmeTemperature);
    Serial.println(" *C");
    //bmeTemperature = roundf(bmeTemperature * 100) / 100;
    return bmeTemperature;
}
//BME280
float getBme280Humidity()
{
    bmeHumidity = bme.readHumidity();
    Serial.print("BME280 Humidity = ");
    Serial.print(bmeHumidity);
    Serial.println(" %");
    //bmeHumidity = roundf(bmeHumidity * 100) / 100;
    return bmeHumidity;
}
//BME280
float getBme280Pressure()
{
    bmePressure = bme.readPressure() / 100.0F;
    Serial.print("BME280 Pressure = ");
    Serial.print(bmePressure);
    Serial.println(" hPa");
    bmePressure = roundf(bmePressure * 100) / 100;
    return bmePressure;
}

//MHZ19
int *getCO2()
{
    co2[0] = myMHZ19.getCO2();
    Serial.print("MHZ19 CO2: ");
    Serial.println(co2[0]);
    co2[1] = myMHZ19.getCO2Raw();
    Serial.print("MHZ19 Raw CO2: ");
    Serial.println(co2[1]);
    Serial.print("MHZ19 Range CO2: ");
    Serial.println(myMHZ19.getRange());
    Serial.print("MHZ19 Background CO2: ");
    Serial.println(myMHZ19.getBackgroundCO2());
    Serial.print("MHZ19 Temp adjustment: ");
    Serial.println(myMHZ19.getTempAdjustment());
    Serial.print("MHZ19 ABC Status: ");
    myMHZ19.getABC() ? Serial.println("ON") : Serial.println("OFF");
    return co2;
}
//MHZ19
void calibrateMHZ19()
{
    Serial.println("MHZ19 Calibrating MHZ19");
    myMHZ19.calibrate();
}
//MHZ19
void modifyAutoCalibrationMHZ19(bool flag)
{
    Serial.println("MHZ19 Modifiying auto-calibration");
    myMHZ19.autoCalibration(flag, 24);
    Serial.print("MHZ19 ABC Status: ");
    myMHZ19.getABC() ? Serial.println("ON") : Serial.println("OFF");
}

//SDS011
void wakeupSDS011()
{
    sds.wakeup();
    vTaskDelay(10000);
}
//SDS011
void goToSleepSDS011()
{
    WorkingStateResult state = sds.sleep();
    if (state.isWorking())
    {
        Serial.println("ERROR SDS011: Problem with sleeping the sensor.");
    }
    else
    {
        Serial.println("SDS011 Sensor is sleeping");
    }
}

//SDS011
float *getPM()
{
    wakeupSDS011();
    PmResult pmResult = sds.queryPm();
    if (pmResult.isOk())
    {
        pm[0] = pmResult.pm25;
        pm[1] = pmResult.pm10;
        Serial.print("SDS011 PM2.5 = ");
        Serial.print(pmResult.pm25);
        Serial.print(", PM10 = ");
        Serial.println(pmResult.pm10);
    }
    else
    {
        Serial.print("ERROR SDS011: Could not read values from sensor SDS011, reason: ");
        Serial.println(pmResult.statusToString());
    }
    //goToSleepSDS011();
    pm[0] = roundf(pm[0] * 100) / 100; //SDS011
    pm[1] = roundf(pm[1] * 100) / 100; //SDS011
    return pm;
}

//SGP30
void autoCalibrationSGP30(bool flag)
{
    autoCalibrationCO2 = flag;
}

//SGP30
uint32_t getAbsoluteHumidity(float temperature, float humidity)
{
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = 1000.0f * absoluteHumidity;                                                                                       // [mg/m^3]
    return absoluteHumidityScaled;
}

//SGP30
int *getSGP30()
{
    if (!sgp.IAQmeasure())
    {
        Serial.println("ERROR SGP30: Measurement failed");
    }
    sgp30[0] = sgp.TVOC;
    sgp30[1] = sgp.eCO2;
    Serial.print("SGP30 TVOC ");
    Serial.print(sgp30[0]);
    Serial.print(" ppb\t");
    Serial.print("eCO2 ");
    Serial.print(sgp30[1]);
    Serial.println(" ppm");

    if (!sgp.IAQmeasureRaw())
    {
        Serial.println("ERROR SGP30: Raw Measurement failed");
    }
    sgp30[2] = sgp.rawH2;
    sgp30[3] = sgp.rawEthanol;
    Serial.print("SGP30 Raw H2 ");
    Serial.print(sgp30[3]);
    Serial.print(" \t");
    Serial.print("Raw Ethanol ");
    Serial.print(sgp30[3]);
    Serial.println("");
    Serial.print("SGP30 counterSGP: ");
    Serial.println(counterSGP);

    if (counterSGP % 2 == 0)
    {
        Serial.println("SGP30 ****Adjust humidity calibration");
        float *ptr_bme280_valuesSGP30 = getBme280Values(false);
        sgp.setHumidity(getAbsoluteHumidity(ptr_bme280_valuesSGP30[1], ptr_bme280_valuesSGP30[2]));
    }

    if (counterSGP % 30 == 0)
    {
        counterSGP = 0;
        uint16_t TVOC_base, eCO2_base;
        if (!sgp.getIAQBaseline(&eCO2_base, &TVOC_base))
        {
            Serial.println("ERROR SGP30 - Failed to get baseline readings");
        }
        Serial.print("SGP30 ****Baseline values: eCO2: 0x");
        Serial.print(eCO2_base, HEX);
        Serial.print(" & TVOC: 0x");
        Serial.println(TVOC_base, HEX);
    }
    if (counterSGP == 30)
    {
        counterSGP = 0;
    }
    else
    {
        counterSGP++;
    }
    // Prepare the EEPROMWrite   intervall
    unsigned long currentMillis = millis();
    if ((currentMillis - previousMillis >= EEpromWrite * 3600000) && autoCalibrationCO2)
    {
        previousMillis = currentMillis;             // reset the loop
        sgp.getIAQBaseline(&eCO2_base, &TVOC_base); // get the new baseline
        EEPROM.put(1, TVOC_base);                   // Write new baselines into EEPROM
        EEPROM.put(10, eCO2_base);
        EEPROM.commit();
        Serial.println("SGP30 Saving new calibration in EEPROM");
        Serial.print("SGP30 TVOC_base: ");
        Serial.println(TVOC_base);
        Serial.print("SGP30 eCO2_base: ");
        Serial.println(eCO2_base);
    }
    return sgp30;
}

//SGP30
void calibrateSGP30()
{
    Serial.println("SGP30 Calibration of SGP30");
    sgp.getIAQBaseline(&eCO2_base, &TVOC_base); // get the new baseline
    EEPROM.put(1, TVOC_base);                   // Write new baselines into EEPROM
    EEPROM.put(10, eCO2_base);
    EEPROM.commit();
    Serial.println("SGP30 Saving new calibration in EEPROM");
    Serial.print("SGP30 TVOC_base: ");
    Serial.println(TVOC_base);
    Serial.print("SGP30 eCO2_base: ");
    Serial.println(eCO2_base);
}

//VEML7700
void getVEML7700Info()
{
    Serial.print(F("VEML7700 Gain: "));
    switch (veml.getGain())
    {
    case VEML7700_GAIN_1:
        Serial.println("1");
        break;
    case VEML7700_GAIN_2:
        Serial.println("2");
        break;
    case VEML7700_GAIN_1_4:
        Serial.println("1/4");
        break;
    case VEML7700_GAIN_1_8:
        Serial.println("1/8");
        break;
    }
    Serial.print(F("VEML7700 Integration Time (ms): "));
    switch (veml.getIntegrationTime())
    {
    case VEML7700_IT_25MS:
        Serial.println("25");
        break;
    case VEML7700_IT_50MS:
        Serial.println("50");
        break;
    case VEML7700_IT_100MS:
        Serial.println("100");
        break;
    case VEML7700_IT_200MS:
        Serial.println("200");
        break;
    case VEML7700_IT_400MS:
        Serial.println("400");
        break;
    case VEML7700_IT_800MS:
        Serial.println("800");
        break;
    }
}

//VEML7700
float *getVEML7700()
{
    for (int i = 0; i < 4; ++i)
    {
        veml7700[i] = 0;
    }
    veml7700[0] = veml.readLux();
    veml7700[1] = veml.readLux();
    veml7700[2] = veml.readWhite();
    veml7700[3] = veml.readALS();
    Serial.print("VEML7700 Lux: ");
    Serial.println(veml7700[1]);
    Serial.print("VEML7700 White: ");
    Serial.println(veml7700[2]);
    Serial.print("VEML7700 Raw ALS: ");
    Serial.println(veml7700[3]);
    return veml7700;
}
float getVEML7700Lux()
{
    veml7700Lux = veml.readLux();
    Serial.print("VEML7700 Lux: ");
    Serial.println(veml7700Lux);
    return veml7700Lux;
}
float getVEML7700White()
{
    veml7700White = veml.readWhite();
    Serial.print("VEML7700 White: ");
    Serial.println(veml7700White);
    return veml7700White;
}
int getVEML7700ALS()
{
    veml7700ALS = veml.readALS();
    Serial.print("VEML7700 Raw ALS: ");
    Serial.println(veml7700ALS);
    return veml7700ALS;
}
float getVEML7700_()
{
    return veml.readLux();
}

//VEML7700
void setVEML7700PowerSave(bool powerSaveEnabled)
{
    veml.powerSaveEnable(powerSaveEnabled);
    if (powerSaveEnabled == true)
    {
        veml.setPowerSaveMode(VEML7700_POWERSAVE_MODE4);
    }
}

//Battery
float getVoltageBatteryValue()
{
    vBattery = (float)(analogRead(BATTERY)) / 4095 * 2 * 3.3 * 1.1;
    /*
  The ADC value is a 12-bit number, so the maximum value is 4095 (counting from 0).
  To convert the ADC integer value to a real voltage you’ll need to divide it by the maximum value of 4095,
  then double it (note above that Adafruit halves the voltage), then multiply that by the reference voltage of the ESP32 which 
  is 3.3V and then vinally, multiply that again by the ADC Reference Voltage of 1100mV.
  */
    Serial.print("BAT Vbat = ");
    Serial.print(vBattery);
    Serial.println(" Volts");
    return vBattery;
}

//Global
void sensors_init()
{
    //BME280
    I2CBME.begin(I2C_SDA, I2C_SCL, 100000); //BME280
    bool status;
    status = bme.begin(0x76, &I2CBME);
    if (!status)
    {
        Serial.println("ERROR BME280: Could not find a valid BME280 sensor, check wiring!");
    }

    //MHZ19
    mySerial.begin(BAUDRATE);
    myMHZ19.begin(mySerial);
    //myMHZ19.autoCalibration(autoCalibrationCO2, EEpromWrite);

    //SDS011
    sds.begin();
    wakeupSDS011();

    //SGP30
    if (!sgp.begin())
    {
        Serial.println("ERROR SGP30: Sensor not found");
        while (1)
            ;
    }
    Serial.print("SGP30 Found SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);
    EEPROM.get(1, TVOC_base);
    EEPROM.get(10, eCO2_base);
    if (eCO2_base != 0 && TVOC_base != 0xFFFF && eCO2_base != 0xFFFF)
    {
        Serial.println("SGP30 Previous calibration applied");
        sgp.setIAQBaseline(TVOC_base, eCO2_base);
    }
    else
    {
        Serial.println("SGP30 Default values applied");
        sgp.setIAQBaseline(0x94EC, 0x9253);
    }
    Serial.print("SGP30 ****Baseline values in EEPROM: eCO2: 0x");
    Serial.print(eCO2_base, HEX);
    Serial.print(" & TVOC: 0x");
    Serial.println(TVOC_base, HEX);

    //VEML7700
    if (!veml.begin())
    {
        Serial.println("ERROR VEML7700: Sensor VEML7700 not found");
        while (1)
            ;
    }
    Serial.println("VEML7700 Sensor VEML7700 found");
    veml.setGain(VEML7700_GAIN_1_4);            // Check table for specific precision
    veml.setIntegrationTime(VEML7700_IT_200MS); // Check table for specific precision
    getVEML7700Info();

    //Battery
    pinMode(BATTERY, INPUT);
}