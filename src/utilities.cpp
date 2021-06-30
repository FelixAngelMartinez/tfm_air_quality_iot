#include "utilities.h"
#include <Arduino.h>
#include <Adafruit_NeoPixel.h> //Led strip
#include "EasyBuzzer.h"        // Buzzer

// Definitions
#define PIN T2   // Led strip
#define GREEN 1  // Led strip
#define YELLOW 2 // Led strip
#define RED 3    // Led strip
#define BLUE 4   // Led strip

unsigned int frequency = 1000;    //Buzzer
unsigned int onDuration = 50;     //Buzzer
unsigned int offDuration = 100;   //Buzzer
unsigned int beeps = 10;          //Buzzer
unsigned int pauseDuration = 500; //Buzzer
unsigned int cycles = 2;          //Buzzer

// Instances objects creation
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800); // Led strip

// Led strip
void colorWipe(uint32_t c, uint8_t waitTime)
{
    for (uint16_t i = 0; i < strip.numPixels(); i++)
    {
        strip.setPixelColor(i, c);
        strip.show();
        vTaskDelay(waitTime);
    }
}

// Led strip
void setColorRGB(uint8_t color)
{
    if (color == 1)
    {
        colorWipe(strip.Color(0, 255, 0), 50); // Green
    }
    else if (color == 2)
    {
        colorWipe(strip.Color(255, 255, 0), 50); // Yellow
    }
    else if (color == 3)
    {
        colorWipe(strip.Color(255, 0, 0), 50); // Red
    }
    else if (color == 4)
    {
        colorWipe(strip.Color(0, 0, 255), 50); // Blue
    }
}

//Buzzer
void buzzerSound()
{
    EasyBuzzer.update();
}
//Buzzer
void stopBuzzer()
{
    EasyBuzzer.stopBeep();
}
//Buzzer
void buzzerCallback()
{
    Serial.print("Buzzer callBack Function");
}

//Library
void utilities_init()
{
    strip.begin();           //Led
    strip.setBrightness(80); //Led
    strip.show();            //Led Initialize all pixels to 'off

    EasyBuzzer.setPin(T0); //Buzzer
    EasyBuzzer.beep(
        frequency,     // Frequency in hertz(HZ).
        onDuration,    // On Duration in milliseconds(ms).
        offDuration,   // Off Duration in milliseconds(ms).
        beeps,         // The number of beeps per cycle.
        pauseDuration, // Pause duration.
        cycles,        // The number of cycle.
        buzzerCallback // [Optional] Callback. A function to call when the sequence ends.
    );
    stopBuzzer(); //Buzzer
}