#ifndef LEDSTRIPE_H /* include guards */
#define LEDSTRIPE_H
#include <Arduino.h>

void colorWipe(uint32_t c, uint8_t waitTime);
void setColorRGB(uint8_t color);
void buzzerSound();
void stopBuzzer();
void buzzerCallback();
void utilities_init();

#endif /* LEDSTRIPE_H */