#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>

void blink(unsigned char no)
{
  for (int i=0; i<no; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a half second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(500);                       // wait for a half second
  }
}

void printPaddedInt16b(int16_t val) {
  if (val > 0) {
    Serial.print(" ");
    if(val < 10000){ Serial.print("0"); }
    if(val < 1000 ){ Serial.print("0"); }
    if(val < 100  ){ Serial.print("0"); }
    if(val < 10   ){ Serial.print("0"); }
  } else {
    Serial.print("-");
    if(abs(val) < 10000){ Serial.print("0"); }
    if(abs(val) < 1000 ){ Serial.print("0"); }
    if(abs(val) < 100  ){ Serial.print("0"); }
    if(abs(val) < 10   ){ Serial.print("0"); }
  }
  Serial.print(abs(val));
}

#endif