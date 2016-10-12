#include "core_pins.h"
#include "pins_arduino.h"
#include "HardwareSerial.h"
#include "WProgram.h"

elapsedMillis timer;
unsigned long time_decimal;

void setup()
{
  Serial.begin(9600);  //Start USB serial
  pinMode(12, INPUT); //Want to RX interrupt signal on this pin
  interrupts();  // enable Interrupts
  attachInterrupt(12, syncTime, RISING);
}

void loop()
{
  cli();
  //Serial.println(timer);
  sei();
  delay(500);
}

void syncTime(void)
{
  Serial.print("Before: ");
  Serial.println(timer);
  time_decimal = timer % 1000;
  if (time_decimal < 500) {
    timer -= time_decimal; 
  } else {
    timer += (1000 - time_decimal);
  }
  Serial.print("After: ");
  Serial.println(timer);
}
