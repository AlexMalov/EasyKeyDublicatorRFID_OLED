/*
  LedSwitch PullUp - example program for using dual function on one button
  
  last modified: 22.03.2018

  Author: Berran Remzi
  https://github.com/bercho
*/

#include "DualFunctionButton.h"

#define LED 13
#define buttonInput 7

DualFunctionButton button(buttonInput, 1000, INPUT_PULLUP);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  pinMode(buttonInput, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (button.shortPress()) {
    digitalWrite(LED, HIGH);
  }
  if (button.longPress()) {
    digitalWrite(LED, LOW);
  }
}