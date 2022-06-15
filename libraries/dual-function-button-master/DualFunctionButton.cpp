
/*******************************************************************
 * 
 * Written by Berran Remzi | March 22, 2018 | https://github.com/bercho
 * 
 *******************************************************************/

#include "DualFunctionButton.h"

DualFunctionButton::DualFunctionButton(int buttonP, long longPressT, char inputMode) {
  this-> mode = inputMode;
  this-> buttonPin = buttonP;
  this-> longPressTime = longPressT;
  pinMode(this-> buttonPin, inputMode);
}

bool DualFunctionButton::longPress() {
  evaluatePress();
  if (this->longPressDetected == 1) {
    this->longPressDetected = 0;
    return true;
  }
  else return 0;
}

bool DualFunctionButton::shortPress() {
  evaluatePress();
  if (this->shortPressDetected == 1) {
    this->shortPressDetected = 0;
    return true;
  }
  else return 0;
}

void DualFunctionButton::evaluatePress() {
  bool pinStatus = digitalRead(this->buttonPin);
  if ((pinStatus == HIGH && this->mode==INPUT) || (pinStatus == LOW && this->mode==INPUT_PULLUP)) {
    if (this->buttonActive == false) {
      this->buttonActive = true;
      this->buttonTimer = millis();
    }
    if ((unsigned long)(millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      this->longPressActive = true;
      this->longPressDetected = true;
    }
  } else {
    if (this->buttonActive == true) {
      if (this->longPressActive == true) {
        this->longPressActive = false;
      }
      else {
        this->shortPressDetected = true;
      }
      this->buttonActive = false;
    }
  }
}


