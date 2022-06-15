
/*******************************************************************
 * 
 * Written by Berran Remzi | March 22, 2018 | https://github.com/bercho
 * 
 *******************************************************************/

#ifndef DUALFUNCTIONBUTTON_H
#define DUALFUNCTIONBUTTON_H

#include <Arduino.h>

class DualFunctionButton {
  public:
    DualFunctionButton(int buttonP, long longPressT){
        DualFunctionButton(buttonP, longPressT, (char)INPUT);
    };
    DualFunctionButton(int buttonP, long longPressT, char inputMode);
    bool longPress();
    bool shortPress();
    void evaluatePress();
    
  private:
    int buttonPin;
    char mode;
    boolean outputOneState = false;
    boolean outputTwoState = false;
    boolean shortPressDetected = false;
    boolean longPressDetected = false;
    unsigned long buttonTimer = 0;
    unsigned int longPressTime = 250;

    boolean buttonActive = false;
    boolean longPressActive = false;
};

#endif
