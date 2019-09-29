/*
 * Rotary encoder library for Arduino.
 */

#ifndef rotary_h
#define rotary_h

#include "Arduino.h"

// Enable this to emit codes twice per step.
#define HALF_STEP

// Enable weak pullups
#define ENABLE_PULLUPS

// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20


class Rotary
{
  public:
    const unsigned char BUTTON_RESET = 0x00;
    const unsigned char BUTTON_PRESSED = 0x01;
    const unsigned char BUTTON_RELEASED = 0x10;
    const unsigned char BUTTON_PRESSED_RELEASED = 0x11;
    Rotary(char, char);
    Rotary(char, char, char);
    // Process pin(s)
    unsigned char process();
    unsigned char clockwise();
    unsigned char counterClockwise();
    bool buttonPressedReleased(short);
    bool buttonPressedHeld(short);
    unsigned char readButton();
    void resetButton();
  private:
  	void init(char, char);
    unsigned char state;
    unsigned char pin1;
    unsigned char pin2;
    unsigned char buttonPin;
    unsigned char buttonState;
    unsigned long buttonTimer;
};

#endif
 
