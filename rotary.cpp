/* Rotary encoder handler for arduino. v1.1
 *
 * Copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3.
 * Contact: bb@cactii.net
 *
 * A typical mechanical rotary encoder emits a two bit gray code
 * on 3 output pins. Every step in the output (often accompanied
 * by a physical 'click') generates a specific sequence of output
 * codes on the pins.
 *
 * There are 3 pins used for the rotary encoding - one common and
 * two 'bit' pins.
 *
 * The following is the typical sequence of code on the output when
 * moving from one step to the next:
 *
 *   Position   Bit1   Bit2
 *   ----------------------
 *     Step1     0      0
 *      1/4      1      0
 *      1/2      1      1
 *      3/4      0      1
 *     Step2     0      0
 *
 * From this table, we can see that when moving from one 'click' to
 * the next, there are 4 changes in the output code.
 *
 * - From an initial 0 - 0, Bit1 goes high, Bit0 stays low.
 * - Then both bits are high, halfway through the step.
 * - Then Bit1 goes low, but Bit2 stays high.
 * - Finally at the end of the step, both bits return to 0.
 *
 * Detecting the direction is easy - the table simply goes in the other
 * direction (read up instead of down).
 *
 * To decode this, we use a simple state machine. Every time the output
 * code changes, it follows state, until finally a full steps worth of
 * code is received (in the correct order). At the final 0-0, it returns
 * a value indicating a step in one direction or the other.
 *
 * It's also possible to use 'half-step' mode. This just emits an event
 * at both the 0-0 and 1-1 positions. This might be useful for some
 * encoders where you want to detect all positions.
 *
 * If an invalid state happens (for example we go from '0-1' straight
 * to '1-0'), the state machine resets to the start until 0-0 and the
 * next valid codes occur.
 *
 * The biggest advantage of using a state machine over other algorithms
 * is that this has inherent debounce built in. Other algorithms emit spurious
 * output with switch bounce, but this one will simply flip between
 * sub-states until the bounce settles, then continue along the state
 * machine.
 * A side effect of debounce is that fast rotations can cause steps to
 * be skipped. By not requiring debounce, fast rotations can be accurately
 * measured.
 * Another advantage is the ability to properly handle bad state, such
 * as due to EMI, etc.
 * It is also a lot simpler than others - a static state table and less
 * than 10 lines of logic.
 */

 /* Modified 5/22/2015 by Phill Fisk
  * Added methods and constructor to handle button-equiped rotary encoders
  * Added support methods to return clockwise and counter-clockwise defines
  */

#include "Arduino.h"
#include "rotary.h"

/*
 * The below state table has, for each state (row), the new state
 * to set based on the next encoder output. From left to right in,
 * the table, the encoder outputs are 00, 01, 10, 11, and the value
 * in that position is the new state to set.
 */

#define R_START 0x0

#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5
//original code from Buxtronixs
//const unsigned char ttable[6][4] = {
  // R_START (00)
  //{R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  //{R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  //{R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  //{R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  //{R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  //{R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
//};

/* Modified 5/04/2019 by Carlos Siles
  * Modified table to follow sequence 00>10>11>01>01
  */
const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START,           R_CCW_BEGIN,  R_CW_BEGIN,    R_START_M},
  // R_CCW_BEGIN
  {R_START,           R_CCW_BEGIN,  R_START,       R_START_M | DIR_CCW},
  // R_CW_BEGIN
  { R_START,          R_START,      R_CW_BEGIN ,   R_START_M | DIR_CW},
  // R_START_M (11)
  {R_START,           R_CW_BEGIN_M, R_CCW_BEGIN_M, R_START_M},
  // R_CW_BEGIN_M
  {R_START | DIR_CW,  R_CW_BEGIN_M, R_START_M,     R_START_M},
  // R_CCW_BEGIN_M
  {R_START | DIR_CCW, R_START_M,    R_CCW_BEGIN_M, R_START_M},
};

#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

//original code from Buxtronixs
//const unsigned char ttable[7][4] = {
  // R_START
  //{R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  //{R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  //{R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  //{R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  //{R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  //{R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  //{R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
//};

/* Modified 5/04/2019 by Carlos Siles
  * Modified table to follow sequence 00>10>11>01>01
  */
  
const unsigned char ttable[7][4] = {
// R_START
{R_START,           R_CCW_BEGIN, R_CW_BEGIN,   R_START},
  // R_CW_FINAL
{R_START | DIR_CW,  R_CW_FINAL,  R_START,      R_CW_NEXT},
  // R_CW_BEGIN
{R_START,           R_START,     R_CW_BEGIN,   R_CW_NEXT},
  // R_CW_NEXT
{R_START,           R_CW_FINAL,  R_CW_BEGIN,   R_CW_NEXT},
  // R_CCW_BEGIN
{R_START,           R_CCW_BEGIN, R_START,      R_CCW_NEXT},
  // R_CCW_FINAL
{R_START | DIR_CCW, R_START,     R_CCW_FINAL,  R_CCW_NEXT},
  // R_CCW_NEXT
{R_START,           R_CCW_BEGIN, R_CCW_FINAL,  R_CCW_NEXT},
};

#endif

/*
 * Constructor. Each arg is the pin number for each encoder contact.
 */
Rotary::Rotary(char _pin1, char _pin2) {
  // Moved to init method()
  init(_pin1, _pin2);
}

/*
 * Overloaded constructor, added to support rotary encoders that 
 * also include a button
 */
Rotary::Rotary(char _pin1, char _pin2, char _buttonPin) {
  // intiialize the rotary encoder button
  buttonTimer = 0;
  buttonPin = _buttonPin;
  pinMode(buttonPin, INPUT);
#ifdef ENABLE_PULLUPS
  digitalWrite(buttonPin, HIGH);
#endif

  // run the original initialization
  init(_pin1, _pin2);
}

/*
 * Moved constructor initialization logic here to support multiple constructors
 */
void Rotary::init(char _pin1, char _pin2) {
  // Assign variables.
  pin1 = _pin1;
  pin2 = _pin2;
  // Set pins to input.
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
#ifdef ENABLE_PULLUPS
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, HIGH);
#endif
  // Initialise state.
  state = R_START;
}

unsigned char Rotary::process() {
  // Grab state of input pins.
  unsigned char pinstate = (digitalRead(pin2) << 1) | digitalRead(pin1);
  // Determine new state from the pins and state table.
  state = ttable[state & 0xf][pinstate];
  // Return emit bits, ie the generated event.
  return state & 0x30;
}

/*
 * Added to return clockwise def. makes sketch easier to read 
 * (just check against this method for movement)
 */
unsigned char Rotary::clockwise() {
  return DIR_CW;
}

/*
 * Added to return counter-clockwise def. makes sketch easier to read 
 * (just check against this method for movement)
 */
unsigned char Rotary::counterClockwise() {
  return DIR_CCW;
}

/*
 * Reads the rotary encoder button, and returns true if the 
 * button has been pressed and released, based on the debounce dealy passed in
 *
 * Presuming the button is wired with one lead to a GPIO pin and the other pin 
 * to ground, the button will read high when open (not pressed) and low when
 * closed (pressed). In other words, a transition of high -> low -> high (1 -> 0 -> 1)
 * will indicate a button "press". The debounce delay is configurable (passed in)
*/
bool Rotary::buttonPressedReleased(short debounce_delay) {
  if (buttonState == BUTTON_PRESSED) {                  // If the button has been pressed
    if (millis() - buttonTimer > debounce_delay) {      // and the debounce timer has expired
      if (digitalRead(buttonPin)) {                     // and the pin reads HIGH (open)
        buttonState |= BUTTON_RELEASED;                 // then the button has been pressed and released
      }
    }
  }
  else {                                                // if the button hasn't been pressed yet, read the pin
    if (!digitalRead(buttonPin)) {                      // if the pin is LOW (closed), then the button has been pushed in
      buttonState |= BUTTON_PRESSED;                    // set the state
      buttonTimer = millis();                           // and start the timer
    }
  }

  // Check to see if the button has been pressed and released
  if (buttonState == (BUTTON_PRESSED | BUTTON_RELEASED)) { 
    buttonState = 0x00;                                 // Reset the button state before returning true
    return true;
  }
  else {
    return false;
  }
}

/*
* Reads the encoder button, and returns true if the button has been
* pressed and held down for a given amount of time.
*/
bool Rotary::buttonPressedHeld(short delay_millis) {
  // Is the button closed (being pressed)?  
  if (!digitalRead(buttonPin)) {
    // If this is the first time we've checked the button, set the state and start the timer
    if (buttonState != BUTTON_PRESSED) {
      buttonState = BUTTON_PRESSED;
      buttonTimer = millis();
    }
    else {
      // Otherwise, check the timer to see if it's expired
      if (millis() - buttonTimer > delay_millis) {
        // the wait timer has expired, reset the button state
        buttonState = BUTTON_RESET;
        // and return true indicating the button has been held down for long enough
        return true;
      }
    }
  }
  else {
    // button is open (not being pressed) was it already pressed?
    if (buttonState == BUTTON_PRESSED) {
      // it was pressed and released too quickly, reset it
      buttonState = BUTTON_RESET;
    }
  }

  return false;
}

/*
* Reads the encoder button and returns the current state (pressed OR released)
* Does not return the composite state (just for checking the state right now)
*/
unsigned char Rotary::readButton() {
  if (digitalRead(buttonPin)) {
    return BUTTON_RELEASED;
  }
  else {
    return BUTTON_PRESSED;
  }
}

/*
* Resets the state of the button
*/
void Rotary::resetButton() {
  buttonState = BUTTON_RESET;
}