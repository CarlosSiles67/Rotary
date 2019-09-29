/* Example sketch for Rotary library. 
 * Original library written by Ben Buxton (available at http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html)
 * Extended to support encoders with a push button
 *
 * Simple demonstration that will output to the serial monitor when the enocder is turned
 * or the button is pressed.
 * 
 */
#include <rotary.h>;

// Initialize the Rotary object
// Rotary(Encoder Pin 1, Encoder Pin 2, Button Pin)
Rotary r = Rotary(5, 3, 4);

void setup() {
  Serial.begin(9600);
  Serial.println("Initialized");
}

void loop() {
  // During each loop, check the encoder to see if it's been changed.
  volatile unsigned char val = r.process();

  // if the encoder has been turned, check the direction
  if (val) {
    if (val == r.clockwise()) {
      Serial.println("Clockwise");                    
    }
    else if (val == r.counterClockwise()) {
      Serial.println("Counter-Clockwise");
    }
  }

  // Check to see if the button has been pressed.
  // Passes in a debounce delay of 20 milliseconds
  if (r.buttonPressedReleased(20)) {
    Serial.println("Button pressed");
  }
}


