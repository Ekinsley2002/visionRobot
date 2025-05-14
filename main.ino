// Welcome to my quadrupled MKII's controller program
// There is (hopefully) an attatched read me with doumentation of how this works!
// Furthermore, all of my coding sessions are logged in a text document as well. Enjoy!

#include "mainHelper.h"


void setup() {
  // begin serial reading
  Serial.begin(9600);
  irrecv.enableIRIn();

  // attatch each servo to the pins
  for( int i = 0; i < 8; i++ ) {
        servos[ i ].attach(servoPins[ i] );
  }
 
  //homePosition( servos[] );
}

void loop() {

  if (irrecv.decode(&results)) {
    unsigned long key = results.value;
    Serial.println(key, HEX);  // Use this to find codes for buttons

    switch (key) {
      case 0xFFA25D:  // "0" button
        homePosition( servos );
        break;

      case 0xFF30CF:  // "1" button
        // moveAllServos(lowPosition);
        break;

      case 0xFF18E7:  // "2" button
        // moveAllServos(45);
        break;

      case 0xFF7A85:  // "3" button
        // moveAllServos(135);
        break;

      default:
        Serial.println("Unknown button");
        break;
    }

    irrecv.resume();  // Receive the next code
  }
}

// Function name: gradualMovement
// Input: initial value, increment value, servo
// Output: Gradual movement (No sudden movement that will brown the board)
// Process: This will take the initial value and slowly increment one and write to the servo
bool gradualMovement( int initialValue, int incrementValue, Servo servo ) {

  return false
}

// Function name: homePosition
// Input: all the servos for the legs      U   L  U   L    U   L  U   L
// Output: Movement to the home position (180, 0, 0, 180, 180, 0, 0, 180)
// Process: This writes all the values for home position, if it is the starting occurence then just write
// otherwise, find current positions and gradually step.
void homePosition( Servo servos[], bool starting ) {
                                                                                     
  // check if this is the startup
  if( starting ) {

    for( int i = 0; i < 8; i++ ) {

        servos[i].write(homePositions[i]);
    }
  }

  // otherwise gradual step
  else {
    

  }
}

// Function name: MoveRobot
// Input: - Array in the following order with a upper and lower for each: FL, FR, BL, BR
//        - Next is an array of changes in degrees (e.x. 0, 20, 0, 20, 0, 20, 0, 20)
//        - Lastly, an array of +/- for either positive or negative diretion for the motors
// Output: Modularized movement
// Process: I will have one for loop going through each of the three arrays, adding or subtracting each degree value.
//   Once I have the value to write I will send the values to a gradual movement function to slowly move the piece.
bool moveRobot( Servo servos[], int offsets, char operators[] ) {

  // TODO
  return false;
  }
