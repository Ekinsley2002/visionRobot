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
 
  homePosition( servos, true );
}

void loop() {
  int offsets[8] = {10, 10, 10, 10, 10, 10, 10, 10};
  char ops[8] = {'-', '+', '-', '+', '-', '+', '-', '+'};
  if (irrecv.decode(&results)) {
    unsigned long key = results.value;
    Serial.println(key, HEX);  // Use this to find codes for buttons

    switch (key) {
      case 0xFFA25D:  // "0" button
        homePosition( servos, false );
        break;

      case 0xFF30CF:  // "1" button
        moveRobot( servos, offsets, ops, homePositions );
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

void getMotorPositions( int motorPositions[], Servo (&servos)[8] ) {

  // go through each servo and read the current angle
  for( int i = 0; i < 8; i++ ) {

    // save each motor position into the array
    motorPositions[ i ] = servos[ i ].read();
  }
}

void getOperators(int motorPositions[], int destinationPositions[], char operators[]) {

  for( int i = 0; i < 8; ++i )
    {
      operators[i] = (motorPositions[i] > destinationPositions[i]) ? '-' : '+';
    }
}

void getPositionOffsets( int offsets[], int positions[], int destinationPositions[] ) {

  for( int i = 0; i < 8; i++ ) {

    offsets[ i ] = abs( destinationPositions[ i ] - positions[ i ]);
  }
}

// Function name: gradualMovement
// Input: initial value, increment value, servo
// Output: Gradual movement (No sudden movement that will brown the board)
// Process: This will take the initial value and slowly increment one and write to the servo
void gradualMovement( int initialValue, int incrementValue, int offset, Servo &servo, char op ) {

  if( op == '+' ) {
    for( int i = incrementValue; i < offset; i += incrementValue ) {

      servo.write( initialValue + i );
      delay(15);
    }
  }
  else if( op == '-' ) {
    for( int i = incrementValue; i < offset; i += incrementValue ) {

      servo.write( initialValue - i );
      delay(15);
    }
  }
}

// Function name: homePosition
// Input: all the servos for the legs      U   L  U   L    U   L  U   L
// Output: Movement to the home position (180, 0, 0, 180, 180, 0, 0, 180)
// Process: This writes all the values for home position, if it is the starting occurence then just write
// otherwise, find current positions and gradually step.
void homePosition( Servo (&servos)[8], bool starting ) {

  // initialize variables
  int motorPositions[8], homeOffsets[8];
  char ops[8];

  // check if this is the startup
  if( starting ) {

    for( int i = 0; i < 8; i++ ) {

        servos[i].write(homePositions[i]);
    }
  }

  // otherwise gradual step
  else {
    
    // I need to find out what each motors position is currently
    getMotorPositions( motorPositions, servos );

    // I need to find the offset of each current posiition to get to home position
    getPositionOffsets( homeOffsets, motorPositions, homePositions );

    // I need to find out the +/- for each motor
    getOperators( motorPositions, homePositions, ops );

    // now I need to call move robot to gradually move each motor
    moveRobot( servos, homeOffsets, ops, motorPositions );
  }
}

// Function name: MoveRobot
// Input: - Array in the following order with a upper and lower for each: FL, FR, BL, BR
//        - Next is an array of changes in degrees (e.x. 0, 20, 0, 20, 0, 20, 0, 20)
//        - Lastly, an array of +/- for either positive or negative diretion for the motors
// Output: Modularized movement, potential to setup LED output
// Process: I will have one for loop going through each of the three arrays, adding or subtracting each degree value.
//   Once I have the value to write I will send the values to a gradual movement function to slowly move the piece.
bool moveRobot( Servo (&servos)[8], int offsets[], char operators[], int motorPositions[] ) {

  for( int i = 0; i < 2; i++ ) {

    gradualMovement( motorPositions[ i ], 1, offsets[ i ], servos[ i ], operators[ i ] );
  }
  return false;
  }


