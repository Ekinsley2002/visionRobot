// Welcome to my quadrupled MKII's controller program
// There is (hopefully) an attatched read me with doumentation of how this works!
// Furthermore, all of my coding sessions are logged in a text document as well. Enjoy!

#include "mainHelper.h"


void setup() {
  // begin serial reading
  Serial.begin(115200);
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);

  // attatch each servo to the pins
  for( int i = 0; i < 8; i++ ) {
        servos[ i ].attach( servoPins[ i ] );
  }
 
  homePosition( servos, true );
  delay(8000);
}

void loop() {
  /*
  if( !IrReceiver.decode() ) {
    return;
  }

  uint32_t cmd = IrReceiver.decodedIRData.command;

  switch( cmd ) {
    case 0x10:

      servos[0].write(180);
      break;

    case 0x11:

      servos[0].write(160);
      break;

    default:
      Serial.println(F("→ no action"));
      break;
  }
  IrReceiver.resume();
  */
  delay(250);
  gallopingGait( servos );
  delay(250);
  homePosition( servos, true );
}

void gallopingGait( Servo (&servos)[8] ) {
  /*
  // move the FL and BR lower motors 10
  servos[ 1 ].write(0);
  servos[ 7 ].write(180);

  // move FR and BL upper motors 20
  servos[ 2 ].write(0);
  servos[ 4 ].write(180);

  // move FR and BL lower motors 10
  servos[3].write(180);
  servos[5].write(0);
  */
  for( int i = 0; i < 8; i++ ) {

    servos[i].write(jumpPositions[i]);
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
void gradualMovement( int initialValue, int offset, Servo &servo, char op ) {

  if( op == '+' ) {

    int finalAngle = initialValue + offset;

    for( int i = 0; i < offset; i++ ) {

      servo.write( initialValue + i );
      delay(15);
    }
  servo.write(finalAngle);
  }
  else if( op == '-' ) {

    int finalAngle = initialValue - offset;

    for( int i = 0; i < offset; i++  ) {

      servo.write( initialValue - i );
      delay(15);
    }

    servo.write(finalAngle);
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
void moveRobot( Servo (&servos)[8], int offsets[], char operators[], int motorPositions[] ) {

  for( int i = 0; i < 2; i++ ) {

    gradualMovement( motorPositions[ i ], offsets[ i ], servos[ i ], operators[ i ] );
  }
  return false;
  }

       /*
        int motorPositions[8];
        getMotorPositions(motorPositions, servos);

        // 2) use your hard-coded offsets & ops
        int offsets[8] = {10,10,10,10,10,10,10,10};
        char ops[8]    = {'-','+','-','+','-','+','-','+'};

        // 3) actually move
        moveRobot(servos, offsets, ops, motorPositions);  // only 2 servos for test*/

