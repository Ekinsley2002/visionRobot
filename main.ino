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
 
  // go to home position on startup
  homePosition( servos );

  delay(3000);
}

// main loop of the robot
void loop() {
  
  if( !IrReceiver.decode() ) {
    return;
  }

  // decode the data of the IR remote
  auto &d = IrReceiver.decodedIRData;

  // determine if 0 (home) or 1 (gallop)
  switch( d.command ) {

    case 0x16:

      homePosition( servos );
      break;

    case 0x0c:

      gallopingGait( servos );
      break;
  }

  IrReceiver.resume();
}

void gallopingGait( ServoEasing (&servos)[8] ) {

  // continues walk cycle until home position
  while( 1 ) {

    auto &d = IrReceiver.decodedIRData;

    if( IrReceiver.decode() ) {

      uint8_t cmd = IrReceiver.decodedIRData.command;

      IrReceiver.resume();

      if (cmd == 0x16) {

        // exit back to home
        homePosition(servos);
        return;
      }
    }
    
    // phase 1
    moveRobot( servos, gallopPhase1, motorTimingGallop1 );
    delay(250);

    // phase 2
    moveRobot( servos, gallopPhase2, motorTimingGallop2 );
    delay(250);

    // phase 3
    moveRobot( servos, gallopPhase3, motorTimingGallop3 );
    delay(250);

    // phase 4
    moveRobot( servos, homePositions, motorTimingHomePositon );
    delay(250);

    // phase 5
    moveRobot( servos, gallopPhase4, motorTimingGallop1 );
    delay(250);

    // phase 6
    moveRobot( servos, gallopPhase5, motorTimingGallop4 );
    delay(250);

    // phase 7
    moveRobot( servos, gallopPhase6, motorTimingGallop4 );
    delay(250);

    // phase 8
    moveRobot( servos, homePositions, motorTimingHomePositon );
    delay(250);
  }
}

// Function name: homePosition
// Input: all the servos for the legs      U   L  U   L    U   L  U   L
// Output: Movement to the home position (180, 0, 0, 180, 180, 0, 0, 180)
// Process: Moves the robot to home position
void homePosition( ServoEasing ( &servos )[ 8 ] ) {

  moveRobot( servos, homePositions, motorTimingHomePositon );
}

// Function name: MoveRobot
// Input: - Array in the following order with a upper and lower for each: FL, FR, BL, BR
// Output: Modularized movement, potential to setup LED output
// Process: One loop to set positions, then ease all motors to destinations
void moveRobot( ServoEasing ( &servos )[ 8 ], int destinationPositions[], int timings[] ) {

  for( int i = 0; i < 8; i++ ) {

    servos[i].setEaseTo( destinationPositions[i], timings[i] );
  }

  // move all servos at once
  synchronizeAllServosStartAndWaitForAllServosToStop();
}
