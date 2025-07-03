#ifndef MY_HELPERS_H
#define MY_HELPERS_H

// Libraries
#include <Arduino.h>
#include <IRremote.hpp>
#include <ServoEasing.hpp>
#include "motorUtilities.h"

// Setup IR
const int IR_PIN = 11;
decode_results results;
ServoEasing servos[8];

// Constants

// home position (every upper value is -+20)
const int homePositions[ 8 ] = { 160, 0, 20, 180, 160, 0, 20, 180 };

const int gallopPhase1[ 8 ] = { 140, 0, 20, 180, 160, 0, 40, 180 };

const int gallopPhase2[ 8 ] = { 150, 30, 0, 180, 180, 0, 30, 150 };

const int gallopPhase3[ 8 ] = { 180, 0, 20, 180, 160, 0, 0, 180 };

// opposite side (FR and BL drag)
const int gallopPhase4[ 8 ] = { 160, 0, 40, 180, 140, 0, 20, 180 };

const int gallopPhase5[ 8 ] = { 180, 0, 30, 150, 150, 30, 0, 180 };

const int gallopPhase6[ 8 ] = { 160, 0, 0, 180, 180, 0, 20, 180 };

// timings
const int motorTimingGallop1[ 8 ] = { 400, 400, 400, 400, 400, 400, 400, 400 };

const int motorTimingGallop2[ 8 ] = { 600, 600, 600, 600, 600, 600, 600, 600 };

const int motorTimingGallop3[ 8 ] = { 600, 600, 600, 600, 600, 600, 600, 600 };

const int motorTimingGallop4[ 8 ] = { 600, 600, 600, 600, 600, 600, 600, 600 };

// quicker home positions
const int motorTimingHomePositon[ 8 ] = { 600, 600, 600, 600, 600, 600, 600, 600 };

// Pins
const int servoPins[ 8 ] = { 2, 3, 4, 5, 6, 7, 8, 9 };

// Function Declorations

// Function name: gallopingGait
// Input: servos
// Output: specific instructions on how to move the robot
// Process: This will call ease motor with specific motor positions and timings
void gallopingGait( ServoEasing (&servos)[8] );

// Function name: homePosition
// Input: all the servos for the legs      U   L  U   L    U   L  U   L
// Output: Movement to the home position (180, 0, 0, 180, 180, 0, 0, 180)
// Process: This will call the moveRobot function with the above movements
void homePosition( ServoEasing (&servos)[8] );

// Function name: MoveRobot
// Input: - Array in the following order with a upper and lower for each: FL, FR, BL, BR
//        - Next is an array of changes in degrees (e.x. 0, 20, 0, 20, 0, 20, 0, 20)
//        - Lastly, an array of +/- for either positive or negative diretion for the motors
// Output: Modularized movement
// Process: I will have one for loop going through each of the three arrays, adding or subtracting each degree value.
//   Once I have the value to write I will send the values to a gradual movement function to slowly move the piece.
void moveRobot( ServoEasing (&servos)[8], int destinationPositions[], int timings[] );

#endif
