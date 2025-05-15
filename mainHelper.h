#ifndef MY_HELPERS_H
#define MY_HELPERS_H

// Libraries
#include <Arduino.h>
#include <IRremote.h>
#include <Servo.h>

// Setup IR
const int RECV_PIN = 11;
IRrecv irrecv(RECV_PIN);
decode_results results;

// Constants
Servo servos[8];

const int homePositions[ 8 ] = { 180, 0, 0, 180, 180, 0, 0, 180};

// Pins
const int servoPins[ 8 ] = {2, 3, 4, 5, 6, 7, 8, 9};

// Function Declorations

void getMotorPositions( int motorPositions[], Servo (&servos)[8] );

void getOperators( int motorPositions[], int destinationPositions[], char operators[] );

void getPositionOffsets( int offsets[], int positions[], int destinationPositions[] );

// Function name: gradualMovement
// Input: initial value, increment value, servo
// Output: Gradual movement (No sudden movement that will brown the board)
// Process: This will take the initial value and slowly increment one and write to the servo
void gradualMovement( int initialValue, int incrementValue, int offset, Servo &servo, char op ); // op == operator

// Function name: homePosition
// Input: all the servos for the legs      U   L  U   L    U   L  U   L
// Output: Movement to the home position (180, 0, 0, 180, 180, 0, 0, 180)
// Process: This will call the moveRobot function with the above movements
void homePosition( Servo (&servos)[8], bool starting );

// Function name: MoveRobot
// Input: - Array in the following order with a upper and lower for each: FL, FR, BL, BR
//        - Next is an array of changes in degrees (e.x. 0, 20, 0, 20, 0, 20, 0, 20)
//        - Lastly, an array of +/- for either positive or negative diretion for the motors
// Output: Modularized movement
// Process: I will have one for loop going through each of the three arrays, adding or subtracting each degree value.
//   Once I have the value to write I will send the values to a gradual movement function to slowly move the piece.
bool moveRobot( Servo (&servos)[8], int offsets[], char operators[], int motorPositions[] );

#endif
