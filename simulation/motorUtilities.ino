#include "motorUtilities.h"

void getOperators( int motorPositions[], int destinationPositions[], char operators[] ) {

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
