//------------------------------------------------------------------------------
//
//  Description: This file contains the function for prototypes
//
//  Reewaj Adhikari
//  Sept 2025
//  Built with Code Composer Version: CCS12.8.1
//
//------------------------------------------------------------------------------
#include "msp430.h"
#include "macros.h"
#include "functions.h"


void Init_LEDs(void){
//------------------------------------------------------------------------------
// LED Configurations
//------------------------------------------------------------------------------
// Turns on both LEDs
  P1OUT &= ~RED_LED;
  P6OUT &= ~GRN_LED;
//------------------------------------------------------------------------------
}
