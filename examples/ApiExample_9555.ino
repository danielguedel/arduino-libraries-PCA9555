/*
   Copyright (C) 2021 Daniel Guedel

   This file is subject to the terms and conditions of the GNU Lesser
   General Public License v2.1. See the file LICENSE in the top level
   directory for more details.
*/

#define _DEBUG

#ifdef _DEBUG
  #define _CONSOLE
  #define BAUDRATE 115200
#else
  #undef _CONSOLE
#endif

#include "Arduino.h"
#include "PCA9555.h"

#define PCA9555_INTERRUPT_PIN 11 // PCA9555 interrupt on pin ~D11 (INT0)
#define PCA9555_ADDRESS 0x27 // device address of PCA9555

PCA9555 pca9555 = PCA9555(); // instance of PCA9555 without interrupt

void setup() {
  // pin initialization
  pinMode(PCA9555_INTERRUPT_PIN, INPUT_PULLUP); // pin initialization for PCA9555 interrupt input with internal pull-up resistor enabled
  attachInterrupt(digitalPinToInterrupt(PCA9555_INTERRUPT_PIN), PCA9555_interruptHandler, FALLING); // enable interrupt handler for PCA9555 interrupt with falling edge trigger

  // initialize serial communication
#ifdef _CONSOLE
  Serial.begin(BAUDRATE); // open serial communications with defined baudrate
  // loop not required for debug console only
  while (!Serial) {
   ; // wait until port is open (only necessary for native USB port)
  }
  delay(500);
#endif

  // initialize the PCA9555
  pca9555.begin(PCA9555_ADDRESS, &Wire); // initialization of the PCA9555
  for (uint8_t i = 0; i < 14; i++){
    pca9555.pinMode(i, INPUT); // configure all pins as input
  }

#ifdef _CONSOLE
  Serial.println("\nStarting application...");
  Serial.println("Application successfully started, all tasks are running");
#endif
}

void loop() {
}

void PCA9555_interruptHandler() {
#ifdef _DEBUG
  Serial.println("PCA9555 interrupt has occured");
#endif
}
