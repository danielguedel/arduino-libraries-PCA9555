/*
 * Copyright (C) 2021 Daniel Guedel
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#ifndef PCA9555_H_
#define PCA9555_H_

#include <Arduino.h>
#include <Wire.h>

// Register definitions (page 6, table 4)
#define REG_INPUT0  0x00 // Input port 0
#define REG_INPUT1  0x01 // Input port 1
#define REG_OUTPUT0 0x02 // Output port 0
#define REG_OUTPUT1 0x03 // Output port 1
#define REG_INVERT0 0x04 // Polarity Inversion port 0
#define REG_INVERT1 0x05 // Polarity Inversion port 1
#define REG_CONFIG0 0x06 // Configuration port 0
#define REG_CONFIG1 0x07 // Configuration port 1

// Names of ports (page 5, table3). Ports IO1_0...IO1_7 are mapped to IO_8...IO_15
enum {
    IO_0, IO_1, IO_2 , IO_3 , IO_4 , IO_5 , IO_6 , IO_7 ,
    IO_8, IO_9, IO_10, IO_11, IO_12, IO_13, IO_14, IO_15
};

class PCA9555 {

/******************************* PUBLIC METHODS *******************************/
public:

    /**
     * Constructor for PCA9555
     *
     * @param interruptPin  Interrupt pin (optional)
     */
    PCA9555(int interruptPin = -1);

    /**
     * Initialization of PCA9532
     *
     * @param deviceAddress I2C address of the PCA9555
     * @param wire          Reference to TwoWire for I2C communication
     */
    void begin(uint8_t deviceAddress, TwoWire *wire);

    /**
     * Configure a given pin to input/output
     *
     * @param pin       Pin to configure
     * @param direction Direction (input/output)
     */
    void pinMode(uint8_t pin, uint8_t direction);

    /**
     * Read the state of a given pin
     *
     * @param pin Pin to read
     */
    uint8_t digitalRead(uint8_t pin);

    /**
     * Write the given state to a given pin
     *
     * @param pin   Pin to write
     * @param value Value to write
     */
    void digitalWrite(uint8_t pin, uint8_t value );

    /**
     * Read the state of a given pin (incoming logic level)
     *
     * @param pin State of pin
     */
    uint8_t stateOfPin(uint8_t pin);

/****************************** PRIVATE METHODS *******************************/
private:

    /**
     * Pointer of class PCA9555
     */
    static PCA9555* instancePointer;

    /**
     * Interrupt subroutine
     * Called when INT pin of PCA9555 is activated
     */
    static void alertISR(void);

    /**
     * Read the state off all pins
     */
    void pinStates();

    /**
     * 16-bit representation of configuration registers
     */
    union {
      struct {
          uint8_t _configurationRegister_low;
          uint8_t _configurationRegister_high;
      };
      uint16_t _configurationRegister;
    };

    /**
     * 16-bit representation of input registers
     */
    union {
      struct {
        uint8_t _inputRegister_low;
        uint8_t _inputRegister_high;
      };
      uint16_t _inputRegister;
    };

    /**
     * 16-bit representation of output registers
     */
    union {
      struct {
        uint8_t _outputRegister_low;
        uint8_t _outputRegister_high;
      };
      uint16_t _outputRegister;
    };

    /**
    * Write data to a register
    *
    * @param registerAddress Register address to write to
    * @param data            Data to write
    */
    void writeReg(uint8_t registerAddress, uint8_t data);

    /**
    * Read data from a register
    *
    * @param registerAddress Register address to read from
     *
     * @return byte read from given registerAddress
     * @return -1 if no byte was available to be read
    */
    uint8_t readReg(uint8_t registerAddress);

    /**
     * I2C address of device.
     */
    uint8_t _deviceAddress;

    /**
     * Object for I2C communication
     */
    TwoWire *_wire;
};
#endif // PCA9555_H_