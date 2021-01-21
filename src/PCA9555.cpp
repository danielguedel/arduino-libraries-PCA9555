/*
 * Copyright (C) 2021 Daniel Guedel
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include "PCA9555.h"

PCA9555* PCA9555::instancePointer = 0;

/******************************* PUBLIC METHODS *******************************/


    /**
     * Constructor for PCA9555
     *
     * @param interruptPin  Interrupt pin (optional)
     */
PCA9555::PCA9555(int interruptPin) {

  _outputRegister = 0;

  if(interruptPin >= 0)
  {
    instancePointer = this;
    attachInterrupt(digitalPinToInterrupt(interruptPin), PCA9555::alertISR, LOW);
  }
}

    /**
     * Initialization of PCA9532
     *
     * @param deviceAddress I2C address of the PCA9555
     * @param wire          Reference to TwoWire for I2C communication
     */
void PCA9555::begin(uint8_t deviceAddress, TwoWire *wire) {

  _deviceAddress = deviceAddress;

  _wire = wire;
  _wire->begin();
}

    /**
     * Configure a given pin to input/output
     *
     * @param pin       Pin to configure
     * @param direction Direction (input/output)
     */
void PCA9555::pinMode(uint8_t pin, uint8_t direction) {

  if (pin <= 15) {
    if (direction == OUTPUT) {
      _configurationRegister = _configurationRegister & ~(1 << pin);
    }
    else {
      _configurationRegister = _configurationRegister | (1 << pin);
    }
      writeReg(REG_CONFIG0, _configurationRegister_low);
      writeReg(REG_CONFIG1, _configurationRegister_high);
  }
}

    /**
     * Read the state of a given pin
     *
     * @param pin Pin to read
     */
uint8_t PCA9555::digitalRead(uint8_t pin) {

  if (pin > 15 ) {
    return 255;
  }
  else {
    _inputRegister_low  = readReg(REG_INPUT0);
    _inputRegister_high |= readReg(REG_INPUT1) << 8;
    if ((_inputRegister & (1 << pin)) > 0){
      return HIGH;
    }
    else {
      return LOW;
    }
  }
}

    /**
     * Write the given state to a given pin
     *
     * @param pin   Pin to write
     * @param value Value to write
     */
void PCA9555::digitalWrite(uint8_t pin, uint8_t value) {

  if (value > 0) {
    _outputRegister = _outputRegister | (1 << pin);
  }
  else {
    _outputRegister = _outputRegister & ~(1 << pin);
  }
  writeReg(REG_OUTPUT0, _outputRegister_low);
  writeReg(REG_OUTPUT1, _outputRegister_high);
}

    /**
     * Read the state of a given pin (incoming logic level)
     *
     * @param pin State of pin
     */
uint8_t PCA9555::stateOfPin(uint8_t pin){

  if ((_inputRegister & (1 << pin)) > 0){
    return HIGH;
  }
  else {
    return LOW;
  }
}

/****************************** PRIVATE METHODS *******************************/


    /**
     * Interrupt subroutine
     * Called when INT pin of PCA9555 is activated
     */
void PCA9555::alertISR()
{
  if (instancePointer != 0)
  {
    instancePointer->pinStates();
  }
}

    /**
     * Read the state off all pins
     */
void PCA9555::pinStates() {

  _inputRegister_low = readReg(REG_INPUT0);
  _inputRegister_high = readReg(REG_INPUT1);
}

    /**
    * Write data to a register
    *
    * @param registerAddress Register address to write to
    * @param data            Data to write
    */
void PCA9555::writeReg(uint8_t registerAddress, uint8_t data) {

  _wire->beginTransmission(_deviceAddress);
  _wire->write(registerAddress);
  _wire->write(data);
  _wire->endTransmission();
}

    /**
    * Read data from a register
    *
    * @param registerAddress Register address to read from
     *
     * @return byte read from given registerAddress
     * @return -1 if no byte was available to be read
    */

uint8_t PCA9555::readReg(uint8_t registerAddress) {

  _wire->beginTransmission(_deviceAddress);
  _wire->write(registerAddress);
  _wire->endTransmission();

  _wire->requestFrom(_deviceAddress, (uint8_t) 1);

  if (_wire->available() == 1) {
    return _wire->read();
  }

  return -1;
}