// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 2016-05-14 github.com/eadf
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2016-05-14 - First revision

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012, 2016 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/


#ifndef MPU6050_WRAPPER_H
#define MPU6050_WRAPPER_H

#define MPU6050_DMP_FIFO_RATE_DIVISOR 9  // Set FIFO rate to 20Hz (200/(9+1))
#include "MPU6050_6Axis_MotionApps20.h"

class MPU6050_Wrapper {
  typedef void (*interrupt_t)();
  
  friend class MPU6050_Array;
  public:
    MPU6050_Wrapper(uint8_t interruptPin, uint8_t ad0Pin, interrupt_t interrupt) : _mpu(MPU6050_ADDRESS_AD0_HIGH), _interruptPin(interruptPin), _ad0Pin(ad0Pin), _interrupt(interrupt)  {
      pinMode(_interruptPin, INPUT_PULLUP);
      pinMode(_ad0Pin, OUTPUT);
      digitalWrite(_ad0Pin, LOW);
    }

    uint8_t dmpInitialize() {
      return _devStatus = _mpu.dmpInitialize();
    }

    uint8_t getIntStatus() {
      return _mpuIntStatus = _mpu.getIntStatus();
    }

    uint16_t dmpGetFIFOPacketSize() {
      return _packetSize = _mpu.dmpGetFIFOPacketSize();
    }

    uint16_t getFIFOCount() {
      //Serial.print(F(" getFIFOCount on _ad0Pin:")); Serial.println(_ad0Pin);
      return _fifoCount = _mpu.getFIFOCount();
    }

    uint16_t resetFIFO() {
      _mpu.resetFIFO();
      return getFIFOCount();
    }

    void getFIFOBytes(uint8_t* fifoBuffer) {
      _mpu.getFIFOBytes(fifoBuffer, _packetSize);
      _fifoCount -= _packetSize;
    }

 private:
    void enable(bool status) {
      //Serial.print(F(" Setting AD0 pin ")); Serial.print(_ad0Pin); Serial.print(F(" to ")); Serial.println( status ? F("on") : F("off"));
      digitalWrite(_ad0Pin, status);
    }
    
public:
    MPU6050 _mpu;
    uint8_t _interruptPin;
    interrupt_t _interrupt;
    uint8_t _ad0Pin;
    uint8_t _mpuIntStatus = 0; // holds actual interrupt status byte from MPU
    uint8_t _devStatus = 0;    // return status after each device operation (0 = success, !0 = error)
    uint8_t _packetSize = 0;   // expected DMP packet size (default is 42 bytes)
    uint16_t _fifoCount = 0;   // count of all bytes currently in FIFO
};

class MPU6050_Array {
  public:
    MPU6050_Array(uint8_t size): _size(size) {
      _array = (MPU6050_Wrapper**) malloc(sizeof(MPU6050_Wrapper*) * _size);
    }

    void add(uint8_t interruptPin, uint8_t ad0Pin, void (interrupt)()) {
      if (_fillIndex >= _size) halt(F("MPU6050_Array::add() : overflow"));
      _array[_fillIndex++] = new MPU6050_Wrapper(interruptPin, ad0Pin, interrupt);
    }

    MPU6050_Wrapper* select(uint8_t device) {
      if (_currentIndex == device)
        return getCurrent();
      for (int i = 0; i < _fillIndex; i++)
        _array[i]->enable(i == device);
      _currentIndex = device;
      // give the IMU some time to realize that the AD0 pin is altered
      delay(4);
      return getCurrent();
    }

    inline MPU6050_Wrapper* getCurrent() {
      return _array[_currentIndex];
    }

    void initialize() {
      for (int i = 0; i < _fillIndex; i++) {
        select(i);
        _array[i]->_mpu.initialize();
      }
    }

    bool testConnection() {
      for (int i = 0; i < _fillIndex; i++) {
        select(i);
        if (!_array[i]->_mpu.testConnection())
          return false;
      }
      return true;
    }

    void dmpInitialize() {
      for (int i = 0; i < _fillIndex; i++) {
        select(i);
        _array[i]->dmpInitialize();
      }
    }

    bool programDmp(uint8_t mpu) {
      MPU6050_Wrapper* currentMPU = select(mpu);
      if (currentMPU->_devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        currentMPU->_mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(currentMPU->_interruptPin));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(currentMPU->_interruptPin), currentMPU->_interrupt, RISING);
        currentMPU->getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));

        // get expected DMP packet size for later comparison
        currentMPU->dmpGetFIFOPacketSize();
        if (currentMPU->_devStatus) {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          Serial.print(F("DMP: ")); Serial.print(mpu);
          Serial.print(F(" Initialization failed (code "));
          Serial.print(currentMPU->_devStatus);
          halt(F(") Halting.."));
        } else {
          return currentMPU->_devStatus;
        }
      } else {
        Serial.print(F("MPU6050 error code "));
        Serial.print(currentMPU->_devStatus);
        halt(F(" Halting.."));
      }
    }
    
    void halt(const __FlashStringHelper* errMessage = NULL) {
      if (errMessage)
        Serial.println(errMessage);
      while (true)
        ;
    }

  private:
    MPU6050_Wrapper** _array = NULL;
    uint8_t _fillIndex = 0; // where add() will place next mpu*
    int8_t _currentIndex = -1;
    int8_t _size = -1;
};

#endif
