// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_Wrapper.h"
#include "TogglePin.h"
#include "DeathTimer.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


/* =========================================================================
  NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
  depends on the MPU-6050's INT pin being connected to the Arduino's
  external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
  digital I/O pin 2.
   ========================================================================= */

/* =========================================================================
  NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
  when using Serial.write(buf, len). The Teapot output uses this method.
  The solution requires a modification to the Arduino USBAPI.h file, which
  is fortunately simple, but annoying. This will be fixed in the next IDE
  release. For more info, see these links:

  http://arduino.cc/forum/index.php/topic,109987.0.html
  http://code.google.com/p/arduino/issues/detail?id=958
   ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION
// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER
// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_PITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT


#ifdef OUTPUT_TEAPOT
// Teapot demo can only output from one MPU6050
const bool useSecondMpu = false;
MPU6050_Array mpus(1);
#else
const bool useSecondMpu = true;
MPU6050_Array mpus(useSecondMpu ? 2 : 1);
#endif

#define AD0_PIN_0 4  // Connect this pin to the AD0 pin on MPU #0
#define AD0_PIN_1 5  // Connect this pin to the AD0 pin on MPU #1

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define OUTPUT_SERIAL Serial
//#define OUTPUT_SERIAL Serial2
//#define OUTPUT_SERIAL softSerial


uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

TogglePin activityLed(LED_PIN, 100);
DeathTimer deathTimer(5000L);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);

  while (!Serial)
    ; // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpus.add(AD0_PIN_0);
  if (useSecondMpu) mpus.add(AD0_PIN_1);

  mpus.initialize();

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  if (mpus.testConnection()) {
    Serial.println(F("MPU6050 connection successful"));
  } else {
    mpus.halt(F("MPU6050 connection failed, halting"));
  }

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ; // empty buffer
  while (!Serial.available())
    activityLed.update(); // flash led while waiting for data
  while (Serial.available() && Serial.read())
    ; // empty buffer again
  activityLed.setPeriod(500); // slow down led to 2Hz

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  mpus.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  MPU6050_Wrapper* currentMPU = mpus.select(0);
  currentMPU->_mpu.setXGyroOffset(220);
  currentMPU->_mpu.setYGyroOffset(76);
  currentMPU->_mpu.setZGyroOffset(-85);
  currentMPU->_mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  if (useSecondMpu) {
    currentMPU = mpus.select(1);
    currentMPU->_mpu.setXGyroOffset(220);
    currentMPU->_mpu.setYGyroOffset(76);
    currentMPU->_mpu.setZGyroOffset(-85);
    currentMPU->_mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  }
  mpus.programDmp(0);
  if (useSecondMpu)
    mpus.programDmp(1);
}


void handleMPUevent(uint8_t mpu) {

  MPU6050_Wrapper* currentMPU = mpus.select(mpu);
  // reset interrupt flag and get INT_STATUS byte
  currentMPU->getIntStatus();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((currentMPU->_mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT))
      || currentMPU->_fifoCount >= 1024) {
    // reset so we can continue cleanly
    currentMPU->resetFIFO();
    Serial.println(F("FIFO overflow!"));
    return;
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  if (currentMPU->_mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

    // read and dump a packet if the queue contains more than one
    while (currentMPU->_fifoCount >= 2 * currentMPU->_packetSize) {
      // read and dump one sample
      Serial.print("DUMP"); // this trace will be removed soon
      currentMPU->getFIFOBytes(fifoBuffer);
    }

    // read a packet from FIFO
    currentMPU->getFIFOBytes(fifoBuffer);

#ifdef OUTPUT_READABLE_QUATERNION
    // display quaternion values in easy matrix form: w x y z
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    OUTPUT_SERIAL.print("quat:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(q.w);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(q.x);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(q.y);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
    // display Euler angles in degrees
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetEuler(euler, &q);
    OUTPUT_SERIAL.print("euler:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(euler[0] * 180 / M_PI);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(euler[1] * 180 / M_PI);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.println(euler[2] * 180 / M_PI);
#endif

#if defined(OUTPUT_READABLE_YAWPITCHROLL) or defined(OUTPUT_READABLE_PITCHROLL)
    // display Euler angles in degrees
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetGravity(&gravity, &q);
    currentMPU->_mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if defined(OUTPUT_READABLE_YAWPITCHROLL)
    OUTPUT_SERIAL.print("y");
#endif
    OUTPUT_SERIAL.print("pr:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
#if defined(OUTPUT_READABLE_YAWPITCHROLL)
    OUTPUT_SERIAL.print(ypr[0] * 180 / M_PI);
    OUTPUT_SERIAL.print("\t");
#endif
    OUTPUT_SERIAL.print(ypr[1] * 180 / M_PI);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.println(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetAccel(&aa, fifoBuffer);
    currentMPU->_mpu.dmpGetGravity(&gravity, &q);
    currentMPU->_mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    OUTPUT_SERIAL.print("areal:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(aaReal.x);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(aaReal.y);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    currentMPU->_mpu.dmpGetQuaternion(&q, fifoBuffer);
    currentMPU->_mpu.dmpGetAccel(&aa, fifoBuffer);
    currentMPU->_mpu.dmpGetGravity(&gravity, &q);
    currentMPU->_mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    currentMPU->_mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    OUTPUT_SERIAL.print("aworld:"); OUTPUT_SERIAL.print(mpu); OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(aaWorld.x);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.print(aaWorld.y);
    OUTPUT_SERIAL.print("\t");
    OUTPUT_SERIAL.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
    // display quaternion values in InvenSense Teapot demo format:
    // Note that this does not differentialte between your mpus
    teapotPacket[2] = fifoBuffer[0];
    teapotPacket[3] = fifoBuffer[1];
    teapotPacket[4] = fifoBuffer[4];
    teapotPacket[5] = fifoBuffer[5];
    teapotPacket[6] = fifoBuffer[8];
    teapotPacket[7] = fifoBuffer[9];
    teapotPacket[8] = fifoBuffer[12];
    teapotPacket[9] = fifoBuffer[13];
    OUTPUT_SERIAL.write(teapotPacket, 14);
    teapotPacket[11]++;// packetCount, loops at 0xFF on purpose
#endif

  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  static uint8_t mpu = 0;
  static MPU6050_Wrapper* currentMPU = NULL;
  if (useSecondMpu) {
    for (int i=0;i<2;i++) {
      mpu=(mpu+1)%2; // failed attempt at round robin
      currentMPU = mpus.select(mpu);
      if (currentMPU->isDue()) {
        handleMPUevent(mpu);
      }
    }
  } else {
    mpu=0;
    currentMPU = mpus.select(mpu);
    if (currentMPU->isDue()) {
      handleMPUevent(mpu);
    }
  }
  
  // other program behavior stuff here
  // .
  // .
  // .
  // if you are really paranoid you can frequently test in between other
  // stuff to see if mpuInterrupt is true, and if so, "break;" from the
  // while() loop to immediately process the MPU data
  // .
  // .
  // .

  activityLed.update();
  deathTimer.update();
}

