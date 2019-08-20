#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include "Wire.h"

MPU6050 mpu;

//mpu interrupt setup
#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//stroke buffer to hold 3 seconds worth of data
uint8_t strokeBuf[300];
uint8_t rollingBuf[50];
uint8_t rateBuf[3];

//MPU orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

const int chipSelect = 4;

//MPU interrupt detection
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  delay(1000);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    mpu.setRate(9);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  mpu.setXAccelOffset(-4196);
  mpu.setYAccelOffset(-451);
  mpu.setZAccelOffset(3511);
  mpu.setXGyroOffset(119);
  mpu.setYGyroOffset(53);
  mpu.setZGyroOffset(-10);


  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}

void loop() {

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
    // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }

  // check for overflow (this should never happen unless our code is too inefficient)
  else if ((mpuIntStatus & 0x01 << (MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x01 << (MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // read a packet from FIFO


    while (fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
    }

    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);


    mpu.setIntEnabled(false);

    for (int i = 1; i < strokeBuf.length(); i++)
    {
      //move all elements in array back one
      strokeBuf[i] = strokeBuf[i-1];
    }

    strokeBuf[0] = sqrt(aaReal.x^2 +aaReal.y^2+aaReal.z^2);




    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    mpu.setIntEnabled(true);
  }
}

}
