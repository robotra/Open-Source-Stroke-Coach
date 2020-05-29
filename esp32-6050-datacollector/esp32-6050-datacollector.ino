#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

MPU6050 mpu;


#define INTERRUPT_PIN 15  // using pin 15 on ESP32 because any pin can be an interrupt
#define LED_PIN 2 // (ESP32 is 2, Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;


// Set the variables for SD Cards
#define cardSelect 5 //SD Card pin select
File logfile;
char filename[15];

// Software serial for GPS
// Connect the GPS TX (transmit) pin to Digital 4
// Connect the GPS RX (receive) pin to Digital 2
static const int RXPin = 4, TXPin = 2;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device, using software serial for flexibility in pin assignemnt
SoftwareSerial ss(RXPin, TXPin);


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(21, 22, 400000);
  Wire.setClock(400000);

  pinMode(13, OUTPUT);
  initGPS();

  Serial.begin(115200);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  delay(500);
  mpu.initialize();
  delay(500);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  delay(500);
  devStatus = mpu.dmpInitialize();
  delay(500);

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(79);
  mpu.setYGyroOffset(65);
  mpu.setZGyroOffset(19);
  mpu.setXAccelOffset(-3011);
  mpu.setYAccelOffset(-1833);
  mpu.setZAccelOffset(1030); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);


    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    digitalWrite(LED_PIN, HIGH);
  }
  strcpy(filename, "/ANALOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[7] = '0' + i / 10;
    filename[8] = '0' + i % 10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  File logfile = SD.open(filename, FILE_WRITE);
  if ( ! logfile ) {
    Serial.print("Couldnt create ");
    Serial.println(filename);
  }
  logfile.close();
  Serial.print("Writing to ");
  Serial.println(filename);

  pinMode(8, OUTPUT);
  Serial.println("Ready!");

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    detachInterrupt(INTERRUPT_PIN)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    logfile = SD.open(filename, FILE_APPEND);
    logfile.print(millis()); logfile.print(",");// Raw time in HHMMSSCC format (u32)
    logfile.print(gps.time.value()); logfile.print(",");// Raw time in HHMMSSCC format (u32)
    logfile.print(aaReal.x); logfile.print(",");
    logfile.print(aaReal.y); logfile.print(",");
    logfile.print(aaReal.z); logfile.print(",");
    while (ss.available() > 0)
    {
      gps.encode(ss.read());
    }
    gpsWrite();
    Serial.print(gps.time.value()); Serial.print(",");// Raw time in HHMMSSCC format (u32)
    Serial.print(aaReal.x); Serial.print(",");
    Serial.print(aaReal.y); Serial.print(",");
    Serial.print(aaReal.z); Serial.print(",");

    logfile.println(",");
    Serial.println(",");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    mpu.resetFIFO();
    attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
  }
}

void gpsWrite()
{
  logfile.print(gps.location.lat(), 6);
  logfile.print(",");
  logfile.print(gps.location.lng(), 6);
  logfile.print(",");
  logfile.print(gps.speed.value());
  logfile.print(",");
  logfile.print(gps.course.deg());
  logfile.println(",");
  Serial.print(gps.location.lat(), 6);
  Serial.print(",");
  Serial.print(gps.location.lng(), 6);
  Serial.print(",");
  Serial.print(gps.speed.value());
  Serial.print(",");
  Serial.print(gps.course.deg());
}

void initGPS()
{
  ss.begin(GPSBaud);
  delay(1000);

  // you can send various commands to get it started
  // Sending command for RMCGGA
  ss.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  // Sending command for 5Hz
  ss.println("$PMTK220,200*2C");
}
