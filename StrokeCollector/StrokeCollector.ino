/* Building an ESP32 Based stroke coach, built on JRowbergs I2C library, for 6050 acclerometer interfacing
   https://github.com/jrowberg/i2cdevlib
   An arduino based implementation a low pass filter, from Bill Williams
   https://github.com/Billwilliams1952/Arduino-Cascadable-Low-Pass-Filter
   With another version for the addition of an Adafruit Ultimate GPS module
   https://www.adafruit.com/product/746


*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Lpf.h>
#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"

//initialize MPU object
MPU6050 mpu(0x68);

//ESP32 board constants
#define INTERRUPT_PIN 15  // using pin 15 on ESP32 because any pin can be an interrupt
#define LED_PIN 2 // (ESP32 is 2, Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// Set the variables for SD Cards
//FInal version probably wont need an SD card with the ESP32 due to the large enough internal memory (maybe)
#define cardSelect 5 //SD Card pin select
File logfile;
char filename[15];

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector


#define BANDWIDTH_HZ          1.0     // 3-dB bandwidth of the filter
#define SAMPLE_TIME_SEC       1e-2   // How often are we upodating the loop? The LPF tracks the sample time internally
#define SIGNAL_FREQUENCY_HZ   0.3     // Our test input signal frequency

LPF lpfx(BANDWIDTH_HZ, IS_BANDWIDTH_HZ);
LPF lpfy(BANDWIDTH_HZ, IS_BANDWIDTH_HZ);
LPF lpfz(BANDWIDTH_HZ, IS_BANDWIDTH_HZ);



SSD1306Wire display(0x3c, 21, 22);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h


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
  Wire.begin(21, 22, 600000);


  pinMode(13, OUTPUT);

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

    //Set sample collection rate to 100Hz
    mpu.setRate(9);

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


  //display initialization
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_24);

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    detachInterrupt(INTERRUPT_PIN);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    float aVect = sqrt(aaReal.x * aaReal.x + aaReal.y * aaReal.y + aaReal.z * aaReal.z);

    display.clear();

    // The coordinates define the right end of the text
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 22, String("test"));

    display.display();

    logfile = SD.open(filename, FILE_APPEND);
    logfile.print(millis()); logfile.print(",");// Raw time in HHMMSSCC format (u32)
    logfile.print(lpfx.NextValue(aaReal.x)); logfile.print(",");
    logfile.print(lpfy.NextValue(aaReal.y)); logfile.print(",");
    logfile.print(lpfz.NextValue(aaReal.z)); logfile.print(",");
    logfile.println(",");
    logfile.close();
    
    Serial.print(millis()); Serial.print(",");// Raw time in HHMMSSCC format (u32)
    Serial.print(lpfx.NextValue(aaReal.x)); Serial.print(",");
    Serial.print(lpfy.NextValue(aaReal.y)); Serial.print(",");
    Serial.print(lpfz.NextValue(aaReal.z)); Serial.print(",");
    Serial.print(aVect); Serial.print(",");
    Serial.println(",");

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    mpu.resetFIFO();
    attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
  }
}
