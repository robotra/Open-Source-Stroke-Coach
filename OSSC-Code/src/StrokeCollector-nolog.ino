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
#include <SoftwareSerial.h>
#include <Lpf.h>
#include "SSD1306Wire.h" // legacy: #include "SSD1306.h"
#include "CircularBuffer.h"
#include "SCFont.h"

//initialize MPU object
MPU6050 mpu(0x68);

//ESP32 board constants
#define INTERRUPT_PIN 15 // using pin 15 on ESP32 because any pin can be an interrupt
#define LED_PIN 2        // (ESP32 is 2, Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector

#define BANDWIDTH_HZ 1.0        // 3-dB bandwidth of the filter
#define SAMPLE_TIME_SEC 1e-2    // How often are we upodating the loop? The LPF tracks the sample time internally
#define SIGNAL_FREQUENCY_HZ 0.3 // Our test input signal frequency

LPF lpfx(BANDWIDTH_HZ, IS_BANDWIDTH_HZ);
LPF lpfy(BANDWIDTH_HZ, IS_BANDWIDTH_HZ);
LPF lpfz(BANDWIDTH_HZ, IS_BANDWIDTH_HZ);

SSD1306Wire display(0x3c, SDA, SCL); // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h
int rRate = 0;
String dAx = "";
String dAy = "";
String dAz = "";
String dAv = "";
String dAr = "";

CircularBuffer<int, 1000> aBuffer;
CircularBuffer<int, 100> mBuffer;
CircularBuffer<int, 1000> rBuffer;
float max_v;
float pTime;
float cTime;

int secs = 0;
int pSecs = 0;
String timeStr = "";
int sleepTimer = 0;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(21, 22, 600000);

  pinMode(13, OUTPUT);

  Serial.begin(115200);
  delay(100);
  // initialize device
  mpu.initialize();
  delay(50);
  devStatus = mpu.dmpInitialize();
  delay(50);

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(79);
  mpu.setYGyroOffset(65);
  mpu.setZGyroOffset(19);
  mpu.setXAccelOffset(-3011);
  mpu.setYAccelOffset(-1833);
  mpu.setZAccelOffset(1030); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
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
  }
  else
  {
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

  pinMode(8, OUTPUT);
  Serial.println("Ready!");

  //display initialization
  display.displayOn();
  display.init();
  display.flipScreenVertically();
  display.setFont(SCFont);

  //Setup interrupt on GPIO13
  digitalWrite(GPIO_NUM_13, LOW);
  pinMode(GPIO_NUM_13, INPUT_PULLDOWN);
}

void getTime()
{
  detachInterrupt(INTERRUPT_PIN);
  secs = millis() / 1000;
  String secStr = "";
  if (secs % 60 < 10)
  {
    secStr = "0" + String(secs % 60);
  }
  else
  {
    secStr = String(secs % 60);
  }
  timeStr = String((secs / 60) % 60) + ":" + secStr;
  attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
}

void sleepyTimer()
{
  detachInterrupt(INTERRUPT_PIN);
  if (abs(lpfy.NextValue(aaReal.y)) > 1000 || abs(lpfx.NextValue(aaReal.x)) > 1000)
  {
    pSecs = secs;
    sleepTimer = 0;
  }
  if (sleepTimer > 30)
  {
    sleepRoutine();
  }
  attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
}

void sleepRoutine()
{
  delay(50);
  //Setup logic low wakeup on GPIO 13
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 1); //1 = High, 0 = Low
  //Configure wakeup sources
  display.displayOff();  
  digitalWrite(GPIO_NUM_13, LOW);
  delay(1000);
  esp_deep_sleep_start();
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop()
{

  detachInterrupt(INTERRUPT_PIN);
  detachInterrupt(GPIO_NUM_13);
  // Show initial text
  display.clear();
  getTime();
  display.drawString(32, 4, String(timeStr));
  display.drawString(32, 30, String(dAr));
  display.display();

  int sleepMillis = millis();
  while (digitalRead(GPIO_NUM_13) == HIGH)
  {

    display.clear();
    display.drawString(32, 4, String(millis() - sleepMillis));
    display.display();

    if (millis() - sleepMillis > 3000)
    {
      sleepRoutine();
      break;
    }
  }

  attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
  // ================================================================
  // ===                    Accel Interrupt LOOP                  ===
  // ================================================================

  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet
    detachInterrupt(INTERRUPT_PIN);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    float aaX = lpfx.NextValue(aaReal.x);
    float aaY = lpfy.NextValue(aaReal.y);
    float aaZ = lpfz.NextValue(aaReal.z);

    float aVal = aaY + aaZ;
    //Add filtered accel value to head of buffer
    aBuffer.unshift(aVal);

    //every loop set max accel to zero
    max_v = 1;

    //go through the first 100 elements of the Accel buffer
    for (int i = 0; i < 100; i++)
    {
      // if the accel value in i is the biggest
      if (aBuffer[i] >= max_v)
      {
        //change max_v
        max_v = aBuffer[i];
      }
      //add max value to the max value buffer
      mBuffer.unshift(max_v);
    }

    float vAvg = 1.0;

    //calcualte average of the previous [size of buffer] maximum values
    for (int i = 0; i < 100; i++)
    {
      vAvg += mBuffer[i];
    }
    vAvg = vAvg / 100;

    // if the current acceleration value is greater than 90% of avereage of the previous 100 max values
    // and it has been at least a second since the last measurement,
    if (aVal > 0.9 * vAvg && millis() - pTime > 1000)
    {
      //add rate based on peak to peak to the rate measurement buffer
      rBuffer.unshift(1 / ((millis() - pTime) / 60000));
      //reset previous time
      pTime = millis();
    }

    sleepTimer = secs - pSecs;
    //write values to file every 3 seconds
    if (millis() - rRate >= 3000)
    {
      dAx = String(lpfx.NextValue(aaReal.x));
      dAy = String(lpfy.NextValue(aaReal.y));
      dAz = String(lpfz.NextValue(aaReal.z));
      dAv = String(sqrt(aaX * aaX + aaY * aaY + aaZ * aaZ));
      dAr = String((rBuffer[0] + rBuffer[1] + rBuffer[2]) / 3);
      rRate = millis();
    }

    sleepyTimer();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    mpu.resetFIFO();
    attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);
  }
}

