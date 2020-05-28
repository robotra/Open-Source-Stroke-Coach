#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


//MPU constants
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int LED_BUILTIN = 2;

// Set the pins used
#define cardSelect 5

// Software serial for GPS
// Connect the GPS TX (transmit) pin to Digital 4
// Connect the GPS RX (receive) pin to Digital 2
static const int RXPin = 4, TXPin = 2;
static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// blink out an error code
void error(uint8_t errno) 
{
  while (1) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i = errno; i < 10; i++) {
      delay(200);
    }
  }
}


char filename[15];

void setup() 
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("\r\nAnalog logger test");
  pinMode(13, OUTPUT);
  init6050();
  initGPS();
  //
  pinMode (LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    digitalWrite(LED_BUILTIN, HIGH);
    error(2);
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
    error(3);
  }
  logfile.close();
  Serial.print("Writing to ");
  Serial.println(filename);

  pinMode(8, OUTPUT);
  Serial.println("Ready!");
}

void getAccels()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers

  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void loop() 
{
  digitalWrite(8, HIGH);
  getAccels();
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ); Serial.print(",");
  File logfile;
  logfile = SD.open(filename, FILE_APPEND);
  logfile.print(AcX); logfile.print(",");
  logfile.print(AcY); logfile.print(",");
  logfile.print(AcZ); logfile.print(",");

  while (ss.available() > 0)
  {
    gps.encode(ss.read());
  }

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
  Serial.println(",");

  logfile.close();
  digitalWrite(8, LOW);

  digitalWrite(LED_BUILTIN, LOW);
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH);
}

void init6050() 
{
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
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
