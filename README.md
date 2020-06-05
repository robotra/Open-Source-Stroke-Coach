# Open-Source-Stroke-Coach
This project aims to replicate some of hte funcationality of the NK StrokeCoach, but at a much lower cost. 

Current hardware requirement: 
* ESP32 
* mpu6050 accelerometer
* MCP1700 Low Dropout voltage regulator
* I2C display

Optional:
* GPS (I used the Adafruit ultimate GPS breakout)

## Software
Stroke detection is based on the values from teh accelerometer, smoothed by a low pass filter. This filtered data is then used to feed a peak detection algorithm, and time between peaks is used to calculate stroke rate. 

## To-do
* GPS
* Save to SD card as GPX file
* Then build out integration to the Strava API
