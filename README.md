# Open-Source-Stroke-Coach
This project aims to replicate some of the funcationality of the NK StrokeCoach, but at a much lower cost. 

Current hardware requirement: 
* ESP32 
* mpu6050 accelerometer
* I2C display

## Software
Stroke detection is based on the values from the accelerometer, smoothed by a low pass filter. This filtered data is then used to feed a peak detection algorithm, and time between peaks is used to calculate stroke rate. 

## To-do
* GPS
* Save to SD card as GPX file
* Then build out integration to the Strava API
* BLE HRM connection
* UI
* Minimize scope creep
* PCB
