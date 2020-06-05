EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Regulator_Linear:MCP1700-3302E_SOT23 U1
U 1 1 5ED289C5
P 1100 1150
F 0 "U1" H 1100 1392 50  0000 C CNN
F 1 "MCP1700-3302E_SOT23" H 1100 1301 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1100 1375 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 1100 1150 50  0001 C CNN
	1    1100 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5ED29767
P 1100 1450
F 0 "#PWR03" H 1100 1200 50  0001 C CNN
F 1 "GND" H 1105 1277 50  0000 C CNN
F 2 "" H 1100 1450 50  0001 C CNN
F 3 "" H 1100 1450 50  0001 C CNN
	1    1100 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR01
U 1 1 5ED2A0AC
P 800 1150
F 0 "#PWR01" H 800 1000 50  0001 C CNN
F 1 "+BATT" V 815 1277 50  0000 L CNN
F 2 "" H 800 1150 50  0001 C CNN
F 3 "" H 800 1150 50  0001 C CNN
	1    800  1150
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x02_Female J1
U 1 1 5ED2BC1D
P 750 2400
F 0 "J1" H 642 2075 50  0000 C CNN
F 1 "Conn_01x02_Female" H 642 2166 50  0000 C CNN
F 2 "Connector_JST:JST_SH_SM02B-SRSS-TB_1x02-1MP_P1.00mm_Horizontal" H 750 2400 50  0001 C CNN
F 3 "~" H 750 2400 50  0001 C CNN
	1    750  2400
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR02
U 1 1 5ED2D12D
P 950 2300
F 0 "#PWR02" H 950 2150 50  0001 C CNN
F 1 "+BATT" V 965 2428 50  0000 L CNN
F 2 "" H 950 2300 50  0001 C CNN
F 3 "" H 950 2300 50  0001 C CNN
	1    950  2300
	0    1    1    0   
$EndComp
$Comp
L OSSC-eagle-import:GND #01
U 1 1 5ED2DFDC
P 950 2500
F 0 "#01" H 1000 2550 50  0001 C CNN
F 1 "GND" H 1028 2538 42  0000 L CNN
F 2 "" H 950 2500 50  0001 C CNN
F 3 "" H 950 2500 50  0001 C CNN
	1    950  2500
	1    0    0    -1  
$EndComp
Text Label 6550 2000 0    50   ~ 0
3.3V
$Comp
L RF_Module:ESP32-WROOM-32 U2
U 1 1 5ED2796E
P 3150 2800
F 0 "U2" H 3650 4300 50  0000 C CNN
F 1 "ESP32-WROOM-32" H 3600 4150 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 3150 1300 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf" H 2850 2850 50  0001 C CNN
	1    3150 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5ED2E0F5
P 3150 4200
F 0 "#PWR05" H 3150 3950 50  0001 C CNN
F 1 "GND" H 3155 4027 50  0000 C CNN
F 2 "" H 3150 4200 50  0001 C CNN
F 3 "" H 3150 4200 50  0001 C CNN
	1    3150 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5ED353E5
P 6850 1850
F 0 "#PWR08" H 6850 1600 50  0001 C CNN
F 1 "GND" H 6855 1677 50  0000 C CNN
F 2 "" H 6850 1850 50  0001 C CNN
F 3 "" H 6850 1850 50  0001 C CNN
	1    6850 1850
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C3
U 1 1 5ED353EB
P 6700 1850
F 0 "C3" H 6815 1896 50  0000 L CNN
F 1 "0.1uF" H 6815 1805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6738 1700 50  0001 C CNN
F 3 "~" H 6700 1850 50  0001 C CNN
	1    6700 1850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5EDA47BE
P 6450 3400
F 0 "#PWR07" H 6450 3150 50  0001 C CNN
F 1 "GND" H 6455 3227 50  0000 C CNN
F 2 "" H 6450 3400 50  0001 C CNN
F 3 "" H 6450 3400 50  0001 C CNN
	1    6450 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5ED8F6BF
P 2100 1950
F 0 "C1" H 2215 1996 50  0000 L CNN
F 1 "0.1uF" H 2215 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2138 1800 50  0001 C CNN
F 3 "~" H 2100 1950 50  0001 C CNN
	1    2100 1950
	-1   0    0    1   
$EndComp
$Comp
L OSSC-eagle-import:RESISTOR0402 R1
U 1 1 5ED90842
P 2100 1600
F 0 "R1" V 2146 1522 50  0000 R CNN
F 1 "10k" V 2055 1522 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2100 1600 50  0001 C CNN
F 3 "" H 2100 1600 50  0001 C CNN
	1    2100 1600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2100 1400 3150 1400
Wire Wire Line
	2550 1600 2350 1600
Wire Wire Line
	2350 1600 2350 1800
Wire Wire Line
	2350 1800 2100 1800
Connection ~ 2100 1800
$Comp
L power:GND #PWR04
U 1 1 5ED93DF9
P 2100 2100
F 0 "#PWR04" H 2100 1850 50  0001 C CNN
F 1 "GND" H 2105 1927 50  0000 C CNN
F 2 "" H 2100 2100 50  0001 C CNN
F 3 "" H 2100 2100 50  0001 C CNN
	1    2100 2100
	1    0    0    -1  
$EndComp
Text GLabel 3750 3100 2    50   Output ~ 0
I2C-SCL
Text GLabel 5750 2500 0    50   Input ~ 0
I2C-SCL
Text GLabel 5750 2400 0    50   Input ~ 0
I2C-SDA
Text GLabel 3750 3000 2    50   Output ~ 0
I2C-SDA
Text GLabel 3750 2500 2    50   Output ~ 0
MPU-6050-INT
Text GLabel 7150 2400 2    50   Input ~ 0
MPU-6050-INT
$Comp
L Sensor_Motion:MPU-6050 U3
U 1 1 5ED2C978
P 6450 2700
F 0 "U3" H 6450 1911 50  0000 C CNN
F 1 "MPU-6050" H 6700 2150 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm" H 6450 1900 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 6450 2550 50  0001 C CNN
	1    6450 2700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5EDA2E9B
P 3550 5250
F 0 "J2" H 3630 5242 50  0000 L CNN
F 1 "Conn_01x04" H 3630 5151 50  0000 L CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_1x04_P1.27mm_Vertical" H 3550 5250 50  0001 C CNN
F 3 "~" H 3550 5250 50  0001 C CNN
	1    3550 5250
	1    0    0    -1  
$EndComp
Text GLabel 3350 5150 0    50   Input ~ 0
VCC-3.3V
Text GLabel 3350 5250 0    50   Input ~ 0
GND
Text GLabel 3350 5350 0    50   Input ~ 0
I2C-SCL
Text GLabel 3350 5450 0    50   Input ~ 0
I2C-SDA
Text GLabel 1400 1150 2    50   Output ~ 0
VCC-3.3V
Text GLabel 6550 1850 1    50   Input ~ 0
VCC-3.3V
Text GLabel 3150 1400 1    50   Input ~ 0
VCC-3.3V
Text Notes 2850 5600 0    50   ~ 0
I2C-Connector for Display
$Comp
L Device:C C5
U 1 1 5EDAE547
P 7350 3150
F 0 "C5" H 7465 3196 50  0000 L CNN
F 1 "0.1uF" H 7465 3105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7388 3000 50  0001 C CNN
F 3 "~" H 7350 3150 50  0001 C CNN
	1    7350 3150
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5EDAEF87
P 7350 3300
F 0 "#PWR09" H 7350 3050 50  0001 C CNN
F 1 "GND" H 7355 3127 50  0000 C CNN
F 2 "" H 7350 3300 50  0001 C CNN
F 3 "" H 7350 3300 50  0001 C CNN
	1    7350 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 3000 7150 3000
$Comp
L power:GND #PWR06
U 1 1 5EDB00DB
P 5750 2900
F 0 "#PWR06" H 5750 2650 50  0001 C CNN
F 1 "GND" H 5755 2727 50  0000 C CNN
F 2 "" H 5750 2900 50  0001 C CNN
F 3 "" H 5750 2900 50  0001 C CNN
	1    5750 2900
	0    1    1    0   
$EndComp
Wire Wire Line
	5750 2600 5750 2900
Connection ~ 5750 2900
Wire Wire Line
	5750 3000 5750 2900
$Comp
L Device:C C4
U 1 1 5EDB1937
P 7300 2900
F 0 "C4" H 7415 2946 50  0000 L CNN
F 1 "2.3nF" H 7415 2855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7338 2750 50  0001 C CNN
F 3 "~" H 7300 2900 50  0001 C CNN
	1    7300 2900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7450 2900 7450 3300
Wire Wire Line
	7450 3300 7350 3300
Connection ~ 7350 3300
Wire Wire Line
	6550 2000 6550 1850
$Comp
L Device:C C2
U 1 1 5EDBBAD5
P 6200 2000
F 0 "C2" H 6315 2046 50  0000 L CNN
F 1 "2.2nF" H 6315 1955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6238 1850 50  0001 C CNN
F 3 "~" H 6200 2000 50  0001 C CNN
	1    6200 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 1850 6050 1850
Wire Wire Line
	6050 1850 6050 2000
Connection ~ 6550 1850
Text GLabel 6200 4100 0    50   Input ~ 0
I2C-SDA
Text GLabel 6200 4200 0    50   Input ~ 0
I2C-SCL
$Comp
L Device:R R2
U 1 1 5EDBF4B2
P 6350 4100
F 0 "R2" V 6250 4250 50  0000 C CNN
F 1 "10k" V 6234 4100 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6280 4100 50  0001 C CNN
F 3 "~" H 6350 4100 50  0001 C CNN
	1    6350 4100
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5EDBFD58
P 6350 4200
F 0 "R3" V 6143 4200 50  0000 C CNN
F 1 "10k" V 6234 4200 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 6280 4200 50  0001 C CNN
F 3 "~" H 6350 4200 50  0001 C CNN
	1    6350 4200
	0    1    1    0   
$EndComp
Text GLabel 6600 4100 2    50   Input ~ 0
VCC-3.3V
Wire Wire Line
	6600 4100 6500 4100
Wire Wire Line
	6500 4200 6500 4100
Connection ~ 6500 4100
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 5EDD0851
P 3950 1700
F 0 "J3" H 4030 1742 50  0000 L CNN
F 1 "Conn_01x01" H 4030 1651 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 3950 1700 50  0001 C CNN
F 3 "~" H 3950 1700 50  0001 C CNN
	1    3950 1700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 5EDD12E1
P 3950 1900
F 0 "J4" H 4030 1942 50  0000 L CNN
F 1 "Conn_01x01" H 4030 1851 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 3950 1900 50  0001 C CNN
F 3 "~" H 3950 1900 50  0001 C CNN
	1    3950 1900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
