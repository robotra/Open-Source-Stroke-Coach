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
P 1450 1400
F 0 "U1" H 1450 1642 50  0000 C CNN
F 1 "MCP1700-3302E_SOT23" H 1450 1551 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1450 1625 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 1450 1400 50  0001 C CNN
	1    1450 1400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5ED29767
P 1450 1700
F 0 "#PWR0101" H 1450 1450 50  0001 C CNN
F 1 "GND" H 1455 1527 50  0000 C CNN
F 2 "" H 1450 1700 50  0001 C CNN
F 3 "" H 1450 1700 50  0001 C CNN
	1    1450 1700
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR0103
U 1 1 5ED2A0AC
P 1150 1400
F 0 "#PWR0103" H 1150 1250 50  0001 C CNN
F 1 "+BATT" V 1165 1527 50  0000 L CNN
F 2 "" H 1150 1400 50  0001 C CNN
F 3 "" H 1150 1400 50  0001 C CNN
	1    1150 1400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 1400 2100 1400
Connection ~ 3150 1400
Wire Wire Line
	5600 2050 5600 1400
Wire Wire Line
	4800 2450 4250 2450
Wire Wire Line
	4250 2450 4250 3000
Wire Wire Line
	4250 3000 3750 3000
$Comp
L Sensor_Motion:MPU-6050 U3
U 1 1 5ED2C978
P 5500 2750
F 0 "U3" H 5500 1961 50  0000 C CNN
F 1 "MPU-6050" H 5750 2200 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm" H 5500 1950 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 5500 2600 50  0001 C CNN
	1    5500 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 2550 4300 2550
Wire Wire Line
	4300 2550 4300 3100
Wire Wire Line
	4300 3100 3750 3100
Wire Wire Line
	6200 2450 6200 1950
Wire Wire Line
	6200 1950 4050 1950
Wire Wire Line
	4050 1950 4050 2500
Wire Wire Line
	4050 2500 3750 2500
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
L power:+BATT #PWR0105
U 1 1 5ED2D12D
P 950 2300
F 0 "#PWR0105" H 950 2150 50  0001 C CNN
F 1 "+BATT" V 965 2428 50  0000 L CNN
F 2 "" H 950 2300 50  0001 C CNN
F 3 "" H 950 2300 50  0001 C CNN
	1    950  2300
	0    1    1    0   
$EndComp
$Comp
L OSSC-eagle-import:GND #0101
U 1 1 5ED2DFDC
P 950 2500
F 0 "#0101" H 1000 2550 50  0001 C CNN
F 1 "GND" H 1028 2538 42  0000 L CNN
F 2 "" H 950 2500 50  0001 C CNN
F 3 "" H 950 2500 50  0001 C CNN
	1    950  2500
	1    0    0    -1  
$EndComp
Text Label 5600 2050 0    50   ~ 0
3.3V
$Comp
L RF_Module:ESP32-WROOM-32 U2
U 1 1 5ED2796E
P 3150 2800
F 0 "U2" H 3150 4381 50  0000 C CNN
F 1 "ESP32-WROOM-32" H 2750 4300 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 3150 1300 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf" H 2850 2850 50  0001 C CNN
	1    3150 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 1400 5450 1400
$Comp
L power:GND #PWR0104
U 1 1 5ED2E0F5
P 3150 4200
F 0 "#PWR0104" H 3150 3950 50  0001 C CNN
F 1 "GND" H 3155 4027 50  0000 C CNN
F 2 "" H 3150 4200 50  0001 C CNN
F 3 "" H 3150 4200 50  0001 C CNN
	1    3150 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5ED353E5
P 5450 1100
F 0 "#PWR01" H 5450 850 50  0001 C CNN
F 1 "GND" H 5455 927 50  0000 C CNN
F 2 "" H 5450 1100 50  0001 C CNN
F 3 "" H 5450 1100 50  0001 C CNN
	1    5450 1100
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5ED353EB
P 5450 1250
F 0 "C2" H 5565 1296 50  0000 L CNN
F 1 "0.1mu" H 5565 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5488 1100 50  0001 C CNN
F 3 "~" H 5450 1250 50  0001 C CNN
	1    5450 1250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5EDA47BE
P 5500 3450
F 0 "#PWR0102" H 5500 3200 50  0001 C CNN
F 1 "GND" H 5505 3277 50  0000 C CNN
F 2 "" H 5500 3450 50  0001 C CNN
F 3 "" H 5500 3450 50  0001 C CNN
	1    5500 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5ED8F6BF
P 2100 1950
F 0 "C1" H 2215 1996 50  0000 L CNN
F 1 "0.1mu" H 2215 1905 50  0000 L CNN
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
Connection ~ 2100 1400
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
L power:GND #PWR0106
U 1 1 5ED93DF9
P 2100 2100
F 0 "#PWR0106" H 2100 1850 50  0001 C CNN
F 1 "GND" H 2105 1927 50  0000 C CNN
F 2 "" H 2100 2100 50  0001 C CNN
F 3 "" H 2100 2100 50  0001 C CNN
	1    2100 2100
	1    0    0    -1  
$EndComp
Connection ~ 5450 1400
Wire Wire Line
	3150 1400 5450 1400
$EndSCHEMATC
