EESchema Schematic File Version 4
LIBS:fan controller-cache
EELAYER 26 0
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
L MCU_Microchip_ATmega:ATmega328P-PU U1
U 1 1 5BFB3CC0
P 2300 3450
F 0 "U1" H 1659 3496 50  0000 R CNN
F 1 "ATmega328P-PU" H 1659 3405 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 2300 3450 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 2300 3450 50  0001 C CNN
	1    2300 3450
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C1
U 1 1 5BFB3DBB
P 1150 3350
F 0 "C1" H 1328 3396 50  0000 L CNN
F 1 "10u" H 1328 3305 50  0000 L CNN
F 2 "" H 1150 3350 50  0001 C CNN
F 3 "~" H 1150 3350 50  0001 C CNN
	1    1150 3350
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C2
U 1 1 5BFB3E7F
P 3200 3700
F 0 "C2" H 3378 3746 50  0000 L CNN
F 1 "0.1u" H 3378 3655 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D10.0mm_W2.5mm_P5.00mm" H 3200 3700 50  0001 C CNN
F 3 "~" H 3200 3700 50  0001 C CNN
	1    3200 3700
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:2N7000 Q1
U 1 1 5BFB3FA6
P 4250 3400
F 0 "Q1" H 4455 3446 50  0000 L CNN
F 1 "2N7000" H 4455 3355 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 4450 3325 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N7000.pdf" H 4250 3400 50  0001 L CNN
	1    4250 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5BFB40CC
P 3600 4150
F 0 "R1" H 3670 4196 50  0000 L CNN
F 1 "6800" H 3670 4105 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P12.70mm_Horizontal" V 3530 4150 50  0001 C CNN
F 3 "~" H 3600 4150 50  0001 C CNN
	1    3600 4150
	1    0    0    -1  
$EndComp
$Comp
L Motor:Fan M1
U 1 1 5BFB4178
P 4350 3000
F 0 "M1" H 4508 3096 50  0000 L CNN
F 1 "Fan" H 4508 3005 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 4350 3010 50  0001 C CNN
F 3 "~" H 4350 3010 50  0001 C CNN
	1    4350 3000
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x05_Counter_Clockwise J1
U 1 1 5BFB4287
P 3700 2200
F 0 "J1" H 3750 2617 50  0000 C CNN
F 1 "Prog" H 3750 2526 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical" H 3700 2200 50  0001 C CNN
F 3 "~" H 3700 2200 50  0001 C CNN
	1    3700 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 3950 3200 4950
Wire Wire Line
	3200 4950 2300 4950
Wire Wire Line
	3200 1950 2400 1950
Wire Wire Line
	2300 1950 2400 1950
Connection ~ 2400 1950
Wire Wire Line
	2300 1950 1150 1950
Wire Wire Line
	1150 1950 1150 3100
Connection ~ 2300 1950
Wire Wire Line
	1150 3600 1150 4950
Wire Wire Line
	1150 4950 2300 4950
Connection ~ 2300 4950
Connection ~ 3200 4950
$Comp
L Device:Thermistor TH1
U 1 1 5BFB48A6
P 3600 3050
F 0 "TH1" H 3705 3096 50  0000 L CNN
F 1 "Thermistor" H 3705 3005 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0414_L11.9mm_D4.5mm_P7.62mm_Vertical" H 3600 3050 50  0001 C CNN
F 3 "~" H 3600 3050 50  0001 C CNN
	1    3600 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1950 3200 2700
Connection ~ 3200 2700
Wire Wire Line
	3200 2700 3200 3450
Wire Wire Line
	3800 3400 3800 4550
Wire Wire Line
	2900 4550 3800 4550
Wire Wire Line
	3600 3250 2900 3250
Wire Wire Line
	3600 2850 3600 2700
Connection ~ 3600 2700
Wire Wire Line
	3600 2700 3200 2700
Wire Wire Line
	3600 4000 3600 3250
Connection ~ 3600 3250
Wire Wire Line
	3600 4300 3600 4950
Connection ~ 3600 4950
Wire Wire Line
	3600 4950 4000 4950
Wire Wire Line
	3500 2000 3000 2000
Wire Wire Line
	3000 2000 3000 2550
Wire Wire Line
	3000 2550 2900 2550
Wire Wire Line
	3500 2100 3450 2100
Wire Wire Line
	3500 2200 2950 2200
Wire Wire Line
	2950 2200 2950 3750
Wire Wire Line
	2950 3750 2900 3750
Wire Wire Line
	3500 2300 3050 2300
Wire Wire Line
	3050 2300 3050 2750
Wire Wire Line
	3050 2750 2900 2750
Wire Wire Line
	3500 2400 3100 2400
Wire Wire Line
	3100 2400 3100 2650
Wire Wire Line
	3100 2650 2900 2650
Wire Wire Line
	4000 2100 4000 2200
Wire Wire Line
	4000 2200 4000 2300
Connection ~ 4000 2200
Wire Wire Line
	4000 2300 4000 2400
Connection ~ 4000 2300
Wire Wire Line
	3200 4950 3600 4950
Wire Wire Line
	4000 2400 4000 4950
Connection ~ 4000 2400
Connection ~ 4000 4950
Wire Wire Line
	3800 3400 4050 3400
Wire Wire Line
	3600 2700 4350 2700
Wire Wire Line
	4350 3600 4350 4950
Wire Wire Line
	4000 4950 4350 4950
$EndSCHEMATC
