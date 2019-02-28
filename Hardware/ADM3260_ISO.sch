EESchema Schematic File Version 4
LIBS:Grove_Monitor-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 7
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
L Openponics:ADM3260 U2
U 1 1 5BF22E41
P 4700 3300
AR Path="/5C67B9C5/5BF22E41" Ref="U2"  Part="1" 
AR Path="/5C75CE04/5BF22E41" Ref="U3"  Part="1" 
AR Path="/5C68641C/5BF22E41" Ref="U4"  Part="1" 
F 0 "U2" H 4700 4167 50  0000 C CNN
F 1 "ADM3260" H 4700 4076 50  0000 C CNN
F 2 "Package_SO:SSOP-20_5.3x7.2mm_P0.65mm" H 4700 2450 50  0001 C CNN
F 3 "http://www.analog.com/media/en/technical-documentation/data-sheets/ADM3053.pdf" H 4400 4000 50  0001 C CNN
	1    4700 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5BF234DD
P 3350 3700
AR Path="/5C67B9C5/5BF234DD" Ref="C1"  Part="1" 
AR Path="/5C75CE04/5BF234DD" Ref="C7"  Part="1" 
AR Path="/5C68641C/5BF234DD" Ref="C13"  Part="1" 
F 0 "C1" H 3465 3746 50  0000 L CNN
F 1 "10uF" H 3465 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3388 3550 50  0001 C CNN
F 3 "~" H 3350 3700 50  0001 C CNN
	1    3350 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5BF23525
P 3750 3700
AR Path="/5C67B9C5/5BF23525" Ref="C2"  Part="1" 
AR Path="/5C75CE04/5BF23525" Ref="C8"  Part="1" 
AR Path="/5C68641C/5BF23525" Ref="C14"  Part="1" 
F 0 "C2" H 3865 3746 50  0000 L CNN
F 1 "0.1uF" H 3865 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3788 3550 50  0001 C CNN
F 3 "~" H 3750 3700 50  0001 C CNN
	1    3750 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5BF235D0
P 4100 4000
AR Path="/5C67B9C5/5BF235D0" Ref="#PWR0113"  Part="1" 
AR Path="/5C75CE04/5BF235D0" Ref="#PWR0119"  Part="1" 
AR Path="/5C68641C/5BF235D0" Ref="#PWR0125"  Part="1" 
F 0 "#PWR0113" H 4100 3750 50  0001 C CNN
F 1 "GND" H 4105 3827 50  0000 C CNN
F 2 "" H 4100 4000 50  0001 C CNN
F 3 "" H 4100 4000 50  0001 C CNN
	1    4100 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 4000 4100 3900
Wire Wire Line
	4100 3500 4200 3500
Wire Wire Line
	4200 3600 4100 3600
Connection ~ 4100 3600
Wire Wire Line
	4100 3600 4100 3500
Wire Wire Line
	4200 3700 4100 3700
Connection ~ 4100 3700
Wire Wire Line
	4100 3700 4100 3600
Wire Wire Line
	4200 3800 4100 3800
Connection ~ 4100 3800
Wire Wire Line
	4100 3800 4100 3700
Wire Wire Line
	4200 3900 4100 3900
Connection ~ 4100 3900
Wire Wire Line
	4100 3900 4100 3800
$Comp
L power:GND #PWR0114
U 1 1 5BF23CE7
P 3750 3900
AR Path="/5C67B9C5/5BF23CE7" Ref="#PWR0114"  Part="1" 
AR Path="/5C75CE04/5BF23CE7" Ref="#PWR0120"  Part="1" 
AR Path="/5C68641C/5BF23CE7" Ref="#PWR0126"  Part="1" 
F 0 "#PWR0114" H 3750 3650 50  0001 C CNN
F 1 "GND" H 3755 3727 50  0000 C CNN
F 2 "" H 3750 3900 50  0001 C CNN
F 3 "" H 3750 3900 50  0001 C CNN
	1    3750 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5BF23CFC
P 3350 3900
AR Path="/5C67B9C5/5BF23CFC" Ref="#PWR0115"  Part="1" 
AR Path="/5C75CE04/5BF23CFC" Ref="#PWR0121"  Part="1" 
AR Path="/5C68641C/5BF23CFC" Ref="#PWR0127"  Part="1" 
F 0 "#PWR0115" H 3350 3650 50  0001 C CNN
F 1 "GND" H 3355 3727 50  0000 C CNN
F 2 "" H 3350 3900 50  0001 C CNN
F 3 "" H 3350 3900 50  0001 C CNN
	1    3350 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 3500 3350 3550
Wire Wire Line
	3750 3500 3750 3550
Wire Wire Line
	3750 3850 3750 3900
Wire Wire Line
	3350 3850 3350 3900
Wire Wire Line
	5200 3600 5300 3600
Wire Wire Line
	5300 3600 5300 3700
Wire Wire Line
	5200 3700 5300 3700
Connection ~ 5300 3700
Wire Wire Line
	5300 3700 5300 3800
Wire Wire Line
	5200 3800 5300 3800
Connection ~ 5300 3800
Wire Wire Line
	5300 3800 5300 3900
Wire Wire Line
	5200 3900 5300 3900
Connection ~ 5300 3900
Wire Wire Line
	5300 3900 5300 4000
$Comp
L Device:R R19
U 1 1 5BF29754
P 5700 2900
AR Path="/5C67B9C5/5BF29754" Ref="R19"  Part="1" 
AR Path="/5C75CE04/5BF29754" Ref="R5"  Part="1" 
AR Path="/5C68641C/5BF29754" Ref="R9"  Part="1" 
F 0 "R19" H 5770 2946 50  0000 L CNN
F 1 "4.7K" H 5770 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5630 2900 50  0001 C CNN
F 3 "~" H 5700 2900 50  0001 C CNN
	1    5700 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R20
U 1 1 5BF2978F
P 6000 2900
AR Path="/5C67B9C5/5BF2978F" Ref="R20"  Part="1" 
AR Path="/5C75CE04/5BF2978F" Ref="R6"  Part="1" 
AR Path="/5C68641C/5BF2978F" Ref="R10"  Part="1" 
F 0 "R20" H 6070 2946 50  0000 L CNN
F 1 "4.7K" H 6070 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5930 2900 50  0001 C CNN
F 3 "~" H 6000 2900 50  0001 C CNN
	1    6000 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 3100 6000 3100
Wire Wire Line
	6000 3050 6000 3100
Connection ~ 6000 3100
Wire Wire Line
	6000 3100 6300 3100
Wire Wire Line
	5200 3200 5700 3200
Wire Wire Line
	5700 3050 5700 3200
Connection ~ 5700 3200
Wire Wire Line
	5700 3200 6300 3200
$Comp
L Device:C C3
U 1 1 5BF2BEE5
P 5600 3700
AR Path="/5C67B9C5/5BF2BEE5" Ref="C3"  Part="1" 
AR Path="/5C75CE04/5BF2BEE5" Ref="C9"  Part="1" 
AR Path="/5C68641C/5BF2BEE5" Ref="C15"  Part="1" 
F 0 "C3" H 5715 3746 50  0000 L CNN
F 1 "10uF" H 5715 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5638 3550 50  0001 C CNN
F 3 "~" H 5600 3700 50  0001 C CNN
	1    5600 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5BF2BF2E
P 6000 3700
AR Path="/5C67B9C5/5BF2BF2E" Ref="C4"  Part="1" 
AR Path="/5C75CE04/5BF2BF2E" Ref="C10"  Part="1" 
AR Path="/5C68641C/5BF2BF2E" Ref="C16"  Part="1" 
F 0 "C4" H 6115 3746 50  0000 L CNN
F 1 "10uF" H 6115 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6038 3550 50  0001 C CNN
F 3 "~" H 6000 3700 50  0001 C CNN
	1    6000 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5BF2BF53
P 6400 3700
AR Path="/5C67B9C5/5BF2BF53" Ref="C5"  Part="1" 
AR Path="/5C75CE04/5BF2BF53" Ref="C11"  Part="1" 
AR Path="/5C68641C/5BF2BF53" Ref="C17"  Part="1" 
F 0 "C5" H 6515 3746 50  0000 L CNN
F 1 "0.1uF" H 6515 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6438 3550 50  0001 C CNN
F 3 "~" H 6400 3700 50  0001 C CNN
	1    6400 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5BF2BF9F
P 6850 3700
AR Path="/5C67B9C5/5BF2BF9F" Ref="C6"  Part="1" 
AR Path="/5C75CE04/5BF2BF9F" Ref="C12"  Part="1" 
AR Path="/5C68641C/5BF2BF9F" Ref="C18"  Part="1" 
F 0 "C6" H 6965 3746 50  0000 L CNN
F 1 "0.1uF" H 6965 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6888 3550 50  0001 C CNN
F 3 "~" H 6850 3700 50  0001 C CNN
	1    6850 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 3500 5600 3550
Wire Wire Line
	6000 3550 6000 3500
Wire Wire Line
	6400 3500 6400 3550
Wire Wire Line
	6850 3500 6850 3550
Wire Wire Line
	6850 3900 6850 3850
Wire Wire Line
	6400 3850 6400 3900
Wire Wire Line
	6000 3900 6000 3850
Wire Wire Line
	5600 3850 5600 3900
Wire Wire Line
	6000 2750 6000 2700
Wire Wire Line
	6000 2700 5700 2700
Wire Wire Line
	5700 2750 5700 2700
Wire Wire Line
	5200 2900 5250 2900
NoConn ~ 4200 3300
NoConn ~ 5200 3300
$Comp
L Device:R R3
U 1 1 5BF3A6FE
P 7250 2350
AR Path="/5C67B9C5/5BF3A6FE" Ref="R3"  Part="1" 
AR Path="/5C75CE04/5BF3A6FE" Ref="R7"  Part="1" 
AR Path="/5C68641C/5BF3A6FE" Ref="R11"  Part="1" 
F 0 "R3" H 7320 2396 50  0000 L CNN
F 1 "16.9K" H 7320 2305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7180 2350 50  0001 C CNN
F 3 "~" H 7250 2350 50  0001 C CNN
	1    7250 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5BF3A72E
P 7250 2750
AR Path="/5C67B9C5/5BF3A72E" Ref="R4"  Part="1" 
AR Path="/5C75CE04/5BF3A72E" Ref="R8"  Part="1" 
AR Path="/5C68641C/5BF3A72E" Ref="R12"  Part="1" 
F 0 "R4" H 7320 2796 50  0000 L CNN
F 1 "10K" H 7320 2705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7180 2750 50  0001 C CNN
F 3 "~" H 7250 2750 50  0001 C CNN
	1    7250 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 2600 7250 2550
Connection ~ 7250 2550
Wire Wire Line
	7250 2550 7150 2550
Wire Wire Line
	7250 2900 7250 2950
Wire Wire Line
	5200 2700 5250 2700
Connection ~ 5700 2700
Wire Wire Line
	5350 2600 5350 2700
Connection ~ 5350 2700
Wire Wire Line
	5350 2700 5700 2700
Wire Wire Line
	5200 2800 5250 2800
Wire Wire Line
	5250 2800 5250 2700
Connection ~ 5250 2700
Wire Wire Line
	5250 2700 5350 2700
Wire Wire Line
	4200 2800 4150 2800
Wire Wire Line
	4150 2800 4150 2700
Connection ~ 4150 2700
Wire Wire Line
	4150 2700 4200 2700
Wire Wire Line
	4050 2600 4050 2700
Wire Wire Line
	4050 2700 4150 2700
Wire Wire Line
	7250 2500 7250 2550
Wire Wire Line
	7250 2150 7250 2200
Text Notes 7400 2600 0    50   ~ 0
Vout = 2.44 + 2.5 Vref = 5V Logic
$Comp
L Openponics:ATLAS_STAMP STP1
U 1 1 5C68755F
P 8800 4050
AR Path="/5C67B9C5/5C68755F" Ref="STP1"  Part="1" 
AR Path="/5C75CE04/5C68755F" Ref="STP2"  Part="1" 
AR Path="/5C68641C/5C68755F" Ref="STP3"  Part="1" 
F 0 "STP1" V 8113 4300 60  0000 C CNN
F 1 "ATLAS_STAMP" V 8219 4300 60  0000 C CNN
F 2 "Openponics_Devices:ATLAS_STAMP_EZO" H 8800 5050 60  0001 C CNN
F 3 "" H 8800 5050 60  0000 C CNN
	1    8800 4050
	0    1    1    0   
$EndComp
$Comp
L Openponics:BNC P1
U 1 1 5C68F076
P 7800 4300
AR Path="/5C67B9C5/5C68F076" Ref="P1"  Part="1" 
AR Path="/5C75CE04/5C68F076" Ref="P2"  Part="1" 
AR Path="/5C68641C/5C68F076" Ref="P3"  Part="1" 
F 0 "P1" V 7717 4401 60  0000 L CNN
F 1 "BNC" V 7808 4401 40  0000 L CNN
F 2 "Openponics_Devices:WL-EB08040502_snk" H 7800 4300 60  0001 C CNN
F 3 "" H 7800 4300 60  0000 C CNN
	1    7800 4300
	0    -1   1    0   
$EndComp
Wire Wire Line
	7800 4150 7800 4050
Wire Wire Line
	7800 4050 8100 4050
Wire Wire Line
	8100 4300 8000 4300
Wire Wire Line
	8000 3700 8000 3800
Wire Wire Line
	8000 3800 8100 3800
Wire Wire Line
	10000 3800 10200 3800
Wire Wire Line
	10200 3800 10200 4500
Wire Wire Line
	10350 4300 10000 4300
Wire Wire Line
	10350 4050 10000 4050
Wire Wire Line
	3450 3200 4200 3200
Wire Wire Line
	3450 3100 4200 3100
Text Notes 3100 2900 0    60   ~ 0
Pull ups exisit on \nmain schematic 
Wire Notes Line
	4700 1750 10950 1750
Wire Notes Line
	10950 1750 10950 5200
Wire Notes Line
	10950 5200 4700 5200
Wire Notes Line
	4700 5200 4700 1750
Text Notes 7350 1700 0    79   ~ 16
ISOLATION ZONE
Text HLabel 3450 3100 0    59   Input ~ 0
SCL*
Text HLabel 3450 3200 0    59   Input ~ 0
SDA*
Text HLabel 5250 4650 0    59   Input ~ 0
V_ISO
Text HLabel 5250 4800 0    59   Input ~ 0
GND_ISO
Wire Wire Line
	5250 4800 5300 4800
Wire Wire Line
	5300 4800 5300 4850
Wire Wire Line
	5250 4650 5300 4650
Wire Wire Line
	5300 4650 5300 4600
Text Label 5350 2600 0    59   ~ 0
V_ISO
Text Label 5600 3500 0    59   ~ 0
V_ISO
Text Label 6000 3500 0    59   ~ 0
V_ISO
Text Label 6400 3500 0    59   ~ 0
V_ISO
Text Label 6850 3500 0    59   ~ 0
V_ISO
Text Label 7250 2150 0    59   ~ 0
V_ISO
Text Label 8000 3700 0    59   ~ 0
V_ISO
Text Label 6850 3900 0    59   ~ 0
GND_ISO
Text Label 6400 3900 0    59   ~ 0
GND_ISO
Text Label 6000 3900 0    59   ~ 0
GND_ISO
Text Label 5600 3900 0    59   ~ 0
GND_ISO
Text Label 5300 4000 0    59   ~ 0
GND_ISO
Text Label 5300 4850 0    59   ~ 0
GND_ISO
Text Label 5300 4600 0    59   ~ 0
V_ISO
Text Label 10200 4500 0    59   ~ 0
GND_ISO
Text Label 7250 2950 0    59   ~ 0
GND_ISO
Text Label 6300 3100 0    60   ~ 0
SCL_ISO
Text Label 6300 3200 0    60   ~ 0
SDA_ISO
Text Label 10350 4300 0    60   ~ 0
SCL_ISO
Text Label 10350 4050 0    60   ~ 0
SDA_ISO
Text Label 5250 2900 0    60   ~ 0
VSEL
Text Label 7150 2550 2    60   ~ 0
VSEL
$Comp
L Device:C C21
U 1 1 5C6D9DF3
P 2500 3700
AR Path="/5C67B9C5/5C6D9DF3" Ref="C21"  Part="1" 
AR Path="/5C75CE04/5C6D9DF3" Ref="C23"  Part="1" 
AR Path="/5C68641C/5C6D9DF3" Ref="C25"  Part="1" 
F 0 "C21" H 2615 3746 50  0000 L CNN
F 1 "10uF" H 2615 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2538 3550 50  0001 C CNN
F 3 "~" H 2500 3700 50  0001 C CNN
	1    2500 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C22
U 1 1 5C6D9DFA
P 2900 3700
AR Path="/5C67B9C5/5C6D9DFA" Ref="C22"  Part="1" 
AR Path="/5C75CE04/5C6D9DFA" Ref="C24"  Part="1" 
AR Path="/5C68641C/5C6D9DFA" Ref="C26"  Part="1" 
F 0 "C22" H 3015 3746 50  0000 L CNN
F 1 "0.1uF" H 3015 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2938 3550 50  0001 C CNN
F 3 "~" H 2900 3700 50  0001 C CNN
	1    2900 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5C6D9E07
P 2900 3900
AR Path="/5C67B9C5/5C6D9E07" Ref="#PWR09"  Part="1" 
AR Path="/5C75CE04/5C6D9E07" Ref="#PWR016"  Part="1" 
AR Path="/5C68641C/5C6D9E07" Ref="#PWR020"  Part="1" 
F 0 "#PWR09" H 2900 3650 50  0001 C CNN
F 1 "GND" H 2905 3727 50  0000 C CNN
F 2 "" H 2900 3900 50  0001 C CNN
F 3 "" H 2900 3900 50  0001 C CNN
	1    2900 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5C6D9E0D
P 2500 3900
AR Path="/5C67B9C5/5C6D9E0D" Ref="#PWR07"  Part="1" 
AR Path="/5C75CE04/5C6D9E0D" Ref="#PWR014"  Part="1" 
AR Path="/5C68641C/5C6D9E0D" Ref="#PWR018"  Part="1" 
F 0 "#PWR07" H 2500 3650 50  0001 C CNN
F 1 "GND" H 2505 3727 50  0000 C CNN
F 2 "" H 2500 3900 50  0001 C CNN
F 3 "" H 2500 3900 50  0001 C CNN
	1    2500 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 3500 2500 3550
Wire Wire Line
	2900 3500 2900 3550
Wire Wire Line
	2900 3850 2900 3900
Wire Wire Line
	2500 3850 2500 3900
$Comp
L power:+3.3V #PWR0112
U 1 1 5C873C09
P 4050 2600
AR Path="/5C67B9C5/5C873C09" Ref="#PWR0112"  Part="1" 
AR Path="/5C75CE04/5C873C09" Ref="#PWR0123"  Part="1" 
AR Path="/5C68641C/5C873C09" Ref="#PWR0146"  Part="1" 
F 0 "#PWR0112" H 4050 2450 50  0001 C CNN
F 1 "+3.3V" H 4065 2773 50  0000 C CNN
F 2 "" H 4050 2600 50  0001 C CNN
F 3 "" H 4050 2600 50  0001 C CNN
	1    4050 2600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0116
U 1 1 5C87535A
P 3750 3500
AR Path="/5C67B9C5/5C87535A" Ref="#PWR0116"  Part="1" 
AR Path="/5C75CE04/5C87535A" Ref="#PWR0124"  Part="1" 
AR Path="/5C68641C/5C87535A" Ref="#PWR0147"  Part="1" 
F 0 "#PWR0116" H 3750 3350 50  0001 C CNN
F 1 "+3.3V" H 3765 3673 50  0000 C CNN
F 2 "" H 3750 3500 50  0001 C CNN
F 3 "" H 3750 3500 50  0001 C CNN
	1    3750 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0117
U 1 1 5C875383
P 3350 3500
AR Path="/5C67B9C5/5C875383" Ref="#PWR0117"  Part="1" 
AR Path="/5C75CE04/5C875383" Ref="#PWR0128"  Part="1" 
AR Path="/5C68641C/5C875383" Ref="#PWR0148"  Part="1" 
F 0 "#PWR0117" H 3350 3350 50  0001 C CNN
F 1 "+3.3V" H 3365 3673 50  0000 C CNN
F 2 "" H 3350 3500 50  0001 C CNN
F 3 "" H 3350 3500 50  0001 C CNN
	1    3350 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0118
U 1 1 5C8753AC
P 2900 3500
AR Path="/5C67B9C5/5C8753AC" Ref="#PWR0118"  Part="1" 
AR Path="/5C75CE04/5C8753AC" Ref="#PWR0129"  Part="1" 
AR Path="/5C68641C/5C8753AC" Ref="#PWR0149"  Part="1" 
F 0 "#PWR0118" H 2900 3350 50  0001 C CNN
F 1 "+3.3V" H 2915 3673 50  0000 C CNN
F 2 "" H 2900 3500 50  0001 C CNN
F 3 "" H 2900 3500 50  0001 C CNN
	1    2900 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0122
U 1 1 5C8753D5
P 2500 3500
AR Path="/5C67B9C5/5C8753D5" Ref="#PWR0122"  Part="1" 
AR Path="/5C75CE04/5C8753D5" Ref="#PWR0145"  Part="1" 
AR Path="/5C68641C/5C8753D5" Ref="#PWR0150"  Part="1" 
F 0 "#PWR0122" H 2500 3350 50  0001 C CNN
F 1 "+3.3V" H 2515 3673 50  0000 C CNN
F 2 "" H 2500 3500 50  0001 C CNN
F 3 "" H 2500 3500 50  0001 C CNN
	1    2500 3500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
