EESchema Schematic File Version 4
LIBS:Long_Distance_Cap_Touch-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 8
Title "Tentacle for Raspberry Pi"
Date "2016-11-04"
Rev "00.04"
Comp "Whitebox Labs"
Comment1 "Dual isolation, I2C Bus Design"
Comment2 "Pedro Martin"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Openponics:ATLAS_STAMP STP401
U 1 1 581D48A8
P 5000 4175
F 0 "STP401" V 5550 3925 60  0000 C CNN
F 1 "ATLAS_STAMP" H 5000 4425 60  0000 C CNN
F 2 "Openponics_Devices:ATLAS_STAMP_EZO" H 5000 5175 60  0001 C CNN
F 3 "~" H 5000 5175 60  0000 C CNN
	1    5000 4175
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR024
U 1 1 581D48AF
P 4250 3875
F 0 "#PWR024" H 4250 3835 30  0001 C CNN
F 1 "+3.3V" H 4250 3985 30  0000 C CNN
F 2 "" H 4250 3875 60  0000 C CNN
F 3 "" H 4250 3875 60  0000 C CNN
	1    4250 3875
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 581D48B5
P 6250 3975
F 0 "#PWR025" H 6250 3975 30  0001 C CNN
F 1 "GND" H 6250 3905 30  0001 C CNN
F 2 "" H 6250 3975 60  0000 C CNN
F 3 "" H 6250 3975 60  0000 C CNN
	1    6250 3975
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 4175 4300 4175
Wire Wire Line
	6400 4425 6200 4425
Wire Notes Line
	3800 3350 3800 6050
Wire Notes Line
	6650 6050 6650 3350
Text Notes 5500 3625 0    60   ~ 0
EZO TEMP (not isolated)
$Comp
L Openponics:BNC P401
U 1 1 581D48C5
P 4000 4425
F 0 "P401" V 4125 4425 60  0000 C CNN
F 1 "BNC" V 4100 4225 40  0000 C CNN
F 2 "Openponics_Devices:WL-EB08040502_snk" H 4000 4425 60  0001 C CNN
F 3 "~" H 4000 4425 60  0000 C CNN
	1    4000 4425
	0    -1   1    0   
$EndComp
Wire Wire Line
	6200 4175 6400 4175
Text HLabel 6400 4175 2    60   Output ~ 0
SDA
Text HLabel 6400 4425 2    60   Input ~ 0
SCL
Wire Wire Line
	6200 3925 6250 3925
Wire Wire Line
	6250 3925 6250 3975
Wire Wire Line
	4300 3925 4250 3925
Wire Wire Line
	4250 3925 4250 3875
Wire Wire Line
	4000 4275 4000 4175
Wire Wire Line
	4200 4425 4300 4425
Wire Notes Line
	6650 3350 3800 3350
Wire Notes Line
	3800 6050 6650 6050
$EndSCHEMATC
