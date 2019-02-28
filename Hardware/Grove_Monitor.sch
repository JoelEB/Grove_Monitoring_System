EESchema Schematic File Version 4
LIBS:Grove_Monitor-cache
EELAYER 26 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 7
Title "Grove Monitoring System Adapter"
Date "2019-02-16"
Rev "V1.0"
Comp "Openponics"
Comment1 "Joel Bartlett"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR01
U 1 1 5337EC9B
P 9900 3150
F 0 "#PWR01" H 9900 3150 30  0001 C CNN
F 1 "GND" H 9900 3080 30  0001 C CNN
F 2 "" H 9900 3150 60  0000 C CNN
F 3 "" H 9900 3150 60  0000 C CNN
	1    9900 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 53E8CB5D
P 2950 1050
F 0 "R1" V 3030 1050 40  0000 C CNN
F 1 "4.7K" V 2957 1051 40  0000 C CNN
F 2 "Resistors:0603" V 2880 1050 30  0001 C CNN
F 3 "~" H 2950 1050 30  0000 C CNN
	1    2950 1050
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 53E8CB63
P 3250 1050
F 0 "R2" V 3330 1050 40  0000 C CNN
F 1 "4.7K" V 3257 1051 40  0000 C CNN
F 2 "Resistors:0603" V 3180 1050 30  0001 C CNN
F 3 "~" H 3250 1050 30  0000 C CNN
	1    3250 1050
	-1   0    0    1   
$EndComp
$Comp
L Device:CP1 C101
U 1 1 5448282F
P 9900 1400
F 0 "C101" H 9950 1500 50  0000 L CNN
F 1 "470µF" H 9950 1300 50  0000 L CNN
F 2 "Capacitors:PANASONIC_D" H 9900 1400 60  0001 C CNN
F 3 "~" H 9900 1400 60  0000 C CNN
	1    9900 1400
	1    0    0    -1  
$EndComp
NoConn ~ 9650 1250
NoConn ~ 9650 1650
NoConn ~ 9650 1850
NoConn ~ 9650 1950
NoConn ~ 9650 2150
NoConn ~ 9650 2450
NoConn ~ 9650 2650
NoConn ~ 9650 2850
NoConn ~ 9650 2950
NoConn ~ 9650 3050
NoConn ~ 7150 1650
NoConn ~ 7150 1750
NoConn ~ 7150 1850
NoConn ~ 7150 1950
NoConn ~ 7150 2050
NoConn ~ 7150 2150
NoConn ~ 7150 2250
NoConn ~ 7150 2450
NoConn ~ 7150 2750
NoConn ~ 7150 2850
NoConn ~ 7150 2950
$Comp
L power:+3.3V #PWR04
U 1 1 5658AFCD
P 7050 1050
F 0 "#PWR04" H 7050 1010 30  0001 C CNN
F 1 "+3.3V" H 7050 1160 30  0000 C CNN
F 2 "" H 7050 1050 60  0000 C CNN
F 3 "" H 7050 1050 60  0000 C CNN
	1    7050 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1550 7000 1550
Connection ~ 9900 1150
Wire Wire Line
	9900 1050 9900 1150
Wire Wire Line
	7150 1150 7050 1150
$Sheet
S 1000 4000 950  400 
U 581D467C
F0 "nonisolated1" 60
F1 "nonisolated.sch" 60
F2 "SDA" O R 1950 4150 60 
F3 "SCL" I R 1950 4300 60 
$EndSheet
Wire Wire Line
	7050 1150 7050 1050
Wire Wire Line
	9900 1150 9650 1150
Wire Wire Line
	5050 1750 5200 1750
Wire Wire Line
	5200 1750 5200 1250
NoConn ~ 9650 1450
NoConn ~ 9650 1550
Text Notes 1400 1000 0    60   ~ 0
Channel 1
Text Notes 1400 2000 0    60   ~ 0
Channel 2
Text Notes 1400 3050 0    60   ~ 0
Channel 3
Wire Wire Line
	5200 1250 5900 1250
$Comp
L SparkFun-Boards:PARTICLE_PHOTON JP1
U 1 1 5BF543F6
P 9200 4350
F 0 "JP1" H 9150 5204 45  0000 C CNN
F 1 "PARTICLE_PHOTON" H 9150 5120 45  0000 C CNN
F 2 "Boards:PARTICLE_PHOTON" H 9200 5200 20  0001 C CNN
F 3 "" H 9200 4350 60  0001 C CNN
F 4 "XXX-00000" H 9150 5131 60  0001 C CNN "Field4"
	1    9200 4350
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Hardware:STAND-OFF H1
U 1 1 5BF5456E
P 10000 7000
F 0 "H1" H 10000 7100 45  0001 C CNN
F 1 "STAND-OFF" H 10000 6900 45  0001 C CNN
F 2 "Openponics_Hardware:STAND-OFF_LARGE" H 10000 7150 20  0001 C CNN
F 3 "" H 10000 7000 50  0001 C CNN
F 4 "XXX-00000" H 10078 7000 60  0001 L CNN "Field4"
	1    10000 7000
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Hardware:STAND-OFF H2
U 1 1 5BF545CA
P 10100 7000
F 0 "H2" H 10100 7100 45  0001 C CNN
F 1 "STAND-OFF" H 10100 6900 45  0001 C CNN
F 2 "Openponics_Hardware:STAND-OFF_LARGE" H 10100 7150 20  0001 C CNN
F 3 "" H 10100 7000 50  0001 C CNN
F 4 "XXX-00000" H 10178 7000 60  0001 L CNN "Field4"
	1    10100 7000
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Hardware:STAND-OFF H3
U 1 1 5BF545E5
P 10200 7000
F 0 "H3" H 10200 7100 45  0001 C CNN
F 1 "STAND-OFF" H 10200 6900 45  0001 C CNN
F 2 "Openponics_Hardware:STAND-OFF_LARGE" H 10200 7150 20  0001 C CNN
F 3 "" H 10200 7000 50  0001 C CNN
F 4 "XXX-00000" H 10278 7000 60  0001 L CNN "Field4"
	1    10200 7000
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Hardware:STAND-OFF H4
U 1 1 5BF54600
P 10300 7000
F 0 "H4" H 10300 7100 45  0001 C CNN
F 1 "STAND-OFF" H 10300 6900 45  0001 C CNN
F 2 "Openponics_Hardware:STAND-OFF_LARGE" H 10300 7150 20  0001 C CNN
F 3 "" H 10300 7000 50  0001 C CNN
F 4 "XXX-00000" H 10378 7000 60  0001 L CNN "Field4"
	1    10300 7000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0101
U 1 1 5BF5B865
P 10100 3700
F 0 "#PWR0101" H 10100 3660 30  0001 C CNN
F 1 "+3.3V" H 10100 3810 30  0000 C CNN
F 2 "" H 10100 3700 60  0000 C CNN
F 3 "" H 10100 3700 60  0000 C CNN
	1    10100 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5BF5B8BD
P 8050 3900
F 0 "#PWR0102" H 8050 3900 30  0001 C CNN
F 1 "GND" H 8050 3830 30  0001 C CNN
F 2 "" H 8050 3900 60  0000 C CNN
F 3 "" H 8050 3900 60  0000 C CNN
	1    8050 3900
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5BF5B8F2
P 10450 4050
F 0 "#PWR0103" H 10450 4050 30  0001 C CNN
F 1 "GND" H 10450 3980 30  0001 C CNN
F 2 "" H 10450 4050 60  0000 C CNN
F 3 "" H 10450 4050 60  0000 C CNN
	1    10450 4050
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8300 3850 8050 3850
Wire Wire Line
	8050 3850 8050 3900
Wire Wire Line
	10000 3750 10100 3750
Wire Wire Line
	10100 3750 10100 3700
Wire Wire Line
	8250 3700 8250 3750
Wire Wire Line
	8250 3750 8300 3750
$Comp
L Openponics:PARTICLE_MESH U1
U 1 1 5C0B97D6
P 7250 5800
F 0 "U1" H 7250 6937 60  0000 C CNN
F 1 "PARTICLE_MESH" H 7250 6831 60  0000 C CNN
F 2 "Openponics_Templates:Particle_Mesh" H 7250 5800 60  0001 C CNN
F 3 "" H 7250 5800 60  0001 C CNN
	1    7250 5800
	1    0    0    -1  
$EndComp
$Sheet
S 1050 1100 850  600 
U 5C67B9C5
F0 "ADM3260_ISO_1" 60
F1 "ADM3260_ISO.sch" 60
F2 "SCL*" B R 1900 1500 59 
F3 "SDA*" B R 1900 1300 59 
F4 "V_ISO" O L 1050 1300 59 
F5 "GND_ISO" O L 1050 1500 59 
$EndSheet
Wire Wire Line
	2100 1250 2950 1250
Wire Wire Line
	2250 1350 3250 1350
Wire Wire Line
	2950 1200 2950 1250
$Comp
L power:+3.3V #PWR0107
U 1 1 5C7507FE
P 3100 800
F 0 "#PWR0107" H 3100 760 30  0001 C CNN
F 1 "+3.3V" H 3100 910 30  0000 C CNN
F 2 "" H 3100 800 60  0000 C CNN
F 3 "" H 3100 800 60  0000 C CNN
	1    3100 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 800  3100 850 
Wire Wire Line
	3100 850  2950 850 
Wire Wire Line
	2950 850  2950 900 
Wire Wire Line
	3250 900  3250 850 
Wire Wire Line
	3250 850  3100 850 
Connection ~ 3100 850 
Wire Wire Line
	1900 1300 2100 1300
Wire Wire Line
	1900 1500 2250 1500
$Sheet
S 1050 2100 850  600 
U 5C75CE04
F0 "ADM3260_ISO_2" 60
F1 "ADM3260_ISO.sch" 60
F2 "SCL*" B R 1900 2500 59 
F3 "SDA*" B R 1900 2300 59 
F4 "V_ISO" O L 1050 2300 59 
F5 "GND_ISO" O L 1050 2500 59 
$EndSheet
$Comp
L power:VCC_ISO2 #PWR0108
U 1 1 5C75ED79
P 800 2200
F 0 "#PWR0108" H 800 2050 50  0001 C CNN
F 1 "VCC_ISO2" H 817 2373 50  0000 C CNN
F 2 "" H 800 2200 50  0001 C CNN
F 3 "" H 800 2200 50  0001 C CNN
	1    800  2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND_ISO2 #PWR0109
U 1 1 5C75EE0B
P 800 2550
F 0 "#PWR0109" H 800 2300 50  0001 C CNN
F 1 "GND_ISO2" H 805 2377 50  0000 C CNN
F 2 "" H 800 2550 50  0001 C CNN
F 3 "" H 800 2550 50  0001 C CNN
	1    800  2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  2200 800  2300
Wire Wire Line
	800  2300 1050 2300
Wire Wire Line
	1050 2500 800  2500
Wire Wire Line
	800  2500 800  2550
Wire Wire Line
	2250 1350 2250 1500
Wire Wire Line
	2100 1250 2100 1300
Connection ~ 2100 1300
Wire Wire Line
	2100 1300 2100 2300
Connection ~ 2250 1500
Wire Wire Line
	2250 1500 2250 2500
Wire Wire Line
	1900 2300 2100 2300
Connection ~ 2100 2300
Wire Wire Line
	1900 2500 2250 2500
Connection ~ 2250 2500
Wire Wire Line
	2100 2300 2100 3350
Wire Wire Line
	2250 2500 2250 3500
$Sheet
S 1050 3150 850  550 
U 5C68641C
F0 "ADM3260_ISO_3" 60
F1 "ADM3260_ISO.sch" 60
F2 "SCL*" B R 1900 3500 60 
F3 "SDA*" B R 1900 3350 60 
F4 "V_ISO" O L 1050 3350 60 
F5 "GND_ISO" O L 1050 3500 60 
$EndSheet
$Comp
L power:VCC_ISO3 #PWR0110
U 1 1 5C686579
P 800 3300
F 0 "#PWR0110" H 800 3150 50  0001 C CNN
F 1 "VCC_ISO3" H 817 3473 50  0000 C CNN
F 2 "" H 800 3300 50  0001 C CNN
F 3 "" H 800 3300 50  0001 C CNN
	1    800  3300
	1    0    0    -1  
$EndComp
$Comp
L power:GND_ISO3 #PWR0111
U 1 1 5C6865D9
P 800 3550
F 0 "#PWR0111" H 800 3300 50  0001 C CNN
F 1 "GND_ISO3" H 805 3377 50  0000 C CNN
F 2 "" H 800 3550 50  0001 C CNN
F 3 "" H 800 3550 50  0001 C CNN
	1    800  3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  3300 800  3350
Wire Wire Line
	800  3350 1050 3350
Wire Wire Line
	800  3550 800  3500
Wire Wire Line
	800  3500 1050 3500
Wire Wire Line
	1900 3350 2100 3350
Wire Wire Line
	1900 3500 2250 3500
Connection ~ 2250 3500
Wire Wire Line
	2250 3500 2250 4300
$Sheet
S 650  6900 1100 200 
U 5C69D170
F0 "Current_Sense_1" 60
F1 "Current_Sense.sch" 60
F2 "OUT" O R 1750 7000 60 
$EndSheet
Wire Wire Line
	1750 7000 1950 7000
Text GLabel 1950 7000 2    60   Input ~ 0
I_OUT_1
$Sheet
S 650  7250 1100 200 
U 5C6A2AB7
F0 "Current_Sense_2" 60
F1 "Current_Sense.sch" 60
F2 "OUT" O R 1750 7350 60 
$EndSheet
Wire Wire Line
	1750 7350 1950 7350
Text GLabel 1950 7350 2    60   Input ~ 0
I_OUT_2
Wire Wire Line
	9900 1150 9900 1250
Wire Wire Line
	9900 1550 9900 1600
Wire Wire Line
	1950 4300 2250 4300
Wire Wire Line
	1950 4150 2100 4150
Wire Wire Line
	2100 4150 2100 3350
Connection ~ 2100 3350
Wire Wire Line
	800  1500 800  1550
Wire Wire Line
	1050 1500 800  1500
Wire Wire Line
	800  1300 1050 1300
Wire Wire Line
	800  1200 800  1300
$Comp
L power:GND_ISO1 #PWR0106
U 1 1 5C6BD977
P 800 1550
F 0 "#PWR0106" H 800 1300 50  0001 C CNN
F 1 "GND_ISO1" H 805 1377 50  0000 C CNN
F 2 "" H 800 1550 50  0001 C CNN
F 3 "" H 800 1550 50  0001 C CNN
	1    800  1550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC_ISO1 #PWR0105
U 1 1 5C6BD92F
P 800 1200
F 0 "#PWR0105" H 800 1050 50  0001 C CNN
F 1 "VCC_ISO1" H 817 1373 50  0000 C CNN
F 2 "" H 800 1200 50  0001 C CNN
F 3 "" H 800 1200 50  0001 C CNN
	1    800  1200
	1    0    0    -1  
$EndComp
Text Label 6350 1250 0    60   ~ 0
SDA
Text Label 6350 1350 0    60   ~ 0
SCL
Text GLabel 7850 6450 2    60   Input ~ 0
SDA
Text GLabel 7850 6350 2    60   Input ~ 0
SCL
Wire Wire Line
	7850 6350 7750 6350
Wire Wire Line
	7850 6450 7750 6450
$Comp
L power:GND #PWR0134
U 1 1 5C69586F
P 6550 5350
F 0 "#PWR0134" H 6550 5350 30  0001 C CNN
F 1 "GND" H 6550 5280 30  0001 C CNN
F 2 "" H 6550 5350 60  0000 C CNN
F 3 "" H 6550 5350 60  0000 C CNN
	1    6550 5350
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6750 5250 6550 5250
Wire Wire Line
	6550 5250 6550 5350
$Comp
L power:+3.3V #PWR0135
U 1 1 5C697D15
P 6550 4850
F 0 "#PWR0135" H 6550 4810 30  0001 C CNN
F 1 "+3.3V" H 6550 4960 30  0000 C CNN
F 2 "" H 6550 4850 60  0000 C CNN
F 3 "" H 6550 4850 60  0000 C CNN
	1    6550 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 4850 6550 5050
Wire Wire Line
	6550 5050 6750 5050
Text GLabel 10100 4750 2    60   Input ~ 0
SCL
Text GLabel 10100 4850 2    60   Input ~ 0
SDA
Wire Wire Line
	10100 4750 10000 4750
Wire Wire Line
	10100 4850 10000 4850
$Comp
L SparkFun-Connectors:QWIIC_CONNECTORJS-1MM J3
U 1 1 5C69F38D
P 4950 1950
F 0 "J3" H 4906 2454 45  0000 C CNN
F 1 "QWIIC" H 4906 2370 45  0000 C CNN
F 2 "Openponics_Connectors:Qwiic_Connector" H 4950 2450 20  0001 C CNN
F 3 "" H 4950 1950 50  0001 C CNN
F 4 "CONN-13694" H 4906 2381 60  0001 C CNN "Field4"
	1    4950 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0136
U 1 1 5C6D2DFD
P 5300 1800
F 0 "#PWR0136" H 5300 1760 30  0001 C CNN
F 1 "+3.3V" H 5300 1910 30  0000 C CNN
F 2 "" H 5300 1800 60  0000 C CNN
F 3 "" H 5300 1800 60  0000 C CNN
	1    5300 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0137
U 1 1 5C6D2E18
P 5100 2000
F 0 "#PWR0137" H 5100 2000 30  0001 C CNN
F 1 "GND" H 5100 1930 30  0001 C CNN
F 2 "" H 5100 2000 60  0000 C CNN
F 3 "" H 5100 2000 60  0000 C CNN
	1    5100 2000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5050 1950 5100 1950
Wire Wire Line
	5100 1950 5100 2000
Wire Wire Line
	5050 1850 5300 1850
Wire Wire Line
	5300 1850 5300 1800
Wire Wire Line
	5750 1750 5900 1750
$Comp
L SparkFun-Connectors:QWIIC_CONNECTORJS-1MM J4
U 1 1 5C6D99AA
P 5650 1950
F 0 "J4" H 5606 2454 45  0000 C CNN
F 1 "QWIIC" H 5606 2370 45  0000 C CNN
F 2 "Openponics_Connectors:Qwiic_Connector" H 5650 2450 20  0001 C CNN
F 3 "" H 5650 1950 50  0001 C CNN
F 4 "CONN-13694" H 5606 2381 60  0001 C CNN "Field4"
	1    5650 1950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0138
U 1 1 5C6D99B1
P 6000 1800
F 0 "#PWR0138" H 6000 1760 30  0001 C CNN
F 1 "+3.3V" H 6000 1910 30  0000 C CNN
F 2 "" H 6000 1800 60  0000 C CNN
F 3 "" H 6000 1800 60  0000 C CNN
	1    6000 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0139
U 1 1 5C6D99B7
P 5800 2000
F 0 "#PWR0139" H 5800 2000 30  0001 C CNN
F 1 "GND" H 5800 1930 30  0001 C CNN
F 2 "" H 5800 2000 60  0000 C CNN
F 3 "" H 5800 2000 60  0000 C CNN
	1    5800 2000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5750 1950 5800 1950
Wire Wire Line
	5800 1950 5800 2000
Wire Wire Line
	5750 1850 6000 1850
Wire Wire Line
	6000 1850 6000 1800
Wire Wire Line
	5900 1750 5900 1250
Connection ~ 5900 1250
Wire Wire Line
	5900 1250 7150 1250
$Comp
L Openponics:ATLAS_STAMP STP4
U 1 1 5C708C70
P 1900 5350
F 0 "STP4" V 1213 5600 60  0000 C CNN
F 1 "ATLAS_STAMP" V 1319 5600 60  0000 C CNN
F 2 "Openponics_Devices:ATLAS_STAMP_EZO" H 1900 6350 60  0001 C CNN
F 3 "" H 1900 6350 60  0000 C CNN
	1    1900 5350
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0140
U 1 1 5C70DE32
P 1100 5000
F 0 "#PWR0140" H 1100 4960 30  0001 C CNN
F 1 "+3.3V" H 1100 5110 30  0000 C CNN
F 2 "" H 1100 5000 60  0000 C CNN
F 3 "" H 1100 5000 60  0000 C CNN
	1    1100 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 5000 1100 5100
Wire Wire Line
	1100 5100 1200 5100
$Comp
L Connector_Generic:Conn_01x03 J5
U 1 1 5C710985
P 1400 6100
F 0 "J5" H 1480 6142 50  0000 L CNN
F 1 "Conn_01x03" H 1480 6051 50  0000 L CNN
F 2 "Connectors:SCREWTERMINAL-3.5MM-3" H 1400 6100 50  0001 C CNN
F 3 "~" H 1400 6100 50  0001 C CNN
	1    1400 6100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0141
U 1 1 5C71C92F
P 3200 5150
F 0 "#PWR0141" H 3200 5150 30  0001 C CNN
F 1 "GND" H 3200 5080 30  0001 C CNN
F 2 "" H 3200 5150 60  0000 C CNN
F 3 "" H 3200 5150 60  0000 C CNN
	1    3200 5150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3100 5100 3200 5100
Wire Wire Line
	3200 5100 3200 5150
Wire Wire Line
	3100 5350 3200 5350
Wire Wire Line
	3100 5600 3200 5600
Text GLabel 3200 5350 2    60   Input ~ 0
SDA
Text GLabel 3200 5600 2    60   Input ~ 0
SCL
Wire Notes Line
	500  6400 3750 6400
Text Notes 2700 4750 0    79   ~ 16
Flow Meter
Text Notes 550  650  0    79   ~ 16
Isolated Atlas-Scientific Stamps
Wire Notes Line
	2650 1400 3600 1400
Wire Notes Line
	2650 450  2650 3850
Text Notes 2750 650  0    79   ~ 16
I2C Pullups
Wire Notes Line
	6250 6750 6250 450 
Text Notes 7050 650  0    79   ~ 16
Microcontrollers/Single Board Computers
Wire Notes Line
	4650 500  4650 2250
Text Notes 4900 650  0    79   ~ 16
Qwiic Connectors
Text Notes 3750 2100 2    40   ~ 0
INT
Text Notes 3750 1675 2    40   ~ 0
SCL
Text Notes 3750 1800 2    40   ~ 0
SDA
Text Notes 3750 1900 2    40   ~ 0
GND
Text Notes 3750 2000 2    40   ~ 0
3.3V
$Comp
L Connector_Generic:Conn_01x05 J6
U 1 1 5C778763
P 3900 1850
F 0 "J6" H 3820 1425 50  0000 C CNN
F 1 "Conn_01x05" H 3820 1516 50  0000 C CNN
F 2 "Connectors:1X05_LONGPADS" H 3900 1850 50  0001 C CNN
F 3 "~" H 3900 1850 50  0001 C CNN
	1    3900 1850
	-1   0    0    1   
$EndComp
Wire Wire Line
	3250 1200 3250 1350
Wire Wire Line
	2950 1250 4350 1250
Connection ~ 2950 1250
Connection ~ 5200 1250
Connection ~ 3250 1350
Wire Wire Line
	4100 1650 4250 1650
Wire Wire Line
	4250 1650 4250 1350
Connection ~ 4250 1350
Wire Wire Line
	4250 1350 3250 1350
Wire Wire Line
	4350 1750 4350 1250
Connection ~ 4350 1250
Wire Wire Line
	4350 1250 5200 1250
Wire Wire Line
	4100 1750 4350 1750
$Comp
L power:GND #PWR0142
U 1 1 5C79DCDC
P 4200 2100
F 0 "#PWR0142" H 4200 2100 30  0001 C CNN
F 1 "GND" H 4200 2030 30  0001 C CNN
F 2 "" H 4200 2100 60  0000 C CNN
F 3 "" H 4200 2100 60  0000 C CNN
	1    4200 2100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4100 1950 4200 1950
Wire Wire Line
	4200 1950 4200 2100
$Comp
L power:+3.3V #PWR0143
U 1 1 5C7A1077
P 4450 1800
F 0 "#PWR0143" H 4450 1760 30  0001 C CNN
F 1 "+3.3V" H 4450 1910 30  0000 C CNN
F 2 "" H 4450 1800 60  0000 C CNN
F 3 "" H 4450 1800 60  0000 C CNN
	1    4450 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1800 4450 1850
Wire Wire Line
	4450 1850 4100 1850
Wire Wire Line
	4100 2050 4300 2050
Text GLabel 4300 2050 2    60   Input ~ 0
INT
Wire Notes Line
	3600 450  3600 2250
Text Notes 3750 650  0    79   ~ 16
AUX CONN
Text Notes 3650 950  0    49   ~ 0
Can be used to connect\nto other Atlas-Scientific\nsensors and pumps 
Text Notes 2300 4150 0    79   ~ 16
Non-Isolated \nAtlas-Scientific Stamps
Wire Notes Line
	450  3850 3750 3850
Wire Notes Line
	2550 6400 2550 8000
Text Notes 950  6550 0    79   ~ 16
Current Sensing
$Comp
L power:+3.3V #PWR0144
U 1 1 5C7EFD27
P 1150 5950
F 0 "#PWR0144" H 1150 5910 30  0001 C CNN
F 1 "+3.3V" H 1150 6060 30  0000 C CNN
F 2 "" H 1150 5950 60  0000 C CNN
F 3 "" H 1150 5950 60  0000 C CNN
	1    1150 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 5950 1150 6000
Wire Wire Line
	1150 6000 1200 6000
Wire Wire Line
	1200 5350 1150 5350
Wire Wire Line
	1200 5600 1150 5600
Text Label 1150 5600 2    61   ~ 0
P_GND
Text Label 1150 5350 2    61   ~ 0
PROBE
Text Label 1100 6100 2    61   ~ 0
PROBE
Text Label 1100 6200 2    61   ~ 0
P_GND
Wire Wire Line
	1100 6200 1200 6200
Wire Wire Line
	1200 6100 1100 6100
Wire Wire Line
	10000 4650 10100 4650
Wire Wire Line
	10000 4550 10100 4550
Wire Wire Line
	10000 4450 10100 4450
Wire Wire Line
	10000 4350 10100 4350
Wire Wire Line
	10000 4250 10100 4250
Wire Wire Line
	10000 4050 10450 4050
Text GLabel 7850 5750 2    61   Input ~ 0
D7
Text GLabel 7850 5850 2    61   Input ~ 0
D6
Text GLabel 7850 5950 2    61   Input ~ 0
D5
Text GLabel 7850 6050 2    61   Input ~ 0
D4
Text GLabel 7850 6150 2    61   Input ~ 0
D3
Text GLabel 7850 6250 2    61   Input ~ 0
D2
Wire Wire Line
	7850 5750 7750 5750
Wire Wire Line
	7850 5850 7750 5850
Wire Wire Line
	7850 5950 7750 5950
Wire Wire Line
	7850 6050 7750 6050
Wire Wire Line
	7850 6150 7750 6150
Wire Wire Line
	7850 6250 7750 6250
Text GLabel 10100 4150 2    61   Input ~ 0
D7
Text GLabel 10100 4250 2    61   Input ~ 0
D6
Text GLabel 10100 4350 2    61   Input ~ 0
D5
Text GLabel 10100 4450 2    61   Input ~ 0
D4
Text GLabel 10100 4550 2    61   Input ~ 0
D3
Text GLabel 10100 4650 2    61   Input ~ 0
D2
Wire Wire Line
	10000 4150 10100 4150
$Comp
L power:+5V #PWR0104
U 1 1 5C879968
P 8250 3700
F 0 "#PWR0104" H 8250 3550 50  0001 C CNN
F 1 "+5V" H 8265 3873 50  0000 C CNN
F 2 "" H 8250 3700 50  0001 C CNN
F 3 "" H 8250 3700 50  0001 C CNN
	1    8250 3700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0151
U 1 1 5C8799C7
P 7900 5300
F 0 "#PWR0151" H 7900 5150 50  0001 C CNN
F 1 "+5V" H 7915 5473 50  0000 C CNN
F 2 "" H 7900 5300 50  0001 C CNN
F 3 "" H 7900 5300 50  0001 C CNN
	1    7900 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 5300 7900 5550
Wire Wire Line
	7900 5550 7750 5550
$Comp
L power:+5V #PWR0152
U 1 1 5C88189F
P 9900 1050
F 0 "#PWR0152" H 9900 900 50  0001 C CNN
F 1 "+5V" H 9915 1223 50  0000 C CNN
F 2 "" H 9900 1050 50  0001 C CNN
F 3 "" H 9900 1050 50  0001 C CNN
	1    9900 1050
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Hardware:STAND-OFF H5
U 1 1 5C889B19
P 10050 7250
F 0 "H5" H 10050 7350 45  0001 C CNN
F 1 "STAND-OFF" H 10050 7150 45  0001 C CNN
F 2 "MeowWolf_Hardware:STAND-OFF" H 10050 7400 20  0001 C CNN
F 3 "" H 10050 7250 50  0001 C CNN
F 4 "XXX-00000" H 10128 7250 60  0001 L CNN "Field4"
	1    10050 7250
	1    0    0    -1  
$EndComp
$Comp
L SparkFun-Hardware:STAND-OFF H6
U 1 1 5C889B3C
P 10150 7250
F 0 "H6" H 10150 7350 45  0001 C CNN
F 1 "STAND-OFF" H 10150 7150 45  0001 C CNN
F 2 "MeowWolf_Hardware:STAND-OFF" H 10150 7400 20  0001 C CNN
F 3 "" H 10150 7250 50  0001 C CNN
F 4 "XXX-00000" H 10228 7250 60  0001 L CNN "Field4"
	1    10150 7250
	1    0    0    -1  
$EndComp
Text GLabel 7050 3750 0    60   Input ~ 0
I_OUT_1
Text GLabel 7050 4000 0    60   Input ~ 0
I_OUT_2
Text GLabel 6850 4250 0    60   Input ~ 0
INT
Wire Wire Line
	4250 1350 5100 1350
Wire Wire Line
	5750 1650 5800 1650
Wire Wire Line
	5800 1650 5800 1350
Connection ~ 5800 1350
Wire Wire Line
	5800 1350 7150 1350
Wire Wire Line
	5050 1650 5100 1650
Wire Wire Line
	5100 1650 5100 1350
Connection ~ 5100 1350
Wire Wire Line
	5100 1350 5800 1350
$Comp
L power:GND #PWR0153
U 1 1 5C8A6E6D
P 7000 3100
F 0 "#PWR0153" H 7000 3100 30  0001 C CNN
F 1 "GND" H 7000 3030 30  0001 C CNN
F 2 "" H 7000 3100 60  0000 C CNN
F 3 "" H 7000 3100 60  0000 C CNN
	1    7000 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 3050 7000 3050
Wire Wire Line
	7000 3050 7000 3100
Wire Wire Line
	7000 2350 7150 2350
$Comp
L Openponics:RASPBERRY_PI_2 PI_BOARD101
U 1 1 5658ACBD
P 8400 2050
F 0 "PI_BOARD101" H 9050 900 60  0000 C CNN
F 1 "RASPBERRY_PI_2" H 7800 3100 60  0000 C CNN
F 2 "Openponics_Templates:RasPi-40pin_Connector" H 7850 2150 60  0001 C CNN
F 3 "" H 7850 2150 60  0000 C CNN
	1    8400 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 1350 9750 1350
Wire Wire Line
	9750 1350 9750 1600
Wire Wire Line
	9750 1600 9900 1600
Connection ~ 9900 1600
Wire Wire Line
	9650 2050 9900 2050
Connection ~ 9900 2050
Wire Wire Line
	9900 2050 9900 2550
Wire Wire Line
	9650 2550 9900 2550
Connection ~ 9900 2550
Wire Wire Line
	9900 2550 9900 2750
Wire Wire Line
	9650 2750 9900 2750
Connection ~ 9900 2750
Wire Wire Line
	9900 2750 9900 3150
Wire Wire Line
	9650 1750 9900 1750
Wire Wire Line
	9900 1600 9900 1750
Connection ~ 9900 1750
Wire Wire Line
	9900 1750 9900 2050
Wire Wire Line
	7000 1550 7000 2350
Wire Wire Line
	7000 2350 7000 3050
Connection ~ 7000 2350
Connection ~ 7000 3050
Wire Wire Line
	7150 1450 6900 1450
Text GLabel 6900 1450 0    60   Input ~ 0
D4
Wire Wire Line
	7150 2550 6900 2550
Wire Wire Line
	6900 2650 7150 2650
Text GLabel 6900 2550 0    60   Input ~ 0
D5
Text GLabel 6900 2650 0    60   Input ~ 0
D6
Wire Wire Line
	9650 2250 10000 2250
Wire Wire Line
	9650 2350 10000 2350
Text GLabel 10000 2350 2    60   Input ~ 0
D7
Text GLabel 10000 2250 2    60   Input ~ 0
D8
Text GLabel 6700 5850 0    60   Input ~ 0
A5
Wire Wire Line
	6700 5850 6750 5850
Text GLabel 8200 4350 0    60   Input ~ 0
A5
Wire Wire Line
	8200 4350 8300 4350
Text GLabel 7150 3750 2    60   Input ~ 0
A5
Wire Wire Line
	7150 3750 7100 3750
$Comp
L Connector_Generic:Conn_01x03 J7
U 1 1 5C91060F
P 3000 7400
F 0 "J7" H 3080 7442 50  0000 L CNN
F 1 "Conn_01x03" H 2950 7200 50  0000 L CNN
F 2 "Connectors:SCREWTERMINAL-3.5MM-3" H 3000 7400 50  0001 C CNN
F 3 "~" H 3000 7400 50  0001 C CNN
	1    3000 7400
	-1   0    0    1   
$EndComp
$Comp
L Device:R R21
U 1 1 5C910699
P 3400 7200
F 0 "R21" V 3480 7200 40  0000 C CNN
F 1 "4.7K" V 3407 7201 40  0000 C CNN
F 2 "Resistors:0603" V 3330 7200 30  0001 C CNN
F 3 "~" H 3400 7200 30  0000 C CNN
	1    3400 7200
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0154
U 1 1 5C91BA7A
P 3250 7550
F 0 "#PWR0154" H 3250 7550 30  0001 C CNN
F 1 "GND" H 3250 7480 30  0001 C CNN
F 2 "" H 3250 7550 60  0000 C CNN
F 3 "" H 3250 7550 60  0000 C CNN
	1    3250 7550
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0155
U 1 1 5C91BAAC
P 3250 6950
F 0 "#PWR0155" H 3250 6910 30  0001 C CNN
F 1 "+3.3V" H 3250 7060 30  0000 C CNN
F 2 "" H 3250 6950 60  0000 C CNN
F 3 "" H 3250 6950 60  0000 C CNN
	1    3250 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 7300 3250 7300
Wire Wire Line
	3250 7300 3250 7000
Wire Wire Line
	3400 7050 3400 7000
Wire Wire Line
	3400 7000 3250 7000
Connection ~ 3250 7000
Wire Wire Line
	3250 7000 3250 6950
Wire Wire Line
	3200 7400 3400 7400
Wire Wire Line
	3400 7400 3400 7350
Wire Wire Line
	3200 7500 3250 7500
Wire Wire Line
	3250 7500 3250 7550
Wire Notes Line
	3750 3850 3750 8000
Text Notes 2600 6650 0    70   ~ 14
DS18B20 Waterproof\nTemperature Sensor
Text GLabel 6700 5750 0    60   Input ~ 0
A4
Wire Wire Line
	6700 5750 6750 5750
Text GLabel 8200 4450 0    60   Input ~ 0
A4
Wire Wire Line
	8200 4450 8300 4450
Text GLabel 7150 4000 2    60   Input ~ 0
A4
Wire Wire Line
	7150 4000 7100 4000
Wire Wire Line
	3400 7400 3450 7400
Connection ~ 3400 7400
Text GLabel 3450 7400 2    60   Input ~ 0
SIG
Text GLabel 6850 4450 0    60   Input ~ 0
SIG
Text GLabel 7150 4450 2    61   Input ~ 0
D4
Wire Wire Line
	7150 4450 6850 4450
Wire Wire Line
	6850 4250 7150 4250
Text GLabel 7150 4250 2    61   Input ~ 0
D5
$Comp
L Connector_Generic:Conn_01x02 J8
U 1 1 5C6FA07B
P 3000 2050
F 0 "J8" H 2920 1725 50  0000 C CNN
F 1 "Conn_01x02" H 2920 1816 50  0000 C CNN
F 2 "Connectors:SCREWTERMINAL-3.5MM-2" H 3000 2050 50  0001 C CNN
F 3 "~" H 3000 2050 50  0001 C CNN
	1    3000 2050
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0156
U 1 1 5C708164
P 3400 2100
F 0 "#PWR0156" H 3400 2100 30  0001 C CNN
F 1 "GND" H 3400 2030 30  0001 C CNN
F 2 "" H 3400 2100 60  0000 C CNN
F 3 "" H 3400 2100 60  0000 C CNN
	1    3400 2100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3200 2050 3400 2050
Wire Wire Line
	3400 2050 3400 2100
$Comp
L power:+5V #PWR0157
U 1 1 5C70F520
P 3400 1900
F 0 "#PWR0157" H 3400 1750 50  0001 C CNN
F 1 "+5V" H 3415 2073 50  0000 C CNN
F 2 "" H 3400 1900 50  0001 C CNN
F 3 "" H 3400 1900 50  0001 C CNN
	1    3400 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 1900 3400 1950
Wire Wire Line
	3400 1950 3200 1950
Wire Notes Line
	2650 2250 6250 2250
Text Notes 2700 1550 0    79   ~ 16
5V PWR Input
$Comp
L Openponics:OPENPONICS_LOGO LOGO1
U 1 1 5C72FF71
P 9450 7050
F 0 "LOGO1" H 9450 6818 60  0001 C CNN
F 1 "OPENPONICS_LOGO" H 9450 7282 60  0001 C CNN
F 2 "Openponics_Logos:Openponics_Logo_Plain_45mm" H 9450 7050 60  0001 C CNN
F 3 "" H 9450 7050 60  0001 C CNN
	1    9450 7050
	1    0    0    -1  
$EndComp
$Comp
L Openponics:OPENPONICS_LOGO LOGO2
U 1 1 5C730302
P 8500 7050
F 0 "LOGO2" H 8500 6818 60  0001 C CNN
F 1 "OPENPONICS_LOGO" H 8500 7282 60  0001 C CNN
F 2 "Openponics_Logos:Openponics_Logo_Horizontal_16mm" H 8500 7050 60  0001 C CNN
F 3 "" H 8500 7050 60  0001 C CNN
	1    8500 7050
	1    0    0    -1  
$EndComp
$Comp
L Openponics:OPENPONICS_LOGO LOGO3
U 1 1 5C730708
P 7550 7050
F 0 "LOGO3" H 7550 6818 60  0001 C CNN
F 1 "OPENPONICS_LOGO" H 7550 7282 60  0001 C CNN
F 2 "Openponics_Logos:Openponics_Logo_Horizontal_3_inch" H 7550 7050 60  0001 C CNN
F 3 "" H 7550 7050 60  0001 C CNN
	1    7550 7050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J9
U 1 1 5C7A01C0
P 7350 3600
F 0 "J9" H 7430 3642 50  0000 L CNN
F 1 "Conn_01x01" H 7430 3551 50  0000 L CNN
F 2 "Connectors:1X01" H 7350 3600 50  0001 C CNN
F 3 "~" H 7350 3600 50  0001 C CNN
	1    7350 3600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J10
U 1 1 5C7A01FC
P 7350 3850
F 0 "J10" H 7430 3892 50  0000 L CNN
F 1 "Conn_01x01" H 7430 3801 50  0000 L CNN
F 2 "Connectors:1X01" H 7350 3850 50  0001 C CNN
F 3 "~" H 7350 3850 50  0001 C CNN
	1    7350 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 3850 7100 3850
Wire Wire Line
	7100 3850 7100 4000
Connection ~ 7100 4000
Wire Wire Line
	7100 4000 7050 4000
Wire Wire Line
	7150 3600 7100 3600
Wire Wire Line
	7100 3600 7100 3750
Connection ~ 7100 3750
Wire Wire Line
	7100 3750 7050 3750
$EndSCHEMATC
