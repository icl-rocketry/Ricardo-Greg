EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Connection ~ 4950 3800
Wire Wire Line
	4950 3700 4950 3800
Wire Wire Line
	4950 3800 5000 3800
Wire Wire Line
	4950 3900 4950 3800
$Comp
L power:GND #PWR?
U 1 1 61988BF8
P 5000 3800
AR Path="/61988BF8" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61988BF8" Ref="#PWR0119"  Part="1" 
AR Path="/61997866/61988BF8" Ref="#PWR?"  Part="1" 
F 0 "#PWR0119" H 5000 3550 50  0001 C CNN
F 1 "GND" V 5005 3672 50  0000 R CNN
F 2 "" H 5000 3800 50  0001 C CNN
F 3 "" H 5000 3800 50  0001 C CNN
	1    5000 3800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4650 3900 4750 3900
Wire Wire Line
	4650 3800 4650 3900
Wire Wire Line
	4550 3800 4650 3800
$Comp
L Device:R_Small R?
U 1 1 61988C01
P 4850 3900
AR Path="/61988C01" Ref="R?"  Part="1" 
AR Path="/61925999/61988C01" Ref="R16"  Part="1" 
AR Path="/61997866/61988C01" Ref="R?"  Part="1" 
F 0 "R16" V 4654 3900 50  0000 C CNN
F 1 "5.1k" V 4745 3900 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4850 3900 50  0001 C CNN
F 3 "~" H 4850 3900 50  0001 C CNN
	1    4850 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	4550 3700 4750 3700
$Comp
L Device:R_Small R?
U 1 1 61988C0E
P 4850 3700
AR Path="/61988C0E" Ref="R?"  Part="1" 
AR Path="/61925999/61988C0E" Ref="R15"  Part="1" 
AR Path="/61997866/61988C0E" Ref="R?"  Part="1" 
F 0 "R15" V 5046 3700 50  0000 C CNN
F 1 "5.1k" V 4955 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4850 3700 50  0001 C CNN
F 3 "~" H 4850 3700 50  0001 C CNN
	1    4850 3700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3800 5000 3950 5000
Connection ~ 3800 5000
Wire Wire Line
	3800 5000 3800 5150
$Comp
L power:GND #PWR?
U 1 1 61988C18
P 3800 5150
AR Path="/61988C18" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61988C18" Ref="#PWR0120"  Part="1" 
AR Path="/61997866/61988C18" Ref="#PWR?"  Part="1" 
F 0 "#PWR0120" H 3800 4900 50  0001 C CNN
F 1 "GND" H 3805 4977 50  0000 C CNN
F 2 "" H 3800 5150 50  0001 C CNN
F 3 "" H 3800 5150 50  0001 C CNN
	1    3800 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 5000 3800 5000
NoConn ~ 4550 4700
NoConn ~ 4550 4600
Text Label 5100 5300 1    50   ~ 0
VBUS
Text Label 5200 5300 1    50   ~ 0
Data-
Text Label 5300 5300 1    50   ~ 0
Data+
Text Label 4600 4050 0    50   ~ 0
Data-
Wire Wire Line
	4550 4050 4550 4000
Connection ~ 4550 4050
Wire Wire Line
	4550 4050 4600 4050
Wire Wire Line
	4550 4100 4550 4050
Text Label 4600 4250 0    50   ~ 0
Data+
Wire Wire Line
	4550 4250 4600 4250
Wire Wire Line
	4550 4200 4550 4250
$Comp
L Connector:USB_C_Receptacle_USB2.0 J?
U 1 1 61988C49
P 3950 4100
AR Path="/61988C49" Ref="J?"  Part="1" 
AR Path="/61925999/61988C49" Ref="J3"  Part="1" 
AR Path="/61997866/61988C49" Ref="J?"  Part="1" 
F 0 "J3" H 4057 4967 50  0000 C CNN
F 1 "USB_C_Receptacle_USB2.0" H 4057 4876 50  0000 C CNN
F 2 "Connector_USB:USB_C_Receptacle_GCT_USB4085" H 4100 4100 50  0001 C CNN
F 3 "https://www.usb.org/sites/default/files/documents/usb_type-c.zip" H 4100 4100 50  0001 C CNN
	1    3950 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61988C86
P 5200 5700
AR Path="/61988C86" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61988C86" Ref="#PWR0121"  Part="1" 
AR Path="/61997866/61988C86" Ref="#PWR?"  Part="1" 
F 0 "#PWR0121" H 5200 5450 50  0001 C CNN
F 1 "GND" H 5205 5527 50  0000 C CNN
F 2 "" H 5200 5700 50  0001 C CNN
F 3 "" H 5200 5700 50  0001 C CNN
	1    5200 5700
	1    0    0    -1  
$EndComp
$Comp
L Power_Protection:SP0503BAHT D?
U 1 1 61988C8C
P 5200 5500
AR Path="/61988C8C" Ref="D?"  Part="1" 
AR Path="/61925999/61988C8C" Ref="D4"  Part="1" 
AR Path="/61997866/61988C8C" Ref="D?"  Part="1" 
F 0 "D4" H 5405 5546 50  0000 L CNN
F 1 "SP0503BAHT" H 5405 5455 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-143" H 5425 5450 50  0001 L CNN
F 3 "http://www.littelfuse.com/~/media/files/littelfuse/technical%20resources/documents/data%20sheets/sp05xxba.pdf" H 5325 5625 50  0001 C CNN
	1    5200 5500
	1    0    0    -1  
$EndComp
Connection ~ 4550 4250
Wire Wire Line
	4550 4250 4550 4300
$Comp
L power:GND #PWR0122
U 1 1 619952DA
P 4400 1700
F 0 "#PWR0122" H 4400 1450 50  0001 C CNN
F 1 "GND" V 4405 1572 50  0000 R CNN
F 2 "" H 4400 1700 50  0001 C CNN
F 3 "" H 4400 1700 50  0001 C CNN
	1    4400 1700
	-1   0    0    1   
$EndComp
$Comp
L power:VBUS #PWR0123
U 1 1 61996CDA
P 5250 2650
F 0 "#PWR0123" H 5250 2500 50  0001 C CNN
F 1 "VBUS" H 5265 2823 50  0000 C CNN
F 2 "" H 5250 2650 50  0001 C CNN
F 3 "" H 5250 2650 50  0001 C CNN
	1    5250 2650
	-1   0    0    1   
$EndComp
Text Label 4500 1950 0    50   ~ 0
Data-
Text Label 4500 1850 0    50   ~ 0
Data+
NoConn ~ 5550 2650
NoConn ~ 5650 2650
$Comp
L power:VBUS #PWR0124
U 1 1 61A236C4
P 4600 3350
F 0 "#PWR0124" H 4600 3200 50  0001 C CNN
F 1 "VBUS" V 4615 3478 50  0000 L CNN
F 2 "" H 4600 3350 50  0001 C CNN
F 3 "" H 4600 3350 50  0001 C CNN
	1    4600 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3350 4600 3500
Wire Wire Line
	4600 3500 4550 3500
Wire Wire Line
	4500 1950 4750 1950
Wire Wire Line
	4500 1850 4750 1850
Wire Wire Line
	4400 1700 4400 1750
$Comp
L Device:C_Small C8
U 1 1 61A28787
P 4050 2400
F 0 "C8" H 3958 2354 50  0000 R CNN
F 1 "1.0uF" H 3958 2445 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4050 2400 50  0001 C CNN
F 3 "~" H 4050 2400 50  0001 C CNN
	1    4050 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	3550 2150 3550 2500
Wire Wire Line
	4050 2300 4050 2150
Connection ~ 4050 2150
Wire Wire Line
	4050 2150 3550 2150
Wire Wire Line
	4450 2300 4450 2150
Wire Wire Line
	4050 2500 4250 2500
$Comp
L power:GND #PWR0125
U 1 1 61A3A0E4
P 4250 2550
F 0 "#PWR0125" H 4250 2300 50  0001 C CNN
F 1 "GND" H 4255 2377 50  0000 C CNN
F 2 "" H 4250 2550 50  0001 C CNN
F 3 "" H 4250 2550 50  0001 C CNN
	1    4250 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 2500 4250 2550
Connection ~ 4250 2500
Wire Wire Line
	4250 2500 4450 2500
Wire Wire Line
	4050 2150 4450 2150
Connection ~ 4450 2150
$Comp
L power:VBUS #PWR0126
U 1 1 61A46D5F
P 3550 2500
F 0 "#PWR0126" H 3550 2350 50  0001 C CNN
F 1 "VBUS" H 3565 2673 50  0000 C CNN
F 2 "" H 3550 2500 50  0001 C CNN
F 3 "" H 3550 2500 50  0001 C CNN
	1    3550 2500
	-1   0    0    1   
$EndComp
Text HLabel 5650 750  1    50   Output ~ 0
RTS
Text HLabel 5550 750  1    50   Output ~ 0
RXD
Text HLabel 5250 750  1    50   Output ~ 0
DTR
Text HLabel 5450 750  1    50   Output ~ 0
TXD
NoConn ~ 5350 1050
NoConn ~ 4750 1550
NoConn ~ 4750 1650
NoConn ~ 5750 1050
NoConn ~ 5150 1050
$Comp
L Device:R_Small R17
U 1 1 61A58884
P 5400 2850
F 0 "R17" H 5341 2804 50  0000 R CNN
F 1 "4.7k" H 5341 2895 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5400 2850 50  0001 C CNN
F 3 "~" H 5400 2850 50  0001 C CNN
	1    5400 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	5350 2650 5350 2750
Wire Wire Line
	5350 2750 5400 2750
Wire Wire Line
	5650 1050 5650 750 
Wire Wire Line
	5550 1050 5550 750 
Wire Wire Line
	5450 1050 5450 750 
Wire Wire Line
	5250 1050 5250 750 
Wire Wire Line
	4450 2150 4750 2150
$Comp
L iclr:INTERFACE-CP2102-GMR(QFN28) U3
U 1 1 6198EFBF
P 5550 1850
F 0 "U3" H 6394 1896 50  0000 L CNN
F 1 "INTERFACE-CP2102-GMR(QFN28)" H 6394 1805 50  0000 L CNN
F 2 "iclr:QFN28G_0.5-5X5MM" H 5550 1850 50  0001 L BNN
F 3 "" H 5550 1850 50  0001 L BNN
F 4 "CP2102-GMR-QFN28" H 5550 1850 50  0001 L BNN "VALUE"
F 5 "CP2102-GMR" H 5550 1850 50  0001 L BNN "MPN"
	1    5550 1850
	1    0    0    -1  
$EndComp
NoConn ~ 6350 1550
NoConn ~ 6350 1650
NoConn ~ 6350 1750
NoConn ~ 6350 1850
NoConn ~ 6350 1950
NoConn ~ 6350 2050
NoConn ~ 6350 2150
NoConn ~ 5850 1050
NoConn ~ 5750 2650
NoConn ~ 5850 2650
$Comp
L power:+3.3V #PWR?
U 1 1 61A9124D
P 5400 2950
AR Path="/61A9124D" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61A9124D" Ref="#PWR0127"  Part="1" 
AR Path="/61997866/61A9124D" Ref="#PWR?"  Part="1" 
F 0 "#PWR0127" H 5400 2800 50  0001 C CNN
F 1 "+3.3V" H 5415 3123 50  0000 C CNN
F 2 "" H 5400 2950 50  0001 C CNN
F 3 "" H 5400 2950 50  0001 C CNN
	1    5400 2950
	-1   0    0    1   
$EndComp
NoConn ~ 5450 2650
Text HLabel 3700 2050 0    50   Output ~ 0
3.3V
$Comp
L Device:C_Small C9
U 1 1 61AA33A7
P 4400 1900
F 0 "C9" H 4308 1854 50  0000 R CNN
F 1 "0.1uF" H 4308 1945 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4400 1900 50  0001 C CNN
F 3 "~" H 4400 1900 50  0001 C CNN
	1    4400 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C7
U 1 1 61AA4224
P 3850 1900
F 0 "C7" H 3941 1946 50  0000 L CNN
F 1 "4.7uF" H 3941 1855 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_3x5.3" H 3850 1900 50  0001 C CNN
F 3 "~" H 3850 1900 50  0001 C CNN
	1    3850 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 1750 4400 1750
Connection ~ 4400 1750
Wire Wire Line
	4400 1750 4400 1800
Wire Wire Line
	3700 2050 3850 2050
Wire Wire Line
	4400 2000 4400 2050
Wire Wire Line
	4400 2050 4750 2050
Wire Wire Line
	3850 2000 3850 2050
Wire Wire Line
	4400 2050 3850 2050
Connection ~ 4400 2050
Connection ~ 3850 2050
Wire Wire Line
	3850 1800 3850 1750
Wire Wire Line
	3850 1750 4400 1750
$Comp
L Connector:TestPoint TP17
U 1 1 61AB7CB8
P 2050 3900
F 0 "TP17" V 2004 4088 50  0000 L CNN
F 1 "TestPoint" V 2095 4088 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 2250 3900 50  0001 C CNN
F 3 "~" H 2250 3900 50  0001 C CNN
	1    2050 3900
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP18
U 1 1 61AB87F3
P 2050 4100
F 0 "TP18" V 2004 4288 50  0000 L CNN
F 1 "TestPoint" V 2095 4288 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 2250 4100 50  0001 C CNN
F 3 "~" H 2250 4100 50  0001 C CNN
	1    2050 4100
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP19
U 1 1 61AB9568
P 2050 4300
F 0 "TP19" V 2004 4488 50  0000 L CNN
F 1 "TestPoint" V 2095 4488 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 2250 4300 50  0001 C CNN
F 3 "~" H 2250 4300 50  0001 C CNN
	1    2050 4300
	0    1    1    0   
$EndComp
Text Label 1750 3900 0    50   ~ 0
Data-
Wire Wire Line
	1750 3900 2050 3900
Text Label 1750 4100 0    50   ~ 0
Data+
Wire Wire Line
	1750 4100 2050 4100
Text Label 1750 4300 0    50   ~ 0
VBUS
Wire Wire Line
	1750 4300 2050 4300
$Comp
L Device:LED_Small D?
U 1 1 61B104EA
P 6400 3650
AR Path="/61B104EA" Ref="D?"  Part="1" 
AR Path="/61925999/61B104EA" Ref="D5"  Part="1" 
F 0 "D5" H 6400 3445 50  0000 C CNN
F 1 "USB_LED" H 6400 3536 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" V 6400 3650 50  0001 C CNN
F 3 "~" V 6400 3650 50  0001 C CNN
	1    6400 3650
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R18
U 1 1 61B104F0
P 6100 3650
F 0 "R18" V 5904 3650 50  0000 C CNN
F 1 "470" V 5995 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6100 3650 50  0001 C CNN
F 3 "~" H 6100 3650 50  0001 C CNN
	1    6100 3650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6300 3650 6200 3650
$Comp
L power:GND #PWR?
U 1 1 61B11F2F
P 6500 3650
AR Path="/61B11F2F" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61B11F2F" Ref="#PWR0128"  Part="1" 
AR Path="/61997866/61B11F2F" Ref="#PWR?"  Part="1" 
F 0 "#PWR0128" H 6500 3400 50  0001 C CNN
F 1 "GND" V 6505 3522 50  0000 R CNN
F 2 "" H 6500 3650 50  0001 C CNN
F 3 "" H 6500 3650 50  0001 C CNN
	1    6500 3650
	0    -1   -1   0   
$EndComp
$Comp
L power:VBUS #PWR0129
U 1 1 61B168A6
P 6000 3650
F 0 "#PWR0129" H 6000 3500 50  0001 C CNN
F 1 "VBUS" V 6015 3778 50  0000 L CNN
F 2 "" H 6000 3650 50  0001 C CNN
F 3 "" H 6000 3650 50  0001 C CNN
	1    6000 3650
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C10
U 1 1 61A2A124
P 4450 2400
F 0 "C10" H 4358 2354 50  0000 R CNN
F 1 "0.1uF" H 4358 2445 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4450 2400 50  0001 C CNN
F 3 "~" H 4450 2400 50  0001 C CNN
	1    4450 2400
	-1   0    0    1   
$EndComp
$EndSCHEMATC
