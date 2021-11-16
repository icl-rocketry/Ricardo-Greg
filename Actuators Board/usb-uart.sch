EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
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
AR Path="/61925999/61988BF8" Ref="#PWR?"  Part="1" 
AR Path="/61997866/61988BF8" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 5000 3550 50  0001 C CNN
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
AR Path="/61925999/61988C01" Ref="R?"  Part="1" 
AR Path="/61997866/61988C01" Ref="R?"  Part="1" 
F 0 "R?" V 4654 3900 50  0000 C CNN
F 1 "5.1k" V 4745 3900 50  0000 C CNN
F 2 "" H 4850 3900 50  0001 C CNN
F 3 "~" H 4850 3900 50  0001 C CNN
	1    4850 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	4550 3700 4750 3700
$Comp
L Device:D_Schottky D?
U 1 1 61988C08
P 6000 3500
AR Path="/61988C08" Ref="D?"  Part="1" 
AR Path="/61925999/61988C08" Ref="D?"  Part="1" 
AR Path="/61997866/61988C08" Ref="D?"  Part="1" 
F 0 "D?" V 5954 3579 50  0000 L CNN
F 1 "D_Schottky" V 6045 3579 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-123F" H 6000 3500 50  0001 C CNN
F 3 "~" H 6000 3500 50  0001 C CNN
	1    6000 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 61988C0E
P 4850 3700
AR Path="/61988C0E" Ref="R?"  Part="1" 
AR Path="/61925999/61988C0E" Ref="R?"  Part="1" 
AR Path="/61997866/61988C0E" Ref="R?"  Part="1" 
F 0 "R?" V 5046 3700 50  0000 C CNN
F 1 "5.1k" V 4955 3700 50  0000 C CNN
F 2 "" H 4850 3700 50  0001 C CNN
F 3 "~" H 4850 3700 50  0001 C CNN
	1    4850 3700
	0    -1   -1   0   
$EndComp
Connection ~ 6000 3650
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
AR Path="/61925999/61988C18" Ref="#PWR?"  Part="1" 
AR Path="/61997866/61988C18" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 3800 4900 50  0001 C CNN
F 1 "GND" H 3805 4977 50  0000 C CNN
F 2 "" H 3800 5150 50  0001 C CNN
F 3 "" H 3800 5150 50  0001 C CNN
	1    3800 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 5000 3800 5000
Text Label 4550 3500 0    50   ~ 0
VBUS
Text Label 5800 4700 2    50   ~ 0
VBUS
$Comp
L Device:R_Small R?
U 1 1 61988C21
P 5900 4700
AR Path="/61988C21" Ref="R?"  Part="1" 
AR Path="/61925999/61988C21" Ref="R?"  Part="1" 
AR Path="/61997866/61988C21" Ref="R?"  Part="1" 
F 0 "R?" V 5704 4700 50  0000 C CNN
F 1 "22.1k" V 5795 4700 50  0000 C CNN
F 2 "" H 5900 4700 50  0001 C CNN
F 3 "~" H 5900 4700 50  0001 C CNN
	1    5900 4700
	0    1    1    0   
$EndComp
NoConn ~ 4550 4700
NoConn ~ 4550 4600
$Comp
L power:GND #PWR?
U 1 1 61988C29
P 6100 5000
AR Path="/61988C29" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61988C29" Ref="#PWR?"  Part="1" 
AR Path="/61997866/61988C29" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6100 4750 50  0001 C CNN
F 1 "GND" H 6105 4827 50  0000 C CNN
F 2 "" H 6100 5000 50  0001 C CNN
F 3 "" H 6100 5000 50  0001 C CNN
	1    6100 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4700 6500 4700
Connection ~ 6100 4700
Wire Wire Line
	6100 4800 6100 4700
Wire Wire Line
	6000 4700 6100 4700
$Comp
L Device:R_Small R?
U 1 1 61988C33
P 6100 4900
AR Path="/61988C33" Ref="R?"  Part="1" 
AR Path="/61925999/61988C33" Ref="R?"  Part="1" 
AR Path="/61997866/61988C33" Ref="R?"  Part="1" 
F 0 "R?" H 6159 4946 50  0000 L CNN
F 1 "47.5k" H 6159 4855 50  0000 L CNN
F 2 "" H 6100 4900 50  0001 C CNN
F 3 "~" H 6100 4900 50  0001 C CNN
	1    6100 4900
	1    0    0    -1  
$EndComp
Connection ~ 6500 4700
Text Label 5100 5300 1    50   ~ 0
VBUS
Text Label 5200 5300 1    50   ~ 0
Data-
Text Label 5300 5300 1    50   ~ 0
Data+
Wire Wire Line
	6000 4400 6600 4400
Text Label 6600 4900 2    50   ~ 0
Data-
Text Label 6600 4800 2    50   ~ 0
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
AR Path="/61925999/61988C49" Ref="J?"  Part="1" 
AR Path="/61997866/61988C49" Ref="J?"  Part="1" 
F 0 "J?" H 4057 4967 50  0000 C CNN
F 1 "USB_C_Receptacle_USB2.0" H 4057 4876 50  0000 C CNN
F 2 "" H 4100 4100 50  0001 C CNN
F 3 "https://www.usb.org/sites/default/files/documents/usb_type-c.zip" H 4100 4100 50  0001 C CNN
	1    3950 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 4400 6000 3650
$Comp
L power:VDD #PWR?
U 1 1 61988C50
P 6000 3350
AR Path="/61988C50" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61988C50" Ref="#PWR?"  Part="1" 
AR Path="/61997866/61988C50" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6000 3200 50  0001 C CNN
F 1 "VDD" H 6017 3523 50  0000 C CNN
F 2 "" H 6000 3350 50  0001 C CNN
F 3 "" H 6000 3350 50  0001 C CNN
	1    6000 3350
	1    0    0    -1  
$EndComp
NoConn ~ 7800 3800
NoConn ~ 7800 3900
NoConn ~ 7800 4300
NoConn ~ 7800 4500
$Comp
L Interface_USB:CP2102N-A01-GQFN24 U?
U 1 1 61988C72
P 7200 4400
AR Path="/61988C72" Ref="U?"  Part="1" 
AR Path="/61925999/61988C72" Ref="U?"  Part="1" 
AR Path="/61997866/61988C72" Ref="U?"  Part="1" 
F 0 "U?" H 7200 5481 50  0000 C CNN
F 1 "CP2102N-A01-GQFN24" H 7200 5390 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm_ThermalVias" H 7650 3600 50  0001 L CNN
F 3 "https://www.silabs.com/documents/public/data-sheets/cp2102n-datasheet.pdf" H 7250 3350 50  0001 C CNN
	1    7200 4400
	1    0    0    -1  
$EndComp
Connection ~ 7300 5300
Wire Wire Line
	6000 3650 6200 3650
$Comp
L Device:R_Small R?
U 1 1 61988C7A
P 6200 3750
AR Path="/61988C7A" Ref="R?"  Part="1" 
AR Path="/61925999/61988C7A" Ref="R?"  Part="1" 
AR Path="/61997866/61988C7A" Ref="R?"  Part="1" 
F 0 "R?" H 6259 3796 50  0000 L CNN
F 1 "22K" H 6259 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6200 3750 50  0001 C CNN
F 3 "~" H 6200 3750 50  0001 C CNN
	1    6200 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 61988C80
P 6200 4050
AR Path="/61988C80" Ref="R?"  Part="1" 
AR Path="/61925999/61988C80" Ref="R?"  Part="1" 
AR Path="/61997866/61988C80" Ref="R?"  Part="1" 
F 0 "R?" H 6259 4096 50  0000 L CNN
F 1 "43K" H 6259 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6200 4050 50  0001 C CNN
F 3 "~" H 6200 4050 50  0001 C CNN
	1    6200 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61988C86
P 5200 5700
AR Path="/61988C86" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61988C86" Ref="#PWR?"  Part="1" 
AR Path="/61997866/61988C86" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 5200 5450 50  0001 C CNN
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
AR Path="/61925999/61988C8C" Ref="D?"  Part="1" 
AR Path="/61997866/61988C8C" Ref="D?"  Part="1" 
F 0 "D?" H 5405 5546 50  0000 L CNN
F 1 "SP0503BAHT" H 5405 5455 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-143" H 5425 5450 50  0001 L CNN
F 3 "http://www.littelfuse.com/~/media/files/littelfuse/technical%20resources/documents/data%20sheets/sp05xxba.pdf" H 5325 5625 50  0001 C CNN
	1    5200 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 4400 8100 4400
Wire Wire Line
	7800 4000 8100 4000
Wire Wire Line
	6550 3800 6600 3800
Wire Wire Line
	6550 3700 6550 3800
Wire Wire Line
	6550 3500 7100 3500
$Comp
L Device:R_Small R?
U 1 1 61988C99
P 6550 3600
AR Path="/61988C99" Ref="R?"  Part="1" 
AR Path="/61925999/61988C99" Ref="R?"  Part="1" 
AR Path="/61997866/61988C99" Ref="R?"  Part="1" 
F 0 "R?" H 6609 3646 50  0000 L CNN
F 1 "1K" H 6609 3555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6550 3600 50  0001 C CNN
F 3 "~" H 6550 3600 50  0001 C CNN
	1    6550 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 5300 7200 5300
Wire Wire Line
	7300 5400 7300 5300
$Comp
L power:GND #PWR?
U 1 1 61988CA1
P 7300 5400
AR Path="/61988CA1" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61988CA1" Ref="#PWR?"  Part="1" 
AR Path="/61997866/61988CA1" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7300 5150 50  0001 C CNN
F 1 "GND" H 7305 5227 50  0000 C CNN
F 2 "" H 7300 5400 50  0001 C CNN
F 3 "" H 7300 5400 50  0001 C CNN
	1    7300 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 4200 8100 4200
Wire Wire Line
	7800 4100 8100 4100
Connection ~ 7100 3500
Wire Wire Line
	7100 3500 7200 3500
Wire Wire Line
	7100 3400 7100 3500
$Comp
L power:+3.3V #PWR?
U 1 1 61988CAE
P 7100 3400
AR Path="/61988CAE" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61988CAE" Ref="#PWR?"  Part="1" 
AR Path="/61997866/61988CAE" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 7100 3250 50  0001 C CNN
F 1 "+3.3V" H 7115 3573 50  0000 C CNN
F 2 "" H 7100 3400 50  0001 C CNN
F 3 "" H 7100 3400 50  0001 C CNN
	1    7100 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61988CB4
P 6200 4150
AR Path="/61988CB4" Ref="#PWR?"  Part="1" 
AR Path="/61925999/61988CB4" Ref="#PWR?"  Part="1" 
AR Path="/61997866/61988CB4" Ref="#PWR?"  Part="1" 
F 0 "#PWR?" H 6200 3900 50  0001 C CNN
F 1 "GND" H 6205 3977 50  0000 C CNN
F 2 "" H 6200 4150 50  0001 C CNN
F 3 "" H 6200 4150 50  0001 C CNN
	1    6200 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 3900 6200 3850
Connection ~ 6200 3900
Wire Wire Line
	6200 3950 6200 3900
Wire Wire Line
	6500 3900 6200 3900
Wire Wire Line
	6500 4700 6500 3900
Wire Wire Line
	6600 4700 6500 4700
NoConn ~ 6600 4200
NoConn ~ 6600 4100
NoConn ~ 7800 5000
NoConn ~ 7800 4900
NoConn ~ 7800 4800
NoConn ~ 7800 4700
Text HLabel 8100 4000 2    50   Output ~ 0
RTS
Text HLabel 8100 4100 2    50   Output ~ 0
RXD
Text HLabel 8100 4200 2    50   Output ~ 0
TXD
Text HLabel 8100 4400 2    50   Output ~ 0
DTR
Connection ~ 4550 4250
Wire Wire Line
	4550 4250 4550 4300
$EndSCHEMATC