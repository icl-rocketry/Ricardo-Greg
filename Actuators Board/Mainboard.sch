EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
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
L power:+3.3V #PWR020
U 1 1 5DA60792
P 6250 2050
F 0 "#PWR020" H 6250 1900 50  0001 C CNN
F 1 "+3.3V" H 6265 2223 50  0000 C CNN
F 2 "" H 6250 2050 50  0001 C CNN
F 3 "" H 6250 2050 50  0001 C CNN
	1    6250 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 2050 6250 2350
$Comp
L power:GND #PWR021
U 1 1 5DA6110D
P 6250 5350
F 0 "#PWR021" H 6250 5100 50  0001 C CNN
F 1 "GND" H 6255 5177 50  0000 C CNN
F 2 "" H 6250 5350 50  0001 C CNN
F 3 "" H 6250 5350 50  0001 C CNN
	1    6250 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 5150 6250 5350
$Comp
L power:+3.3V #PWR06
U 1 1 5DA6E370
P 1200 7200
F 0 "#PWR06" H 1200 7050 50  0001 C CNN
F 1 "+3.3V" H 1215 7373 50  0000 C CNN
F 2 "" H 1200 7200 50  0001 C CNN
F 3 "" H 1200 7200 50  0001 C CNN
	1    1200 7200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5DA6FF9D
P 1200 7400
F 0 "R7" H 1259 7446 50  0000 L CNN
F 1 "10K" H 1259 7355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1200 7400 50  0001 C CNN
F 3 "~" H 1200 7400 50  0001 C CNN
	1    1200 7400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5DA70D8A
P 900 7550
F 0 "C2" V 671 7550 50  0000 C CNN
F 1 "1nF" V 762 7550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 900 7550 50  0001 C CNN
F 3 "~" H 900 7550 50  0001 C CNN
	1    900  7550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5DA7199D
P 650 7550
F 0 "#PWR03" H 650 7300 50  0001 C CNN
F 1 "GND" H 655 7377 50  0000 C CNN
F 2 "" H 650 7550 50  0001 C CNN
F 3 "" H 650 7550 50  0001 C CNN
	1    650  7550
	1    0    0    -1  
$EndComp
Wire Wire Line
	650  7550 800  7550
Text Notes 7400 7500 0    50   ~ 0
MainBoard\n
Text Notes 10600 7700 0    50   ~ 0
0.1\n\n
$Comp
L Connector:Micro_SD_Card_Det_Hirose_DM3AT J3
U 1 1 5DA7D60F
P 10200 1450
F 0 "J3" H 10150 2267 50  0000 C CNN
F 1 "Micro_SD_Card_Det" H 10150 2176 50  0000 C CNN
F 2 "Connector_Card:microSD_HC_Hirose_DM3AT-SF-PEJM5" H 12250 2150 50  0001 C CNN
F 3 "https://www.hirose.com/product/en/download_file/key_name/DM3/category/Catalog/doc_file_id/49662/?file_category_id=4&item_id=195&is_series=1" H 10200 1550 50  0001 C CNN
	1    10200 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR030
U 1 1 5DA7F40A
P 9000 900
F 0 "#PWR030" H 9000 750 50  0001 C CNN
F 1 "+3.3V" H 9015 1073 50  0000 C CNN
F 2 "" H 9000 900 50  0001 C CNN
F 3 "" H 9000 900 50  0001 C CNN
	1    9000 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 900  9000 1350
$Comp
L power:GND #PWR031
U 1 1 5DA80ED1
P 9000 2150
F 0 "#PWR031" H 9000 1900 50  0001 C CNN
F 1 "GND" H 9005 1977 50  0000 C CNN
F 2 "" H 9000 2150 50  0001 C CNN
F 3 "" H 9000 2150 50  0001 C CNN
	1    9000 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2150 9000 1550
Wire Wire Line
	9000 1550 9300 1550
Wire Wire Line
	8700 1450 9300 1450
Wire Wire Line
	8700 1650 9300 1650
Wire Wire Line
	9000 1350 9300 1350
$Comp
L power:GND #PWR036
U 1 1 5DA9E24B
P 11000 2050
F 0 "#PWR036" H 11000 1800 50  0001 C CNN
F 1 "GND" H 11005 1877 50  0000 C CNN
F 2 "" H 11000 2050 50  0001 C CNN
F 3 "" H 11000 2050 50  0001 C CNN
	1    11000 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	11000 2050 11000 1950
Text Label 7200 3850 2    50   ~ 0
MISO
Text Label 7200 3750 2    50   ~ 0
SCLK
Wire Wire Line
	8700 1250 9300 1250
Text Label 8700 1650 0    50   ~ 0
MISO
Text Label 8700 1450 0    50   ~ 0
SCLK
Text Label 8700 1250 0    50   ~ 0
MOSI
Wire Wire Line
	5450 2550 5650 2550
Text Label 5450 2550 0    50   ~ 0
EN
Text Label 1550 7550 2    50   ~ 0
EN
Text Label 7200 2550 2    50   ~ 0
BOOT
$Comp
L power:+3.3V #PWR09
U 1 1 5DAB272A
P 1800 7200
F 0 "#PWR09" H 1800 7050 50  0001 C CNN
F 1 "+3.3V" H 1815 7373 50  0000 C CNN
F 2 "" H 1800 7200 50  0001 C CNN
F 3 "" H 1800 7200 50  0001 C CNN
	1    1800 7200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R8
U 1 1 5DAB35D0
P 1800 7400
F 0 "R8" H 1859 7446 50  0000 L CNN
F 1 "10K" H 1859 7355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1800 7400 50  0001 C CNN
F 3 "~" H 1800 7400 50  0001 C CNN
	1    1800 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 7550 1800 7550
Wire Wire Line
	1800 7550 1800 7500
Text Label 2100 7550 2    50   ~ 0
BOOT
$Comp
L power:GND #PWR07
U 1 1 5DAB55F6
P 1800 6750
F 0 "#PWR07" H 1800 6500 50  0001 C CNN
F 1 "GND" H 1805 6577 50  0000 C CNN
F 2 "" H 1800 6750 50  0001 C CNN
F 3 "" H 1800 6750 50  0001 C CNN
	1    1800 6750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5DAB5946
P 2150 6700
F 0 "C3" V 1921 6700 50  0000 C CNN
F 1 "1nF" V 2012 6700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2150 6700 50  0001 C CNN
F 3 "~" H 2150 6700 50  0001 C CNN
	1    2150 6700
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R9
U 1 1 5DAB6448
P 2600 6500
F 0 "R9" V 2404 6500 50  0000 C CNN
F 1 "470" V 2495 6500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2600 6500 50  0001 C CNN
F 3 "~" H 2600 6500 50  0001 C CNN
	1    2600 6500
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 6500 1800 6700
Wire Wire Line
	2050 6700 1800 6700
Connection ~ 1800 6700
Wire Wire Line
	1800 6700 1800 6750
Wire Wire Line
	2250 6700 2500 6700
Wire Wire Line
	2500 6700 2500 6500
Connection ~ 2500 6500
Wire Wire Line
	2900 6500 2700 6500
Text Label 2900 6500 2    50   ~ 0
BOOT
$Comp
L power:GND #PWR02
U 1 1 5DABBFE1
P 650 6750
F 0 "#PWR02" H 650 6500 50  0001 C CNN
F 1 "GND" H 655 6577 50  0000 C CNN
F 2 "" H 650 6750 50  0001 C CNN
F 3 "" H 650 6750 50  0001 C CNN
	1    650  6750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5DABBFE7
P 1000 6700
F 0 "C1" V 771 6700 50  0000 C CNN
F 1 "1nF" V 862 6700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1000 6700 50  0001 C CNN
F 3 "~" H 1000 6700 50  0001 C CNN
	1    1000 6700
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5DABBFED
P 1450 6500
F 0 "R5" V 1254 6500 50  0000 C CNN
F 1 "470" V 1345 6500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1450 6500 50  0001 C CNN
F 3 "~" H 1450 6500 50  0001 C CNN
	1    1450 6500
	0    1    1    0   
$EndComp
Wire Wire Line
	650  6500 650  6700
Wire Wire Line
	900  6700 650  6700
Connection ~ 650  6700
Wire Wire Line
	650  6700 650  6750
Wire Wire Line
	1100 6700 1350 6700
Wire Wire Line
	1350 6700 1350 6500
Connection ~ 1350 6500
Wire Wire Line
	1750 6500 1550 6500
Text Label 1750 6500 2    50   ~ 0
EN
Text Label 7200 3250 2    50   ~ 0
BARO_CS
Wire Wire Line
	6850 3250 7200 3250
$Comp
L Device:LED_Small D3
U 1 1 5DB110D6
P 2600 1300
F 0 "D3" H 2600 1095 50  0000 C CNN
F 1 "TX_LED" H 2600 1186 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" V 2600 1300 50  0001 C CNN
F 3 "~" V 2600 1300 50  0001 C CNN
	1    2600 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D2
U 1 1 5DB13286
P 2600 950
F 0 "D2" H 2600 745 50  0000 C CNN
F 1 "RX_LED" H 2600 836 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" V 2600 950 50  0001 C CNN
F 3 "~" V 2600 950 50  0001 C CNN
	1    2600 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R10
U 1 1 5DB4A6E6
P 2900 950
F 0 "R10" V 2704 950 50  0000 C CNN
F 1 "470" V 2795 950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2900 950 50  0001 C CNN
F 3 "~" H 2900 950 50  0001 C CNN
	1    2900 950 
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R11
U 1 1 5DB4B3E7
P 2900 1300
F 0 "R11" V 2704 1300 50  0000 C CNN
F 1 "470" V 2795 1300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2900 1300 50  0001 C CNN
F 3 "~" H 2900 1300 50  0001 C CNN
	1    2900 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	2700 1300 2800 1300
Wire Wire Line
	2700 950  2800 950 
Wire Wire Line
	2300 950  2500 950 
Wire Wire Line
	2300 1300 2500 1300
$Comp
L power:GND #PWR012
U 1 1 5DB5396A
P 3900 1700
F 0 "#PWR012" H 3900 1450 50  0001 C CNN
F 1 "GND" H 3905 1527 50  0000 C CNN
F 2 "" H 3900 1700 50  0001 C CNN
F 3 "" H 3900 1700 50  0001 C CNN
	1    3900 1700
	1    0    0    -1  
$EndComp
Text Label 2300 950  0    50   ~ 0
RXD
Text Label 2300 1300 0    50   ~ 0
TXD
Wire Wire Line
	6850 2550 7200 2550
Wire Wire Line
	6850 2650 7200 2650
Wire Wire Line
	6850 2850 7200 2850
Wire Wire Line
	6850 4250 7200 4250
Wire Wire Line
	6850 3750 7200 3750
Wire Wire Line
	6850 3850 7200 3850
Wire Wire Line
	6850 4150 7200 4150
NoConn ~ 2850 4100
NoConn ~ 2850 4200
NoConn ~ 2850 4300
NoConn ~ 2850 4400
NoConn ~ 1650 3500
NoConn ~ 1650 3600
NoConn ~ 9300 1750
NoConn ~ 9300 1850
NoConn ~ 9300 1950
NoConn ~ 9300 1050
$Comp
L power:GND #PWR01
U 1 1 5DAD9189
P 750 4700
F 0 "#PWR01" H 750 4450 50  0001 C CNN
F 1 "GND" H 755 4527 50  0000 C CNN
F 2 "" H 750 4700 50  0001 C CNN
F 3 "" H 750 4700 50  0001 C CNN
	1    750  4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  4700 750  4600
Wire Wire Line
	750  4600 650  4600
Wire Wire Line
	1300 4000 1300 3800
Wire Wire Line
	1650 4100 1550 4100
Wire Wire Line
	1550 4100 1550 3300
Wire Wire Line
	1550 3300 1250 3300
Wire Wire Line
	1250 3350 1250 3300
Connection ~ 1250 3300
Wire Wire Line
	1250 3300 1250 3250
$Comp
L power:GND #PWR04
U 1 1 5DAEED0E
P 1250 3550
F 0 "#PWR04" H 1250 3300 50  0001 C CNN
F 1 "GND" H 1255 3377 50  0000 C CNN
F 2 "" H 1250 3550 50  0001 C CNN
F 3 "" H 1250 3550 50  0001 C CNN
	1    1250 3550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR08
U 1 1 5DAEFF21
P 2150 2800
F 0 "#PWR08" H 2150 2650 50  0001 C CNN
F 1 "+3.3V" H 2165 2973 50  0000 C CNN
F 2 "" H 2150 2800 50  0001 C CNN
F 3 "" H 2150 2800 50  0001 C CNN
	1    2150 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 2800 2150 2900
Wire Wire Line
	2150 2900 2250 2900
Connection ~ 2150 2900
Wire Wire Line
	2850 3500 3150 3500
Wire Wire Line
	2850 3600 3150 3600
Text Label 3150 3500 2    50   ~ 0
RXD
Text Label 3150 3600 2    50   ~ 0
TXD
Text Label 7200 2650 2    50   ~ 0
RXD
Text Label 7200 2850 2    50   ~ 0
TXD
$Comp
L power:GND #PWR010
U 1 1 5DB188B7
P 2350 4800
F 0 "#PWR010" H 2350 4550 50  0001 C CNN
F 1 "GND" H 2355 4627 50  0000 C CNN
F 2 "" H 2350 4800 50  0001 C CNN
F 3 "" H 2350 4800 50  0001 C CNN
	1    2350 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4800 2350 4700
Wire Wire Line
	2350 4700 2250 4700
Wire Wire Line
	1000 7550 1200 7550
Wire Wire Line
	1200 7550 1200 7500
Connection ~ 1200 7550
Wire Wire Line
	1200 7550 1550 7550
Wire Wire Line
	1800 7200 1800 7300
Wire Wire Line
	1200 7200 1200 7300
Wire Wire Line
	1650 5200 1650 5150
Wire Wire Line
	1650 5150 2100 5150
Wire Wire Line
	1650 6150 1650 6200
Wire Wire Line
	1650 6200 2100 6200
$Comp
L Device:R_Small R1
U 1 1 5DB9F807
P 1100 5400
F 0 "R1" V 904 5400 50  0000 C CNN
F 1 "10K" V 995 5400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1100 5400 50  0001 C CNN
F 3 "~" H 1100 5400 50  0001 C CNN
	1    1100 5400
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5DBA0737
P 1100 5950
F 0 "R2" V 904 5950 50  0000 C CNN
F 1 "10K" V 995 5950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1100 5950 50  0001 C CNN
F 3 "~" H 1100 5950 50  0001 C CNN
	1    1100 5950
	0    1    1    0   
$EndComp
Wire Wire Line
	1200 5950 1350 5950
Wire Wire Line
	1350 5400 1200 5400
Wire Wire Line
	1000 5950 950  5950
Wire Wire Line
	1000 5400 900  5400
Wire Wire Line
	950  5950 950  5600
Wire Wire Line
	950  5600 1650 5600
Connection ~ 950  5950
Wire Wire Line
	950  5950 800  5950
Wire Wire Line
	900  5400 900  5750
Wire Wire Line
	900  5750 1650 5750
Connection ~ 900  5400
Wire Wire Line
	900  5400 800  5400
Text Label 800  5400 0    50   ~ 0
DTR
Text Label 800  5950 0    50   ~ 0
RTS
Text Label 2100 5150 2    50   ~ 0
EN
Text Label 2100 6200 2    50   ~ 0
BOOT
$Comp
L Device:R_Small R6
U 1 1 5DBC28E4
P 1600 3000
F 0 "R6" H 1659 3046 50  0000 L CNN
F 1 "1K" H 1659 2955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1600 3000 50  0001 C CNN
F 3 "~" H 1600 3000 50  0001 C CNN
	1    1600 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2900 2150 2900
Wire Wire Line
	1600 3100 1600 3200
Wire Wire Line
	1600 3200 1650 3200
Wire Wire Line
	2850 3400 3150 3400
Wire Wire Line
	2850 3800 3150 3800
Text Label 3150 3400 2    50   ~ 0
RTS
Text Label 3150 3800 2    50   ~ 0
DTR
Wire Wire Line
	1300 3800 1400 3800
$Comp
L Power_Protection:SP0503BAHT D1
U 1 1 5DBEFF1C
P 1500 4600
F 0 "D1" H 1705 4646 50  0000 L CNN
F 1 "SP0503BAHT" H 1705 4555 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-143" H 1725 4550 50  0001 L CNN
F 3 "http://www.littelfuse.com/~/media/files/littelfuse/technical%20resources/documents/data%20sheets/sp05xxba.pdf" H 1625 4725 50  0001 C CNN
	1    1500 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 4400 1400 3800
Connection ~ 1400 3800
Wire Wire Line
	1400 3800 1650 3800
$Comp
L power:GND #PWR05
U 1 1 5DC01586
P 1500 4800
F 0 "#PWR05" H 1500 4550 50  0001 C CNN
F 1 "GND" H 1505 4627 50  0000 C CNN
F 2 "" H 1500 4800 50  0001 C CNN
F 3 "" H 1500 4800 50  0001 C CNN
	1    1500 4800
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 5DC16656
P 3150 950
F 0 "#PWR011" H 3150 800 50  0001 C CNN
F 1 "+3.3V" H 3165 1123 50  0000 C CNN
F 2 "" H 3150 950 50  0001 C CNN
F 3 "" H 3150 950 50  0001 C CNN
	1    3150 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D6
U 1 1 5DC1F7E5
P 3750 950
F 0 "D6" H 3750 745 50  0000 C CNN
F 1 "PWR_LED" H 3750 836 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" V 3750 950 50  0001 C CNN
F 3 "~" V 3750 950 50  0001 C CNN
	1    3750 950 
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R14
U 1 1 5DC1F7EB
P 3450 950
F 0 "R14" V 3254 950 50  0000 C CNN
F 1 "470" V 3345 950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 3450 950 50  0001 C CNN
F 3 "~" H 3450 950 50  0001 C CNN
	1    3450 950 
	0    1    1    0   
$EndComp
Wire Wire Line
	3550 950  3650 950 
Wire Wire Line
	3150 950  3350 950 
Wire Wire Line
	3900 950  3850 950 
$Comp
L Device:R_Small R4
U 1 1 5DAEB42F
P 1250 3450
F 0 "R4" H 1309 3496 50  0000 L CNN
F 1 "43K" H 1309 3405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1250 3450 50  0001 C CNN
F 3 "~" H 1250 3450 50  0001 C CNN
	1    1250 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5DAEAED8
P 1250 3150
F 0 "R3" H 1309 3196 50  0000 L CNN
F 1 "22K" H 1309 3105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 1250 3150 50  0001 C CNN
F 3 "~" H 1250 3150 50  0001 C CNN
	1    1250 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 4000 1300 4000
Wire Wire Line
	1050 4000 1050 3050
Wire Wire Line
	1050 3050 1250 3050
Text Label 8700 1150 0    50   ~ 0
SD_CS
Wire Wire Line
	8700 1150 9300 1150
Text Label 7200 4250 2    50   ~ 0
SD_CS
$Comp
L Sensor_Pressure:MS5607-02BA U5
U 1 1 5DAF7991
P 10300 4600
F 0 "U5" H 10630 4646 50  0000 L CNN
F 1 "MS5607-02BA" H 10630 4555 50  0000 L CNN
F 2 "Package_LGA:LGA-8_3x5mm_P1.25mm" H 10300 4600 50  0001 C CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5607-02BA03%7FB2%7Fpdf%7FEnglish%7FENG_DS_MS5607-02BA03_B2.pdf%7FCAT-BLPS0035" H 10300 4600 50  0001 C CNN
	1    10300 4600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR034
U 1 1 5DAF89F9
P 10300 4100
F 0 "#PWR034" H 10300 3950 50  0001 C CNN
F 1 "+3.3V" H 10315 4273 50  0000 C CNN
F 2 "" H 10300 4100 50  0001 C CNN
F 3 "" H 10300 4100 50  0001 C CNN
	1    10300 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 4100 10300 4200
$Comp
L power:GND #PWR035
U 1 1 5DAFAAA1
P 10300 5100
F 0 "#PWR035" H 10300 4850 50  0001 C CNN
F 1 "GND" H 10305 4927 50  0000 C CNN
F 2 "" H 10300 5100 50  0001 C CNN
F 3 "" H 10300 5100 50  0001 C CNN
	1    10300 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 5100 10300 5000
Text Label 9550 4500 0    50   ~ 0
MISO
Text Label 9550 4600 0    50   ~ 0
MOSI
Text Label 9550 4700 0    50   ~ 0
SCLK
Text Label 9550 4800 0    50   ~ 0
BARO_CS
Wire Wire Line
	9550 4500 9900 4500
Wire Wire Line
	9550 4600 9900 4600
Wire Wire Line
	9550 4700 9900 4700
Wire Wire Line
	9550 4800 9900 4800
Wire Wire Line
	6850 3650 7200 3650
Wire Wire Line
	6850 3550 7200 3550
Text Label 7200 3550 2    50   ~ 0
RADIO_RX
Text Label 7200 3650 2    50   ~ 0
RADIO_TX
$Comp
L power:GND #PWR033
U 1 1 5DB72471
P 9550 3750
F 0 "#PWR033" H 9550 3500 50  0001 C CNN
F 1 "GND" H 9555 3577 50  0000 C CNN
F 2 "" H 9550 3750 50  0001 C CNN
F 3 "" H 9550 3750 50  0001 C CNN
	1    9550 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 3650 9550 3650
Wire Wire Line
	9550 3650 9550 3750
$Comp
L power:+3.3V #PWR032
U 1 1 5DB78938
P 9550 2600
F 0 "#PWR032" H 9550 2450 50  0001 C CNN
F 1 "+3.3V" H 9565 2773 50  0000 C CNN
F 2 "" H 9550 2600 50  0001 C CNN
F 3 "" H 9550 2600 50  0001 C CNN
	1    9550 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 2600 9550 2750
Wire Wire Line
	9550 2750 9650 2750
Wire Wire Line
	9650 2950 9250 2950
Wire Wire Line
	9650 2850 9250 2850
Text Label 9250 2850 0    50   ~ 0
RADIO_RX
Text Label 9250 2950 0    50   ~ 0
RADIO_TX
$Comp
L Connector:8P8C_LED J2
U 1 1 5DB9241E
P 5850 7050
F 0 "J2" H 5850 6483 50  0000 C CNN
F 1 "8P8C_LED" H 5850 6574 50  0000 C CNN
F 2 "Connector_RJ:RJ45_Amphenol_RJHSE538X" V 5850 7075 50  0001 C CNN
F 3 "~" V 5850 7075 50  0001 C CNN
	1    5850 7050
	-1   0    0    1   
$EndComp
Wire Wire Line
	5450 6750 5100 6750
Wire Wire Line
	5450 6850 5100 6850
Text Label 5100 6750 0    50   ~ 0
CAN_HIGH
Text Label 5100 6850 0    50   ~ 0
CAN_LOW
$Comp
L RF_Module:ESP32-WROOM-32 U3
U 1 1 5DA5F4C7
P 6250 3750
F 0 "U3" H 6250 5331 50  0000 C CNN
F 1 "ESP32-WROOM-32D" H 6250 5240 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 6250 2250 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf" H 5950 3800 50  0001 C CNN
	1    6250 3750
	1    0    0    -1  
$EndComp
Text Label 7600 3950 2    50   ~ 0
SDA
Text Label 7600 4050 2    50   ~ 0
SCL
Wire Wire Line
	6850 3950 7300 3950
Wire Wire Line
	6850 4050 7400 4050
Wire Wire Line
	7400 4050 7400 3850
Connection ~ 7400 4050
Wire Wire Line
	7400 4050 7600 4050
Wire Wire Line
	7300 3950 7300 3850
Connection ~ 7300 3950
$Comp
L Device:R_Small R22
U 1 1 5DB1BC5B
P 7400 3750
F 0 "R22" H 7459 3796 50  0000 L CNN
F 1 "4.7K" H 7459 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7400 3750 50  0001 C CNN
F 3 "~" H 7400 3750 50  0001 C CNN
	1    7400 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R21
U 1 1 5DB1C9E2
P 7300 3750
F 0 "R21" H 7359 3796 50  0000 L CNN
F 1 "4.7K" H 7359 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 7300 3750 50  0001 C CNN
F 3 "~" H 7300 3750 50  0001 C CNN
	1    7300 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 3650 7350 3650
$Comp
L power:+3.3V #PWR025
U 1 1 5DB29AD6
P 7350 3600
F 0 "#PWR025" H 7350 3450 50  0001 C CNN
F 1 "+3.3V" H 7365 3773 50  0000 C CNN
F 2 "" H 7350 3600 50  0001 C CNN
F 3 "" H 7350 3600 50  0001 C CNN
	1    7350 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 3600 7350 3650
Connection ~ 7350 3650
Wire Wire Line
	7350 3650 7400 3650
Wire Wire Line
	6850 3150 7200 3150
$Comp
L power:GND #PWR018
U 1 1 5DB9DB5B
P 5300 7550
F 0 "#PWR018" H 5300 7300 50  0001 C CNN
F 1 "GND" H 5305 7377 50  0000 C CNN
F 2 "" H 5300 7550 50  0001 C CNN
F 3 "" H 5300 7550 50  0001 C CNN
	1    5300 7550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR019
U 1 1 5DBA5D7E
P 5350 7200
F 0 "#PWR019" H 5350 7050 50  0001 C CNN
F 1 "+3.3V" H 5365 7373 50  0000 C CNN
F 2 "" H 5350 7200 50  0001 C CNN
F 3 "" H 5350 7200 50  0001 C CNN
	1    5350 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 7450 5450 7450
Wire Wire Line
	6850 4650 7200 4650
Wire Wire Line
	6850 4550 7200 4550
Text Label 7200 4650 2    50   ~ 0
CAN_RX
Text Label 7200 4550 2    50   ~ 0
CAN_TX
Wire Wire Line
	3450 7450 3450 7350
Connection ~ 3450 7450
Wire Wire Line
	3050 7450 3450 7450
Wire Wire Line
	3050 7150 3050 7450
Text Label 2850 6950 0    50   ~ 0
CAN_RX
Text Label 2850 6850 0    50   ~ 0
CAN_TX
Wire Wire Line
	3050 6950 2850 6950
Wire Wire Line
	3050 6850 2850 6850
Wire Wire Line
	3450 7500 3450 7450
$Comp
L power:GND #PWR014
U 1 1 5DBF5BA5
P 3450 7500
F 0 "#PWR014" H 3450 7250 50  0001 C CNN
F 1 "GND" H 3455 7327 50  0000 C CNN
F 2 "" H 3450 7500 50  0001 C CNN
F 3 "" H 3450 7500 50  0001 C CNN
	1    3450 7500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR013
U 1 1 5DBECF96
P 3450 6400
F 0 "#PWR013" H 3450 6250 50  0001 C CNN
F 1 "+3.3V" H 3465 6573 50  0000 C CNN
F 2 "" H 3450 6400 50  0001 C CNN
F 3 "" H 3450 6400 50  0001 C CNN
	1    3450 6400
	1    0    0    -1  
$EndComp
Text Label 4150 7050 2    50   ~ 0
CAN_LOW
Text Label 4150 6950 2    50   ~ 0
CAN_HIGH
Wire Wire Line
	3850 7050 4150 7050
Text Label 7200 3150 2    50   ~ 0
IMU_CS
Text Label 7200 4150 2    50   ~ 0
MOSI
Wire Wire Line
	3900 950  3900 1700
Wire Wire Line
	3000 950  3150 950 
Connection ~ 3150 950 
Wire Wire Line
	3000 950  3000 1300
Connection ~ 3000 950 
$Sheet
S 750  950  500  150 
U 5DB9F80E
F0 "power" 50
F1 "power.sch" 50
$EndSheet
$Comp
L Device:Buzzer BZ1
U 1 1 5DBF25B1
P 8400 2450
F 0 "BZ1" H 8552 2479 50  0000 L CNN
F 1 "Buzzer" H 8552 2388 50  0000 L CNN
F 2 "Buzzer_Beeper:Buzzer_TDK_PS1240P02BT_D12.2mm_H6.5mm" V 8375 2550 50  0001 C CNN
F 3 "~" V 8375 2550 50  0001 C CNN
	1    8400 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 2750 5200 2650
Wire Wire Line
	5200 2750 5200 2850
Connection ~ 5200 2750
$Comp
L Device:R_Small R15
U 1 1 5DCA6D89
P 5150 7250
F 0 "R15" V 4954 7250 50  0000 C CNN
F 1 "470" V 5045 7250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5150 7250 50  0001 C CNN
F 3 "~" H 5150 7250 50  0001 C CNN
	1    5150 7250
	0    1    1    0   
$EndComp
$Comp
L Device:LED D7
U 1 1 5DCA786C
P 4900 7250
F 0 "D7" H 4893 7466 50  0000 C CNN
F 1 "CABLE_DETECT" H 4893 7375 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" H 4900 7250 50  0001 C CNN
F 3 "~" H 4900 7250 50  0001 C CNN
	1    4900 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 6850 6550 6850
Wire Wire Line
	6250 7450 6550 7450
Text Label 6550 6850 2    50   ~ 0
CAN_RX
Text Label 6550 7450 2    50   ~ 0
CAN_TX
$Comp
L Device:R_Small R19
U 1 1 5DCF3973
P 6250 7250
F 0 "R19" H 6309 7296 50  0000 L CNN
F 1 "470" H 6309 7205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6250 7250 50  0001 C CNN
F 3 "~" H 6250 7250 50  0001 C CNN
	1    6250 7250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R18
U 1 1 5DCF45CE
P 6250 6650
F 0 "R18" H 6309 6696 50  0000 L CNN
F 1 "470" H 6309 6605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6250 6650 50  0001 C CNN
F 3 "~" H 6250 6650 50  0001 C CNN
	1    6250 6650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR022
U 1 1 5DCF51EA
P 6250 6550
F 0 "#PWR022" H 6250 6400 50  0001 C CNN
F 1 "+3.3V" H 6265 6723 50  0000 C CNN
F 2 "" H 6250 6550 50  0001 C CNN
F 3 "" H 6250 6550 50  0001 C CNN
	1    6250 6550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR023
U 1 1 5DCF594A
P 6250 7150
F 0 "#PWR023" H 6250 7000 50  0001 C CNN
F 1 "+3.3V" H 6265 7323 50  0000 C CNN
F 2 "" H 6250 7150 50  0001 C CNN
F 3 "" H 6250 7150 50  0001 C CNN
	1    6250 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 1400 7950 1400
$Comp
L power:GND #PWR026
U 1 1 5DBE9FD9
P 7950 1950
F 0 "#PWR026" H 7950 1700 50  0001 C CNN
F 1 "GND" H 8000 1750 50  0000 C CNN
F 2 "" H 7950 1950 50  0001 C CNN
F 3 "" H 7950 1950 50  0001 C CNN
	1    7950 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 1700 7150 1700
Wire Wire Line
	7150 1700 7150 1950
Wire Wire Line
	7150 1950 7550 1950
Connection ~ 7550 1950
Wire Wire Line
	7550 1950 7550 1900
Wire Wire Line
	7900 1700 8200 1700
Wire Wire Line
	7900 1600 8200 1600
Wire Wire Line
	7200 1500 6900 1500
Wire Wire Line
	7200 1400 6900 1400
Text Label 8200 1600 2    50   ~ 0
SCLK
Text Label 8200 1700 2    50   ~ 0
MOSI
Text Label 6900 1500 0    50   ~ 0
MISO
Text Label 6900 1400 0    50   ~ 0
FLASH_CS
$Comp
L Mainboard-rescue:XBEE-iclr JP2
U 1 1 5DBE2F9A
P 10350 3150
F 0 "JP2" H 10350 3910 45  0000 C CNN
F 1 "XBEE" H 10350 3826 45  0000 C CNN
F 2 "iclr-hw:XBEE" H 10350 3750 20  0001 C CNN
F 3 "" H 10350 3150 50  0001 C CNN
F 4 "XXX-00000" H 10350 3731 60  0000 C CNN "Field4"
	1    10350 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 7450 5300 7550
Wire Wire Line
	4750 7250 4700 7250
$Comp
L power:GND #PWR015
U 1 1 5DD6E6D3
P 4700 7450
F 0 "#PWR015" H 4700 7200 50  0001 C CNN
F 1 "GND" H 4705 7277 50  0000 C CNN
F 2 "" H 4700 7450 50  0001 C CNN
F 3 "" H 4700 7450 50  0001 C CNN
	1    4700 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 7450 4700 7250
Wire Wire Line
	5250 7250 5450 7250
Wire Wire Line
	5350 7200 5350 7350
Wire Wire Line
	5350 7350 5450 7350
Wire Wire Line
	6850 4350 7200 4350
Wire Wire Line
	6850 4750 7200 4750
Wire Wire Line
	6850 4850 7200 4850
Text Label 7200 4350 2    50   ~ 0
Nuke1
Text Label 7200 4750 2    50   ~ 0
Cont1
Text Label 7200 4850 2    50   ~ 0
Cont2
$Comp
L Device:Q_NMOS_GSD Q?
U 1 1 5DE0D306
P 8200 3250
AR Path="/5DDE6CFA/5DE0D306" Ref="Q?"  Part="1" 
AR Path="/5DDE1103/5DE0D306" Ref="Q?"  Part="1" 
AR Path="/5DE0D306" Ref="Q3"  Part="1" 
F 0 "Q3" H 8406 3296 50  0000 L CNN
F 1 "PMV20ENR" H 8406 3205 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8400 3350 50  0001 C CNN
F 3 "~" H 8200 3250 50  0001 C CNN
	1    8200 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DE0D30D
P 8300 3700
AR Path="/5DDE6CFA/5DE0D30D" Ref="#PWR?"  Part="1" 
AR Path="/5DDE1103/5DE0D30D" Ref="#PWR?"  Part="1" 
AR Path="/5DE0D30D" Ref="#PWR029"  Part="1" 
F 0 "#PWR029" H 8300 3450 50  0001 C CNN
F 1 "GND" H 8305 3527 50  0000 C CNN
F 2 "" H 8300 3700 50  0001 C CNN
F 3 "" H 8300 3700 50  0001 C CNN
	1    8300 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D?
U 1 1 5DE49A6B
P 8300 2800
AR Path="/5DAD2180/5DE49A6B" Ref="D?"  Part="1" 
AR Path="/5DB4FA2A/5DE49A6B" Ref="D?"  Part="1" 
AR Path="/5DDE6CFA/5DE49A6B" Ref="D?"  Part="1" 
AR Path="/5DDE1103/5DE49A6B" Ref="D?"  Part="1" 
AR Path="/5DE49A6B" Ref="D8"  Part="1" 
F 0 "D8" V 8346 2721 50  0000 R CNN
F 1 "D_Schottky" V 8255 2721 50  0000 R CNN
F 2 "Diode_SMD:D_SOD-123F" H 8300 2800 50  0001 C CNN
F 3 "http://www.smc-diodes.com/propdf/DSS12U%20THRU%20DSS125U%20N1873%20REV.A.pdf" H 8300 2800 50  0001 C CNN
	1    8300 2800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8300 2950 8300 3050
Wire Wire Line
	8300 2550 8300 2650
Wire Wire Line
	8300 3450 8300 3700
$Comp
L power:+3.3V #PWR028
U 1 1 5DEC6485
P 8300 2350
F 0 "#PWR028" H 8300 2200 50  0001 C CNN
F 1 "+3.3V" H 8315 2523 50  0000 C CNN
F 2 "" H 8300 2350 50  0001 C CNN
F 3 "" H 8300 2350 50  0001 C CNN
	1    8300 2350
	1    0    0    -1  
$EndComp
NoConn ~ 9650 3050
NoConn ~ 9650 3150
NoConn ~ 9650 3250
NoConn ~ 9650 3350
NoConn ~ 9650 3450
NoConn ~ 9650 3550
NoConn ~ 11050 2750
NoConn ~ 11050 2850
NoConn ~ 11050 2950
NoConn ~ 11050 3050
NoConn ~ 11050 3150
NoConn ~ 11050 3250
NoConn ~ 11050 3350
NoConn ~ 11050 3450
NoConn ~ 11050 3550
NoConn ~ 11050 3650
Wire Wire Line
	3450 6400 3450 6650
$Comp
L Interface_CAN_LIN:SN65HVD230 U2
U 1 1 5DBD91FC
P 3450 6950
F 0 "U2" H 3450 7431 50  0000 C CNN
F 1 "SN65HVD230" H 3450 7340 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3450 6450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn65hvd230.pdf" H 3350 7350 50  0001 C CNN
	1    3450 6950
	1    0    0    -1  
$EndComp
NoConn ~ 3050 7050
NoConn ~ 5450 6950
NoConn ~ 5450 7050
NoConn ~ 5450 7150
$Comp
L Device:LED_Small D5
U 1 1 5DF0B24E
P 2600 2000
F 0 "D5" H 2600 1795 50  0000 C CNN
F 1 "TX_LED" H 2600 1886 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" V 2600 2000 50  0001 C CNN
F 3 "~" V 2600 2000 50  0001 C CNN
	1    2600 2000
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D4
U 1 1 5DF0B254
P 2600 1650
F 0 "D4" H 2600 1445 50  0000 C CNN
F 1 "RX_LED" H 2600 1536 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" V 2600 1650 50  0001 C CNN
F 3 "~" V 2600 1650 50  0001 C CNN
	1    2600 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R12
U 1 1 5DF0B25A
P 2900 1650
F 0 "R12" V 2704 1650 50  0000 C CNN
F 1 "470" V 2795 1650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2900 1650 50  0001 C CNN
F 3 "~" H 2900 1650 50  0001 C CNN
	1    2900 1650
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R13
U 1 1 5DF0B260
P 2900 2000
F 0 "R13" V 2704 2000 50  0000 C CNN
F 1 "470" V 2795 2000 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 2900 2000 50  0001 C CNN
F 3 "~" H 2900 2000 50  0001 C CNN
	1    2900 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	2700 2000 2800 2000
Wire Wire Line
	2700 1650 2800 1650
Wire Wire Line
	3000 1650 3000 2000
Connection ~ 3000 1650
Wire Wire Line
	3000 1650 3000 1300
Connection ~ 3000 1300
Text Label 2150 1650 0    50   ~ 0
RADIO_RX
Wire Wire Line
	2150 1650 2500 1650
Wire Wire Line
	2150 2000 2500 2000
Text Label 2150 2000 0    50   ~ 0
RADIO_TX
$Comp
L Mainboard-rescue:USB_B_Micro-Connector J1
U 1 1 5DF84F1A
P 750 4200
F 0 "J1" H 807 4667 50  0000 C CNN
F 1 "USB_B_Micro" H 807 4576 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Amphenol_10103594-0001LF_Horizontal" H 900 4150 50  0001 C CNN
F 3 "~" H 900 4150 50  0001 C CNN
	1    750  4200
	1    0    0    -1  
$EndComp
Connection ~ 1050 4000
Connection ~ 750  4600
Wire Wire Line
	1050 4400 1300 4400
Text Label 1300 4400 2    50   ~ 0
USB_DET
$Comp
L Device:LED_Small D17
U 1 1 5DFAB0B1
P 4100 950
F 0 "D17" H 4100 745 50  0000 C CNN
F 1 "USB_LED" H 4100 836 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric_Castellated" V 4100 950 50  0001 C CNN
F 3 "~" V 4100 950 50  0001 C CNN
	1    4100 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R36
U 1 1 5DFAB0B7
P 4400 950
F 0 "R36" V 4204 950 50  0000 C CNN
F 1 "470" V 4295 950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4400 950 50  0001 C CNN
F 3 "~" H 4400 950 50  0001 C CNN
	1    4400 950 
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 950  4300 950 
Wire Wire Line
	4000 950  3900 950 
Connection ~ 3900 950 
Wire Wire Line
	4500 950  4900 950 
Text Label 4900 950  2    50   ~ 0
USB_DET
$Comp
L Connector:Conn_01x02_Male J9
U 1 1 5DFF606F
P 1050 6300
F 0 "J9" V 1112 6344 50  0000 L CNN
F 1 "Conn_01x02_Male" V 1203 6344 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1050 6300 50  0001 C CNN
F 3 "~" H 1050 6300 50  0001 C CNN
	1    1050 6300
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x02_Male J10
U 1 1 5DFF8440
P 2200 6300
F 0 "J10" V 2262 6344 50  0000 L CNN
F 1 "Conn_01x02_Male" V 2353 6344 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 2200 6300 50  0001 C CNN
F 3 "~" H 2200 6300 50  0001 C CNN
	1    2200 6300
	0    1    1    0   
$EndComp
Wire Wire Line
	650  6500 950  6500
Wire Wire Line
	1050 6500 1350 6500
Wire Wire Line
	1800 6500 2100 6500
Wire Wire Line
	2200 6500 2500 6500
Wire Wire Line
	6850 3450 7200 3450
Text Label 7200 3450 2    50   ~ 0
FLASH_CS
Wire Wire Line
	6850 3050 7200 3050
$Comp
L power:VDD #PWR016
U 1 1 5DC7CCF8
P 4800 2650
F 0 "#PWR016" H 4800 2500 50  0001 C CNN
F 1 "VDD" H 4817 2823 50  0000 C CNN
F 2 "" H 4800 2650 50  0001 C CNN
F 3 "" H 4800 2650 50  0001 C CNN
	1    4800 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 2650 4900 2650
Wire Wire Line
	4800 2850 4900 2850
Wire Wire Line
	4800 2900 4800 2850
$Comp
L power:GND #PWR017
U 1 1 5DC615DF
P 4800 2900
F 0 "#PWR017" H 4800 2650 50  0001 C CNN
F 1 "GND" H 4805 2727 50  0000 C CNN
F 2 "" H 4800 2900 50  0001 C CNN
F 3 "" H 4800 2900 50  0001 C CNN
	1    4800 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 2850 5100 2850
Wire Wire Line
	5200 2650 5100 2650
$Comp
L Device:R_Small R17
U 1 1 5DC08F93
P 5000 2850
F 0 "R17" V 5196 2850 50  0000 C CNN
F 1 "10K" V 5105 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5000 2850 50  0001 C CNN
F 3 "~" H 5000 2850 50  0001 C CNN
	1    5000 2850
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R16
U 1 1 5DC08487
P 5000 2650
F 0 "R16" V 5196 2650 50  0000 C CNN
F 1 "10K" V 5105 2650 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 5000 2650 50  0001 C CNN
F 3 "~" H 5000 2650 50  0001 C CNN
	1    5000 2650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5200 2750 5650 2750
Text Label 7200 3050 2    50   ~ 0
LORA_CS
Connection ~ 2350 4700
$Comp
L Interface_USB:CP2102N-A01-GQFN24 U1
U 1 1 5DABCA8E
P 2250 3800
F 0 "U1" H 2250 4881 50  0000 C CNN
F 1 "CP2102N-A01-GQFN24" H 2250 4790 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-24-1EP_4x4mm_P0.5mm_EP2.6x2.6mm_ThermalVias" H 2700 3000 50  0001 L CNN
F 3 "https://www.silabs.com/documents/public/data-sheets/cp2102n-datasheet.pdf" H 2300 2750 50  0001 C CNN
	1    2250 3800
	1    0    0    -1  
$EndComp
NoConn ~ 2850 3900
NoConn ~ 2850 3700
NoConn ~ 2850 3300
NoConn ~ 2850 3200
$Comp
L Connector:TestPoint TP3
U 1 1 5E251BF2
P 3150 3400
F 0 "TP3" V 3104 3588 50  0000 L CNN
F 1 "TestPoint" V 3195 3588 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 3350 3400 50  0001 C CNN
F 3 "~" H 3350 3400 50  0001 C CNN
	1    3150 3400
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP4
U 1 1 5E252848
P 3150 3500
F 0 "TP4" V 3104 3688 50  0000 L CNN
F 1 "TestPoint" V 3195 3688 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 3350 3500 50  0001 C CNN
F 3 "~" H 3350 3500 50  0001 C CNN
	1    3150 3500
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP5
U 1 1 5E260318
P 3150 3600
F 0 "TP5" V 3104 3788 50  0000 L CNN
F 1 "TestPoint" V 3195 3788 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 3350 3600 50  0001 C CNN
F 3 "~" H 3350 3600 50  0001 C CNN
	1    3150 3600
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP6
U 1 1 5E26DF3E
P 3150 3800
F 0 "TP6" V 3104 3988 50  0000 L CNN
F 1 "TestPoint" V 3195 3988 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 3350 3800 50  0001 C CNN
F 3 "~" H 3350 3800 50  0001 C CNN
	1    3150 3800
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP7
U 1 1 5E2B5454
P 4150 6950
F 0 "TP7" V 4104 7138 50  0000 L CNN
F 1 "TestPoint" V 4195 7138 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 4350 6950 50  0001 C CNN
F 3 "~" H 4350 6950 50  0001 C CNN
	1    4150 6950
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP8
U 1 1 5E2C32DB
P 4150 7050
F 0 "TP8" V 4104 7238 50  0000 L CNN
F 1 "TestPoint" V 4195 7238 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 4350 7050 50  0001 C CNN
F 3 "~" H 4350 7050 50  0001 C CNN
	1    4150 7050
	0    1    1    0   
$EndComp
Wire Wire Line
	3850 6950 4150 6950
Wire Wire Line
	7300 3950 7600 3950
$Comp
L Connector:TestPoint TP12
U 1 1 5E3888D9
P 7600 3950
F 0 "TP12" V 7554 4138 50  0000 L CNN
F 1 "TestPoint" V 7645 4138 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7800 3950 50  0001 C CNN
F 3 "~" H 7800 3950 50  0001 C CNN
	1    7600 3950
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP13
U 1 1 5E3963BA
P 7600 4050
F 0 "TP13" V 7554 4238 50  0000 L CNN
F 1 "TestPoint" V 7645 4238 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7800 4050 50  0001 C CNN
F 3 "~" H 7800 4050 50  0001 C CNN
	1    7600 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	6750 1600 7200 1600
Wire Wire Line
	6750 1550 6750 1600
$Comp
L Connector:TestPoint TP18
U 1 1 5DEEBC25
P 7200 3450
F 0 "TP18" V 7154 3638 50  0000 L CNN
F 1 "TestPoint" V 7245 3638 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7400 3450 50  0001 C CNN
F 3 "~" H 7400 3450 50  0001 C CNN
	1    7200 3450
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP19
U 1 1 5DF070A0
P 7200 3150
F 0 "TP19" V 7154 3338 50  0000 L CNN
F 1 "TestPoint" V 7245 3338 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7400 3150 50  0001 C CNN
F 3 "~" H 7400 3150 50  0001 C CNN
	1    7200 3150
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP22
U 1 1 5DF30015
P 7200 3250
F 0 "TP22" V 7154 3438 50  0000 L CNN
F 1 "TestPoint" V 7245 3438 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7400 3250 50  0001 C CNN
F 3 "~" H 7400 3250 50  0001 C CNN
	1    7200 3250
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP20
U 1 1 5DF4B310
P 7200 4650
F 0 "TP20" V 7154 4838 50  0000 L CNN
F 1 "TestPoint" V 7245 4838 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7400 4650 50  0001 C CNN
F 3 "~" H 7400 4650 50  0001 C CNN
	1    7200 4650
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP21
U 1 1 5DF58E12
P 7200 4550
F 0 "TP21" V 7154 4738 50  0000 L CNN
F 1 "TestPoint" V 7245 4738 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7400 4550 50  0001 C CNN
F 3 "~" H 7400 4550 50  0001 C CNN
	1    7200 4550
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP2
U 1 1 5DF668A2
P 7200 4250
F 0 "TP2" V 7154 4438 50  0000 L CNN
F 1 "TestPoint" V 7245 4438 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7400 4250 50  0001 C CNN
F 3 "~" H 7400 4250 50  0001 C CNN
	1    7200 4250
	0    1    1    0   
$EndComp
$Comp
L Connector:TestPoint TP1
U 1 1 5DF9EC0F
P 7150 1700
F 0 "TP1" V 7104 1888 50  0000 L CNN
F 1 "TestPoint" V 7195 1888 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7350 1700 50  0001 C CNN
F 3 "~" H 7350 1700 50  0001 C CNN
	1    7150 1700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R23
U 1 1 5E0CD038
P 8050 1400
F 0 "R23" H 8109 1446 50  0000 L CNN
F 1 "10K" H 8109 1355 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 8050 1400 50  0001 C CNN
F 3 "~" H 8050 1400 50  0001 C CNN
	1    8050 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 1200 7950 1400
$Comp
L Mainboard-rescue:W25Q64FV-iclr U4
U 1 1 5E21AD44
P 7550 1550
F 0 "U4" H 7550 1965 50  0000 C CNN
F 1 "W25Q64FV" H 7550 1874 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 7550 1550 50  0001 C CNN
F 3 "" H 7550 1550 50  0001 C CNN
	1    7550 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 1200 8050 1300
Wire Wire Line
	7900 1500 8050 1500
Wire Wire Line
	8050 1200 7950 1200
$Comp
L Device:R_Small R20
U 1 1 5E26120B
P 6750 1450
F 0 "R20" H 6809 1496 50  0000 L CNN
F 1 "10K" H 6809 1405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 6750 1450 50  0001 C CNN
F 3 "~" H 6750 1450 50  0001 C CNN
	1    6750 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 1350 6750 1050
$Comp
L Jumper:SolderJumper_2_Open JP5
U 1 1 5E28C1E7
P 7800 850
F 0 "JP5" H 7800 1055 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 7800 964 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 7800 850 50  0001 C CNN
F 3 "~" H 7800 850 50  0001 C CNN
	1    7800 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 1050 7950 1050
Wire Wire Line
	7950 1050 7950 1200
Connection ~ 7950 1200
Wire Wire Line
	7950 1050 7950 850 
Connection ~ 7950 1050
$Comp
L Mainboard-rescue:+3.3V-Mainboard-cache #PWR024
U 1 1 5E2C7938
P 7300 800
F 0 "#PWR024" H 7300 650 50  0001 C CNN
F 1 "+3.3V" H 7315 973 50  0000 C CNN
F 2 "" H 7300 800 50  0001 C CNN
F 3 "" H 7300 800 50  0001 C CNN
	1    7300 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 800  7300 850 
Wire Wire Line
	7300 850  7650 850 
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 5E2D684C
P 7700 1950
F 0 "JP1" H 7700 2155 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 7700 2064 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_TrianglePad1.0x1.5mm" H 7700 1950 50  0001 C CNN
F 3 "~" H 7700 1950 50  0001 C CNN
	1    7700 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 1950 7950 1950
$Comp
L Connector:TestPoint TP24
U 1 1 5E31D198
P 7950 1050
F 0 "TP24" V 7904 1238 50  0000 L CNN
F 1 "TestPoint" V 7995 1238 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 8150 1050 50  0001 C CNN
F 3 "~" H 8150 1050 50  0001 C CNN
	1    7950 1050
	0    1    1    0   
$EndComp
Wire Wire Line
	6850 2950 7200 2950
Text Label 5300 2850 0    50   ~ 0
LORA_INT
Wire Wire Line
	5300 2850 5650 2850
Text Label 7200 2950 2    50   ~ 0
LORA_RESET
NoConn ~ 5650 3750
NoConn ~ 5650 3850
NoConn ~ 5650 3950
NoConn ~ 5650 4050
NoConn ~ 5650 4150
NoConn ~ 5650 4250
$Comp
L Connector:Conn_01x02_Male J12
U 1 1 5E483BF5
P 9050 2850
F 0 "J12" H 9158 3031 50  0000 C CNN
F 1 "Conn_01x02_Male" H 9158 2940 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 9050 2850 50  0001 C CNN
F 3 "~" H 9050 2850 50  0001 C CNN
	1    9050 2850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5E49D4BB
P 9400 4400
F 0 "#PWR0105" H 9400 4150 50  0001 C CNN
F 1 "GND" H 9405 4227 50  0000 C CNN
F 2 "" H 9400 4400 50  0001 C CNN
F 3 "" H 9400 4400 50  0001 C CNN
	1    9400 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 4400 9900 4400
$Comp
L Transistor_BJT:MMDT2222A Q1
U 1 1 5E4F19D5
P 1550 5400
F 0 "Q1" H 1740 5446 50  0000 L CNN
F 1 "MMDT2222A" H 1740 5355 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6" H 1750 5500 50  0001 C CNN
F 3 "http://www.diodes.com/_files/datasheets/ds30125.pdf" H 1550 5400 50  0001 C CNN
	1    1550 5400
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:MMDT2222A Q1
U 2 1 5E4F3D62
P 1550 5950
F 0 "Q1" H 1740 5996 50  0000 L CNN
F 1 "MMDT2222A" H 1740 5905 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-363_SC-70-6" H 1750 6050 50  0001 C CNN
F 3 "http://www.diodes.com/_files/datasheets/ds30125.pdf" H 1550 5950 50  0001 C CNN
	2    1550 5950
	1    0    0    1   
$EndComp
Wire Wire Line
	1050 4200 1600 4200
Wire Wire Line
	1050 4300 1500 4300
Wire Wire Line
	1500 4400 1500 4300
Connection ~ 1500 4300
Wire Wire Line
	1500 4300 1650 4300
Wire Wire Line
	1600 4400 1600 4200
Connection ~ 1600 4200
Wire Wire Line
	1600 4200 1650 4200
Wire Wire Line
	8000 3250 7800 3250
Text Label 7800 3250 0    50   ~ 0
Buzzer
Wire Wire Line
	6850 2750 7200 2750
Text Label 7200 2750 2    50   ~ 0
Buzzer
$Comp
L Connector:TestPoint TP23
U 1 1 5DF14B43
P 7200 3350
F 0 "TP23" V 7154 3538 50  0000 L CNN
F 1 "TestPoint" V 7245 3538 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7400 3350 50  0001 C CNN
F 3 "~" H 7400 3350 50  0001 C CNN
	1    7200 3350
	0    1    1    0   
$EndComp
Text Label 7200 3350 2    50   ~ 0
MAG_CS
Wire Wire Line
	7200 3350 6850 3350
Text Label 7200 4450 2    50   ~ 0
Nuke2
Wire Wire Line
	6850 4450 7200 4450
$Comp
L power:+3.3V #PWR0108
U 1 1 5E2F0B8B
P 4800 3450
F 0 "#PWR0108" H 4800 3300 50  0001 C CNN
F 1 "+3.3V" H 4815 3623 50  0000 C CNN
F 2 "" H 4800 3450 50  0001 C CNN
F 3 "" H 4800 3450 50  0001 C CNN
	1    4800 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3450 4800 3550
Connection ~ 4800 3550
$Comp
L Device:R_Small R26
U 1 1 5E30F368
P 4900 3550
F 0 "R26" V 4704 3550 50  0000 C CNN
F 1 "10K" V 4795 3550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4900 3550 50  0001 C CNN
F 3 "~" H 4900 3550 50  0001 C CNN
	1    4900 3550
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 3550 5350 3550
Text Label 5350 3550 2    50   ~ 0
BARO_CS
$Comp
L Device:R_Small R31
U 1 1 5E320DFC
P 4900 3850
F 0 "R31" V 4704 3850 50  0000 C CNN
F 1 "10K" V 4795 3850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4900 3850 50  0001 C CNN
F 3 "~" H 4900 3850 50  0001 C CNN
	1    4900 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 3850 5350 3850
Text Label 5350 3850 2    50   ~ 0
FLASH_CS
$Comp
L Device:R_Small R37
U 1 1 5E33008C
P 4900 4150
F 0 "R37" V 4704 4150 50  0000 C CNN
F 1 "10K" V 4795 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4900 4150 50  0001 C CNN
F 3 "~" H 4900 4150 50  0001 C CNN
	1    4900 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 4150 5350 4150
Text Label 5350 4150 2    50   ~ 0
IMU_CS
$Comp
L Device:R_Small R38
U 1 1 5E34F390
P 4900 4450
F 0 "R38" V 4704 4450 50  0000 C CNN
F 1 "10K" V 4795 4450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4900 4450 50  0001 C CNN
F 3 "~" H 4900 4450 50  0001 C CNN
	1    4900 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 4450 5350 4450
Text Label 5350 4450 2    50   ~ 0
LORA_CS
$Comp
L Device:R_Small R39
U 1 1 5E35EDA1
P 4900 4750
F 0 "R39" V 4704 4750 50  0000 C CNN
F 1 "10K" V 4795 4750 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4900 4750 50  0001 C CNN
F 3 "~" H 4900 4750 50  0001 C CNN
	1    4900 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 4750 5350 4750
Text Label 5350 4750 2    50   ~ 0
MAG_CS
Wire Wire Line
	4800 3550 4800 3850
Connection ~ 4800 3850
Wire Wire Line
	4800 3850 4800 4150
Connection ~ 4800 4150
Wire Wire Line
	4800 4150 4800 4450
Connection ~ 4800 4450
Wire Wire Line
	4800 4450 4800 4750
$Comp
L Device:R_Small R40
U 1 1 5E390404
P 4900 5050
F 0 "R40" V 4704 5050 50  0000 C CNN
F 1 "10K" V 4795 5050 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" H 4900 5050 50  0001 C CNN
F 3 "~" H 4900 5050 50  0001 C CNN
	1    4900 5050
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 5050 5350 5050
Text Label 5350 5050 2    50   ~ 0
SD_CS
Wire Wire Line
	4800 5050 4800 4750
Connection ~ 4800 4750
$Comp
L Connector:TestPoint TP25
U 1 1 5E3F6828
P 7200 3050
F 0 "TP25" V 7154 3238 50  0000 L CNN
F 1 "TestPoint" V 7245 3238 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_1.0x1.0mm_Drill0.5mm" H 7400 3050 50  0001 C CNN
F 3 "~" H 7400 3050 50  0001 C CNN
	1    7200 3050
	0    1    1    0   
$EndComp
$Comp
L Device:D_Schottky D9
U 1 1 5E667A26
P 1050 2900
F 0 "D9" V 1004 2979 50  0000 L CNN
F 1 "D_Schottky" V 1095 2979 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-123F" H 1050 2900 50  0001 C CNN
F 3 "~" H 1050 2900 50  0001 C CNN
	1    1050 2900
	0    1    1    0   
$EndComp
Connection ~ 1050 3050
$Comp
L power:VDD #PWR053
U 1 1 5E668CEC
P 1050 2750
F 0 "#PWR053" H 1050 2600 50  0001 C CNN
F 1 "VDD" H 1067 2923 50  0000 C CNN
F 2 "" H 1050 2750 50  0001 C CNN
F 3 "" H 1050 2750 50  0001 C CNN
	1    1050 2750
	1    0    0    -1  
$EndComp
$EndSCHEMATC
