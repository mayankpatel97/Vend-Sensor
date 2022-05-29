EESchema Schematic File Version 4
LIBS:VendSensor-cache
EELAYER 29 0
EELAYER END
$Descr A3 16535 11693
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
L Diode:1N4007 D1
U 1 1 5DB2E3B0
P 7750 9425
F 0 "D1" H 7750 9209 50  0000 C CNN
F 1 "M7" H 7750 9300 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 7750 9250 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 7750 9425 50  0001 C CNN
	1    7750 9425
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C16
U 1 1 5DB30080
P 8050 9675
F 0 "C16" H 7800 9650 50  0000 L CNN
F 1 "100uF/25V" H 7600 9550 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 8088 9525 50  0001 C CNN
F 3 "~" H 8050 9675 50  0001 C CNN
	1    8050 9675
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 9525 8050 9425
Wire Wire Line
	7900 9425 8050 9425
Connection ~ 8050 9425
Wire Wire Line
	8050 9925 8050 9825
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5DBA4457
P 8050 9300
F 0 "#FLG02" H 8050 9375 50  0001 C CNN
F 1 "PWR_FLAG" H 8050 9473 50  0000 C CNN
F 2 "" H 8050 9300 50  0001 C CNN
F 3 "~" H 8050 9300 50  0001 C CNN
	1    8050 9300
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5DBA5D63
P 7825 9925
F 0 "#FLG01" H 7825 10000 50  0001 C CNN
F 1 "PWR_FLAG" V 7825 10052 50  0000 L CNN
F 2 "" H 7825 9925 50  0001 C CNN
F 3 "~" H 7825 9925 50  0001 C CNN
	1    7825 9925
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8050 9300 8050 9425
Wire Wire Line
	8050 9925 7825 9925
Wire Wire Line
	7150 9425 7600 9425
$Comp
L power:GND #PWR055
U 1 1 5E8192B2
P 7225 9675
F 0 "#PWR055" H 7225 9425 50  0001 C CNN
F 1 "GND" H 7230 9502 50  0000 C CNN
F 2 "" H 7225 9675 50  0001 C CNN
F 3 "" H 7225 9675 50  0001 C CNN
	1    7225 9675
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 9625 7225 9625
Wire Wire Line
	7225 9625 7225 9675
$Comp
L Interface_Optical:TSOP38G36 U1
U 1 1 5FBBC86C
P 1800 1425
F 0 "U1" H 1788 1850 50  0000 C CNN
F 1 "TSSP4056" H 1788 1759 50  0000 C CNN
F 2 "Apogee:tssp4056" H 1750 1050 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82731/tsop38g36.pdf" H 2450 1725 50  0001 C CNN
	1    1800 1425
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1625 2325 1625
Wire Wire Line
	2325 1625 2325 1725
$Comp
L power:GND #PWR018
U 1 1 5FBBE151
P 2325 1725
F 0 "#PWR018" H 2325 1475 50  0001 C CNN
F 1 "GND" H 2330 1552 50  0000 C CNN
F 2 "" H 2325 1725 50  0001 C CNN
F 3 "" H 2325 1725 50  0001 C CNN
	1    2325 1725
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5FBBE400
P 1025 1375
F 0 "C1" H 1075 1300 50  0000 L CNN
F 1 "0.1uF" H 1050 1225 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1063 1225 50  0001 C CNN
F 3 "~" H 1025 1375 50  0001 C CNN
	1    1025 1375
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR017
U 1 1 5FBC17D4
P 2325 1075
F 0 "#PWR017" H 2325 925 50  0001 C CNN
F 1 "+3.3V" H 2340 1248 50  0000 C CNN
F 2 "" H 2325 1075 50  0001 C CNN
F 3 "" H 2325 1075 50  0001 C CNN
	1    2325 1075
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 1225 2325 1225
$Comp
L power:GND #PWR02
U 1 1 5FBC4AF1
P 1025 1525
F 0 "#PWR02" H 1025 1275 50  0001 C CNN
F 1 "GND" H 1030 1352 50  0000 C CNN
F 2 "" H 1025 1525 50  0001 C CNN
F 3 "" H 1025 1525 50  0001 C CNN
	1    1025 1525
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 5FBC547E
P 1025 1225
F 0 "#PWR01" H 1025 1075 50  0001 C CNN
F 1 "+3.3V" H 1040 1398 50  0000 C CNN
F 2 "" H 1025 1225 50  0001 C CNN
F 3 "" H 1025 1225 50  0001 C CNN
	1    1025 1225
	1    0    0    -1  
$EndComp
Text GLabel 2450 1425 2    50   Input ~ 0
IR1
Wire Wire Line
	2200 1425 2450 1425
Wire Wire Line
	2325 1075 2325 1225
$Comp
L Interface_Optical:TSOP38G36 U2
U 1 1 5FBCF6EC
P 1825 2650
F 0 "U2" H 1813 3075 50  0000 C CNN
F 1 "TSSP4056" H 1813 2984 50  0000 C CNN
F 2 "Apogee:tssp4056" H 1775 2275 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82731/tsop38g36.pdf" H 2475 2950 50  0001 C CNN
	1    1825 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2225 2850 2350 2850
Wire Wire Line
	2350 2850 2350 2950
$Comp
L power:GND #PWR020
U 1 1 5FBCF6F8
P 2350 2950
F 0 "#PWR020" H 2350 2700 50  0001 C CNN
F 1 "GND" H 2355 2777 50  0000 C CNN
F 2 "" H 2350 2950 50  0001 C CNN
F 3 "" H 2350 2950 50  0001 C CNN
	1    2350 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5FBCF702
P 1050 2600
F 0 "C2" H 1100 2525 50  0000 L CNN
F 1 "0.1uF" H 1075 2450 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1088 2450 50  0001 C CNN
F 3 "~" H 1050 2600 50  0001 C CNN
	1    1050 2600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR019
U 1 1 5FBCF70C
P 2350 2300
F 0 "#PWR019" H 2350 2150 50  0001 C CNN
F 1 "+3.3V" H 2365 2473 50  0000 C CNN
F 2 "" H 2350 2300 50  0001 C CNN
F 3 "" H 2350 2300 50  0001 C CNN
	1    2350 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2225 2450 2350 2450
$Comp
L power:GND #PWR04
U 1 1 5FBCF717
P 1050 2750
F 0 "#PWR04" H 1050 2500 50  0001 C CNN
F 1 "GND" H 1055 2577 50  0000 C CNN
F 2 "" H 1050 2750 50  0001 C CNN
F 3 "" H 1050 2750 50  0001 C CNN
	1    1050 2750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR03
U 1 1 5FBCF721
P 1050 2450
F 0 "#PWR03" H 1050 2300 50  0001 C CNN
F 1 "+3.3V" H 1065 2623 50  0000 C CNN
F 2 "" H 1050 2450 50  0001 C CNN
F 3 "" H 1050 2450 50  0001 C CNN
	1    1050 2450
	1    0    0    -1  
$EndComp
Text GLabel 2475 2650 2    50   Input ~ 0
IR2
Wire Wire Line
	2225 2650 2475 2650
Wire Wire Line
	2350 2300 2350 2450
$Comp
L Interface_Optical:TSOP38G36 U3
U 1 1 5FBD498A
P 1825 3925
F 0 "U3" H 1813 4350 50  0000 C CNN
F 1 "TSSP4056" H 1813 4259 50  0000 C CNN
F 2 "Apogee:tssp4056" H 1775 3550 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82731/tsop38g36.pdf" H 2475 4225 50  0001 C CNN
	1    1825 3925
	1    0    0    -1  
$EndComp
Wire Wire Line
	2225 4125 2350 4125
Wire Wire Line
	2350 4125 2350 4225
$Comp
L power:GND #PWR022
U 1 1 5FBD4996
P 2350 4225
F 0 "#PWR022" H 2350 3975 50  0001 C CNN
F 1 "GND" H 2355 4052 50  0000 C CNN
F 2 "" H 2350 4225 50  0001 C CNN
F 3 "" H 2350 4225 50  0001 C CNN
	1    2350 4225
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5FBD49A0
P 1050 3875
F 0 "C3" H 1100 3800 50  0000 L CNN
F 1 "0.1uF" H 1075 3725 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1088 3725 50  0001 C CNN
F 3 "~" H 1050 3875 50  0001 C CNN
	1    1050 3875
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR021
U 1 1 5FBD49AA
P 2350 3575
F 0 "#PWR021" H 2350 3425 50  0001 C CNN
F 1 "+3.3V" H 2365 3748 50  0000 C CNN
F 2 "" H 2350 3575 50  0001 C CNN
F 3 "" H 2350 3575 50  0001 C CNN
	1    2350 3575
	1    0    0    -1  
$EndComp
Wire Wire Line
	2225 3725 2350 3725
$Comp
L power:GND #PWR06
U 1 1 5FBD49B5
P 1050 4025
F 0 "#PWR06" H 1050 3775 50  0001 C CNN
F 1 "GND" H 1055 3852 50  0000 C CNN
F 2 "" H 1050 4025 50  0001 C CNN
F 3 "" H 1050 4025 50  0001 C CNN
	1    1050 4025
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 5FBD49BF
P 1050 3725
F 0 "#PWR05" H 1050 3575 50  0001 C CNN
F 1 "+3.3V" H 1065 3898 50  0000 C CNN
F 2 "" H 1050 3725 50  0001 C CNN
F 3 "" H 1050 3725 50  0001 C CNN
	1    1050 3725
	1    0    0    -1  
$EndComp
Text GLabel 2475 3925 2    50   Input ~ 0
IR3
Wire Wire Line
	2225 3925 2475 3925
Wire Wire Line
	2350 3575 2350 3725
$Comp
L Interface_Optical:TSOP38G36 U5
U 1 1 5FBD49CC
P 1850 5150
F 0 "U5" H 1838 5575 50  0000 C CNN
F 1 "TSSP4056" H 1838 5484 50  0000 C CNN
F 2 "Apogee:tssp4056" H 1800 4775 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82731/tsop38g36.pdf" H 2500 5450 50  0001 C CNN
	1    1850 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 5350 2375 5350
Wire Wire Line
	2375 5350 2375 5450
$Comp
L power:GND #PWR026
U 1 1 5FBD49D8
P 2375 5450
F 0 "#PWR026" H 2375 5200 50  0001 C CNN
F 1 "GND" H 2380 5277 50  0000 C CNN
F 2 "" H 2375 5450 50  0001 C CNN
F 3 "" H 2375 5450 50  0001 C CNN
	1    2375 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5FBD49E2
P 1075 5100
F 0 "C5" H 1125 5025 50  0000 L CNN
F 1 "0.1uF" H 1100 4950 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1113 4950 50  0001 C CNN
F 3 "~" H 1075 5100 50  0001 C CNN
	1    1075 5100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR025
U 1 1 5FBD49EC
P 2375 4800
F 0 "#PWR025" H 2375 4650 50  0001 C CNN
F 1 "+3.3V" H 2390 4973 50  0000 C CNN
F 2 "" H 2375 4800 50  0001 C CNN
F 3 "" H 2375 4800 50  0001 C CNN
	1    2375 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 4950 2375 4950
$Comp
L power:GND #PWR010
U 1 1 5FBD49F7
P 1075 5250
F 0 "#PWR010" H 1075 5000 50  0001 C CNN
F 1 "GND" H 1080 5077 50  0000 C CNN
F 2 "" H 1075 5250 50  0001 C CNN
F 3 "" H 1075 5250 50  0001 C CNN
	1    1075 5250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5FBD4A01
P 1075 4950
F 0 "#PWR09" H 1075 4800 50  0001 C CNN
F 1 "+3.3V" H 1090 5123 50  0000 C CNN
F 2 "" H 1075 4950 50  0001 C CNN
F 3 "" H 1075 4950 50  0001 C CNN
	1    1075 4950
	1    0    0    -1  
$EndComp
Text GLabel 2500 5150 2    50   Input ~ 0
IR4
Wire Wire Line
	2250 5150 2500 5150
Wire Wire Line
	2375 4800 2375 4950
$Comp
L Interface_Optical:TSOP38G36 U4
U 1 1 5FBED703
P 1825 6575
F 0 "U4" H 1813 7000 50  0000 C CNN
F 1 "TSSP4056" H 1813 6909 50  0000 C CNN
F 2 "Apogee:tssp4056" H 1775 6200 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82731/tsop38g36.pdf" H 2475 6875 50  0001 C CNN
	1    1825 6575
	1    0    0    -1  
$EndComp
Wire Wire Line
	2225 6775 2350 6775
Wire Wire Line
	2350 6775 2350 6875
$Comp
L power:GND #PWR024
U 1 1 5FBED70F
P 2350 6875
F 0 "#PWR024" H 2350 6625 50  0001 C CNN
F 1 "GND" H 2355 6702 50  0000 C CNN
F 2 "" H 2350 6875 50  0001 C CNN
F 3 "" H 2350 6875 50  0001 C CNN
	1    2350 6875
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5FBED719
P 1050 6525
F 0 "C4" H 1100 6450 50  0000 L CNN
F 1 "0.1uF" H 1075 6375 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1088 6375 50  0001 C CNN
F 3 "~" H 1050 6525 50  0001 C CNN
	1    1050 6525
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR023
U 1 1 5FBED723
P 2350 6225
F 0 "#PWR023" H 2350 6075 50  0001 C CNN
F 1 "+3.3V" H 2365 6398 50  0000 C CNN
F 2 "" H 2350 6225 50  0001 C CNN
F 3 "" H 2350 6225 50  0001 C CNN
	1    2350 6225
	1    0    0    -1  
$EndComp
Wire Wire Line
	2225 6375 2350 6375
$Comp
L power:GND #PWR08
U 1 1 5FBED72E
P 1050 6675
F 0 "#PWR08" H 1050 6425 50  0001 C CNN
F 1 "GND" H 1055 6502 50  0000 C CNN
F 2 "" H 1050 6675 50  0001 C CNN
F 3 "" H 1050 6675 50  0001 C CNN
	1    1050 6675
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR07
U 1 1 5FBED738
P 1050 6375
F 0 "#PWR07" H 1050 6225 50  0001 C CNN
F 1 "+3.3V" H 1065 6548 50  0000 C CNN
F 2 "" H 1050 6375 50  0001 C CNN
F 3 "" H 1050 6375 50  0001 C CNN
	1    1050 6375
	1    0    0    -1  
$EndComp
Text GLabel 2475 6575 2    50   Input ~ 0
IR5
Wire Wire Line
	2225 6575 2475 6575
Wire Wire Line
	2350 6225 2350 6375
$Comp
L Interface_Optical:TSOP38G36 U6
U 1 1 5FBED745
P 1850 7800
F 0 "U6" H 1838 8225 50  0000 C CNN
F 1 "TSSP4056" H 1838 8134 50  0000 C CNN
F 2 "Apogee:tssp4056" H 1800 7425 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82731/tsop38g36.pdf" H 2500 8100 50  0001 C CNN
	1    1850 7800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 8000 2375 8000
Wire Wire Line
	2375 8000 2375 8100
$Comp
L power:GND #PWR028
U 1 1 5FBED751
P 2375 8100
F 0 "#PWR028" H 2375 7850 50  0001 C CNN
F 1 "GND" H 2380 7927 50  0000 C CNN
F 2 "" H 2375 8100 50  0001 C CNN
F 3 "" H 2375 8100 50  0001 C CNN
	1    2375 8100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5FBED75B
P 1075 7750
F 0 "C6" H 1125 7675 50  0000 L CNN
F 1 "0.1uF" H 1100 7600 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1113 7600 50  0001 C CNN
F 3 "~" H 1075 7750 50  0001 C CNN
	1    1075 7750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR027
U 1 1 5FBED765
P 2375 7450
F 0 "#PWR027" H 2375 7300 50  0001 C CNN
F 1 "+3.3V" H 2390 7623 50  0000 C CNN
F 2 "" H 2375 7450 50  0001 C CNN
F 3 "" H 2375 7450 50  0001 C CNN
	1    2375 7450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 7600 2375 7600
$Comp
L power:GND #PWR012
U 1 1 5FBED770
P 1075 7900
F 0 "#PWR012" H 1075 7650 50  0001 C CNN
F 1 "GND" H 1080 7727 50  0000 C CNN
F 2 "" H 1075 7900 50  0001 C CNN
F 3 "" H 1075 7900 50  0001 C CNN
	1    1075 7900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 5FBED77A
P 1075 7600
F 0 "#PWR011" H 1075 7450 50  0001 C CNN
F 1 "+3.3V" H 1090 7773 50  0000 C CNN
F 2 "" H 1075 7600 50  0001 C CNN
F 3 "" H 1075 7600 50  0001 C CNN
	1    1075 7600
	1    0    0    -1  
$EndComp
Text GLabel 2500 7800 2    50   Input ~ 0
IR6
Wire Wire Line
	2250 7800 2500 7800
Wire Wire Line
	2375 7450 2375 7600
$Comp
L Interface_Optical:TSOP38G36 U7
U 1 1 5FBED787
P 1850 9075
F 0 "U7" H 1838 9500 50  0000 C CNN
F 1 "TSSP4056" H 1838 9409 50  0000 C CNN
F 2 "Apogee:tssp4056" H 1800 8700 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82731/tsop38g36.pdf" H 2500 9375 50  0001 C CNN
	1    1850 9075
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 9275 2375 9275
Wire Wire Line
	2375 9275 2375 9375
$Comp
L power:GND #PWR030
U 1 1 5FBED793
P 2375 9375
F 0 "#PWR030" H 2375 9125 50  0001 C CNN
F 1 "GND" H 2380 9202 50  0000 C CNN
F 2 "" H 2375 9375 50  0001 C CNN
F 3 "" H 2375 9375 50  0001 C CNN
	1    2375 9375
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5FBED79D
P 1075 9025
F 0 "C7" H 1125 8950 50  0000 L CNN
F 1 "0.1uF" H 1100 8875 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1113 8875 50  0001 C CNN
F 3 "~" H 1075 9025 50  0001 C CNN
	1    1075 9025
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR029
U 1 1 5FBED7A7
P 2375 8725
F 0 "#PWR029" H 2375 8575 50  0001 C CNN
F 1 "+3.3V" H 2390 8898 50  0000 C CNN
F 2 "" H 2375 8725 50  0001 C CNN
F 3 "" H 2375 8725 50  0001 C CNN
	1    2375 8725
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 8875 2375 8875
$Comp
L power:GND #PWR014
U 1 1 5FBED7B2
P 1075 9175
F 0 "#PWR014" H 1075 8925 50  0001 C CNN
F 1 "GND" H 1080 9002 50  0000 C CNN
F 2 "" H 1075 9175 50  0001 C CNN
F 3 "" H 1075 9175 50  0001 C CNN
	1    1075 9175
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR013
U 1 1 5FBED7BC
P 1075 8875
F 0 "#PWR013" H 1075 8725 50  0001 C CNN
F 1 "+3.3V" H 1090 9048 50  0000 C CNN
F 2 "" H 1075 8875 50  0001 C CNN
F 3 "" H 1075 8875 50  0001 C CNN
	1    1075 8875
	1    0    0    -1  
$EndComp
Text GLabel 2500 9075 2    50   Input ~ 0
IR7
Wire Wire Line
	2250 9075 2500 9075
Wire Wire Line
	2375 8725 2375 8875
$Comp
L Interface_Optical:TSOP38G36 U8
U 1 1 5FBED7C9
P 1875 10300
F 0 "U8" H 1863 10725 50  0000 C CNN
F 1 "TSSP4056" H 1863 10634 50  0000 C CNN
F 2 "Apogee:tssp4056" H 1825 9925 50  0001 C CNN
F 3 "http://www.vishay.com/docs/82731/tsop38g36.pdf" H 2525 10600 50  0001 C CNN
	1    1875 10300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2275 10500 2400 10500
Wire Wire Line
	2400 10500 2400 10600
$Comp
L power:GND #PWR032
U 1 1 5FBED7D5
P 2400 10600
F 0 "#PWR032" H 2400 10350 50  0001 C CNN
F 1 "GND" H 2405 10427 50  0000 C CNN
F 2 "" H 2400 10600 50  0001 C CNN
F 3 "" H 2400 10600 50  0001 C CNN
	1    2400 10600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5FBED7DF
P 1100 10250
F 0 "C8" H 1150 10175 50  0000 L CNN
F 1 "0.1uF" H 1125 10100 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1138 10100 50  0001 C CNN
F 3 "~" H 1100 10250 50  0001 C CNN
	1    1100 10250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR031
U 1 1 5FBED7E9
P 2400 9950
F 0 "#PWR031" H 2400 9800 50  0001 C CNN
F 1 "+3.3V" H 2415 10123 50  0000 C CNN
F 2 "" H 2400 9950 50  0001 C CNN
F 3 "" H 2400 9950 50  0001 C CNN
	1    2400 9950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2275 10100 2400 10100
$Comp
L power:GND #PWR016
U 1 1 5FBED7F4
P 1100 10400
F 0 "#PWR016" H 1100 10150 50  0001 C CNN
F 1 "GND" H 1105 10227 50  0000 C CNN
F 2 "" H 1100 10400 50  0001 C CNN
F 3 "" H 1100 10400 50  0001 C CNN
	1    1100 10400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR015
U 1 1 5FBED7FE
P 1100 10100
F 0 "#PWR015" H 1100 9950 50  0001 C CNN
F 1 "+3.3V" H 1115 10273 50  0000 C CNN
F 2 "" H 1100 10100 50  0001 C CNN
F 3 "" H 1100 10100 50  0001 C CNN
	1    1100 10100
	1    0    0    -1  
$EndComp
Text GLabel 2525 10300 2    50   Input ~ 0
IR8
Wire Wire Line
	2275 10300 2525 10300
Wire Wire Line
	2400 9950 2400 10100
Text GLabel 13175 2975 2    50   Input ~ 0
IR1
Text GLabel 13175 3075 2    50   Input ~ 0
IR2
Text GLabel 11975 3175 0    50   Input ~ 0
IR3
Text GLabel 11975 3275 0    50   Input ~ 0
IR4
Text GLabel 11975 3375 0    50   Input ~ 0
IR5
Text GLabel 11975 3475 0    50   Input ~ 0
IR6
Text GLabel 11975 3575 0    50   Input ~ 0
IR7
Text GLabel 11975 3675 0    50   Input ~ 0
IR8
Text GLabel 11975 3975 0    50   Input ~ 0
IR9
Text GLabel 11975 4075 0    50   Input ~ 0
IR10
Text GLabel 11975 4175 0    50   Input ~ 0
IR11
Text GLabel 11975 4275 0    50   Input ~ 0
IR12
$Comp
L MCU_ST_STM32F0:STM32F030C8Tx U15
U 1 1 5FC20027
P 12575 3075
F 0 "U15" H 12950 1475 50  0000 C CNN
F 1 "STM32F030C8Tx" H 13175 1375 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 12075 1575 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00088500.pdf" H 12575 3075 50  0001 C CNN
	1    12575 3075
	1    0    0    -1  
$EndComp
Wire Wire Line
	12475 1475 12475 1350
Wire Wire Line
	12475 1350 12575 1350
Wire Wire Line
	12675 1350 12675 1475
Wire Wire Line
	12575 1475 12575 1350
Connection ~ 12575 1350
Wire Wire Line
	12575 1350 12675 1350
Wire Wire Line
	12575 1350 12575 1225
$Comp
L Device:C C24
U 1 1 5FC2E0B7
P 14075 1125
F 0 "C24" H 14125 1050 50  0000 L CNN
F 1 "0.1uF" H 14100 975 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 14113 975 50  0001 C CNN
F 3 "~" H 14075 1125 50  0001 C CNN
	1    14075 1125
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR079
U 1 1 5FC2E0C1
P 14075 1275
F 0 "#PWR079" H 14075 1025 50  0001 C CNN
F 1 "GND" H 14080 1102 50  0000 C CNN
F 2 "" H 14075 1275 50  0001 C CNN
F 3 "" H 14075 1275 50  0001 C CNN
	1    14075 1275
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR078
U 1 1 5FC2E0CB
P 14075 975
F 0 "#PWR078" H 14075 825 50  0001 C CNN
F 1 "+3.3V" H 14090 1148 50  0000 C CNN
F 2 "" H 14075 975 50  0001 C CNN
F 3 "" H 14075 975 50  0001 C CNN
	1    14075 975 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C23
U 1 1 5FC30F81
P 13750 1125
F 0 "C23" H 13800 1050 50  0000 L CNN
F 1 "0.1uF" H 13775 975 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 13788 975 50  0001 C CNN
F 3 "~" H 13750 1125 50  0001 C CNN
	1    13750 1125
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR076
U 1 1 5FC30F8B
P 13750 1275
F 0 "#PWR076" H 13750 1025 50  0001 C CNN
F 1 "GND" H 13755 1102 50  0000 C CNN
F 2 "" H 13750 1275 50  0001 C CNN
F 3 "" H 13750 1275 50  0001 C CNN
	1    13750 1275
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR075
U 1 1 5FC30F95
P 13750 975
F 0 "#PWR075" H 13750 825 50  0001 C CNN
F 1 "+3.3V" H 13765 1148 50  0000 C CNN
F 2 "" H 13750 975 50  0001 C CNN
F 3 "" H 13750 975 50  0001 C CNN
	1    13750 975 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C22
U 1 1 5FC33B83
P 13425 1125
F 0 "C22" H 13475 1050 50  0000 L CNN
F 1 "0.1uF" H 13450 975 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 13463 975 50  0001 C CNN
F 3 "~" H 13425 1125 50  0001 C CNN
	1    13425 1125
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR074
U 1 1 5FC33B8D
P 13425 1275
F 0 "#PWR074" H 13425 1025 50  0001 C CNN
F 1 "GND" H 13430 1102 50  0000 C CNN
F 2 "" H 13425 1275 50  0001 C CNN
F 3 "" H 13425 1275 50  0001 C CNN
	1    13425 1275
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR073
U 1 1 5FC33B97
P 13425 975
F 0 "#PWR073" H 13425 825 50  0001 C CNN
F 1 "+3.3V" H 13440 1148 50  0000 C CNN
F 2 "" H 13425 975 50  0001 C CNN
F 3 "" H 13425 975 50  0001 C CNN
	1    13425 975 
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR071
U 1 1 5FC36474
P 12575 1225
F 0 "#PWR071" H 12575 1075 50  0001 C CNN
F 1 "+3.3V" H 12590 1398 50  0000 C CNN
F 2 "" H 12575 1225 50  0001 C CNN
F 3 "" H 12575 1225 50  0001 C CNN
	1    12575 1225
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR072
U 1 1 5FC38B4E
P 12575 4825
F 0 "#PWR072" H 12575 4575 50  0001 C CNN
F 1 "GND" H 12580 4652 50  0000 C CNN
F 2 "" H 12575 4825 50  0001 C CNN
F 3 "" H 12575 4825 50  0001 C CNN
	1    12575 4825
	1    0    0    -1  
$EndComp
Wire Wire Line
	12475 4675 12475 4750
Wire Wire Line
	12475 4750 12575 4750
Wire Wire Line
	12675 4750 12675 4675
Wire Wire Line
	12575 4675 12575 4750
Connection ~ 12575 4750
Wire Wire Line
	12575 4750 12675 4750
Wire Wire Line
	12575 4825 12575 4750
$Comp
L power:GND #PWR070
U 1 1 5FC49A70
P 11500 2000
F 0 "#PWR070" H 11500 1750 50  0001 C CNN
F 1 "GND" H 11505 1827 50  0000 C CNN
F 2 "" H 11500 2000 50  0001 C CNN
F 3 "" H 11500 2000 50  0001 C CNN
	1    11500 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR069
U 1 1 5FC4E80B
P 11350 2050
F 0 "#PWR069" H 11350 1800 50  0001 C CNN
F 1 "GND" H 11355 1877 50  0000 C CNN
F 2 "" H 11350 2050 50  0001 C CNN
F 3 "" H 11350 2050 50  0001 C CNN
	1    11350 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	11350 1675 11350 1750
Wire Wire Line
	11350 1675 11975 1675
$Comp
L Device:R_US R10
U 1 1 5FC5910E
P 11350 1425
F 0 "R10" H 11418 1471 50  0000 L CNN
F 1 "10K" H 11418 1380 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 11390 1415 50  0001 C CNN
F 3 "~" H 11350 1425 50  0001 C CNN
	1    11350 1425
	1    0    0    -1  
$EndComp
Wire Wire Line
	11350 1575 11350 1675
Connection ~ 11350 1675
Wire Wire Line
	12775 1350 12775 1475
Wire Wire Line
	12675 1350 12775 1350
Connection ~ 12675 1350
$Comp
L Device:C C21
U 1 1 5FC4E801
P 11350 1900
F 0 "C21" H 11150 2000 50  0000 L CNN
F 1 "0.1uF" H 11100 1800 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 11388 1750 50  0001 C CNN
F 3 "~" H 11350 1900 50  0001 C CNN
	1    11350 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5FC6A11A
P 7300 1900
F 0 "D3" V 7339 1782 50  0000 R CNN
F 1 "GREEN" V 7248 1782 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7300 1900 50  0001 C CNN
F 3 "~" H 7300 1900 50  0001 C CNN
	1    7300 1900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R3
U 1 1 5FC73EBE
P 7300 1600
F 0 "R3" H 7375 1650 50  0000 L CNN
F 1 "330E" H 7375 1575 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7340 1590 50  0001 C CNN
F 3 "~" H 7300 1600 50  0001 C CNN
	1    7300 1600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR056
U 1 1 5FC74CDF
P 7100 1350
F 0 "#PWR056" H 7100 1200 50  0001 C CNN
F 1 "+3.3V" H 7115 1523 50  0000 C CNN
F 2 "" H 7100 1350 50  0001 C CNN
F 3 "" H 7100 1350 50  0001 C CNN
	1    7100 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1350 7100 1400
Wire Wire Line
	7100 1400 7300 1400
Wire Wire Line
	7300 1400 7300 1450
Text GLabel 7450 2150 2    50   Input ~ 0
GREEN
Wire Wire Line
	7300 2050 7300 2150
Wire Wire Line
	7300 2150 7450 2150
$Comp
L Regulator_Linear:LD1117S33TR_SOT223 U14
U 1 1 5FCAE937
P 8775 9425
F 0 "U14" H 8775 9667 50  0000 C CNN
F 1 "LD1117S33TR_SOT223" H 8775 9576 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 8775 9625 50  0001 C CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00000544.pdf" H 8875 9175 50  0001 C CNN
	1    8775 9425
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 5FCB0DF6
P 8325 9700
F 0 "C19" H 8375 9625 50  0000 L CNN
F 1 "0.1uF" H 8350 9550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8363 9550 50  0001 C CNN
F 3 "~" H 8325 9700 50  0001 C CNN
	1    8325 9700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C20
U 1 1 5FCBA761
P 9200 9675
F 0 "C20" H 9250 9600 50  0000 L CNN
F 1 "0.1uF" H 9225 9525 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9238 9525 50  0001 C CNN
F 3 "~" H 9200 9675 50  0001 C CNN
	1    9200 9675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR066
U 1 1 5FCC5288
P 8775 9975
F 0 "#PWR066" H 8775 9725 50  0001 C CNN
F 1 "GND" H 8780 9802 50  0000 C CNN
F 2 "" H 8775 9975 50  0001 C CNN
F 3 "" H 8775 9975 50  0001 C CNN
	1    8775 9975
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR067
U 1 1 5FCC8F3C
P 9325 9300
F 0 "#PWR067" H 9325 9150 50  0001 C CNN
F 1 "+3.3V" H 9340 9473 50  0000 C CNN
F 2 "" H 9325 9300 50  0001 C CNN
F 3 "" H 9325 9300 50  0001 C CNN
	1    9325 9300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8475 9425 8325 9425
Wire Wire Line
	8325 9425 8325 9550
Wire Wire Line
	9075 9425 9200 9425
Wire Wire Line
	9325 9425 9325 9300
Wire Wire Line
	9200 9525 9200 9425
Connection ~ 9200 9425
Wire Wire Line
	9200 9425 9325 9425
Wire Wire Line
	8325 9850 8325 9925
Wire Wire Line
	8325 9925 8775 9925
Wire Wire Line
	9200 9925 9200 9825
Wire Wire Line
	8775 9725 8775 9925
Connection ~ 8775 9925
Wire Wire Line
	8775 9925 9200 9925
Wire Wire Line
	8775 9975 8775 9925
Text GLabel 11975 3875 0    50   Input ~ 0
IRLED1
Text GLabel 13175 3175 2    50   Input ~ 0
IRLED2
Text GLabel 11975 3775 0    50   Input ~ 0
IRLED3
Text GLabel 13175 3275 2    50   Input ~ 0
IRLED4
Text GLabel 13175 4075 2    50   Input ~ 0
IRLED5
Text GLabel 13175 3575 2    50   Input ~ 0
IRLED6
Text GLabel 13175 3975 2    50   Input ~ 0
IRLED7
Text GLabel 13175 3675 2    50   Input ~ 0
IRLED8
Text GLabel 13175 3875 2    50   Input ~ 0
IRLED9
Text GLabel 11975 2975 0    50   Input ~ 0
IRLED10
Text GLabel 13175 3775 2    50   Input ~ 0
IRLED11
Text GLabel 11975 3075 0    50   Input ~ 0
IRLED12
$Comp
L Device:R_US R9
U 1 1 5FCF501B
P 8300 1625
F 0 "R9" H 8375 1675 50  0000 L CNN
F 1 "330E" H 8375 1600 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8340 1615 50  0001 C CNN
F 3 "~" H 8300 1625 50  0001 C CNN
	1    8300 1625
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 1425 8300 1475
$Comp
L power:+3.3V #PWR064
U 1 1 5FCF9962
P 8300 1425
F 0 "#PWR064" H 8300 1275 50  0001 C CNN
F 1 "+3.3V" H 8315 1598 50  0000 C CNN
F 2 "" H 8300 1425 50  0001 C CNN
F 3 "" H 8300 1425 50  0001 C CNN
	1    8300 1425
	1    0    0    -1  
$EndComp
Wire Wire Line
	11500 1875 11975 1875
Wire Wire Line
	11500 1875 11500 2000
$Comp
L Device:LED D4
U 1 1 5FCF5011
P 8300 1925
F 0 "D4" V 8339 1807 50  0000 R CNN
F 1 "YELLOW" V 8248 1807 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8300 1925 50  0001 C CNN
F 3 "~" H 8300 1925 50  0001 C CNN
	1    8300 1925
	0    -1   -1   0   
$EndComp
Text GLabel 8450 2200 2    50   Input ~ 0
YELLOW
Wire Wire Line
	8300 2075 8300 2200
Wire Wire Line
	8300 2200 8450 2200
Text GLabel 11975 2275 0    50   Input ~ 0
RED
Text GLabel 11975 2575 0    50   Input ~ 0
GREEN
Text GLabel 11975 2375 0    50   Input ~ 0
YELLOW
$Comp
L Transistor_BJT:BC847 Q2
U 1 1 5FE982A8
P 8150 5150
F 0 "Q2" H 8341 5196 50  0000 L CNN
F 1 "BC847" H 8341 5105 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 8350 5075 50  0001 L CIN
F 3 "http://www.infineon.com/dgdl/Infineon-BC847SERIES_BC848SERIES_BC849SERIES_BC850SERIES-DS-v01_01-en.pdf?fileId=db3a304314dca389011541d4630a1657" H 8150 5150 50  0001 L CNN
	1    8150 5150
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_US R8
U 1 1 5FE982B2
P 8500 5150
F 0 "R8" V 8400 5100 50  0000 L CNN
F 1 "1K" V 8325 5100 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8540 5140 50  0001 C CNN
F 3 "~" H 8500 5150 50  0001 C CNN
	1    8500 5150
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR059
U 1 1 5FE982BC
P 8050 5350
F 0 "#PWR059" H 8050 5100 50  0001 C CNN
F 1 "GND" H 8055 5177 50  0000 C CNN
F 2 "" H 8050 5350 50  0001 C CNN
F 3 "" H 8050 5350 50  0001 C CNN
	1    8050 5350
	-1   0    0    -1  
$EndComp
Text GLabel 8650 5150 2    50   Input ~ 0
OUT
Text GLabel 7150 9525 2    50   Input ~ 0
OUTPUT
Text GLabel 7525 4750 0    50   Input ~ 0
OUTPUT
Text GLabel 13175 4175 2    50   Input ~ 0
OUT
$Comp
L Device:C C10
U 1 1 5FF7BEA3
P 3275 7800
F 0 "C10" H 3325 7725 50  0000 L CNN
F 1 "10uF" H 3300 7650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3313 7650 50  0001 C CNN
F 3 "~" H 3275 7800 50  0001 C CNN
	1    3275 7800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR036
U 1 1 5FF7BEAD
P 3275 7950
F 0 "#PWR036" H 3275 7700 50  0001 C CNN
F 1 "GND" H 3280 7777 50  0000 C CNN
F 2 "" H 3275 7950 50  0001 C CNN
F 3 "" H 3275 7950 50  0001 C CNN
	1    3275 7950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR035
U 1 1 5FF7BEB7
P 3275 7650
F 0 "#PWR035" H 3275 7500 50  0001 C CNN
F 1 "+3.3V" H 3290 7823 50  0000 C CNN
F 2 "" H 3275 7650 50  0001 C CNN
F 3 "" H 3275 7650 50  0001 C CNN
	1    3275 7650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5FF8B8E9
P 3000 1300
F 0 "C9" H 3050 1225 50  0000 L CNN
F 1 "10uF" H 3025 1150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3038 1150 50  0001 C CNN
F 3 "~" H 3000 1300 50  0001 C CNN
	1    3000 1300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR034
U 1 1 5FF8B8F3
P 3000 1450
F 0 "#PWR034" H 3000 1200 50  0001 C CNN
F 1 "GND" H 3005 1277 50  0000 C CNN
F 2 "" H 3000 1450 50  0001 C CNN
F 3 "" H 3000 1450 50  0001 C CNN
	1    3000 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR033
U 1 1 5FF8B8FD
P 3000 1150
F 0 "#PWR033" H 3000 1000 50  0001 C CNN
F 1 "+3.3V" H 3015 1323 50  0000 C CNN
F 2 "" H 3000 1150 50  0001 C CNN
F 3 "" H 3000 1150 50  0001 C CNN
	1    3000 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR077
U 1 1 5FDA75B8
P 9875 6675
F 0 "#PWR077" H 9875 6525 50  0001 C CNN
F 1 "+3.3V" H 9890 6848 50  0000 C CNN
F 2 "" H 9875 6675 50  0001 C CNN
F 3 "" H 9875 6675 50  0001 C CNN
	1    9875 6675
	1    0    0    -1  
$EndComp
Wire Wire Line
	9875 6775 10175 6775
Connection ~ 9875 6775
Wire Wire Line
	9875 6775 9875 6675
Wire Wire Line
	10175 6975 10125 6975
Wire Wire Line
	10175 6775 10175 6975
Wire Wire Line
	9550 6775 9875 6775
Wire Wire Line
	9550 6975 9550 6775
Wire Wire Line
	9625 6975 9550 6975
Text GLabel 10425 7375 2    50   Input ~ 0
IRLED8
Text GLabel 9325 7375 0    50   Input ~ 0
IRLED7
Text GLabel 10425 7275 2    50   Input ~ 0
IRLED6
Text GLabel 9325 7275 0    50   Input ~ 0
IRLED5
Text GLabel 10425 7175 2    50   Input ~ 0
IRLED4
Text GLabel 9325 7175 0    50   Input ~ 0
IRLED3
Text GLabel 10425 7075 2    50   Input ~ 0
IRLED2
Text GLabel 9325 7075 0    50   Input ~ 0
IRLED1
$Comp
L Mechanical:MountingHole H1
U 1 1 5FFFA422
P 12875 8875
F 0 "H1" H 12975 8921 50  0000 L CNN
F 1 "MountingHole" H 12975 8830 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 12875 8875 50  0001 C CNN
F 3 "~" H 12875 8875 50  0001 C CNN
	1    12875 8875
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 6001FFAD
P 12875 9175
F 0 "H2" H 12975 9221 50  0000 L CNN
F 1 "MountingHole" H 12975 9130 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_Pad" H 12875 9175 50  0001 C CNN
F 3 "~" H 12875 9175 50  0001 C CNN
	1    12875 9175
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C27
U 1 1 6003D828
P 10875 6625
F 0 "C27" H 10625 6600 50  0000 L CNN
F 1 "100uF/25V" H 10425 6500 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 10913 6475 50  0001 C CNN
F 3 "~" H 10875 6625 50  0001 C CNN
	1    10875 6625
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 6003E3ED
P 10875 6775
F 0 "#PWR0101" H 10875 6525 50  0001 C CNN
F 1 "GND" H 10880 6602 50  0000 C CNN
F 2 "" H 10875 6775 50  0001 C CNN
F 3 "" H 10875 6775 50  0001 C CNN
	1    10875 6775
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 6003EDBA
P 10875 6475
F 0 "#PWR0102" H 10875 6325 50  0001 C CNN
F 1 "+3.3V" H 10890 6648 50  0000 C CNN
F 2 "" H 10875 6475 50  0001 C CNN
F 3 "" H 10875 6475 50  0001 C CNN
	1    10875 6475
	1    0    0    -1  
$EndComp
Wire Notes Line
	6500 575  750  575 
Wire Notes Line
	750  575  750  11000
Wire Notes Line
	750  11000 6500 11000
Wire Notes Line
	6500 575  6500 11000
Wire Notes Line
	6675 8725 6675 11000
Wire Notes Line
	6675 11000 11425 11000
Wire Notes Line
	11425 11000 11425 8725
Wire Notes Line
	11425 8725 6675 8725
$Comp
L Connector_Generic:Conn_01x05 J3
U 1 1 5FC1FEA7
P 9800 1525
F 0 "J3" H 9718 1942 50  0000 C CNN
F 1 "Conn_01x05" H 9718 1851 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 9800 1525 50  0001 C CNN
F 3 "~" H 9800 1525 50  0001 C CNN
	1    9800 1525
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR068
U 1 1 5FC5A7D6
P 11350 1275
F 0 "#PWR068" H 11350 1125 50  0001 C CNN
F 1 "+3.3V" H 11365 1448 50  0000 C CNN
F 2 "" H 11350 1275 50  0001 C CNN
F 3 "" H 11350 1275 50  0001 C CNN
	1    11350 1275
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 5FC28A3C
P 10300 1250
F 0 "#PWR0103" H 10300 1100 50  0001 C CNN
F 1 "+3.3V" H 10315 1423 50  0000 C CNN
F 2 "" H 10300 1250 50  0001 C CNN
F 3 "" H 10300 1250 50  0001 C CNN
	1    10300 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5FC28EF3
P 10375 1575
F 0 "#PWR0104" H 10375 1325 50  0001 C CNN
F 1 "GND" H 10380 1402 50  0000 C CNN
F 2 "" H 10375 1575 50  0001 C CNN
F 3 "" H 10375 1575 50  0001 C CNN
	1    10375 1575
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 1325 10300 1325
Wire Wire Line
	10300 1325 10300 1250
Wire Wire Line
	10000 1525 10375 1525
Wire Wire Line
	10375 1525 10375 1575
Text GLabel 10000 1425 2    50   Input ~ 0
SWCLK
Text GLabel 10000 1625 2    50   Input ~ 0
SWDIO
Text GLabel 10000 1725 2    50   Input ~ 0
NRST
Text GLabel 11225 1675 0    50   Input ~ 0
NRST
Wire Wire Line
	11225 1675 11350 1675
Text GLabel 13175 4375 2    50   Input ~ 0
SWCLK
Text GLabel 13175 4275 2    50   Input ~ 0
SWDIO
$Comp
L Device:R_US R5
U 1 1 5FC53988
P 9475 7075
F 0 "R5" V 9375 7025 50  0000 L CNN
F 1 "100E" V 9300 7025 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9515 7065 50  0001 C CNN
F 3 "~" H 9475 7075 50  0001 C CNN
	1    9475 7075
	0    1    -1   0   
$EndComp
$Comp
L Device:R_US R7
U 1 1 5FC67B54
P 9475 7175
F 0 "R7" V 9375 7125 50  0000 L CNN
F 1 "100E" V 9300 7125 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9515 7165 50  0001 C CNN
F 3 "~" H 9475 7175 50  0001 C CNN
	1    9475 7175
	0    1    -1   0   
$EndComp
$Comp
L Device:R_US R11
U 1 1 5FC67E55
P 9475 7275
F 0 "R11" V 9375 7225 50  0000 L CNN
F 1 "100E" V 9300 7225 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9515 7265 50  0001 C CNN
F 3 "~" H 9475 7275 50  0001 C CNN
	1    9475 7275
	0    1    -1   0   
$EndComp
$Comp
L Device:R_US R12
U 1 1 5FC68236
P 9475 7375
F 0 "R12" V 9375 7325 50  0000 L CNN
F 1 "100E" V 9300 7325 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 9515 7365 50  0001 C CNN
F 3 "~" H 9475 7375 50  0001 C CNN
	1    9475 7375
	0    1    -1   0   
$EndComp
$Comp
L Device:R_US R15
U 1 1 5FC6FFBD
P 10275 7075
F 0 "R15" V 10175 7025 50  0000 L CNN
F 1 "100E" V 10100 7025 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 10315 7065 50  0001 C CNN
F 3 "~" H 10275 7075 50  0001 C CNN
	1    10275 7075
	0    1    -1   0   
$EndComp
$Comp
L Device:R_US R16
U 1 1 5FC705DE
P 10275 7175
F 0 "R16" V 10175 7125 50  0000 L CNN
F 1 "100E" V 10100 7125 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 10315 7165 50  0001 C CNN
F 3 "~" H 10275 7175 50  0001 C CNN
	1    10275 7175
	0    1    -1   0   
$EndComp
$Comp
L Device:R_US R17
U 1 1 5FC70840
P 10275 7275
F 0 "R17" V 10175 7225 50  0000 L CNN
F 1 "100E" V 10100 7225 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 10315 7265 50  0001 C CNN
F 3 "~" H 10275 7275 50  0001 C CNN
	1    10275 7275
	0    1    -1   0   
$EndComp
$Comp
L Device:R_US R18
U 1 1 5FC70B8B
P 10275 7375
F 0 "R18" V 10175 7325 50  0000 L CNN
F 1 "100E" V 10100 7325 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 10315 7365 50  0001 C CNN
F 3 "~" H 10275 7375 50  0001 C CNN
	1    10275 7375
	0    1    -1   0   
$EndComp
NoConn ~ 13175 3375
NoConn ~ 13175 3475
NoConn ~ 13175 4475
NoConn ~ 11975 4375
NoConn ~ 11975 2675
NoConn ~ 11975 2775
Text GLabel 11975 2075 0    50   Input ~ 0
XTAL1
Text GLabel 11975 2175 0    50   Input ~ 0
XTAL2
Wire Wire Line
	8050 4750 8050 4950
Wire Wire Line
	7525 4750 8050 4750
Wire Wire Line
	8050 9425 8325 9425
Connection ~ 8325 9425
Wire Wire Line
	8050 9925 8325 9925
Connection ~ 8050 9925
Connection ~ 8325 9925
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J2
U 1 1 6002232F
P 9825 7175
F 0 "J2" H 9875 7592 50  0000 C CNN
F 1 "Conn_02x05_Odd_Even" H 9875 7501 50  0000 C CNN
F 2 "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical" H 9825 7175 50  0001 C CNN
F 3 "~" H 9825 7175 50  0001 C CNN
	1    9825 7175
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 60056E35
P 6950 9525
F 0 "J1" H 6868 9200 50  0000 C CNN
F 1 "Conn_01x03" H 6868 9291 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-03A_1x03_P2.54mm_Vertical" H 6950 9525 50  0001 C CNN
F 3 "~" H 6950 9525 50  0001 C CNN
	1    6950 9525
	-1   0    0    1   
$EndComp
$EndSCHEMATC