This repository contains code and documentation required to reboot older Phantom Premium models (circa 2000) sold by SensAble Technologies Inc. It also contains mathematical models for kinematics, dynamics, simulation, and a high fidelity Solidoworks model and Unity visualization.

# Phantom Hardware

## DC Motor: Maxon RE 25 118743

|Data|Value|
|---|---|
|Nominal Voltage|12 V|
|No load speed|4860 rpm|
|No load current|	26.3 mA|
|Nominal speed|	3800 rpm|
|Nominal torque (max. continuous torque)|	28.6 mNm|
|Nominal current (max. continuous current)|	1.24 A|
|Stall torque|	129 mNm|
|Stall current|	5.5 A|
|Max. efficiency|	87 %|
|Terminal resistance|	2.18 Ω|
|Terminal inductance|	0.238 mH|
|Torque constant|	23.4 mNm/A|
|Speed constant|	407 rpm/V|
|Speed / torque gradient|	37.7 rpm/mNm|
|Mechanical time constant|	4.25 ms|
|Rotor inertia|	10.3 gcm²|
|Thermal resistance housing-ambient|	14 K/W|
|Thermal resistance winding-housing|	3.1 K/W|
|Thermal time constant winding|	12.0 s|
|Thermal time constant motor|	612 s|
|Ambient temperature|	-20...+85 °C|
|Max. winding temperature|	+100 °C|

## Encoder: HEDM-5500
|Data|Value|
|---|---|
|Resolution|1024 cpr|

# Phantom Pinouts

![Phantom Pinout](https://raw.githubusercontent.com/mahilab/Phantom/251c0e3e5d1103b492218ae2294e221b7b354ee5/docs/phantom_pinout.svg?token=AHBVCA2CFB6SP2ZFLN3FNQ2637EQA)

## Motor Encoders (D-Sub DA-15)

|Pin|Signal|Pin|Signal|Pin|Signal|
|---|---|---|---|---|---|
|  1| ENC1 A (black)  |  6|   | 11| ENC2 B (blue/black) |
|  2| ENC3 A (white) |  7|   | 12| GND (black/white) |
|  3| ENC2 A (red) |  8|   | 13|   |
|  4| 5V  (green) |  9| ENC1 B (green/black) | 14|   |
|  5|   | 10| ENC3 B (orange/black) | 15| |

## Motor Power (DIN 8 270° Style)

|Pin|Signal|Pin|Signal|
|---|---|---|---|
|  1| N/C    |  5| Motor 3- (black)  |
|  2| Motor 1+ (red)    |  6| Motor 2+ (white)  | 
|  3| Motor 1- (black)  |  7| Motor 2- (black)  |
|  4| Motor 3+ (green)  |  8| N/C    |

# Resources

http://medesign.seas.upenn.edu/index.php/Guides/Phantom
https://www.mitpressjournals.org/doi/pdf/10.1162/pres.17.4.327
