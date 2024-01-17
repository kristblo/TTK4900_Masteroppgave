##Journal
Notes and findings made during the semester. It's easier to collect everything in one file than to have an assortment of "misc" files. Replaces the journaling style commit messages from the specialisation project.
Date format: DDMMYY


###160124
Read the USB application note AN4879. It recommends two specific components for ESD protection, both of which are availabe on Mouser. Should probably implement that. Also, a transistor is probably not necessary to drive the DP line. The figure text on fig 5 implies that the GPIO pin just needs to be taken out of reset/floating when VBUS is detected.

###170124
Looks like all the MCU changes are implementable without issues. Taking care to move optical switches and end switch detector to FT pins. Found that the 2N551 NPN BJT transistor can operate the relay. Found that the end switch and wrist optocoupler work on 5V, but measure only 2.7V on the output. Should be well over the 5VT VIH threshold. Breadboarded a circuit for the OPB sensor, works. The sensor (i.e. LED+resistor) was placed between Vcc and OUT. Activates when aperture is abstructed to measure approx 3V. Measured -0.6V before activation, not acceptable. Measured [-0.150,0.050]V before avtivation after removing the LED+resistor, acceptable. Also 5V on activation.

Radical idea: Put the shoulder IMU on the shoulder. No need to have an extra board in the upper arm, it saves on wiring and the assembly can be reused in the hand.
