##Journal
Notes and findings made during the semester. It's easier to collect everything in one file than to have an assortment of "misc" files. Replaces the journaling style commit messages from the specialisation project.
Date format: DDMMYY


###160124
Read the USB application note AN4879. It recommends two specific components for ESD protection, both of which are availabe on Mouser. Should probably implement that. Also, a transistor is probably not necessary to drive the DP line. The figure text on fig 5 implies that the GPIO pin just needs to be taken out of reset/floating when VBUS is detected.

###170124
Looks like all the MCU changes are implementable without issues. Taking care to move optical switches and end switch detector to FT pins. Found that the 2N551 NPN BJT transistor can operate the relay. Found that the end switch and wrist optocoupler work on 5V, but measure only 2.7V on the output. Should be well over the 5VT VIH threshold. Breadboarded a circuit for the OPB sensor, works. The sensor (i.e. LED+resistor) was placed between Vcc and OUT. Activates when aperture is abstructed to measure approx 3V. Measured -0.6V before activation, not acceptable. Measured [-0.150,0.050]V before avtivation after removing the LED+resistor, acceptable. Also 5V on activation.

Radical idea: Put the shoulder IMU on the shoulder. No need to have an extra board in the upper arm, it saves on wiring and the assembly can be reused in the hand.

###180124
Installed the recommended USBLC6 ESD protection from AN4879. Completed the shoulder board with few notes. Rotated the MCU 90deg ccw relative to MK1 because it fit better with the motor drivers. Not a huge impact overall, if anything it made the whole layout simpler/easier to look at. Had some trouble with the encoder connector footprints not matching the symbol's pad enumeration, switched to odd/even compared to ccw in MK1. Electrically identical ofc.

Voltage tuning resistors are connected with the other leg compared to MK1. Don't remember reassigning them in that design, but it shouldn't matter. Tested on OV.

Consider setting up I2C headers such that it can be used instead of CAN if that really doesn't work. For the hand, that probably means committing to having both IMUs on the same bus.


###190124
The added hardware complexity of being able to switch between CAN and I2C is not worth it. Would need extra jumpers (or even DIP-switches) to enable or disable all relevant pins, and introducing two IMUs on one I2C w/o testing is asking for trouble. The TTK8 breadboard tests showed that the CAN hardware is probably correct.

Changed end switch to PA5 to better fit torso layout.
Decided against finding a new TVS. I'm not sure I'd find one actually able to protect my 48V lines w/o frying the other lines. All my data lines are protected with the old one, CDSC706, and the 48V lines are to some extent protected by diodes near all the voltage regulators.


###240124
Found dc-buck ZMR330FTA for use on the IMU board. This saves one wire (or 14%!) from the interface. Still need gnd, 5v, opt, int, sda, scl, but a simple test with jumper wires indicates that this will be OK. I'm increasingly unsure of how I'm supposed to route wires inside the hand, however.

For wiring: try to rework the board outline to make space for them. The new IMU header solution was to remove the 2x2 interrupt header/jumper in favor of a 1x2 header for IMU and OPT interrupts, for a marginally smaller footprint than the previous design. The IMU board itself can have a pulldown and ENABLE jumper to pull all unused low -- plenty of space there.


###250124
Further investigation of the hand board shows that I can expand it by 2mm right and fit all the connectors on vertical headers, similar to the original ribbon between A and B. Also made a similar tab to move the OPT header ~2.5mm up.

Switched I2C ports for the IMUs such that the on-board IMU gets port 3. Fits better with the layout.

Realised the larger damping caps on the MCU were connected between the zener and its LED instead of 3V3. Make sure that's not the case on other designs!

Finished traces and silk on hand. IMPORTANT: Find a footprint for "Flush mount socket" on the encoder connectors before sending to JLC.

###260124
Decided to reuse the old sockets instead of trying to find a replacement model. Don't know enough about the type of connector etc. Made a new footprint by increasing the hole diameter of a standard 2.54mm header from 1.0mm to 1.5mm, and the pad diameter to 2.2mm to comply with JLC's requirement of 0.3mm minimum annular ring. Shifted both footprints 0.3mm left on the PCB, looked like that might be a better fit. Motivated by the observation that the sockets were not placed on the centerline between the square holes.
