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

Created Hand B as a separate Kicad project for the optical sensor in order to comply with the goal of avoiding multiboard production. Two layers, all standard settings.

###020224
Discovered that fitting all the wires inside the lower arm will be a challenge, at least w/o spaghetti wires. They don't really fit without interfering with the actuation belts. Not insurmountable, but probaby won't be able to put the covers back on without some minor modding.

###030224
Ordered the wrong CAN transcievers. Got GT instead of BT, which don't have the variable VIO pin. Will salvage the BTs from Mk1. The TRx on the test bench unit which iirc was attached to the test bench MCU is also GT, which may explain the negative result from the CAN tests.

Opened the rail board to look at the DSUB pinout, found that the 48V fill zone is unfilled due to lower priority than the surrounding zone. Will need to be scraped and soldered on arrival!

###180224
Solved the CAN transceiver problem by salvaging from the old units. Strongly regretting not ordering a full complement of voltage regulators and relays. The relays were a pain in the ass to remove, and the voltage regs now have 40 week lead time if I ever need new ones. Still got two left from the old hand. Also forgot to order the 20 pins, so that's a delay.

Something went wrong during the soldering of the shoulder, I think, as there now is a connection between the voltage regulators making them move in tandem when I tune them. Solution: this is why we isolate our heat sinks with silpads <3

All three units tuned for voltage.

Set the relays to toggle on and off with one second intervals, workSet the relays to toggle on and off with one second intervals, works.

###210224
Discovered that I've flipped the resistors of the USB voltage divider. Switched them, still no detection. I realise that I probably need to activate the DP driver somehow, but I haven't found the right place to do it -- probably some callback function.

UART works with the devkit, just had to remove two solder bridges. Makes USB less appealing.

I2C does not seem to work, might be hardware. The waveform looks very unstable, unlike previously. Possibly need a different resistor value, or reworking the solder as the IMU board connector really is full of glue. On the other hand, the hand on-board IMU does not seem to work either. Next up: recreate the test bench IMU setup and hook it up to the hand. Find a 5V-3V3 buck somewhere.

###220224
Breadboard IMU works (got whoami) from hand, shoulder IMU also works. This means that the schematic is correct, and that the configuration of both I2C1 and I2C3 is viable. Poor soldering is improbable, as two IMU boards (soldered a second one) and the hand are not all likely to fail. Next up: a thorough review of the PCB layouts to look for possible fuckery, such as bad fill zones.

###230224
Found an Adafruit replacement for the IMUs, MMA8451 breakout. 5V supply, interrupt output etc. Will bite the bullet and order two before I go home if I can't find the solution to the I2C problem.

Beeped all pads on the hand and IMU boards, all clear. Found one difference between shoulder (functional) and hand: C22 on the hand is connected to pin 5 VDDIO on hand, but not on the shoulder. Hand is technically correct, but shoulder is functional, so... Ditto for C1 on the IMU. Removing C1 did not change anything.

Tested with a new IMU on the IMU board, same result. Got one clue, however: when the breadboard unit's interrupts are left floating, it too becomes unable to respond. Unfortunately, setting the relevant GPIO pins high on the hand did not solve anythin. Got one clue, however: when the breadboard unit's interrupts are left floating, it too becomes unable to respond. Unfortunately, setting the relevant GPIO pins high on the hand did not solve anythingg. Looks like it's going to be Adafruit. Got screengrabs of the bad IMU board test and the successful breadboard test.

Z axis of shoulder also seems to be dead, but not X axis. Might it actually be that this is not my fault, and I'm just dealing with a bad batch? I'll never know for sure. The shoulder Z axis would have been optimal, but X will work too!

Motor drivers work as expected. Turns out the CCRs are unique to each channel, so the DT function currently always writes to CCR1, 2 and 3. Should probably be part of a motor driver struct in the real project.

###240224
Got CAN working. Should probably cite the youtube video? After watching the first video I simply switched to normal mode, and flashed torso and shoulder with corresponding sender IDs and filters. Now I'm even more certain that my trouble with CAN before Christmas was due to having ordered the wrong transciever.

Will need to read RM0316 chapters 8 (Peripheral interconnect matrix), 13 (DMA), 15 (ADC), 30 (UART) and 31 (bxCAN) as research for the architechture. A key point will be to offload the CPUs as much as possible, in particular CAN/UART should as far as possible avoid the CPU. Unfortunately, UART5 is not compatible with "continuous communication using DMA". Figures!

Discovered that the main connector on bogie is still wrong, most likely a 180 degree rotation around the Z axis. Other than that, all units seem to fit acceptably well. Measures will need to be taken to properly isolate the hand, such as filing down all solder points and taping conductive contact surfaces in the chassis.

###260224
Main connector was 180 degrees around the X axis, i.e. pins 1 and 11 switched places with 10 and 20, respectively. Bogie Mk1.2 changes: Edited bogie schematic by "flipping the symbol from top to bottom", and leaving everything else. Retraced PCB accordingly. Changed mount points from 3.5mm to 3.7mm. Updated silk of main connector to match new layout, version to 1.2. Now feel relatively confident that it will work.

As the rail board was missing a fill zone, I figure I might as well reorder that too. Rail Mk1.2 changes: Updated version silk, fixed fill zones and some additional stitching.

Decided to check Hand B as well, and it is faulted. Will need to debug that before ordering the next batch.

###280224
Didn't explicitly find the source of the Hand B fault, but footprint pins 1 and 2 of the optical switched were swapped. Fixed and sent Hand B, Rail and Bogie Mk1.2 to production. Also ordered full complement of parts from Mouser, don't want to spend the time trying to dismantle Mk1.1 for parts.

###060324
Bogie OK. Optical sensor works, but the MCU will always measure between OUT and GND, not Vcc and OUT as the design is made for. Solution: Put resistors between Vcc and OUT, and OUT and GND of equal size (or possibly slightly higher between OUT and GND). During normal operation, the output pin will measure 2.5V (or higher) which is sufficient for FT VIH (1.85V). When the sensor is triggered, the lower half will be shorted to ground, making the OUT voltage 0V. Configure interrupt for falling edge, and Bob's your uncle. Tested on breadboard, not irl. Soldered resistors on Hand B.

The connection from torso to rail seems OK, except the rows on the silk of the rail card are switched horisontally as the signals reach the actual DSUB connector.

Before properly getting to work, I will shorten the hand A/B connector, remove the IMU board wires, and fasten the torso power regs with silpads and nylon bolts.

###070324
Something wrong with the PWM generation, need to look up the counter period calculation stuff.

###080324
The counter period in the timer init is what actually decides the pwm fqc, CTR_PRD needs to mirror that. The higher the frequency, the lower the resolution -> Can probably survive a lower frequency than 25kHz to get more precise control than 1/2880th. This may be part of the motor characterisation and can be solved analytically, but experimentally 10kHz (7200) is also okay.

Got a funny bug trying to use strcpy to parse input strings. My parser sets all indeces in a holding buffer to 0 upon registering an enter key press. In any other case, it increases the pointer to the holding buffer and then inserts the character. This makes the first entry of the buffer always 0, which strcpy (and sprintf) interprets as EOF. Solution: insert character, then increase pointer.

Various bugs later, I have positioning over UART. Need to figure out how to safely translate full rotational range and count, i.e. overflow from the encoders. Important note: The encoder timers must be activated to function. Also, positive paadrag means the GoFWD should be called.

Will probably need to get interrupts if multiple motors are to be controlled outside of the main loop.

Forwards and backwards is not the same for all motors.

Not registering input from the bogie encoder. Could be HW.

###090324
The encoder bug was indeed HW, the 5V pin on the torso connector was not properly soldered. Neither was one of the ground pins on the bogie, but I don't think that was related.

CAN is not working. Open a new branch with a clean slate and follow the youtube video again.

###120324
Got CAN working again, as well as set up a structure for the project.
CAN: Followed the instructions carefully, and it worked. Then I replicated the setup in the main branch, and it did not work. For some reason, the rx interrupt callback wouldn't trigger. Solution: delete the generated can.c file from main, and check it back out from can\_verification onto main. Then it worked again.
NOTE: restarted cubemx, the generated code. CAN did not break, so I assume this means it is safe to continue.

Structure: my own \_driver files are in a separate folder. Modifying the Makefile between generation in cubeMx is safe. Also discovered a way to make default variables using structs and variable define statements, see can\_driver.c and can\_driver.h.

Got more trouble as soon as I started to send data. Can't find the correct data type for IDE and RTR, which corrupts the whole frame apparently. Fair enough, wasn't really interested in them anyway. They're hardcoded to their default values of STD and DATA now. Instead, I added an argument for mailbox, which I might actually end up needing. Also added the shoulder back into the loop, works.

###130324
Set up a system of function pointers to call on string commands, but having trouble passing data into them. Will probably need use pointers.

###140324
Got a ways further on the string parsing system, but passing information between entities remains a problem. I thought I'd solved motor control by having a static struct writeable by functions in the motor driver header, but apparently even changes to that fall out of scope once the interrupt handler is done (haven't been able to confirm that entirely). Consider running an "update motor power" function in main while, with the function definition in the motor driver.

###150324
Having serious trouble understanding how to use static variables for motor control, i.e. where their scope ends. The solution seems to be to make the static variable in the .c file, and local functions to use it via pointer. Then, to provide an interface outside of the .c file, make interface functions which call the driver functions without passing the static variable directly.
Ex:
motor\_driver\_set\_power(&static\_var, power) is called by
motor\_interface\_set\_power(power),
where the latter SHOULD work from any file which includes the relevant .h file.

All in the name of clean interfaces and memory protection. Additionally, I had a good time debugging the fact that writing to uart doesn't always work in main unless a delay of at least 20ms is inserted. Not sure why, but I suspect it's related to interrupt routines somehow.

Implemented encoder counter which doesn't entirely shit itself after just 16 bits. Interprets a difference in measurement of more than 1000 as a jump in the encoder register, which means it's entirely reliant on being able to measure this faster than every 2 rotations of the motor. Probably safe for low speeds, possibly entirely unsafe for higher.

###160324
Description of module function names:
module\_driver\_get\_thing(): Return a value relevant to the module, effectively only useable inside the module if using static var.
module\_driver\_set\_thing(thingVal): Set the same value, also only for internal useage.
module\_driver\_update\_thing(): Use a heuristic to automatically update the value, may or may not make use of the setter function.
module\_interface\_get\_thing(): Call the corresponding driver function. This function is avaiable outside the module, and is the intended way to interact with a module.

That is: driver functions are for module internal usage and communicate directly with hardware. Interface functions are for external usage. Why? Want to minimise the use of global variables, but also need somewhere to store data. Module internal variables (such as the motor controller descriptor) should be static, which prevents their useage outside of their compilation module (data encapsulation). This necessitates a set of interface functions not directly using the static variables.

Pinch polarity is opposite of everything else, gotta rewrite some stuff.

Rail has a movement range of approximately 145000 encoder counts.
Shoulder range approx 70000 counts.

The CAN driver function names need to be rewritten to conform to the above naming scheme. Started writing the CAN rx handler, and there needs to be multiple message handlers, possibly make another set of function pointers. Immediately, having the unique motor ids internal to the structs and thus changing with each build seems cumbersome. Arguably rewrite the current to do an ACTIVE UNIT check and select manually, as the number of motors is not subject to change. Encoding string input to the can messsages is not difficult, however.

###170324
Got CAN control working! Also fixed some of the function names. I think I should probably make the rx handler more slim, e.g. by storing data in one static uint8 array per type of command rather than parsing it in the interrupt handler directly. Could set a flag which is polled by the main function to look for new messages. However, the MCU runs at 72MHz, the CAN bus at 500kHz, and the UART at 115kHz. I ultimately don't think it'll be a problem that the interrupt handler takes too long, as long as any "real" computation happens in the main function. Besides, what is the point of the CAN FIFO if not to store unread messages?

Next priority is to make an accelerometer driver. It absolutely has to accommodate the shoulder unit, and it should be adaptable for both I2C handles on both hand and shoulder. A set of CAN and string commands should let a motor be controlled such that the accelerometer reaches a certain G measurement.

Thoughts on zero calibration, state estimation: Rail has the end switch, and I think the encoder counter is reliable enough that a range of 145000 clicks is always safe. Shoulder has accelerometer, all good. Twist has the optical sensor, and can probably rely on encoder. Pinch can be gently run until it stops in either end during a calibration phase, then rely on encoder, ditto for wrist after twist has been secured 90 deg on the joint. Elbow may be run until stop while elbow points straight up, but that is inelegant.

As a last-ditch, desperate attempt at making the Adafruits work, I could try to make a voltage divider to get the 5V down to 3.3V, which I suppose is really the "logic level" of my STMs.

###190324
Shoulder accelerometer: acceleration works for x and y axes. X is irrelevant (always 0), Y will be 0 when joint point upwards, plus/minus 1 when horizontal. Needs an offset of approximately 2500, probably due to slighty skewed installation. Rotation rate works for all axes, but only X is relevant.

Wrote a motor controller.h, it almost completely shoulder centric. The loop now works, but is completely unstable. Will need to implement something so that the motor doesn't spaz out, such as setting isMoving to 0 or simply lowering the gain even more.

###200324
Struggling to make the torso to shoulder update loop work. I seems almost random whether or not the function call stack actually results in a correct read or not, will need some debugging.

Got an angle measurement thingy:
Shoulder has -332 encoder clicks per degree forward (CCW when sitting at the desk)
Elbow: 703 clicks/deg forward CW
Wrist: 300 clicks/deg forward CCW
Twist: 155 clicks/deg forward CCW viewed from the front
Pinch: 500 clicks/mm forward opening
Rail: 95 clicks/mm forward towards the end switch

Spent a couple hours debugging a compiler error that only solved by rebooting the computer ffs. Seemed like the CAN rx interrupt read a message twice, returning trash. Wrote an algorithm for controlling the shoulder on paper. FIND OUT: does the accelerometer do the sine conversion to angle for me, or will I need trig?

###210324
For some reason, the CAN rx interrupt seems to trigger twice when receiving a message from the shoulder/accelerometer, and it contains only trash on the second reading. The second interrupt is handled before the control loop finishes, so all accelerometer readings via CAN are trash as far as the control loop is concerned. The solution is to only handle the message when newmsg is 0. Feels very hacky. The printing of debug messages, understandably, seems to alter behaviour in a very hard to predict manner. I really wish the debugger worked. 
Edit: The messages now come in random order, effectively making it impossible to incorporate accelerometer data. Tried manually clearing the CAN_IT_RX_FIFO0_MSG_PENDING flag as well, but no luck.


Decided to rewrite the can driver such that the interrupts almost don't do anything, and a poller/handler in the main does most of the work. Made two lists of 8 functions, one for rx handling and one for tx handling. The idea is that each function corresponds to a mailbox, defined as a struct of a newmsg flag and a data container, and represents an action. I.e. breaking up the large handlers from earlier. Things look okay on the transmit side, but rx must be implemented. The first byte in a CAN message now represents the mailbox and corresponding handler functions, so rx must be rewritten so that they don't contain M or A. There probably should be a global list somewhere so that I don't need to remember which mailbox/function pair is dedicated to which functions, but then again it's an embedded system.

When the CAN rewrite is done, I should do a VERY SELECTIVE merge with main. Don't want the accelerometer loop disturbing things, but also don't want to lose the math stuff as it's still relevant for encoder setpoints. Next goal should be to write controllers for each joint, like with the shoulder/accelerometer loop, just without CAN.

###220324
Rewrote the CAN driver. ID is structured with three fields, accelerometer, motor and command. Accelerometer is ID9..8, motor ID7..5, and command ID4..0. Set up another filter bank so that the accelerometer field is DC for motor commands and vice versa. Main loop runs two loops, one for handling each of tx and rx. Rx has a set of action functions associated with it, and an enum to keep track of valid commands. The HAL_rx now only pushes the message into an array of rx messages, which the main loop responds to. As the rx and tx mailboxes are both of equal length, some will probably not be used: the message type decides which mailbox is used, and message types are mostly explicitly either rx or tx. However, we're not short on space and this seems like a safe structure. I've ditched the set of accelerometer commands for now, but may reimplement. Also added doxygen to the driver files, seems to look good! Updated from the command palette, be sure to have the right file open/selected before running. Had to enable basically every 'EXTRACT_' flag to make it realise I actually want every function.

###230324
Changed the CAN config filter names and bank numbers to be less youtubey, including giving the configs unique names on the off chance that reusing them is bad. Making one last attempt at getting stable accelerometer readings, if for no other reason than having a proof of concept.

Got accelerometer readings back, works when only trying to read one value. Will test a bit more.

Otherwise: Got joint control back up with a primitive PI controller, can now set rads/mm setpoints. Set Ki to 0 for most joints as the friction in the system makes it a bit unstable -- either spends too long stabilizing or overshoots heavily. Got more accurate estimates for the resolution of all joints, works quite well. Still working on low speeds ofc, but the encoder counter seems to keep track without issue.

##240324
Set up a timer interrupt on approx 344Hz on TIM2. Idea: based on CAN bus rates etc, accelerometer and motor pos should not be polled more frequently than 350Hz. The interrupt sets to flags, defined in the joint controller module, declaring that enough time has passed since last poll that a new poll is probably safe. This is to avoid bus contention. This may be simpler than trying to find a heuristic for when to measure acceleration in the control loop, and will probably also be helpful for ROS integration: The software can then poll for position and speed in the control loop, with a well-defined (ish) time step.

Good day for architecture! Set up a new set of CAN commands, get axis. Sends a request for rotation and acceleration data for the given axis. Acceleration works well, rotation may need some debugging for whatever reason. Proof of concept in the main loop. One IMPORTANT drawback is that the polling mechanism will always favor messages early in the queue, need to figure out some sort of priority or just think harder on whether it's really a problem. Before merge: Integrate accelerometer readings into the shoulder control loop. Then get to work on ADCs, and hopefully start work on ROS after that.

###250324
Decided to skip right on to ADC driver. Needs a bit more testing, but the main ffeature of turning the relays off above a certain threshold seems to work. The trick was to configure them for continuous mode, that way they'll just keep going after start.

Consider rewriting interface functions in joint control (and others?) to use the structs directly via the struct arrays instead of driver functions. Can get rid of a lot of unnecessary and confusing code that way. And maybe strip some of the joint interface functions? See how many are actually needed after accelerometer integration.

Erlend brought up an interesting point re I2C: could be a polling issue. If the data is sampled just as an edge is rising/falling, the data could be trash, or work spuriously -- which is what I'm seeing. CubeMX doesn't let me change edge polling, and I think I2C has clock and data offset by 90 degrees anyway, but I should investigate the rise/fall time parameters. IMU datasheet doesn't mention it, but still worth a try.

Tomorrow: Integrate accelerometer into control loop, test the ADC driver more extensively. Then remove unnecessary functions and write comments. Consider adding a string cmd to toggle relay, could be nice to be able to reset if the ADC triggers.


###260324
ADC integration complete, can now reliably store and read amps as a float. The shoulder motor seems to cut out at too high power settings, i.e. if the setpoint is too far away from current position. Need to figure that out, and should get to work on implementing a PID with proper timesteps over easter. Then ROS:)) Shoulder accelerometer feedback is implemented as part of joint update power. Hard coded for shoulder atm, but should be safe for other motors. Problem: The accelerometer reads 0 as soon as it passes 90 degrees, at least in the negative direction. This should be adjusted for somehow, e.g. by discarding/compensating for readings with a very high delta like in the motor driver.

Not finished with documentation, the joint controller is a brute. Made file headers with more extensive descriptions, looks neat in Doxygen.

###300324
Didn't bother commenting all the private functions of the joint controller. For PID control: simply set a timer interrupt for 10kHz and update control on a flag. Use error and previous error to do the D term. Tuning is pointless until the proper power supply is available. I term: avoid windup by zeroeing the term when error crosses 0, e.g. by XOR'ing error with previous error on poll.

Commenting is pretty much complete, shipping to Jo Arve for review.

Tomorrow:
1. Refactor motor driver to use a list of descriptors instead of the if 0/1 checks. Additionally, change the motor encoder define statements to 0 and 1 instead of 1 and 2.
2. Implement PID and very quickly test tuning.
3. Make a homing/startup/calibration algorithm. This will also introduce a global state variable, might justify another controller layer.

###310324
From yesterday: motor and encoder defines are based on hw silk designations, changed motor driver struct names to motor 1 and 2. Put them in a list, so correct indeces are still 0 and 1, respectively. This is still a bit unintuitive, and should probably be commented on.

PID implemented and tested for all joints. For I windup: only activate the I term when de/dt is less than 0.000017, corresponding to 10deg/sec or 0.17mm/sec. Strategy for tuning, which will probably still be relevant for the final voltage supply: Kp a bit higher than for the pure P controller. Kd 1/4 of Kp, and Ki 1/400 of Kp as initial values.

Rail homing OK, but needs 4V of force to even trigger the switch. Set up a state machine framework with error, idle, calibration and operating as global states, and one calibration state for each joint. Idea is to work through all individually, then stitch together in a sequence.

Briefly tried to set rise time and fall time for I2C to safe values for 400kHz and 100kHz (200, 1000 ns resp.) and tested on the hand onboard IMU on I2C3, but no luck.

###010424
Found a workable calibration scheme: Move until delta is 0, then move back a specified number of clicks, then stop and zero. It's becoming increasingly unintuitive that the joint position isn't actually used for anything.

Tried to use the scheme on elbow, effectively requiring power for both elbow and shoulder. There just isn't enought juice in the Mascot box for that, so will need to get the bigger power supply frist. If the box fries everything, at least I can start writing. Scratch that, the problem was that the torso mcu doesn't get updates from the shoulder mcu while its joints are calibrating. Solution: check for stale accelerometer data, and use the encoder.

While the method does seem to work, setting the total count to 0 only works from the set total count function. The zero function (formerly init) seems to set a random value. Calibration for all joints except shoulder tested.

Strategy for calibration procedure:
Make a new CAN filter with the upper bit active for global messages.
1. Torso starts by calibrating rail, setting global state to calib rail and emitting the appropriate CAN message.
2. Torso stablises shoulder in the vertical position, sets global state to twist clib when done.
3. Hand responds to global state by doing twist then pinch calib (setting states as it goes). Passes the buck to shoulder by setting global state wrist calib.
4. Shoulder requests 90 degrees on twist to ensure safe position (could also be set by hand immediately after calib), does wrist and elbow.

###030424
Something fundamentally wrong with how I manipulate endoder values. Zeroeing does not work as expected, but does something. Twist calib works if zero and set total is called at the beginning, but only set total at the end, and only if the joint has not moved away from its init position. I am updating the encodercount while in idle state, but that should have been zeroed by the zero function.

Calibration works. Rail calibration is not very robust, tends to go into a state of unsuccessfully sending CAN messages. Suspect the needle gets stuck, continuously triggering the interrupt and disturbing CAN trx. Safe-ish solution: place the rail almost at the end, just a few mm before the switch is triggered. Wrist also weakly coupled with the elbow, and for unknown reasons does not correct on its own. But it works! Should grease everything, then cover up. Possibly adjust rail to ramp up and down.

Set the power limit on shoulder and rail to 50 from 70, made it more stable. Really about time I integrated the real power supply.

###040424
Soldered the connector, seems to work. Deciding safe motor voltages: I have datasheets for all sizes of Pittman motors. Assuming "V1" is the voltage rating of the motor and trusting my work from the specialisation project:

Rail: 40mmx55mm 30.3V => Ipk=4.9A; KT=0.0472Nm/A
Shoulder: 54mmx113mm 30.V => Ipk=18A; KT=0.0972Nm/A
Elbow: 40mmx61mm 24V => Ipk=8.2A; KT=0.0365Nm/A
Wrist: 30mmx62mm 30V => Ipk=4.5A; KT=0.0272Nm/A
Twist/pinch (Escap): 23LT2R 12V => Imc=0.92A; KT=0.0124Nm/A

The 42V supply is not a viable option. For one, I broke the connector, and in any case it turns out even 30V is effectively too much. I got to use Finn's PeakTech 1535 laboratory switching mode power supply, which maxes out at 32V/20A. The regulators get very hot, so turned it down to 25 and re-tuned PID from there. Not fun, and got some strange instabilities because of it. The robot tried to beat itself to death several times.

1. The torso was effectively working at 50Hz because of my delay in main, probable cause for instability. It must have been just under a threshold previously, and at 30V it just crapped out.
2. Experimented with control schema. Rate limited PID, where udot is incorporated, could potentially work, but would need safeties to ensure that power could switch sign without penalty. Briefly considered changing to a pure velocity controller, but I don't know how I'd make it stop at the right position. And the current setup is not made for rate control, anyway.
3. Incorporated update_position into the loop. Error is now calculated from that, and previous position is also stored in case someone wants to use that later.Really need to make a "control loop" function/state, looks messy atm.
4. Need feedback on whether to focus on ROS or low level. If low level: incorporate torque calculations into ADC driver. Also divided the conversion constant by 10, ADC readouts look much more believable!

5. Need to prepare for integration with ROS. Go through every print statement and check that they are HMI only!

###050424
Implemented two CAN messages: JOINT_POS_REQ and JOINT_POS_TX as motor type messages. Syntax: data[0]=e/r, data[1..4]=clicks/rads. Motor number is masked out of ID in REQ and inserted back into TX. Torso should then be able to see all messages, but shoulder and hand only those relevant to their motors.

Removed a lot of dead code, but should still maybe remove functions that simply aren't in use. Tried compiling without string_cmd_parser, which didn't crash. Implies that uart messages are appropriately encapsulated in if-preprocs. Discovered that the accelerometer position must override the encoder position, because the encoder updates run so much faster than accelerometer updates.

I may just have fried the whole CAN circuit. In a completely unpredictable manner, the hand vbus caught while calibrating and were pulled out, and I'm not sure that power was decoupled in time before +25V hit CANL. Update: not fried

Need to disregard encoder data during calibration of shoulder joint. Tried overwriting encoder count in update_pos with accelerometer position and resolution, bu this introduces vibrations, likely due to interference between update_encoder_count in main and inaccuracy in the resolution estimation.

###090424
Wrote shoulder calibration procedure as effectively identical to update pos w/o the acc/enc check. It works, but is unstable if the elbow joint is extended at an acute angle. During operations, shoulder struggles with accuracy at high angles. Hypothesis: Too low voltage to effectively actuate the shoulder, and/or integral effect not strong enough. This is another reason for a velocity controller being preferable to a positional controller: the vc could have increased torque until the appropriate velocity was reached. Recommend keeping shoulder within +-1.2rad.

Also seeing that the calibration is not always successful, and that running it twice in a row does not necessarily work. This is probably due to the state machine having been tacked on top of an already complex system.

Stress test: Shoulder +-1.2rad when elbow at -1.5rad works well, maintaining good accuracy. Shoulder between 1.2rad and -0.5rad when elbow at 0 works well, but more noise audible around 1rad compared to elbow -1.5rad. Moving elbow from -1.5rad to 0 while shoulder at 1.2 rad makes shoulder droop temporarily. Hypothesis: low voltage makes the shoulder struggle to keep up during increased torque, and/or integral action too low.

For the thesis: Tune PID at voltage levels 20, 25, 30V using ZN, make qualitative assessment.

Downloaded ROS2 Humble Hawksbill, started following various tutorials. Got a bit ahead of myself, but rviz seems promising. Need to follow the tutorials to get a grip on TF, launcher scripts etc in the intermediate section after completing the beginner stuff.

###100424
Followed some of the intermediate lessons, but I think rviz and TF is a bit too heavy and can be automated by moveit. Moveit uses rviz, but has functionality to let me do motion planning etc.

###110424
Wrote URDF of the Orca, visualising using moveit and moveit setup assistant.
Problem:
1. 'Capabilities' error: launches.py line 203 changed with fix from [StackExchange](https://robotics.stackexchange.com/questions/110134/trouble-launching-moveit2-configurations)

2. 'Type double not string' error on rviz launch: [Locale problem](https://github.com/ros-planning/moveit2/issues/1049) , fix problem by starting command with LC_NUMERIC=en_US.UTF-8

3. Added acceleration and velocity limits for all joints, necessary for planning, in joint_limits.yaml, then rebuilt the project. Path planning works.

Phind suggests I use rviz/moveit to set up publishing of planned trajectories onto a topic, which a listening Serial node can then translate and send to UART. Need to find out if this is real-time or a "set" of commands.


###120424
Something broke towards the end yesterday after I started the tutorial for C++/MoveIt. One thing that almost certainly was wrong was that I installed the moveit package globally under Humble, as well as in the already existing build from source. Moveit complained about a double definition somewhere, anyway. Cleaned the slate and installed Iron instead. Building MoveIt on top of that resulted in fewer errors.

Created a package under code/ros called orca_moveit_config using setup assistant. Only changes made after setup was adding acceleration limits as floats (not ints!) to joint_limits.yaml. That means:
1. MoveIt is built from source under home/ws_moveit.

2. My orca package is built under ttk.../ros.

3. Useage requires sourcing ROS (automatically from bash), moveit (manually from ws_moveit) and orca_moveit (manually from ttk).

4. Launching the sim still requires LC_NUMERIC even though that was configured as part of the ROS install.

###130424
Everything exept reliable parsing of the input strings works. It seems completely random whether or not string comparison of the header makes it through.
Possible solutions: Everything happens in an interrupt handler, which is preempted by one of my many other timing interrupts etc. Will need to make a flag system or something. Alternatively, sophons are real. Frustrating as fuck.

Started writing a dedicated ROS message decoder, similar structure to CAN with flags etc. Problem: Only one joint pos setpoint mailbox, need to ensure that no positions are lost. Rewrite the CAN module to accommodate, or separate (FIFO) queueing structure in the ROS module?

Rewrite CAN module. Nbd. Added one message structure for dual positions, idea was to use that in conjunction with a queue system in the ROS module. However, KISS: add another message type s.t. one for shoulder and one for hand. Use filtering to ensure only one gets through at the relevant unit. queue_tx overwrites data in every instance, so setpoints will be lost if not transmitted. May result in more staccato movements, but that's OK.

Detailing the need for more messages: Joint setpoint only holds one position, and must be transmitted before the next setpoint is queued lest it be lost. ROS module receives (and sets) all setpoints at once -> would only send the last (i.e. wrist) for each ROS message received. Need to deal with that somehow. CAN messages can only be sent at 344Hz, so sending 4 positions consecutively at 100Hz is not possible (effectively 400Hz). If packing both setpoints in one msg, get 200Hz effectively, i.e. 2 msgs at 100Hz, which should be fine.

Rail and shoulder work smoothly with ROS integration! Tested with elbow only using the already existing joint setpoint message, works okay too but saw signs of bus contention: somewhat irregular shoulder movements (bad acc readings) and elbow was a bit jittery (slow/staccato movement). Granted, went straight to Operating state because I got tired of waiting for calibration.

###160424
All joints except pinch controllable from rviz. Run setup assistant again and reconfigure as there is something preventing the pincher from moving in rviz as it is. Set rail in its own planning group, noticed that the planner sometimes includes rail unnecessarily. Also increase maximum velocities and acceleration on everything except rail 2x, twist 5x.
