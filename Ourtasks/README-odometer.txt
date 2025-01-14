README-odometer.txt
01/10/25

Notes on the odometer function and shaft_encoder2 repo

Purpose of shaft_encoder 2 repo is to implement the 360 PPR encoder (connected to the motor shaft) to measure speed and acceleration for initial spin-up tests with the drum.
The target hardware is the DiscoveryF4 board and interface board for the levelwind, however the levelwind is not mounted on the winch. Therefore the shaft_encoder2 repo only addresses the encoder, but with the organization in mind that it will merge with the levelwind code, in the drum repo.
1. shaft_encoder2/params
There are two parameter files for two instances of the s*2 repo. One is for a proxy, and the other for the demo winch. Using two parameter files avoids the problems of differences and "git pull" conflicts.

The parameter files are named with the prefix of the CAN  ID for the unit, e.g../mm 83200000 [compile with parameter files for CAN ID 0x83200000]./cc 83200000 192.168.2.50 32125 [compile and load over CAN]
2. Loading over CAN
s*2 is loaded at 0x08010000, and therefore requires a startup to move the interrupt vector to this address before the jump. The CAN loader does this. Before loading the s*2, the CAN loader needs to be loaded.cd ~/GliderWinchItems/CANloader1/ldrfixedDiscoveryf4./mm 83200000 [where 83200000 is the CAN ID of the unit]This will compile and flash the loader.
Next step: compile and load s*2cd ~/~/GliderWinchItems//s*2./cc 83200000 192.168.2.50 32125
3. CAN msgs
Three CAN msgs with readings are sent by OdometerTask (via odometer_items.c). Each carries two, four byte readings. In all except msg3 the readings a floats. In msg3 the encoder counter is carried as an unsigned int.
   p->cid_msg1_encoder  = CANID_MSG1_ENCODER1; // 83A00000 FF_FF  DiscoveryF4 encoder demo winch: lineout, speed
   p->cid_msg2_encoder  = CANID_MSG2_ENCODER1; // 83E00000 FF_FF  DiscoveryF4 encoder demo winch: accel, encoder speed
   p->cid_msg3_encoder  = CANID_MSG3_ENCODER1; // 84200000 FF_S32 DiscoveryF4 encoder demo winch: drum speed, encoder counter
MSGs 2 and 3 are for the early testing and MSG1 is for final use. Since all three are not needed at the same time, the parameter file can set whether the msg is sent: 0 = not send, 1 = send.
  p->msg_enable[0] = 1; // MSG1
   p->msg_enable[1] = 1; // MSG2
   p->msg_enable[2] = 1; // MSG3
4. Measurement strategy
The encoder channels A & B connect to 32b timers TIM2 and TIM5. The inputs to TIM2 cause an input capture of the timer upon each edge of each channel of the encoder. The inputs also connect to TIM5 and the TIM5 counter increments/decrements with each edge of each encoder channel. 

This captures the encoder counter and timer time of each encoder transition. Both edges of both channels causing interrupts produce 1440 steps per revolution. The encoder steps are signed so that the max +2,147,483,647, -2,147,483,648. This far greater than needed to accommodate the longest line.
The encoder TIM2 interrupt is set for max so that other interrupts will not delay its execution, as the time between interrupts at a max motor speed is on the order of 7 us.
Since code running under interrupt priorities greater than 5 cannot send FreeRTOS notifications, a second interrupt is used. The TIM2 interrupt triggers the I2C3_ER interrupt, which is set to priority 5. It then notifies OdometerTask. 
The TIM2 interrupt processing does the basic storing of the timer values. Like edges (i.e. rising & falling, of like channels (A & B) are stored. 
The TIM2 output capture interrupts at 64 sec. When this occurs, the latest like edges & channels readings stored are placed in buffers and the lower level interrupt triggered. The lower level interrupt computes the differences of the times and encoder counts from the previous (and saves the most recent to be the next previous). This produces the number of encoder counts and time duration for each of the like pairs of edges during the 1/64th interval. OdometerTask is then notified.
Odometer task "wait for notification", upon finding the bit in the notification signalling a new set of reading differences are ready, computes the speed and acceleration. The speed for each of the four edges is first computed by dividing the encoder count difference by the time difference. The four speeds are summed to give an average. 
The acceleration is computed by differencing the speeds, and summing. Scale factors are then applied to convert to rpm for motor speed, and radians per sec^2 for acceleration.

For comparison purposes, the TIM2 output capture (end of 1/46th sec) interrupt also stores the encoder count. That count is passed to the lower level interrupt the same way. One difference is that the time duration is always 1/64th sec, whereas the like edges & channels will have small time and count differences. The effect of using like edges and channels reduces the jitter slightly. A test at ~3600 rpm showed the standard deviation improved from about 4.5 to 4.3. If the interrupt processing at high motor speeds becomes a problem, the simplified version may be good enough.

5. CAN data rate
The (enabled) CAN msg are sent every time a new reading set is ready, unless the speed has been very near zero for about 1/2 second. When the speed is within a +/- threshold the msgs continue at the fast rate until the zero-sensing timeout occurs.