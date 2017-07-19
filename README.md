# MoxieBox by No Eye Deer Labs

The MoxieBox project combines hardware and code to create a device that can be added to an electrically powered vehicle to add sound effects during acceleration, braking, cornering and other motion conditions as well as adding a fully programable horn. The hardware used is as follows:

-Sparkfun/Robertsonics WAV Trigger
-GY-521 MPU-6050 3 Axis Accelerometer Gyroscope Module
-Arduino Pro Mini 328 - 5V/16MHz
-Generic 12v ~50W amplifier (we used one based on the TI TPA3118D2 in a bridgeable 25W+25W config)
-A box to put it in
-A cheap horn type loudspeaker

Out of all of the above it was the Wav Trigger that swallowed the budget, there ate probably much cheaper solutions but we had one on hand and it is polyphonic (able to play multiple sounds at the same time) which adds something special to that drifting maneuver!

The target use case and basis of the name is to be used in the No Eye Deer Labs Racing team entry in the Power Wheels Series (http://www.powerracingseries.org/). Here you can actually win the race by not crossing the line first but by how many moxie points your kart acquires by pure entertainment value. So why not have your cart play sounds, especially sounds where you can get very creative what they are because basically if it can be captured and placed in a WAV file you are good to go ;-)