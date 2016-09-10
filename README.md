# SBE19AutoDeployment
Arduino code to autodeploy SBE19 CTD

Author: Daniel Shyles, University of Tennessee, University of Hawaii, Manoa

Full documentation for SBE19 here: (https://docs.google.com/document/d/1JOsFXGsdgp4u3ZDvOZeornCtRBQNrNk9b98lPtS5Vhk)

/**** SBE-19 CTD Automated Deployment with Arduino (README.txt) ****/

This program sends a signal to the CTD to log 20 samples every ten minutes:
Two versions:
Clean output for telemetry -> This Version
Verbose output for debugging (https://drive.google.com/open?id=0B6GaVHv-A8dpdGx5WUxJcUFoSjg)

/**** Steps to prepare the sensor for field deployment ****/

To hook up the 5T Pump to the CTD, please refer here (https://docs.google.com/document/d/1JOsFXGsdgp4u3ZDvOZeornCtRBQNrNk9b98lPtS5Vhk/edit#bookmark=id.yvyzzwi50pwm)

Before deployment, the minimum conductivity raw frequency should be reset to 3166Hz, which is the default conductivity for salt water (~2890Hz for fresh water). To do so:
With CTD connected to Hercules via USB, or any other serial command prompt, Type ‘SP’.
Set the minimum conductivity raw frequency to 3166Hz
You can also set the pump delay time. Once the above conductivity is detected, the CTD will wait to turn on the pump however long you deem necessary. 

/**** Attaching the Arudino chip for automated sampling: ****/

Digital pin (D#)  D0 (RX) receives data from the CTD, and D1 (TX) transmits instructions from the Arudino controller, and should be connected to the TX/RX pins (respectively) on the DE-9 pin serial port going to the CTD. 
GND from the DE-9 pin serial port should be connected to GND on the digital pin side of the Arduino
With the digital pins on the Arduino oriented away from you, VCC from the DE-9 pin serial port should be connected to the top right pin of the six pins labeled ISP at the right of the Arduino. 
SoftwareSerial (digital) pins 2 and 3 on the Arudino are configured to transmit the final output of the CTD via telemetry to a receiver connected to ethernet so that the data can then be uploaded to the Coastal Monitoring website. 

/**** Autodeployment Initiation, and Arduino Procedure: ****/

The command ‘D,’ for “deploy,”  must be sent to the Arduino chip via a serial command prompt, such as the Arduino serial monitor. 
You can remove the necessity to initiate in this way by removing the if statement on line 198, leaving the call to the deployCTD(); method.
The Arduino will then run a loop to begin a new log on the CTD (SBE19 command ‘GL’, §2-2.1). 
The Arduino will then wait for the expected HEX serial data from the SBE19, each a string of 12 characters, and record them to an array until the length of the array is equal to the limit set by the maxSamples parameter, which you can set on line 29. (I have set the default to 20 samples)
When the Arduino has received the maximum number of expected samples, it will send a series of Ctrl-C and Ctrl-Z commands to the CTD, which will cease sampling.
The Arduino will then parse each sample in the samples array, and convert each to human-readable data.
The Arduino will then send the human-readable data to SoftwareSerial pin 3 (TX), intended for telemetry transmission in the format: “SBE19_C#.############_T#.############_P#.############,” 12 sig-figs.
For example, one sample reading that I took while debugging was: “SBE19_C0.036149601936_T18.019744873046_P14.229980468750”
You can change the number of significant figures and format beginning on line 622 of the Arduino script.
The Arduino will then wait for a time prescribed by the parameter deployWaitInterval, set on line 30 of the Arduino script. I currently have it set to ten minutes (6e5 ms).
After the prescribed time has elapsed, the Arduino will send the SBE19 command ‘RL,’ which will instruct the CTD to resume logging, preserving all previous samples on the CTD memory. 
Then the cycle repeats from step 3, above.
When retrieving the CTD from the field, you can send the command ‘K’ to the Arduino via the serial monitor to cancel all activity and await the command ‘D’ again. 

Note: Recall that we do not have the watchdog sleep function enabled on the Arduino (I’ve commented out our attempts in the script). The Arduino waits between deployments via the delay() function. 



