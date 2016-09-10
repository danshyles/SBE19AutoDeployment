//Code to automate deployment of Seabird SBE19 CTD
//Author Daniel Shyles, July 2016

/*WDT BYTE variables for setting timer value
WDTO_15MS
WDTO_30MS
WDTO_60MS
WDTO_120MS
WDTO_250MS
WDTO_500MS
WDTO_1S
WDTO_2S
WDTO_4S
WDTO_8S */
 
#include <stdlib.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
#define arr_len(x) sizeof(x)/sizeof(*x)

SoftwareSerial telemetry(2,3); //RX,TX
//For Watchdog Timer
#define LED_PIN (13)
volatile int f_wdt=1;

//Deployment Parameters (User Input)
const int maxSamples = 20; //How many samples per deployment?
const int deployWaitInterval = 600000; //time to wait between deployments in ms.

long strtol (const char *__nptr, char **__endptr, int __base);
char inByte = 0; // incoming serial byte
char outByte = 0;
boolean loopDeployment = true;
int CTDhexsize = 12; // How many characters are we expecting for each sample?
int inByteCount = 0;
int inByteCountMax = 0; // How many bytes do you want to reject? (This is no longer necessary, but I'll keep in just in case)
boolean haveCR = false; // Is this character a carriage return?
boolean deployed = false;
boolean acceptInByte = false; 
int GL_Flag = 0;
int start_Flag = 0;
String stringOne = "";
String samples[maxSamples];
String line;
int sampleCounter = 0;

//Output parameters, see https://drive.google.com/drive/u/0/folders/0B6GaVHv-Aafs8dpc1VXQWkwTE10b3M
//Reference Hi/Lo 
double Refhi = 0;
double Reflo = 0;
char RefHEX[7];

//Temp & Cond Freq Correction Coefficients
double Cor_KK = 2.4018669e-11;
double Cor_X1 = 9.6036247e-9;
double Cor_X2 = 1.1949587e-7;
double Cor_X3 = (Cor_X2-Cor_X1)/(Cor_X2*Cor_X1); 
double Cor_PC = (1/((1e6)*Cor_KK));
double Cor_a = 0;
double Cor_b = 0;
double Cor_fraw = 0; //Raw frequency (input)  
double Cor_fcor = 0; //Corrected frequency (output)
double Cor_fhisq = 0; //Refhi squared
double Cor_flosq = 0; //Reflo squared
//Temperature Calibration
double TempFreq = 0;
char TempHEX[5];
double TempDec = 0;
double Temperature = 0;
const double Temp_g = 4.15296085e-3;
const double Temp_h = 5.88952455e-4;
const double Temp_i = 3.18066751e-6;
const double Temp_j = -1.87560171e-6;
const double Temp_fITS = 1000.0;
const double Temp_a = 3.64763682e-3;
const double Temp_b = 5.79392501e-4;
const double Temp_c = 8.06670097e-6;
const double Temp_d = -1.87526027e-6;
const double Temp_fIPTS = 2372.837;

//Conductivity Calibration
double CondFreq = 0;
char CondHEX[5];
double CondDec = 0;
double Conductivity = 0;
double CondNRFreq = 0; //if deployed in fresh water (Narrow Range)
const double Cond_g = -4.06566390;
const double Cond_h = 4.87801815e-1;
const double Cond_i = 1.55280047e-4;
const double Cond_j = 1.05109678e-6;
const double Cond_CPcor = -9.5700e-8;
const double Cond_CTcor = 3.2500e-6;
const double Cond_a = 2.75286294e-3;
const double Cond_b = 4.83868973e-1;
const double Cond_c = -4.05877456e+0;
const double Cond_d = -8.45849515e-5;
const double Cond_m = 2.3;
const double Cond_Cell = 2000.0; //[S/m]
const double Cond_R = 100.0; //[Ω]

//Pressure Calibration
double PressFreq = 0;
char PressHEX[5];
double PressDec = 0;
double Pressure = 0; //in psia
double PressureDB = 0; // in decibars
//Quadratic Coefficients
const double Press_PA0 = 2.518923e+3;
const double Press_PA1 = -6.504022e-1;
const double Press_PA2 = 8.491770e-8;
//Straight Line Fit
const double Press_M = -6.503676e-1;
const double Press_B = 2.519446e+3;
//
//ISR(WDT_vect)
//{
//  Serial.println("FINISHED WDT");
//  if(f_wdt == 0)
//  {
//    f_wdt=1;
//  }
//  else
//  {
//    Serial.println("WDT Overrun!!!");
//  }
//}

//void enterSleep(void)
//{
//  Serial.println("Entered Sleep Function");
//  delay(100);
//  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
//  delay(100);
//  Serial.println("Set Sleep Mode");
//  delay(100);
//  sleep_enable();
//  delay(100);
//  Serial.println("Enabled Sleep");
//  /* Now enter sleep mode. */
//  Serial.println("About to Sleep");
//  delay(100);
//  //sleep_mode();
//  sleep_cpu();
//  delay(100);
//  Serial.println("WDT complete. Wake up.");
//  /* The program will continue from here after the WDT timeout*/
//  sleep_disable(); /* First thing to do is disable sleep. */
//  delay(100);
//  Serial.println("Disabled Sleep");
//
//  /* Re-enable the peripherals. */
//  power_all_enable();
//  delay(100);
//  Serial.println("Power on");
//}

void setup() {
  //disable watchdog timer so interrupt/reset does not occur when I don't want it to.
//  wdt_disable();
  // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600, SERIAL_8N1);
  Serial1.begin(9600, SERIAL_7E1);
  telemetry.begin(9600);

  
  Serial.println("Initializing...");
  delay(100); //Allow for serial print to complete.
  pinMode(13,OUTPUT);

//  /*** Setup the WDT ***/
//  /* Clear the reset flag. */
//  MCUSR &= ~(1<<WDRF);
//  /* In order to change WDE or the prescaler, we need to
//   * set WDCE (This will allow updates for 4 clock cycles).
//   */
//  WDTCSR |= (1<<WDCE) | (1<<WDE);
//  /* set new watchdog timeout prescaler value */
//  WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
//  /* Enable the WD interrupt (note no reset). */
//  WDTCSR |= _BV(WDIE);
//  Serial.println("Initialization complete.");
//  delay(100); //Allow for serial print to complete.
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // if I send to Arduino via USB
  if (Serial.available() > 0) {
    outByte = Serial.read();
    
    //sent "D" for Deploy
    if(outByte == 68) {
      deployCTD();
    }

    //sent "E" for Enter/Return
    if(outByte == 69) {
      Serial.println("You said to press enter");
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
      Serial1.write(0x0D);
    }

    //sent "S" for Start, used to turn on the logger manually.
    if(outByte == 83) {
      Serial1.write(0x0D);
      Serial.println("You said to start");
      delay(10);
      Serial1.write("GL");
      delay(10);
      Serial1.write(0x0D);
      delay(10);
    }
    //sent "Y" for Approve logging
    if(outByte == 89) {
      delay(10);
      Serial1.write("Y");
      delay(10);
      Serial1.write(0x0D);
      delay(10);
      Serial1.write(0x19);
      delay(10);
      Serial1.write(0x0D);
      delay(10);
      Serial.println("Sent everything"); 
    }

    //sent "K" for Kill, back to standby 
    if(outByte == 75) {
      killCTD();
    }
    
  } //end if Serial.available()

 // if we get a valid byte from CTD, read to inByte:
  if (Serial1.available() > 0) {
    //get incoming byte:
    inByte = Serial1.read();
    delay(50);
    if (deployed == true) {
      readCTD(inByte);
    }
  }
} //end loop();


//////////////////DEPLOY//////////////////

void deployCTD() {
  Serial.println("You said deploy");
  //Turn on LED13 for monitoring
  digitalWrite(LED_PIN, HIGH);
  //setup samples array, erase it every time CTD deploys
  for (int i = 0; i < arr_len(samples); i++) {
    samples[i] = "";
  }
  sampleCounter = 0;
  line = "";
  //write commands to CTD
  //A few <CR>
  Serial1.write(0x0D);
  delay(50);
  Serial1.write(0x0D);
  delay(50);
  Serial1.write(0x0D);
  delay(50);
  if(GL_Flag == 0) {
    //write GL to initiate logging
    Serial1.write("GL");
    Serial.println("Sent GL");
    delay(1000);
    Serial1.write(0x0D);
    delay(1000);
    Serial1.write("Y");
    Serial.println("Sent Y");
    delay(1000);
    Serial1.write(0x0D);
    delay(1000);
    Serial1.write(0x19);
    Serial.println("Sent ^Y");
    delay(1000);
    Serial1.write(0x0D);
    delay(1000);
    //every other time loop starts, use RL
    GL_Flag = 1;
  } else {
    //write RL to resume logging  
    Serial1.write("RL");
    Serial.println("Sent RL");
    delay(1000);
    Serial1.write(0x0D);
    delay(1000);
    Serial1.write("Y");
    Serial.println("Sent Y");
    delay(1000);
    Serial1.write(0x0D);
    delay(1000);
    Serial1.write(0x19);
    Serial.println("Sent ^Y");
    delay(1000);
    Serial1.write(0x0D);
    delay(1000);
  }
  //set up lag so CTD can start taking data before we start reading to samples array
  Serial.println("Wait for CTD to start taking data");
  delay(5000);
  Serial.println("Deployed set to true");
  deployed = true;
  Serial.println("start collecting data");  
}

//////////////READ///////////////

void readCTD(char inByte) {
//Count inBytes, switch on taking samples after garbage characters finish
//UPDATE: not really necessary anymore. I have a better way of parsing HEX vs garbage below.
//Still, I'll keep it in case it's useful to reject initial characters in this way.
  if(inByteCount < inByteCountMax) {
    inByteCount += 1;
  }

  if(inByteCount == inByteCountMax) {
    //start accepting bytes into sample array
    acceptInByte = true;  
  }
  
  Serial.println(inByte);
  //if CTD sends a <CR>
  if(inByte == 0x0D) {
    //Serial.println("Got a <CR>, (0x0D)");
    haveCR = true;
    //check if next one is 0x0A, if so, we have a new piece of data
  } 
  if(inByte == 0x0A) {
    //Serial.println("Got 0x0A");
  }
  if(inByte == 0x0A && haveCR && acceptInByte) {
//    Serial.println("Got a line feed (0x0A), Record string");
  //Completed new HEX stream, record to samples
    //Serial.println("*********Final String*********");
    //Serial.println(stringOne);
    line = stringOne;
    stringOne = "";
  //      Serial.println("Printing Line");
  //      Serial.println(line);
    //Reset to look for next <CR><LF>
    haveCR = false;
  } else if(isxdigit(inByte) && (int)inByte < 71 && acceptInByte) {
    //^^^Only include HEX characters in string
    //inByte is still building the HEX stream
    //Serial.println("IS HEX");
      stringOne += inByte;
  }

//  If we get a non HEX character, we don't want that string. Delete stringOne, and reset everything
  if(!isxdigit(inByte) && inByte != 0x0A && inByte != 0x0D && inByte > 69 && acceptInByte) {
    //Serial.println("!!!!!!!!!!!!!!!!!!!!!!!NOT HEX, SKIP!!!!!!!!!!!!!!!!!!!!!!!!");  
    stringOne = "";
    for (int i = 0; i < arr_len(samples); i++) {
      samples[i] = "";
    }
    sampleCounter = 0;
    line = "";
  }

//    Until samples[maxSamples] has something, keep adding to samples[]. 
//    When the samples[maxSamples] is reached, then hit ^Z/^C.
    if (sampleCounter < maxSamples) {
      //avoid garbage (This should be handled from earier HEX filtering, but just in case)
      //CTDhexsize ensures that the string is the length we expect (default = 12)
      if (line != "" && !line.startsWith("start") 
        && !line.startsWith("resume") 
        && !line.startsWith("GL")
        && !line.startsWith("RL")
        && !line.startsWith("S")
        && !line.startsWith("are you sure")
        && !line.startsWith("?")
        && line.length() == CTDhexsize) {
          
        samples[sampleCounter] = line;
        Serial.print("Sample ");
        Serial.print(sampleCounter);
        Serial.print(": ");
        Serial.println(samples[sampleCounter]);
        sampleCounter += 1;
        line = "";
      }
    } else {
      //reached samples[maxSamples]. Exit deployment.
      Serial.println("finished collecting data");
      //Ctrl-Z a few times to stop logging
      delay(500);
      Serial1.write(0x1A);
      Serial.println("pressed ctrl-z once");
      delay(500);
      Serial1.write(0x1A);
      Serial.println("pressed ctrl-z twice");
      delay(500);
      Serial1.write(0x1A);
      Serial.println("pressed ctrl-z thrice");
      delay(500);
      //Ctrl-C a few times to ensure we're back to S>
      Serial1.write(0x03);
      Serial.println("pressed ctrl-c once");
      delay(500);
      Serial1.write(0x03);
      Serial.println("pressed ctrl-c twice");
      delay(500);
      Serial1.write(0x03);
      Serial.println("pressed ctrl-c thrice");
      delay(500);
      Serial.println("Waiting ten minutes, then repeat");
      Serial.println(0x0D);
      deployed = 0;

      
      //convert HEX data from CTD to engineering units:
      //conversion is according to CTD calibration and SBE19 profiler conversion protocol in manual.

      Serial.println("/*****Final Results*****/");
      for (int i = 0; i < arr_len(samples); i++) {
        Serial.println("\n");
        Serial.print("------Sample ");
        Serial.print(i);
        Serial.println("------");
        Serial.println(samples[i]);
        if(samples[i].charAt(0) == '0' && samples[i].charAt(1) == '5') {
          Serial.println("HI-FREQ Standard (salt water)");
          for (int j = 0; j < 7; j++) {
            if(j<6) {
              RefHEX[j] = samples[i].charAt(j+2);
            }else {
              //add \0 character to prevent bogus characters
              RefHEX[j] = 0;
            }
            if(j < 5) {
              //Pressure has format pppp at the end of the HEX string
              PressHEX[j] = samples[i].charAt(j+8);
            } else if (j == 5) {
              PressHEX[j] = 0;  
            }
          }
          Refhi = strtol(RefHEX, NULL, 16)/256;
          PressDec = strtol(PressHEX, NULL, 16);
//          Serial.print("RefHEX: ");
//          Serial.println(RefHEX);
          Serial.print("Refhi: ");
          Serial.println(Refhi);
//          Serial.print("PressHEX: ");
//          Serial.println(PressHEX);
          Serial.print("Pressure #: ");
          Serial.println(PressDec);
        } else if(samples[i].charAt(0) == '0' && samples[i].charAt(1) == '8') {
          Serial.println("HI-FREQ Narrow (fresh water)");  
          for (int j = 0; j < 7; j++) {
            if(j<6) {
              RefHEX[j] = samples[i].charAt(j+2);
            }else {
              //add \0 character to prevent bogus characters
              RefHEX[j] = 0;
            }
            if(j < 5) {
              //Pressure has format pppp at the end of the HEX string
              PressHEX[j] = samples[i].charAt(j+8);
            } else if (j == 5) {
              PressHEX[j] = 0;  
            }
          }
          Refhi = strtol(RefHEX, NULL, 16)/256;
          PressDec = strtol(PressHEX, NULL, 16);
//          Serial.print("RefHEX: ");
//          Serial.println(RefHEX);
          Serial.print("Refhi_NR: ");
          Serial.println(Refhi);
//          Serial.print("PressHEX: ");
//          Serial.println(PressHEX);
//          Serial.print("Pressure #: ");
//          Serial.println(PressDec);
        } else if(samples[i].charAt(0) == 'F' && samples[i].charAt(1) == 'F') {
          Serial.println("LO-FREQ");  
          for (int j = 0; j < 7; j++) {
            if(j<6) {
              RefHEX[j] = samples[i].charAt(j+2);
            }else {
              //add \0 character to prevent bogus characters
              RefHEX[j] = 0;
            }
            if(j < 5) {
              //Pressure has format pppp at the end of the HEX string
              PressHEX[j] = samples[i].charAt(j+8);
            } else if (j == 5) {
              PressHEX[j] = 0;  
            }
          }
          Reflo = strtol(RefHEX, NULL, 16)/256;
          PressDec = strtol(PressHEX, NULL, 16);
//          Serial.print("RefHEX: ");
//          Serial.println(RefHEX);
          Serial.print("Reflo: ");
          Serial.println(Reflo);
//          Serial.print("PressHEX: ");
//          Serial.println(PressHEX);
//          Serial.print("Pressure #: ");
//          Serial.println(PressDec);
        } else {
          Serial.println("Regular Data");
          for (int j = 0; j < 5; j++) {
            if(j < 4) {
              TempHEX[j] = samples[i].charAt(j);
              CondHEX[j] = samples[i].charAt(j+4);
              PressHEX[j] = samples[i].charAt(j+8);
            } else {
              TempHEX[j] = 0;
              CondHEX[j] = 0;
              PressHEX[j] = 0;  
            }
          }
//          Serial.println("---Temperature---");
//          Serial.println("HEX");
//          Serial.println(TempHEX);
//          Serial.println(arr_len(TempHEX));
          //Get Temperature in Decimal
          TempDec = strtol(TempHEX, NULL, 16);
//          Serial.println("Decimal");
//          Serial.println(TempDec);
          //Calculate raw frequency
//          Serial.println("Raw Frequency");
          TempFreq = (TempDec/17)+1950;
//          Serial.println(TempFreq);
          //Corrected frequency using references
          Cor_a = ((Refhi*Refhi)-(Reflo*Reflo))/Cor_X3;
          Cor_b = (Reflo*Reflo)-(Cor_a/Cor_X2);
          TempFreq = sqrt((((TempFreq*TempFreq)-Cor_b)/Cor_a)-Cor_PC);
//          Serial.println("Corrected Frequency");
//          Serial.println(TempFreq);
          //Convert TempFrequency to Temperature, units: °C
          //ITS-90 temperature
          Temperature = 1/(Temp_g + Temp_h*(log(Temp_fITS/TempFreq)) + Temp_i*(pow(log(Temp_fITS/TempFreq),2)) + Temp_j*(pow(log(Temp_fITS/TempFreq),3))) - 273.15;
          Serial.print("Temperature: ");
          Serial.print(Temperature,6);
          Serial.println("°C");
          //IPTS-68 temperature

//          Serial.println("---Pressure---");
//          Serial.println(PressHEX);
//          Serial.println(arr_len(PressHEX));
          //Get Pressure in Decimal
          PressDec = strtol(PressHEX, NULL, 16);
          //if bit 14 is 0, the decimal number for pressure is positive. 
          //if bit 14 is 1, the decimal number is negative. 
          //These half-byte hex values have a second bit of zero, 
          //corresponding to the 14th bit of four hex values.
          if(!strchr("014589CD",PressHEX[0])) {
            //Serial.println("This is a negative pressure number");
            PressDec = -PressDec;  
          }
//          Serial.println("Decimal");
//          Serial.println(PressDec);
//          Serial.println("Raw Frequency");
//          Serial.println(PressDec);
          //Calculate Pressure in psia
          Pressure = Press_M*PressDec + Press_B; //[psia]
          //Pressure in decibars
          PressureDB = 0.689475728*Pressure;
          Serial.print("Pressure: ");
          Serial.print(Pressure,6);
          Serial.print(" psia, ");
          Serial.print(PressureDB,6);
          Serial.println(" decibars");
          
//          Serial.println("---Conductivity---");
//          Serial.println("HEX");
//          Serial.println(CondHEX);
//          Serial.println(arr_len(CondHEX));
          //Get Conductivity in Decimal
          CondDec = strtol(CondHEX, NULL, 16);
//          Serial.println("Decimal");
//          Serial.println(CondDec);
//          Serial.println("Raw Standard (salt water) Frequency");
          CondFreq = sqrt(CondDec*2900+6250000);
//          Serial.println(CondFreq);
          //calculate corrected conductivity frequency
          CondFreq = sqrt((((CondFreq*CondFreq)-Cor_b)/Cor_a)-Cor_PC);
//          Serial.print("Corrected Frequency: ");
//          Serial.print(CondFreq);
//          Serial.println(" Hz");
          //convert CondFreq from Hz to kHz for Conductivity calculation
          CondFreq = 0.001*CondFreq;
//          Serial.print(CondFreq);
//          Serial.println(" kHz");
          //calculate pseudo-conductivity (conductivity if 100Ω resister were not present)
          Conductivity = (Cond_g+Cond_h*(pow(CondFreq,2))+Cond_i*(pow(CondFreq,3))+Cond_j*(pow(CondFreq,4)))/(10*(1+(Cond_CTcor*Temperature)+(Cond_CPcor*PressureDB)));
//          Serial.print("Pseudo-Conductivity: ");
//          Serial.print(Conductivity);
//          Serial.println(" [S/m]");
          //calculate true-conductivity (considering 100Ω resistor in series)
          Conductivity = (Cond_Cell*Conductivity)/(Cond_Cell-Cond_R*Conductivity);
          Serial.print("Conductivity: ");
          Serial.print(Conductivity,6);
          Serial.println(" [S/m]");
//          Serial.println("Raw Narrow Range (fresh water) Frequency");
//          CondNRFreq = sqrt(CondDec*303+6250000);
//          Serial.println(CondNRFreq);



          /////////  Output final data to transceiver  /////////
          telemetry.write("SBE19_C");
          telemetry.print(Conductivity,12);
          telemetry.write("_T");
          telemetry.print(Temperature,12);
          telemetry.write("_P");
          telemetry.print(Pressure,12);
          telemetry.write("\n");
        }

        
        
        //Get Temperature data:
        
        
        //flush samples array entry to prepare for next sampling session
        samples[i] = "";
      }
      sampleCounter = 0;
      line = "";
      inByteCount = 0;
      acceptInByte = false;

      //wait for next deployment
      delay(deployWaitInterval);
      
//      if(f_wdt == 1)  {
//        /* Toggle the LED */
//        Serial.println("f_wdt on, START SLEEP SEQUENCE");
//        delay(3000);
//        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//        delay(100);
//        Serial.print("Toggle LED ");
//        delay(100);
//        Serial.println(digitalRead(LED_PIN));
//        /* Don't forget to clear the flag. */
//        f_wdt = 0;
//    
//        /* Re-enter sleep mode. */
//        enterSleep();
//      }
//      else {
//        /* Do nothing. */
//      }
     
      Serial.println("Starting a new deployment...");
      if(loopDeployment) {
        deployCTD();
      }
      
    } // end else, which cancels CTD deployment and outputs data.
}


/*
void delayWDT(byte timer) {
  Serial.println("!!!!!!!!!!!!ABOUT TO SLEEP!!!!!!!!!!!");
  delay(3000);
  sleep_enable(); //enable the sleep capability
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //set the type of sleep mode. Default is Idle
  ADCSRA &= ~(1<<ADEN); //Turn off ADC before going to sleep (set ADEN bit to 0)
  WDTCSR |= 0b00011000;    //Set the WDE bit and then clear it when set the prescaler, WDCE bit must be set if changing WDE bit   
  WDTCSR =  0b01000000 | timer; //Or timer prescaler byte value with interrupt selectrion bit set
 // WDTCSR = 0b01000110; //This sets the WDT to 1 second
  wdt_reset(); //Reset the WDT 
  Serial.println("!!!!!!!!!!!!Really gon' sleep nah!!!!!!!!!!!");
  sleep_cpu(); //enter sleep mode. Next code that will be executed is the ISR when interrupt wakes Arduino from sleep
  sleep_disable(); //disable sleep mode
  ADCSRA |= (1<<ADEN); //Turn the ADC back on
  Serial.println("!!!!!!!!!!!!Woke up!!!!!!!!!!!");
}
*/
//This is the interrupt service routine for the WDT. It is called when the WDT times out. 
//This ISR must be in your Arduino sketch or else the WDT will not work correctly
//ISR(WDT_vect) {
//  Serial.println("WatchDog Finished");
//  if(f_wdt == 0)
//  {
//    f_wdt=1;
//  }
//  else
//  {
//    Serial.println("WDT Overrun!!!");
//  }
//}

void killCTD() {
  Serial.println("You said to stop");
  digitalWrite(LED_PIN, LOW);
  Serial1.write(0x1A);
  Serial1.write(0x0D);
  Serial1.write(0x1A);
  Serial1.write(0x0D);
  Serial1.write(0x1A);
  Serial1.write(0x0D);
  Serial1.write(0x03);
  Serial1.write(0x0D);
  Serial1.write(0x03);
  Serial1.write(0x0D);
  Serial1.write(0x03);
  Serial1.write(0x0D);  
}

