/* started June 9th
Tests the program up through the burn phase using drops
*/

/*IMU LAYOU*/

/*
+----------+
|         *| RST   PITCH  ROLL  HEADING
ADR |*        *| SCL
INT |*        *| SDA     X    Z-->    /->
PS1 |*        *| GND     |            |
PS0 |*        *| 3VO     v            \-Z
|         *| VIN
+----------+
*/

#include <Wire.h> //Wire library
#include <SFE_BMP180.h> //Barometer library
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SPI.h>
#include <SD.h>

#define FS 2 //this is the loop frequency in Hz as an integer
double frequency = 2;//this is the loop frequency in Hz as a double

Adafruit_MotorShield AFMS = Adafruit_MotorShield();// Create the motor shield object with the default I2C address

Adafruit_BNO055 bno = Adafruit_BNO055(55);//instantiates an object that refers to the IMU at the I2C address of 0x55

SFE_BMP180 pressure; //define the BMP180 barometer object (Address = 0x1D)

           // Connect a stepper motor with 200 steps per revolution (1.8 degree)
           // to motor port #1 (M1 and M2)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

const int chipSelect = 8;//this is the pin that is used for chip select on the SD card shield

const int zeroSwitch = 6;//this is the pin that the zero-position limit switch is inserted into (the other end of the switch is grounded)

const int maxSwitch = 7;//this is the pin that the maximum-position limit switch is inserted into (the other end of the switch is grounded)

const int armPin = 5;//this is the pin that the rotary switch to set the rocket into launch detect is attached to

int stepUnit = 2;//this is the # of steps that can be actuated in a 0.5 second period

File dataFile;//File object that is associated with the file being written to

char filename[] = "LOGGER00.txt";//the name for the text file that is being written to, the number will change (see further down in code)

                 /*-----------INITIALIZE GLOBAL SYSTEM STATE VARIABLES--------------*/

const double idealGasConstant = 287.058;//in J/kg*K

                    //sensor variables found using barometer
double baseline = 0.0; // baseline pressure, used to set the zero for our relative altitude measurement
double barAlt = 0.0; //in meters; variable to store the barometer's latest altitude measurement
double prevBarAlt = 0.0;//in meters - variable to store the previous barometer's altitude measurement
double barPressure = 0.0;//in Pascals
double barTemp = 0.0;//in Kelvin
double barDensity = 0.0;//in kg/m^3
double barVel = 0.0;//in m/s - the variable that stores the current vertical velocity from barometer

          //sensor variables found using IMU
double accelX = 0.0;//in m/s^2 - the variable that stores the current X-axis acceleration
double accelY = 0.0;//in m/s^2 - the variable that stores the current Y-axis acceleration
double accelZ = 0.0;//in m/s^2 - the variable that stores the current Z-axis acceleration
double eulerX = 0.0;//in deg - the variable that stores the current X-axis rotation/euler angle
double eulerY = 0.0;//in deg - the variable that stores the current Y-axis rotation/euler angle
double eulerZ = 0.0;//in deg - the variable that stores the current Z-axis rotation/euler angle
double gyroX = 0.0;//in rad/s - the variable that stores the current X-axis rotation rate
double gyroY = 0.0;//in rad/s - the variable that stores the current Y-axis rotation rate
double gyroZ = 0.0;//in rad/s - the variable that stores the current Z-axis rotation rate
double gravX = 0.0;//in m/s^2 - the variable that stores the current X-axis component of gravity
double gravY = 0.0;//in m/s^2 - the variable that stores the current Y-axis component of gravity
double gravZ = 0.0;//in m/s^2 - the variable that stores the current Z-axis component of gravity
double accelInitialBias = 0.0;//the initial acceleration bias found from the setup phase
double accelBias = 0.0;//the acceleration bias at the current
double sensitivity = 2.0;//the sensitivity in meters used in the accel bias calc

             //filter characteristics and output variables
const double tau = 0.3;//the time constant used in the filter
double gain = tau / (tau + 0.1);//the gain of the filter - CHANGES DURING COMPLEMENTARY FILTER DEPENDING ON FILTER PERIOD TIME
double filteredVelocity = 0.0;//variable the filtered velocity

                //timkeeping
int currentTime = 0; //the current system time in milliseconds
int lastTime = 0;  //the last system time in milliseconds
double currentTimeSec = 0.0; //the current loop time in seconds
double lastTimeSec = 0.0; //the last loop time in seconds
int phase = 1; //this is the switch case variable used to store the phase of flight we are in
boolean emergencyAbort = false;

//------------------INITIALIZE ADS CONTROL SYSTEM VARIABLES-----------------------------//

////flap state variables
double CdR = 0.335;
int currentStepPosition = 0;
double CdA = 0.335;
//double prevCdR = 0.335;


//Search bounds for CdS solver
double CdL = 0.1; //variable for lower search bound for CdS
double CdU = 3; //upper search bound for CdS
double CdS = 0.3;

//reset variables for search bounds

double CdLSTATIC = CdL; // reset for CdS lower search bounds
double CdUSTATIC = CdU; //reset CdL search limit
const double errTol = 0.05; //percent. specifies the search error for CdS solver
const int maxIt = 50; //maxt iterations to run for CdSsolver CHANGE BACK
const double apTar = 3048.0; //meters, apogee target
const double m = 19.856385; // UPDATE BEFORE LAUNCH   //19.707; //kg, rocket burnout mass
const double S = 0.0197; //m^2, rocket cross sectional reference area
const double g = 9.80665;//in m/s^2 - standard gravity
const double p1 = 0.002581; //const value for CFD flap actuation function SUBJECT TO CHANGE
const double p2 = 0.3228; //const value for CFD actuation function. SUBJECT TO CHANGE
double FlapPosCurrent = 0.0; //current flap posistion. This is always the actual posistion, not the commanded. (degree)
double funcErrTotal = 0.0; //percent, variable for the summed up total function error
double funcErrLast = 0.0; //percent, variable for the last calculated percent error of the CFD function
double funcERR = 0.0; //percent, variable for the mean CFD function error
int programTotIter = 1; //counts the number of times the whole program runs
const double VcutOff = 15.0; //m/s, velocity when flaps are fully closed

               //variables used for predicted limits of flap drag coeffcients
double CdAU = 2; //predicted upper range Cd of fins at full deployment
double CdAL = 0.25; //predicted lower Cd at flaps closed
double dFlapMax = 11.37; //degrees, maximum actuation at one time. SUBJECT TO CHANGE
int dStepMax = 16;
double FlapPosMax = 20; //degrees. Maximum position that the flaps can be adjusted to. SUBJECT TO CHANGE
double FlapPos = 0.0; //sets intial commanded flap position
int CdAminCdRSignOld = 0; //intializes the sign for the prop. flap actuation program
int CdAminCdRSignNew = 0;
double dFlap = 0.0; //sets actuation step
double dFlapProp = dFlapMax / 2; //degrees, the intial flap adjustment for the flaps via prop. control. SUBJECT TO CHANGE
double CdROld = 0.34; //last recorded required CdR
double CdAOld = 0.34;
//const double frequency = 1; //sec, the data polling rate. SUBJECT TO CHANGE <<<< COMMENTED OUT BEFORE POLLING RATE CHANGES

//variables used for  triple J CdR correction
const double p00 = 5.707;
const double p10 = -0.001297;
const double p01 = 0.01625;
const double p20 = -.000000241; //DBL CHECK THIS
const double p11 = -.000003447; //DBL CHECK
const double p02 = -.0000136; //DBL CHECK

                //the variables above that are not declared as constant may change, but they cannot revert back to orginal values. hence they are declared as global

                /*
                Phase 1: Waiting for arming
                Phase 2: Waiting for launch
                Phase 3: Burn phase
                Phase 4: Coast phase
                Phase 5: Apogee phase + shutdown
                Phase 6: Wait for recovery
                */

void setup(void) {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  //  /*----------------------SD CARD WRITING SETUP--------------------*/
  //  //Serial.println("starting writing to SD card");
  pinMode(53, OUTPUT);
  //
  pinMode(chipSelect, OUTPUT);
  //
  if (!SD.begin(chipSelect)) {
    //    //    Serial.println("Card failed, or not present");
    delay(50);
    //    // don't do anything more:
    return;
  }
  //  Serial.println("card initialized.");

  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (!SD.exists(filename)) {
      // only open a new file if it doesn't exist
      //      Serial.println(filename);
      dataFile = SD.open(filename, FILE_WRITE);
      dataFile.println("Header (written in setup)");
      //      dataFile.close();
      //      dataFile.flush();
      break;  // leave the loop!
    }
  }
  //
  //  Serial.println("SD card file initialized");
  //
  //  /*-------------------BAROMETER SENSOR SETUP--------------------------------*/
  if (pressure.begin()) {
    //    Serial.println("BMP180 init success");
    delay(50);
  }
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    //    Serial.println("BMP180 init fail (disconnected?)\n\n");
  }
  //finds the 0 AGL pressure
  //  Serial.println("getting baseline pressure");
  for (int i = 1; i <= 10; i++) {//gathers 10 samples of accelerationd data to find the initial bias
    baseline += getPressure();//gets the baseline pressure equivalent to 0 AGL
  }
  baseline = baseline / (10.0); //finds the average of the pressure while held still to get a final baseline pressure
                  //  Serial.println(baseline);

  double aTemp, PTemp;//prepares temporary variables for altitude and pressure
  PTemp = getPressure();//finds the current pressure
  delay(50);

  barPressure = 100.0 * PTemp; //converts P in hPa to barPressure in Pascals
  aTemp = pressure.altitude(PTemp, baseline); //finds the altitude in meters AGL
  prevBarAlt = aTemp;//sets the first prevBarAlt equal to the current altitude. used to find the first barometric vertical velocity calculation.
             //  //  Serial.println(aTemp);
             //
             //  /*---------------INERTIAL MEASUREMENT UNIT (IMU) SENSOR SETUP---------------*/
             //
             //  //BEGIN CALIBRATION
  if (!bno.begin())
  {
    //    /* There was a problem detecting the BNO055 ... check your connections */
    //    //    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    delay(50);
    while (1);
  }
  bno.setExtCrystalUse(true);//Dont remember what this does but this is necessary
                 //
                 //
  digitalWrite(LED_BUILTIN, LOW);//sets the led in pin 13 to LOW
                   //
  pinMode(armPin, INPUT_PULLUP);//the rotary switch to set system to launch detect mode
                  //
  bool calibrated = false;
  do { //loop continues until system1 is equal to 3, which means the sensor is fully calibrated (0 is uncalibrated, and 3 is maximum calibration)
    uint8_t systemCal, gyroCal, accelCal, magCal = 0;
    bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);
    //    //      Serial.print("CALIBRATION: Sys=");
    //    //      Serial.print(systemCal, DEC);
    //    //      Serial.print(" Gyro=");
    //    //      Serial.print(gyroCal, DEC);
    //    //      Serial.print(" Accel=");
    //    //      Serial.print(accelCal, DEC);
    //    //      Serial.print(" Mag=");
    //    //      Serial.println(magCal, DEC);
    if (systemCal >= 3 && gyroCal >= 3 && accelCal >= 3 && magCal >= 3) {
      calibrated = true;
      //      //      digitalWrite(LED_BUILTIN, HIGH);//sets the led in pin 13 to HIGH to signal that the system is calibrated
    }
    else {
      calibrated = false;
      //      //      digitalWrite(LED_BUILTIN, LOW);//sets the led in pin 13 to LOW
    }
    delay(100);
  } while (calibrated == false);
  //
  //
  //
  //  //END CALIBRATION
  //
  //  //finds the initial accelerometer bias
  for (int i = 1; i <= 10; i++) {//gathers 10 samples of accelerationd data to find the initial bias
    imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    accelZ = accelZ + linaccel.z();
  }
  accelBias = accelZ / 10.0;//averages the ten prior samples
  accelInitialBias = accelBias;//sets the initial accel bias to the current bias
  accelZ = 0.0;//resets current accel measurement to zero


         /*---------------STEPPER MOTOR SYSTEM START + CALIBRATION-------------------*/

         //the ratio between the input step and the output flap angle is related by (76 steps of the stepper motor) = (54 flap angle degrees)
         //
         //check stepper motor wiring! M1 -> M2 wiring order (left to right): GREEN, BLACK, BLUE, RED. Leave YELLOW and WHITE disconnected
  pinMode(zeroSwitch, INPUT_PULLUP);//when the switch is OPEN, this pin is HIGH.
  pinMode(maxSwitch, INPUT_PULLUP);//when the switch is CLOSED, this pin is LOW.

  AFMS.begin();  // create with the default frequency 1.6KHz
           //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  myMotor->setSpeed(10);  // set to 10 rpm

  myMotor->step(28, FORWARD, SINGLE);
  myMotor->step(28, BACKWARD, SINGLE);
  myMotor->step(28, FORWARD, SINGLE);
  myMotor->step(28, BACKWARD, SINGLE);
  delay(10);
  myMotor->step(28, FORWARD, SINGLE);
  myMotor->step(28, BACKWARD, SINGLE);
  myMotor->step(28, FORWARD, SINGLE);
  myMotor->step(28, BACKWARD, SINGLE);

  while (digitalRead(zeroSwitch) != LOW) {//digitalRead(zeroSwitch) will read LOW when the switch is pressed
    myMotor->step(2, BACKWARD, SINGLE);
  }
  currentStepPosition = 0;

  myMotor->release();//ENSURES THAT STEPPER IS TURNED OFF SO THAT IT CAN'T WASTE POWER

             /*------------------------------BEGIN TIME KEEPING--------------------------*/

  lastTime = millis();//gets the first last reading before we enter the main loop
  lastTimeSec = ((double)lastTime) / 1000;//converts to a second
  dataFile.close();
}

void loop() {
  int ph2r = 1; //TEST
          /*
          Phase 1: Waiting for arming
          Phase 2: Waiting for launch
          Phase 3: Burn phase
          Phase 4: Coast phase
          Phase 5: Apogee phase + shutdown
          Phase 6: Wait for recovery
          */
  if (phase == 1) {
    currentTime = millis();
    if (abs(currentTime - lastTime) >= 100) {//100 ms time loop = 10 Hz operation
      currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
      lastTime = currentTime;
      lastTimeSec = currentTimeSec;

      /*--------------------------Phase 1: WAITING FOR ARMING----------------------------*/
      if (digitalRead(armPin) == LOW) {//will set the phase to launch detect once the switch is armed
        phase = 2;//phase set to 2, corresponding to launch detect
      }
      //      Serial.println("Phase 1");
    }
  }
  else if (phase == 2) {


    //if (ph2r == 1) { //TEST
    //  Serial.println('G'); //TEST
    //  ph2r = 0; //TEST
    //} //TEST

    // if ((Serial.available()>0) && (accelZ < 2*g)){
    //      float data = Serial.parseFloat(); //read in Matlab data
    //    accelZ = double(data);
    //
    //    }
    //    if (accelZ > (2*g)){
    //      //for (int i=0; i<=10;i++) {
    //  Serial.println('3');
    //    }
    //Everything above = TEST

    currentTime = millis();
    if (abs(currentTime - lastTime) >= 100) {//100 ms time loop = 10 Hz operation
      currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
      lastTime = currentTime;
      lastTimeSec = currentTimeSec;


      /*------------------------Phase 2: LAUNCH DETECT---------------------------------*/
      complementary(0.1);//this will update the current pressure, current altitude, current density, current acceleration, and current vertical velocity using filters
                 //the 0.1 means that it is using 0.1 seconds (10 Hz) for the period when calculating velocity

      writeToCard();

      if (accelZ > 2.0 * g) { //will set the phase to burn phase/launch detected if the acceleration goes above 2G
        phase = 3;
      }
      //      Serial.println("Phase 2");
    }
  }
  else if (phase == 3) {
    currentTime = millis();
    if (abs(currentTime - lastTime) >= 100) {//100 ms time loop = 10 Hz operation
      currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
      lastTime = currentTime;
      lastTimeSec = currentTimeSec;
      /*------------------------Phase 3: BURN PHASE------------------------------------*/
      complementary(0.1);//this will update the current pressure, current altitude, current density, current acceleration, and current vertical velocity using filters
                 //the 0.1 means that it is using 0.1 seconds (10 Hz) for the period when calculating velocity

      writeToCard();
      //      if (digitalRead(armPin) == HIGH) {//will set the phase to launch detect once the switch is armed
      //        phase = 4;//phase set to 2, corresponding to launch detect
      //        //        delay(10);
      //      }
      if (accelZ < (-0.8 * g) && barAlt > 1050.0) { //will set to coast phase if the acceleration is less than -1 G and the altitude is above 1200 m (3937 ft)
        phase = 4;
      }
      //    Serial.println("Phase 3");
    }
  }
  else if (phase == 4) {
    currentTime = millis();
    if (abs(currentTime - lastTime) >= 500) {//500 ms time loop = 2 Hz operation
      currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
      lastTime = currentTime;
      lastTimeSec = currentTimeSec;
      /*-----------------------Phase 4: COAST PHASE + ADS OPERATION-------------------*/
      complementary(0.5);

      if (eulerZ > 45.0 || eulerZ < -45.0) { //NEW
        emergencyAbort = true;

      }
      //old CdA func
      // CdA = 2 * abs((m * accelZ) + (m * g)) / (barDensity * filteredVelocity * filteredVelocity * S);
      CdA = 2 * (((m * accelZ) + (m * g)) / (-barDensity * filteredVelocity * filteredVelocity * S));
      //new CdA above
      // ----------------------------CFD FUNCTION ERROR CALC---------------------------
      // double CdAFuncGuess = ((p1 * FlapPosCurrent) + p2); //finds what the CFD function thinks CdA should be
      //funcErrLast = ((abs(CdAFuncGuess - CdA)) / CdA) * 100; //last calculated CFD function error, percent
      //funcErrTotal = funcErrTotal + funcErrLast; //percent, calculates the total percent error
      funcERR = 1000000.0;//funcErrTotal / programTotIter; //percent, calculates the mean CFD function error
                //---------------------END CFD FUNCTION ERROR CALC---------------------------------

      if (filteredVelocity > VcutOff && emergencyAbort == false) {
        //----------------------CDAU AND CDAL SANITY CHECK------------------------
        //checks to see if CdA is out of bounds of specified limts
        //if so, redfines limits
        if (CdA > CdAU) {
          CdAU = CdA; //sets new upper bound equal to CdA
        }
        if (CdA < CdAL) {
          CdAL = CdA;
          //same as above
          if (CdAL <= 0) {
            CdAL = 0.06;
          }
        }
        if (CdAL < CdL) {
          CdL = CdAL - 0.02;
          CdLSTATIC = CdL;
  9          if (CdL <= 0) {
            CdL = 0.02;
            CdLSTATIC = CdL;
          }
        }
        if (CdAU > CdU) {
          CdU = CdAU + 10;
          CdUSTATIC = CdU;
        }


        //------------------- END SANITY CHECK-----------------------------------

        //---------------------TRIPLE J CDR FINDER-------------------------------
        double PercentError = p00 + (p10 * barAlt) + (p01 * filteredVelocity) + (p20 * (barAlt * barAlt)) + (p11 * barAlt * filteredVelocity) + (p02 * (filteredVelocity * filteredVelocity)); //calculates the percent error on CdS vs CdR
                                                                                                     // funcRT = ((((-((m*(log((2*g*m))))/(Cds*barDensity*S)))-((-(m*(log((Cds*barDensity*S*(Vi^2))+(2*g*m))))/(Cds*barDensity*S))))+Xi) - apTar) (@Cds);

                                                                                                     //---------------CDS FINDER WITHIN TRIPLE J-----------------------------
        int iterCDS = 0; //counts number of iterations within simple Cd solver
        int runCDS = 1; //contrls while loop. 1 = run
        double pError = 100; //percent error of CdS calculation
        if (((((-((m * (log((2 * g * m)))) / (CdL * barDensity * S))) - ((-(m * (log((CdL * barDensity * S * (filteredVelocity * filteredVelocity)) + (2 * g * m)))) / (CdL * barDensity * S)))) + barAlt) - apTar) <= 0) { //checks if solution is possible

          CdS = CdL; //sets CdS equal to lower bound
          runCDS = 0; //ends CdS solver

        }

        if (((((-((m * (log((2 * g * m)))) / (CdU * barDensity * S))) - ((-(m * (log((CdU * barDensity * S * (filteredVelocity * filteredVelocity)) + (2 * g * m)))) / (CdU * barDensity * S)))) + barAlt) - apTar) >= 0) {
          CdS = CdU; //sets CdS to CdU
          runCDS = 0; //terminates CDS solver
        }
        // while loop bisection for CdS
        while ((pError > errTol) && (runCDS == 1) && (iterCDS < maxIt)) {
          CdS = (CdL + CdU) / 2; //assumes CdS is midpt of search interval
          iterCDS = iterCDS + 1; //counts number of iterations
                       //determines if the solution is in the search interval
          double test = ((((-((m * (log((2 * g * m)))) / (CdL * barDensity * S))) - ((-(m * (log((CdL * barDensity * S * (filteredVelocity * filteredVelocity)) + (2 * g * m)))) / (CdL * barDensity * S)))) + barAlt) - apTar) * ((((-((m * (log((2 * g * m)))) / (CdS * barDensity * S))) - ((-(m * (log((CdS * barDensity * S * (filteredVelocity * filteredVelocity)) + (2 * g * m)))) / (CdS * barDensity * S)))) + barAlt) - apTar);
          if (test < 0) {
            CdU = CdS; //halfs the search interval
          }
          if (test > 0) {
            CdL = CdS; //same as above
          }
          if (test == 0) {
            runCDS = 0; //terminates program. CdS is perfect solution
          }
          //calculate percent error in CdS, not relative to CdR
          pError = (abs(((((-((m * (log((2 * g * m)))) / (CdS * barDensity * S))) - ((-(m * (log((CdS * barDensity * S * (filteredVelocity * filteredVelocity)) + (2 * g * m)))) / (CdS * barDensity * S)))) + barAlt) - apTar)) / apTar) * 100;
        }
        //reset search bounds
        CdU = CdUSTATIC;
        CdL = CdLSTATIC;
        //----------------------------END CDS FINDER WITHIN TRIPLE J ALG-----------------------------
        CdR = ((CdS * (PercentError / 100))) + CdS; //corrects CdS to get CdR
                              //------------------------END TRIPLE J CDR FINDER---------------------------------------------
                              //----------------------------------------FLAP ADJUSTMENT CALC--------------------------------------
        if ((CdR > CdAL) && (CdR < CdAU)) {
          if (0.0 >(funcERR)) { //only runs cfd function if it estimated to be within a fine enough error tolerance
            FlapPos = (((CdR / 4) - p2) / p1); //sets commanded flap posistion;
                               //            if (FlapPos > FlapPosMax) {
                               //              FlapPos = FlapPosMax; //doesnt allow for over extending flaps
                               //            }
                               //            if (FlapPos < 0) {
                               //              FlapPos = 0; //doesnt allow for negative commands
                               //            }
                               //            dFlap = (FlapPos - FlapPosCurrent); //determines dFlap
                               //            if ((abs(dFlap) > dFlapMax)) {
                               //              if (dFlap < 0) {
                               //                dFlap = -dFlapMax; //sets to max possible actuation
                               //              }
                               //              if (dFlap > 0) {
                               //                dFlap = dFlapMax; //same as above
                               //              }
                               //              FlapPos = FlapPosCurrent + dFlap; //adjust flap posistion command
                               //            }

          }
          else {
            if (CdA > CdR) {
              CdAminCdRSignNew = 0;
            }
            if (CdA < CdR) {
              CdAminCdRSignNew = 1;
            }
            if (CdAminCdRSignNew != CdAminCdRSignOld) {
              dFlapProp = dFlapProp / 2; //reduces step by 1/2
            }
            else {
              if ((abs(CdROld - CdR)) > dFlapProp) {
                dFlapProp = dFlapProp * 5; //drastically increases stepsize id dCdR outpaces dCdA
              }
              else {
                if (CdAminCdRSignNew == CdAminCdRSignOld && (abs(CdA - CdR) > ((CdAU - CdAL) / FlapPosMax)*dFlapProp)) { //last term is estimate of new CdA
                  dFlapProp = dFlapProp * 1.5; //increases step size
                }
              }
            }
            if (dFlapProp > dFlapMax) {
              dFlapProp = dFlapMax; //prevents over stepping
            }
            if (CdA > CdR) {
              FlapPos = FlapPosCurrent - dFlapProp; //adjusts commanded flap position
            }
            if (CdA < CdR) {
              FlapPos = FlapPosCurrent + dFlapProp; //same as above
            }
            //reset variables
            CdAOld = CdA;
            CdROld = CdR;
            CdAminCdRSignOld = CdAminCdRSignNew;

          }
        }
        else {
          if (CdR >= CdAU) {
            dFlap = FlapPosMax - FlapPosCurrent; //sets flap actuation step
            if (dFlap > dFlapMax) {
              dFlap = dFlapMax;
            }
            FlapPos = FlapPosCurrent + dFlap;
          }
          else {
            if (CdR <= CdAL) {
              dFlap = FlapPosCurrent;
              if (dFlap > dFlapMax) {
                dFlap = dFlapMax;
              }
              FlapPos = FlapPosCurrent - dFlap;
            }
          }

        }

        if (digitalRead(zeroSwitch) == LOW) {//digitalRead(zeroSwitch) will read low when the switch is pressed
          currentStepPosition = 0; //NEW
        }

        if (eulerZ > 30.0 || eulerZ < -30.0) { //NEW
          FlapPos = 0;
        }





        if (FlapPos > FlapPosMax) {
          FlapPos = FlapPosMax; //doesnt allow for over extending flaps
        }
        if (FlapPos < 0) {
          FlapPos = 0; //doesnt allow for negative commands
        }
        dFlap = (FlapPos - FlapPosCurrent); //determines dFlap
        if ((abs(dFlap) > dFlapMax)) {
          if (dFlap < 0) {
            dFlap = -dFlapMax; //sets to max possible actuation
          }
          if (dFlap > 0) {
            dFlap = dFlapMax; //same as above
          }
          FlapPos = FlapPosCurrent + dFlap; //adjust flap posistion command
        }

        writeToCard();

        //insert motor command based upon FlapPos
        //then, after command, update FlapPosCurrent in angle degrees
        int StepR = (int)(FlapPos * 76.0 / 54.0);
        int dStep = 0;
        if (currentStepPosition < StepR)
        {
          dStep = StepR - currentStepPosition;
          if (dStep > dStepMax)
          {
            dStep = dStepMax;
          }
          myMotor->setSpeed(10);
          myMotor->step(dStep, FORWARD, SINGLE);
          currentStepPosition += dStep;

        }
        else if (currentStepPosition > StepR) {
          dStep = currentStepPosition - StepR;
          if (dStep > dStepMax) {
            dStep = dStepMax;
          }
          myMotor->setSpeed(10);
          myMotor->step(dStep, BACKWARD, SINGLE);
          currentStepPosition -= dStep;
        }

        FlapPosCurrent = ((double)currentStepPosition) * (54.0 / 76.0);


        //-------------------------------END FLAP ADJUSTMENT-------------------------------
      }

      //------------------------------*END PRE-APOGEE SECTION*--------------------------------

      //----------------------*APOGEE APPROACH SECTION*-----------------------------------------

      //----------------------FLAP CLOSING PROCEDURE---------------------------------------
      else if (((filteredVelocity < VcutOff) && (digitalRead(zeroSwitch) == HIGH)) || emergencyAbort == true) {
        dFlap = FlapPosCurrent;
        if (dFlap > dFlapMax) {
          dFlap = dFlapMax;
        }
        FlapPos = FlapPosCurrent - dFlap;

        if (digitalRead(zeroSwitch) == LOW) {//digitalRead(zeroSwitch) will read low when the switch is pressed
          currentStepPosition = 0;
        }

        writeToCard();
        //insert motor command based upon FlapPos
        //then, after command, update FlapPosCurrent in angle degrees
        int StepR = (int)(FlapPos * 76.0 / 54.0);
        int dStep = 0;
        if (currentStepPosition < StepR)
        {
          dStep = StepR - currentStepPosition;
          if (dStep > dStepMax)
          {
            dStep = dStepMax;
          }
          myMotor->setSpeed(10);
          myMotor->step(dStep, FORWARD, SINGLE);
          currentStepPosition += dStep;

        }
        else if (currentStepPosition > StepR) {
          dStep = currentStepPosition - StepR;
          if (dStep > dStepMax) {
            dStep = dStepMax;
          }
          myMotor->setSpeed(10);
          myMotor->step(dStep, BACKWARD, SINGLE);
          currentStepPosition -= dStep;
        }

        if (currentStepPosition < 2) {
          myMotor->release();
        }

        FlapPosCurrent = ((double)currentStepPosition) * (54.0 / 76.0);
        if (digitalRead(zeroSwitch) == LOW) { //NEW, dbl check this, as it may put program into phase 5 too early
          phase = 5;
        }

      }
      //---------------END FLAP CLOSING--------------------------------------------

      //---------------END *POST-APOGEE SECTION*----------------------------------

      programTotIter = programTotIter + 1;

    }//keep all the flap actuation crap inside this bracket


  }
  else if (phase == 5) { //NEW, ALL OF PHASE 5
    currentTime = millis();
    if (abs(currentTime - lastTime) >= 100) {//100 ms time loop = 10 Hz operation
      currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
      lastTime = currentTime;
      lastTimeSec = currentTimeSec;
      complementary(0.1);
      myMotor->release();

      if (barAlt >= 50.0) {
        if (digitalRead(zeroSwitch) == LOW) {//digitalRead(zeroSwitch) will read low when the switch is pressed
          currentStepPosition = 0;
        }
        writeToCard();
      }
      else {
        phase == 6;
      }
    }
  }
}

double getPressure()
{
  char status;
  double T, P, p0, a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      barTemp = (double)T;
      barTemp += 273.15;//converts barTemp to Kelvin
                // Start a pressure measurement:
                // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
                // If request is successful, the number of ms to wait is returned.
                // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(0);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
// TEMP FOR ALL OF VOID GET PRESSURE

//}

void complementary(double timePeriod) {
  //  char var1 = 'a';
  //
  //if (phase == 3){
  //  int numVar = 0;
  //  int V1 = 1;
  //  int V2 = 1;
  //  while (numVar < 2) {
  //
  //
  //    if (Serial.available() > 0){
  //      char data = Serial.read();
  //      if ((data = 'A') && (V1 == 1)){
  //        float barAltF = Serial.parseFloat();
  //      barAlt = double(barAltF);
  //      V1 = 0;
  //      numVar = numVar+1;
  //      Serial.println('1');
  //      }
  //      if ((data = 'X') && (V2 == 1)){
  //        float accelZF = Serial.parseFloat();
  //      accelZ = double(accelZF);
  //      V2 = 0;
  //      numVar = numVar+1;
  //      }
  //    }
  //}
  //
  //String CdAstr = 'C'+String(CdA);
  ////for (int i=0; i<=1;i++) {
  //  Serial.println(CdAstr);
  //
  //  String FiltVelstr = 'V'+String(filteredVelocity);
  ////for (int i=0; i<=1;i++) {
  //while (Serial.read() != 'C'){
  //
  //}
  //  Serial.println(FiltVelstr);
  //
  //}
  //
  //    if (phase == 4){
  //  int numVar = 0;
  //  int V1 = 1;
  //  int V2 = 1;
  //  int V3 = 1;
  //
  //  while (numVar < 3) {
  //
  //
  //    if (Serial.available() > 0){
  //      char data = Serial.read();
  //      if ((data = 'A') && (V1 == 1)){
  //        float barAltF = Serial.parseFloat();
  //      barAlt = double(barAltF);
  //      V1 = 0;
  //      numVar = numVar+1;
  //      Serial.println('1');
  //      }
  //      if ((data = 'X') && (V2 == 1)){
  //        float accelZF = Serial.parseFloat();
  //      accelZ = double(accelZF);
  //      V2 = 0;
  //      numVar = numVar+1;
  //      Serial.println('2');
  //      }
  //       if ((data = 'D') && (V3 == 1)){
  //        float DenF = Serial.parseFloat();
  //      barDensity = double(DenF);
  //      V3 = 0;
  //      numVar = numVar+1;
  //      }
  //    }
  //}
  //String CdAstr = 'C'+String(CdA);
  ////for (int i=0; i<=1;i++) {
  //  Serial.println(CdAstr);
  //
  //  String FlapPosStr = 'F'+String(FlapPosCurrent);
  ////for (int i=0; i<=1;i++) {
  //
  //while (Serial.read() != 'C'){
  //
  //}
  //  Serial.println(FlapPosStr);
  //
  //
  //
  //
  //
  //  String CdRstr = 'R'+String(CdR);
  ////for (int i=0; i<=1;i++) {
  //while (Serial.read() != 'F'){
  //
  //}
  //  Serial.println(CdRstr);
  //
  //
  //  String Vel4str = 'V'+String(filteredVelocity);
  ////for (int i=0; i<=1;i++) {
  //while (Serial.read() != 'R'){
  //
  //}
  //  Serial.println(Vel4str);
  //
  //  String FlapPosJJStr = 'P'+String(FlapPos);
  ////for (int i=0; i<=1;i++) {
  //while (Serial.read() != 'V'){
  //
  //}
  //  Serial.println(FlapPosJJStr);
  //
  //    String FuncErrStr = 'E'+String(funcERR);
  ////for (int i=0; i<=1;i++) {
  //while (Serial.read() != 'P'){
  //
  //}
  //  Serial.println(FuncErrStr);
  //
  //}
  // ALL TEST ABOVE











  double P;//new variable to temporarily store pressure number
  P = getPressure();//gets the current pressure
  barPressure = 100.0 * P; //converts P in hPa to barPressure in Pascals and updates the current pressure
  barAlt = pressure.altitude(P, baseline); //in meters AGL - gets the new current altitude
  barDensity = barPressure / (idealGasConstant * barTemp);//kg/m^3 - gets the new current density

  imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);//polls IMU sensor for accelerometer data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);//polls IMU sensor for Euler angle data
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);//polls IMU sensor for gyroscope data
                                       //  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);//polls IMU sensor for gravity vector data

  accelX = linaccel.x();//updates the current x-axis-acceleration
  accelY = linaccel.y();//updates the current y-axis-acceleration
  accelZ = linaccel.z() - accelBias;//updates the current z-axis-acceleration

  eulerX = euler.x();//updates the current x-axis-acceleration
  eulerY = euler.y();//updates the current y-axis-acceleration
  eulerZ = euler.z();//updates the current z-axis-acceleration

  gyroX = gyro.x();//updates the current x-axis-acceleration
  gyroY = gyro.y();//updates the current y-axis-acceleration
  gyroZ = gyro.z();//updates the current z-axis-acceleration


           //
           //  gravX = gravity.x();//updates the current x-axis-acceleration
           //  gravY = gravity.y();//updates the current y-axis-acceleration
           //  gravZ = gravity.z();//updates the current z-axis-acceleration

  gain = tau / (tau + timePeriod);//the gain of the filter
  double altDelta = barAlt - prevBarAlt;
  barVel = altDelta / timePeriod;//gets the barometric velocity
  prevBarAlt = barAlt;//sets the previous barometric altitude to the current barometric altitude
  filteredVelocity = (gain) * (filteredVelocity + (accelZ)* timePeriod) + (1.0 - gain) * (barVel);//complementary filter
  if (filteredVelocity == 0.0 || abs(altDelta) <= sensitivity)//sets the new accelerometer bias to help reduce drift
  {
    accelBias = accelBias + (accelZ / 10.0);
  }
}


void writeToCard() {
  String dataString = "";
  /*Time (sec), Altitude (m), Density (kg/m^3), AccelX (m/s^2), AccelY (m/s^2), AccelZ (m/s^2), AccelZBias (m/s^2),
  EulerX (deg), EulerY (deg),  EulerZ (deg), GyroX (rad/s), GyroY (rad/s), GyroZ (rad/s),  FilteredVelocity (m/s), Cd Required, Step Position, Current Cd*/

  dataString.concat(currentTimeSec);
  dataString.concat(",");
  dataString.concat(barAlt);
  dataString.concat(",");
  dataString.concat(barDensity);
  dataString.concat(",");
  dataString.concat(accelX);
  dataString.concat(",");
  dataString.concat(accelY);
  dataString.concat(",");
  dataString.concat(accelZ);
  dataString.concat(",");
  dataString.concat(accelBias);
  dataString.concat(",");
  dataString.concat(eulerX);
  dataString.concat(",");
  dataString.concat(eulerY);
  dataString.concat(",");
  dataString.concat(eulerZ);
  dataString.concat(",");
  dataString.concat(gyroX);
  dataString.concat(",");
  dataString.concat(gyroY);
  dataString.concat(",");
  dataString.concat(gyroZ);
  dataString.concat(",");
  dataString.concat(filteredVelocity);
  dataString.concat(",");
  dataString.concat(CdR);
  dataString.concat(",");
  dataString.concat(currentStepPosition);
  dataString.concat(",");
  dataString.concat(CdA);

  dataFile = SD.open(filename, FILE_WRITE);
  //  Serial.println(dataString);
  if (dataFile) {
    dataFile.println(dataString);
    //        dataFile.flush();
    //    Serial.println("Wrote state to card");
  }
  dataFile.close();
}

// ALL TEMP ABOVE OF WRITE TO SD
