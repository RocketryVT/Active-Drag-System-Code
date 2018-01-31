/* started June 9th
Tests the program up through the burn phase using drops
*/

/*IMU LAYOUT*/

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
#include "globalVariables.h"
#include "adsVariables.h"

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

								 

		/*
		Phase 1: Waiting for arming
		Phase 2: Waiting for launch
		Phase 3: Burn phase
		Phase 4: Coast phase
		Phase 5: Apogee phase + shutdown
		Phase 6: Wait for recovery
		*/

void sanityCheck();
void setup();
void complementary(double timePeriod);
void writeToCard();

void phase1();
void phase2();
void phase3();
void phase4();
void phase5();



int main()
{

	phase = 1;
	Serial.println("Start Phase 1\n");
	while (phase == 1)
	{
		phase1();
	}
	Serial.println("End Phase 1\n");
	Serial.println("Start Phase 2\n");
	while (phase == 2)
	{
		phase2();
	}
	Serial.println("End Phase 2\n");
	Serial.println("Start Phase 3\n");
	while (phase == 3)
	{
		phase3();
	}
	Serial.println("End Phase 3\n");
	Serial.println("Start Phase 4\n");
	while (phase == 4)
	{
		phase4();
	}
	Serial.println("End Phase 4\n");
	Serial.println("Start Phase 5\n");
	while (phase == 5)
	{
		phase5();
	}
	Serial.println("End Phase 5\n");
	
}



void phase1()
{
	currentTime = millis();
	if (abs(currentTime - lastTime) >= 100) //100 ms time loop = 10 Hz operation
	{
		currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
		lastTime = currentTime;
		lastTimeSec = currentTimeSec;

		/*--------------------------Phase 1: WAITING FOR ARMING----------------------------*/
		if (digitalRead(armPin) == LOW) //will set the phase to launch detect once the switch is armed
		{
			phase = 2;//phase set to 2, corresponding to launch detect
		}
				
	}
}

void phase2()
{

	currentTime = millis();
	if (abs(currentTime - lastTime) >= 100) //100 ms time loop = 10 Hz operation
	{
		currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
		lastTime = currentTime;
		lastTimeSec = currentTimeSec;


		/*------------------------Phase 2: LAUNCH DETECT---------------------------------*/
		complementary(0.1); // Runs the Complementary Function Defined below (in the future maybe in a different header file)

		writeToCard();

		if (accelZ > 2.0 * g)  //will set the phase to burn phase/launch detected if the acceleration goes above 2G
		{
			phase = 3;
		}
		//      Serial.println("Phase 2");
	}
}

void phase3()
{
	currentTime = millis();
	if (abs(currentTime - lastTime) >= 100) //100 ms time loop = 10 Hz operation
	{
		currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
		lastTime = currentTime;
		lastTimeSec = currentTimeSec;
		/*------------------------Phase 3: BURN PHASE------------------------------------*/
		complementary(0.1); //Runs the Complementary Function

		writeToCard();
		
		if (accelZ < (-0.8 * g) && barAlt > 1050.0)  //will set to coast phase if the acceleration is less than -1 G and the altitude is above 1050 m (3445 ft)
		{
			phase = 4;
		}
	}
}

void phase4()
{
	currentTime = millis();
	if (abs(currentTime - lastTime) >= 500) //500 ms time loop = 2 Hz operation
	{
		currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
		lastTime = currentTime;
		lastTimeSec = currentTimeSec;
		
		complementary(0.5);
		
		/*-----------------------Phase 4: COAST PHASE + ADS OPERATION-------------------*/
		if (eulerZ > 45.0 || eulerZ < -45.0) //Emergency Abort for angles greater than 45
		{ 
			emergencyAbort = true;
		}
		
		CdA = 2 * (((m * accelZ) + (m * g)) / (-barDensity * filteredVelocity * filteredVelocity * S));
		
		//----------------------------CFD FUNCTION ERROR CALC----------------------------//
		// double CdAFuncGuess = ((p1 * FlapPosCurrent) + p2); //finds what the CFD function thinks CdA should be
		//funcErrLast = ((abs(CdAFuncGuess - CdA)) / CdA) * 100; //last calculated CFD function error, percent
		//funcErrTotal = funcErrTotal + funcErrLast; //percent, calculates the total percent error
		
		funcERR = 1000000.0;//funcErrTotal / programTotIter; //percent, calculates the mean CFD function error
		//---------------------END CFD FUNCTION ERROR CALC---------------------//

		if (filteredVelocity > VcutOff && emergencyAbort == false) 
		{
			
			sanityCheck();

			//---------------------TRIPLE J CDR FINDER---------------------//
			double PercentError = p00 + (p10 * barAlt) + (p01 * filteredVelocity) + (p20 * (barAlt * barAlt)) + (p11 * barAlt * filteredVelocity) + (p02 * (filteredVelocity * filteredVelocity)); //calculates the percent error on CdS vs CdR
			
			//---------------CDS FINDER WITHIN TRIPLE J---------------//
			int iterCDS = 0; //counts number of iterations within simple Cd solver
			int runCDS = 1; //contrls while loop. 1 = run
			double pError = 100; //percent error of CdS calculation
			if (((((-((m * (log((2 * g * m)))) / (CdL * barDensity * S))) - ((-(m * (log((CdL * barDensity * S * (filteredVelocity * filteredVelocity)) + (2 * g * m)))) / (CdL * barDensity * S)))) + barAlt) - apTar) <= 0) //checks if solution is possible
			{ 

				CdS = CdL; //sets CdS equal to lower bound
				runCDS = 0; //ends CdS solver

			}
			
			if (((((-((m * (log((2 * g * m)))) / (CdU * barDensity * S))) - ((-(m * (log((CdU * barDensity * S * (filteredVelocity * filteredVelocity)) + (2 * g * m)))) / (CdU * barDensity * S)))) + barAlt) - apTar) >= 0) 
			{
				CdS = CdU; //sets CdS to CdU
				runCDS = 0; //terminates CDS solver
			}
			
			// while loop bisection for CdS
			while ((pError > errTol) && (runCDS == 1) && (iterCDS < maxIt)) 
			{
				CdS = (CdL + CdU) / 2; //assumes CdS is midpt of search interval
				iterCDS = iterCDS + 1; //counts number of iterations
									   
				
				double test = ((((-((m * (log((2 * g * m)))) / (CdL * barDensity * S))) - ((-(m * (log((CdL * barDensity * S * (filteredVelocity * filteredVelocity)) + (2 * g * m)))) / (CdL * barDensity * S)))) + barAlt) - apTar) * ((((-((m * (log((2 * g * m)))) / (CdS * barDensity * S))) - ((-(m * (log((CdS * barDensity * S * (filteredVelocity * filteredVelocity)) + (2 * g * m)))) / (CdS * barDensity * S)))) + barAlt) - apTar);
				
				if (test < 0) 
				{
					CdU = CdS; //halfs the search interval
				}
				else if (test > 0) 
				{
					CdL = CdS; //same as above
				}
				else 
				{
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
			//------------------------END TRIPLE J CDR FINDER---------------------
			//------------------------FLAP ADJUSTMENT CALC------------------------


			if ((CdR >= CdAL) && (CdR <= CdAU)) 
			{
				if (0.0 >(funcERR))  //only runs cfd function if it estimated to be within a fine enough error tolerance
				{
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
				else 
				{
					if (CdA > CdR) 
					{
						CdAminCdRSignNew = 0;
					}
					else if (CdA < CdR) 
					{
						CdAminCdRSignNew = 1;
					}
					
					if (CdAminCdRSignNew != CdAminCdRSignOld) 
					{
						dFlapProp = dFlapProp / 2; //reduces step by 1/2
					}
					else 
					{
						if ((abs(CdROld - CdR)) > dFlapProp) 
						{
							dFlapProp = dFlapProp * 5; //drastically increases stepsize id dCdR outpaces dCdA
						}
						else 
						{
							if (CdAminCdRSignNew == CdAminCdRSignOld && (abs(CdA - CdR) > ((CdAU - CdAL) / FlapPosMax)*dFlapProp))  //last term is estimate of new CdA
							{
								dFlapProp = dFlapProp * 1.5; //increases step size
							}
						}
					}
					
					/*THIS PART CHOOSES WHETHER THE FLAPS ARE GOING OUT OR IN*/
					if (dFlapProp > dFlapMax) 
					{
						dFlapProp = dFlapMax; //prevents over stepping
					}
					if (CdA > CdR) 
					{
						FlapPos = FlapPosCurrent - dFlapProp; //adjusts commanded flap position
					}
					if (CdA < CdR) 
					{
						FlapPos = FlapPosCurrent + dFlapProp; //same as above
					}
					//reset variables
					CdAOld = CdA;
					CdROld = CdR;
					CdAminCdRSignOld = CdAminCdRSignNew;

				}
			}
			else 
			{
				if (CdR > CdAU) 
				{
					dFlap = FlapPosMax - FlapPosCurrent; //sets flap actuation step
					if (dFlap > dFlapMax) 
					{
						dFlap = dFlapMax;
					}
					FlapPos = FlapPosCurrent + dFlap;
				}
				else 
				{
					if (CdR < CdAL) 
					{
						dFlap = FlapPosCurrent;
						if (dFlap > dFlapMax) 
						{
							dFlap = dFlapMax;
						}
						FlapPos = FlapPosCurrent - dFlap;
					}
				}

			}


			if (digitalRead(zeroSwitch) == LOW) //digitalRead(zeroSwitch) will read low when the switch is pressed
			{
				currentStepPosition = 0; //NEW
			}

			/*HOPEFULLY ALLOWS THE ROCKET TO CORRECT ITSELF BASED ON THE CP / CG */
			if (eulerZ > 30.0 || eulerZ < -30.0)  //NEW
			{
				FlapPos = 0;
			}





			if (FlapPos > FlapPosMax) 
			{
				FlapPos = FlapPosMax; //doesnt allow for over extending flaps
			}
			if (FlapPos < 0) 
			{
				FlapPos = 0; //doesnt allow for negative commands
			}
			dFlap = (FlapPos - FlapPosCurrent); //determines dFlap
			if ((abs(dFlap) > dFlapMax)) 
			{
				if (dFlap < 0) 
				{
					dFlap = -dFlapMax; //sets to max possible actuation
				}
				else if (dFlap > 0) 
				{
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
			else if (currentStepPosition > StepR) 
			{
				dStep = currentStepPosition - StepR;
				if (dStep > dStepMax) 
				{
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
		else if (((filteredVelocity < VcutOff) && (digitalRead(zeroSwitch) == HIGH)) || emergencyAbort == true) 
		{
			dFlap = FlapPosCurrent;
			if (dFlap > dFlapMax) 
			{
				dFlap = dFlapMax;
			}
			FlapPos = FlapPosCurrent - dFlap;

			if (digitalRead(zeroSwitch) == LOW) //digitalRead(zeroSwitch) will read low when the switch is pressed
			{
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
			else if (currentStepPosition > StepR) 
			{
				dStep = currentStepPosition - StepR;
				if (dStep > dStepMax) 
				{
					dStep = dStepMax;
				}
				myMotor->setSpeed(10);
				myMotor->step(dStep, BACKWARD, SINGLE);
				currentStepPosition -= dStep;
			}

			if (currentStepPosition < 2) 
			{
				myMotor->release();
			}

			FlapPosCurrent = ((double)currentStepPosition) * (54.0 / 76.0);
			if (digitalRead(zeroSwitch) == LOW)  //NEW, dbl check this, as it may put program into phase 5 too early
			{
				phase = 5;
			}

		}
		//---------------END FLAP CLOSING--------------------------------------------

		//---------------END *POST-APOGEE SECTION*----------------------------------

		programTotIter = programTotIter + 1;

	} //keep all the flap actuation crap inside this bracket


}

void phase5() //NEW, ALL OF PHASE 5
{ 
	currentTime = millis();
	if (abs(currentTime - lastTime) >= 100) //100 ms time loop = 10 Hz operation
	{
		currentTimeSec += ((double)abs(currentTime - lastTime)) / 1000;
		lastTime = currentTime;
		lastTimeSec = currentTimeSec;
		complementary(0.1);
		myMotor->release();

		if (barAlt >= 50.0) 
		{
			if (digitalRead(zeroSwitch) == LOW) //digitalRead(zeroSwitch) will read low when the switch is pressed
			{
				currentStepPosition = 0;
			}
			writeToCard();
		}
		else 
		{
			phase == 6;
		}
	}
}


void sanityCheck()
{
	//----------------------CDAU AND CDAL SANITY CHECK----------------------//
	//Redefines the limits of the Cd at full deployment and no deployment based on the calculated Cd
	//The lower limit of CdAL = 0.06 is maintained

	if (CdA > CdAU)
	{
		CdAU = CdA;
	}
	if (CdA < CdAL)
	{
		CdAL = CdA;
		if (CdAL <= 0)
		{
			CdAL = 0.06;
		}
	}

	//Redefines the search limits of the bisection function based on the new limits of the Cd as found above
	if (CdAL < CdL) && (CdL <= 0)
	{
		
		CdL = 0.02;
		CdLSTATIC = CdL;
		
	}
	elseif (CdAL < CdL)
	{
		CdL = CdAL - 0.02;
		CdLSTATIC = CdL;
	}
	else
	{
		CdU = CdAU + 10;
		CdUSTATIC = CdU;
	}
	//-------------------END SANITY CHECK-------------------//
}

void setup(void) 
{
	Serial.begin(9600);

	pinMode(LED_BUILTIN, OUTPUT);

	//  /*----------------------SD CARD WRITING SETUP--------------------*/
	Serial.println("starting writing to SD card");
	pinMode(53, OUTPUT);
	
	pinMode(chipSelect, OUTPUT);
	
	if (!SD.begin(chipSelect)) 
	{
		Serial.println("Card failed, or not present");
		delay(50);
		// don't do anything more:
		return;
	}
	Serial.println("card initialized.");

	for (uint8_t i = 0; i < 100; i++) 
	{
		filename[6] = i / 10 + '0';
		filename[7] = i % 10 + '0';
		if (!SD.exists(filename))  // only opens a new file if it doesn't exist
		{
			Serial.println(filename);
			dataFile = SD.open(filename, FILE_WRITE);
			dataFile.println("Header (written in setup)");
			break;
		}
	}
	//
	Serial.println("SD card file initialized");
	//
	//  /*-------------------BAROMETER SENSOR SETUP--------------------------------*/
	if (pressure.begin()) 
	{
		Serial.println("BMP180 init success");
		delay(50);
	}
	else
	{
		// Oops, something went wrong, this is usually a connection problem,
		// see the comments at the top of this sketch for the proper connections.
	Serial.println("BMP180 init fail (disconnected?)\n\n");
	}
	//finds the 0 AGL pressure
	Serial.println("getting baseline pressure");
	for (int i = 1; i <= 10; i++) //gathers 10 samples of accelerationd data to find the initial bias
	{
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
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		delay(50);
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
		if (systemCal >= 3 && gyroCal >= 3 && accelCal >= 3 && magCal >= 3) 
		{
			calibrated = true;
			//      //      digitalWrite(LED_BUILTIN, HIGH);//sets the led in pin 13 to HIGH to signal that the system is calibrated
		}
		delay(100);
	} while (system1 < 3);
	
	//
	//
	//
	//  //END CALIBRATION
	//
	//  //finds the initial accelerometer bias
	for (int i = 1; i <= 10; i++) //gathers 10 samples of accelerationd data to find the initial bias
	{
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

	while (digitalRead(zeroSwitch) != LOW) //digitalRead(zeroSwitch) will read LOW when the switch is pressed
	{
		myMotor->step(2, BACKWARD, SINGLE);
	}
	currentStepPosition = 0;

	myMotor->release();//ENSURES THAT STEPPER IS TURNED OFF SO THAT IT CAN'T WASTE POWER

	/*------------------------------BEGIN TIME KEEPING--------------------------*/

	lastTime = millis();//gets the first last reading before we enter the main loop
	lastTimeSec = ((double)lastTime) / 1000;//converts to a second
	dataFile.close();
}

void complementary(double timePeriod) 
{


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
	if (filteredVelocity == 0.0 || abs(altDelta) <= sensitivity) //sets the new accelerometer bias to help reduce drift
	{
		accelBias = accelBias + (accelZ / 10.0);
	}
}

void writeToCard() 
{
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
	if (dataFile) 
	{
		dataFile.println(dataString);
		//        dataFile.flush();
		//    Serial.println("Wrote state to card");
	}
	dataFile.close();
}
