//Active Drag Code, Coded by Taber Fisher and Salim Roustom for the Rocketry at Virginia Tech Design Team, Launch 6/26/18

//Include Files for SD Card and associated variables
#include <SPI.h>
#include <SD.h>
String fileName = "Data";

//Include Files for IMU and associated variables
#include <Wire.h>
#include <LSM6.h>
LSM6 imu; //Creates the IMU object
char report[80]; //Charater for the writing of IMU data
const double gConversionFactor = 0.224 * 0.001; // Allows for conversions between Gs and IMU units

//Libraries for Servo and associated variables -------------------------------------------------
#include <Servo.h>
Servo myServo; //Object which controls Servo
int sPos = 60; //Used to control the position of the servo


//Declaration of Constant Variables and State Vector  ------------------------------------------
const double g = 9.80665; //gravity acceleration, m/s2
const double L0 = -0.0065; //Temperature lapse rate, K/m
const double M = 28.9645; //molar mass of air, g/mol
const double R = 8.3144998*1000; //Universal gas constant, J/kmol
double stateVector[] = {0,0,0};
double constantVals[] = {0,0,0,0,0};

//Launch associated variables-------------------------------------------------------------------
const double launchThreshold = -4; //in Gs
const double burnoutThreshold = 0; //in Gs

void setup()
{
  //Arduino Setup
  Serial.begin(115200);
  
  //SD Card Setup -----------------------------------------------------------------------------
  const int chipSelect = 8; //This corresponds to the pin required for the SD card shield
  
  Serial.println("Initializing SD Card, Please Wait....");
  pinMode(chipSelect, OUTPUT);

  if (!SD.begin(chipSelect))
  {
    Serial.println("The SD Card Failed to Initialized, check to make sure that the SD card is in the slot and there is power");
    return;
  }

  Serial.println("The SD Card Initialized Successfully");
  
  //IMU Setup -------------------------------------------------------------------------------
  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  Serial.println("Successfully Initialized IMU");
  imu.enableDefault();

  //Servo Setup -------------------------------------------------------------------------------
  myServo.attach(10);  // attaches the servo on pin 10 to the servo object
  myServo.write(60);
  myServo.write(65); // Keeps the servo at the minimum actuation in the rocket without pushing on the inside

  File testFile = SD.open(fileName, FILE_WRITE);
  testFile.println("This is the top of the Data File ------------------------------------------------");
  printStateVector();
  testFile.close();

  constSetup();
  
  //End Setup ----------------------------------------------------------------------------------
}

void loop() 
{
  printString("Main Loop Started");

  detectLaunch();

  unsigned long launchTime = millis();
  
  detectBurnout(launchTime);

  //Applies the hard coded first correction until the IMU calculates the expected apogee
  myServo.write(85);

}

//This method reads the IMY and prints it to a file
void readIMU()
{
    imu.read();

    snprintf(report, sizeof(report), "{%6d, %6d, %6d, %6d, %6d, %6d}", imu.a.x, imu.a.y, imu.a.z, imu.g.x, imu.g.y, imu.g.z);
    
    String requiredString = "";

    File testFile = SD.open(fileName, FILE_WRITE);
    
    digitalWrite(LED_BUILTIN, HIGH); 
    if (testFile)
    {
      unsigned long timeStamp = millis();

      testFile.print(timeStamp);
      testFile.print( ": ");
      testFile.print( report );

      testFile.println(); //create a new row to read data more clearly
      testFile.close();   //close file
    }
}

//This Method allows me to print strings on one line
void printString(String toWrite)
{
    File testFile = SD.open(fileName, FILE_WRITE);
    imu.read();
    
    digitalWrite(LED_BUILTIN, HIGH); 
    if (testFile)
    {
      testFile.print(toWrite);
      
      testFile.println();
      testFile.close();   //close file
    }
}

//This method allows me to print integers in the SD card
void printString(int toWrite)
{
    File testFile = SD.open(fileName, FILE_WRITE);
    imu.read();

    String writable = String(toWrite);
    
    digitalWrite(LED_BUILTIN, HIGH); 
    if (testFile)
    {
      testFile.print(writable);
      
      testFile.println();
      testFile.close();   //close file
    }
}

//This method allows me to print doubles in the SD card
void printString(double toWrite)
{
    File testFile = SD.open(fileName, FILE_WRITE);
    imu.read();

    String writable = String(toWrite, 4);
    
    digitalWrite(LED_BUILTIN, HIGH); 
    if (testFile)
    {
      testFile.print(writable);
      
      testFile.println();
      testFile.close();   //close file
    }
}

//This function prints the state vector to the SD card
void printStateVector()
{
  for(int i = 0; i < sizeof(stateVector); i++)
  {
    printString(stateVector[i]);
  }
}

//This method sets up the state vector and the constants necessary for the launch
void constSetup()
{
  double newState[] = {1, 2, 3};
  double newConst[] = {1, 2, 3, 4, 5};

  for (int i = 0; i < sizeof(newState); i++)
  {
      stateVector[i] = newState[i];
  }
  
  for (int i = 0; i < sizeof(constantVals); i++)
  {
      constantVals[i] = newConst[i];
  }

  
  printString("The state vector and necessary constants have been setup");
}

//This method detects the launch of the rocket through a threshold value and reads data while waiting
void detectLaunch()
{
  imu.read();

  Serial.println(imu.a.z);

  //Flipped so that the condition is false until the rocket is launched
  while (imu.a.z > (launchThreshold / gConversionFactor))
  {
      imu.read();
      readIMU();
      printString("The rocket is on pad");
  }
}

//This method dectects burnout of the rocket motor either through a threshold acceleration or through a time period
void detectBurnout(double launchTime)
{
  imu.read();

  unsigned long burnoutCheck = 0;

  //Flipped so that is false until the condition is met
  while ((imu.a.z < (burnoutThreshold / gConversionFactor)) && (burnoutCheck < 4621))
  {
    imu.read();
    readIMU();
    burnoutCheck = millis() - launchTime;
  }

  if (imu.a.z > (burnoutThreshold / gConversionFactor))
  {
    printString("Burnout Detected due to Acceleration Change");
  }
  else
  {
    printString("Burnout Detected due to Timout");
  }
}

//This function retracts fins and stops writing to SD card
void retractFins()
{
  printString("Retracting Fins. End of Data File ----------------------------------------------");
  myServo.write(65);
}

//This function determines when the rocket has reached apogee
bool detectApogee()
{
  imu.read();

  if (imu.a.z >= 1)
  {
    printString("The rocket has reached Apogee!");
    retractFins();
    return true;
  }
  else
  {
    printString("detectApogee was called and the rocket has not reached Apogee yet");
    return false;
  }
}

//This function predicts the expeced apogee of the rocket and the CD necessary to actuate the ADS to
double prediction()
{
  double sPred;
  printString("The CD needs to be change to");
  if((sPred > 60) && (sPred < 110))
  {
    myServo.write(sPred);
  }
  return(sPred);
}


