//Active Drag Code, Coded by Taber Fisher and Salim Roustom for the Rocketry at Virginia Tech Design Team, Launch 6/26/18

//Include Files for SD Card and associated variables
#include <SPI.h>
#include <SD.h>
String fileName = "Quest.txt";

//Include Files for IMU and associated variables
#include <Wire.h>
#include <LSM6.h>
#include <LPS.h>
LSM6 imu; //Creates the IMU object
LPS ps;
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
const double R = 8.3144998 * 1000; //Universal gas constant, J/kmol

//Launch associated variables-------------------------------------------------------------------
const double launchThreshold = -4; //in Gs
const double burnoutThreshold = 0; //in Gs

//Variables Associated with ODE ----------------------------------------------------------------
//Global Variables
const int n = 4; // number of state variables in the ODE
double t = 0; // current time
int steps = 0;

// Constants
const double gamma = 1.4;
const double Rbar = R / M;

//Flap CD
double flapPercentCD;

//Launch Day Input Variables
const double pGROUND = 29.8 * 3386.39; //Ground pressure (convert from in Hg to Pa)
const double TGROUND = 310.928; // ground temperature, K (313.706 K = 105 F, 310.928 K = 100 F)
const double rhoGROUND = pGROUND / Rbar / TGROUND;

//Drag Information
const double Cdbody = 0.39; // rocket profile coefficient of drag at M = 0.3
const double CdFlaps = 1.28; // Flat plate estimate for flaps' coefficient of drag

//Intermediate Variables
const double onePLUSgMoverRL0 = 1 + (g*M / R / L0); // The 1000 is needed to get the numerator in kg so that kgm2/s2 will cancel out with J
const double negativeoneover2m0 = -1 / 25.207000000000000 / 2;

//Initial Conditions
const double x0 = 0; // initial horizontal position meters
const double V0 = 248.93; // burnout velocity, m/s
const double z0 = 1191.8; // burnout altitude (ALG), m
const double theta0 = 1.412302977421291; // burout zenith angle, rad

double Y_prev[n] = {x0, z0, theta0, V0}; // Initial state of the ODE
double Y_current[n] = {39.3743811851986, 1433.08267455424, 1.40553400310464, 229.313640927776}; // one timestep of the ODE

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
  ps.enableDefault();

  //Servo Setup -------------------------------------------------------------------------------
  myServo.attach(10);  // attaches the servo on pin 10 to the servo object
  myServo.write(60);
  myServo.write(65); // Keeps the servo at the minimum actuation in the rocket without pushing on the inside

  File testFile = SD.open(fileName, FILE_WRITE);
  testFile.println("This is the top of the Data File ------------------------------------------------");
  testFile.close();

  //End Setup ----------------------------------------------------------------------------------
}

void loop()
{
  printString("Main Loop Started");

  detectLaunch();

  unsigned long launchTime = millis();

  detectBurnout(launchTime);

  //Applies the hard coded first correction until the IMU calculates the expected apogee
  myServo.write(110);

  while(!detectApogee())
  {
    myServo.write(prediction());
  }

}

//This method reads the IMU and prints it to a file
void readIMU()
{
  imu.read();

  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);
  float temperature = ps.readTemperatureC();

  //Prints to SD card Acceleration in x, y, z, then rotation in x, y, z, then pressure, altitude, temperature
  snprintf(report, sizeof(report), "%6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d", imu.a.x, imu.a.y, imu.a.z, imu.g.x, imu.g.y, imu.g.z, pressure, altitude, temperature);

  String requiredString = "";

  File testFile = SD.open(fileName, FILE_WRITE);

  digitalWrite(LED_BUILTIN, HIGH);
  if (testFile)
  {
    unsigned long timeStamp = millis();

    testFile.print(timeStamp);
    testFile.print( ", ");
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
void printInt(int toWrite)
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
void printDouble(double toWrite)
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
  while ((imu.a.z < (burnoutThreshold / gConversionFactor)) && (burnoutCheck < 6750))
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

  double finalResults[5];
  for (int i = 0; i <= 4; i ++)
  {
    double p = i * .25;
    finalResults[i] = odeSolver(1, p);
  }

  double minDiff = 10000;
  int minId = 0;

  for (int i = 1; i < 4; i++)
  {
    if ( abs(finalResults[i] - 10000) < minDiff)
    {
      minId = i;
    }
  }


  double startV = (minId * .25) - .1;
  double specificResults[5];
  for (int i = 0; i <= 4; i ++)
  {
    double currP = startV + (1 * i);
    specificResults[i] = odeSolver(.1, currP);
  }

  minDiff = 10000;
  minId = 0;

  for ( int i = 1; i < 4; i++)
  {
    if ( abs(specificResults[i] - 3048) < minDiff)
    {
      minId = i;
    }
  }

  //Converts Flap Percentages to Servo actuations
  int sPred = (specificResults[minId] * 50) + 60;

  if ((sPred > 60) && (sPred < 110))
  {
    myServo.write(sPred);
  }

  printString("The CD needs to be change to");
  printInt(sPred);

  return (sPred);
}

double odeSolver(double dt, double flapPercent)
{
  for(int i = 0; i < 4; i++)
  {
    Y_prev[i] = Y_current[i];
  }

  complementaryFilter();
  
  double Y_temp[n]; // temp array for the ODE function output.
  double K1[n], K2[n], K3[n], K4[n];

  // calculate another timestep: t0 --> t1
  // K1 = Y_p(t0, Y_current)
  ODE(t, Y_current, K1);

  // K2 = Y_p(t0 + dt/2, Y_current + dt/2 * K1)
  for (int i = 0; i < n; i++) {
    Y_temp[i] = Y_current[i] + K1[i] * dt * 0.5;
  }
  ODE(t + dt * 0.5, Y_temp, K2);

  // K3 = Y_p(t0 + dt/2, Y_current + dt/2 * K2)
  for (int i = 0; i < n; i++) {
    Y_temp[i] = Y_current[i] + K2[i] * dt * 0.5;
  }
  ODE(t + dt * 0.5, Y_temp, K3);

  // K4 = Y_p(t0 + dt, Y_current + dt * K3)
  for (int i = 0; i < n; i++) {
    Y_temp[i] = Y_current[i] + K3[i] * dt;
  }
  ODE(t + dt, Y_temp, K4);


  // Calculate new step:
  // Y_nextstep = Y_current + dt/6 * (K1 + 2*(K2+K3) + K4)
  for (int i = 0; i < n; i++) {
    Y_current[i] = Y_current[i] + dt / 6.0 * (K1[i] + 2.0 * (K2[i] + K3[i]) + K4[i]);
  }
  t = t + dt;

  return Y_current[1];
}

void ODE(double t, double Y[], double Y_p[])
{
  /*
    // simple 2-mass oscillator:
    // y = pos1, pos2, vel1, vel2
    Y_p[0]=Y[2];
    Y_p[1]=Y[3];
    Y_p[2]=-Y[0]*c1 + (Y[1]-Y[0])*c2 - Y[2]*d1;
    Y_p[3]=-(Y[1]-Y[0])*c2 - Y[3]*d2;*/

  //Using new variables
  Y_p[0] = Y[3] * cos(Y[2]);
  Y_p[1] = Y[3] * sin(Y[2]);
  Y_p[2] = -g * cos(Y[2]) / Y[3];
  Y_p[3] = (negativeoneover2m0 * (rhoGROUND * pow((1 - L0 * Y[1] / TGROUND), onePLUSgMoverRL0)) * pow(Y[3], 2)) * (((30.8451 * 0.00064516) * (0.445623145893995 + -0.290001570655917 * Y[3] / sqrt(R * gamma * (TGROUND + L0 * Y[1])) + 0.561689194232028 * pow(Y[3], 2) / R / gamma / (TGROUND + L0 * Y[1]))) + flapPercentCD) - (g * sin(Y[2]));
}


void complementaryFilter()
{
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);

  double baroV = getBaroV();
  double altV = getAltV();

  Y_current[3] = (.5 * getBaroV()) + (.5 * getAltV());
  Y_current[1] = altitude;

  String toPrint = "The new state vector is: ";
  toPrint.concat(Y_current[0]);
  toPrint.concat(Y_current[1]);
  toPrint.concat(Y_current[2]);
  toPrint.concat(Y_current[3]);
  
  printString(toPrint);
}

double getBaroV()
{
  long firstTime = millis();
  float pressure = ps.readPressureMillibars();
  float firstAlt = ps.pressureToAltitudeMeters(pressure);
  pressure = ps.readPressureMillibars();
  long dt = millis() - firstTime;
  float secondAlt = ps.pressureToAltitudeMeters(pressure);
  

  double firstX = Y_prev[0];
  double secondX = Y_current[0];
  long dtx = 12;

  return (sqrt( pow(((secondAlt - firstAlt) / dt / 1000), 2) + pow(((secondX - firstX) / dtx / 1000), 2)));
}

double getAltV()
{
  long firstTime = millis();
  imu.read();
  double firstXY = sqrt(pow(imu.a.x, 2) + pow(imu.a.y, 2));
  double firstZ = imu.a.z;
  long dt = millis() - firstTime;
  imu.read();
  double secondXY = sqrt(pow(imu.a.x, 2) + pow(imu.a.y, 2));
  double secondZ = imu.a.z;
  
  double vXY = 0.5 * dt * (firstXY + secondXY);
  double vZ = 0.5 * dt * (firstZ + secondZ);

  return (sqrt( pow(vXY, 2) + pow(vZ, 2)));  
}

