//Servo Libraries
#include <Servo.h>

//SD card libraries
#include <SPI.h>
#include <SD.h>

const int chipSelect = 8;

#include <Wire.h>
#include <LSM6.h>

LSM6 imu;

char report[80];

Servo myservo;  // create servo object to control a servo

int pos = 60;

//These variables are for reading data as long as it is less than 1 second
unsigned long currentTime = millis();
unsigned long previousTime = currentTime;

unsigned long launchTime;

//SD File name
String fileName = "TestYay";

//This converts IMU data to Gs for reading and comparing
double conversionFactor = 0.224 * 0.001;

//These values are the the threshold g's for launch and burnout
double launchThreshold = -1.25;
double burnoutThreshold = .2; 

void setup() 
{
  Serial.begin(115200);
  
  //Servo Setup
  myservo.attach(10);  // attaches the servo on pin 9 to the servo object

  //SD Card Setup -----------------------------------------------------
  Serial.println("Initializing SD Card, Please Wait....");
  pinMode(chipSelect, OUTPUT);

  if (!SD.begin(chipSelect))
  {
    Serial.println("Something Messed Up!!");
    return;
  }

  Serial.println("You be good");

  Wire.begin();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
}

void loop() 
{
    myservo.write(60);
    myservo.write(65);

    printString("Before Launch");
    
    detectLaunch();
    printString("Launch");
    
    launchTime = millis();
    detectBurnout(launchTime);
    printString("Burnout");
    
    actuateServo();
    printString("Actuated and Waiting Forever");

    waitForever();
}


void readIMU()
{
    imu.read();

    snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
    imu.a.x, imu.a.y, imu.a.z,
    imu.g.x, imu.g.y, imu.g.z);
    
    String requiredString = "";

    File testFile = SD.open(fileName, FILE_WRITE);
    
    digitalWrite(LED_BUILTIN, HIGH); 
    if (testFile)
    {
      unsigned long timeStamp = millis();

      testFile.print(timeStamp);
      testFile.print( " ms  ");
      testFile.print( report );

      testFile.println(); //create a new row to read data more clearly
      testFile.close();   //close file
    }
}

void readDelay(int timeGap)
{
    previousTime = millis();
    currentTime = millis();
    while ((currentTime - previousTime) < timeGap)
    {
        readIMU();
        currentTime = millis();
    }
}

void actuateServo()
{
  for (int i = 60; i <= 110; i+=5)
  {
    myservo.write(i);
    readDelay(500);
    
  }

  for (int i = 110; i >= 60; i-=5)
  {
    myservo.write(i);
    readDelay(500);
  }

  for (int i = 60; i <= 65; i+=5)
  {
    myservo.write(i);
    readDelay(500);
    
  }
}

void detectLaunch()
{
  imu.read();

  Serial.println(imu.a.z);

  //Flipped so that is false until the condition is met
  while (imu.a.z > (launchThreshold / conversionFactor))
  {
      imu.read();
  }
}

void detectBurnout(double launchTime)
{
  imu.read();

  unsigned long burnoutCheck = 0;

  //Flipped so that is false until the condition is met
  while ((imu.a.z < (burnoutThreshold / conversionFactor)) && (burnoutCheck < 7000))
  {
    imu.read();
    burnoutCheck = millis() - launchTime;
  }
}

void waitForever()
{
  while (1 == 1)
  {
  }
}

void printString(String toWrite)
{
    File testFile = SD.open(fileName, FILE_WRITE);
    imu.read();
    
    digitalWrite(LED_BUILTIN, HIGH); 
    if (testFile)
    {
      testFile.print(toWrite + " " + imu.a.z);
      
      testFile.println();
      testFile.close();   //close file
    }
}


