#pragma once
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