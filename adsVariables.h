#pragma once
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

