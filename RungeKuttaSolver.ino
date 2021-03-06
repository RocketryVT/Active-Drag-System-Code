const double dt=1;      // Timestep used by the algorithm
const int n=4;                // number of state variables in the ODE
int steps=0;                  // number of calculated steps
double t=0;                 // current time



double Y_temp[n];            // temp array for the ODE function output.
double K1[n] = {0, 0, 0, 0};
double K2[n] = {0, 0, 0, 0};
double K3[n] = {0, 0, 0, 0};
double K4[n] = {0, 0, 0, 0};   

//prev const
const double c1=1;
const double c2=1;
const double d1=0.01;
const double d2=0.01;

// Constants
const double g = 9.80665;
const double L0 = -0.0065;
const double M = 28.9645;
const double R = 8.3144998*1000;
const double gamma = 1.4; 
const double Rbar = R/M;
const double m0 = 25.207000000000000;

//Model Constants
const double a = 0.445623145893995;
const double b = -0.290001570655917;
const double c = 0.561689194232028;

//Launch Day Input Variables
const double pGROUND = 29.8*3386.39; //Ground pressure (convert from in Hg to Pa)
const double TGROUND = 310.928; // ground temperature, K (313.706 K = 105 F, 310.928 K = 100 F)
const double rhoGROUND = pGROUND/Rbar/TGROUND;

//Rocket and ADS Geometry
const double d = 6.274*0.0254; // rocket diameter (inches)
const double Sflaps = 0; // previous value: 7.656*0.00064516/2;
const double SCDflaps = 0; // ????
const double S = 30.8451*0.00064516; // rocket reference area, m2 (given by OpenRocket)

//Drag Information
const double Cdbody = 0.39; // rocket profile coefficient of drag at M = 0.3
const double Cdflaps = 1.28; // Flat plate estimate for flaps' coefficient of drag

//Intermediate Variables
const double onePLUSgMoverRL0 = 1+(g*M/R/L0); // The 1000 is needed to get the numerator in kg so that kgm2/s2 will cancel out with J
const double negativeoneover2m0 = -1/m0/2;

//Initial Conditions
const double x0 = 0; // initial horizontal position meters
const double V0 = 248.93; // burnout velocity, m/s
const double z0 = 1191.8; // burnout altitude (ALG), m
const double theta0 = 1.412302977421291; // burout zenith angle, rad

double Y_current[n]={x0, z0, theta0, V0};    // Initial state of the ODE

// Debugging parameters:
long time_lastprintout = millis();


void setup() {
  // initialize Serial to output the integrator results.
  Serial.begin(9600);
  Serial.println("Begun");
}

void loop() {
  // calculate another timestep: t0 --> t1

  // K1 = Y_p(t0, Y_current)
  ODE(t, Y_current, K1);
  
  // K2 = Y_p(t0 + dt/2, Y_current + dt/2 * K1)
  for (int i=0; i<n; i++) {
    Y_temp[i] = Y_current[i] + K1[i]*dt*0.5;
  }
  ODE(t + dt*0.5, Y_temp, K2);

  // K3 = Y_p(t0 + dt/2, Y_current + dt/2 * K2)
  for (int i=0; i<n; i++) {
    Y_temp[i] = Y_current[i] + K2[i]*dt*0.5;
  }
  ODE(t + dt*0.5, Y_temp, K3);

  // K4 = Y_p(t0 + dt, Y_current + dt * K3)
  for (int i=0; i<n; i++) {
    Y_temp[i] = Y_current[i] + K3[i]*dt;
  }
  ODE(t + dt, Y_temp, K4);


  // Calculate new step:
  // Y_nextstep = Y_current + dt/6 * (K1 + 2*(K2+K3) + K4)
  for (int i=0; i<n; i++) {
    Y_current[i] = Y_current[i] + dt / 6.0 * (K1[i] + 2.0*(K2[i] + K3[i]) + K4[i]);
  }
  t=t+dt;
  steps++;
  
  
  
  // Print output each X calculated steps:
  if (steps>=20) {
    Serial.print("state at ");
    Serial.print(t);
    Serial.print("s: ");
    for (int i=0; i<n; i++) {
      Serial.print(Y_current[i]);
      Serial.print(" ");
    }

    Serial.print(" calculation time: ");
    Serial.print(millis()-time_lastprintout);
    time_lastprintout=millis();
  
    Serial.println();
    
    steps=0;
  }
  
  
  
}

void ODE(double t, double Y[], double Y_p[]) {
  /*
  // simple 2-mass oscillator:
  // y = pos1, pos2, vel1, vel2
  Y_p[0]=Y[2];
  Y_p[1]=Y[3];
  Y_p[2]=-Y[0]*c1 + (Y[1]-Y[0])*c2 - Y[2]*d1;
  Y_p[3]=-(Y[1]-Y[0])*c2 - Y[3]*d2;*/

  //Using new variables
  Y_p[0]= Y[3]*cos(Y[2]);
  
  
  Y_p[1]= Y[3]*sin(Y[2]);
  
  
  Y_p[2]= -g*cos(Y[2])/Y[3];
  
  
  Y_p[3]= (negativeoneover2m0*(rhoGROUND*pow((1-L0*Y[1]/TGROUND),onePLUSgMoverRL0))*pow(Y[3],2))*((S*(a+b*Y[3]/sqrt(R*gamma*(TGROUND+L0*Y[1]))+c*pow(Y[3],2)/R/gamma/(TGROUND+L0*Y[1])))+SCDflaps)-(g*sin(Y[2]));
  
}



void printValues()
{
  for (int i = 0; i < 4; i++)
  {
    Serial.print(Y_current[i]);
    Serial.print(" ");
  }
  Serial.println();

  for (int i = 0; i < 4; i++)
  {
    Serial.print(K1[i]);  
    Serial.print(" ");
  }
  Serial.println();

  for (int i = 0; i < 4; i++)
  {
    Serial.print(K2[i]);
    Serial.print(" ");  
  }
  Serial.println();

  for (int i = 0; i < 4; i++)
  {
    Serial.print(K3[i]);  
    Serial.print(" ");
  }
  Serial.println();

  for (int i = 0; i < 4; i++)
  {
    Serial.print(K4[i]); 
    Serial.print(" "); 
  }
  Serial.println();
}

