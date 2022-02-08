#include <Wire.h>
#include <Zumo32U4.h>
#include <ME480FSM.h>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonC buttonC;
Zumo32U4LCD lcd;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Encoders encoder1;
Zumo32U4IMU imu;

FSMTimer timer1(3000);
FSMTimer timer2(750);
FSMTimer timer3(2600);
FSMTimer timer4(1000);

/*
 * FSM Variables
 */
//Inputs
boolean btnC;
boolean tOneExceeded;
boolean tTwoExceeded;
boolean tThreeExceeded;
boolean tFourExceeded;
//States
boolean sfe;
boolean wrn;
boolean lne;
boolean rot;
boolean stt;
boolean pst;
//State Transitions
boolean noState;
boolean sfe_wrn;
boolean sfe_sfe;
boolean wrn_sfe;
boolean wrn_lne;
boolean wrn_wrn;
boolean lne_stt;
boolean lne_lne;
boolean lne_pst;
boolean rot_lne;
boolean rot_rot;
boolean rot_stt;
boolean stt_stt;
boolean stt_rot;
boolean pst_pst;

/*
 * Universal Variables
 */

float Ksum=1;
float U;
float T1;
float T2;
float DT;
float convert = 1000000.0;
float maxVoltage;

/*
 * PD on Lateral Position
 */

float Kp_LP = 229;
float Kd_LP = 77;
float U_LP;
float derivativeError_LP;

/*
 * PI on Forward Velocity
 */
float maxVelocity = 53.80; 
float Kp_FV = 0.118;
float Ki_FV = 7.8;
float counts_R;
float counts_L;
float rightVel;
float leftVel;
float goalVel = 14;
float error_FVR;
float error_FVL;
float intError_FVR;
float intError_FVL;
float integralError;
float U_FVR;
float U_FVL;

/*
 * Line Following
 */
float lineerror_m; 
float U_lneR;
float U_lneL;

/*
 * Proximity
 */
float proxCounts;
float arrGoalPosition[4] = {0,0.18,0.34,0.18};
float arrGoalAngPos[4] = {-0.5,-2,90,0};
float arrayPosSize = 4;
float currentPosition_R;
float currentPosition_L;

//Angular Controller Subvariables
float currAngPos;
float currDPS;
float goalAngPos;

float error_ang;
float derivativeError_ang;

float U_PD_ang;
//Angular
float Ksum_ang = 1;
float Kp_d_ang = 0.9456;
float Kd_ang =  0.0111;

boolean finalPositionExceeded;
float imuDrift;
float IMU_convert = 70/1000.0;
float radius = 0.019;


float goalPosition;
float error_R;
float error_L;
float derivativeError_R;
float derivativeError_L;
float U_PD_R;
float U_PD_L;

float Kp_d = 107.4022;
float Kd = 4.7733;

//Misc
int j = 0;
int i = 0;
int k = 0;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS] = {0,0,0,0,0};
int proxDist = 0;

const byte addressDevice   = 0x6B;  //hexadecimal device address for your IMU chip
const byte addressRegister = 0x23;  //hexadecimal register address for dps sensitivity
const byte code245dpsFS    = 0x00;  //hexadecimal code for 245dps full scale
const byte code500dpsFS    = 0x10;  //hexadecimal code for 500dps full scale
const byte code2000dpsFS   = 0x20;  //hexadecimal code for 2000dps full scale

void calibrateLineSensors()
{
  ledYellow(1);
  lcd.clear();
  lcd.print(F("Line cal"));

  for (uint16_t i = 0; i < 400; i++)
  {
    lcd.gotoXY(0, 1);
    lcd.print(i);
    lineSensors.calibrate();
  }

  ledYellow(0);
  lcd.clear();
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();

  lcd.clear();
  lcd.print(F("Press A"));
  lcd.gotoXY(0, 1);
  lcd.print(F("to calib"));
  buttonA.waitForButton();
  calibrateLineSensors();

  Serial.print("Voltage: ");
  maxVoltage = readBatteryMillivolts()/1000;
  Serial.println(maxVoltage);

  if (!imu.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to initialize IMU sensors."));
      delay(100);
    }
  }

  imu.enableDefault();
  imu.configureForTurnSensing(); 
  imu.writeReg(addressDevice, addressRegister, code2000dpsFS);
  /*
  for (int i; i < 1000; i++) {
    imu.read();
    imuDrift = imu.g.z + imuDrift;
  }
  imuDrift = imuDrift / 1000;
  */
}

void loop()
{

  
  if(lne){
    proxSensors.read();
    proxCounts = proxSensors.countsFrontWithLeftLeds();
  }
  /*
  Serial.print(lineerror_m,4);
  Serial.print("\t");
  Serial.println(proxCounts);
  Serial.println();
  */
  T1 = T2;
  T2 = micros()/convert; 
  DT = T2-T1;
  
  //Block 1
  noState = !(sfe||wrn||lne||stt||rot||pst);
  btnC = buttonC.isPressed();
  tOneExceeded = timer1.TMR;
  tTwoExceeded = timer2.TMR;
  tThreeExceeded=timer3.TMR;
  tFourExceeded=timer4.TMR;

  //Block 2
  sfe_wrn = sfe&&btnC;
  sfe_sfe = sfe&&!btnC;
  //wrn_sfe = wrn&&!btnC&&!tOneExceeded;
  wrn_wrn = wrn&&!tOneExceeded;
  wrn_lne = wrn&&tOneExceeded;
  lne_lne = lne&&!lne_stt;
  lne_stt = lne&&proxCounts==6;
  stt_rot = stt&&tThreeExceeded;
  rot_stt = rot&&tTwoExceeded&&j!=3;
  rot_rot = rot&&!tTwoExceeded;
  stt_stt = stt&&!tThreeExceeded;
  rot_lne = rot&&tTwoExceeded&&j==3;
  
  //Block 3
  sfe = noState||sfe_sfe;
  wrn = sfe_wrn||wrn_wrn;
  lne = wrn_lne||lne_lne||rot_lne;
  rot = stt_rot||rot_rot;
  stt = lne_stt||rot_stt||stt_stt;
  
  //Block 4
  
  if((lne&&(!wrn_lne))||(stt&&j==0)){
    lineSensors.readCalibrated(lineSensorValues);
    int error = readLineAAB(lineSensorValues);
    lineerror_m = error*0.0127/1000;
    lineerror_m;
    derivativeError_LP = (lineerror_m-derivativeError_LP)/DT;
    derivativeError_LP = lineerror_m;
    U_LP = Ksum*(Kp_LP*lineerror_m+Kd_LP*derivativeError_LP);
    U_LP = U_LP*400/maxVoltage;
    counts_R = encoder1.getCountsAndResetRight()/617.4*2*PI;
    counts_L = encoder1.getCountsAndResetLeft()/617.4*2*PI;
    rightVel = counts_R/DT;
    leftVel = counts_L/DT;
    error_FVR = goalVel - rightVel;
    error_FVL = goalVel - leftVel;
    intError_FVR = error_FVR*DT+intError_FVR;

    intError_FVL = error_FVL*DT+intError_FVL;
    U_FVR = Ksum*(Kp_FV*error_FVR+Ki_FV*intError_FVR);
    U_FVL = Ksum*(Kp_FV*error_FVL+Ki_FV*intError_FVL);
    U_FVR = U_FVR*400/maxVelocity;
    U_FVL = U_FVL*400/maxVelocity;
    //U_lneR = 100+(U_LP);
    //U_lneL = 100-(U_LP);
    U_lneR = U_FVR+U_LP;
    U_lneL = U_FVR-U_LP;

    if(!stt){
       motors.setSpeeds(U_lneL,U_lneR);
    }
  }

  if(lne_stt||rot_stt||stt_rot||rot_lne){
    encoder1.getCountsAndResetRight()/617.4*2*PI;
    encoder1.getCountsAndResetLeft()/617.4*2*PI; 
    motors.setSpeeds(0,0);
    U_PD_ang = 0;
    currDPS = 0;
    currAngPos;
    derivativeError_ang = 0;
    error_ang = 0;
    currentPosition_R = 0;
    currentPosition_L = 0;
    derivativeError_R = 0;
    derivativeError_L = 0;
    derivativeError_LP = 0;
    lineerror_m = 0;
    error_FVR = 0;
    intError_FVR = 0;
    error_R = 0;
    error_L = 0;

    U_PD_R = 0;
    U_PD_L = 0;
  }

  if(lne_stt){
    U_lneL = 0;
    U_lneR = 0;
    U_FVR = 0;
    U_FVL = 0;
    U_LP = 0;
  }
  if(rot_lne){
    j = 0;
  }
  
  if(rot&&!(stt_rot)){
    imu.read(); 
 
    if(i==0){
      DT = 0;
      i++;
    }
    
    currDPS = (imu.g.z)*IMU_convert;
    currAngPos = currAngPos + currDPS * DT;
    goalAngPos = arrGoalAngPos[j];
    
    derivativeError_ang = error_ang;
    error_ang = currAngPos-goalAngPos;
    derivativeError_ang = (error_ang - derivativeError_ang)/DT;
    
    U_PD_ang = Ksum_ang*(Kp_d_ang*error_ang+Kd_ang*derivativeError_ang);
    U_PD_ang = U_PD_ang*400/maxVoltage;
    if(U_PD_ang>200){
      U_PD_ang=200;
    }
    /*
    Serial.print("Current: ");
    Serial.print(currAngPos);
    Serial.print("Goal: ");
    Serial.print(goalAngPos);
    Serial.print("Rot U: ");
    Serial.println(U_PD_ang);
    */
    motors.setSpeeds(U_PD_ang,-U_PD_ang);
    
  }

  if(stt&&!(rot_stt)){
    if(j!=0){
        counts_R = encoder1.getCountsAndResetRight()/617.4*2*PI;
        counts_L = encoder1.getCountsAndResetLeft()/617.4*2*PI; 
    }

    currentPosition_R = currentPosition_R + counts_R*radius;
    currentPosition_L = currentPosition_L + counts_L*radius;
    goalPosition = arrGoalPosition[j];
    
    derivativeError_R = error_R;
    derivativeError_L = error_L;
        
    error_R = goalPosition-currentPosition_R;
    error_L = goalPosition-currentPosition_L;
    
    derivativeError_R = (error_R - derivativeError_R)/DT;
    derivativeError_L = (error_L - derivativeError_R)/DT;
    
    U_PD_R = Ksum*(Kp_d*error_R+Kd*derivativeError_R);
    U_PD_R = U_PD_R * 400/maxVoltage;
    
    U_PD_L = Ksum*(Kp_d*error_L+Kd*derivativeError_L);
    U_PD_L = U_PD_L * 400/maxVoltage;
    
    if(U_PD_R>100){
      U_PD_R=100;
    }
    if(U_PD_L>100){
      U_PD_L=100;
    }
    if(j==0){
      U_PD_L = U_PD_L - U_LP;
      U_PD_R = U_PD_R + U_LP;  
    }
    
    motors.setSpeeds(U_PD_L,U_PD_R);
    /*
    Serial.print(goalPosition);
    Serial.print("\t");
    Serial.println(currentPosition_R);
    */
    /*
    Serial.print("Current: ");
    Serial.print(currentPosition_R);
    Serial.print("goal Position");
    Serial.println(goalPosition);
    */
  }

  if(rot_stt){
    j++;
  }

  //Timers

  timer1.update(wrn);
  timer2.update(rot);
  timer3.update(stt);
  
  // FSM Comments
  
  /*
  Serial.print("Safe: ");
  Serial.print(sfe);
  Serial.print("\t");
  Serial.print("Warn: ");
  Serial.print(wrn);
  Serial.print("\t");
  Serial.print("Line: ");
  Serial.print(lne);
  Serial.print("\t");
  Serial.print("Rot: ");
  Serial.print(rot);
  Serial.print("\t");
  Serial.print("Straight: ");
  Serial.print(stt);
  Serial.print(" j: ");
  Serial.println(j);
  */

  
  
  /*
  Serial.print("Counts: ");
  Serial.print(counts_R);
  Serial.print(" Vel: ");
  Serial.print(rightVel);
  Serial.print(" Goal: ");
  Serial.print(goalVel);
  Serial.print(" Error: ");
  Serial.print(error_FVR);
  Serial.print(" Integral Error: ");
  Serial.print(intError_FVR);
  Serial.print(" U: ");
  Serial.print(U_FVR);
  */
  /*
  if(stt&&j==0){
    Serial.print("Error: ");
    Serial.print(lineerror_m,5);
    Serial.print(" U: ");
    Serial.println(U_LP);
    Serial.print(" Right: ");
    Serial.print(U_FVR);
    Serial.print(U_PD_R);
  
  }
  */
  
  
  // U Values
  
  /*
  Serial.print(" Left: ");
  Serial.println(U_lneL); 
  */
  
  /*
  lcd.gotoXY(0, 0);
  lcd.println(U_LP,8);
  lcd.gotoXY(0, 1);
  lcd.println(lineerror_m,8);
  */
    Serial.print(T1);
    Serial.print("\t");
    Serial.print(currAngPos);
    Serial.print("\t");
    Serial.println(goalAngPos);

  

}


int readLineAAB(unsigned int vals[5]){
  int pos = vals[1]-vals[3];
  return pos;
}
