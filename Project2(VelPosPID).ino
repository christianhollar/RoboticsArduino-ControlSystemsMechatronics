//Imports
#include <Wire.h>
#include <ME480FSM.h>
#include <Zumo32U4.h>
#include <LiquidCrystal.h>

// Input Array
float arrGoalTime[2] = {2,2};
float arrGoalPosition[8] = {0.15,0.15,0.35,0.45,0.7,0.7,0.45,0.6};
float arrayPosSize = 8;
//float arrGoalPosition[1]={0.5};
float arrGoalVelocity[2]={2,2};
//float arrGoalAngPos[1]={142};
//float arrGoalAngPos[7] = {0,0,-95,-112,-180,112,95};
float arrGoalAngPos[7] = {0,0,-137,100,-180,-100,137};

float goalAngDPS;

//Inputs
boolean btnC;
boolean tOneExceeded;
boolean tTwoExceeded;
boolean tThreeExceeded;

// States

boolean rdy;
boolean wrn;
boolean stt;
boolean rot;
boolean pst;

// State Transitions

boolean noState;

boolean rdy_wrn;
boolean wrn_stt;
boolean stt_rot;
boolean rot_stt;
boolean rot_pst;

boolean rdy_rdy;
boolean wrn_wrn;
boolean stt_stt;
boolean rot_rot;
boolean pst_pst;

// Gains
float Kp_d = 190.9308;
float Kp_i = 0.0077;

float Kd = 4.7733;
float Ki = 0.3096;

float Ksum = 1.0;

//Controller Subvariables

float currentPosition_R;
float currentPosition_L;
float goalPosition;
float currentVelocity;
float goalVelocity;

float error_R;
float error_L;
float error_DPS;
float error_PI;
float derivativeError_R;
float derivativeError_L;
float integralError;
float integralError_DPS;

boolean errorPositionExceeded;
boolean errorAngleExceeded;
boolean finalPositionExceeded;

float U_PD_R;
float U_PD_L;
float U_PI;
float U_PI_DPS;

float motorSpeed;
float currentVoltage;
float maxVoltage;

//Data Acquisition

float T1;
float T2;
float counts_R;
float counts_L;
float DT;
float convert=1000000.0;
float radius = 0.019;

//FSM Declaration
FSMTimer timer1(3000);
FSMTimer timer2(2000);
FSMTimer timer3(2000);
FSMTimer timer4(1000);
//Zumo Declaration
Zumo32U4IMU imu;
Zumo32U4ButtonC buttonC;
Zumo32U4Encoders encoder1;
Zumo32U4Motors motor;

//Misc
int j = 0;
int i = 0;
//Inputs

//Angular Controller Subvariables
float currAngPos;
float currDPS;
float goalAngPos;

float error_ang;
float derivativeError_ang;

float U_PD_ang;
float Ksum_ang = 1;
float Kp_d_ang = 0.3096;
float Kd_ang = 0.0077;

//Angular Data Acquisition
float T1_ang;
float T2_ang;
float DT_ang;
float IMU_convert = 70/1000.0;
float imuDrift;

//
const byte addressDevice   = 0x6B;  //hexadecimal device address for your IMU chip
const byte addressRegister = 0x23;  //hexadecimal register address for dps sensitivity
const byte code245dpsFS    = 0x00;  //hexadecimal code for 245dps full scale
const byte code500dpsFS    = 0x10;  //hexadecimal code for 500dps full scale
const byte code2000dpsFS   = 0x20;  //hexadecimal code for 2000dps full scale

void setup() {
  Wire.begin();
  Serial.begin(115200);
    
  /* 
  for(int i = 0; i<sizeof(arrGoalTime); i++){
    arrGoalVelocity[i]=arrGoalVelocity[i]/arrGoalPosition[i];
  }
  */

  // try to initialize the imu and report an error if initialization fails
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
  
  for (int i; i < 1000; i++) {
    imu.read();
    imuDrift = imu.g.z + imuDrift;
  }
  imuDrift = imuDrift / 1000;
}

void loop() {
  unsigned long currentMillis = millis();  
  noState = !(rdy||wrn||stt||rot||pst);
  
  //Block 1
  
  btnC= buttonC.isPressed();
  tOneExceeded=timer1.TMR;
  tTwoExceeded=timer2.TMR;
  tThreeExceeded=timer3.TMR;

  //Block 2

  rdy_wrn = rdy&&btnC;
  wrn_stt = wrn&&tOneExceeded;
  stt_rot = stt&&tTwoExceeded;
  rot_stt = rot&&tThreeExceeded;
  rot_pst = stt&&finalPositionExceeded;

  rdy_rdy = rdy&&(!rdy_wrn);
  wrn_wrn = wrn&&(!wrn_stt);
  stt_stt = stt&&(!stt_rot);
  rot_rot = rot&&(!rot_stt)&&(!rot_pst);
  pst_pst = pst;
  
  //Block 3
  
  rdy = rdy_rdy||noState;
  wrn = rdy_wrn||wrn_wrn;
  stt = wrn_stt||rot_stt||stt_stt;
  rot = stt_rot||rot_rot;
  pst = rot_pst||pst_pst;
  
  //Block 4
  if(rdy_wrn){   
    Serial.print("Voltage: ");
    maxVoltage = readBatteryMillivolts()/1000;
    Serial.println(maxVoltage);
  }
  
  if(stt&&(!rot_stt)){
    counts_R = encoder1.getCountsAndResetRight()/617.4*2*PI;
    counts_L = encoder1.getCountsAndResetLeft()/617.4*2*PI;
    T1=micros()/(convert);
    DT=(T1-T2);
    T2=T1;
    
    currDPS = (imu.g.z-imuDrift)*IMU_convert;
    goalAngDPS = 0;

    error_DPS = currAngPos-goalAngPos;
    integralError_DPS = (error_DPS*DT) + integralError;    
    U_PI_DPS = Ksum*(Kp_d_ang*error_ang+Kd_ang*integralError);
    U_PI_DPS = U_PD_ang*400/maxVoltage;
    
    /*
     * PD Controller
     */
     
    
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
    
    if(U_PD_R>200){
      U_PD_R=200;
    }
    if(U_PD_L>200){
      U_PD_L=200;
    }

    U_PD_R = U_PD_R-U_PI_DPS;
    U_PD_L = U_PD_L+U_PI_DPS;
    if(j!=arrayPosSize-1){
      motor.setSpeeds(U_PD_L,U_PD_R);
    }
    
    if(j==(arrayPosSize-1)){
      while(!timer4.TMR){
        motor.setSpeeds(245,245);
        timer4.update(true);
      }
      motor.setSpeeds(0,0);
    }
    
    
    /*
     * PI Controller
     */
    /*
    currentVelocity = counts_R/DT;
    goalVelocity = arrGoalVelocity[j];

    error_PI = goalVelocity - currentVelocity;
    integralError = (error_PI*DT) + integralError;
    
    U_PI = Ksum*(Kp_i*error_PI+Ki*integralError);
    motor.setSpeeds(U_PI,U_PI);
    */
    /*
    Serial.print("PI");
    Serial.print("\t");
    Serial.print(U_PI);
    Serial.print("\t");
    Serial.print("PD");
    Serial.print("\t");
    Serial.print(U_PD);
    U = U_PD;
    */

    //motor.setSpeeds(U_PD_L,U_PD_R);    
   /* 
    Serial.print(" Current ");
    Serial.print(currentPosition);
    Serial.print(" Goal ");
    Serial.print(goalPosition);
    Serial.print(" U: ");
    Serial.print(U_PD);
    Serial.print(" Ksum: ");
    Serial.print(Ksum);
    Serial.print(" Kpd: ");
    Serial.print(Kp_d);
    Serial.print("Kd");
    Serial.print(Kd);
    Serial.print("ROT");
    Serial.println(rot);
   */ 
  }
  
  if(stt_rot){
    /*
    Serial.println("StraightRotateStart");
    Serial.print("Counts R");
    Serial.print(counts_R);
    Serial.print("Counts L");
    Serial.print(counts_L);
    Serial.print(" CurrentR ");
    Serial.print(currentPosition_R);
    Serial.print(" CurrentL ");
    Serial.print(currentPosition_L);    
    Serial.print(" Goal ");
    Serial.print(goalPosition);
    Serial.print(" UR: ");
    Serial.print(U_PD_R);
    Serial.print(" UL: ");
    Serial.println(U_PD_L);
    */
    motor.setSpeeds(0,0);
    currentPosition_R = 0;
    currentPosition_L = 0;
    goalPosition = 0;
    error_R = 0;
    error_L = 0;
    derivativeError_R = 0;
    derivativeError_L = 0;
    U_PD_R = 0;
    U_PD_L = 0;
    imu.read();
    goalAngPos = arrGoalAngPos[j];
    encoder1.getCountsAndResetLeft();
    encoder1.getCountsAndResetRight();
    motor.setSpeeds(0,0);
    Serial.println("StraightRotateEnd");

  }
  
  if(rot&&(!stt_rot)){
    if(j==arrayPosSize-1){
      while(1){}
      finalPositionExceeded=true;
    }
    imu.read(); 
    T1_ang=micros()/(convert);
    DT_ang=(T1_ang-T2_ang);
    T2_ang=T1_ang;
    
    if(i==0){
      DT_ang = 0;
      i++;
    }
    
    currDPS = (imu.g.z-imuDrift)*IMU_convert;
    currAngPos = currAngPos + currDPS * DT_ang;
    goalAngPos = arrGoalAngPos[j];
    
    derivativeError_ang = error_ang;
    error_ang = currAngPos-goalAngPos;
    derivativeError_ang = (error_ang - derivativeError_ang)/DT_ang;
    
    U_PD_ang = Ksum_ang*(Kp_d_ang*error_ang+Kd_ang*derivativeError_ang);
    U_PD_ang = U_PD_ang*400/maxVoltage;

    motor.setSpeeds(U_PD_ang,-U_PD_ang);

    /*
    Serial.print(" T1: ");
    Serial.print(T1_ang);
    Serial.print(" T2: ");
    Serial.print(T2_ang);
    Serial.print(" DT_ang: ");
    Serial.print(DT_ang);
    Serial.print(" Current: ");
    Serial.print(currAngPos);
    Serial.print(" Goal: ");
    Serial.print(goalAngPos);
    Serial.print(" U: ");
    Serial.println(U_PD_ang);
    */
  }
  
  if(rot_stt){
    /*
    Serial.println("RotateStraightStart");
    Serial.print("Counts R");
    Serial.print(counts_R);
    Serial.print("Counts L");
    Serial.print(counts_L);
    Serial.print(" CurrentR ");
    Serial.print(currentPosition_R);
    Serial.print(" CurrentL ");
    Serial.print(currentPosition_L);    
    Serial.print(" Goal ");
    Serial.print(goalPosition);
    Serial.print(" UR: ");
    Serial.print(U_PD_R);
    Serial.print(" UL: ");
    Serial.println(U_PD_L);
    */
    currAngPos = 0;
    goalAngPos = 0;
    error_ang = 0;
    derivativeError_ang = 0;
    U_PD_ang = 0;
    j++;
    encoder1.getCountsAndResetLeft();
    encoder1.getCountsAndResetRight();
    motor.setSpeeds(0,0);
    Serial.println("RotateStraightEnd");

  }
  if(pst){
    //setVoltage(0);
  }
  
  
  timer1.update(wrn);
  timer2.update(stt);
  timer3.update(rot);
  timer4.update(j==arrayPosSize-1);
  if(!pst){
    /*
    Serial.print(" Ready: ");
    Serial.print(rdy);
    Serial.print(" Warning: ");
    Serial.print(wrn);
    Serial.print(" Straight: ");
    Serial.print(stt);
    Serial.print(" Rotate: ");
    Serial.print(rot);
    Serial.print(" Post: ");
    Serial.print(pst);
    Serial.print("STRAIGHT_ROTATE");
    Serial.print(stt_rot);
    Serial.print("ROTATE_STRAIGHT");
    Serial.println(rot_stt);
    */
  }
  
  /*
  Serial.print("J");
  Serial.println(j);
  Serial.print("Pos Size:");
  Serial.println(arrayPosSize);
  Serial.println(finalPositionExceeded);
  */
  
    Serial.print("Counts R");
    Serial.print(counts_R);
    Serial.print("Counts L");
    Serial.print(counts_L);
    Serial.print(" CurrentR ");
    Serial.print(currentPosition_R);
    Serial.print(" CurrentL ");
    Serial.print(currentPosition_L);    
    Serial.print(" Goal ");
    Serial.print(goalPosition);
    Serial.print(" UR: ");
    Serial.print(U_PD_R);
    Serial.print(" UL: ");
    Serial.println(U_PD_L);
    

   
}
