#include <Wire.h>
#include <Zumo32U4.h>
#include <ME480FSM.h>

//Zumo

Zumo32U4Encoders encoders;
Zumo32U4ButtonC buttonC;

//Project Inputs

unsigned long inputTime;
unsigned long inputVoltage;

//FSM Declaration

FSMTimer timer1(2000);
FSMTimer timer2(inputTime);

/*
 * List of FSM Booleans
 */

//Inputs

boolean btnC;
boolean tOneExceeded;
boolean tTwoExceeded;
boolean tThreeExceeded;

//State Transitions

boolean rdy_wrn;
boolean wrn_rnn;
boolean rnn_flt;
boolean flt_rdy;

boolean rdy_rdy;
boolean wrn_wrn;
boolean rnn_rnn;
boolean flt_flt;

//States

boolean rdy;
boolean wrn;
boolean rnn;
boolean flt;

boolean noState;

//Ouputs

PololuBuzzer buzzer;
unsigned int alarmOneFreq;
unsigned int alarmTwoFreq;
unsigned int alarmDuration;
unsigned char alarmVolume;
int16_t speedV;

//Misc

unsigned long previousMillis = 0;

void setup() {

  inputTime = 0;
  inputVoltage = 0;
  
  alarmOneFreq = 8000;
  alarmTwoFreq = 4000;
  alarmDuration = 1000;
  alarmVolume = 10;
  Serial.begin(115200);

  inputVoltageRatio = inputVoltage/5;
  speedV = -400 + (inputVoltageRatio*800);
  
}

void loop() {
  unsigned long currentMillis = millis();
  noState = !(rdy||wrn||rnn||flt);
  //Block 1
  
  btnC= buttonC.isPressed();
  tOneExceeded=timer1.TMR;
  tTwoExceeded=timer2.TMR;
  tThreeExceeded=timer3.TMR;
  
  //Block 2

  rdy_wrn=rdy&&btnC;
  wrn_rnn=wrn&&tOneExceeded;
  rnn_flt=rnn&&tTwoExceeded;
  flt_rdy=flt&&!(buzzer.isPlaying());

  rdy_rdy = rdy&&(!rdy_wrn)||noState;
  wrn_wrn = wrn&&!(wrn_rnn);
  rnn_rnn = rnn&&!(rnn_flt);
  flt_flt = flt&&!(flt_rdy);
  
  //Block 3
  
  rdy = (flt_rdy||rdy_rdy);
  wrn = (rdy_wrn||wrn_wrn);
  rnn = (wrn_rnn||rnn_rnn);
  flt = (rnn_flt||flt_flt);
  
  //Block 4

  ledYellow(rnn);
  
  timer1.update(wrn);
  timer2.update(rnn);
  timer3.update(flt);

  if(rdY_wrn){
    buzzer.playFrequency(alarmOneFreq, alarmDuration, alarmVolume);
  }

  if(wrn_rdy){
    setSpeeds(speedV,speedV);
  }
  
  if(rnn_flt){
    buzzer.playFrequency(alarmTwoFreq, alarmDuration, alarmVolume);
    setSpeeds(0,0);
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (ledState == false) {
      ledState = true;
    } else {
      ledState = false
    }
  ledYellow(ledState);
  
}
