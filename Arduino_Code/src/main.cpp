#include <Arduino.h>
#include <Stepper.h>
#include <Wire.h> // Wire Bibliothek einbinden
#include <LiquidCrystal_I2C.h> // Vorher hinzugefügte LiquidCrystal_I2C Bibliothek einbinden
#include "stepperControlCombined9Pizzakurzcopsaf.h"
#include "0_Structs.h"
#include "Timer.h"
#include "MotorDriver.h"
#include "PWM_Functions.h"
#include "DriveSystem.h"
#include "Y_DistSensors.h"
#include "funcs_fuer_Statue.h"
#include "X_Programs.h"
#include "klassn.h"
#include "Ser_proz.h"
#include "ServoTimer2.h"
#define ENDSCHALT 13
//Serv Servo2(7,START2);
void stepset(Schrittmot& step1){
  step1.schritte=50;//nachher wieder 300
while(digitalRead(ENDSCHALT) && step1.schritte>0){
  step1.schritter();
}
step1.schritte=-50;
while(step1.schritter());
}
void setup() {
  
  Serial.begin(9600);
  Serial.println("beginn");
  for(int i=0;i<NUMBEROFSENORS+2;i++){
    pinMode(A0+i,INPUT);
  }
  //LiquidCrystal_I2C lcd(0x27, 20, 4); 
  pinMode(7,OUTPUT);
  Schrittmot Step1(SPR,SPEED);
  Serv Servo1(7,START1);
  pinMode(PUMPENPIN,OUTPUT);

    pinMode(A3,INPUT);
  digitalWrite(PUMPENPIN,LOW);
  pinMode(ENDSCHALT,INPUT);
  //lcd.init();
  //lcd.backlight(); 
//  Serial.begin(9600);status
//  Serial.println("Start");
//  Serial.println("Start");
//  Serial.println("Start");
  
  //aktorenini();  
  pinMode(LEFT_DIRPIN, OUTPUT);
  pinMode(LEFT_PULSEPIN, OUTPUT);
  pinMode(LEFT_ENABLEPIN, OUTPUT);
  pinMode(RIGHT_DIRPIN, OUTPUT);
  pinMode(RIGHT_PULSEPIN, OUTPUT);
  pinMode(RIGHT_ENABLEPIN, OUTPUT);

  // Setup-Pins mit internem Pullup
  //pinMode(PROGRAM_PIN_0, INPUT_PULLUP);
  //pinMode(PROGRAM_PIN_1, INPUT_PULLUP);
  
  // Kommunikations-Pins mit internem Pullup
//  pinMode(COMM_PIN_OUT, OUTPUT);
//  pinMode(COMM_PIN_IN, INPUT_PULLUP);
  
  digitalWrite(LEFT_DIRPIN, LOW);
  digitalWrite(RIGHT_DIRPIN, LOW);

  digitalWrite(LEFT_ENABLEPIN, HIGH);
  digitalWrite(RIGHT_ENABLEPIN, HIGH);
  pinMode(STPEN,OUTPUT);
  pinMode(STPDIR,OUTPUT);
  // pinMode(STP3,OUTPUT);
  // pinMode(STP4,OUTPUT);
  //pinMode(A2,OUTPUT);
  // Abstandssensoren
  for(int i=0; i<NUMBEROFSENORS; i++) {
    OldValue[i] = 0;
  }

  initMotorDriverHardware(p_leftMd, LEFT_DIRPIN, LEFT_PULSEPIN, LEFT_ENABLEPIN, LEFT_FALSE_DIRECTION, UNIT, LEFT_CIRCUMFERENCE, LEFT_TICKSPERROTATION);
  initMotorDriverSoftware(p_leftMd, getChannel(), LEFT_MIN_ANGULARVELOCITY, LEFT_MIN_ANGULARACCELERATION, LEFT_MAX_ANGULARVELOCITY, LEFT_MAX_ANGULARACCELERATION, LEFT_VELOCITY_CONTROLINTERVALL, LEFT_DUTY_CYCLE);
  p_leftMd->whichTemp = 0; // Identifizierung
  setAngVelo(p_leftMd, LEFT_ACTUAL_ANGULARVELOCITY);
  setAngAccel(p_leftMd, LEFT_ACTUAL_ANGULARACCELERATION);
  initChannel(p_leftMd->ch, p_leftMd->highTime, p_leftMd->lowTime, timerControlRisingEdgeLeft, timerControlFallingEdgeLeft, p_leftMd, p_leftMd);

// Motor Driver Right
  initMotorDriverHardware(p_rightMd, RIGHT_DIRPIN, RIGHT_PULSEPIN, RIGHT_ENABLEPIN, RIGHT_FALSE_DIRECTION, UNIT, RIGHT_CIRCUMFERENCE, RIGHT_TICKSPERROTATION);
  initMotorDriverSoftware(p_rightMd, getChannel(), RIGHT_MIN_ANGULARVELOCITY, RIGHT_MIN_ANGULARACCELERATION, RIGHT_MAX_ANGULARVELOCITY, RIGHT_MAX_ANGULARACCELERATION, RIGHT_VELOCITY_CONTROLINTERVALL, RIGHT_DUTY_CYCLE);
  p_rightMd->whichTemp = 1;
  setAngVelo(p_rightMd, RIGHT_ACTUAL_ANGULARVELOCITY);
  setAngAccel(p_rightMd, RIGHT_ACTUAL_ANGULARACCELERATION);
  initChannel(p_rightMd->ch, p_rightMd->highTime, p_rightMd->lowTime, timerControlRisingEdgeRight, timerControlFallingEdgeRight, p_rightMd, p_rightMd);

  distToCenter = DISTANCE_TO_CENTER; // in cm -> In Zukunft Variable von DriveSystem

  timerSetup(); // Timer jetzt schon aktiv -> Unschön: Lieber erst aktivieren, sobald der erste Channel aktiv
long serles=0;
char buffers[2][6];
char derbuffer[18];
  memset(derbuffer,0,18);
  for(int i=0;i<2;i++){
  memset(buffers[i],0,6);
  }
int globalstatus=0;
int strecke=0;
int j=0;
int k=0;
bool availprev=false;
long uper=0;
int status=0;
bool start=false;
int zut;
int dur=0;
long t_akt=0;
bool setd;
stepset(Step1);
while(1){
    if (p_leftMd->calcNextAngVelo) {
    calcInterruptTime(p_leftMd);
  }
  if (p_rightMd->calcNextAngVelo) {
    calcInterruptTime(p_rightMd);
  }
          p_leftMd->earlyStopFlag = false;
        p_rightMd->earlyStopFlag = false;
        p_leftMd->earlyStopFinished = false;
        p_rightMd->earlyStopFinished = false;
    if(ser_prozed(serles,availprev,derbuffer,j,k,status,dur)){
      setd=false;
      start=true;
setState(Step1,Servo1,status,dur,start);
//erielle_statemachine(status,dur,start);
    }
Serial.println(sensorFeedback());    
 if(!Step1.schritter()){
  //Serial.print(schritte);Serial.print(":\t");Serial.print(z1temp);Serial.print("\t");Serial.println(z2temp);
  if (micros()-t_akt>20000){
    t_akt=micros();
  if(!Servo1.servo_akt()){
    //Servo2.servo_akt();
    if(status>4 && status!=interrupt){
      status=0;
      go=true;
    }
  }
 }
 }

    if (p_leftMd->calcNextAngVelo) {
    calcInterruptTime(p_leftMd);
  }
  if (p_rightMd->calcNextAngVelo) {
    calcInterruptTime(p_rightMd);
  }
  // Ereignis zwingt zum Stoppen
  if (false) {
    earlyStopNow(p_leftMd, p_rightMd);
    // Auslöser wieder auf false/deaktivieren etc.
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 100)
  {

    previousMillis = currentMillis;
    // Verarbeitung von sensorFeedback();
    /*switch (sensorFeedback()) {
      case 0:
      if ((p_leftMd->earlyStopFinished) && (p_rightMd->earlyStopFinished)) {
        p_leftMd->earlyStopFlag = false;
        p_rightMd->earlyStopFlag = false;
        p_leftMd->earlyStopFinished = false;
        p_rightMd->earlyStopFinished = false;
        //spl("RETURN_RETURN_RETURN_RETURN_RETURN");
        returnDriving(p_leftMd, p_rightMd);
      }
      break;
      case 1:
      if ((p_leftMd->ch->active) || (p_rightMd->ch->active)) {
        if ((p_leftMd->dirForward) && (p_rightMd->dirForward)) {
          earlyStopNow(p_leftMd, p_rightMd);
        }
      }
      break;
      case 2:
        if ((p_leftMd->ch->active) || (p_rightMd->ch->active)) {
          earlyStopNow(p_leftMd, p_rightMd);
        }
      break;
      case 3:
      if ((p_leftMd->ch->active) || (p_rightMd->ch->active)) {
        if (!(p_leftMd->dirForward) && !(p_rightMd->dirForward)) {
          earlyStopNow(p_leftMd, p_rightMd);
        }
      }
      break;
    }  */

  }
   
 
  if(go){
    go=false;
    if(status!=interrupt){
    Serial.println("1");
    }
    status=0;
    
  }
}
}




void loop() {


  //  switch (program) {
  //   case 0: // Homologation
  //     notfall(true);
  //   break;
  //   case 1: // Links zuerst hinfahren
  //     notfall(true);
  //   break;
  //   case 2: // Rechts zuerst hinfahren
  //     notfall(false);
  //   break;
  //   case 3:
  //     notfall(false);
  //   break;
  // }
}