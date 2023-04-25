
#include "ServoTimer2.h"
class Schrittmot{
  public:
    Schrittmot(int spr, int speed,int dirp=STPDIR,int enp=STPEN){
        usecs=60000000/(spr*speed);
        dirpin=dirp;
        enpin=enp;
        //myStepper=new Stepper(spr, STP1, STP2, STP3, STP4);
        schritte=0;
        stepkram=0;
    }
    
    void sprak(int st){
   
          switch (st) {
      case 0:  // 1010
        digitalWrite(enpin,HIGH);
      break;
      case 1:  // 0110
      digitalWrite(enpin,LOW);
    }
    }
    bool schritter(){//funktion für schrittmotoren adafruit benutzt dummerweise "delay" bei der step funktion, darum nur 1 schritt auf einmal
  if(abs(schritte)>0){
    static long k=8000000;
    //Serial.println(schritte);
  if(micros()-stepkram>usecs){
    stepkram=micros();
    if(schritte>0){
      digitalWrite(dirpin,LOW);
    if(k%2==0){
    schritte--;
    }
    sprak(k%2);
    k++;
    }
       if(schritte<0){
        digitalWrite(dirpin,HIGH);
    if(k%2==0){
    schritte++;
    }
    
    sprak(k%2);
    k--;
  }
  }
  return true; 
  }
  return false;
}
    int dirpin;
    int enpin;
    int schritte;
    long usecs;
    static long aktschritt;
    unsigned long stepkram;
    //Stepper* myStepper;
};

class Serv{
    public:
    Serv(int PIN, int z_bas){
      pinMode(PIN,OUTPUT);
        t=0;
        del=0;
        aktiv=false;
        z_soll=z_bas;
        pin=PIN;
        Servo2.attach(PIN);
        
    }
    void turner(){
if(aktiv && micros()-t>20000-ztemp){
  t=micros();
  digitalWrite(pin,HIGH);
  aktiv=false;
  
}
else if (!aktiv && micros()-t>ztemp){
  t=micros();
  digitalWrite(pin,LOW);
  aktiv=true;
}
}
bool servo_akt(){
  if(abs(ztemp-z_soll)>2){
    //Serial.println("was");
  ztemp=ztemp+(z_soll-ztemp)*pow(sin(del*2*CONST_PI/150),2);
  Servo2.write(ztemp);
  del++;
  //Serial.print(pin);Serial.print(":\t");Serial.print(ztemp);Serial.print("\t");Serial.println(z);
  return true;
}
del=0;
return false;
}
    bool aktiv;
    int z_soll;
    int ztemp;
    int del;
    long t;
    int pin;
    ServoTimer2 Servo2;
};

// Serv Servo1(6,START1);
// //Serv Servo2(7,START2);
// Schrittmot Step1(SPR,SPEED);