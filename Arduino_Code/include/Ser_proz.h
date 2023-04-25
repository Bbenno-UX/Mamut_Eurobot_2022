class LiquidCrystal_I2C;
class Servo1;
class Schrittmot;

bool ser_prozed(long& serles,bool& availprev,char* derbuffer,int& j,int& k,int& Status,int& dur){
      if (Serial.available() && micros()-serles>4000){//Serial-buffer braucht 3 ms, um sich zu füllen, hab ich gehört
  char akt;
  availprev=true;
  serles=micros();
    akt=Serial.read();
    if(akt==',' || (akt>=48 && akt<=57)){
      derbuffer[k]=akt;
      k++;
    }

      }
      else if(Serial.available()==0 && availprev && micros()-serles>4000){
      k=0;
      j=0;
      serles=micros();
  availprev=false;
      sscanf(derbuffer,"%i,%i",&Status,&dur);
  memset(derbuffer,0,18);
  return true;
  //Serial.print(z1);Serial.print("\t");Serial.print(z2);Serial.print("\t");Serial.println(schritte);
  }
  return false;
}
enum state{unok_0,vorwaerts,rot_rechts,rot_links,rueckwaerts,schritt_hoch,schritt_runter,Servo,PUMPE,interrupt,ballmotor_raus,ballmotor_rein};

void setState(Schrittmot& Step1, Serv& Servo1,int Status, int& strecke,bool& start){
  static int prevstate=0;
  static unsigned long waitTimeFlag=0;
  static bool waitTimeActive = false;
  static int waitTime=0;
  switch(Status){
    break;
     case vorwaerts:
      driveLine(p_leftMd, p_rightMd, strecke);
      motionCommand = 0;
      break;
         case rot_rechts:
      rotier(strecke,false);
      motionCommand = 0;
      break;
         case rot_links:
      rotier(-strecke,false);
      motionCommand = 0;
      break;
         case rueckwaerts:
      driveLine(p_leftMd, p_rightMd, -strecke);
      motionCommand = 0;
      break;
        case schritt_hoch:
        digitalWrite(Step1.dirpin,HIGH);
        Step1.schritte=strecke;
        break;
        case schritt_runter:
        digitalWrite(Step1.dirpin,LOW);
        Step1.schritte=-strecke;
        break;
        case Servo:
        Servo1.z_soll=strecke;
        break;
        case PUMPE:
        digitalWrite(PUMPENPIN,strecke);
        break;
        case interrupt:
            unsigned int stps=(p_leftMd->nomCount-p_leftMd->ch->count)*p_leftMd->circumference/p_leftMd->ticksPerRotation;
            if(stps>0){
            p_leftMd->ch->count=p_leftMd->nomCount-1;
            p_rightMd->ch->count=p_rightMd->nomCount-1;
            }
            timerControlFallingEdge(p_leftMd);
            timerControlFallingEdge(p_rightMd);
            
            Serial.print(prevstate);Serial.print(",");Serial.println(stps);
            break;
        case ballmotor_rein://fuck, wie mach ich das mit dem motor?
              digitalWrite(DC_MOTOR_PIN,HIGH);
              Serial.println("1");
        break;
        case ballmotor_raus:
            digitalWrite(DC_MOTOR_PIN,LOW);
            Serial.println("1");
        break;
        default:
          waitTimeFlag = millis();
      waitTimeActive = true;
      waitTime = 1000;
      motionCommand = 0;
        strecke=strecke;

    }
        if(Status!=0){
        prevstate=Status;
        }
}


void lcd_ausgabe(LiquidCrystal_I2C& lcd,int status,int dur){
  lcd.setCursor(0, 0);//Hier wird die Position des ersten Zeichens festgelegt. In diesem Fall bedeutet (0,0) das erste Zeichen in der ersten Zeile. 
lcd.print("Befehl:"); 
lcd.setCursor(0, 1);// In diesem Fall bedeutet (0,1) das erste Zeichen in der zweiten Zeile. 
lcd.print(status); 
lcd.setCursor(0, 2);// In diesem Fall bedeutet (0,1) das erste Zeichen in der zweiten Zeile. 
lcd.print(dur); 
}
