
// Motor links und rechts
struMotorDriver *p_leftMd = getMotorDriver();
struMotorDriver *p_rightMd = getMotorDriver();

// Ablaufsteuerung in der Loop
int motionCommand = 1;

int program = -1;

void inkrMotionCommand() {
  

      //motionCommand = 0;


  int fo=0;
  while(1){
    fo++;
//    spl(fo);
    if(fo>200){
      
      go=true;
      motionCommand = ++motionCommandValue;
      break;
    }
  }
}  
void rotier(float winkl,bool gelb){
if(!gelb){
  winkl=-winkl;
}
rotate(p_leftMd, p_rightMd, winkl);
}

void rotier(){

}


// void serielle_statemachine(int status, int strecke,bool& start){
//   static unsigned long waitTimeFlag=0;
//   static bool waitTimeActive = false;
//   static int waitTime=0;
//   if(start){
//     start=false;
//       Serial.println("jap");
//   switch(status){
//     break;
//      case 1:
//       driveLine(p_leftMd, p_rightMd, strecke);
//       motionCommand = 0;
//       break;
//          case 2:
//       rotier(strecke,false);
//       motionCommand = 0;
//       break;
//          case 3:
//       rotier(-strecke,false);
//       motionCommand = 0;
//       break;
//          case 4:
//       driveLine(p_leftMd, p_rightMd, -strecke);
//       motionCommand = 0;
//       break;
//       default:
  
//       waitTimeFlag = millis();
//       waitTimeActive = true;
//       waitTime = 1000;
//       motionCommand = 0;
//   }
//   }
// }


void notfall(bool gelb) {
  static unsigned long waitTimeFlag;
  static bool waitTimeActive = false;
  static int waitTime;

  switch (motionCommand) {
    case 0:                                 // Tue nichts
    break;
   case 1:
    spl("1n");
      driveLine(p_leftMd, p_rightMd, NOTFALLBOGENLAENGE*0.25);
      motionCommand = 0;
    break;
    
    case 2:
    spl("2n");
      rotier(90,gelb);
      motionCommand = 0;
    break;
    case 3:
    spl("3n");
      driveLine(p_leftMd, p_rightMd, NOTFALLBOGENLAENGE*0.95);
      motionCommand = 0;
    break;
    case 4:
    spl("4n");
      rotier(-90,gelb);
      motionCommand = 0;
      break;
      case 5:
      spl("5n");
      driveLine(p_leftMd, p_rightMd, NOTFALLBOGENLAENGE*0.5);
      motionCommand = 0;
      break;
    
    case 6:
    spl("6n");
      rotier(-90,gelb);
            motionCommand = 0;
      break;
    case 7:
    spl("7n");
      driveLine(p_leftMd, p_rightMd, NOTFALLBOGENLAENGE*0.45);
      motionCommand = 0;
      break;
    case 8:
      driveLine(p_leftMd, p_rightMd, -NOTFALLBOGENLAENGE*0.45);
      motionCommand = 0;
      break;
      break;
    case 9:
    spl("9n");
   rotier(90,gelb);
                motionCommand = 0;
      break;
   case 10:
   spl("10");
    driveLine(p_leftMd, p_rightMd, NOTFALLBOGENLAENGE*0.38);
                motionCommand = 0;
      break;
      case 11:
      spl("11");
      rotier(-90,gelb);
      motionCommand = 0;
      break;
      case 12:
      driveLine(p_leftMd, p_rightMd, NOTFALLBOGENLAENGE*0.75);
      motionCommand = 0;
      break;
      case 13:
      rotier(-78,gelb);
      motionCommand = 0;
      break;
        case 14:
    spl("9n");
      driveLine(p_leftMd, p_rightMd, NOTFALLBOGENLAENGE*0.85);
      motionCommand = 0;
      break;
        case 15:
      driveLine(p_leftMd, p_rightMd, -NOTFALLBOGENLAENGE*0.25);
      motionCommand = 0;
      break;
        case 16:
      rotier(180,gelb);
      motionCommand = 0;
      break;
        case 17:
      rotier(-180,gelb);
      motionCommand = 0;
      break;
          case 18:
      rotier(180,gelb);
      motionCommand = 0;
      break;
      case 19:
          rotier(-180,gelb);
      motionCommand = 0;
      break;  
      case 20:
       driveLine(p_leftMd, p_rightMd, NOTFALLBOGENLAENGE*0.25);
       motionCommand=0;
break;      
                                                               
  }
  if ((waitTimeActive) && (millis() - waitTimeFlag >= waitTime)) { // Warte Zeit
    inkrMotionCommand();
    waitTimeActive = false;
  }
}

void Lila() {
  static unsigned long waitTimeFlag;
  static bool waitTimeActive = false;
  static int waitTime;

  switch (motionCommand) {
    case 0:                                 // Tue nichts
    break;
    case 1:
      driveLine(p_leftMd, p_rightMd, -6);
      motionCommand = 0;
    break;
    case 2:
      waitTimeFlag = millis();
      waitTimeActive = true;
      waitTime = 1000;
      motionCommand = 0;
    case 3:
      driveLine(p_leftMd, p_rightMd, 45*0.56);
      motionCommand = 0;
    break;
    case 4:
      waitTimeFlag = millis();
      waitTimeActive = true;
      waitTime = 1000;
      motionCommand = 0;
    break;
    case 5:
      rotate(p_leftMd, p_rightMd, -95*0.83);      //statt-92
      motionCommand = 0;
    break;
    case 6:
      waitTimeFlag = millis();
      waitTimeActive = true;
      waitTime = 1000;
      motionCommand = 0;
    break;
    case 7:
      driveLine(p_leftMd, p_rightMd, 97*0.56); //statt95
      motionCommand = 0;
    break;
    case 8:
      waitTimeFlag = millis();
      waitTimeActive = true;
      waitTime = 1000;
      motionCommand = 0;
    break;
    case 9:
      rotate(p_leftMd, p_rightMd, -35*0.95); //statt-35
      motionCommand = 0;
    break;
    case 10:
      waitTimeFlag = millis();
      waitTimeActive = true;
      waitTime = 1000;
      motionCommand = 0;
    break;
    case 11:
      driveLine(p_leftMd, p_rightMd, 70*0.56);    //Ziel Erste Aufgabe
      motionCommand = 0;
    break;
    case 12:
    spl("mom");

      while(!motorenprozedur1()){}             //++++++++++++++++++++++++Delay Aktion1++++++++++++++++++++++++++++++++++
      spl("durch");
      //motionCommand = 0;
    break;    
    case 13:
      driveLine(p_leftMd, p_rightMd, -20*0.56); 
      motionCommand = 0;
    break;
    case 14:
      waitTimeFlag = millis();
      waitTimeActive = true;
      waitTime = 1000;
      motionCommand = 0;
    break;
    case 15:
      rotate(p_leftMd, p_rightMd, -130*0.87);
      motionCommand = 0;
    break;
    case 16:
      waitTimeFlag = millis();
      waitTimeActive = true;
      waitTime = 1000;
      motionCommand = 0;
    break;
    case 17:
      //driveLine(p_leftMd, p_rightMd, 240*0.56);     //zweite Aufgabe
      driveLine(p_leftMd, p_rightMd, 160*0.56);
      motionCommand = 0;
    break;                                                     
  }
  if ((waitTimeActive) && (millis() - waitTimeFlag >= waitTime)) { // Warte Zeit
    inkrMotionCommand();
    waitTimeActive = false;
  }
}

void EasyPoints() {
  static unsigned long waitTimeFlag;
  static bool waitTimeActive = false;
  static int waitTime;

  switch (motionCommand) {
    case 0:                                 // Tue nichts
    break;
    case 1:
      driveLine(p_leftMd, p_rightMd, 30);
      motionCommand = 0;
    break;
    case 2:
      waitTimeFlag = millis();
      waitTimeActive = true;
      waitTime = 10000;
      motionCommand = 0;
    case 3:
      driveLine(p_leftMd, p_rightMd, -33);
      motionCommand = 0;
    break;
    case 4:
      digitalWrite(LEFT_ENABLEPIN, LOW); // Schaltet die Motoren ab
      digitalWrite(RIGHT_ENABLEPIN, LOW);
      motionCommand = 0;
    break;                                                                 
  }
  if ((waitTimeActive) && (millis() - waitTimeFlag >= waitTime)) { // Warte Zeit
    inkrMotionCommand();
    waitTimeActive = false;
  }
}
