//in nmaindatei oder so rein mit den definitionen
#define MAGNET 13 //sag mal bitte, welcher pin noch frei ist
#define PINM1 7
#define PINM2 6//in die steppercontroldatei
#define PINM3 3
#define PINM4 2
#define RICHTUNG1 HIGH //für den fall falscher verkabelung einfach umswitchen
#define RICHTUNG2 LOW
#define RICHTUNG3 HIGH
#define RICHTUNG4 LOW
long magntim;
bool timabgleich(int delta){
  if(millis()-magntim>delta){
    magntim=millis();
  return true;
  }
  return false; 
}
void aktorenini() {
//in setup
  // put your setup code here, to run once:
  //einfach in z_mainsetup klatschen

pinMode(MAGNET,OUTPUT);
digitalWrite(MAGNET,HIGH);
pinMode(PINM1,OUTPUT);
pinMode(PINM2,OUTPUT);
pinMode(PINM3,OUTPUT);
pinMode(PINM4,OUTPUT);

}
void forwerts1(){
    digitalWrite(PINM1,RICHTUNG1);
  digitalWrite(PINM2,RICHTUNG2);
}
void forwerts2(){
  digitalWrite(PINM3,RICHTUNG4);
  digitalWrite(PINM4,RICHTUNG3);  
}
void rugwerts1(){
  digitalWrite(PINM1,RICHTUNG2);
  digitalWrite(PINM2,RICHTUNG1);  
}
void rugwerts2(){
    digitalWrite(PINM3,RICHTUNG3);
  digitalWrite(PINM4,RICHTUNG4);
}
void schdob(){
  
  digitalWrite(PINM1,LOW);
  digitalWrite(PINM2,LOW);
  digitalWrite(PINM3,LOW);
  digitalWrite(PINM4,LOW);
}
void setupmagnet(){
  magntim=millis();
}
bool motorenprozedur1(){
//statue aufheben und so. 
  static int magncommand;
  switch(magncommand){
    case 0: 
    setupmagnet();
      forwerts1();
      magncommand++;
    break;
    case 1:
      if(timabgleich(4500)){
        schdob();
        magncommand++;
      }
    break;
    case 2:
       if(timabgleich(1000)){
        rugwerts1();
        magncommand++;
      }
    break;
    case 3:
       if(timabgleich(7800)){
        schdob();
        magncommand++;
      }    
    break;
    case 4:
       if(timabgleich(580)){
        forwerts2();
        magncommand++;
      }     
    break;
    case 5:
        if(timabgleich(500)){
        schdob();
        magncommand++;
      } 
    break;
    case 6:
        if(timabgleich(1000)){
        rugwerts2();
        magncommand++;
      } 
    break;
    case 7:
         if(timabgleich(500)){
        schdob();
        inkrMotionCommand();
         spl(motionCommandValue);
magncommand++;        
        return true;
    break;
    case 8:
    return true; 
      } 

      
    break;    
    
  }
  return false;
}

bool motorenprozedur2(){
//ablegen
  static int magncomm;
  switch(magncomm){
    case 0: 
      forwerts1();
      setupmagnet();
      magncomm++;
    break;
    case 1:
      if(timabgleich(4500)){
       schdob();
        
        magncomm++;
      }
    break;
    case 2:
       if(timabgleich(1000)){
        digitalWrite(MAGNET,LOW);
        magncomm++;
      }
    break;
        case 3:
       if(timabgleich(1000)){
        rugwerts1();
        magncomm++;
      }
    break;
        case 4:
       if(timabgleich(3000)){
        schdob();
        magncomm++;
      
       inkrMotionCommand();
      spl(motionCommandValue);
        return true;
    break;
    case 8:
    return true; 
      }
  } 
  return false; 
}

//loop mal raus, aber so rufst du die funktionen auf
/*void loop(){ 
  do{
    
  }while(!motorenprozedur1());
  do{
    
  }while(!motorenprozedur2());  


}*/
