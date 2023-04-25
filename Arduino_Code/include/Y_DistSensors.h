//functions
int checkCollision(); //returns value from enum Colisionstate
bool checkThreshold(int Value, int ThresholdLow, int ThresholdHigh, int PreviousResult); 
int readValue(int Sensor); //TODO: write this function
int sensorFeedback();

//defines
#define NUMBEROFSENORS 6
// the first sensor on the backside, the onse bevore are in the front (counting starts with 0)
#define START_BACKSIDE 3
// TODO: set some good values
// TODO: and have fun while finding them
#define UPPERTHRESHOLD 420
#define LOWERTHRESHOLD 360

enum Collisionstate {collision_none, collision_front, collision_everywhere, collision_back};

//global vars
static int OldValue[NUMBEROFSENORS];
unsigned long previousMillis = 0;



int sensorFeedback(){
  int tempSensor;
  for(int i=0; i<NUMBEROFSENORS; i++)
  {
     int Value = readValue(i);
     //bool Output = checkThreshold(Value, LOWERTHRESHOLD, UPPERTHRESHOLD, OldValue[i]);

     if(OldValue[i] != Value)
     {
       tempSensor = checkCollision();
       // TODO: Output from Schmitttriger has changed, do something with it. Maybe trigger some callbacks or something... what ever you want
     }

     for(int i=0; i<NUMBEROFSENORS; i++)
     {
       OldValue[i] = Value;
     }
  }
  return tempSensor;
}
  
int checkCollision()
{
  bool Output[NUMBEROFSENORS];
  for(int i=0; i<NUMBEROFSENORS; i++)
  {
     int Value = readValue(i);
     // Serial.print("Value");
     // Serial.println(Value);
     
     Output[i] = checkThreshold(Value, LOWERTHRESHOLD, UPPERTHRESHOLD, OldValue[i]); 
     // Serial.print(i);
     // Serial.println(Output[i]);   
  }
  // set the new old values  
  for(int i=0; i<NUMBEROFSENORS; i++)
  {
    OldValue[i] = Output[i];
  }
  
  // boolianmagic inc. 
  bool front = false, back = false;
  for(int i = 0; i<START_BACKSIDE; i++)
  {
    front |= Output[i];    
  }
  for(int i = START_BACKSIDE; i<NUMBEROFSENORS; i++)
  {
    back |= Output[i];
  }

  //Return the right result
  if(front&&back)
  {
    return collision_everywhere;
  }
  if(front)
  {
    return collision_front;
  }
  if(back)
  {
    return collision_back;
  }
  return collision_none;
}

int readValue(int Sensor)
{
  if(Sensor>3){
    Sensor+=2;
  }
  return analogRead(14+Sensor);
  
  // if(Sensor > 3 ){
  //   return analogRead(16+Sensor);
  // }else{
  //   return analogRead(14+Sensor);
  // }
}

//schmitttrigger
bool checkThreshold(int Value, int ThresholdLow, int ThresholdHigh, int PreviousResult)
{
  if(PreviousResult)
  {
    if (Value < ThresholdLow)
    {
      return false;
    }
    else
    {
      return true;  
    }
  }
  else //Old Value is false
  {
    if (Value > ThresholdHigh)
    {
      return true;
    }
    else
    {
      return false;  
    }    
  }
}
