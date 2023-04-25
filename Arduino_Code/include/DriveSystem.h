
struct driveSystem {
  int velocity;
  int acceleration;
  int angVelocity;
  int angAcceleration;

  int lineLength;
  int curveRadius;

  int radMeassureRotation;
  int xPosition;
  int yPosition;

  struct struMotorDriver *md[ANZ_MOTOR_DRIVER];
  int distToCenter[ANZ_MOTOR_DRIVER];
};
int distToCenter; // Unschön

void rotate(struct struMotorDriver *mdL, struct struMotorDriver *mdR, int degree) {
  if (degree >= 0) {
    mdL->dirForward = true;
    mdR->dirForward = false; //CW (Rechts gedreht)
  } else {
    mdL->dirForward = false; //CCW (Links gedreht)
    mdR->dirForward = true;
    degree = -degree;
  }  
  
  float tickRatioLeft = ((2.*(float)distToCenter*CONST_PI)/(float)mdL->circumference)*((float)degree/360.); // distToCenter noch vom Struct DriveSystem lesen
  float tickRatioRight = ((2.*(float)distToCenter*CONST_PI)/(float)mdL->circumference)*((float)degree/360.);
//  sp("tickRatioRotation");
//  spl(tickRatioLeft);
  drive(mdL, mdR, tickRatioLeft, tickRatioRight);
}

void driveLine(struct struMotorDriver *mdL, struct struMotorDriver *mdR, int distance) {
  if (distance >= 0) {
    mdL->dirForward = true;
    mdR->dirForward = true;
  } else {
    mdL->dirForward = false;
    mdR->dirForward = false;
    distance = -distance;
  }
  float tickRatioLeft = (float)distance/(float)mdL->circumference;
  float tickRatioRight = (float)distance/(float)mdL->circumference;
  drive(mdL, mdR, tickRatioLeft, tickRatioRight);
}
void drive(struct struMotorDriver *mdL, struct struMotorDriver *mdR, float tickRatioLeft, float tickRatioRight) {
  int mStepsL = tickRatioLeft*mdL->ticksPerRotation;
  int mStepsTillBrakeL = mStepsL-(((float)(mdL->nomAngVelo - mdL->actAngVelo)/(float)(mdL->nomAngAccel))*(float)mdL->veloControlInterval)-1-mdL->veloControlInterval; //-1: Toleranz wegen runden, lieber ein Schritt vorher MinGeschw. erreichen

  if (mStepsTillBrakeL < 0) {
    mStepsTillBrakeL = mStepsL/2; // Ab der hälfte wieder bremsen -> Zu kurze Strecke, Vsoll kann nicht erreicht werden
  }
  /*
  // Für Strecken <= 1cm wahrscheinlich -> Funktioniert aber noch nicht
  while (mStepsTillBrakeL < 0){
    // Für zu kurze Strecken
    mdL->nomAngVelo -= 1;
    mStepsTillBrakeL = mStepsL-(((float)(mdL->nomAngVelo - mdL->actAngVelo)/(float)(mdL->nomAngAccel))*(float)mdL->veloControlInterval)-1-mdL->veloControlInterval;
  } 
  */
//  sp("NomAngAccel:");
//  spl((((float)(mdL->nomAngVelo - mdL->actAngVelo)/(float)(mdL->nomAngAccel))*(float)mdL->veloControlInterval));
//  sp("BrakeSteps-All:");
//  sp(mStepsTillBrakeL);
//  sp("-");
//  spl(mStepsL);

  int mStepsR = tickRatioRight*mdR->ticksPerRotation;
  int mStepsTillBrakeR = mStepsR-(((float)(mdR->nomAngVelo - mdR->actAngVelo)/(float)(mdR->nomAngAccel))*(float)mdR->veloControlInterval)-1-mdR->veloControlInterval; // -veloControlInterval, weil erst berechnet wird und 1 Intervall später die Werte übernommen werden
  if (mStepsTillBrakeR < 0) {
    mStepsTillBrakeR = mStepsR/2;
  }
  /*
  while (mStepsTillBrakeR < 0){
    // Für zu kurze Strecken
    mdR->nomAngVelo -= 1;
    mStepsTillBrakeR = mStepsR-(((float)(mdR->nomAngVelo - mdR->actAngVelo)/(float)(mdR->nomAngAccel))*(float)mdR->veloControlInterval)-1-mdR->veloControlInterval;
  } 
  */
  calcInterruptTime(mdL); // Weil nicht mit Geschw. 0 angefangen wird
  calcInterruptTime(mdR);
  driveSteps(mdL, mStepsTillBrakeL, mStepsL);
  driveSteps(mdR, mStepsTillBrakeR, mStepsR);

  activateTimer(mdL->ch);
  activateTimer(mdR->ch);
  serviceRoutine();
}

void driveVelocity(struct struMotorDriver *mdL, struct struMotorDriver *mdR) {
  
}

void earlyStopNow(struct struMotorDriver *mdL, struct struMotorDriver *mdR) {
  //spl("early!");
  unsigned long countL = getCount(mdL->ch);
  unsigned long countR = getCount(mdR->ch);
  if ((countL < (mdL->brakeCount - 5)) && (countR < (mdR->brakeCount - 5))) { // 5: Toleranz, in der er die Standard-Bremsung durchführt
    // Standard-Bremsung zu weit in Zukunft -> vorzeitige Bremsung
    mdL->earlyStop = true;
    mdR->earlyStop = true;
    mdL->earlyStopFlag = true;
    mdR->earlyStopFlag = true;
  }
}

void returnDriving(struct struMotorDriver *mdL, struct struMotorDriver *mdR) {
  //digitalWrite(13, HIGH);
  // Werte vor dem vorzeitigen Bremsen wieder nutzen, um Fahrt wieder aufzunehmen
  int tempCountL = mdL->earlyStopCountFlag - mdL->actCount; // Count des Abbruchs - Count des Anfangs des Fahrbefehls
  int tempCountR = mdR->earlyStopCountFlag - mdR->actCount;
  int mStepsTillBrakeL = mdL->brakeCount - tempCountL;
  int mStepsTillBrakeR = mdR->brakeCount - tempCountR;
  int mStepsL = mdL->nomCount - tempCountL;
  int mStepsR = mdR->nomCount - tempCountR;
  
  if (mStepsTillBrakeL < 0) {
    mStepsTillBrakeL = mStepsL/2; // Ab der hälfte wieder bremsen -> Zu kurze Strecke, Vsoll kann nicht erreicht werden
  }  
  if (mStepsTillBrakeR < 0) {
    mStepsTillBrakeR = mStepsR/2;
  }
  driveSteps(mdL, mStepsTillBrakeL, mStepsL);
  driveSteps(mdR, mStepsTillBrakeR, mStepsR);
}

void curve() {
  
}
