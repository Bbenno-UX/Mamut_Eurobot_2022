struct struMotorDriver* getMotorDriver() {
  static int nextMotorDriver = 0;
  if (nextMotorDriver < MAX_MOTOR_DRIVER){
    return &(motorDriver[nextMotorDriver++]);
  } else return NULL; // kein MotorDriver mehr verfügbar
}

void initMotorDriverSoftware(struct struMotorDriver *md, struct struChannel *ch, int minAngVelo, int minAngAccel, int maxAngVelo, int maxAngAccel, int veloControlInterval, float tickDutyCycle) {
  
  md->ch = ch;
  md->minAngVelo = minAngVelo;
  md->minAngAccel = minAngAccel;
  md->maxAngVelo = maxAngVelo;
  md->maxAngAccel = maxAngAccel;
  md->veloControlInterval = veloControlInterval; 
  md->veloControlCount = veloControlInterval; // Damit direkt der Puls berechnet wird
  md->tickDutyCycle = tickDutyCycle;

  // Wird so festgelegt:
  md->nomAngVelo = 0;
  md->actAngVelo = 0;
  md->lastNomAngVelo = 0;
  md->nomAngAccel = 0;
  md->actAngAccel = 0;
  md->lastNomAngAccel = 0;
  md->calcNextAngVelo = false;
  md->reachedTargetVelo = false;
  md->earlyStop = false;
  md->earlyStopFlag = false;
  
  md->dirForward = true;
}
void initMotorDriverHardware(struct struMotorDriver *md, int dirPin, int pulsePin, int enablePin, bool changeDir, float unitFactor, int circumference, int ticksPerRotation) {
  md->dirPin = dirPin;
  md->pulsePin = pulsePin;
  md->enablePin = enablePin;
  
  md->changeDir = changeDir;
  md->unitFactor = unitFactor;
  md->circumference = circumference;
  md->ticksPerRotation = ticksPerRotation;
  md->factVeloToTicks = ((float)md->circumference*1000000.)/((float)md->ticksPerRotation);//2*CONST_PI*md->ticksPerRotation;//*1000000;  // 10^6 für Mikrosekunden
  
}

void setAngVelo(struct struMotorDriver *md, int angularVelocity) {
  md->lastNomAngVelo = md->actAngVelo; // letzte gefahrene Geschwindigkeit
  if (angularVelocity < md->minAngVelo) {
    md->nomAngVelo = md->minAngVelo;  
  } else if (angularVelocity > md->maxAngVelo) {
    md->nomAngVelo = md->maxAngVelo;
  } else {
    md->nomAngVelo = angularVelocity;
  }
}

void setAngAccel(struct struMotorDriver *md, int angularAcceleration) {
    md->lastNomAngAccel = md->actAngAccel; // letzte gefahrene Beschleunigung
  if (angularAcceleration < md->minAngAccel) {
    md->nomAngAccel = md->minAngAccel;  
  } else if (angularAcceleration > md->maxAngAccel) {
    md->nomAngAccel = md->maxAngAccel;
  } else {
    md->nomAngAccel = angularAcceleration;
  }
}

void toggleDir(struct struMotorDriver *md, unsigned int *lowTime, unsigned int *highTime) {
  // Nur wenn Geschwindigkeit auf 0
  md->changeDir = !md->changeDir;
}

void driveSteps(struct struMotorDriver *md, int brakeSteps, int nomSteps) {
  // Fährt bestimmte Anzahl an Interrupts
  md->lastNomAngVelo = md->nomAngVelo; // Soll-Geschw. zwischenspeichern
//  sp(md->whichTemp);
//  sp(":Change:");
//  sp(md->changeDir);
//  sp(":Forw:");
//  spl(md->dirForward);
  if (md->changeDir == md->dirForward) {
    digitalWrite(md->dirPin, HIGH); // In Zukunft mit Bitoperationen?
//    sp(md->whichTemp);
//    spl(":DirHIGH");
  } else {
    digitalWrite(md->dirPin, LOW);
//    sp(md->whichTemp);
//    spl(":DirLOW");
  }
  
  md->actCount = getCount(md->ch); // Vielleicht nicht benötigt
  md->brakeCount = md->actCount + brakeSteps;
  md->nomCount = md->actCount + nomSteps;
  activateTimer(md->ch);
}

void calcInterruptTime(struct struMotorDriver *md) {

  // Aktuelle Beschleunigung berechnen
//  int angAccelSign = (md->actAngAccel < md->nomAngAccel) ? 1 : -1;
//  int min1Accel = md->nomAngAccel * md->angAccelFactor; // Durch den Faktor wird eine Steigung der Motorbeschleunigung erzeugt -> Sanfteres Anfahren
//  int min2Accel = md->actAngAccel - md->nomAngAccel;
//  min2Accel = abs(min2Accel);
//  md->actAngAccel += angAccelSign * min(min1Accel, min2Accel); // Wenn in Parametern von min() gerechnet wird kann es zu Fehlern kommmen (laut Arduino.cc) -> min1Accel,min2Accel

  // Aktuelle Geschwindigkeit berechnen
  int angVeloSign; // = (md->actAngVelo < md->nomAngVelo) ? 1 : -1;
//  sp("Calc:");
//  sp(md->whichTemp);
//  sp(":ACT");
//  sp(md->actAngVelo);
//  sp("NOM");
//  sp(md->nomAngVelo);
  if(md->actAngVelo < md->nomAngVelo) {
    angVeloSign = 1;
//    sp("SIGN:");
//    spl("1");
  } else if(md->actAngVelo > md->nomAngVelo){
    angVeloSign = -1;
//    sp("SIGN:");
//    spl("-1");
  } else {
    angVeloSign = 0; // Verhindert die Änderung, wenn Sollwert=Istwert
  }

  // Hier nominalwert der Winkelbeschl. nutzen -> Macht die Beschl.-Berechnung unnötig
  int min1Velo = md->nomAngAccel; //md->actAngAccel;
  int min2Velo = md->actAngVelo - md->nomAngVelo;
  min2Velo = abs(min2Velo);
  md->actAngVelo += angVeloSign * min(min1Velo, min2Velo);
  if (md->actAngVelo == md->nomAngVelo) {
    md->reachedTargetVelo = true;
  }
  // Interrupt-Zeitberechnung
  md->highTime = ((float)md->factVeloToTicks * (float)md->tickDutyCycle)/(float)md->actAngVelo; // Periodendauer Anteilig High-Anteil // -32 durch Vorteiler 256 Offset // Vielleicht falsch: 1 / Winkelgeschwindigkeit -> Sekunden pro Grad im Bogenmaß (s/rad)
  md->lowTime = ((float)md->factVeloToTicks * (1. - (float)md->tickDutyCycle))/(float)md->actAngVelo;

  md->calcNextAngVelo = false;
}
