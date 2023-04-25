void timerControlRisingEdge(struct struMotorDriver *md);
void timerControlRisingEdgeLeft(void *motorDriver) {
  struct struMotorDriver *md = (struct struMotorDriver*) motorDriver;
  setLeftPulsePinHigh();
//  sp("LeftCountRise:");
//  spl(getCount(md->ch));
  timerControlRisingEdge(md);
//  sp(md->actAngVelo);
//  sp(" ");
}
void timerControlRisingEdgeRight(void *motorDriver) {
  struct struMotorDriver *md = (struct struMotorDriver*) motorDriver;
  setRightPulsePinHigh();
//  sp("RightCountRise:");
//  spl(getCount(md->ch));
  timerControlRisingEdge(md);
  //spl(md->actAngVelo);
  }
void timerControlRisingEdge(struct struMotorDriver *md) { //int countAction) {
  unsigned long count = getCount(md->ch);
//  if(md->whichTemp == 0) {
//    sp(md->actAngVelo);
//    sp(" ");
//  } else {
//    spl(md->actAngVelo);
//  }
//  spl(md->actAngVelo);
  if ((count == md->brakeCount) || md->earlyStop) {
//    spl("CountOrBrake:");
//    if (md->earlyStop)
//      spl("Stop!");
//    if (!md->earlyStop)
//      spl("Brake?");
//    if ((count == md->brakeCount))
//      spl("Brake!");
//    if (!(count == md->brakeCount))
//      spl("Stop?");
    setAngVelo(md,0); // Falls das Fahrzeug nach einem Fahrbefehl nicht stehen soll, muss diese Stelle angepasst werden
    md->veloControlCount = 0; // Wird diesen Durchlauf noch auf 1 erhöht
    md->calcNextAngVelo = true;
    md->reachedTargetVelo = false;
  }
  if (md->earlyStop) {
    md->earlyStopCountFlag = count;
    md->earlyStop = false;
  }

  if (md->veloControlCount++ == md->veloControlInterval) { // Wird erst nach Vergleich erhöht -> soll erst nach x Pulsen Geschw. ändern
    // jetzt wieder Geschwindigkeit ändern
    if (!md->reachedTargetVelo) {// Führt dazu, dass die Geschw.-Erhöhung für nächste Runde berechnet wird
    md->ch->highTime = md->highTime;
    md->ch->lowTime = md->lowTime;
    md->calcNextAngVelo = true; 
    md->veloControlCount = 1; // Auf 1, weil es schon der Start vom nächsten Puls ist
    }
  }
}

void timerControlFallingEdgeLeft(void *motorDriver) {
  struct struMotorDriver *md = (struct struMotorDriver*) motorDriver;
  setLeftPulsePinLow();
//  sp("LeftCountFall:");
//  sp(getCount(md->ch));
  timerControlFallingEdge(md);
}

void timerControlFallingEdgeRight(void *motorDriver) {
  struct struMotorDriver *md = (struct struMotorDriver*) motorDriver;
  setRightPulsePinLow();
//  sp("RightCountFall:");
//  spl(getCount(md->ch));
  timerControlFallingEdge(md);
}
void timerControlFallingEdge(struct struMotorDriver *md) {
  unsigned long count = getCount(md->ch);
  if (count >= md->nomCount || (md->earlyStopFlag)) {    
    deactivateTimer(md->ch); //Zielwert wurde erreicht
    md->actAngVelo = 0; // Falls das Fahrzeug nach einem Fahrbefehl nicht stehen soll, muss diese Stelle angepasst werden
    md->nomAngVelo = md->lastNomAngVelo; // Setzt die Geschw. wieder auf vor den Bremsbefehl zurück
    md->veloControlCount = 0;
    md->reachedTargetVelo = false;
    if ((md->whichTemp==1) && (!md->earlyStopFlag)) { // Pfusch: Rechter MotorDriver - Der 2. && Wenn keine vorzeitige Bremsung ausgeführt wurde
      inkrMotionCommand();
    }
    if (md->earlyStopFlag) {
      md->earlyStopFinished = true; // Dadurch kann Fahrt nach einer vorzeitigen Bremsung wieder aufgenommen werden 
    }
  }
}
