//int getChannel() {
//  static int nextChannel = 0;
//  if (nextChannel < T1_MAX_CHANNELS){
//    return nextChannel++; // Wenn ein Channel verfügbar ist, gib diesen zurück und inkrementiere anschließend
//  } else return -1;
//}
void serviceRoutine();
struct struChannel* getChannel() {
  static int nextChannel = 0;
  if (nextChannel < T1_MAX_CHANNELS){
    return &(channel[nextChannel++]); // Wenn ein Channel verfügbar ist, gib diesen zurück und inkrementiere anschließend
  } else return NULL;
}

unsigned long getCount(struct struChannel* ch) {
  return ch->count;
}

void setCount(struct struChannel* ch, unsigned long newCount) {
  ch->count = newCount;
}

void initChannel(struct struChannel* ch, unsigned int highTime, unsigned int lowTime, void (*risingFlankAction) (void*), void (*fallingFlankAction) (void*), void *parameterRising, void *parameterFalling) {
  ch->highTime = highTime;
  ch->lowTime = lowTime;
  ch->nextChange = 0;
  ch->count = 0;
  ch->setHigh = true;
  ch->active = false;
  ch->risingFlankAction = risingFlankAction;
  ch->fallingFlankAction = fallingFlankAction;
  ch->parameterRising = parameterRising;
  ch->parameterFalling = parameterFalling;
}


void timerSetup() {
  //Timer 1 setup-----------------------------------------------
  noInterrupts();           //Damit hier auf keinen Fall andere Interrupts storen

  TCCR1A = 0;               //die beiden control register auf 0 setzen
  TCCR1B = 0;               //um Fehler durch mogliche Voreinstellungen zu verhindern
  TCNT1 = 0;                //falls schon mal etwas hochgez hlt wurde, dann Inhalt loschen

  OCR1A = 10000;            //(Output Compare Register) wenn TCNT1 das erreicht, dann Interrupt

  TCCR1B |= (1 << WGM12);   //hiermit wird bestimmt dass das Output Compare Register (OCR1A) auch zum Vergleichen genomen wird (CTC MODE) clear timer on compare
  //TCCR1B |= (1 << CS12);  //abhangig von den CS bits wird ein PRESCALER gesetzt. Hier 256
  //TCCR1B |= ((1 << CS11) | (1 << CS10));//hier 64
  TCCR1B |= (1 << CS11); // hier 8
  TIMSK1 |=  (1 << OCIE1A);//Interrupt wenn TCNT1 = OCR1A       (Timer 1 Output Compare A Match Interrupt Enable 1 setzen)
  //TIMSK1 &= ~(1 << OCIE1A);  //hier dann halt kein Interrupt  (Timer 1 Output Compare A Match Interrupt Enable 0 setzen)
  interrupts();
  //---------------------------------------------------
  OCR1A = 65535; // Unschön, lieber Timer deaktiviert lassen, bis erster Channel aktiv
}


ISR (TIMER1_COMPA_vect) // Timer 1 Compare Service Routine - Ausführung wenn (Timer 1 Counter) TCNT1 = OCR1A
{
  TIMER_1_NEXT_INTERRUPT = 65535; // Vermeidet Interrupt während der ISR
  serviceRoutine();
  TCNT1  = 0; // Timer auf 0 setzen
}

void activateTimer(struct struChannel *ch) {
  ch->active = true;
  TIMSK1 |=  (1 << OCIE1A);
}

void deactivateTimer(struct struChannel *ch) {
  ch->active = false;
  bool tempStillChannelsActive = false;
  // Wenn kein Channel aktiv ist
  for (int i = 0; i < T1_MAX_CHANNELS; i++) {
    tempStillChannelsActive = tempStillChannelsActive || channel[i].active;
  }
  if (!tempStillChannelsActive) {
    TIMSK1 &= ~(1 << OCIE1A); // Timer deaktivieren
  }
}

void serviceRoutine() {
//   unsigned long timer = micros();
    for (int i = 0; i < T1_MAX_CHANNELS; i++) {      
      if (channel[i].active) {
        if (channel[i].nextChange == 0) { // Hier ggf noch Toleranz in Höhe der Dauer der ISR einfügen um nicht zu spät zu pulsen
          if (channel[i].setHigh) {
            channel[i].count++;
            channel[i].risingFlankAction(channel[i].parameterRising); //channel[i].countAction); // Sollte counter überprüfen und Channel ggf wieder aktiv schalten
            if (channel[i].active) // Für den Fall, dass Channel nach risingFlankAction inaktiv bleibt bleibt setHigh auf true
              channel[i].setHigh = false;
          } else {
            channel[i].fallingFlankAction(channel[i].parameterFalling);//channel[i].countAction);
            channel[i].setHigh = true;
          }
          channel[i].nextChange = channel[i].setHigh ? channel[i].lowTime : channel[i].highTime; // Nächste Flanke steigend -> Low-Zeit
        }
  //      Serial.print("Timerx:");
  //      Serial.println(i);
  //      sp("highTime:");
  //      spl(channel[i].highTime);
  //      sp("lowTime:");
  //      spl(channel[i].lowTime);
      }  
  }
  // Zeit für den nächsten Interrupt festlegen, durch vergleichen der channels
  int tempNextInterrupt = 65535; // Höchster Wert beim 16 Bit Timer
  for (int i = 0; i < T1_MAX_CHANNELS; i++) {
    if (channel[i].active) {
      if (channel[i].nextChange < tempNextInterrupt)
        tempNextInterrupt = channel[i].nextChange;
    }
  }  
  // Zeit für den nächsten Interrupt abziehen
  for (int i = 0; i < T1_MAX_CHANNELS; i++) {
    if (channel[i].active) {
        channel[i].nextChange -= tempNextInterrupt;
    }
  }  
  TIMER_1_NEXT_INTERRUPT = tempNextInterrupt;  
//  timer = micros()-timer;
//  Serial.println(timer);
}
