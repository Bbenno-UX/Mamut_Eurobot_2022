// Channel-Struct, um über einen Timer mehrere PWM-Signale laufen zu lassen
// Prinzip: Jeder channel hat eine Zeit bis zum nächsten Interrupt (nextChange).
//          Diese Zeiten werden verglichen, sodass der Zeitpunkt für den nächsten
//          Interrupt ermittelt wird.
bool go=false;

struct struChannel{
   //Channel für PWM - Signale
                          // |¯|___
   unsigned int highTime; // |¯|
   unsigned int lowTime;  //    ___
   unsigned int nextChange; // Zeit bis zur nächsten Änderung dieses Channels
   unsigned long count; // Zum Zählen der Interrupts dieses Channels
   //int countAction; // Zum ausführen einer Aktion, wenn Counter auf 0, bei -1 inaktiv
   bool setHigh; // setHigh == true -> risingFlankAction(), sonst fallingFlank Action()
   bool active; // Timer aktiv?
   void (*risingFlankAction)(void *parameterRising);
   void (*fallingFlankAction)(void *parameterFalling);
   void *parameterRising;
   void *parameterFalling;
} channel[T1_MAX_CHANNELS];



// Motor-Driver-Struct: Jeder Motor hat verschiedene Konstanten, Variablen und Funktionen
struct struMotorDriver{
  int whichTemp; // Zur Identifizierung der Motoren, soll ersetzt werden
  int dirPin;
  int pulsePin;
  int enablePin;

  float unitFactor; // Faktor für die Maßeinheit (Grundeinheit m), bspw. 0.01 bei cm
  int circumference; // Umfang Motor in Maßeinheit nach unitFactor

  bool changeDir; // Falls der Motor entgegen der gewünschten Richtung dreht (Vorzeichentausch)
  bool dirForward; // Fährt Richtung vorwärts
  
  int nomAngVelo; // Sollwert Winkelgeschwindigkeit 
  int actAngVelo; // Istwert Winkelgeschwindigkeit
  int lastNomAngVelo; // letzter Sollwert (eigentlich wird letzter Istwert zugewiesen) Winkelgeschwindigkeit

  int nomAngAccel; // Winkelbeschleunigung
  int actAngAccel;
  int lastNomAngAccel;
  
  //float angAccelFactor; // Faktor Winkelbeschleunigung (Steigung der Beschleunigung)
  int veloControlInterval; // Intervall, nach wievielen Ticks die Geschw. neu berechnet/geprüft wird -> ist indir. prop. zur Steigung der Winkelbeschl.
  // *******************************
  //Calibration Factors hier einfügen
  // Nur für Motor (z.B. wenn die Motoren unterschiedlich schnell fahren)
  // *******************************
  int minAngVelo;
  int minAngAccel;
  int maxAngVelo;
  int maxAngAccel;

  float tickDutyCycle; // Gibt das Verhältnis zwischen highTime und Periodendauer eines Ticks an // Hängt vielleicht mit Bestromung zusammen???
  float factVeloToTicks; // Geschwindigkeit in Ticks Faktor
  int ticksPerRotation; // Geht es vielleicht auch ohne?- Aber erstmal so!
  void (*tick)(); // Soll nach jedem Tick aufgerufen werden

  int actCount; // Vielleicht nicht benötigt
  int nomCount;
  int brakeCount; // Nach so vielen Steps soll der Bremsvorgang gestartet werden
  int veloControlCount; // Zählt hoch und wird mit veloControlInterval verglichen
  struChannel *ch;
  bool calcNextAngVelo; // ist diese Variable true, wird zum nächstmöglichen Zeitpunkt die nächste Geschwindigkeit berechnet
  bool reachedTargetVelo;
  bool earlyStop; // Sofort Anhalten
  bool earlyStopFlag; // Sofort Anhalten Merker
  bool earlyStopFinished; // Fahrt kann wieder aufgenommen werden
  unsigned long earlyStopCountFlag; // Hält den Stand an Steps vor dem vorzeitigen Stop fest
                         // |¯|___
  unsigned int lowTime;  //    ___
  unsigned int highTime; // |¯|
} motorDriver[MAX_MOTOR_DRIVER];
