// Pin Belegung
#define LEFT_DIRPIN 11
#define LEFT_PULSEPIN 12
#define LEFT_ENABLEPIN 8
#define RIGHT_DIRPIN 9
#define RIGHT_PULSEPIN 10
#define RIGHT_ENABLEPIN 8
#define NOTFALLBOGENLAENGE 100
// Setup durch DIP-Schalter
#define DC_MOTOR_PIN 4
//#define PROGRAM_PIN_1 13
#define START1 1900
#define START2 1600
#define SPR 200
#define SPEED 50
#define PIN_1 6
#define PUMPENPIN 6
//#define PIN2 7
#define STPEN 3
#define STPDIR 2
// #define STP3 6
// #define STP4 7
// Kommunikation mit Arduino Uno
//#define COMM_PIN_OUT A4
//#define COMM_PIN_IN A5


#define LEFT_FALSE_DIRECTION false
#define RIGHT_FALSE_DIRECTION false
#define UNIT 0.01
// Umfang
#define LEFT_CIRCUMFERENCE 20
#define RIGHT_CIRCUMFERENCE 20

#define LEFT_TICKSPERROTATION 800
#define RIGHT_TICKSPERROTATION 800

// Min Winkelgeschwindigkeit // Aktuell noch so Geschwindigkeit
#define LEFT_MIN_ANGULARVELOCITY 1
#define RIGHT_MIN_ANGULARVELOCITY 1

// Min Winkelbeschleunigung
#define LEFT_MIN_ANGULARACCELERATION 1
#define RIGHT_MIN_ANGULARACCELERATION 1

// Max Winkelgeschwindigkeit
#define LEFT_MAX_ANGULARVELOCITY 100
#define RIGHT_MAX_ANGULARVELOCITY 100

// Max Winkelbeschleunigung
#define LEFT_MAX_ANGULARACCELERATION 2
#define RIGHT_MAX_ANGULARACCELERATION 2

// Aktuelle Winkelgeschwindigkeit
#define LEFT_ACTUAL_ANGULARVELOCITY 10
#define RIGHT_ACTUAL_ANGULARVELOCITY 10

// Aktuelle Winkelbeschleunigung
#define LEFT_ACTUAL_ANGULARACCELERATION 3
#define RIGHT_ACTUAL_ANGULARACCELERATION 3

#define LEFT_VELOCITY_CONTROLINTERVALL 10
#define RIGHT_VELOCITY_CONTROLINTERVALL 10

#define LEFT_DUTY_CYCLE 0.8
#define RIGHT_DUTY_CYCLE 0.8

#define DISTANCE_TO_CENTER 9


//Durch Port-Manipulation schneller im Schreiben (statt digitalWrite)
// PortD Digital Pin 0-7
// PortB Digital Pin 8-13
#define LEFT_PULSEPIN_HIGH    PORTB |=  0b00010000; // PIN 12 - Bitoperation: Setzt nur diesen Pin auf HIGH
#define LEFT_PULSEPIN_LOW     PORTB &=  ~0b00010000;

#define RIGHT_PULSEPIN_HIGH   PORTB |=  0b00000100; // PIN 10
#define RIGHT_PULSEPIN_LOW    PORTB &=  ~0b00000100; // Bitoperation(UND-Verknüpfung mit Komplement): Setzt nur diesen PIN auf LOW

//#define COMM_PIN_OUT_HIGH     PORTD |=  0b01000000; // PIN 6
//#define COMM_PIN_OUT_LOW      PORTD &=  ~0b01000000;
//#define COMM_PIN_OUT_HIGH     digitalWrite(A4,HIGH);//PORTD |=  0b01000000; // PIN 6
//#define COMM_PIN_OUT_LOW    digitalWrite(A4,LOW);//  PORTD &=  ~0b01000000;

//#define READ_COMM_PIN_IN (0b10000000 & PIND) // Pin 7
//#define READ_COMM_PIN_IN digitalRead(A5)
#define ANZ_MOTOR_DRIVER 2
#define T1_MAX_CHANNELS 2
#define MAX_MOTOR_DRIVER 2

#define TIMER_1_NEXT_INTERRUPT OCR1A

#define CONST_PI 3.14159

#define sp(a) Serial.print(a)
#define spl(a) Serial.println(a)

// Makros als FunktsGH
void setLeftPulsePinHigh() {
  LEFT_PULSEPIN_HIGH;    
}
void setLeftPulsePinLow() {
  LEFT_PULSEPIN_LOW;  
}
void setRightPulsePinHigh() {
    RIGHT_PULSEPIN_HIGH;
}
void setRightPulsePinLow() {
  RIGHT_PULSEPIN_LOW;  
}

void setBothPulsePinsHigh() {
    LEFT_PULSEPIN_HIGH;
    RIGHT_PULSEPIN_HIGH;
}

void setBothPulsePinsLow() {
    LEFT_PULSEPIN_LOW;
    RIGHT_PULSEPIN_LOW;    
}
void timerControlFallingEdge(struct struMotorDriver *md);
void timerControlFallingEdgeLeft(struct struMotorDriver *p_md = NULL);
void timerControlFallingEdgeRight(struct struMotorDriver *p_md = NULL);
void drive(struct struMotorDriver *mdL, struct struMotorDriver *mdR, float tickRatioLeft, float tickRatioRight);

void inkrMotionCommand();

int motionCommandValue = 1;