/*
   windchill.ino

   windchill control software
*/

#include <Encoder.h>

#define SENSOR0 A0
#define SENSOR1 A1
#define SENSOR2 A2
#define SENSOR3 A3
#define SENSOR4 A4
#define SENSOR5 A5
#define SENSOR6 A6
#define SENSOR7 A7
#define SENSOR8 A8
#define SENSOR9 A9
#define SENSOR10 A10
#define SENSOR11 A11
#define SENSOR12 A12
#define SENSOR13 A13
#define SENSOR14 A14
#define SENSOR15 A15

#define PIN0 0 // RX0 & USB TO TTL
#define PIN1 1 // TX0 & USB TO TTL
#define PIN2 2 // INT0 & PWM
#define PIN3 3 // INT1 & PWM
#define PIN4 4 // PWM
#define PIN5 5 // PWM
#define PIN6 6 // PWM
#define PIN7 7 // PWM
#define PIN8 8 // PWM
#define PIN9 9 // PWM
#define PIN10 10 // PWM
#define PIN11 11 // PWM
#define PIN12 12 // PWM
#define PIN13 13 // LED & PWM
#define PIN14 14 // TX3
#define PIN15 15 // RX3
#define PIN16 16 // TX2
#define PIN17 17 // RX2
#define DCMOTORENCODER1CHA 18// TX1 & INT5
#define DCMOTORENCODER1CHB 19 // RX1 & INT4
#define DCMOTORENCODER2CHA 20 // INT3
#define DCMOTORENCODER2CHB 21 // INT2
#define PIN22 22
#define PIN23 23
#define PIN24 24
#define PIN25 25
#define PIN26 26
#define PIN27 27
#define PIN28 28
#define PIN29 29
#define PIN30 30
#define PIN31 31
#define PIN32 32
#define PIN33 33
#define PIN34 34
#define PIN35 35
#define PIN36 36
#define PIN37 37
#define PIN38 38
#define PIN39 39
#define DCMOTORL1 40
#define DCMOTORL2 41
#define DCMOTORL3 42
#define DCMOTORL4 43
#define DCMOTORENABLE1 44 // PWM
#define DCMOTORENABLE2 45 // PWM
#define PIN46 46 // PWM
#define PIN47 47
#define PIN48 48
#define PIN49 49
#define PIN50 50
#define PIN51 51
#define PIN52 52
#define PIN53 53
#define PIN54 54
#define PIN55 55

void (*state)(void);

// position of the device relative to bottom left corner
double position_x; // [m]
double position_y; // [m]
// maximum coordinates of the system
double max_x; // [m]
double max_y; // [m]

// encoder data objects
Encoder dcmotor1(DCMOTORENCODER1CHA, DCMOTORENCODER1CHB);
Encoder dcmotor2(DCMOTORENCODER2CHA, DCMOTORENCODER2CHB);

// initializes the pins
void setup() {
  // sets initial state to on procedure
  state = &on;

  Serial.begin(9600);

  // SET DC MOTOR PINS AS OUTPUT
  pinMode(DCMOTORENABLE1, OUTPUT);
  pinMode(DCMOTORENABLE2, OUTPUT);
  pinMode(DCMOTORL1, OUTPUT);
  pinMode(DCMOTORL2, OUTPUT);
  pinMode(DCMOTORL3, OUTPUT);
  pinMode(DCMOTORL4, OUTPUT);

  // SET DC MOTOR PINS TO TURN MOTORS OFF
  analogWrite(DCMOTORENABLE1, 0);
  analogWrite(DCMOTORENABLE2, 0);
  digitalWrite(DCMOTORL1, LOW);
  digitalWrite(DCMOTORL2, LOW);
  digitalWrite(DCMOTORL3, LOW);
  digitalWrite(DCMOTORL4, LOW);
}

// runs function of whichever state the system is currently in
void loop() {
  (*state)();
}

// all systems powered but not performing any actions
// can transition to state of performing actions
void standby() {
  // TURNS BOTH DC MOTORS OFF
  analogWrite(DCMOTORENABLE1, 0);
  analogWrite(DCMOTORENABLE2, 0);
  digitalWrite(DCMOTORL1, LOW);
  digitalWrite(DCMOTORL2, LOW);
  digitalWrite(DCMOTORL3, LOW);
  digitalWrite(DCMOTORL4, LOW);
  return;
}

// turns the device on and performs all checks necessary
void on() {
  state = &standby;
}

// reads sensors to determine location of the device relative to its environment
void calibrate() {
  return;
}

