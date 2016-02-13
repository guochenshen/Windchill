/*
   windchill.ino

   windchill control software
*/

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

#define INPUT0 0 // RX0 & USB TO TTL
#define INPUT1 1 // TX0 & USB TO TTL
#define INPUT2 2 // INT0 & PWM
#define INPUT3 3 // INT1 & PWM
#define INPUT4 4 // PWM
#define INPUT5 5 // PWM
#define INPUT6 6 // PWM
#define INPUT7 7 // PWM
#define INPUT8 8 // PWM
#define INPUT9 9 // PWM
#define INPUT10 10 // PWM
#define INPUT11 11 // PWM
#define INPUT12 12 // PWM
#define INPUT13 13 // LED & PWM
#define INPUT14 14 // TX3
#define INPUT15 15 // RX3
#define INPUT16 16 // TX2
#define INPUT17 17 // RX2
#define INPUT18 18// TX1 & INT5
#define INPUT19 19 // RX1 & INT4
#define INPUT20 20 // INT3
#define INPUT21 21 // INT2
#define INPUT22 22
#define INPUT23 23
#define INPUT24 24
#define INPUT25 25
#define INPUT26 26
#define INPUT27 27
#define INPUT28 28
#define INPUT29 29
#define INPUT30 30
#define INPUT31 31
#define INPUT32 32
#define INPUT33 33
#define INPUT34 34
#define INPUT35 35
#define INPUT36 36
#define INPUT37 37
#define INPUT38 38
#define INPUT39 39
#define INPUT40 40
#define DCMOTORENABLE1 41
#define DCMOTORENABLE2 42
#define INPUT43 43
#define DCMOTORL1 44 // PWM
#define DCMOTORL2 45 // PWM
#define DCMOTORL3 46 // PWM
#define DCMOTORL4 47
#define INPUT48 48
#define INPUT49 49
#define INPUT50 50
#define INPUT51 51
#define INPUT52 52
#define INPUT53 53
#define INPUT54 54
#define INPUT55 55

void (*state)(void);

void setup() {
  state = &on;
  
  pinMode(DCMOTORENABLE1, OUTPUT);
  pinMode(DCMOTORENABLE2, OUTPUT);
  pinMode(DCMOTORL1, OUTPUT);
  pinMode(DCMOTORL2, OUTPUT);
  pinMode(DCMOTORL3, OUTPUT);
  pinMode(DCMOTORL4, OUTPUT);

  digitalWrite(DCMOTORENABLE1, LOW);
  digitalWrite(DCMOTORENABLE2, LOW);
}

void loop() {
  (*state)();
}

void on() {
  return;
}

void standby() {
  return;
}

void calibrate() {
  return;
}

