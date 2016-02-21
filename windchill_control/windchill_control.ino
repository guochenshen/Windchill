/*
   windchill.ino

   windchill control software
*/

#include <PID_v1.h>
#include <Encoder.h>

// comment out #define DEBUG to disable debug printing
#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
#endif

// analog input pins
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

// digital pins
#define PIN0 0 // RX0 & USB TO TTL
#define PIN1 1 // TX0 & USB TO TTL
#define PIN2 2 // INT0 & PWM
#define PIN3 3 // INT1 & PWM
#define LIMIT1 4 // PWM
#define LIMIT2 5 // PWM
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

#define DCENCODERREVOLUTION 4741.44 // encoder ticks per revolution

// macros to define dc motors
#define DCLEFT 0;
#define DCRIGHT 1;

// macros to define direcction
#define FORWARD 1
#define BACKWARD -1

void (*state)(void); // current state of the machine
void (*savestate)(void); // saved state for returning to a state

// position of the device relative to bottom left corner
double position_x; // [m]
double position_y; // [m]
// maximum coordinates of the system
double max_x; // [m]
double max_y; // [m]

// encoder data objects
Encoder dcmotorenc1(DCMOTORENCODER1CHA, DCMOTORENCODER1CHB);
Encoder dcmotorenc2(DCMOTORENCODER2CHA, DCMOTORENCODER2CHB);
int32_t prevenc1; // previous encoder 1 position reading
int32_t prevenc2; // previous encoder 2 position reading
double prevvel1; // previous encoder 1 velocity reading
double prevvel2; // previous encoder 2 velocity reading

// time values
unsigned long prevtime; // previous time, used for velocity calculations

// PID values for dc motor 1
double dcsetpoint1; // setpoint
double dcinput1; // input value
double dcoutput1; // output PWM value
// PID constants dc motor 1
double kp1 = 10.0;
double ki1 = 0.0;
double kd1 = 0.0;
// PID values for dc motor 2
double dcsetpoint2; // setpoint
double dcinput2; // input value
double dcoutput2; // output PWM value
// PID constants for dc motor 2
double kp2 = 10.0;
double ki2 = 0.0;
double kd2 = 0.0;

// PID objects
PID dcmotorpid1(&dcinput1, &dcoutput1, &dcsetpoint1, kp1, ki1, kd1, DIRECT);
PID dcmotorpid2(&dcinput2, &dcoutput2, &dcsetpoint2, kp2, ki2, kd2, DIRECT);

// initializes the system
void setup() {
  // sets initial state to on state
  state = &on;
  // set save state as null
  savestate = NULL;
  // begin serial
  Serial.begin(9600);

  // dc motor output pins
  pinMode(DCMOTORENABLE1, OUTPUT);
  pinMode(DCMOTORENABLE2, OUTPUT);
  pinMode(DCMOTORL1, OUTPUT);
  pinMode(DCMOTORL2, OUTPUT);
  pinMode(DCMOTORL3, OUTPUT);
  pinMode(DCMOTORL4, OUTPUT);

  // limit switch input pins
  // HIGH when not triggered, LOW when triggered
  pinMode(LIMIT1, INPUT_PULLUP);
  pinMode(LIMIT2, INPUT_PULLUP);

  // set dc motors to turn off and reset PID
  dcmotorreset();
  // reset values for next state
  exitstate();
}

// main loop of state machine
// runs current function of state machine
void loop() {
  // check adhesion to window, adjust fan speed to ensure adhesion
  // check_fan();
  (*state)();
}

// turns the device on and performs all checks necessary before starting
void on() {
  // check all systems to ensure funtionality

  // once system is placed onto window, system switches to standy
  if (true) {
    state = &standby;
    exitstate();
  }
}

// calibrates the system and determines position
void calibrate() {
  // calibrate and determine position

  // once calibration is complete, return to standby state
  if (true) {
    state = &standby;
  }
  else {
    return;
  }
}

// all systems powered but not performing any actions
// can transition to state of performing actions
void standby() {
  DEBUG_PRINT("STANDBY STATE");

  // turns both dc motors off
    dcmotoroff();
    
  // temporary, check time passage, future will be on button push
  if (((micros() - prevtime) / 1000000) > 1) {
    state = &forward_climb;
    prevtime = micros();
  }
  // checks if calibrate button is pushed
  else if (false) {
    state = &calibrate;
  }
  else {
    return;
  }
}

// reads sensors to determine location of the device relative to its environment
void calibrate() {
  return;
}

void forward() {
  DEBUG_PRINT("Forward State");
  // turns on PID control
  dcmotorpidon();

  dcmotordirection(DCLEFT, FORWARD);
  dcmotordirection(DCRIGHT, FORWARD);
  analogWrite(DCMOTORENABLE1,255);
  analogWrite(DCMOTORENABLE2, 255);

  if (abs(dcmotorenc1.read()) > 720) {
    dcmotorreset();
    state = &right;
    exitstate();
  }
  else {
    return;
  }
}

void forward_climb() {
  DEBUG_PRINT("Forward Climb State");
  // turns on PID control
  dcmotorpidon();

  dcmotordirection(0, FORWARD);
  dcmotordirection(1, FORWARD);
  analogWrite(DCMOTORENABLE1, 255);
  analogWrite(DCMOTORENABLE2, 255);

  if (readlimit(1) == LOW) {
    dcmotoroff();
    delay(250);
    dcmotorreset();
    state = &right;
    exitstate();
  }
  else {
    return;
  }
}

void forward_descend() {
  DEBUG_PRINT("Forward Descend State");
  // turns on PID control
  dcmotorpidon();

  dcmotordirection(0, FORWARD);
  dcmotordirection(0, FORWARD);
  analogWrite(DCMOTORENABLE1, 255);
  analogWrite(DCMOTORENABLE2, 255);
}

void right() {
  DEBUG_PRINT("Right State");
  if (savestate != state) {
    savestate = &right;
    state = &reverse_turn;
  }
  else {
    savestate = &right;
  }

  // turns on PID control software
  dcmotorpidon();

  dcmotordirection(DCLEFT, FORWARD);
  dcmotordirection(DCRIGHT, BACKWARD);
  analogWrite(DCMOTORENABLE1, 255);
  analogWrite(DCMOTORENABLE2, 255);

  if (abs(dcmotorenc1.read()) > 720) {
    dcmotoroff();
    delay(500);
    dcmotorreset();
    state = &forward_climb;
    exitstate();
    savestate = NULL;
  }
  else  {
    return;
  }
}

void left() {
  DEBUG_PRINT("Left State");
  if (savestate != state) {
    savestate = &left;
    state= &reverse_turn;
  }
  else {
    savestate = &left;
  }
  // turns on PID control software
  dcmotorpidon();

  dcmotordirection(0, BACKWARD);
  dcmotordirection(1, FORWARD);
  analogWrite(DCMOTORENABLE1, 255);
  analogWrite(DCMOTORENABLE2, 255);

  if (abs(dcmotorenc1.read()) > 90) {
    dcmotoroff();
    delay(500);
    dcmotorreset();
    state = &forward_climb;
    exitstate();
    savestate = NULL;
  }
  else {
    return;
  }
}

void reverse() {
  DEBUG_PRINT("Reverse State");
  // turns on PID control software
  dcmotorpidon();

  if (true) {
    return;
  }
  else if (false) {
    dcmotorreset();
    exitstate();
  }
}

/*
 * performs a short reverse to turn properly
 */
void reverse_turn() {
  DEBUG_PRINT("Reverse Turn State");
  // turns on PID control software
  dcmotorpidon();

  dcmotordirection(0, BACKWARD);
  dcmotordirection(1, BACKWARD);
  analogWrite(DCMOTORENABLE1,255);
  analogWrite(DCMOTORENABLE2, 255);

  // perform short backup and return to previous state
  if (abs(dcmotorenc1.read()) > 360) {
    dcmotoroff();
    delay(500);
    state = savestate;
    dcmotorreset();
    exitstate();
  }
  else {
    return;
  }
}

// ********** MOTOR UTILITY FUNCTIONS **********

/*
 * reset dc motors and PID
 */
void dcmotorreset() {
  dcmotorpid1.SetMode(MANUAL);
  dcmotorpid2.SetMode(MANUAL);
  analogWrite(DCMOTORENABLE1, 0);
  analogWrite(DCMOTORENABLE2, 0);
  digitalWrite(DCMOTORL1, LOW);
  digitalWrite(DCMOTORL2, LOW);
  digitalWrite(DCMOTORL3, LOW);
  digitalWrite(DCMOTORL4, LOW);
}

void dcmotoroff() {
  analogWrite(DCMOTORENABLE1, 0);
  analogWrite(DCMOTORENABLE2, 0);
}

/*
 * turn on dc motor pid
 */
void dcmotorpidon() {
  dcmotorpid1.SetMode(AUTOMATIC);
  dcmotorpid2.SetMode(AUTOMATIC);
}

/*
 * turn off dc motor pid
 */
void dcmotorpidoff() {
  dcmotorpid1.SetMode(MANUAL);
  dcmotorpid2.SetMode(MANUAL);
}

// dc motor control
void dcmotordirection(int motor, int d) {
  if (d == 0){
    return;
  }
  if (motor == 0) {
    if (d == FORWARD) {
      digitalWrite(DCMOTORL1, LOW);
      digitalWrite(DCMOTORL2, HIGH);
    }
    else if (d == BACKWARD) {
      digitalWrite(DCMOTORL1, HIGH);
      digitalWrite(DCMOTORL2, LOW);
    }
  }
  if (motor == 1) {
    if (d == FORWARD) {
      digitalWrite(DCMOTORL3, HIGH);
      digitalWrite(DCMOTORL4, LOW);
    }
    else if (d == BACKWARD) {
      digitalWrite(DCMOTORL3, LOW);
      digitalWrite(DCMOTORL4, HIGH);
    }
  }
}

// ********** LIMIT SWITCH UTILITY FUNCTIONS **********

int readlimit(int limit) {
  int value;
  switch (limit) {
    case 0: value = digitalRead(LIMIT1);
      break;
    case 1: value = digitalRead(LIMIT2);
      break;
    default: value = NULL;
      break;
  }
  return value;
}

// ********** STATE UTILITY FUNCTIONS **********

/*
 * exit state procedure
 * tidies up variables for next state
 */
void exitstate() {
  prevtime = micros();
  dcmotorenc1.write(0);
  dcmotorenc2.write(0);
}
