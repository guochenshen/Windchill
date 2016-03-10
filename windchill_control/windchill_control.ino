/*
   windchill.ino

   Windchill Control Software
   Hardware: Arduino Mega 2560
*/

#include <PID_v1.h> // PID Library
#include <Encoder.h> // Encoder Library
#include <NewPing.h> // Ultrasonic Distance Sensor Library
#include <SoftwareSerial.h> // Software Serial Library
#include <stdlib.h> // C++ stdlib

// comment out #define DEBUG to disable debug printing
//#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
#endif

// analog input pins
#define SPEED A0 // potentiometer controlling how fast to run the device
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
#define ULTRA1A 4 // PWM - ultrasonic distance 1 trigger
#define ULTRA1B 5 // PWM - ultrasonic distance 1 echo
#define ULTRA2A 6 // PWM - ultrasonic distance 2 trigger
#define ULTRA2B 7 // PWM - utlrasonic distance 2 echo
#define ULTRA3A 8 // PWM - ultrasonic distance 3 triggger
#define ULTRA3B 9 // PWM - ultrasonic distance 3 echo
#define ULTRA4A 10 // PWM - ultrasonic distance 4 trigger
#define ULTRA4B 11 // PWM - ultrasonic distance 4 echo
#define PIN12 12 // PWM
#define PIN13 13 // LED & PWM
#define PIN14 14 // TX3
#define PIN15 15 // RX3
#define PIN16 16 // TX2
#define PIN17 17 // RX2
#define DCMOTORENCODER1CHA 18// TX1 & INT5 - dc motor encoder 1 channel a
#define DCMOTORENCODER1CHB 19 // RX1 & INT4 - dc motor encoder 1 channel b
#define DCMOTORENCODER2CHA 20 // INT3 - dc motor encoder 2 channel a
#define DCMOTORENCODER2CHB 21 // INT2 - dc motor encoder 2 channel b
#define LIMITFRONT1 22 // limit switch front 1
#define LIMITFRONT2 23 // limit switch front 2
#define LIMITLEFT1 24 // limit switch left 1
#define LIMITLEFT2 25 // limit switch left 2
#define LIMITRIGHT1 26 // limit switch right 1
#define LIMITRIGHT2 27 // limit switch right 2
#define LIMITREAR1 28 // limit switch rear 1
#define LIMITREAR2 29 // limit switch rear 2
#define PIN30 30
#define PIN31 31
#define PIN32 32
#define PIN33 33
#define PIN34 34
#define PIN35 35
#define BLUETOOTHRX 36 // bluetooth RX, RX-I pin
#define BLUETOOTHTX 37 // bluetooth TX, TX-O pin
#define PIN38 38
#define PIN39 39
#define DCMOTORL1 40 // dc motor 1 direction 1
#define DCMOTORL2 41 // dc motor 1 direction 2
#define DCMOTORL3 42 // dc motor 2 direction 1
#define DCMOTORL4 43 // dc motor 2 direction 2
#define DCMOTORENABLE1 44 // PWM - dc motor 1 enable pin
#define DCMOTORENABLE2 45 // PWM - dc motor 2 enable pin
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

// device dimensions
#define DEVICE_X 12 // max x dimension of the device in [in]
#define DEVICE_Y 12 // max y dimension of the device in [in]

// device constants
#define DCENCODERREVOLUTION 4741.44 // encoder ticks per revolution
#define WHEELRAD 1 // radius of the wheels in [in]

// sensor constants
#define ULTRAMAXDISTANCE 200 // max distance ofultrasonic in [cm]

// calibration Settings
#define ULTRACALREADINGS 100 // number of readings taken for calibration
#define ULTRACALDELAY 50 // millisecond delay between readings due to hardware limitation

// macros to define dc motors
#define DCLEFT 0
#define DCRIGHT 1

// macros to define motor direction
#define FORWARD 1
#define BACKWARD -1

// macros to define cardination directions
#define FRONT 0
#define LEFT 1
#define RIGHT 2
#define REAR 3

// state machine variables
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

// ultrasonic distance data objects
NewPing frontdistance(ULTRA1A, ULTRA1B, ULTRAMAXDISTANCE);
NewPing leftdistance(ULTRA2A, ULTRA2B, ULTRAMAXDISTANCE);
NewPing rightdistance(ULTRA3A, ULTRA3B, ULTRAMAXDISTANCE);
NewPing reardistance(ULTRA4A, ULTRA4B, ULTRAMAXDISTANCE);

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

// Bluetooth object
SoftwareSerial bluetooth(BLUETOOTHTX, BLUETOOTHRX);

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
  pinMode(LIMITFRONT1, INPUT_PULLUP);
  pinMode(LIMITFRONT2, INPUT_PULLUP);
  pinMode(LIMITRIGHT1, INPUT_PULLUP);
  pinMode(LIMITRIGHT2, INPUT_PULLUP);
  pinMode(LIMITLEFT1, INPUT_PULLUP);
  pinMode(LIMITLEFT2, INPUT_PULLUP);
  pinMode(LIMITREAR2, INPUT_PULLUP);
  pinMode(LIMITREAR2, INPUT_PULLUP);

  // configures bluetooth
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually to enter command mode
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600

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
  // once system is placed onto window, system switches to standy
  if (true) {
    state = &standby;
    exitstate();
  }
}

/* 
 * calibrates the system and determines position
 */
void calibrate() {
  // determine position of the device
  // measured from the center of the device
  unsigned long front = 0;
  unsigned long left = 0;
  unsigned long right = 0;
  unsigned long rear = 0;
  
  for (int i = 0; i < ULTRACALREADINGS; i++) {
    front += frontdistance.ping_in();
    left += leftdistance.ping_in();
    right += rightdistance.ping_in();
    rear += reardistance.ping_in();
    delay(ULTRACALDELAY);
  }

  front /= ULTRACALREADINGS;
  left /= ULTRACALREADINGS;
  right /= ULTRACALREADINGS;
  rear /= ULTRACALREADINGS;

  max_x = front + rear + DEVICE_X;
  max_y = left + right + DEVICE_Y;

  position_x = rear + DEVICE_X;
  position_y = right + DEVICE_Y;

  // once calibration is complete, return to standby state
  if (true) {
    state = &standby;
    exitstate();
  }
  else {
    return;
  }
}

// all systems powered but not performing any actions
// can transition to state of performing actions
void standby() {
  DEBUG_PRINT("Standby State");

  // turns both dc motors off
    dcmotoroff();

  // temporary, check time passage, future will be on button push
  if (((micros() - prevtime) / 1000000) > 1) {
    state = &forward_right;
    exitstate();
  }
  // checks if calibrate button is pushed
  else if (false) {
    state = &calibrate;
  }
  else {
    return;
  }
}

// moves forward traveling right
void forward_right() {
  DEBUG_PRINT("Forward Right State");
  // turns on PID control
  dcmotorpidon();

  dcmotordirection(DCLEFT, FORWARD);
  dcmotordirection(DCRIGHT, FORWARD);
  analogWrite(DCMOTORENABLE1, 255);
  analogWrite(DCMOTORENABLE2, 255);

  if (readlimit(LIMITFRONT1) == LOW) {
    dcmotoroff();
    delay(250);
    dcmotorreset();
    state = &right_uturn;
    exitstate();
  }
  else {
    return;
  }
}

// turn right 90 degrees
void right_uturn() {
  DEBUG_PRINT("Right U-Turn");
  // turns on PID control software
  dcmotorpidon();
//
//  if (savestate != state) {
//    savestate = &right;
//    state = &reverse_turn;
//  }
//  else {
//    savestate = &right;
//  }

  move_distance(-5.5);
  rotate(90);
  move_distance(18.0);
  rotate(90);

  if (true) {
    dcmotoroff();
    delay(250);
    dcmotorreset();
    state = &forward_left;
    exitstate();
  }
  else  {
    return;
  }
}

void left_uturn() {
  DEBUG_PRINT("Right U-Turn");
  // turns on PID control software
  dcmotorpidon();
//
//  if (savestate != state) {
//    savestate = &right;
//    state = &reverse_turn;
//  }
//  else {
//    savestate = &right;
//  }

  move_distance(-5.5);
  rotate(-90);
  move_distance(18.0);
  rotate(-90);

  if (true) {
    dcmotoroff();
    delay(250);
    dcmotorreset();
    state = &forward_right;
    exitstate();
  }
  else  {
    return;
  }
}

void move_distance(double distance) {
  DEBUG_PRINT("Move Distance");
  // turns on PID control software
  dcmotorpidon();

  if (distance > 0) {
    dcmotordirection(DCLEFT, FORWARD);
    dcmotordirection(DCRIGHT, FORWARD);
  }
  else if (distance < 0) {
    dcmotordirection(DCLEFT, BACKWARD);
    dcmotordirection(DCRIGHT, BACKWARD);
  }
  else {
    return;
  }
  
  while (abs(distance * 60) > abs(dcmotorenc1.read())) {
    analogWrite(DCMOTORENABLE1, 255);
    analogWrite(DCMOTORENABLE2, 255);
  }
  exitstate();
}

void rotate(double d) {
  DEBUG_PRINT("Rotate");
    dcmotorpidon();

  if (d > 0) {
    dcmotordirection(DCLEFT, FORWARD);
    dcmotordirection(DCRIGHT, BACKWARD);
  }
  else if (d < 0) {
    dcmotordirection(DCLEFT, BACKWARD);
    dcmotordirection(DCRIGHT, FORWARD);
  }
  else {
    return;
  }
  
  while (abs(d * 9) >  abs(dcmotorenc1.read())) {
    analogWrite(DCMOTORENABLE1, 255);
    analogWrite(DCMOTORENABLE2, 255);
  }
  exitstate();
}

// moves forward descending a wall
void forward_left() {
  DEBUG_PRINT("Forward Left");
  // turns on PID control
  dcmotorpidon();

  dcmotordirection(DCLEFT, FORWARD);
  dcmotordirection(DCRIGHT, FORWARD);
  analogWrite(DCMOTORENABLE1, 255);
  analogWrite(DCMOTORENABLE2, 255);
  
  if (readlimit(LIMITFRONT1) == LOW) {
    dcmotoroff();
    delay(250);
    dcmotorreset();
    state = &left_uturn;
    exitstate();
  }
  else {
    return;
  }
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
    state = &forward_right;
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
    state = &forward_left;
    exitstate();
    savestate = NULL;
  }
  else {
    return;
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
    case LIMITFRONT1: value = digitalRead(LIMITFRONT1);
      break;
    case LIMITFRONT2: value = digitalRead(LIMITFRONT2);
      break;
    case LIMITLEFT1: value = digitalRead(LIMITLEFT1);
      break;
    case LIMITLEFT2: value = digitalRead(LIMITLEFT2);
      break;
    case LIMITRIGHT1: value = digitalRead(LIMITRIGHT1);
      break;
    case LIMITRIGHT2: value = digitalRead(LIMITRIGHT2);
      break;
    case LIMITREAR1: value = digitalRead(LIMITREAR1);
      break;
    case LIMITREAR2: value = digitalRead(LIMITREAR2);
      break;
    default: value = NULL;
      break;
  }
  return value;
}

// ********** ULTRASONIC DISTANCE SENSOR UTILITY FUNCTIONS **********

/*
 * returns the distance read by the sensor
 */
unsigned long distance(int sensor) {
  switch (sensor) {
    case FRONT: return frontdistance.ping_in();
      break;
     case LEFT: return leftdistance.ping_in();
      break;
     case RIGHT: return rightdistance.ping_in();
      break;
     case REAR: return reardistance.ping_in();
      break;
     default: return NULL;
      break;
  }
}

// ********** STATE UTILITY FUNCTIONS **********

/*
 * exit state procedure
 * called when state is changed
 * tidies up variables for next state
 */
void exitstate() {
  // saves the approximate time the state is transitioned
  prevtime = micros();
  // resets the readings on the encoders
  dcmotorenc1.write(0);
  dcmotorenc2.write(0);
}
