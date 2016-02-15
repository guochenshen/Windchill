/*
   windchill.ino

   windchill control software
*/

#include <PID_v1.h>
#include <Encoder.h>

#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
#endif

#define SENSOR0 A0
#define SENSOR1 A1
#define SENSOR2 A2
#define SENSOR3 A3
#define SENSOR4 A4
#define SENSOR5 A5

#define PIN0 0 // RX0 & USB TO TTL
#define PIN1 1 // TX0 & USB TO TTL
#define DCMOTORENCODER1CHA 2 // INT0
#define DCMOTORENCODER1CHB 3 // INT1 & PWM
#define DCMOTORENCODER2CHA 4
#define DCMOTORENCODER2CHB 5
#define PIN6 6 // PWM
#define DCMOTORL1 7
#define DCMOTORL2 8
#define DCMOTORENABLE1 9 // PWM
#define DCMOTORENABLE2 10 // PWM
#define DCMOTORL3 11 // PWM
#define DCMOTORL4 12
#define PIN13 13 // LED

#define REVOLUTION 720 // encoder ticks per revolution

#define FORWARD 1
#define BACKWARD -1

void (*state)(void); // current state of the machine
void (*savestate)(void); // saved state for returning to state

// position of the device relative to bottom left corner
double position_x; // [m]
double position_y; // [m]
// maximum coordinates of the system
double max_x; // [m]
double max_y; // [m]

// encoder data objects
Encoder dcmotorenc1(DCMOTORENCODER1CHA, DCMOTORENCODER1CHB);
Encoder dcmotorenc2(DCMOTORENCODER2CHA, DCMOTORENCODER2CHB);
int32_t prevenc1;
int32_t prevenc2;
double prevvel1;
double prevvel2;

// time values
unsigned long prevtime;

// PID values for dc motor 1
double dcmotorsetpoint1;
double dcmotorinput1;
double dcmotoroutput1;
// PID constants dc motor 1
double kp1 = 10.0;
double ki1 = 0.0;
double kd1 = 0.0;
// PID values for dc motor 2
double dcmotorsetpoint2;
double dcmotorinput2;
double dcmotoroutput2;
// PID constants for dc motor 2
double kp2 = 10.0;
double ki2 = 0.0;
double kd2 = 0.0;

PID dcmotorpid1(&dcmotorinput1, &dcmotoroutput1, &dcmotorsetpoint1, kp1, ki1, kd1, DIRECT);
PID dcmotorpid2(&dcmotorinput2, &dcmotoroutput2, &dcmotorsetpoint2, kp2, ki2, kd2, DIRECT);

// initializes the pins
void setup() {
  // sets initial state to on procedure
  state = &on;
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

  // set dc motors to turn off and reset
  dcmotorreset();
  // reset values for next state
  exitstate();
}

// runs function of whichever state the system is currently in
void loop() {
  (*state)();
}

// all systems powered but not performing any actions
// can transition to state of performing actions
void standby() {
  DEBUG_PRINT("standby state");
  // TURNS BOTH DC MOTORS OFF
  analogWrite(DCMOTORENABLE1, 0);
  analogWrite(DCMOTORENABLE2, 0);
  digitalWrite(DCMOTORL1, LOW);
  digitalWrite(DCMOTORL2, LOW);
  digitalWrite(DCMOTORL3, LOW);
  digitalWrite(DCMOTORL4, LOW);
  if (((micros() - prevtime) / 1000000) > 1) {
    state = &right;
    prevtime = micros();
  }
  else {
    return;
  }
}

// turns the device on and performs all checks necessary
void on() {
  state = &standby;
  return;
}

// reads sensors to determine location of the device relative to its environment
void calibrate() {
  return;
}

void forward() {
  DEBUG_PRINT("forward state");
  // turns on PID control 
  dcmotorpidon();

  dcmotordirection(0, 1);
  dcmotordirection(1, 1);
  analogWrite(DCMOTORENABLE1, 96);
  analogWrite(DCMOTORENABLE2, 96);
  
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
  analogWrite(DCMOTORENABLE1, 96);
  analogWrite(DCMOTORENABLE2, 96);
}

void forward_descend() {
  DEBUG_PRINT("Forward Descend State");
  // turns on PID control
  dcmotorpidon();

  dcmotordirection(0, FORWARD);
  dcmotordirection(0, FORWARD);
  analogWrite(DCMOTORENABLE1, 155);
  analogWrite(DCMOTORENABLE2, 96);
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

  dcmotordirection(0, FORWARD);
  dcmotordirection(1, BACKWARD);
  analogWrite(DCMOTORENABLE1, 155);
  analogWrite(DCMOTORENABLE2, 96);
  
  if (abs(dcmotorenc1.read()) > 720) {
    dcmotorreset();
    state = &forward;
    exitstate();
    savestate = NULL;
  }
  else  {
    return;
  }
}

void left() {
  DEBUG_PRINT("Left State");
  // turns on PID control software 
  dcmotorpidon();

  dcmotordirection(0, BACKWARD);
  dcmotordirection(1, FORWARD);
  
  if (true) {
    return;
  }
  else if (false) {
    dcmotorreset();
    exitstate();
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
  analogWrite(DCMOTORENABLE1,155);
  analogWrite(DCMOTORENABLE2, 96);

  // perform short backup and return to previous state
  if (abs(dcmotorenc1.read()) > 720) {
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

