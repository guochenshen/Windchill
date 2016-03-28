/*
   windchill_control.ino

   Windchill Control Software
   Hardware: Arduino Mega 2560
*/

#include <PID_v1.h> // PID Library
#include <Encoder.h> // Encoder Library
#include <NewPing.h> // Ultrasonic Distance Sensor Library
#include <SoftwareSerial.h> // Software Serial Library
#include <Wire.h> // Serial Communication Library
#include <Adafruit_Sensor.h> // Adafruit Unified Sensor Library
#include <Adafruit_BNO055.h> // Adafruit BNO055 IMU Library
#include <stdlib.h> // C++ stdlib

#define CODE_VERSION "v0.0.2"

// comment out "#define DEBUG" to disable debug printing
#define DEBUG
#ifdef DEBUG
// switch lines to change output stream
  #define DEBUG_PRINT(x) Serial.println(x)
//#define DEBUG_PRINT(x) bluetooth.println(x)
#else
  #define DEBUG_PRINT(x)
#endif

// comment out "#define LOG" line to disable monitoring the log
#define LOG
#ifdef LOG
  // switch lines to change output stream
  #define LOG(x) Serial.println(x)
//#define LOG(x) bluetooth.println(x)
#else
  #define LOG(x)
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
#define DCMOTORENCODER1CHA 2 // INT0 & PWM
#define DCMOTORENCODER1CHB 3 // INT1 & PWM
#define ULTRA1 4 // PWM - ultrasonic distance 1 trigger
#define ULTRA2 5 // PWM - ultrasonic distance 1 echo
#define ULTRA3 6 // PWM - ultrasonic distance 2 trigger
#define ULTRA4 7 // PWM - utlrasonic distance 2 echo
#define PIN8 8 // PWM - ultrasonic distance 3 triggger
#define PIN9 9 // PWM - ultrasonic distance 3 echo
#define PIN10 10 // PWM - ultrasonic distance 4 trigger
#define PIN11 11 // PWM - ultrasonic distance 4 echo
#define PIN12 12 // PWM
#define PIN13 13 // LED & PWM
#define PIN14 14 // TX3
#define PIN15 15 // RX3
#define PIN16 16 // TX2
#define PIN17 17 // RX2
#define DCMOTORENCODER2CHA 18// TX1 & INT5 - dc motor encoder 1 channel a
#define DCMOTORENCODER2CHB 19 // RX1 & INT4 - dc motor encoder 1 channel b
#define IMU1 20 // INT3 - dc motor encoder 2 channel a
#define IMU2 21 // INT2 - dc motor encoder 2 channel b
#define LIMITLEFT 22 // limit switch front 1
#define LIMITRIGHT 23 // limit switch front 2
#define PIN24 24 // limit switch left 1
#define PIN25 25 // limit switch left 2
#define PIN26 26 // limit switch right 1
#define PIN27 27 // limit switch right 2
#define PIN28 28 // limit switch rear 1
#define PIN29 29 // limit switch rear 2
#define PIN30 30
#define POWER3V 31
#define POWER5V 32
#define POWER12V 33
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

// ******************** CONSTANTS ********************

// device dimension constants
#define DEVICE_X 32 // max x dimension of the device in [cm]
#define DEVICE_Y 32 // max y dimension of the device in [cm]

// component constants
#define DCENCODERREVOLUTION 4741.44 // encoder ticks per revolution
#define WHEELRADIUS 1 // radius of the wheels in [cm]

// sensor constants
#define ULTRAMAXDISTANCE 200 // max distance of ultrasonic in [cm]
#define ULTRADELAY 50 // safe delay of ultrasonic readings

// motor constants
#define DCMOTORLEFT 0
#define DCMOTORRIGHT 1

// motor direction constants
#define FORWARD 1
#define BACKWARD -1

// direction constants
#define FRONT 0
#define LEFT 1
#define RIGHT 2
#define REAR 3

// orientation axis constants
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2


// state machine variables
void (*state)(void); // current state of the machine
void (*savestate)(void); // saved state for returning to a state
boolean same_state;

// maximum coordinates of the system, set during calibration
double max_x; // [m]
double max_y; // [m]

// position of the device relative to bottom left corner
double position_x; // [m]
double position_y; // [m]


// encoder data objects
Encoder dcmotorenc1(DCMOTORENCODER1CHA, DCMOTORENCODER1CHB);
Encoder dcmotorenc2(DCMOTORENCODER2CHA, DCMOTORENCODER2CHB);
int32_t prevenc1; // previous encoder 1 position reading
int32_t prevenc2; // previous encoder 2 position reading
double prevvel1; // previous encoder 1 velocity reading
double prevvel2; // previous encoder 2 velocity reading

// ultrasonic distance data objects
NewPing frontdistance(ULTRA1, ULTRA1, ULTRAMAXDISTANCE);
NewPing leftdistance(ULTRA2, ULTRA2, ULTRAMAXDISTANCE);
NewPing rightdistance(ULTRA3, ULTRA3, ULTRAMAXDISTANCE);
NewPing reardistance(ULTRA4, ULTRA4, ULTRAMAXDISTANCE);

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

// IMU object
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t imu_event;

// initializes the system
void setup() {
  // sets initial state to on state
  state = &on;
  // set save state as null
  savestate = NULL;

  // begin serial
  Serial.begin(9600);

  // configures and starts bluetooth
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually to enter command mode
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600

  // dc motor output pins
  pinMode(DCMOTORENABLE1, OUTPUT);
  pinMode(DCMOTORENABLE2, OUTPUT);
  pinMode(DCMOTORL1, OUTPUT);
  pinMode(DCMOTORL2, OUTPUT);
  pinMode(DCMOTORL3, OUTPUT);
  pinMode(DCMOTORL4, OUTPUT);

  // limit switch input pins
  // HIGH when not triggered, LOW when triggered
  pinMode(LIMITLEFT, INPUT_PULLUP);
  pinMode(LIMITRIGHT, INPUT_PULLUP);

  /* Initialise the sensor */
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    DEBUG_PRINT("No BNO055 Detected");
  }
  else {
    bno.getEvent(&imu_event); // read initial values so event is not null
  }

  // set dc motors to turn off and reset PID
  dcmotorreset();
  // reset values for next state
  exit_state();
}


/*
 * runs the current state of the system
 */
void loop() {
  (*state)();
}


/*
 * on procedure for the device
 * performs all necessary checks for the device to operate correctly
 * prints data desired output stream
 */
void on() {
  LOG("Team I : World Wide Window Washers Windchill");
  LOG(CODE_VERSION);

  // checks if the 3.3V rail is enabled
  if (digitalRead(POWER3V)) {
    LOG("3.3V Power Enabled");
  }
  else {
    LOG("3.3V Power Disaled");
  }

  // checks if the 5V rail is enabled
  if (digitalRead(POWER3V)) {
    LOG("5V Power Enabled");
  }
  else {
    LOG("5V Power Disaled");
  }

  // checks if the 12V rail is enabled
  if (digitalRead(POWER3V)) {
    LOG("12V Power Enabled");
  }
  else {
    LOG("12V Power Disaled");
  }

  calibrate(); // read the position data to determine position on window
  reset_position(); // moves the device to starting position

  // exit condition is true, can be changed to disable turning on if checks fail
  if (true) {
    state = &standby;
    exit_state();
  }
}


// all systems powered but not performing any actions
// can transition to state of performing actions
void standby() {
  // prints state once if state remains the same
  if (!same_state) {
    same_state = true;
    LOG("Enter Standby State");
  }
  // prints at every iteration
  DEBUG_PRINT("Standby State");

  // keeps both dc motors off
  dcmotoroff();

  // temporary, check time passage, future will be on button push
  int sec_delay = 5;
  if (((micros() - prevtime) / 100000) > sec_delay) {
    state = &forward_right;
    exit_state();
  }
}




/*
 * exit state procedure
 * called when state is changed
 * tidies up variables for next state
 */
void exit_state() {
  // saves the approximate time the state is transitioned
  prevtime = micros();
  // resets the readings on the encoders
  dcmotorenc1.write(0);
  dcmotorenc2.write(0);

  same_state = false;
}

/*
 * called when transitioning to interrupting state to safely return to original state
 * call return_state() to safely return to the original function that called the interrupt state
 */
void save_state() {

}

/*
 * called when transitioning back to the original state to safely return to original state
 * call save_state() to transition to original state from the interrupt state
 */
void return_state() {

}


/*
 * calibrates the system and determines position
 */
void calibrate() {
  while (!get_position()) {
    rotate_relative(5);
    delay(100);
  }
}

/*
 * reads the position from the 4 ultrasonic sensors
 * updates global variables upon successful localization
 * returns TRUE upon successful localization
 * returns FALSE upon failed localization
 */
boolean get_position() {
  unsigned long front = frontdistance.ping_cm();
  unsigned long left = leftdistance.ping_cm();
  unsigned long right = rightdistance.ping_cm();
  unsigned long rear = reardistance.ping_cm();

  sensors_event_t imu_event;
  bno.getEvent(&imu_event);

  double bounds = 5; // acceptance range of the device

  // y and z axes not relevant
  double angle = imu_event.orientation.x;

  if (angle > (360 - bounds) || angle < bounds) {
    position_x = front + (DEVICE_X / 2);
    position_y = left + (DEVICE_Y / 2);
    return true;
  }
  else if ((90 - bounds) < angle && angle < (90 + bounds)) {
    position_x = right + + (DEVICE_Y / 2);
    position_y = front + (DEVICE_X / 2);
    return true;
  }
  else if ((180 - bounds) < angle && angle < (180 + bounds)) {
    position_x = rear + (DEVICE_X / 2);
    position_y = right + (DEVICE_Y / 2);
    return true;
  }
  else if ((270 - bounds) < angle && angle < (270 + bounds)) {
    position_x = left + (DEVICE_Y / 2);
    position_y = rear + (DEVICE_X / 2);
    return true;
  }
  else {
    return false;
  }
}

// resets position to the top left corner
// position is the closest it can be while still rotating freely and not contact the wall
// border is added for protection
void reset_position() {
  double border = 2.5; // border in [cm]
  double top_x = max(DEVICE_X, DEVICE_Y) * 1.414;
  double left_y = max(DEVICE_Y, DEVICE_Y) * 1.414;

  double delta_x = position_x - top_x;
  double delta_y = position_y - left_y;




}










void rotate_relative(double d) {
  DEBUG_PRINT("Rotate");
    dcmotorpidon();

  if (d > 0) {
    dcmotordirection(DCMOTORLEFT, FORWARD);
    dcmotordirection(DCMOTORRIGHT, BACKWARD);
  }
  else if (d < 0) {
    dcmotordirection(DCMOTORLEFT, BACKWARD);
    dcmotordirection(DCMOTORRIGHT, FORWARD);
  }
  else {
    return;
  }

  while (abs(d * 9) >  abs(dcmotorenc1.read())) {
    analogWrite(DCMOTORENABLE1, 255);
    analogWrite(DCMOTORENABLE2, 255);
  }
  exit_state();
}



/*
 * absolute rotation
 * rotate the device until the angle of the device matches the rotation value desired
 * selects the shortest path of rotation
 */
void rotate_absolute(double d) {
  DEBUG_PRINT("Rotate Absolute");
  // TODO convert d to string and print with the log

  // sanitize input
  double sanitize_degrees = fmod(d, 360);

  // read current angle position from the IMU
  sensors_event_t imu_event;
  bno.getEvent(&imu_event);

  // TODO determine which axis is relevant depending on the
  double current_angle = orientation(X_AXIS);
}







// moves forward traveling right
void forward_right() {
  DEBUG_PRINT("Forward Right State");
  // turns on PID control
  dcmotorpidon();

  dcmotordirection(DCMOTORLEFT, FORWARD);
  dcmotordirection(DCMOTORRIGHT, FORWARD);
  analogWrite(DCMOTORENABLE1, 255);
  analogWrite(DCMOTORENABLE2, 255);
  DEBUG_PRINT(readlimit(LIMITLEFT));

  if (readlimit(LIMITLEFT) == LOW) {
    dcmotoroff();
    delay(250);
    dcmotorreset();
    state = &right_uturn;
    exit_state();
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
  rotate_relative(90);
  move_distance(18.0);
  rotate_relative(90);

  if (true) {
    dcmotoroff();
    delay(250);
    dcmotorreset();
    state = &forward_left;
    exit_state();
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
  rotate_relative(-90);
  move_distance(18.0);
  rotate_relative(-90);

  if (true) {
    dcmotoroff();
    delay(250);
    dcmotorreset();
    state = &forward_right;
    exit_state();
  }
  else  {
    return;
  }
}

void move_distance(double distance) {
  DEBUG_PRINT("Move Distance");
  // turns on PID control software
  dcmotorreset();
  dcmotorpidon();

  if (distance > 0) {
    dcmotordirection(DCMOTORLEFT, FORWARD);
    dcmotordirection(DCMOTORRIGHT, FORWARD);
  }
  else if (distance < 0) {
    DEBUG_PRINT("x");
    dcmotordirection(DCMOTORLEFT, BACKWARD);
    dcmotordirection(DCMOTORRIGHT, BACKWARD);
  }
  else {
    return;
  }

  while (abs(distance * 60) > abs(dcmotorenc1.read())) {
    analogWrite(DCMOTORENABLE1, 255);
    analogWrite(DCMOTORENABLE2, 255);
  }
  exit_state();
}


// moves forward descending a wall
void forward_left() {
  DEBUG_PRINT("Forward Left");
  // turns on PID control
  dcmotorpidon();

  dcmotordirection(DCMOTORLEFT, FORWARD);
  dcmotordirection(DCMOTORRIGHT, FORWARD);
  analogWrite(DCMOTORENABLE1, 255);
  analogWrite(DCMOTORENABLE2, 255);

  if (readlimit(LIMITLEFT) == LOW) {
    dcmotoroff();
    delay(250);
    dcmotorreset();
    state = &left_uturn;
    exit_state();
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

  dcmotordirection(DCMOTORLEFT, FORWARD);
  dcmotordirection(DCMOTORRIGHT, BACKWARD);
  analogWrite(DCMOTORENABLE1, 255);
  analogWrite(DCMOTORENABLE2, 255);

  if (abs(dcmotorenc1.read()) > 720) {
    dcmotoroff();
    delay(500);
    dcmotorreset();
    state = &forward_right;
    exit_state();
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
    exit_state();
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
    exit_state();
  }
  else {
    return;
  }
}

// ******************** IMU UTILITY FUNCTIONS ********************


/*
 * return the orientation read by the IMU
 */
float orientation(int axis) {
  DEBUG_PRINT("Orientation");

  // retrieves the data from the IMU
  bno.getEvent(&imu_event);

  // return the orientation of the axis desired
  switch (axis) {
    case X_AXIS: return imu_event.orientation.x;
    case Y_AXIS: return imu_event.orientation.y;
    case Z_AXIS: return imu_event.orientation.z;
    default: return 0;
  }
}

// ******************** MOTOR UTILITY FUNCTIONS ********************

/*
 * turns the dc motor enable pins off
 * write analog to pins with PWM to control speed
 */
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


/*
 * reset dc motors and PID
 */
void dcmotorreset() {
  // turns off PID control and resets it
  dcmotorpid1.SetMode(MANUAL);
  dcmotorpid2.SetMode(MANUAL);
  // disable motors
  dcmotoroff();
  // reset directions
  // both pins low is brake state for the motors
  digitalWrite(DCMOTORL1, LOW);
  digitalWrite(DCMOTORL2, LOW);
  digitalWrite(DCMOTORL3, LOW);
  digitalWrite(DCMOTORL4, LOW);

  // reset the encoder values to 0
  dcmotorenc1.write(0);
  dcmotorenc2.write(0);
}


/*
 * dc motor control for direction
 * changes the direction settings for the motors
 * acounts for mirrored mounting of motors so forwards is the same for both motors
 *
 * input parameters are the motor selection and the direction selection
 *
 * motor selection (motor parameter) is integer enumeration of motor
 * 0 or DCMOTORLEFT macro for left motor
 * 1 or DCMOTORRIGHT macro for right motor
 *
 * direction selection (d parameter) is integer enumeration of direction
 * 1 or FORWARD macro for forward
 * -1 or BACKWARD macro for backwards
 */
void dcmotordirection(int motor, int d) {
  // invalid direction setting
  if (d == 0){
    return; // don't change motor settings
  }

  // change settings for left motor
  if  (motor == DCMOTORLEFT) {
    if (d == FORWARD) {
      digitalWrite(DCMOTORL1, LOW);
      digitalWrite(DCMOTORL2, HIGH);
    }
    else if (d == BACKWARD) {
      digitalWrite(DCMOTORL1, HIGH);
      digitalWrite(DCMOTORL2, LOW);
    }
  }
  // change settings for right motor
  else if (motor == DCMOTORRIGHT) {
    if (d == FORWARD) {
      digitalWrite(DCMOTORL3, HIGH);
      digitalWrite(DCMOTORL4, LOW);
    }
    else if (d == BACKWARD) {
      digitalWrite(DCMOTORL3, LOW);
      digitalWrite(DCMOTORL4, HIGH);
    }
  }
  // no valid motor selected
  else {
    return;
  }
}

// ******************** LIMIT SWITCH FUNCTION ********************

/*
 * returns the state of the limit switch, HIGH if untripped, LOW if tripped
 * input parameter is integer enumeration of the limit switch pin
 * LOW is tripped to make wiring simpler due to existence of INPUT_PULLUP
 * tripped limit switch pulls pin to GROUND
 *
 * front of device has two limit switches
 * 22 is left or LIMITLEFT macro
 * 23 is right or LIMITRIGHT macro
 *
 * return 0 (equivalent to LOW) if limit switch is tripped
 * return 1 (equivalent to HIGH) if limit switch is untripped
 * return -1 on error
 */

int readlimit(int limit) {
  // stores the value read from the pin
  int value;

  switch (limit) {
    case LIMITLEFT: value = digitalRead(LIMITLEFT); // reads the pin
      break;
    case LIMITRIGHT: value = digitalRead(LIMITRIGHT); // reads the pin
      break;
    default: value = -1; // sets the value as error value
      break;
  }

  return value;
}

// ******************** ULTRASONIC DISTANCE FUNCTION ********************

/*
 * returns the distance read by the sensor in centimeters
 * ignores NewPing library for unit conversion due to limitations in precision
 * uses same macros as the system for conversion to achieve
 *
 * input parameter is an integer enumeration of the directions
 * 0 is front or FRONT macro
 * 1 is left or LEFT macro
 * 2 is right or RIGHT macro
 * 3 is rear or REAR macro
 *
 */
double ultrasonic_distance(int sensor) {
  unsigned int time_delay; // stores the values read from the ultrasonic distance sensor

  switch (sensor) {
    case FRONT:
      time_delay =  frontdistance.ping(); // reads the raw microsecond time delay
      break;
    case LEFT:
      time_delay = leftdistance.ping(); // reads the raw microsecond time delay
      break;
    case RIGHT:
      time_delay =  rightdistance.ping(); // reads the raw microsecond time delay
      break;
    case REAR:
      time_delay =  reardistance.ping(); // reads the raw microsecond time delay
      break;
    default: return 0.0; // no valid ultrasonic sensor specified, return 0
  }

  // casting to float for float division
  float time_delay_float = (float) time_delay;
  // convert time delay to actual distance from target
  float microseconds_to_centimeters = 57.0;

  // return the distance read by the ultrasonic sensor
  return time_delay_float / microseconds_to_centimeters;
}
