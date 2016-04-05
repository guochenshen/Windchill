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
#include <math.h> // math functions

#define CODE_VERSION "v0.0.2"

// comment out "#define DEBUG" to disable debug printing
//#define DEBUG
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
  //#define LOG(x) Serial.println(x)
  #define LOG(x) bluetooth.println(x)
#else
  #define LOG(x)
#endif

// analog input pins
#define SPEED A0 // potentiometer determining the speed
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
#define MOTOR_LEFT_ENCODER_CHA 2 // INT0 & PWM
#define MOTOR_LEFT_ENCODER_CHB 3 // INT1 & PWM
#define ULTRA_FRONT 4 // PWM - ultrasonic distance front
#define ULTRA_LEFT 5 // PWM - ultrasonic distance left
#define ULTRA_RIGHT 6 // PWM - ultrasonic distance right
#define ULTRA_REAR 7 // PWM - utlrasonic distance rear
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
#define MOTOR_RIGHT_ENCODER_CHA 18// TX1 & INT5 - dc motor encoder 1 channel a
#define MOTOR_RIGHT_ENCODER_CHB 19 // RX1 & INT4 - dc motor encoder 1 channel b
#define IMU1 20 // INT3 - dc motor encoder 2 channel a
#define IMU2 21 // INT2 - dc motor encoder 2 channel b
#define LIMIT_LEFT 22 // limit switch front 1
#define LIMIT_RIGHT 23 // limit switch front 2
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
#define MOTOR_LEFT1 40 // dc motor 1 direction 1
#define MOTOR_RIGHT1 41 // dc motor 2 direction 1
#define MOTOR_LEFT2 42 // dc motor 1 direction 2
#define MOTOR_RIGHT2 43 // dc motor 2 direction 2
#define MOTOR_ENABLE_LEFT 44 // PWM - dc motor 1 enable pin
#define MOTOR_ENABLE_RIGHT 45 // PWM - dc motor 2 enable pin
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
#define DEVICE_X 31 // max x dimension of the device in [cm]
#define DEVICE_Y 31 // max y dimension of the device in [cm]
#define WHEEL_RADIUS 3 // wheel radius in [cm]

// component constants
#define DCENCODERREVOLUTION 4741.44 // encoder ticks per revolution
#define WHEELRADIUS 1 // radius of the wheels in [cm]

// sensor constants
#define ULTRAMAXDISTANCE 200 // max distance of ultrasonic in [cm]
#define ULTRADELAY 50 // safe delay of ultrasonic readings

// motor constants
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

// motor direction constants
#define FORWARD 1
#define BACKWARD -1

// direction constants
#define FRONT 0
#define LEFT 1
#define RIGHT 2
#define REAR 3

// orientation axis constants
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2


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
Encoder dcmotorencleft(MOTOR_LEFT_ENCODER_CHA, MOTOR_LEFT_ENCODER_CHB);
Encoder dcmotorencright(MOTOR_RIGHT_ENCODER_CHA, MOTOR_RIGHT_ENCODER_CHB);

// ultrasonic distance data objects
NewPing frontdistance(ULTRA_FRONT, ULTRA_FRONT, ULTRAMAXDISTANCE);
NewPing leftdistance(ULTRA_LEFT, ULTRA_LEFT, ULTRAMAXDISTANCE);
NewPing rightdistance(ULTRA_RIGHT, ULTRA_RIGHT, ULTRAMAXDISTANCE);
NewPing reardistance(ULTRA_REAR, ULTRA_REAR, ULTRAMAXDISTANCE);

// time values
unsigned long prevtime; // previous time, used for velocity calculations

// PID values for dc motor 1
double dcsetpointleft; // setpoint
double dcinputleft; // input value
double dcoutputleft; // output PWM value
// PID constants dc motor 1
double kpleft = 10.0;
double kileft = 0.0;
double kdleft = 0.0;
// PID values for dc motor 2
double dcsetpointright; // setpoint
double dcinputright; // input value
double dcoutputright; // output PWM value
// PID constants for dc motor 2
double kpright = 10.0;
double kiright = 0.0;
double kdright = 0.0;

// PID objects
PID dcmotorpid1(&dcinputleft, &dcoutputleft, &dcsetpointleft, kpleft, kileft, kdleft, DIRECT);
PID dcmotorpid2(&dcinputright, &dcoutputright, &dcsetpointright, kpright, kiright, kdright, DIRECT);

// Bluetooth object
SoftwareSerial bluetooth(BLUETOOTHTX, BLUETOOTHRX);

// IMU object
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t imu_event;






/*
 * Initializes the system and all of its pins
 */
void setup() {
  // sets initial state to on state
  // on state will perform initial checks
  state = &on;
  // set save state as null
  savestate = NULL;

  // begin serial at 9600 baud
  Serial.begin(9600);

  // configures and starts bluetooth
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200 baudrate
  bluetooth.print("$");  // Print three times individually to enter command mode
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  bluetooth.begin(9600);  // Start bluetooth serial at 9600

  // dc motor output pins
  // enable pins for the dc motors, PWM output to control
  pinMode(MOTOR_ENABLE_LEFT, OUTPUT);
  pinMode(MOTOR_ENABLE_RIGHT, OUTPUT);
  // direction pins for the dc motors
  pinMode(MOTOR_LEFT1, OUTPUT);
  pinMode(MOTOR_LEFT2, OUTPUT);
  pinMode(MOTOR_RIGHT1, OUTPUT);
  pinMode(MOTOR_RIGHT2, OUTPUT);

  // limit switch input pins
  // HIGH when not triggered, LOW when triggered
  // INPUT_PULLUP to ensure HIGH when not triggered
  pinMode(LIMIT_LEFT, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT, INPUT_PULLUP);

  // sets maximum speed of the system
  pinMode(SPEED, INPUT);

  // initialize the IMU
  bno.begin();
  bno.getEvent(&imu_event);

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
 * exit state procedure
 * called when state is changed
 * tidies up variables for next state
 */
void exit_state() {
  // set dc motors to turn off and reset PID
  motor_reset();
  // saves the approximate time the state is transitioned
  prevtime = micros();
  // resets the readings on the encoders
  dcmotorencleft.write(0);
  dcmotorencright.write(0);

  same_state = false;
}

/*
 * on procedure for the device
 * performs all necessary checks for the device to operate correctly
 * prints data to desired output stream
 */
void on() {
  LOG("Team I : World Wide Window Washers");
  LOG("Windchill");
  LOG(CODE_VERSION);
  LOG("");

  LOG("System Check:");
  // checks if power rails are enabled
  (digitalRead(POWER3V)) ? LOG("3.3V Power Enabled") : LOG("3.3V Power Disabled");
  (digitalRead(POWER5V)) ? LOG("5V Power Enabled") : LOG("5V Power Disabled");
  (digitalRead(POWER12V)) ? LOG("12V Power Enabled") : LOG("12V Power Disabled");
  LOG("");

  LOG("Calibrating...");
  calibrate(); // read the position data to determine position on window
  LOG("Position:");
  LOG(position_x);
  LOG(position_y);
  LOG("Window Size:");
  LOG(max_x);
  LOG(max_y);
  LOG("Calibrated");

  // exit condition is true, can be changed to disable turning on if checks fail
  if (true) {
    LOG("Completed on procedure, exiting state");
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
  motor_off();

  // wait for 'b' from serial to begin
  if (bluetooth.available()) {
    char c = bluetooth.read();
    if (c == 'r') {
      LOG("Resetting Position...");
      reset_position();
      LOG("Position Reset");
    }
    if (c == 'b') {
      state = &forward_right;
      exit_state();
      LOG("Beginning");
    }
  }
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
    rotate_relative(1);
    delay(50);
  }
}


/*
 * reads the position from the 4 ultrasonic sensors
 * updates global variables upon successful localization
 * returns TRUE upon successful localization
 * returns FALSE upon failed localization
 */
boolean get_position() {
  LOG("");
  LOG("Getting Position");
  LOG("Front:");
  unsigned long front = frontdistance.ping_cm();
  LOG(front);
  LOG("Left:");
  unsigned long left = leftdistance.ping_cm();
  LOG(left);
  LOG("Right:");
  unsigned long right = rightdistance.ping_cm();
  LOG(right);
  LOG("Rear:");
  unsigned long rear = reardistance.ping_cm();
  LOG(rear);

  double bounds = 3; // acceptance range of the device

  double angle = orientation(XAXIS);

  LOG("Angle:");
  LOG(angle);


  if (angle > (360 - bounds) || angle < bounds) {
    position_x = left + (DEVICE_X / 2);
    position_y = front + (DEVICE_Y / 2);
    max_x = left + right + DEVICE_X;
    max_y = front + rear + DEVICE_Y;
    return true;
  }
  else if ((90 - bounds) < angle && angle < (90 + bounds)) {
    position_x = rear + (DEVICE_Y / 2);
    position_y = left + (DEVICE_X / 2);
    max_x = front + rear + DEVICE_Y;
    max_y = left + right + DEVICE_X;
    return true;
  }
  else if ((180 - bounds) < angle && angle < (180 + bounds)) {
    position_x = right + (DEVICE_X / 2);
    position_y = rear + (DEVICE_Y / 2);
    max_x = left + right + DEVICE_X;
    max_y = front + rear + DEVICE_Y;
    return true;
  }
  else if ((270 - bounds) < angle && angle < (270 + bounds)) {
    position_x = front + (DEVICE_Y / 2);
    position_y = right + (DEVICE_X / 2);
    max_x = front + rear + DEVICE_Y;
    max_y = left + right + DEVICE_X;
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
  double sqrt2 = 1.5; // square root of 2, overestimated to avoid contact with walls
  double top_x = (max(DEVICE_X, DEVICE_Y) * sqrt2) + border;
  double left_y = (max(DEVICE_Y, DEVICE_Y) * sqrt2) + border;

  move_absolute(top_x, left_y); // moves the device to the top left corner
  rotate_absolute(0); // rotates the devices to 0 degrees
}




/*
 * moves the device a certain distance in the direction that its facing
 * every motion function in the system is based off of this function
 */
void move_distance(double d) {
  double revolution = (d / (PI * WHEEL_RADIUS));
  int encoder = (int) revolution * DCENCODERREVOLUTION;

  motor_reset();
  motor_pid_on();
  dcmotorpid1.SetOutputLimits(0, get_max_speed());
  dcmotorpid2.SetOutputLimits(0, get_max_speed());

  dcsetpointleft = abs(encoder);
  dcsetpointright = abs(encoder);

  while (dcsetpointleft < encoder && dcsetpointright < encoder) {
    dcinputleft = abs(dcmotorencleft.read());
    dcinputright = abs(dcmotorencright.read());
    dcmotorpid1.Compute();
    dcmotorpid2.Compute();

    motor_speed(MOTOR_LEFT, dcoutputleft);
    motor_speed(MOTOR_RIGHT, dcoutputright);
  }
}

/*
 * moves a relative position from where it currently is
 * will not move past where it cannot move to
 */
void move_relative(double x, double y) {
  LOG("Move Relative Distance");
  // turns on PID control software
  motor_reset();
  motor_pid_on();

  double angle = atan2(y, x);
  double distance = sqrt(sq(x) + sq(y));

  rotate_absolute(angle);

  dcmotordirection(MOTOR_LEFT, FORWARD);
  dcmotordirection(MOTOR_RIGHT, FORWARD);

  while (abs(distance * 60) > abs(dcmotorencleft.read())) {
    motor_speed(MOTOR_LEFT, 255);
    motor_speed(MOTOR_RIGHT, 255);
  }
  exit_state();
}


/*
 * move to an absolute position on the surface
 * will not move if desired position is off the surface
 * requires that the position value is accurate
 */
void move_absolute(double x, double y) {
  LOG("Move Absolute Distance");

  motor_reset();
  motor_pid_on();

  double x_delta = x - position_x;
  double y_delta = y - position_y;
  double angle = atan2(y_delta, x_delta);
  LOG("Distance:");
  double distance = sqrt(sq(x_delta) + sq(y_delta));
  LOG(distance);

  LOG("Angle:");
  LOG(angle);
  rotate_absolute(angle);

  dcmotordirection(MOTOR_LEFT, FORWARD);
  dcmotordirection(MOTOR_RIGHT, FORWARD);

  while (abs(distance * 60) > abs(dcmotorencleft.read())) {
    motor_speed(MOTOR_LEFT, 255);
    motor_speed(MOTOR_RIGHT, 255);
  }
  exit_state();
}


void rotate_relative(double d) {
  DEBUG_PRINT("Rotate");
    motor_pid_on();

  if (d > 0) {
    dcmotordirection(MOTOR_LEFT, FORWARD);
    dcmotordirection(MOTOR_RIGHT, BACKWARD);
  }
  else if (d < 0) {
    dcmotordirection(MOTOR_LEFT, BACKWARD);
    dcmotordirection(MOTOR_RIGHT, FORWARD);
  }
  else {
    return;
  }

  while (abs(d * 9) >  abs(dcmotorencleft.read())) {
    motor_speed(MOTOR_LEFT, 255);
    motor_speed(MOTOR_RIGHT, 255);
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

  // TODO determine which axis is relevant depending on the
  double current_angle = orientation(XAXIS);

  double turn_angle = sanitize_degrees - current_angle;

  while (abs(turn_angle * 9) > abs(dcmotorencleft.read())) {
    dcmotordirection(MOTOR_LEFT, FORWARD);
    dcmotordirection(MOTOR_RIGHT, BACKWARD);

    motor_speed(MOTOR_LEFT, 255);
    motor_speed(MOTOR_RIGHT, 255);
  }
}









/*
 * forward motion state for one direction
 * moves forward until the the limit switch is reached
 */
void forward_right() {
  // prints state once if state remains the same
  if (!same_state) {
    same_state = true;
    LOG("Enter Forward Right State");
  }
  // prints at every iteration
  DEBUG_PRINT("Forward Right State");

  // travel forwards
  dcmotordirection(MOTOR_LEFT, FORWARD);
  dcmotordirection(MOTOR_RIGHT, FORWARD);
  motor_speed(MOTOR_LEFT, 255);
  motor_speed(MOTOR_RIGHT, 255);

  if ((readlimit(LIMIT_LEFT) == LOW) && (readlimit(LIMIT_RIGHT) == LOW)) {
    motor_off();
    motor_reset();
    state = &right_uturn;
    exit_state();

    delay(100);
  }
  else if ((readlimit(LIMIT_LEFT) == LOW) && (readlimit(LIMIT_RIGHT) == HIGH)) {
    correct_limit(LIMIT_RIGHT);
  }
  else if ((readlimit(LIMIT_LEFT) == HIGH) && (readlimit(LIMIT_RIGHT) == LOW)) {
    correct_limit(LIMIT_LEFT);
  }
  else {
    return;
  }
}

/*
 * correct the position of the device against the wall
 */
void correct_limit(int limit) {
  if (limit == LIMIT_RIGHT) {
    while (readlimit(LIMIT_RIGHT) == HIGH) {
      analogWrite(MOTOR_ENABLE_LEFT, 0);
      motor_speed(MOTOR_RIGHT, 255);
    }
  }
  else if (limit == LIMIT_LEFT) {
    while (readlimit(LIMIT_LEFT) == HIGH) {
      motor_speed(MOTOR_LEFT, 255);
      analogWrite(MOTOR_ENABLE_RIGHT, 0);
    }
  }
}


/*
 * forward motion state for opposite direction to forward_right()
 */
void forward_left() {
  // prints state once if state remains the same
  if (!same_state) {
    same_state = true;
    LOG("Enter Forward Left State");
  }
  // prints at every iteration
  DEBUG_PRINT("Forward Left State");

  // travel forwards
  dcmotordirection(MOTOR_LEFT, FORWARD);
  dcmotordirection(MOTOR_RIGHT, FORWARD);
  motor_speed(MOTOR_LEFT, 255);
  motor_speed(MOTOR_RIGHT, 255);

  if ((readlimit(LIMIT_LEFT) == LOW) && (readlimit(LIMIT_RIGHT) == LOW)) {
    motor_off();
    motor_reset();
    state = &left_uturn;
    exit_state();

    delay(100);
  }
  else if ((readlimit(LIMIT_LEFT) == LOW) && (readlimit(LIMIT_RIGHT) == HIGH)) {
    correct_limit(LIMIT_RIGHT);
  }
  else if ((readlimit(LIMIT_LEFT) == HIGH) && (readlimit(LIMIT_RIGHT) == LOW)) {
    correct_limit(LIMIT_LEFT);
  }
  else {
    return;
  }
}
















// turn right 90 degrees
void right_uturn() {
  DEBUG_PRINT("Right U-Turn");
  // turns on PID control software
  motor_pid_on();
//
//  if (savestate != state) {
//    savestate = &right;
//    state = &reverse_turn;
//  }
//  else {
//    savestate = &right;
//  }

  move_relative(0, -5.5);
  rotate_relative(90);
  move_relative(0, 18.0);
  rotate_relative(90);

  if (true) {
    motor_off();
    delay(250);
    motor_reset();
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
  motor_pid_on();
//
//  if (savestate != state) {
//    savestate = &right;
//    state = &reverse_turn;
//  }
//  else {
//    savestate = &right;
//  }

  move_relative(0, -5.5);
  rotate_relative(-90);
  move_relative(18.0, 0);
  rotate_relative(-90);

  if (true) {
    motor_off();
    delay(250);
    motor_reset();
    state = &forward_right;
    exit_state();
  }
  else  {
    return;
  }
}



//// moves forward descending a wall
//void forward_left() {
//  DEBUG_PRINT("Forward Left");
//  // turns on PID control
//  motor_pid_on();
//
//  dcmotordirection(MOTOR_LEFT, FORWARD);
//  dcmotordirection(MOTOR_RIGHT, FORWARD);
//  motor_speed(MOTOR_LEFT, 255);
//  motor_speed(MOTOR_RIGHT, 255);
//
//  if (readlimit(LIMIT_LEFT) == LOW) {
//    motor_off();
//    delay(250);
//    motor_reset();
//    state = &left_uturn;
//    exit_state();
//  }
//  else {
//    return;
//  }
//}

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
  motor_pid_on();

  dcmotordirection(MOTOR_LEFT, FORWARD);
  dcmotordirection(MOTOR_RIGHT, BACKWARD);
  motor_speed(MOTOR_LEFT, 255);
  motor_speed(MOTOR_RIGHT, 255);

  if (abs(dcmotorencleft.read()) > 720) {
    motor_off();
    delay(500);
    motor_reset();
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
  motor_pid_on();

  dcmotordirection(0, BACKWARD);
  dcmotordirection(1, FORWARD);
  motor_speed(MOTOR_LEFT, 255);
  motor_speed(MOTOR_RIGHT, 255);

  if (abs(dcmotorencleft.read()) > 90) {
    motor_off();
    delay(500);
    motor_reset();
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
  motor_pid_on();

  dcmotordirection(0, BACKWARD);
  dcmotordirection(1, BACKWARD);
  motor_speed(MOTOR_LEFT, 255);
  motor_speed(MOTOR_RIGHT, 255);

  // perform short backup and return to previous state
  if (abs(dcmotorencleft.read()) > 360) {
    motor_off();
    delay(500);
    state = savestate;
    motor_reset();
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
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // return the orientation of the axis desired
  switch (axis) {
    case XAXIS: return euler.x();
    case YAXIS: return euler.y();
    case ZAXIS: return euler.z();
    default: return 0;
  }
}

// ****************************** MOTOR UTILITY FUNCTIONS ******************************

/*
 * Turns off the DC motors
 * writes 0 to enable pins to stop motion
 */
void motor_off() {
  analogWrite(MOTOR_ENABLE_LEFT, 0);
  analogWrite(MOTOR_ENABLE_RIGHT, 0);
}

/*
 * turn on dc motor pid
 */
void motor_pid_on() {
  dcmotorpid1.SetMode(AUTOMATIC);
  dcmotorpid2.SetMode(AUTOMATIC);
}

/*
 * turn off dc motor pid
 * turned off to reset the pid
 */
void motor_pid_off() {
  dcmotorpid1.SetMode(MANUAL);
  dcmotorpid2.SetMode(MANUAL);
}

/*
 * reset dc motors and PID
 */
void motor_reset() {
  // turns off PID control and resets it
  motor_pid_off();

  // disable motors
  motor_off();

  // reset directions
  // both pins low is brake state for the motors
  digitalWrite(MOTOR_LEFT1, LOW);
  digitalWrite(MOTOR_LEFT2, LOW);
  digitalWrite(MOTOR_RIGHT1, LOW);
  digitalWrite(MOTOR_RIGHT2, LOW);

  // reset the encoder values to 0
  dcmotorencleft.write(0);
  dcmotorencright.write(0);
}

/*
 * sets the speed of the motors, capping the maximum PWM output to that
 * of the speed knob
 */
void motor_speed(int motor, int s) {
  int max_speed = analogRead(SPEED); // read 0-1023 from the potentiometer
  int output_pwm; // pwm output to the motors

  if (s <= 0) { // prevent speeds less than 0
    output_pwm = 0;
  }
  else if (s >= max_speed) { // cap speeds greater than maximum speed
    output_pwm = max_speed;
  }
  else { // set speed if within range
    output_pwm = s;
  }

  // send output pwm to left motor
  if (motor == MOTOR_LEFT) {
    analogWrite(MOTOR_ENABLE_LEFT, output_pwm);
  }
  // send output pwm to right motor
  else if (motor == MOTOR_RIGHT) {
    analogWrite(MOTOR_ENABLE_RIGHT, output_pwm);
  }
}

/*
 * returns maximum speed as allowed by speed knob
 * returns a value between 0 and 255 for PWM
 */
int get_max_speed() {
  return map(analogRead(SPEED), 0, 1023, 0, 255);
}


/*
 * dc motor control for direction
 * changes the direction settings for the motors
 * acounts for mirrored mounting of motors so forwards is the same for both motors
 *
 * input parameters are the motor selection and the direction selection
 *
 * motor selection (motor parameter) is integer enumeration of motor
 * 0 or MOTOR_LEFT macro for left motor
 * 1 or MOTOR_RIGHT macro for right motor
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
  if  (motor == MOTOR_LEFT) {
    if (d == FORWARD) {
      digitalWrite(MOTOR_LEFT1, HIGH);
      digitalWrite(MOTOR_LEFT2, LOW);
    }
    else if (d == BACKWARD) {
      digitalWrite(MOTOR_LEFT1, LOW);
      digitalWrite(MOTOR_LEFT2, HIGH);
    }
  }
  // change settings for right motor
  else if (motor == MOTOR_RIGHT) {
    if (d == FORWARD) {
      digitalWrite(MOTOR_RIGHT1, LOW);
      digitalWrite(MOTOR_RIGHT2, HIGH);
    }
    else if (d == BACKWARD) {
      digitalWrite(MOTOR_RIGHT1, HIGH);
      digitalWrite(MOTOR_RIGHT2, LOW);
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
 * 22 is left or LIMIT_LEFT macro
 * 23 is right or LIMIT_RIGHT macro
 *
 * return 0 (equivalent to LOW) if limit switch is tripped
 * return 1 (equivalent to HIGH) if limit switch is untripped
 * return -1 on error
 */

int readlimit(int limit) {
  // stores the value read from the pin
  int value;

  switch (limit) {
    case LIMIT_LEFT: value = digitalRead(LIMIT_LEFT); // reads the pin
      break;
    case LIMIT_RIGHT: value = digitalRead(LIMIT_RIGHT); // reads the pin
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
