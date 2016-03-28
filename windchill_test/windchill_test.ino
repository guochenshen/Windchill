/*
   windchill_test.ino

   Windchill Testing Harness
   Manual Testing on Hardware to determine faults, if any
   Testing results conducted with Serial Outputs and LEDs
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
#include <string.h> // string library


#define CODE_VERSION "v0.0.2"

// comment out #define DEBUG to disable debug printing
#define DEBUG
#ifdef DEBUG
// switch lines to change serial printing to bluetooth printing
  #define DEBUG_PRINT(x) Serial.println(x)
//#define DEBUG_PRINT(x) bluetooth.println(x)
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

// device dimensions
#define DEVICE_X 32 // max x dimension of the device in [cm]
#define DEVICE_Y 32 // max y dimension of the device in [cm]

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

// defines orientation axes
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

// state machine variables
void (*state)(void); // current state of the machine
void (*savestate)(void); // saved state for returning to a state
boolean same_state;
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
sensors_event_t event;

bno.getEvent(&event);


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

  // set dc motors to turn off and reset PID
  dcmotorreset();
  // reset values for next state
  exit_state();
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
 * runs the current state of the system
 */
void loop() {
  (*state)();
}
