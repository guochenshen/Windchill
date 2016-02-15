//switch

#define SWITCH1 2         // the number of the input pin
#define SWITCH2 3

volatile int state1 = LOW;
volatile int state2 = LOW;
long debounce_time = 15;
volatile long debounce1 = 0;
volatile long debounce2 = 0;

void setup()
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(SWITCH1), intrpt_switch1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SWITCH2), intrpt_switch2, CHANGE);
}

void loop()
{
  Serial.print(state1);
  Serial.print(" , ");
  Serial.println(state2);
}

void intrpt_switch1() {
    if((long)(micros() - debounce1) >= debounce_time *1000) {
     state1 = !state1;
     debounce1 = micros();
    }
}

void intrpt_switch2() {
     if((long)(micros() - debounce2) >= debounce_time *1000) {
     state2 = !state2;
     debounce2 = micros();
    }
}
