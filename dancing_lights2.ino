#include <rotary.h>

#define HEART 1
//#define BRAIN 1

#include <WriteLight.h>
#include <Metro.h>
#include <SoftwareSerial.h>

#ifdef BRAIN
  #define MODE (LIGHT_MODE_LOCAL_WRITE | LIGHT_MODE_SERIAL_DEBUG | LIGHT_MODE_SERIAL_READ)
  #define FAKE false
  #define BUTTON 31
//  #define SERIAL Serial1
#else //
  #define MODE (LIGHT_MODE_SERIAL_DEBUG|  LIGHT_MODE_SERIAL_WRITE | LIGHT_MODE_ANIMATE)
  #define FAKE false
  #define BUTTON A0
//  #define SERIAL Serial2
#endif

WriteLight light1(LIGHT_UNIT_1);
WriteLight light2(LIGHT_UNIT_2);
WriteLight light3(LIGHT_UNIT_3);
WriteLight light4(LIGHT_UNIT_4);
WriteLight lights(LIGHT_UNIT_ALL);

// LED Intensities
//int rInt              = 255;
//int gInt              = 255;
//int bInt              = 255;


const int INPUT_MAX(500);
const int RANGE_MAX(255);
// Rotary Input
// the order selects the pin assignment
#ifdef HEART
Rotary redRotary(INPUT_MAX, RANGE_MAX, FAKE);    // pins 2,3   (A,B)  A is yellow, B is purple
Rotary greenRotary(INPUT_MAX, RANGE_MAX, FAKE);  // pins 21,20 (A,B)
Rotary blueRotary(INPUT_MAX, RANGE_MAX, FAKE);   // pins 19,18 (A,B)
#else
class rotary {
  public:
  int value(void){};
  void loop(){};
  void setup(rotary&){};
};
rotary redRotary;
rotary greenRotary;
rotary blueRotary;
#endif
// black ground
// gray channel A
// red vcc
// blue channel B


// Pot Inputs
//int rInp(A0);
//int gInp(A1);
//int bInp(A2);

// Mic Inputs
//int rMicInp(A0);
//int gMicInp(A1);
//int bMicInp(A2);

// Array Address Constants
//   Color:
//const int red         = 0;
//const int green       = 1;
//const int blue        = 2;

enum ANIMATION_STATE {
  ANIMATION_START=0,
  ANIMATION_GLOW=0,
  ANIMATION_PULSE,
  ANIMATION_ON,
  ANIMATION_BLINK,
  ANIMATION_POP,
//  ANIMATION_LISTEN,
//  ANIMATION_CYCLE,
  ANIMATION_RANDOM,
  ANIMATION_COP,
  ANIMATION_OFF,
  ANIMATION_END
} g_state(ANIMATION_GLOW);

// animation functions
void glow();
void blink();
void pulse();
void pulse();
void pop_lights();
void listen();
void cycle();
void random_lights();
void lights_off();
void cop();
void lights_on();


// Button Variables (including debouncing)
int  button_pin(BUTTON); 
bool button_pressed(false);
bool debouncing(false);
unsigned long debounce_time;
const unsigned long debounce_timeout(50);
void debounce();
void handleButton();
void reportState(void);

void debounce() {
  int button = analogRead(button_pin);
//  Serial.println(button,DEC);
  bool current_state(button > 500);
  
  if(debouncing) {
    if(millis() > debounce_time + debounce_timeout) {
      debouncing = false;
//      Serial.println(current_state);
      if(current_state != button_pressed) {
        button_pressed = current_state;
//        Serial.print("DEBOUNCED: ");
        if(button_pressed) {
//          Serial.println("PRESSED");
          handleButton();
        } else {
//          Serial.println("RELEASED");
          //handleButton();
        }
      }
    }
  } else {
    if(current_state != button_pressed) {
//      Serial.println("Debouncing");
      debouncing = true;
      debounce_time = millis();
    }
  }    
}

void handleButton(void) {
  int state(g_state);
  int end_state(ANIMATION_END);
  state++;
  state = state % end_state;
  g_state = (ANIMATION_STATE)state;
  dumpState();
}

Metro random_period(3000);

//int redMic() {
//  analogReference(INTERNAL);
//  analogRead(rMicInp);
//  int retval(map(analogRead(rMicInp), 0, 1023, 0, 254));
//  analogReference(DEFAULT);
//  #ifdef UNO
//  Serial.print(" r");
//  Serial.print(retval);
//  Serial.print(" ");
//  #endif
//  return retval;
//}
//int greenMic() {
//  analogReference(INTERNAL);
//  analogRead(gMicInp);
//  int retval(map(analogRead(gMicInp), 0, 1023, 0, 254));
//  analogReference(DEFAULT);
//  #ifdef UNO
//  Serial.print(" g");
//  Serial.print(retval);
//  Serial.print(" ");
//  #endif
//  return retval;
//}
//int blueMic() {
//  
//  analogReference(INTERNAL);
//  analogRead(bMicInp);
//  int retval(map(analogRead(bMicInp), 0, 1023, 0, 254));
//  analogReference(DEFAULT);
//  #ifdef UNO
//  Serial.print(" b");
//  Serial.print(retval);
//  Serial.print(" ");
//  #endif
//  return retval;
//}

//Metro glowTime(50);
void glow() {
//  if(!glowTime.check()) return;
//  Serial.println("glowing");
  int r;
  lights.write(redRotary.value(),greenRotary.value(),blueRotary.value());
}

Metro blinkTime(100);
bool blinkon(false);

void blink() {
  if(!blinkTime.check()) return;
//  Serial.println("blinking");
  blinkon = !blinkon;
  if(blinkon) {
    lights.write(0,0,0);
  } else {
    lights.write(redRotary.value(),
      greenRotary.value(),
      blueRotary.value());
  }
}

//Metro pulseTime(1);
//float pulseCount(1);
int pulseDir(1);
const float pulse_period(5000);
//unsigned long pulse_start(0);

void pulse() {
//  if(!pulseTime.check()) return;
//  Serial.println("pulsing");

  unsigned long now=millis();
  float delta(now % (int)pulse_period);
  float factor;
  if(delta < pulse_period/2) {
    factor = 2.0/pulse_period*delta;
  } else {
    factor = 2.0-(2.0/(pulse_period*delta));
  }

  //lights.write(factor*redNob(),
  //  factor*greenNob(),
  //  factor*blueNob()
    
    lights.write(factor*redRotary.value(),
      factor*greenRotary.value(),
      factor*blueRotary.value());
}

//float popCount(0);
float pop_period(5000);

//Metro popTime(10);

void pop_lights() {

  unsigned long now=millis();
  float delta(now % (int)pop_period);
  float factor;
  factor = delta/pop_period;
  
  lights.write(factor*redRotary.value(),
    factor*greenRotary.value(),
    factor*blueRotary.value());
}


void listen() {
//
//  lights.write(redMic(),greenMic(),blueMic());
//  endColor();
}

//void randomWash () {
//  // every period, choose a new color at random and transition to it
//  // the period has a duty cycle where "on" is transitioning and off is stable at the current color
//
//  // for the transition, change the lead light
//  // toward the target color by the percentage of the duty cycle
//
//  // change the following leads by the percentage as if it were X seconds in the past
//  int period(2000);
//  int duty(333);
//  int following(50);
//
//  int cycle_start;
//
//
//}
int period(500);
void random_lights() {
  if(!random_period.check()) {
    return;
  }
  Serial.println("random lighting");
  
  period++;
  int r,g,b;
  int target;
//  Serial.println(period);
  target = (int)round(random(3));
  r = (int)round(random(255));
  g = (int)round(random(255));
  b = (int)round(random(255));
  //char buffer[20];
  //sprintf(buffer,"rnd %d %d %d %d\n",target,r,g,b);
  //Serial.print(buffer);
  switch(target) {
     case 0: light1.write(r,g,b); break;
     case 1: light2.write(r,g,b); break;
     case 2: light3.write(r,g,b); break;
     case 3: light4.write(r,g,b); break;     
  }
}

Metro animation_timer(1000/10); // 100 frames/second
void animate() {
  if(!animation_timer.check())
    return;
  switch(g_state) {
  case ANIMATION_ON : lights_on(); break;
  case ANIMATION_GLOW: glow(); break;
  case ANIMATION_BLINK: blink(); break;
  case ANIMATION_PULSE: pulse(); break;
  case ANIMATION_POP: pop_lights(); break;
//  case ANIMATION_LISTEN: listen(); break;
//  case ANIMATION_CYCLE: listen(); break;
  case ANIMATION_RANDOM: random_lights(); break;
  case ANIMATION_OFF: lights_off();break;
  case ANIMATION_COP: cop();break;
  default:
    Serial.print("ERROR: unknown state in animate");
    Serial.println(g_state);
  }
    
  return;
}    
  

  

Metro light_time(100);
void lights_on(void) {
  if(!light_time.check()) {
    return;
  }
  
  lights.write(255,255,255);
}
void lights_off(void) {
  if(!light_time.check()) {
    return;
  }
  lights.write(0,0,0);
}

Metro cop_timer(300); // alernate about every half second
bool cop_state(false);

void cop(void) {
  if(!cop_timer.check()) 
    return;
  if(cop_state) {
    light1.write(255,0,0); // 1/3 alternate red
    light2.write(0,0,255); // 2/4 alternate blue
    light3.write(0,0,0); 
    light4.write(0,0,0); 
  } else {
    light1.write(0,0,0); 
    light2.write(0,0,0); 
    light3.write(255,0,0); // 1/3 alternate red
    light4.write(0,0,255); // 2/4 alternate blue

  cop_state=!cop_state;
  } 
}

void setup() {
  delay(100);
  Serial.begin(115200); // &Serial
  Serial1.begin(9600);
  Serial2.begin(9600);
  
  WriteLight::setup(MODE); // sets up the output pins 
  
  redRotary.setup(redRotary);
  greenRotary.setup(greenRotary);
  blueRotary.setup(blueRotary);
  
  Serial.print("SETUP:");
  Serial.println(MODE);
  
  // and set up the button
  pinMode(button_pin, INPUT);
}

void loop() {
  debounce();
  if(WriteLight::isAnimated()) {
//    if(MODE & LIGHT_MODE_SERIAL_DEBUG)
      dumpState();
  
    animate();
  }

  WriteLight::loop();  // call library to read the serial if necessary  
  redRotary.loop();
  greenRotary.loop();
  blueRotary.loop();
}


void dumpState(void) {  
  Serial.print("\t\t\t");
  switch(g_state) {
  case ANIMATION_ON : Serial.println("lights_on()"); break;
  case ANIMATION_GLOW: Serial.println("glow()"); break;
  case ANIMATION_BLINK: Serial.println("blink()"); break;
  case ANIMATION_PULSE: Serial.println("pulse()"); break;
  case ANIMATION_POP: Serial.println("pop_lights()"); break;
//  case ANIMATION_LISTEN: Serial.println("listen()"); break;
//  case ANIMATION_CYCLE: listen(); break;
  case ANIMATION_COP: Serial.println("cop!");break;
  case ANIMATION_RANDOM: Serial.println("random_lights()"); break;
  case ANIMATION_OFF: Serial.println("lights_off()");break;
  default:
    Serial.print("ERROR: unknown state in dumpstate: ");
    Serial.println(g_state);
  }
  return;
}    
