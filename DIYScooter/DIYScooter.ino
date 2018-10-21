//public domain
//bitluni 2018

#include "MPU.h"

//pin configuration
const int BUZZER_PIN = 5;
const int THROTTLE_PIN = 10;
const int BRAKE_PIN = 16;

//throttle parameters (0-255 for PWM)
const int SPEED_MIN = 90;
const int SPEED_START = 100;
const int SPEED_MAX = 180;
const int SPEED_INCREMENT = 20;

//speedup ramp (ms)
const long ACCELERATION_DURATION = 1500;

//duration the speed is kept
const long HOLD_DURATION = 10000;

//throttle reduce per second during decelaration
const float DECELERATION_RATE = 10;
const int INTEGRAL_VALUES = 100;

//tuning parameters
const double GRAVITY_COMPENSATION = 0.1;  //lowpass value for the acceleration detection
const long KICK_THRESHOLD = 40000;        //sensitivity of the kick detection
const long WRONG_THRESHOLD = -40000;      //sensitivity in the other direction
const long WRONG_DELAY = 1500;            //ms that have to pass from the last acc backwards
const long LONG_CLICK_DELAY = 1000;
const long DEBOUNCE_DELAY = 10;
const long CLICK_DELAY = 90;
const long MULTI_CLICK_DELAY = 1000;

//tilt parameters
const long G = 0b100000000000000;         //accel 1g = 2^14
const float G_ATTENUATION = 0.05;
const float TILT_FRONT_MIN_G = 0.90;      //0.9 g
const float TILT_SIDE_ANGLE = 45;         
const long TILT_FRONT_DELAY = 1000;       //dection delay to igoner vibrations
const long TILT_SIDE_DELAY = 1000;
const float BASE_ANGLE = 0;

//turns off after 10min
const long IDLE_OFF_DELAY = 60000;

const bool DEBUG_OUTPUT = false;

//scooter states
enum State
{
  OFF,
  IDLE,
  ACCELERATE,
  HOLD,
  DECELERATE
};

State state = OFF;

//global signals that are set by the dection methods and are indicating the state of the scooter 
float speed = 0;
float topSpeed = 0;
bool kick = false;
bool brake = false;
long lastBrake = 0;
long lastNoBrake = 0;
bool click = false;
int clicks = 0;
long lastClick = -MULTI_CLICK_DELAY;
long idle = 0;
long holdTimeout = 0;
long accelerationTimer = 0;

bool tiltFront = false;
bool tiltSide = false; 
bool tilt = false;

bool wrongDirection = false;
long lastWrongDirection = 0;

//changes state and outputs sounds
void changeState(State s)
{
  state = s;
  switch(state)
  {
    case OFF:
    {
      speed = 0;
      Serial.println("Off");
      tone(BUZZER_PIN, 880, 50);
      delay(100);      
      tone(BUZZER_PIN, 880, 50);
      delay(100);      
      tone(BUZZER_PIN, 880, 50);
      delay(100);      
      break;
    }
    case IDLE:
    {
      idle = 0;
      speed = 0;
      Serial.println("Idle");
      tone(BUZZER_PIN, 880, 50);
      delay(100);      
      tone(BUZZER_PIN, 400, 50);
      delay(100);      
      break;      
    }
    case ACCELERATE:
    {
      accelerationTimer = 0;
      Serial.println("Accelerate");
      tone(BUZZER_PIN, 440, 100);
      break;
    }
    case HOLD:
    {
      holdTimeout = HOLD_DURATION;
      Serial.println("Hold");
      tone(BUZZER_PIN, 1760, 100);
      break;
    }
    case DECELERATE:
    {
      Serial.println("Decelerate");
      break;
    }
  }
}

//setup
void setup()
{
  initMPU();  //init gyro
  Serial.begin(115200);
  //brake pin is set to pull up. 
  //it will be connected to ground when brake is connected and pulls high when brake is engaged or disconnected
  pinMode(BRAKE_PIN, INPUT_PULLUP);
  changeState(OFF);
  //optional hack to get 32khz pwm and smoother throttle signal
  TCCR1B = TCCR1B & 0b11111001;
}

//analyze if the scooter is tilted
void analyzeTilt(long t, long dt)
{
  static float g = 1;
  static float angle = 0;
  static long tiltSideTime = 0;
  static long tiltFrontTime = 0;
  //calculate absolute acceleration with lowpass
  g = g * (1.f - G_ATTENUATION) + sqrt(float(AcX) * float(AcX) + float(AcZ) * float(AcZ)) / G * G_ATTENUATION;
  
  //detect if tilted to the front or back
  if(g < TILT_FRONT_MIN_G)
  {
    //has to be tilted for some time to compensate vibrations
    if(tiltFrontTime > TILT_FRONT_DELAY)
      tiltFront = true;
    if(tiltFrontTime < 0)
      tiltFrontTime = 0;
    tiltFrontTime += dt;
  }
  else
  {
    //not tilted for some time to clear flag
    if(-tiltFrontTime > TILT_FRONT_DELAY)
      tiltFront = false;
    if(tiltFrontTime > 0)
      tiltFrontTime = 0;
    tiltFrontTime -= dt;
  }

  //calculate the angle to the side
  angle = angle * (1.f - G_ATTENUATION) + atan2(AcX, AcZ) / M_PI * 180 * G_ATTENUATION;
  //tiltSide = angle > TILT_SIDE_ANGLE;
  
  //set and reset the tilt flag delayed to compensate for vibrations
  if(fabs(angle) > TILT_SIDE_ANGLE)
  {
    if(tiltSideTime > TILT_SIDE_DELAY)
      tiltSide = true;
    if(tiltSideTime < 0)
      tiltSideTime = 0;
    tiltSideTime += dt;
  }
  else
  {
    if(-tiltSideTime > TILT_SIDE_DELAY)
      tiltSide = false;
    if(tiltSideTime > 0)
      tiltSideTime = 0;
    tiltSideTime -= dt;
  }
  if(DEBUG_OUTPUT)
  {  
    Serial.print(tiltSide);
    Serial.print(" ");
    Serial.print(tiltFront);
    Serial.print(" ");
    Serial.print(angle);
    Serial.print(" ");
    Serial.println(g);
  }
  tilt = tiltSide || tiltFront;
}

//detect if brake is engaged and count consecutive clicks. clicks are debounced.
void analyzeBrake(long t, long dt)
{
  static bool debounce = false;
  lastBrake += dt;
  lastNoBrake += dt;
  lastClick += dt;
  if(digitalRead(BRAKE_PIN))
  {
    if(!brake)
      lastBrake = 0;
    brake = true;
    debounce = false;
  }
  else
  {    
    if(brake)
    {
      if(!debounce)
      {
        debounce = true;
        lastNoBrake = 0;
      }
      else
      {
        if(lastNoBrake > DEBOUNCE_DELAY)
        {
          if(lastBrake > CLICK_DELAY && lastBrake < LONG_CLICK_DELAY)
          {
            if(lastClick > MULTI_CLICK_DELAY)
              clicks = 1;
            else
              clicks++;
            Serial.print("Click ");
            Serial.println(clicks);
            click = true;
            lastClick = 0;
          }      
          brake = false;
        }
      }
    }
  }
}

//detect the kick. ignore backwards kicks and vibrations
void analyzeKick(long t, long dt)
{
  static short values[INTEGRAL_VALUES] = {0};
  static int pos = 0;
  static double avg = 0;
  static long integral = 0;
  //lopassed accelearion value
  avg = avg * (1 - GRAVITY_COMPENSATION) + AcY * GRAVITY_COMPENSATION;
  //compensate slight tilts to keep it around 0
  short value = -(AcY - int(avg));
  //record the new value
  values[pos] = value;
  pos = (pos + 1) % INTEGRAL_VALUES;
  //calculate the weighted integral
  integral = 0;
  for(int i = 0; i < INTEGRAL_VALUES; i++)
  {
    //the values are summed up. the oldes value get the lowest weight
    //this prevents hard edges caused by the gate
    integral += values[(pos + i) % INTEGRAL_VALUES] * 1. / INTEGRAL_VALUES * i;
  }
  
  //is a kick detected
  kick = integral > KICK_THRESHOLD;

  //detect if it's accelerating in the wrong direction
  wrongDirection = integral < WRONG_THRESHOLD;
  if(wrongDirection)
    lastWrongDirection = 0;
  else
    lastWrongDirection += dt;

  if(DEBUG_OUTPUT)
  {
    Serial.print(AcY);
    Serial.print(" ");
    Serial.println(integral);
  }
}

//main loop. it reads the gyro values runs detections and processes all incoming signals
void loop()
{
  static bool waitNoBrake = false;

  //read gyro
  readMPU();
  //claculate time passed
  long t = millis();
  static long lastT = 0;
  long dt = t - lastT;
  if(lastT > t) dt = 10;
  lastT = t;

  //analyze gyro inputs and caculate the global signals
  analyzeTilt(t, dt);
  analyzeKick(t, dt);
  analyzeBrake(t, dt);

//act accodring to the state
  switch(state)
  {
    case OFF:
    {
      if(click)
      {
        //check for tripple click to turn on
        if(clicks == 3)
        {
          changeState(IDLE);
        }
      }
      break;
    }
    case IDLE:
    {
      if(brake)
      {
        //if long brake turn off
        if(!waitNoBrake && lastBrake >= LONG_CLICK_DELAY)
          changeState(OFF);
      }
      else
      {
        waitNoBrake = false;
        //turn off if nothing happening
        if(idle > IDLE_OFF_DELAY)
          changeState(OFF);
        idle += dt;
        if(tilt)
        {
          break;
        }
        //kick detected! go!
        if( kick && lastWrongDirection >= WRONG_DELAY)
        {
          speed = SPEED_START;
          changeState(ACCELERATE);            
        }
      }      
      break;
    }
    case ACCELERATE:
    {
      //brke -> stop!
      if(brake)
      {
        changeState(IDLE);
        break;
      }
      //tilted -> stop!
      if(tiltFront || tiltSide)
      {
        changeState(IDLE);
        break;
      }
      //during acceleration dont accept additional kicks.
      if(accelerationTimer > ACCELERATION_DURATION)
      {
        //hold speed now
        changeState(HOLD);
      }
      accelerationTimer += dt;
      break;
    }
    case HOLD:
    {
      //brake -> stop
      if((brake && lastBrake >= LONG_CLICK_DELAY) || (click && !waitNoBrake))
      {
        changeState(IDLE);
        break;
      }
      //tilt -> stop
      if(tiltFront || tiltSide)
      {
        changeState(IDLE);
        break;
      }
      //increase speed!
      if(kick && lastWrongDirection >= WRONG_DELAY)
      {
        speed = min(SPEED_MAX, speed + SPEED_INCREMENT);
        changeState(ACCELERATE);
        break;
      }
      if(!brake)
        waitNoBrake = false;
      holdTimeout -= dt;
      //decelerate after some time
      if(holdTimeout <= 0)
        changeState(DECELERATE);
      break;
    }
    case DECELERATE:
    {
      //brake -> stop!
      if(brake || tiltFront || tiltSide)
      {
        changeState(IDLE);
        break;
      }
      //kick -> accelerate again
      if(kick && lastWrongDirection >= WRONG_DELAY)
      {
        speed = min(SPEED_MAX, speed + SPEED_INCREMENT);
        changeState(ACCELERATE);
        break;
      }      
      //reduce speed
      speed = max(SPEED_MIN, speed - DECELERATION_RATE * 0.001 * dt);
      //idle if no speed left
      if(speed <= SPEED_MIN)
      {
        changeState(IDLE);
      }      
    }
  }

  //all clicks are processed,clear flag now
  click = false;

  //
  if(state == ACCELERATE || state == HOLD || state == DECELERATE)
  {
    analogWrite(THROTTLE_PIN, int(speed));
  }
  else
    analogWrite(THROTTLE_PIN, 100);
  delay(10);
}

