#include "PS3.h"
#include "SERVO.h"
#include "Motor.h"
#include "NECK.h"

#include "pitches.h"

#include <OpenManipulator.h>
#include "ARM.h"

// #define DEBUG

#define SERVO_PWM_PIN       3

#define LEFT_MOTOR_PWM_PIN  5
#define RIGHT_MOTOR_PWM_PIN 6
#define LEG_MOTOR_PWM_PIN   A1

#define LEFT_MOTOR_DIR_PIN  4
#define RIGHT_MOTOR_DIR_PIN 7
#define LEG_MOTOR_DIR_PIN   A0

#define CART          0
#define ARM           1

#define TRUE          1
#define FALSE         0

#define LEFT          0
#define RIGHT         1

#define JOY_MAX       200
#define JOY_MIN       50

#define PS3_CONTROL_PERIOD  200 //5ms
#define NECK_CONTROL_PERIOD 2   //5us
#define CART_CONTROL_PERIOD 5   //5ms

#define WHEEL_PROFILE_VELOCITY 2
#define LEG_PROFILE_VELOCITY 2

#define MOTOR_FREQ 200

#define	MAX(x,y) ((x) > (y) ? (x) : (y))	
#define	MIN(x,y) ((x) < (y) ? (x) : (y))	

typedef struct
{
  int16_t maximum;
  int16_t minimum;
} limit_t;

typedef struct
{
  int32_t goal;
  limit_t limit;
} ctrl_t;

typedef struct
{
  ctrl_t roll;
  ctrl_t pitch;
  ctrl_t yaw;
} neck_t;

Servo wrist;
Motor left_wheel;
Motor right_wheel;

ctrl_t left_wheel_ctrl;
ctrl_t right_wheel_ctrl;
ctrl_t leg_ctrl;
ctrl_t wrist_ctrl;
neck_t neck_ctrl;

int32_t controlled_goal[2] = {0, 0};
int32_t controlled_leg = 0;

uint32_t select_mode = 0;
uint8_t control_mode = 0;

bool is_connect = FALSE;

static uint32_t tTime[10];

void setup() 
{
  Serial.begin(115200);
#ifdef DEBUG
  while (!Serial);
#endif

  PS3Begin();

  WheelBegin();
  LegBegin();
  WristBegin();
  NECKBegin();
  // ARMBegin();

  HoldGrandMa();
  HoldCart();
}

void loop() 
{
  uint32_t t = millis();
  PS3TaskOn();  

  if ((t-tTime[0]) >= (1000 / PS3_CONTROL_PERIOD))
  {
    if (PS3Available())
    {    
      is_connect = TRUE;

      connectSign();

      // PS3Print();
      if (PS3GetBtn(SELECT))
      {
        select_mode++;    
        control_mode = select_mode % 2;

        if (control_mode == CART)
        {
          PS3LedOff();
          PS3LedOn(LED1);
        }
        else if (control_mode == ARM)
        {
          PS3LedOff();
          PS3LedOn(LED2);
        }
      }
    }
    tTime[0] = t;
  }

  OPMRun();
  controlGrandma();  
}

void controlGrandma()
{
  uint32_t t = micros();

  if (is_connect)
  {
    switch(control_mode)
    {
      case CART:
        if ((t-tTime[1]) >= CART_CONTROL_PERIOD)  
        {
          CartMove();
          tTime[1] = t;
        }         

        if ((t-tTime[2]) >= (1000 * NECK_CONTROL_PERIOD))
        {
          NeckMove();
          tTime[2] = t;
        }        
       break;

      case ARM:
        // ArmMove();
       break;

      default:
       break;
    }
  }
}

void connectSign()
{
  static bool isSign = FALSE;

  if (is_connect == TRUE)
  {
    if (isSign == FALSE)
    {
      int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};

      // note durations: 4 = quarter note, 8 = eighth note, etc.:
      int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4}; 

      // iterate over the notes of the melody:
      for (int thisNote = 0; thisNote < 8; thisNote++) 
      {
        // to calculate the note duration, take one second
        // divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / noteDurations[thisNote];
        tone(BDPIN_BUZZER, melody[thisNote], noteDuration);

        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        // stop the tone playing:
        noTone(BDPIN_BUZZER);
      }

      isSign = TRUE;
    }
  }
  else
  {
    isSign = FALSE;
  }
}

/*////////////////////////////////////////////////////
@ Control Initializion
/*////////////////////////////////////////////////////

void wheelCtrlInit()
{
  right_wheel_ctrl.goal = 0;
  right_wheel_ctrl.limit.maximum = 2000;
  right_wheel_ctrl.limit.minimum = -2000;

  left_wheel_ctrl.goal = 0;
  left_wheel_ctrl.limit.maximum = 2000;
  left_wheel_ctrl.limit.minimum = -2000;
}

void legCtrlInit()
{
  leg_ctrl.goal = 0;
  leg_ctrl.limit.maximum = 2000;
  leg_ctrl.limit.minimum = -2000;
}

void wristCtrlInit()
{
  wrist_ctrl.goal = 90;
  wrist_ctrl.limit.maximum = 180;
  wrist_ctrl.limit.minimum = 0;
}

void neckCtrlInit()
{
  neck_ctrl.roll.goal           = CENTER_POSTIION;
  neck_ctrl.roll.limit.maximum  = 256;
  neck_ctrl.roll.limit.minimum  = -256;

  neck_ctrl.pitch.goal          = CENTER_POSTIION;
  neck_ctrl.pitch.limit.maximum = CENTER_POSTIION+512;
  neck_ctrl.pitch.limit.minimum = 1800;

  neck_ctrl.yaw.goal            = CENTER_POSTIION;
  neck_ctrl.yaw.limit.maximum   = CENTER_POSTIION+512;
  neck_ctrl.yaw.limit.minimum   = CENTER_POSTIION-512;
}

/*////////////////////////////////////////////////////
@ Begin
/*////////////////////////////////////////////////////

void WheelBegin()
{
  wheelCtrlInit();
  left_wheel.attach(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, MOTOR_FREQ);
  right_wheel.attach(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, MOTOR_FREQ);
}

void LegBegin()
{
  legCtrlInit();

  pinMode(LEG_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEG_MOTOR_PWM_PIN, OUTPUT);
}

void WristBegin()
{
  wristCtrlInit();
  wrist.attach(SERVO_PWM_PIN, 50); //20ms
}

// NECKBegin() -> NECK.h

void ARMBegin()
{
  const bool processing = FALSE;
  const bool dynamixel  = TRUE;
  const bool torque     = FALSE;

  initArm();
  OPMInit(arm, LINK_NUM, processing, dynamixel, torque);

  setTorque(TRUE);
}

/*////////////////////////////////////////////////////
@ Hold 
/*////////////////////////////////////////////////////

void HoldGrandMa()
{
  static int32_t init_neck[NECK_MOTOR_COUNT] = {CENTER_POSTIION, CENTER_POSTIION, CENTER_POSTIION};

  neckCtrlInit();

  NECKMove(init_neck);
}

void HoldCart()
{
  left_wheel.move(0, FORWARD);
  right_wheel.move(0, FORWARD);
  // leg.move(0, FORWARD);

  wrist.write(90);
}

/*////////////////////////////////////////////////////
@ PS3 Joystick Control 
/*////////////////////////////////////////////////////

void wheelJoyCtrl()
{
  int8_t get_x_hat = map(PS3GetJoy(LeftHatX), 0, 255, 127, -127); 
  int8_t get_y_hat = map(PS3GetJoy(LeftHatY), 0, 255, 127, -127); 

  float lin_vel = (float)(get_y_hat);
  float ang_vel = (float)(get_x_hat);

  left_wheel_ctrl.goal  = (lin_vel - (ang_vel * 0.750 / 2)) * 16;
  right_wheel_ctrl.goal = (lin_vel + (ang_vel * 0.750 / 2)) * 16;

  left_wheel_ctrl.goal = constrain(left_wheel_ctrl.goal, left_wheel_ctrl.limit.minimum, left_wheel_ctrl.limit.maximum);
  right_wheel_ctrl.goal = constrain(right_wheel_ctrl.goal, right_wheel_ctrl.limit.minimum, right_wheel_ctrl.limit.maximum);
}

void legJoyCtrl()
{
  int8_t get_y_hat = map(PS3GetJoy(LeftHatY), 0, 255, 127, -127);

  leg_ctrl.goal = get_y_hat * 16;

  leg_ctrl.goal = constrain(leg_ctrl.goal, leg_ctrl.limit.minimum, leg_ctrl.limit.maximum);
}

void wristJoyCtrl()
{
  uint16_t get_y_hat = map(PS3GetJoy(LeftHatX), 0, 255, 0, 720);

  wrist_ctrl.goal = get_y_hat;
}

void neckJoyCtrl()
{
  int32_t get_x_hat = map(PS3GetJoy(RightHatX), 0, 255, neck_ctrl.yaw.limit.maximum, neck_ctrl.yaw.limit.minimum); 
  int32_t get_y_hat = map(PS3GetJoy(RightHatY), 0, 255, neck_ctrl.pitch.limit.maximum, neck_ctrl.pitch.limit.minimum);

  int32_t get_L2    = map(PS3GetAnalogBtn(L2),  0, 255, 0, neck_ctrl.roll.limit.minimum);
  int32_t get_R2    = map(PS3GetAnalogBtn(R2),  0, 255, 0, neck_ctrl.roll.limit.maximum); 


  neck_ctrl.roll.goal  = 2048 + (get_L2 + get_R2);
  neck_ctrl.pitch.goal = get_y_hat;
  neck_ctrl.yaw.goal   = get_x_hat;
}

/*////////////////////////////////////////////////////
@ Simple profile
/*////////////////////////////////////////////////////

int32_t getSimpleProfile(int32_t output, int32_t input, int32_t slope)
{
  if (input > output)
    output = MIN(input, output + slope);
  else if (input < output)
    output = MAX(input, output - slope);
  else
    output = input;

  return output;
}

/*////////////////////////////////////////////////////
@ Move
/*////////////////////////////////////////////////////

void CartMove()
{
  WheelMove();
  WristMove();
  LegMove();
}

void WheelMove()
{
  wheelJoyCtrl(); 

  controlled_goal[LEFT] = getSimpleProfile(controlled_goal[LEFT], left_wheel_ctrl.goal, WHEEL_PROFILE_VELOCITY);
  controlled_goal[RIGHT] = getSimpleProfile(controlled_goal[RIGHT], right_wheel_ctrl.goal, WHEEL_PROFILE_VELOCITY);

  if (controlled_goal[0] < 0 && controlled_goal[1] > 0)
  {
    left_wheel.move((-1) * controlled_goal[0], BACKWARD);
    right_wheel.move(controlled_goal[1], FORWARD);
  }
  else if (controlled_goal[0] > 0 && controlled_goal[1] < 0)
  {
    left_wheel.move(controlled_goal[0], FORWARD);
    right_wheel.move((-1) * controlled_goal[1], BACKWARD);
  }
  else if (controlled_goal[0] < 0 && controlled_goal[1] < 0)
  {
    left_wheel.move((-1) * controlled_goal[0], BACKWARD);
    right_wheel.move((-1) * controlled_goal[1], BACKWARD);
  }
  else
  {
    left_wheel.move(controlled_goal[0], FORWARD);
    right_wheel.move(controlled_goal[1], FORWARD);
  }
}

void LegMove()
{
  static uint16_t timer_cnt = 0;

  if (timer_cnt >= 1024) timer_cnt = 0; //60ms

  legJoyCtrl();

  // controlled_leg = getSimpleProfile(controlled_leg, leg_ctrl.goal, LEG_PROFILE_VELOCITY);

  if (leg_ctrl.goal > 0)
    digitalWrite(LEG_MOTOR_DIR_PIN, HIGH);
  else
    digitalWrite(LEG_MOTOR_DIR_PIN, LOW);

  leg_ctrl.goal = map(abs(leg_ctrl.goal), 0, 2000, 0, 1024);

  if (timer_cnt <= leg_ctrl.goal)
    digitalWrite(LEG_MOTOR_PWM_PIN, HIGH);
  else
    digitalWrite(LEG_MOTOR_PWM_PIN, LOW);

  timer_cnt++;
}

void WristMove()
{
  wristJoyCtrl();

  wrist.write(wrist_ctrl.goal);
}

void NeckMove()
{  
  static int32_t neck_goal_position[NECK_MOTOR_COUNT] = {CENTER_POSTIION, CENTER_POSTIION, CENTER_POSTIION};

  neckJoyCtrl();

  neck_goal_position[0] = neck_ctrl.yaw.goal;
  neck_goal_position[1] = neck_ctrl.roll.goal;
  neck_goal_position[2] = neck_ctrl.pitch.goal;

  NECKMove(neck_goal_position);
}

void ArmMove()
{
  uint32_t t = millis();
  static float target_pos[LINK_NUM] = {0.0, };

  if ((t-tTime[3]) >= (1000 / PS3_CONTROL_PERIOD))
  {
    if (PS3GetJoy(LeftHatY) < JOY_MIN)
    {    
      inverseKinematics(arm, GRIP, setPose("forward"), "position");  
      
      for (int i = JOINT1; i < GRIP; i++)
        target_pos[i] = arm[i].joint_angle_;

      setJointAngle(target_pos);
      move(0.16);
    }
    else if (PS3GetJoy(LeftHatY) > JOY_MAX)
    {
      inverseKinematics(arm, GRIP, setPose("back"), "position");  

      for (int i = JOINT1; i < GRIP; i++)
        target_pos[i] = arm[i].joint_angle_;

      setJointAngle(target_pos);
      move(0.16);
    }
    
    if (PS3GetJoy(LeftHatX) < JOY_MIN)
    {
      inverseKinematics(arm, GRIP, setPose("left"), "position");  

      for (int i = JOINT1; i < GRIP; i++)
        target_pos[i] = arm[i].joint_angle_;

      setJointAngle(target_pos);
      move(0.16);
    }
    else if (PS3GetJoy(LeftHatX) > JOY_MAX)
    {
      inverseKinematics(arm, GRIP, setPose("right"), "position");  

      for (int i = JOINT1; i < GRIP; i++)
        target_pos[i] = arm[i].joint_angle_;

      setJointAngle(target_pos);
      move(0.16);
    }
    
    if (PS3GetJoy(RightHatY) < JOY_MIN)
    {
      inverseKinematics(arm, GRIP, setPose("up"), "position");  

      for (int i = JOINT1; i < GRIP; i++)
        target_pos[i] = arm[i].joint_angle_;

      setJointAngle(target_pos);
      move(0.16);
    }
    else if (PS3GetJoy(RightHatY) > JOY_MAX)
    {
      inverseKinematics(arm, GRIP, setPose("down"), "position");  

      for (int i = JOINT1; i < GRIP; i++)
        target_pos[i] = arm[i].joint_angle_;

      setJointAngle(target_pos);
      move(0.16);
    }
    
    if (PS3GetJoy(RightHatX) > JOY_MAX)
    {

      target_pos[4] += (3.0 * DEG2RAD);

      setJointAngle(target_pos);
      move(0.1);
    }
    else if (PS3GetJoy(RightHatX) < JOY_MIN)
    {

      target_pos[4] -= (3.0 * DEG2RAD);

      setJointAngle(target_pos);
      move(0.1);
    }
    
    if (PS3GetBtn(L1))
    {
      setTorque(TRUE);
      getAngle();
    }
    else if (PS3GetBtn(R1))
    {
      setTorque(FALSE);
    }
    else if (PS3GetBtn(TRIANGLE))
    {
      target_pos[1] =  0.0;
      target_pos[2] =  0.0;
      target_pos[3] =  0.0;
      target_pos[4] =  0.0;

      setJointAngle(target_pos);
      move(2.0);
    }
    else if (PS3GetBtn(CROSS))
    {
      target_pos[1] = -0.76;
      target_pos[2] =  0.51;
      target_pos[3] =  0.07;
      target_pos[4] =  0.0;

      setJointAngle(target_pos);
      move(2.0);
    }

    tTime[3] = t;
  }
}