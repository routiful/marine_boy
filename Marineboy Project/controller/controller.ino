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
#define LEG_MOTOR_PWM_PIN   9

#define LEFT_MOTOR_DIR_PIN  4
#define RIGHT_MOTOR_DIR_PIN 7
#define LEG_MOTOR_DIR_PIN   8

#define CART          0
#define ARM           1

#define TRUE          1
#define FALSE         0

#define JOY_MAX       200
#define JOY_MIN       50

#define PS3_CONTROL_PERIOD  200
#define NECK_CONTROL_PERIOD 200
#define CART_CONTROL_PERIOD 200

#define DEGREE_PROFILE_VELOCITY 0.5

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
Motor leg;

ctrl_t left_wheel_ctrl;
ctrl_t right_wheel_ctrl;
ctrl_t leg_ctrl;
ctrl_t wrist_ctrl;
neck_t neck_ctrl;

uint32_t select_mode = 0;
uint8_t control_mode = 0;

bool is_connect = FALSE;

static uint32_t tTime[5];

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

      if (PS3GetBtn(START)) 
      {
        PS3End();
      }
    }
    tTime[0] = t;
  }

  OPMRun();
  controlGrandma();  
}

void controlGrandma()
{
  uint32_t t = millis();

  if (is_connect)
  {
    switch(control_mode)
    {
      case CART:
        if ((t-tTime[1]) >= (1000 / CART_CONTROL_PERIOD))
        {
          CartMove();
          tTime[1] = t;
        }         

        if ((t-tTime[2]) >= (1000 / NECK_CONTROL_PERIOD))
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
  right_wheel_ctrl.limit.maximum = 1000;
  right_wheel_ctrl.limit.minimum = -1000;

  left_wheel_ctrl.goal = 0;
  left_wheel_ctrl.limit.maximum = 1000;
  left_wheel_ctrl.limit.minimum = -1000;
}

void legCtrlInit()
{
  leg_ctrl.goal = 0;
  leg_ctrl.limit.maximum = 1000;
  leg_ctrl.limit.minimum = -1000;
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
  left_wheel.attach(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN, 200);
  right_wheel.attach(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, 200);
}

void LegBegin()
{
  legCtrlInit();
  leg.attach(LEG_MOTOR_PWM_PIN, LEG_MOTOR_DIR_PIN, 200);
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
  leg.move(0, FORWARD);

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

  left_wheel_ctrl.goal  = (lin_vel - (ang_vel * 0.750 / 2)) * 13;
  right_wheel_ctrl.goal = (lin_vel + (ang_vel * 0.750 / 2)) * 13;

  left_wheel_ctrl.goal = constrain(left_wheel_ctrl.goal, left_wheel_ctrl.limit.minimum, left_wheel_ctrl.limit.maximum);
  right_wheel_ctrl.goal = constrain(right_wheel_ctrl.goal, right_wheel_ctrl.limit.minimum, right_wheel_ctrl.limit.maximum);
}

void legJoyCtrl()
{
  int8_t get_y_hat = map(PS3GetJoy(LeftHatY), 0, 255, 127, -127);

  leg_ctrl.goal = get_y_hat * 8;

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
@ PS3 Button Control 
/*////////////////////////////////////////////////////

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

  if (left_wheel_ctrl.goal < 0 && right_wheel_ctrl.goal > 0)
  {
    left_wheel.move((-1) * left_wheel_ctrl.goal, BACKWARD);
    right_wheel.move(right_wheel_ctrl.goal, FORWARD);
  }
  else if (left_wheel_ctrl.goal > 0 && right_wheel_ctrl.goal < 0)
  {
    left_wheel.move(left_wheel_ctrl.goal, FORWARD);
    right_wheel.move((-1) * right_wheel_ctrl.goal, BACKWARD);
  }
  else if (left_wheel_ctrl.goal < 0 && right_wheel_ctrl.goal < 0)
  {
    left_wheel.move((-1) * left_wheel_ctrl.goal, BACKWARD);
    right_wheel.move((-1) * right_wheel_ctrl.goal, BACKWARD);
  }
  else
  {
    left_wheel.move(left_wheel_ctrl.goal, FORWARD);
    right_wheel.move(right_wheel_ctrl.goal, FORWARD);
  }
}

void LegMove()
{
  legJoyCtrl();

  if (leg_ctrl.goal < 0)
    leg.move((-1) * leg_ctrl.goal, BACKWARD);
  else
    leg.move(leg_ctrl.goal, FORWARD);
}

void WristMove()
{
  // static int32_t controlled_goal = 0;

  wristJoyCtrl();

  // if (wrist_ctrl.goal > controlled_goal)
  //   controlled_goal = MIN(wrist_ctrl.goal, controlled_goal + DEGREE_PROFILE_VELOCITY);
  // else if (wrist_ctrl.goal < controlled_goal)
  //   controlled_goal = MAX(wrist_ctrl.goal, controlled_goal - DEGREE_PROFILE_VELOCITY);
  // else
  //   controlled_goal = wrist_ctrl.goal;

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