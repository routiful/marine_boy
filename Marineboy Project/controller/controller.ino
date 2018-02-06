#include "PS3.h"
#include "SERVO.h"
#include "Motor.h"
#include "NECK.h"

#include "pitches.h"

#include <OpenManipulator.h>
#include "ARM.h"

#define SERVO_PWM_PIN 3
#define MOTOR_PWM_PIN 5

#define MOTOR_DIR_PIN 4

#define CART          0
#define NECK          1
#define ARM           2

#define TRUE          1
#define FALSE         0

#define JOY_MAX       200
#define JOY_MIN       50

#define PS3_CONTROL_PERIOD  20
#define NECK_CONTROL_PERIOD 20
#define CART_CONTROL_PERIOD 20

#define WHEEL_PROFILE_VELOCITY 15

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
  int16_t unit;
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

ctrl_t wheel_ctrl;
ctrl_t wrist_ctrl;
neck_t neck_ctrl;

uint32_t select_mode = 0;
uint8_t control_mode = 0;

bool is_connect = FALSE;

static uint32_t tTime[5];

void setup() 
{
  Serial.begin(115200);
  // while (!Serial);

  PS3Begin();
  NECKBegin();
  WheelBegin();
  WristBegin();
  ARMBegin();

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
        control_mode = select_mode % 3;

        if (control_mode == CART)
        {
          PS3LedOff();
          PS3LedOn(LED1);
        }
        else if (control_mode == NECK)
        {
          PS3LedOff();
          PS3LedOn(LED2);
        }
        else if (control_mode == ARM)
        {
          PS3LedOff();
          PS3LedOn(LED3);
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
       break;

      case NECK:
      if ((t-tTime[2]) >= (1000 / NECK_CONTROL_PERIOD))
      {
        NECKMove();
        tTime[2] = t;
      }
       break;

      case ARM:
        ARMMove();
       break;

      default:
       break;
    }
  }
}

void limitSign()
{
  PS3Rumble(true);
  delay(5);
  PS3Rumble(false);
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

void wheelCtrlInit()
{
  wheel_ctrl.goal = 0;
  wheel_ctrl.unit = 50;
  wheel_ctrl.limit.maximum = 1000;
  wheel_ctrl.limit.minimum = -1000;
}

void WheelBegin()
{
  wheelCtrlInit();
  left_wheel.attach(MOTOR_PWM_PIN, MOTOR_DIR_PIN, 200);
}

void wristCtrlInit()
{
  wrist_ctrl.goal = 90;
  wrist_ctrl.unit = 10;
  wrist_ctrl.limit.maximum = 180;
  wrist_ctrl.limit.minimum = 0;
}

void WristBegin()
{
  wristCtrlInit();
  wrist.attach(SERVO_PWM_PIN, 50); //20ms
}

void ARMBegin()
{
  const bool processing = FALSE;
  const bool dynamixel  = TRUE;
  const bool torque     = FALSE;

  initArm();
  OPMInit(arm, LINK_NUM, processing, dynamixel, torque);

  setTorque(TRUE);
}

void neckCtrlInit()
{
  neck_ctrl.roll.goal           = CENTER_POSTIION;
  neck_ctrl.roll.unit           = 30;
  neck_ctrl.roll.limit.maximum  = 2046+256;
  neck_ctrl.roll.limit.minimum  = 2046-256;

  neck_ctrl.pitch.goal          = CENTER_POSTIION;
  neck_ctrl.pitch.unit          = 30;
  neck_ctrl.pitch.limit.maximum = 2046+512;
  neck_ctrl.pitch.limit.minimum = 1800;

  neck_ctrl.yaw.goal            = CENTER_POSTIION;
  neck_ctrl.yaw.unit            = 30;
  neck_ctrl.yaw.limit.maximum   = 2046+512;
  neck_ctrl.yaw.limit.minimum   = 2046-512;
}

void HoldGrandMa()
{
  static int32_t init_neck[NECK_MOTOR_COUNT] = {CENTER_POSTIION, CENTER_POSTIION, CENTER_POSTIION};

  neckCtrlInit();

  wrist.write(90);
  NECKMove(init_neck);
}

void HoldCart()
{
  left_wheel.move(0, FORWARD);
}

void CartMove()
{
  WheelMove();
  WristMove();
}

void wheelBtnCtrl()
{
  if (PS3GetBtn(UP))
    wheel_ctrl.goal = wheel_ctrl.goal + wheel_ctrl.unit;
  else if (PS3GetBtn(DOWN))
    wheel_ctrl.goal = wheel_ctrl.goal - wheel_ctrl.unit;
  else if (PS3GetBtn(CROSS))
    wheel_ctrl.goal = 0;  // speed
}

void checkLimitSpeedOfWheel()
{
  if (wheel_ctrl.goal > wheel_ctrl.limit.maximum)
  {
    //limitSign();
    wheel_ctrl.goal = wheel_ctrl.limit.maximum;
  }
  else if (wheel_ctrl.goal < wheel_ctrl.limit.minimum)
  {
    //limitSign();
    wheel_ctrl.goal = wheel_ctrl.limit.minimum;
  }
}

void WheelMove()
{
  static int32_t control_speed = 0;

  wheelBtnCtrl();
  checkLimitSpeedOfWheel();

  if (wheel_ctrl.goal > control_speed)
    control_speed = MIN(wheel_ctrl.goal, control_speed + WHEEL_PROFILE_VELOCITY);
  else if (wheel_ctrl.goal < control_speed)
    control_speed = MAX(wheel_ctrl.goal, control_speed - WHEEL_PROFILE_VELOCITY);
  else
    control_speed = wheel_ctrl.goal;

  if (control_speed > 0)
    left_wheel.move(control_speed, FORWARD);
  else
    left_wheel.move((-1) * control_speed, BACKWARD);
}

void wristBtnCtrl()
{
  if (PS3GetBtn(CIRCLE))
    wrist_ctrl.goal = wrist_ctrl.goal + wrist_ctrl.unit;
  else if (PS3GetBtn(SQUARE))
    wrist_ctrl.goal = wrist_ctrl.goal - wrist_ctrl.unit;
  else if (PS3GetBtn(TRIANGLE))
    wrist_ctrl.goal = 90; // degree
}

void checkLimitPositionOfWrist()
{
  if (wrist_ctrl.goal >= wrist_ctrl.limit.maximum)
  {
    //limitSign();
    wrist_ctrl.goal = wrist_ctrl.limit.maximum;
  }
  else if (wrist_ctrl.goal <= wrist_ctrl.limit.minimum)
  {
    //limitSign();
    wrist_ctrl.goal = wrist_ctrl.limit.minimum;
  }
}

void WristMove()
{
  wristBtnCtrl();
  checkLimitPositionOfWrist();

  wrist.write(wrist_ctrl.goal);
}

void setNeckPositionUnit()
{
  if (PS3GetBtn(L1))
    neck_ctrl.yaw.unit = neck_ctrl.yaw.unit + 10;
  else if (PS3GetBtn(R1))
    neck_ctrl.yaw.unit = neck_ctrl.yaw.unit - 10;

  if (neck_ctrl.yaw.unit <= 0)
    neck_ctrl.yaw.unit = 0;

  neck_ctrl.roll.unit  = neck_ctrl.yaw.unit;
  neck_ctrl.pitch.unit = neck_ctrl.yaw.unit;
}

void neckBtnCtrl()
{
  if (PS3GetJoy(LeftHatX) > JOY_MAX)
    neck_ctrl.yaw.goal = neck_ctrl.yaw.goal - neck_ctrl.yaw.unit;
  else if (PS3GetJoy(LeftHatX) < JOY_MIN)
    neck_ctrl.yaw.goal = neck_ctrl.yaw.goal + neck_ctrl.yaw.unit;

  if (PS3GetJoy(RightHatX) > JOY_MAX)
    neck_ctrl.roll.goal = neck_ctrl.roll.goal - neck_ctrl.roll.unit;
  else if (PS3GetJoy(RightHatX) < JOY_MIN)
    neck_ctrl.roll.goal = neck_ctrl.roll.goal + neck_ctrl.roll.unit;

  if (PS3GetJoy(LeftHatY) > JOY_MAX)
    neck_ctrl.pitch.goal = neck_ctrl.pitch.goal - neck_ctrl.pitch.unit;
  else if (PS3GetJoy(LeftHatY) < JOY_MIN)
    neck_ctrl.pitch.goal = neck_ctrl.pitch.goal + neck_ctrl.pitch.unit;
}

void checkLimitPositionOfNeck()
{
  if (neck_ctrl.yaw.goal < neck_ctrl.yaw.limit.minimum)
  {
    //limitSign();
    neck_ctrl.yaw.goal = neck_ctrl.yaw.limit.minimum;    
  }
  else if (neck_ctrl.yaw.goal > neck_ctrl.yaw.limit.maximum)
  {
    //limitSign();
    neck_ctrl.yaw.goal = neck_ctrl.yaw.limit.maximum;
  }

  if (neck_ctrl.roll.goal <= neck_ctrl.roll.limit.minimum)
  {
    //limitSign();
    neck_ctrl.roll.goal = neck_ctrl.roll.limit.minimum;
  }
  else if (neck_ctrl.roll.goal >= neck_ctrl.roll.limit.maximum)
  {
    //limitSign();
    neck_ctrl.roll.goal = neck_ctrl.roll.limit.maximum;
  }

  if (neck_ctrl.pitch.goal <= neck_ctrl.pitch.limit.minimum)
  {
    //limitSign();
    neck_ctrl.pitch.goal = neck_ctrl.pitch.limit.minimum;
  }
  else if (neck_ctrl.pitch.goal >= neck_ctrl.pitch.limit.maximum)
  {
    //limitSign();
    neck_ctrl.pitch.goal = neck_ctrl.pitch.limit.maximum;
  }
}

void checkInitPositionOfNeck()
{
  if (PS3GetBtn(CIRCLE))
  {
    neck_ctrl.roll.goal  = CENTER_POSTIION;
    neck_ctrl.pitch.goal = CENTER_POSTIION;
    neck_ctrl.yaw.goal   = CENTER_POSTIION;
  }
}

void NECKMove()
{  
  static int32_t neck_goal_position[NECK_MOTOR_COUNT] = {CENTER_POSTIION, CENTER_POSTIION, CENTER_POSTIION};

  setNeckPositionUnit();
  neckBtnCtrl();
  checkLimitPositionOfNeck();
  checkInitPositionOfNeck();

  neck_goal_position[0] = neck_ctrl.yaw.goal;
  neck_goal_position[1] = neck_ctrl.roll.goal;
  neck_goal_position[2] = neck_ctrl.pitch.goal;

  NECKMove(neck_goal_position);
}

void ARMMove()
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