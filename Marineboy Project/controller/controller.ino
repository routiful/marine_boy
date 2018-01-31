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

void setup() 
{
  Serial.begin(115200);
  while (!Serial);

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
  PS3TaskOn();

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

    delay(50);
  }

  OPMRun();

  controlGrandma();  
}

void controlGrandma()
{
  if (is_connect)
  {
    switch(control_mode)
    {
      case CART:
        CartMove();
       break;

      case NECK:
        NECKMove();
       break;

      case ARM:
        ARMMove();
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
  wrist_ctrl.unit = 5;
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
  const bool processing = false;
  const bool dynamixel  = true;
  const bool torque     = false;

  initArm();
  OPMInit(arm, LINK_NUM, processing, dynamixel, torque);

  setTorque(true);
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
  if (wheel_ctrl.goal >= wheel_ctrl.limit.maximum)
    wheel_ctrl.goal = wheel_ctrl.limit.maximum;
  else if (wheel_ctrl.goal <= wheel_ctrl.limit.minimum)
    wheel_ctrl.goal = wheel_ctrl.limit.minimum;
}

void WheelMove()
{
  wheelBtnCtrl();
  checkLimitSpeedOfWheel();

  if (wheel_ctrl.goal > 0)
    left_wheel.move(wheel_ctrl.goal, FORWARD);
  else
    left_wheel.move((-1) * wheel_ctrl.goal, BACKWARD);
}

void wristBtnCtrl()
{
  if (PS3GetBtn(RIGHT))
    wrist_ctrl.goal = wrist_ctrl.goal + wrist_ctrl.unit;
  else if (PS3GetBtn(LEFT))
    wrist_ctrl.goal = wrist_ctrl.goal - wrist_ctrl.unit;
  else if (PS3GetBtn(TRIANGLE))
    wrist_ctrl.goal = 90; // degree
}

void checkLimitPositionOfWrist()
{
  if (wrist_ctrl.goal >= wrist_ctrl.limit.maximum)
    wrist_ctrl.goal = wrist_ctrl.limit.maximum;
  else if (wrist_ctrl.goal <= wrist_ctrl.limit.minimum)
    wrist_ctrl.goal = wrist_ctrl.limit.minimum;
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
  if (PS3GetJoy(RightHatX) > JOY_MAX)
    neck_ctrl.yaw.goal = neck_ctrl.yaw.goal - neck_ctrl.yaw.unit;
  else if (PS3GetJoy(RightHatX) < JOY_MIN)
    neck_ctrl.yaw.goal = neck_ctrl.yaw.goal + neck_ctrl.yaw.unit;

  if (PS3GetJoy(LeftHatX) > JOY_MAX)
    neck_ctrl.roll.goal = neck_ctrl.roll.goal - neck_ctrl.roll.unit;
  else if (PS3GetJoy(LeftHatX) < JOY_MIN)
    neck_ctrl.roll.goal = neck_ctrl.roll.goal + neck_ctrl.roll.unit;

  if (PS3GetJoy(LeftHatY) > JOY_MAX)
    neck_ctrl.pitch.goal = neck_ctrl.pitch.goal - neck_ctrl.pitch.unit;
  else if (PS3GetJoy(LeftHatY) < JOY_MIN)
    neck_ctrl.pitch.goal = neck_ctrl.pitch.goal + neck_ctrl.pitch.unit;
}

void checkLimitPositionOfNeck()
{
  if (neck_ctrl.yaw.goal <= neck_ctrl.yaw.limit.minimum)
    neck_ctrl.yaw.goal = neck_ctrl.yaw.limit.minimum;
  else if (neck_ctrl.yaw.goal >= neck_ctrl.yaw.limit.maximum)
    neck_ctrl.yaw.goal = neck_ctrl.yaw.limit.maximum;

  if (neck_ctrl.roll.goal <= neck_ctrl.roll.limit.minimum)
    neck_ctrl.roll.goal = neck_ctrl.roll.limit.minimum;
  else if (neck_ctrl.roll.goal >= neck_ctrl.roll.limit.maximum)
    neck_ctrl.roll.goal = neck_ctrl.roll.limit.maximum;

  if (neck_ctrl.pitch.goal <= neck_ctrl.pitch.limit.minimum)
    neck_ctrl.pitch.goal = neck_ctrl.pitch.limit.minimum;
  else if (neck_ctrl.pitch.goal >= neck_ctrl.pitch.limit.maximum)
    neck_ctrl.pitch.goal = neck_ctrl.pitch.limit.maximum;
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
  static float target_pos[LINK_NUM] = {0.0, };

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
  // else if (rcData & RC100_BTN_L)
  // {
  //   inverseKinematics(arm, GRIP, setPose("left"), "position");  

  //   for (int i = JOINT1; i < GRIP; i++)
  //     target_pos[i] = arm[i].joint_angle_;

  //   setJointAngle(target_pos);
  //   move(0.16);
  // }
  // else if (rcData & RC100_BTN_R)
  // {
  //   inverseKinematics(arm, GRIP, setPose("right"), "position");  

  //   for (int i = JOINT1; i < GRIP; i++)
  //     target_pos[i] = arm[i].joint_angle_;

  //   setJointAngle(target_pos);
  //   move(0.16);
  // }
  // else if (rcData & RC100_BTN_1)
  // {
  //   inverseKinematics(arm, GRIP, setPose("up"), "position");  

  //   for (int i = JOINT1; i < GRIP; i++)
  //     target_pos[i] = arm[i].joint_angle_;

  //   setJointAngle(target_pos);
  //   move(0.16);
  // }
  // else if (rcData & RC100_BTN_2)
  // {
  //   setCurrentPos(findMe("Gripper"), grip_on);
  // }
  // else if (rcData & RC100_BTN_3)
  // {
  //   inverseKinematics(arm, GRIP, setPose("down"), "position");  

  //   for (int i = JOINT1; i < GRIP; i++)
  //     target_pos[i] = arm[i].joint_angle_;

  //   setJointAngle(target_pos);
  //   move(0.16);
  // }
  else if (PS3GetBtn(L2))
  {
    target_pos[1] =  0.0;
    target_pos[2] =  0.0;
    target_pos[3] =  0.0;
    target_pos[4] =  0.0;

    setJointAngle(target_pos);
    move(1.0);
  }
  else if (PS3GetBtn(L1))
  {
    target_pos[1] =  0.0;
    target_pos[2] =  95.0 * PI/180.0;
    target_pos[3] = -75.0 * PI/180.0;
    target_pos[4] =  20.0 * PI/180.0;

    setJointAngle(target_pos);
    move(1.0);
  }
}