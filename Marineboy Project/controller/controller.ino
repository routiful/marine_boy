/*
//
//
//  고물수레 프로젝트 v1.4
//  Artist : Marine Boy
//  Software Engineer : Darby Lim (ROBOTIS)
//  Hardware Engineer : Dorian Kim (ROBOTIS)
//
//
*/

#include "PS3.h"
#include "SERVO.h"
#include "Motor.h"
#include "NECK.h"

#include "pitches.h"

#include <OpenManipulator.h>
#include "ARM.h"

//#define DEBUG 
////////////////////////////////////////////////////////////////////////

#define SET_WRIST_MIN_ANGLE -60
#define SET_WRIST_MAX_ANGLE 60
// 죄측 -50 , 우측 30 몸통회전 각도 
#define NECK_VEL 20
#define NECK_ACC 100

#define PAL_MOK_ANGLE 10.0
#define PAL_MOK_TIME 1

////////////////////////////////////////////////////////////////////////

#define SERVO_PWM_PIN       3

#define LEFT_MOTOR_PWM_PIN  5
#define RIGHT_MOTOR_PWM_PIN 6
#define LEG_MOTOR_PWM_PIN   A1

#define LEFT_MOTOR_DIR_PIN  4
#define RIGHT_MOTOR_DIR_PIN 7
#define LEG_MOTOR_DIR_PIN   A0

#define HORN_SIG_PIN             A2
#define LINEAR_FORWARD_SIG_PIN   A3
#define LINEAR_BACKWARD_SIG_PIN  A4
#define TV_SIG_PIN               A5
#define IMAGE_SIG_PIN             2

#define CART          0
#define GRANDMA       1
#define ETC           2

#define TRUE          1
#define FALSE         0

#define LEFT_M          0
#define RIGHT_M         1

#define JOY_MAX       200
#define JOY_MIN       50

#define PS3_CONTROL_PERIOD  200 //5ms

#define NECK_CONTROL_PERIOD 50  //50ms
#define CART_CONTROL_PERIOD 5   //5ms
#define ARM_CONTROL_PERIOD  20  //50ms
#define ETC_CONTROL_PERIOD  50  //50ms

#define WHEEL_PROFILE_VELOCITY 2
#define LEG_PROFILE_VELOCITY 4
// 벨로시티 여윤 
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

bool is_rumble = FALSE;

static uint64_t tTime[10];

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
  NECKBegin(NECK_VEL, NECK_ACC);
  ARMBegin();
  ETCBegin();

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
        is_rumble = false;

        if (control_mode == CART)
        {
          PS3LedOff();
          PS3LedOn(LED1);           
        }
        else if (control_mode == GRANDMA)
        {
          PS3LedOff();
          PS3LedOn(LED2); 
        }
        else if (control_mode == ETC)
        {
          PS3LedOff();
          PS3LedOn(LED3);
        }
      }

      if (is_rumble != true)
      {
        if (control_mode == CART)
        {
          modeChangeSign(1, 3);     
        }
        else if (control_mode == GRANDMA)
        {
          modeChangeSign(2, 3);
        }
        else if (control_mode == ETC)
        {
          modeChangeSign(3, 3);
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
  uint64_t t = micros();

  if (is_connect)
  {
    switch(control_mode)
    {
      case CART:
        ArmMotion();

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

      case GRANDMA:
        ArmMove();

        if ((t-tTime[2]) >= (1000 * NECK_CONTROL_PERIOD))
        {
          NeckMove();
          tTime[2] = t;
        }  
       break;

      case ETC:
        if ((t-tTime[2]) >= (1000 * NECK_CONTROL_PERIOD))
        {
          NeckMove();
          tTime[2] = t;
        }  

        if ((t-tTime[4]) >= (1000 * ETC_CONTROL_PERIOD))
        {
          etcJoyCtrl();
          tTime[4] = t;
        }  
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

void modeChangeSign(uint8_t cnt, uint8_t period)
{
  static uint8_t rumble_cnt = 0;
  static uint8_t time_cnt = 0;

  if (rumble_cnt < cnt)
  {
    if (time_cnt < period)
    {
      PS3Rumble(true);
      time_cnt++;
    }
    else if ((time_cnt >= period) && (time_cnt < period*2))
    {
      PS3Rumble(false);
      time_cnt++;
    }
    else
    {
      time_cnt = 0;
      rumble_cnt++;
    }  
  }
  else
  {
    PS3Rumble(false);
    rumble_cnt = 0;
    time_cnt = 0;

    is_rumble = true;
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
// 0에서 2000까지 속도 변환 가능 
void legCtrlInit()
{
  leg_ctrl.goal = 0;
  leg_ctrl.limit.maximum = 2000;
  leg_ctrl.limit.minimum = -2000;
}
// 0에서 2000까지 속도 변환 가능 
void wristCtrlInit()
{
  wrist_ctrl.goal = 90;
  wrist_ctrl.limit.maximum = 91;
  wrist_ctrl.limit.minimum = 89;
}
// 허리 각도 변환 goal은 초기각도 , 맥시멈 135도 , 미니멈 45도 총 90도 이동 // 실험해 보니 이값은 조정값이 아님 25,26줄 값 수정
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

void ETCBegin()
{
  pinMode(HORN_SIG_PIN, OUTPUT);
  pinMode(LINEAR_FORWARD_SIG_PIN, OUTPUT);
  pinMode(LINEAR_BACKWARD_SIG_PIN, OUTPUT);
  pinMode(TV_SIG_PIN, OUTPUT);
  pinMode(IMAGE_SIG_PIN, OUTPUT);
  digitalWrite(HORN_SIG_PIN, HIGH);
  
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
  leg_ctrl.goal = 0;

  uint16_t init_angle = 90;
  init_angle = map(init_angle, -90, 90, 0, 720);
  wrist.write(init_angle);
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

  left_wheel_ctrl.goal  = (lin_vel - (ang_vel * 0.750 / 2)) * 11;
  right_wheel_ctrl.goal = (lin_vel + (ang_vel * 0.750 / 2)) * 11;

  left_wheel_ctrl.goal = constrain(left_wheel_ctrl.goal, left_wheel_ctrl.limit.minimum, left_wheel_ctrl.limit.maximum);
  right_wheel_ctrl.goal = constrain(right_wheel_ctrl.goal, right_wheel_ctrl.limit.minimum, right_wheel_ctrl.limit.maximum);
}

void legJoyCtrl()
{
  int8_t get_y_hat = map(PS3GetJoy(LeftHatY), 0, 255, 127, -127);

  leg_ctrl.goal = get_y_hat * 7;

  leg_ctrl.goal = constrain(leg_ctrl.goal, leg_ctrl.limit.minimum, leg_ctrl.limit.maximum);
}

void wristJoyCtrl()
{
  int16_t set_min_angle = SET_WRIST_MIN_ANGLE;
  int16_t set_max_angle = SET_WRIST_MAX_ANGLE;

  set_min_angle = map(set_min_angle, -90, 90, 0, 720);
  set_max_angle = map(set_max_angle, -90, 90, 0, 720);

  uint16_t get_y_hat = 0;

  int8_t get_grandma_dir = map(PS3GetJoy(LeftHatY), 0, 255, 127, -127); 

  if (get_grandma_dir > 0)
    get_y_hat = map(PS3GetJoy(LeftHatX), 0, 255, set_min_angle, set_max_angle);
  else if (get_grandma_dir <= 0)
    get_y_hat = map(PS3GetJoy(LeftHatX), 0, 255, set_max_angle, set_min_angle);

  wrist_ctrl.goal = get_y_hat;
}

void neckJoyCtrl()
{
  int32_t get_x_hat = constrain(PS3GetJoy(RightHatX), 0, 255);
  int32_t get_y_hat = constrain(PS3GetJoy(RightHatY), 0, 255);

  int32_t get_L2 = constrain(PS3GetAnalogBtn(L2), 0, 255); 
  int32_t get_R2 = constrain(PS3GetAnalogBtn(R2), 0, 255); 

  get_x_hat = map(get_x_hat, 255, 0, neck_ctrl.yaw.limit.minimum, neck_ctrl.yaw.limit.maximum); 
  get_y_hat = map(get_y_hat, 0, 255, neck_ctrl.pitch.limit.minimum, neck_ctrl.pitch.limit.maximum);

  if (control_mode == CART)
  {
    get_R2    = map(PS3GetAnalogBtn(R2),  0, 255, 0, neck_ctrl.roll.limit.minimum);
    get_L2    = map(PS3GetAnalogBtn(L2),  0, 255, 0, neck_ctrl.roll.limit.maximum); 
  }
  else
  {
    get_R2 = 0; get_L2 = 0;
  }

  neck_ctrl.roll.goal  = 2048 + (get_L2 + get_R2);
  neck_ctrl.pitch.goal = get_y_hat;
  neck_ctrl.yaw.goal   = get_x_hat;
}

void etcJoyCtrl()
{
  static bool horn_btn_state = false;
  static bool linear_forward_btn_state = false;
  static bool linear_backward_btn_state = false;
  static bool tv_btn_state  = false;
  static bool image_btn_state = false;

  // 한번 누르면 온, 또 누르면 오프
  if (PS3GetBtn(UP))     {horn_btn_state             = !horn_btn_state;}
  if (PS3GetBtn(LEFT))   {linear_forward_btn_state   = !linear_forward_btn_state;}
  if (PS3GetBtn(RIGHT))  {linear_backward_btn_state  = !linear_backward_btn_state;}
  if (PS3GetBtn(CROSS))  {tv_btn_state               = !tv_btn_state;}
  if (PS3GetBtn(CIRCLE)) {image_btn_state            = !image_btn_state;}

  if (horn_btn_state == false)
    digitalWrite(HORN_SIG_PIN, HIGH);
  else
    digitalWrite(HORN_SIG_PIN, LOW);

  if (linear_forward_btn_state == false)
    digitalWrite(LINEAR_FORWARD_SIG_PIN, LOW);
  else
    digitalWrite(LINEAR_FORWARD_SIG_PIN, HIGH);

  if (linear_backward_btn_state == false)
    digitalWrite(LINEAR_BACKWARD_SIG_PIN, LOW);
  else
    digitalWrite(LINEAR_BACKWARD_SIG_PIN, HIGH);

  if (tv_btn_state == false)
    digitalWrite(TV_SIG_PIN, LOW);
  else
    digitalWrite(TV_SIG_PIN, HIGH);

  if (image_btn_state == false)
    digitalWrite(IMAGE_SIG_PIN, LOW);
  else
    digitalWrite(IMAGE_SIG_PIN, HIGH);
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

// 모터 천천히 움직임 [ ] * 0.1~0.9 등등으로 변경하면 가속도 증가 - 천천히 속도 올라감 // 최고속도도 영향있음 

  controlled_goal[LEFT_M] = getSimpleProfile(controlled_goal[LEFT_M], left_wheel_ctrl.goal, WHEEL_PROFILE_VELOCITY);
  controlled_goal[RIGHT_M] = getSimpleProfile(controlled_goal[RIGHT_M], right_wheel_ctrl.goal, WHEEL_PROFILE_VELOCITY);

  if (controlled_goal[LEFT_M] < 0 && controlled_goal[RIGHT_M] > 0)
  {
    left_wheel.move((-1) * controlled_goal[LEFT_M] * 0.5, BACKWARD);
    right_wheel.move(controlled_goal[RIGHT_M] * 0.5, FORWARD);
  }
  else if (controlled_goal[LEFT_M] > 0 && controlled_goal[RIGHT_M] < 0)
  {
    left_wheel.move(controlled_goal[LEFT_M] * 1, FORWARD);
    right_wheel.move((-1) * controlled_goal[RIGHT_M] * 1, BACKWARD);
  }
  else if (controlled_goal[LEFT_M] < 0 && controlled_goal[RIGHT_M] < 0)
  {
    left_wheel.move((-1) * controlled_goal[LEFT_M] * 0.9, BACKWARD);
    right_wheel.move((-1) * controlled_goal[RIGHT_M] * 0.9, BACKWARD);
  }
  else
  {
    left_wheel.move(controlled_goal[LEFT_M] * 1, FORWARD);
    right_wheel.move(controlled_goal[RIGHT_M] * 1, FORWARD);
  }

// 모터 급격히 움직임

//  if (left_wheel_ctrl.goal < 0 && right_wheel_ctrl.goal > 0)
//  {
//    left_wheel.move((-1) * left_wheel_ctrl.goal, BACKWARD);
//    right_wheel.move(right_wheel_ctrl.goal, FORWARD);
//  }
//  else if (left_wheel_ctrl.goal > 0 && right_wheel_ctrl.goal < 0)
//  {
//    left_wheel.move(left_wheel_ctrl.goal, FORWARD);
//    right_wheel.move((-1) * right_wheel_ctrl.goal, BACKWARD);
//  }
//  else if (left_wheel_ctrl.goal < 0 && right_wheel_ctrl.goal < 0)
//  {
//    left_wheel.move((-1) * left_wheel_ctrl.goal, BACKWARD);
//    right_wheel.move((-1) * right_wheel_ctrl.goal, BACKWARD);
//  }
//  else
//  {
//    left_wheel.move(left_wheel_ctrl.goal, FORWARD);
//    right_wheel.move(right_wheel_ctrl.goal, FORWARD);
//  }
//  if (left_wheel_ctrl.goal < 0 && right_wheel_ctrl.goal > 0)
//  {
//    left_wheel.move((-1) * left_wheel_ctrl.goal, BACKWARD);
//    right_wheel.move(right_wheel_ctrl.goal, FORWARD);
//  }
//  else if (left_wheel_ctrl.goal > 0 && right_wheel_ctrl.goal < 0)
//  {
//    left_wheel.move(left_wheel_ctrl.goal, FORWARD);
//    right_wheel.move((-1) * right_wheel_ctrl.goal, BACKWARD);
//  }
//  else if (left_wheel_ctrl.goal < 0 && right_wheel_ctrl.goal < 0)
//  {
//    left_wheel.move((-1) * left_wheel_ctrl.goal, BACKWARD);
//    right_wheel.move((-1) * right_wheel_ctrl.goal, BACKWARD);
//  }
//  else
//  {
//    left_wheel.move(left_wheel_ctrl.goal, FORWARD);
//    right_wheel.move(right_wheel_ctrl.goal, FORWARD);
//  }
}

void LegMove()
{
  static uint16_t timer_cnt = 0;

  if (timer_cnt >= 1024) timer_cnt = 0; //60ms

  legJoyCtrl();

  // controlled_leg = getSimpleProfile(controlled_leg, leg_ctrl.goal, LEG_PROFILE_VELOCITY);

  if (leg_ctrl.goal >= 0)
    digitalWrite(LEG_MOTOR_DIR_PIN, LOW);
  else
    digitalWrite(LEG_MOTOR_DIR_PIN, HIGH);

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

  if ((t-tTime[3]) >= (1000 / ARM_CONTROL_PERIOD))
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
    
    if (PS3GetAnalogBtn(R2) > JOY_MAX)
    {
      inverseKinematics(arm, GRIP, setPose("up"), "position");  

      for (int i = JOINT1; i < GRIP; i++)
        target_pos[i] = arm[i].joint_angle_;

      setJointAngle(target_pos);
      move(0.16);
    }
    else if (PS3GetAnalogBtn(L2) > JOY_MAX)
    {
      inverseKinematics(arm, GRIP, setPose("down"), "position");  

      for (int i = JOINT1; i < GRIP; i++)
        target_pos[i] = arm[i].joint_angle_;

      setJointAngle(target_pos);
      move(0.16);
    }
    
    if (PS3GetBtn(R1))
    {
      target_pos[4] += (PAL_MOK_ANGLE * DEG2RAD);

      setJointAngle(target_pos);
      move(PAL_MOK_TIME);
    }
    else if (PS3GetBtn(L1))
    {
      target_pos[4] -= (PAL_MOK_ANGLE * DEG2RAD);

      setJointAngle(target_pos);
      move(PAL_MOK_TIME);
    }
    
    if (PS3GetBtn(UP))
    {
      setTorque(TRUE);
      getAngle();
    }
    else if (PS3GetBtn(DOWN))
    {
      setTorque(FALSE);
    }
    // Add Arm Motion
    else if (PS3GetBtn(TRIANGLE))
    {
      target_pos[1] =  1.07;
      target_pos[2] =  0.52;
      target_pos[3] = -0.51;
      target_pos[4] =  0.63;

      setJointAngle(target_pos);
      move(2.0);
    }
    else if (PS3GetBtn(CROSS))
    {
      target_pos[1] =  0.02;
      target_pos[2] =  0.18;
      target_pos[3] = -0.21;
      target_pos[4] = -0.52;

      setJointAngle(target_pos);
      move(2.0);
    }
    else if (PS3GetBtn(CIRCLE))
    {
      target_pos[1] =  1.52;
      target_pos[2] =  0.41;
      target_pos[3] =  0.35;
      target_pos[4] =  1.05;

      setJointAngle(target_pos);
      move(2.0);
    }
    else if (PS3GetBtn(SQUARE))
    {
      target_pos[1] =  2.21;
      target_pos[2] =  0.06;
      target_pos[3] = -0.70;
      target_pos[4] = -0.36;

      setJointAngle(target_pos);
      move(2.0);
    }   

    tTime[3] = t;
  }
}

void ArmMotion()
{
  uint32_t t = millis();
  static float motion_pos[LINK_NUM] = {0.0, };

  if ((t-tTime[5]) >= (1000 / ARM_CONTROL_PERIOD))
  {
    if (PS3GetBtn(TRIANGLE))
    {
      motion_pos[1] =  1.07;
      motion_pos[2] =  0.52;
      motion_pos[3] = -0.51;
      motion_pos[4] =  0.63;

      setJointAngle(motion_pos);
      move(2.0);
    }
    else if (PS3GetBtn(CROSS))
    {
      motion_pos[1] =  0.02;
      motion_pos[2] =  0.18;
      motion_pos[3] = -0.21;
      motion_pos[4] = -0.52;

      setJointAngle(motion_pos);
      move(2.0);
    }
    else if (PS3GetBtn(CIRCLE))
    {
      motion_pos[1] =  1.52;
      motion_pos[2] =  0.41;
      motion_pos[3] =  0.35;
      motion_pos[4] =  1.05;

      setJointAngle(motion_pos);
      move(2.0);
    }
    else if (PS3GetBtn(SQUARE))
    {
      motion_pos[1] =  2.21;
      motion_pos[2] =  0.06;
      motion_pos[3] = -0.70;
      motion_pos[4] = -0.36;

      setJointAngle(motion_pos);
      move(2.0);
    }   

    tTime[5] = t;
  }
}