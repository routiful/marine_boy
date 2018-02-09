#include "Motor.h"

  Motor::Motor(){}
  Motor::~Motor(){}

  void Motor::attach(uint32_t pwm_pin, uint32_t dir_pin, uint32_t freq)
  {
    this->pwm_pin_ = pwm_pin;
    this->dir_pin_ = dir_pin;

    drv_pwm_set_freq(pwm_pin_, freq);
    drv_pwm_setup(pwm_pin_);

    pinMode(dir_pin_, OUTPUT);
  }

  void Motor::move(uint32_t duty, uint8_t dir)
  {
    uint32_t res = 10;
    uint32_t goal_duty = 0;

    if (dir == FORWARD)
      digitalWrite(dir_pin_, HIGH);
    else if (dir == BACKWARD)
      digitalWrite(dir_pin_, LOW);

    goal_duty = map(duty, 0, 2000, 0, 1024);
    drv_pwm_set_duty(pwm_pin_, res, goal_duty);
  }