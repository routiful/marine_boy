#include "SERVO.h"

  Servo::Servo(){}
  Servo::~Servo(){}

  void Servo::attach(uint32_t pwm_pin, uint32_t freq)
  {
    this->pwm_pin_ = pwm_pin;

    drv_pwm_set_freq(pwm_pin_, freq);
    drv_pwm_setup(pwm_pin_);
  }

  void Servo::write(uint8_t deg)
  {
    uint32_t res = 10;
    uint32_t duty = 0;

    duty = map(deg, 0, 180, 51, 102);
    drv_pwm_set_duty(pwm_pin_, res, duty);
  }