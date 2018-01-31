#include <Arduino.h>

#define FORWARD  0
#define BACKWARD 1

class Motor
{
 public:
  Motor();
  ~Motor();

  uint32_t pwm_pin_;
  uint32_t dir_pin_;

  void attach(uint32_t pwm_pin, uint32_t dir_pin, uint32_t freq = 50);
  void move(uint32_t vel, uint8_t dir);
};