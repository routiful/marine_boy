#include <DynamixelWorkbench.h>

#define NECK_YAW   11
#define NECK_ROLL  12
#define NECK_PITCH 13

#define NECK_MOTOR_COUNT 3

#define FULL_RESOLUTION 4096
#define CENTER_POSTIION 2048

#define BAUD_RATE 1000000

void NECKBegin(uint16_t vel, uint16_t acc);
void NECKMove(int32_t *goal);
