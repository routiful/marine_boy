#include "NECK.h"

static DynamixelWorkbench dxl_wb;

static uint8_t dxl_id[NECK_MOTOR_COUNT] = {NECK_YAW, NECK_ROLL, NECK_PITCH};
static uint8_t dxl_cnt = NECK_MOTOR_COUNT;

void NECKBegin(uint16_t vel, uint16_t acc)
{
  dxl_wb.begin("", BAUD_RATE);
  
  for (int cnt = 0; cnt < NECK_MOTOR_COUNT; cnt++)
  {
    dxl_wb.ping(dxl_id[cnt]);
    dxl_wb.jointMode(dxl_id[cnt], vel, acc);
  }

  dxl_wb.addSyncWrite("Goal_Position");
}

void NECKMove(int32_t *goal)
{
  dxl_wb.syncWrite("Goal_Position", goal);
}