/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <DynamixelWorkbench.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif   

#define BAUDRATE  1000000
#define DXL_ID_1  11
#define DXL_ID_2  12
#define DXL_ID_3  13
#define DXL_NUM 3

DynamixelWorkbench dxl_wb;

uint8_t dxl_id[DXL_NUM] = {DXL_ID_1, DXL_ID_2, DXL_ID_3};
uint8_t dxl_cnt = DXL_NUM;

void setup() 
{
  Serial.begin(57600);
  while(!Serial); // If this line is activated, you need to open Serial Terminal.

  dxl_wb.begin(DEVICE_NAME, BAUDRATE);

  for (int cnt = 0; cnt < dxl_cnt; cnt++)
  {
    dxl_wb.ping(dxl_id[cnt]);
    dxl_wb.jointMode(dxl_id[cnt]);
    dxl_wb.itemWrite(dxl_id[cnt], "Torque_Enable", false);
  }

  dxl_wb.addSyncRead("Present_Position");
}

void loop() 
{  
  int32_t present_position[DXL_NUM] = {0, };
  float present_radian[DXL_NUM] = {0.0, };
  int32_t *get_data;

  get_data = dxl_wb.syncRead("Present_Position");
  for (int cnt = 0; cnt < dxl_cnt; cnt++)
  {
    present_position[cnt] = get_data[cnt];
    present_radian[cnt] = dxl_wb.convertValue2Radian(dxl_id[cnt], present_position[cnt]);
  }
  
  Serial.print("[DXL_1 : ");
  Serial.print(present_radian[0]);

  Serial.print("  DXL_2 : ");
  Serial.print(present_radian[1]);

  Serial.print("  DXL_3 : ");
  Serial.print(present_radian[2]);

  Serial.println("  ]");
}