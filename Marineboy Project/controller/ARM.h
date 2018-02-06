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

/* Authors: Darby Lim */

#ifndef OPENMANIPULATOR_CHAIN_H_
#define OPENMANIPULATOR_CHAIN_H_

#define BASE    0
#define JOINT1  1
#define JOINT2  2
#define JOINT3  3
#define JOINT4  4
#define GRIP    5

#define LINK_NUM 6

OPMLink arm[LINK_NUM];

void initArm()
{
  arm[BASE].name_                      = "Base";
  arm[BASE].me_                        = BASE;
  arm[BASE].mother_                    = -1;
  arm[BASE].sibling_                   = -1;
  arm[BASE].child_                     = JOINT1;
  arm[BASE].p_                         = Eigen::Vector3f::Zero();
  arm[BASE].R_                         = Eigen::Matrix3f::Identity(3,3);
  arm[BASE].joint_angle_               = 0.0;
  arm[BASE].joint_vel_                 = 0.0;
  arm[BASE].joint_acc_                 = 0.0;
  arm[BASE].joint_axis_                = Eigen::Vector3f::Zero();
  arm[BASE].joint_pos_                 = Eigen::Vector3f::Zero();

  arm[JOINT1].name_                    = "Joint1";
  arm[JOINT1].me_                      = JOINT1;
  arm[JOINT1].mother_                  = BASE;
  arm[JOINT1].sibling_                 = -1;
  arm[JOINT1].child_                   = JOINT2;
  arm[JOINT1].p_                       = Eigen::Vector3f::Zero();
  arm[JOINT1].R_                       = Eigen::Matrix3f::Identity(3,3);
  arm[JOINT1].joint_angle_             = 0.0;
  arm[JOINT1].joint_vel_               = 0.0;
  arm[JOINT1].joint_acc_               = 0.0;
  arm[JOINT1].joint_axis_              << 0, -1, 0;
  arm[JOINT1].joint_pos_               << 0.0, -0.0465, 0.0;

  arm[JOINT2].name_                    = "Joint2";
  arm[JOINT2].me_                      = JOINT2;
  arm[JOINT2].mother_                  = JOINT1;
  arm[JOINT2].sibling_                 = -1;
  arm[JOINT2].child_                   = JOINT3;
  arm[JOINT2].p_                       = Eigen::Vector3f::Zero();
  arm[JOINT2].R_                       = Eigen::Matrix3f::Identity(3,3);
  arm[JOINT2].joint_angle_             = 0.0;
  arm[JOINT2].joint_vel_               = 0.0;
  arm[JOINT2].joint_acc_               = 0.0;
  arm[JOINT2].joint_axis_              << 0, 0, 1;
  arm[JOINT2].joint_pos_               << 0, -0.0515, 0.0;

  arm[JOINT3].name_                    = "Joint3";
  arm[JOINT3].me_                      = JOINT3;
  arm[JOINT3].mother_                  = JOINT2;
  arm[JOINT3].sibling_                 = -1;
  arm[JOINT3].child_                   = JOINT4;
  arm[JOINT3].p_                       = Eigen::Vector3f::Zero();
  arm[JOINT3].R_                       = Eigen::Matrix3f::Identity(3,3);
  arm[JOINT3].joint_angle_             = 0.0;
  arm[JOINT3].joint_vel_               = 0.0;
  arm[JOINT3].joint_acc_               = 0.0;
  arm[JOINT3].joint_axis_              << 0, -0.390, 0.920;
  arm[JOINT3].joint_pos_               << 0.0, -0.1198, -0.1264;

  arm[JOINT4].name_                    = "Joint4";
  arm[JOINT4].me_                      = JOINT4;
  arm[JOINT4].mother_                  = JOINT3;
  arm[JOINT4].sibling_                 = -1;
  arm[JOINT4].child_                   = GRIP;
  arm[JOINT4].p_                       = Eigen::Vector3f::Zero();
  arm[JOINT4].R_                       = Eigen::Matrix3f::Identity(3,3);
  arm[JOINT4].joint_angle_             = 0.0;
  arm[JOINT4].joint_vel_               = 0.0;
  arm[JOINT4].joint_acc_               = 0.0;
  arm[JOINT4].joint_axis_              << 1, 0, 0;
  arm[JOINT4].joint_pos_               << 0.064, 0, 0;

  arm[GRIP].name_                      = "Gripper";
  arm[GRIP].me_                        = GRIP;
  arm[GRIP].mother_                    = JOINT4;
  arm[GRIP].sibling_                   = -1;
  arm[GRIP].child_                     = -1;
  arm[GRIP].p_                         = Eigen::Vector3f::Zero();
  arm[GRIP].R_                         = Eigen::Matrix3f::Identity(3,3);
  arm[GRIP].joint_angle_               = 0.0;
  arm[GRIP].joint_vel_                 = 0.0;
  arm[GRIP].joint_acc_                 = 0.0;
  arm[GRIP].joint_axis_                = Eigen::Vector3f::Zero();
  arm[GRIP].joint_pos_                 << 0.106, 0, 0;
}

#endif //OPENMANIPULATOR_CHAIN_H_