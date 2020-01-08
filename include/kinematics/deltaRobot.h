#ifndef DELTAROBOT_H
#define DELTAROBOT_H
//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.

#include "vector3.h"
#include "configuration.h"

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------
void robot_position(float npx,float npy,float npz);
void deltarobot_setup();
void update_ik();
void update_wrist_positions();
void update_elbows();
void update_shoulder_angles();
void robot_tool_offset(int axis,float x,float y,float z);
Vector3 robot_get_end_plus_offset();
struct ArmAngles getArmAngles();

//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------

struct ArmAngles {
  int angleB1;
  int angleB2;
  int angleB3;
};

struct Joint {
  Vector3 pos;
  Vector3 relative;
};


struct Arm {
  Vector3 shoulder;
  Joint elbow;
  Joint wrist;
  Joint wop;
  
  float angle;

  // for motors
  int new_step;
  
  int delta;
  int absdelta;
  int dir;
  int over;

  Vector3 plane_ortho;
  Vector3 plane_normal;

  // for limit switches
  int motor_step_pin;
  int motor_dir_pin;
  int motor_enable_pin;
  int limit_switch_pin;
  int limit_switch_state;  
};

struct DeltaRobot {
  Arm arms[MAX_AXIES];
  Vector3 ee;  // current position of the end effector
  float e;  // rotation of the 4th axis
  Joint base;
  Vector3 tool_offset[NUM_TOOLS];
  int current_tool;
  float default_height;
};

/**
* This file is part of Delta Robot v8.
*
* Delta Robot v8 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Delta Robot v8 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Delta Robot v8. If not, see <http://www.gnu.org/licenses/>.
*/
#endif

