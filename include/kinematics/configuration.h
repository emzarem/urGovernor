#ifndef CONFIGURATION_H
#define CONFIGURATION_H
//------------------------------------------------------------------------------
// Delta Robot v8 - Supports RUMBA 6-axis motor controller
// dan@marginallycelver.com 2014-01-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/DeltaRobotv8 for more information.

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
#define VERBOSE              (0)  // increasing number increases output
#define DEBUG_SWITCHES       (1)  // set to one to see output

#define STEPS_PER_TURN       (3200.0)  // default number of steps per turn * microsteps
#define MICROSTEPS           (16.0)  // microstepping on this microcontroller

#define MAX_ANGLE            (90+85)
#define MIN_ANGLE            (90-30)

// split long lines into pieces to make them more correct.
#define MM_PER_SEGMENT       (10)
#define NUM_TOOLS            (6)
// related to number of instructions that can be buffered.  must be a power of two > 1.
#define MAX_SEGMENTS         (32)

#define TWOPI                (PI*2.0)
#define DEG2RAD              (PI/180.0)
#define RAD2DEG              (180.0/PI)

#define MICROSTEPS_PER_TURN  (STEPS_PER_TURN*MICROSTEPS)
#define MICROSTEP_PER_DEGREE (MICROSTEPS_PER_TURN/360.0)

#define NUM_AXIES	  (3)
#define MAX_AXIES     (3)  // code supports up to this many axies.

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

