/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "mav_exercise.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <stdio.h>
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance_mod(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA
};

// define settings
float oag_color_count_frac_mod = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac_mod = 0.05f;       // floor detection threshold as a fraction of total of image
float oag_max_speed_mod = 0.5f;               // max flight speed [m/s]
float oag_heading_rate_mod = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]

// ###########################################################################################
int32_t px_offset = 0;
// ###########################################################################################

// define and initialise global variables
enum navigation_state_t navigation_state_mod = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count_mod = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count_mod = 0;                // green color count from color filter for floor detection
int32_t floor_centroid_mod = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction_mod = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence_mod = 0;   // a measure of how certain we are that the way ahead if safe.


//` needed to receive output from a separate module running on a parallel process`
const int16_t max_trajectory_confidence_mod = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count_mod = quality;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count_mod = quality;
  floor_centroid_mod = pixel_y;
}


// ###########################################################################################
//` needed to receive output from a separate module running on a parallel process (defined in cv_detect_color_object_mod.xml)
#ifndef OFFSET_DETECTION_ID
#error Please define OFFSET_DETECTION_ID
#endif
static abi_event offset_detection_ev;
static void offset_detection_cb(uint8_t __attribute__((unused)) sender_id,
                            float __attribute__((unused)) pixel_offset)
{
    px_offset = pixel_offset;
}
// ###########################################################################################

/*
 * Initialisation function
 */
void mav_exercise_init(void) {
  // Initialise random values
    srand(time(NULL));
    chooseRandomIncrementAvoidance_mod();

  // bind our colorfilter callbacks to receive the color filter outputs
  // ###########################################################################################
  AbiBindMsgOFFSET_DETECTION(OFFSET_DETECTION_ID, &offset_detection_ev, offset_detection_cb);
  // ###########################################################################################

  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void mav_exercise_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state_mod = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence_mod = 0;
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac_mod * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac_mod * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid_mod / (float)front_camera.output_size.h / 2.f;

  VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count_mod, color_count_threshold, navigation_state_mod);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count_mod, floor_count_threshold);
  VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);
  VERBOSE_PRINT("px_offset: %d\n", px_offset);

  // update our safe confidence using color threshold
  if(color_count_mod < color_count_threshold){
    obstacle_free_confidence_mod++;
  } else {
    obstacle_free_confidence_mod -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence_mod, 0, max_trajectory_confidence_mod);

  float speed_sp = fminf(oag_max_speed_mod, 0.2f * obstacle_free_confidence_mod);

  switch (navigation_state_mod){
    case SAFE:
      if (floor_count_mod < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state_mod = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence_mod == 0){
        navigation_state_mod = OBSTACLE_FOUND;
      } else {
        guidance_h_set_guided_body_vel(speed_sp, 0);
      }

      break;
    case OBSTACLE_FOUND:
      // stop
      guidance_h_set_guided_body_vel(0, 0);

      // randomly select new search direction
      chooseRandomIncrementAvoidance_mod();

      navigation_state_mod = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      guidance_h_set_guided_heading_rate(avoidance_heading_direction_mod * oag_heading_rate_mod);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence_mod >= 2){
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state_mod = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
      // stop
      guidance_h_set_guided_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_guided_heading_rate(avoidance_heading_direction_mod * RadOfDeg(15));

      navigation_state_mod = REENTER_ARENA;

      break;
    case REENTER_ARENA:
      // force floor center to opposite side of turn to head back into arena
      if (floor_count_mod >= floor_count_threshold && avoidance_heading_direction_mod * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence_mod = 0;

        // ensure direction is safe before continuing
        navigation_state_mod = SAFE;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance_mod(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction_mod = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction_mod * oag_heading_rate_mod);
  } else {
    avoidance_heading_direction_mod = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction_mod * oag_heading_rate_mod);
  }
  return false;
}
