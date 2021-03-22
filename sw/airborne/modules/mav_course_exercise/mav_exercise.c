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
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include "autopilot_static.h"
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

uint8_t increase_nav_heading(float incrementDegrees);
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  OUT_OF_BOUNDS,
  HOLD,
  TURN_20_DEG_AND_SEARCH
};

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;
int32_t color_count = 0;               // orange color count from color filter for obstacle detection
int16_t filter_height = 0;
int16_t filter_width = 0;
float divergence_value_exercise = 0; //divergence value needed for the obstacle avoidance algorithm
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead is safe.
float moveDistance = .5;                 // waypoint displacement [m]
float oob_haeding_increment = 5.f;      // heading angle increment if out of bounds [deg]
const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free

uint32_t Section_max_idx;

float object_detector_sensitivity = 0.18;

// Filter Settings
float Heading_constant = 0.3;
float green_fraction_local = 0;

//` needed to receive output from a separate module running on a parallel process`
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#define ORANGE_AVOIDER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width,
                               int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra) {
  color_count = quality;
  filter_height = pixel_height;
  filter_width = pixel_width;
}
static uint8_t calculate_offset2waypoint(struct EnuCoor_i *new_coor, float distanceMeters, float d_heading);
uint8_t moveWaypoint_offset2waypoint(uint8_t waypoint, float distanceMeters, float d_heading);

//` needed to receive output from a separate module running on a parallel process (Opticalflow)
#ifndef OPTICAL_FLOW_ID
#define OPTICAL_FLOW_ID ABI_BROADCAST
#endif

static abi_event optical_flow_ev;



static void optical_flow_cb(uint8_t __attribute__((unused)) sender_id,
                            uint32_t __attribute__((unused)) stamp,
                            int16_t __attribute__((unused)) flow_x,
                            int16_t __attribute__((unused)) flow_y,
                            int16_t __attribute__((unused)) flow_der_x,
                            int16_t __attribute__((unused)) flow_der_y,
                            float __attribute__((unused)) quality,
                            float __attribute__((unused)) size_divergent) {
    divergence_value_exercise = size_divergent;
}

#ifndef BURHAN_FILTER_ABI_ID
#define BURHAN_FILTER_ABI_ID ABI_BROADCAST
#endif

static abi_event burhan_filter_ev;

static void burhan_filter_cb(uint8_t __attribute__((unused)) sender_id,
                             uint32_t __attribute__((unused)) Max_section_idx,
                             float __attribute__((unused)) green_fraction) {
    Section_max_idx = Max_section_idx;
    green_fraction_local = green_fraction;
    fprintf(stderr, "The value of green_count IN THE NAVIGATION MODULE is = %f \n",green_fraction_local);
}

void mav_exercise_init(void) {
  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &optical_flow_ev, optical_flow_cb);
  AbiBindMsgBURHAN_FILTER(BURHAN_FILTER_ABI_ID, &burhan_filter_ev, burhan_filter_cb);
}

void mav_exercise_periodic(void) {
  // only evaluate our state machine if we are flying
  if (!autopilot_in_flight()) {
    return;
  }

  // compute current color thresholds
  // front_camera defined in airframe xml, with the video_capture module
  int32_t color_count_threshold = object_detector_sensitivity * filter_height * filter_width;

//  PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);


    PRINT("MAX_IDX_SECTION IS: %d \n", Section_max_idx);
  if(divergence_value_exercise>=0.02){
      PRINT("OPTICAL FLOW VALUE : %f  \n", divergence_value_exercise);
  }
  // update our safe confidence using color threshold
  if (color_count < color_count_threshold) {
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);
float N_bins;
float n_offset;
float fov_h_heading_calc;
float d_heading;

  switch (navigation_state) {
    case SAFE:
    	N_bins = 13.f; //n_bins
    	n_offset = Section_max_idx * 1.f; // px_offset
    	fov_h_heading_calc = 80.0f * 3.14f / 180.0f; // estimated horizontal field of view
    	d_heading = (2*n_offset- N_bins - 1) / (N_bins+1) * fov_h_heading_calc/2;
    	PRINT("SAFE\n");
//    	moveWaypoint_offset2waypoint(WP_TRAJECTORY, 1.5f * moveDistance, d_heading);
//    	increase_nav_heading((int) (d_heading * 180. / 3.14));
      moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

      if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        navigation_state = OUT_OF_BOUNDS;
//      } else if (obstacle_free_confidence == 0) {
//        navigation_state = OBSTACLE_FOUND;
      } else {
        moveWaypointForward(WP_GOAL, moveDistance);
//        moveWaypoint_offset2waypoint(WP_GOAL, 1.5f * moveDistance, d_heading);
        increase_nav_heading((int) (d_heading * 180. / 3.14));
        PRINT("SAFE->ELSE\n");
      }
      break;
    case OBSTACLE_FOUND:
    	PRINT("OBSTACLE FOUND\n");
      // TODO Change behavior
      // stop as soon as obstacle is found
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      navigation_state = TURN_20_DEG_AND_SEARCH;
      break;
    case TURN_20_DEG_AND_SEARCH:
    	PRINT("TURN_20_DEG_AND_SEARCH\n");
          //shift the heading of 10deg and check if the spot is open
          increase_nav_heading(10.f);
          // make sure we have a couple of good readings before declaring the way safe
          if (obstacle_free_confidence >= 2){
              navigation_state = SAFE;
          }
      break;
    case OUT_OF_BOUNDS:
    	PRINT("OUT_OF_BOUNDS\n");
      // stop
      waypoint_move_here_2d(WP_GOAL);
      waypoint_move_here_2d(WP_TRAJECTORY);

      increase_nav_heading(oob_haeding_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
        // add offset to head back into arena
        increase_nav_heading(oob_haeding_increment);
        navigation_state = SAFE;
      }
      break;
    case HOLD:
    default:
      break;
  }
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees) {
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters) {
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));

  return false;
}



/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor) {
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters) {
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}



////////////////////////////////////////////////////////
static uint8_t calculate_offset2waypoint(struct EnuCoor_i *new_coor, float distanceMeters, float d_heading) {
  float heading = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  PRINT("calc_offset2waypoint %f deg heading, %f deg d_heading\n", heading  * 180 / 3.14, d_heading * 180 / 3.14);
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading + d_heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading + d_heading) * (distanceMeters));

  return false;
}

uint8_t moveWaypoint_offset2waypoint(uint8_t waypoint, float distanceMeters, float d_heading) {
  struct EnuCoor_i new_coor;
  calculate_offset2waypoint(&new_coor, distanceMeters, d_heading);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

