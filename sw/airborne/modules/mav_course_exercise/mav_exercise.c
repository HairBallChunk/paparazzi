/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the navigation mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 */

#include "mav_exercise.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "subsystems/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string,...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)


static uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
static uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
static uint8_t increase_nav_heading(float incrementDegrees);
static uint8_t chooseRandomIncrementAvoidance(void);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

// define settings
float oa_color_count_frac = 0.18f;

// define and initialise global variables
enum navigation_state_t navigation_state = SAFE;
int32_t color_count;                // orange color count from color filter for obstacle detection
int16_t filter_height;
int16_t filter_width;
float divergence_value_exercise;  //divergence value needed for the obstacle avoidance algorithm
int16_t obstacle_free_confidence;   // a measure of how certain we are that the way ahead is safe.
float heading_increment = 15.f;//5.f;          // heading angle increment [deg]
float maxDistance = 1.2f;               // max waypoint displacement [m]

const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free
uint32_t Section_max_idx;
float green_fraction_local;
uint8_t failsafe_obstacle_bool;
int count_obstacle_found = 0;
int count_avoidance = 4;
/*
 * This next section defines an ABI messaging event (http://wiki.paparazziuav.org/wiki/ABI), necessary
 * any time data calculated in another module needs to be accessed. Including the file where this external
 * data is defined is not enough, since modules are executed parallel to each other, at different frequencies,
 * in different threads. The ABI event is triggered every time new data is sent out, and as such the function
 * defined in this file does not need to be explicitly called, only bound in the init function
 */
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
uint8_t moveWaypoint_offset2waypoint(uint8_t waypoint, float distanceMeters, float d_heading);



#ifndef BURHAN_FILTER_ABI_ID
#define BURHAN_FILTER_ABI_ID ABI_BROADCAST
#endif

static abi_event burhan_filter_ev;

static void burhan_filter_cb(uint8_t __attribute__((unused)) sender_id,
                             uint32_t __attribute__((unused)) Max_section_idx,
                             float __attribute__((unused)) green_fraction,
                             uint8_t __attribute__((unused)) failsafe_obstacle) {
    Section_max_idx = Max_section_idx;
    green_fraction_local = green_fraction;
    failsafe_obstacle_bool = failsafe_obstacle;
//    fprintf(stderr, "FAILSAFE IS = %d \n",failsafe_obstacle);
}
/*
 * Initialisation function, setting the colour filter, random seed and heading_increment
 */
void mav_exercise_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
//  AbiBindMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, &optical_flow_ev, optical_flow_cb);
  AbiBindMsgBURHAN_FILTER(BURHAN_FILTER_ABI_ID, &burhan_filter_ev, burhan_filter_cb);
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void mav_exercise_periodic(void)
{
  // only evaluate our state machine if we are flying
  if(!autopilot_in_flight()){
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

//  VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);

  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);
  float N_bins = (float) sections; //n_bins
  float n_offset = Section_max_idx * 1.f; // px_offset cast to float;
  float fov_h_heading_calc = 80.0f * 3.14f / 180.0f; // estimated horizontal field of view in rad;
  float d_heading = 0; //rad
  int d_heading_deg = 0; //deg
//  float green_fraction_threshold = 0.2;

  switch (navigation_state) {
      case SAFE:

          d_heading = (2 * n_offset - N_bins - 1) / (N_bins + 1) * fov_h_heading_calc/2; // convert the bin idx to a heading in rad
          // Move waypoint forward
          moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
          if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY))) {
              navigation_state = OUT_OF_BOUNDS;
          } else if (failsafe_obstacle_bool) {
              navigation_state = OBSTACLE_FOUND;
              count_obstacle_found = 0;
          } else {
              moveWaypointForward(WP_GOAL, moveDistance);
              d_heading_deg = (int) (d_heading * 180. / 3.14);
              increase_nav_heading(d_heading_deg);
              PRINT("SAFE->ELSE: n_offset/d_heading = %f/%i[deg]\n", n_offset, d_heading_deg);
              PRINT("SAFE->ELSE: Section_max_idx = %d \n", Section_max_idx);
          }
          break;

      case OBSTACLE_FOUND:
          PRINT("OBSTACLE FOUND: FAILSAFE OBSTACLE VALUE IS = %d \n", failsafe_obstacle_bool);

          moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);

          //Just adjust your heading
          if (failsafe_obstacle_bool) {
              d_heading = (2 * n_offset - N_bins - 1) / (N_bins + 1) * fov_h_heading_calc / 2;
          }

          count_obstacle_found++;

          if (count_obstacle_found > count_avoidance) {
              navigation_state = SAFE;
          }
          if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_GOAL))) {
              navigation_state = OUT_OF_BOUNDS;
          }

          if (failsafe_obstacle_bool) {
              d_heading_deg = (int) (d_heading * 180. / 3.14);
              increase_nav_heading(d_heading_deg);
          }

          moveWaypointForward(WP_GOAL, moveDistance);

      break;
    case SEARCH_FOR_SAFE_HEADING:
		PRINT("SEARCH_FOR_SAFE_HEADING: FAILSAFE OBSTACLE VALUE IS = %d \n",failsafe_obstacle_bool);
      increase_nav_heading(heading_increment);

      // make sure we have a couple of good readings before declaring the way safe
      if (!failsafe_obstacle_bool){
        navigation_state = SAFE;
      }
      break;
    case OUT_OF_BOUNDS:
    	PRINT("OUT_OF_BOUNDS\n");
      increase_nav_heading(heading_increment);
      moveWaypointForward(WP_TRAJECTORY, 1.5f);

      if (InsideObstacleZone(WaypointX(WP_TRAJECTORY),WaypointY(WP_TRAJECTORY))){
        // add offset to head back into arena
        increase_nav_heading(heading_increment);
        // reset safe counter
        obstacle_free_confidence = 0;
        // ensure direction is safe before continuing
        navigation_state = SEARCH_FOR_SAFE_HEADING;
      }
      break;
    default:
      break;
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(float incrementDegrees)
{
  float new_heading = stateGetNedToBodyEulers_f()->psi + RadOfDeg(incrementDegrees);

  // normalize heading to [-pi, pi]
  FLOAT_ANGLE_NORMALIZE(new_heading);

  // set heading, declared in firmwares/rotorcraft/navigation.h
  // for performance reasons the navigation variables are stored and processed in Binary Fixed-Point format
  nav_heading = ANGLE_BFP_OF_REAL(new_heading);

//  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(new_heading));
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  float heading  = stateGetNedToBodyEulers_f()->psi;

  // Now determine where to place the waypoint you want to go to
  new_coor->x = stateGetPositionEnu_i()->x + POS_BFP_OF_REAL(sinf(heading) * (distanceMeters));
  new_coor->y = stateGetPositionEnu_i()->y + POS_BFP_OF_REAL(cosf(heading) * (distanceMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_move_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Sets the variable 'heading_increment' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    heading_increment = heading_increment;
//    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  } else {
    heading_increment = -heading_increment;
//    VERBOSE_PRINT("Set avoidance increment to: %f\n", heading_increment);
  }
  return false;
}

