/*
 * Copyright (C) 2016 - IMAV 2016
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file modules/computer_vision/flight_plan_guided.c
 * @author IMAV 2016
 */

#include "modules/flight_plan_guided/flight_plan_guided.h"
#include "subsystems/ins.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>
#include <time.h>

#include "modules/computer_vision/marker/detector.h"

#include "mcu_periph/uart.h"
#include "modules/stereocam/stereocam.h"
#include "modules/stereocam/stereoprotocol.h"
#include "modules/stereocam/stereocam2state/stereocam2state.h"


bool marker_lost;

void flight_plan_guided_init(void) {
  marker_lost = true;
} // Dummy


/* Kill throttle */
uint8_t KillEngines(void) {
    autopilot_set_motors_on(FALSE);

    return false;
}


/* Start throttle */
uint8_t StartEngines(void) {
    autopilot_set_motors_on(TRUE);

    return false;
}


/* Reset the altitude reference to the current GPS alt if GPS is used */
uint8_t ResetAlt(void) {ins_reset_altitude_ref(); return false;}

bool TakeOff(float climb_rate) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    guidance_v_set_guided_vz(-climb_rate);
    guidance_h_set_guided_body_vel(0, 0);

    return false;
}

bool WaitUntilAltitude(float altitude) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    if (stateGetPositionEnu_f()->z < altitude) { return true; }

    return false;
}

bool RotateToHeading(float heading) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  guidance_h_set_guided_heading(heading);

  return false;
}

uint8_t Hover(float altitude) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }
    // Horizontal velocities are set to zero
    guidance_h_set_guided_body_vel(0, 0);
//    guidance_v_set_guided_z(-altitude);
    guidance_v_set_guided_vz(0);

    return false;
}

/* Move forward */
uint8_t MoveForward(float vx) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    if (autopilot_mode == AP_MODE_GUIDED) {
        guidance_h_set_guided_body_vel(vx, 0);
    }
    return false;
}

/* Move Right */
uint8_t MoveRight(float vy) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    if (autopilot_mode == AP_MODE_GUIDED) {
        guidance_h_set_guided_body_vel(0, vy);
    }
    return false;
}



bool Land(float end_altitude) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    // return true if not completed

    //For bucket
    guidance_v_set_guided_vz(0.4);
    //For landing pad
//    guidance_v_set_guided_vz(1.5);
    guidance_h_set_guided_pos(marker1.geo_location.x, marker1.geo_location.y);

    if (stateGetPositionEnu_f()->z > end_altitude) {
        return true;
    }

    return false;
}

bool front_cam_set_x_offset(int offset) {
  mt9f002.offset_x = offset;
  mt9f002_set_resolution(&mt9f002);

  return false;
}

static int BUCKET_HEADING_MARGIN = 60;  // px
static float BUCKET_HEADING_RATE = 0.5; // rad/s

bool bucket_heading_change(float altitude) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }
//  guidance_v_set_guided_z(-altitude);
//  guidance_h_set_guided_body_vel(0., 0.);

  if (marker2.detected) {
    marker_lost = false;
    if (!marker2.processed) {
      // Marker detected and not processed
      marker2.processed = true;
      int relative_heading = marker2.pixel.y - 320;

      if (relative_heading > BUCKET_HEADING_MARGIN) {
        // Marker is to the right
        guidance_h_set_guided_heading_rate(BUCKET_HEADING_RATE);
      } else if (relative_heading < -BUCKET_HEADING_MARGIN) {
        // Marker is to the left
        guidance_h_set_guided_heading_rate(-BUCKET_HEADING_RATE);
      } else {
        // Marker is more or less centered
        guidance_h_set_guided_heading_rate(0.);
        return false;
      }
    } else {
      // Marker detected but already processed
      // ** just wait **
    }
  } else {
    // Marker not detected
    guidance_h_set_guided_heading_rate(BUCKET_HEADING_RATE);
  }

  return true;
}

static int BUCKET_POSITION_MARGIN = 100; // > 50 SO DISABLED
static int BUCKET_POSITION_MARGIN_LOST = 50;
static float BUCKET_DRIFT_CORRECTION_RATE = 0.05;
static float BUCKET_APPROACH_SPEED_HIGH = 0.2;
static float BUCKET_APPROACH_SPEED_LOW = 0.05;

bool bucket_approach(float altitude) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }
//  guidance_v_set_guided_z(-altitude);

  if (marker1.detected) {
    // Hand over control to next stage
    return false;
  }

  if (marker2.detected) {
    marker_lost = false;
    if (!marker2.processed) {
      marker2.processed = true;
      int relative_pos = marker2.pixel.y - 320;

      if (abs(relative_pos) > BUCKET_POSITION_MARGIN_LOST) {
        fprintf(stderr, "[bucket] OUT OF FRAME.\n");
        marker_lost = true;
      } else if (relative_pos > BUCKET_POSITION_MARGIN) {
        fprintf(stderr, "[bucket] RIGHT.\n");
        guidance_h_set_guided_body_vel(BUCKET_APPROACH_SPEED_LOW, BUCKET_DRIFT_CORRECTION_RATE);
      } else if (relative_pos < -BUCKET_POSITION_MARGIN) {
        fprintf(stderr, "[bucket] LEFT.\n");
        guidance_h_set_guided_body_vel(BUCKET_APPROACH_SPEED_LOW, -BUCKET_DRIFT_CORRECTION_RATE);
      } else {
        fprintf(stderr, "[bucket] CENTER.\n");
        guidance_h_set_guided_body_vel(BUCKET_APPROACH_SPEED_HIGH, 0.0);
      }
    } else {
      fprintf(stderr, "[bucket] ALREADY PROCESSED.\n");
      // Marker detected but already processed
      // ** just wait **
    }
  } else {
    fprintf(stderr, "[bucket] NOT DETECTED.\n");
    // Marker not detected
    marker_lost = true;
  }

  if (marker_lost) {
    guidance_h_set_guided_body_vel(0., 0.);
  }

  // Loop this function
  return true;
}


#define ERR_MIN 0.05
#define ERR_MAX 0.30
#define ERR_GAIN (1.0 / (ERR_MAX - ERR_MIN))

bool marker_center_land(float x_offset, float z_speed, float end_altitude) {
  if (autopilot_mode != AP_MODE_GUIDED) { return true; }

  if (end_altitude != 0 && stateGetPositionEnu_f()->z < end_altitude) {
      return false;
  }

  struct Marker *marker = &marker1;

  guidance_h_set_guided_heading_rate(0.);

  if (marker->detected) {
    if (!marker->processed) {
      marker->processed = true;

      float psi = stateGetNedToBodyEulers_f()->psi;

      float offset_x = cosf(-psi) * x_offset;
      float offset_y = sinf(-psi) * x_offset;

      float rel_x = marker->geo_relative.x + offset_x;
      float rel_y = marker->geo_relative.y + offset_y;

      // err < 0.05 = acceptable when landed
      // err < 0.10 = acceptable at low altitude
      // err < 0.30 = caution when landing
      // err > 0.30 = NOT GOOD
      // err > 0.50 = probably lost marker
      float err = fabsf(rel_x) + fabsf(rel_y);

      if (z_speed != 0) {
//      fprintf(stderr, "[landing] %.2f, %.2f, %.2f\n", rel_x, rel_y, err);
        float bounded_err = Chop(err, ERR_MIN, ERR_MAX);
        guidance_v_set_guided_vz(z_speed - z_speed * (bounded_err - ERR_MIN) * ERR_GAIN);
      }

      float pos_x = marker->geo_location.x + offset_x;
      float pos_y = marker->geo_location.y - offset_y;

      guidance_h_set_guided_pos(pos_x, pos_y);
    }
  }

  // Loop this function
  return true;
}

bool open_gripper(void) {
  uint8_t msg[1]; msg[0] = 0;
  stereoprot_sendArray(&((UART_LINK).device), msg, 1, 1);
  return false;
}

bool close_gripper(void) {
  uint8_t msg[1]; msg[0] = 1;
  stereoprot_sendArray(&((UART_LINK).device), msg, 1, 1);
  return false;
}

int8_t win_state;

bool fly_through_window(void) {
  static struct FloatVect3 window_loc;
  static float window_dist = 0;
  static float mytime = 0;

  if (autopilot_mode != AP_MODE_GUIDED) { win_state = 0; return true; }

  // TODO window lost recovery
  //if (!win_processed) { // this will limit updates to the speed of the stereocam ~12Hz
  //  win_processed = 1;
    switch (win_state){
      case 0:
        guidance_h_set_guided_pos_relative(stateGetPositionNed_f()->x, stateGetPositionNed_f()->y);
        guidance_v_set_guided_z(-1.2);
        mytime = get_sys_time_float();
        init_pos_filter = 1;
        win_state++;
        break;
      // yaw to get parallel with wall
      case 1:
        printf("disp_diff %d\n", disp_diff);
        if(disp_diff > 1){
          guidance_h_set_guided_heading_rate(-0.5);
          mytime = get_sys_time_float();
        } else if ( disp_diff < -1 ) {
          guidance_h_set_guided_heading_rate(0.5);
          mytime = get_sys_time_float();
        } else {
          guidance_h_set_guided_heading(stateGetNedToBodyEulers_f()->psi);
        }
        if (get_sys_time_float() - mytime > 3.){
          win_state++;
          printf("State advancing to window centering\n");

          // reinitialise filter
          window_loc.x = 3.;
          window_loc.y = 0.;
          window_loc.z = 0.;
        }
        break;
      // centre drone in front of window at about 2m away
      case 2:
        if(gate_detected && gate_processed == 0) {
          printf("going to %f %f\n\n", gate_x_dist - 2., gate_y_dist);

          // position drone 2m in front of window
          guidance_h_set_guided_pos_relative(gate_x_dist - 2., gate_y_dist);
          //guidance_v_set_guided_z(stateGetPositionNed_f()->z - filtered_z_gate);
          gate_processed = 1;

          if (ready_pass_through){
            printf("State advancing to window fly through\n");
            win_state++;
          }
        }

        /*if ( win_cert < 70 ){
          // lowpass filter window geolocation
          window_dist = (float)win_dist / 100.;
          window_loc.x += 0.7 * (window_dist - window_loc.x);
          window_loc.y += 0.7 * ((win_x - 64) * window_dist / 50 - window_loc.y); // assume focal length is 20px
          window_loc.z += 0.7 * ((win_y - 48) * window_dist / 50 - window_loc.z); // down is positive

          printf("guided: %f %f %f %d\n", window_loc.x, window_loc.y, window_loc.z, win_counter);

          if(win_counter++ > 12){ // replace 1s
            guidance_h_set_guided_pos_relative(window_loc.x - 2., window_loc.y);
          }
        } else {
          win_counter -= 2;
        }
        // if position error is small
        if (fabs(window_loc.x - 2) < 0.2 && fabs(window_loc.y) < 0.2 && win_counter > 36 &&
            stateGetSpeedNed_f()->x < 0.1 && stateGetSpeedNed_f()->y < 0.1) {
          win_state++;
          printf("State advancing to window fly through\n");
          win_counter = 0;
        }
        if (win_counter < 0){
          win_counter = 0;
        }*/
        break;
      // fly forward with active control till <2m in front of window
      case 3:
        if(gate_detected){
          guidance_h_set_guided_pos_relative(gate_x_dist + 0.5, gate_y_dist);
        } else {
          mytime = get_sys_time_float();
          win_state++;
        }
        break;
      case 4:
        if (stateGetSpeedNed_f()->x < 0.1 && stateGetSpeedNed_f()->y < 0.1 &&
            get_sys_time_float() - mytime > 5.) {
          init_pos_filter = 1;
          win_state = 0;
          printf("Window fly through complete\n");
          return false;
        }
        break;
      default:
        mytime = get_sys_time_float();
        win_state = 0;
        break;
    }
  //}

  return true;
}
