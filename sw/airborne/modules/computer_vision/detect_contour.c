/*
 * Copyright (C) Roland Meertens and Peng Lu
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/detect_contour.c"
 * @author Roland Meertens and Peng Lu
 *
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/detect_contour.h"

#include "modules/computer_vision/opencv_contour.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>


#include <stdio.h>
#include "modules/wedgebug/wedgebug.h"
#include "modules/wedgebug/wedgebug_opencv.h"
#include "modules/computer_vision/cv.h" // Required for the "cv_add_to_device" function
#include "modules/computer_vision/lib/vision/image.h"// For image-related structures
#include "pthread.h"
#include <stdint.h> // Needed for common types like uint8_t
#include "state.h"
#include "math/pprz_algebra_float.h"// Needed for vector operations, Euclidean distance, FloatVect3 and FloatRMat
#include "math/pprz_algebra.h"// Needed for vector operations (simple subtraction)
#include "math/pprz_geodetic_float.h"// Needed for NedCoor_f
#include "generated/flight_plan.h" // Needed for WP (waypoint) functions and WP IDs (as defined in "ralphthesis2020_stereo_cyberzoo.xml")
#include "firmwares/rotorcraft/autopilot_guided.h" // Needed for guidance functions such as "autopilot_guided_goto_ned" and "guidance_h_set_guided_heading"
#include <math.h> // needed for basic math functions
#include "autopilot.h" // Needed to set states (GUIDED vs NAV)
#include <time.h> // Needed to measure time






#ifndef DETECT_CONTOUR_FPS
#define DETECT_CONTOUR_FPS 4       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(DETECT_CONTOUR_FPS)
#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

// Function
struct image_t *contour_func(struct image_t *img);
struct image_t *contour_func(struct image_t *img)
{

  if (img->type == IMAGE_YUV422) {
    // Call OpenCV (C++ from paparazzi C function)
    find_contour((char *) img->buf, img->w, img->h);

  }
  return img;
}
void detect_contour_periodic(void)
{
    PRINT("We are in");
}
void detect_contour_init(void)
{
  cv_add_to_device(&DETECT_CONTOUR_CAMERA, contour_func, DETECT_CONTOUR_FPS);
  // in the mavlab, bright
  cont_thres.lower_y = 16;  cont_thres.lower_u = 135; cont_thres.lower_v = 80;
  cont_thres.upper_y = 100; cont_thres.upper_u = 175; cont_thres.upper_v = 165;
  //
  // for cyberzoo: Y:12-95, U:129-161, V:80-165, turn white.
  //int y1=16;  int u1=129; int v1=80; % cyberzoo, dark
  //int y2=100; int u2=161; int v2=165;
}

