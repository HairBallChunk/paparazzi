/*
 * Copyright (C) 2019 Kirk Scheper <kirkscheper@gmail.com>
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
 * @file modules/computer_vision/cv_detect_object.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

//#include "modules/computer_vision/cv.h"
//#include "modules/computer_vision/detect_contour.h"
//#include "modules/computer_vision/opencv_contour.h"

// Own header
#include "modules/computer_vision/cv_detect_color_object_mod.h"
//#include "modules/computer_vision/cv.h"
#include "subsystems/abi.h"
#include "std.h"
#include "opencv_example.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pthread.h"
#include "stdlib.h"
#include "subsystems/datalink/downlink.h"
float debug_values[4];

#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

#ifndef COLOR_OBJECT_DETECTOR_FPS1
#define COLOR_OBJECT_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif
#ifndef COLOR_OBJECT_DETECTOR_FPS2
#define COLOR_OBJECT_DETECTOR_FPS2 0 ///< Default FPS (zero means run at camera fps)
#endif

// Filter Settings
float object_detector_sensitivity = 0.18;
uint8_t cod_lum_min1 = 0;
uint8_t cod_lum_max1 = 0;
uint8_t cod_cb_min1 = 0;
uint8_t cod_cb_max1 = 0;
uint8_t cod_cr_min1 = 0;
uint8_t cod_cr_max1 = 0;
uint16_t filter_height1 = 0;
uint16_t filter_width1 = 0;

uint8_t cod_lum_min2 = 0;
uint8_t cod_lum_max2 = 0;
uint8_t cod_cb_min2 = 0;
uint8_t cod_cb_max2 = 0;
uint8_t cod_cr_min2 = 0;
uint8_t cod_cr_max2 = 0;
uint16_t filter_width2 = 0;
uint16_t filter_height2 = 0;

bool cod_draw1 = false;
bool cod_draw2 = false;

//Burhan filter settings:
uint8_t R_green_low = 60, G_green_low = 70, B_green_low = 0; // Lower = [65,20,5]
uint8_t R_green_hi = 100, G_green_hi = 200, B_green_hi = 45; // Higher = [95,255,95]
uint8_t gray_threshold = 20;
uint16_t STEP = 20;
uint8_t filter_height_cut = 120;
uint8_t thresh_lower = 5;
uint8_t sections = 13; //NOTICE, IT APPROXIMATE TO THE CLOSEST INTEGER!!!!!
float window_scale = 0.2;

// define global variables
struct color_object_t{
  int32_t x_c;
  int32_t y_c;
  uint32_t color_count;
  uint16_t filter_height;
  uint16_t filter_width;
  bool updated;
};
struct color_object_t global_filters[2];

// Function
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint16_t filter_height, uint16_t filter_width);

uint16_t Burhan_filter(struct image_t *img, bool draw,
                   uint8_t R_green_low, uint8_t G_green_low, uint8_t B_green_low,
                   uint8_t R_green_hi, uint8_t G_green_hi, uint8_t B_green_hi, uint8_t gray_threshold,
                   uint8_t thresh_lower, uint8_t filter_height_cut, uint8_t sections, float window_scale);

/*
 * object_detector
 * @param img - input image to process
 * @param filter - which detection filter to process
 * @return img
 */
static struct image_t *object_detector(struct image_t *img, uint8_t filter)
{
  uint8_t lum_min, lum_max;
  uint8_t cb_min, cb_max;
  uint8_t cr_min, cr_max;
  uint16_t filter_height, filter_width;
  bool draw;


  switch (filter){
    case 1:
      lum_min = cod_lum_min1;
      lum_max = cod_lum_max1;
      cb_min = cod_cb_min1;
      cb_max = cod_cb_max1;
      cr_min = cod_cr_min1;
      cr_max = cod_cr_max1;
      draw = cod_draw1;
      filter_height = filter_height1;
      filter_width = filter_width1;
      break;
    case 2:
      lum_min = cod_lum_min2;
      lum_max = cod_lum_max2;
      cb_min = cod_cb_min2;
      cb_max = cod_cb_max2;
      cr_min = cod_cr_min2;
      cr_max = cod_cr_max2;
      draw = cod_draw2;
      filter_height = filter_height2;
      filter_width = filter_width2;
      break;
    default:
      return img;
  };

  int32_t x_c, y_c;

  // Filter and find centroid
  uint32_t count = find_object_centroid(img, &x_c, &y_c, draw, lum_min, lum_max, cb_min, cb_max, cr_min, cr_max, filter_height, filter_width);
  //Using the Burhan filter
//  Burhan_filter(img, &Green_percentage, draw, R_green_low, G_green_low, B_green_low,
//                  R_green_hi, G_green_hi, B_green_hi, gray_threshold);


    uint16_t max_idx = Burhan_filter(img, draw, R_green_low, G_green_low, B_green_low,
                  R_green_hi, G_green_hi, B_green_hi, filter_height, thresh_lower, filter_height_cut, sections, window_scale);

    fprintf(stderr,"OUR MAX SECTION IS : %d \n ",max_idx );
  VERBOSE_PRINT("Color count %d: %u, threshold %u, x_c %d, y_c %d\n", camera, object_count, count_threshold, x_c, y_c);
  VERBOSE_PRINT("centroid %d: (%d, %d) r: %4.2f a: %4.2f\n", camera, x_c, y_c,
        hypotf(x_c, y_c) / hypotf(img->w * 0.5, img->h * 0.5), RadOfDeg(atan2f(y_c, x_c)));

  pthread_mutex_lock(&mutex);
  global_filters[filter-1].color_count = count;
  global_filters[filter-1].x_c = x_c;
  global_filters[filter-1].y_c = y_c;
  global_filters[filter-1].filter_height = filter_height;
  global_filters[filter-1].filter_width = filter_width;
  global_filters[filter-1].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *object_detector1(struct image_t *img);
struct image_t *object_detector1(struct image_t *img)
{
  return object_detector(img, 1);
}

struct image_t *object_detector2(struct image_t *img);
struct image_t *object_detector2(struct image_t *img)
{
  return object_detector(img, 2);
}

void color_object_detector_init(void)
{
  memset(global_filters, 0, 2*sizeof(struct color_object_t));
  pthread_mutex_init(&mutex, NULL);
#ifdef COLOR_OBJECT_DETECTOR_CAMERA1
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN1
  cod_lum_min1 = COLOR_OBJECT_DETECTOR_LUM_MIN1;
  cod_lum_max1 = COLOR_OBJECT_DETECTOR_LUM_MAX1;
  cod_cb_min1 = COLOR_OBJECT_DETECTOR_CB_MIN1;
  cod_cb_max1 = COLOR_OBJECT_DETECTOR_CB_MAX1;
  cod_cr_min1 = COLOR_OBJECT_DETECTOR_CR_MIN1;
  cod_cr_max1 = COLOR_OBJECT_DETECTOR_CR_MAX1;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW1
  cod_draw1 = COLOR_OBJECT_DETECTOR_DRAW1;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA1, object_detector1, COLOR_OBJECT_DETECTOR_FPS1);
#endif

#ifdef COLOR_OBJECT_DETECTOR_CAMERA2
#ifdef COLOR_OBJECT_DETECTOR_LUM_MIN2
  cod_lum_min2 = COLOR_OBJECT_DETECTOR_LUM_MIN2;
  cod_lum_max2 = COLOR_OBJECT_DETECTOR_LUM_MAX2;
  cod_cb_min2 = COLOR_OBJECT_DETECTOR_CB_MIN2;
  cod_cb_max2 = COLOR_OBJECT_DETECTOR_CB_MAX2;
  cod_cr_min2 = COLOR_OBJECT_DETECTOR_CR_MIN2;
  cod_cr_max2 = COLOR_OBJECT_DETECTOR_CR_MAX2;
#endif
#ifdef COLOR_OBJECT_DETECTOR_DRAW2
  cod_draw2 = COLOR_OBJECT_DETECTOR_DRAW2;
#endif

  cv_add_to_device(&COLOR_OBJECT_DETECTOR_CAMERA2, object_detector2, COLOR_OBJECT_DETECTOR_FPS2);
#endif
}

/*
 * find_object_centroid
 *
 * Finds the centroid of pixels in an image within filter bounds.
 * Also returns the amount of pixels that satisfy these filter bounds.
 *
 * @param img - input image to process formatted as YUV422.
 * @param p_xc - x coordinate of the centroid of color object
 * @param p_yc - y coordinate of the centroid of color object
 * @param lum_min - minimum y value for the filter in YCbCr colorspace
 * @param lum_max - maximum y value for the filter in YCbCr colorspace
 * @param cb_min - minimum cb value for the filter in YCbCr colorspace
 * @param cb_max - maximum cb value for the filter in YCbCr colorspace
 * @param cr_min - minimum cr value for the filter in YCbCr colorspace
 * @param cr_max - maximum cr value for the filter in YCbCr colorspace
 * @param draw - whether or not to draw on image
 * @return number of pixels of image within the filter bounds.
 */
uint32_t find_object_centroid(struct image_t *img, int32_t* p_xc, int32_t* p_yc, bool draw,
                              uint8_t lum_min, uint8_t lum_max,
                              uint8_t cb_min, uint8_t cb_max,
                              uint8_t cr_min, uint8_t cr_max, uint16_t filter_height, uint16_t filter_width)
{
  uint32_t cnt = 0;
  uint32_t tot_x = 0;
  uint32_t tot_y = 0;
  uint8_t *buffer = img->buf;

//  opencv_example( &buffer, img->w , img->h );

//    find_contour((char *) img->buf, img->w, img->h);

  // Select only the right pixels defined by the filter_height and filter_width variables
  uint16_t img_width_original = img->w;
  uint16_t img_height_original = img->h;
  uint16_t min_y = round((img_height_original-filter_height) * 0.5f );
  uint16_t min_x = round((img_width_original-filter_width) * 0.5f );
  uint16_t max_y = min_y + filter_height;
  uint16_t max_x = min_x + filter_width;
  for (uint16_t y = min_y; y < max_y; y++) {
    for (uint16_t x = min_x; x < max_x; x ++) {
      // Check if the color is inside the specified values
      uint8_t *yp, *up, *vp;
      if (x % 2 == 0) {
        // Even x
        up = &buffer[y * 2 * img->w + 2 * x];      // U
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
        vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
        //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
      } else {
        // Uneven x
        up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
        //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
        vp = &buffer[y * 2 * img->w + 2 * x];      // V
        yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
      }

        if ( (*yp >= lum_min) && (*yp <= lum_max) &&
           (*up >= cb_min ) && (*up <= cb_max ) &&
           (*vp >= cr_min ) && (*vp <= cr_max )) {
        cnt ++;
        tot_x += x;
        tot_y += y;
        if (draw){
          *yp = 255;  // make pixel brighter in image
        }
      }
    }
  }
  if (cnt > 0) {
    *p_xc = (int32_t)roundf(tot_x / ((float) cnt) - filter_width * 0.5f + min_x);
    *p_yc = (int32_t)roundf(filter_height * 0.5f - tot_y / ((float) cnt) + min_y);
  } else {
    *p_xc = 0;
    *p_yc = 0;
  }
  return cnt;
}


uint16_t Burhan_filter(struct image_t *img, bool draw,
                   uint8_t R_green_low, uint8_t G_green_low, uint8_t B_green_low,
                   uint8_t R_green_hi, uint8_t G_green_hi, uint8_t B_green_hi,
                   uint8_t gray_threshold, uint8_t thresh_lower, uint8_t filter_height_cut, uint8_t sections,
                   float window_scale){

    uint16_t size = 0;
    uint16_t ones_count = 0;
    uint16_t wrong_zeros = 0;
    uint8_t *buffer = img->buf;
    uint16_t next_y_value = 0;
    uint16_t bin_array[img->w];
    uint16_t Green_pixel_value[(int) img->h/STEP];
    float Section_value[sections], weight = 0;
    uint16_t idx_section = 0, section_count = 0;
    uint16_t section_value = 0, storage_value = 0, max_idx = 0;
    section_value = (int) img->h/STEP/sections;

    fprintf(stderr,"HEADING VALUES:  ");
    for (uint16_t y = 0; y < img->h; y++) {
        uint8_t cnt = 0;
        uint16_t green_count;
        for (uint16_t x = 0; x < filter_height_cut ; x ++) {
            // Check if the color is inside the specified values
            uint8_t *yp, *up, *vp;
            uint8_t pixel_b, pixel_g, pixel_r, pixel_value_local, pixel_value_local_gray;
            if (x % 2 == 0) {
                // Even x
                up = &buffer[y * 2 * img->w + 2 * x];      // U
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y1
                vp = &buffer[y * 2 * img->w + 2 * x + 2];  // V
                //yp = &buffer[y * 2 * img->w + 2 * x + 3]; // Y2
            } else {
                // Uneven x
                up = &buffer[y * 2 * img->w + 2 * x - 2];  // U
                //yp = &buffer[y * 2 * img->w + 2 * x - 1]; // Y1
                vp = &buffer[y * 2 * img->w + 2 * x];      // V
                yp = &buffer[y * 2 * img->w + 2 * x + 1];  // Y2
            }
            //Transpose YUV in RGB

            pixel_b = (int) fmin(255.f , 1.164f * ( *yp - 16) + 2.018f * ( *up - 128));
            pixel_g = (int) fmin(255.f , 1.164f * ( *yp - 16) - 0.813f * ( *vp - 128) - 0.391f * ( *up - 128));
            pixel_r = (int) fmin(255.f , 1.164f * ( *yp - 16) + 1.596f * ( *vp - 128));

            //CREATE THE BITWISE_AND TOGETHER WITH THE INRANGE FCN
            if ( (pixel_b >= B_green_low) && (pixel_b <= B_green_hi) &&
                 (pixel_g >= G_green_low ) && (pixel_g <= G_green_hi ) &&
                 (pixel_r >= R_green_low ) && (pixel_r <= R_green_hi )) {
//            if ( (*up >= 0) && (*up <= 120) &&
//                 (*vp >= 0 ) && (*vp <= 120 ) &&
//                 (*yp >= 50 ) && (*yp <= 200 )) {
                //IMPLEMENTING THE BGR TO GRAYSCALE
                pixel_value_local_gray = (int) 0.3f * pixel_r + 0.59f * pixel_g + 0.11f * pixel_b;
            }
            else{
                pixel_value_local_gray = 0;
            }
            //IMPLEMENTING THE THRESHOLD FCN
            if(pixel_value_local_gray >= gray_threshold){
                cnt ++;
                bin_array[x] = 1;
                if(draw){
                    *yp = 255; // make pixel brighter in image
                }
            }
            else {
                bin_array[x] = 0;
            }
        }

        // FIND CONTINUOUS ZEROS FUNCTION
        if(y == next_y_value) {
            wrong_zeros = 0;
            ones_count = 0;
            int count2;
            for (int k = filter_height_cut; k < img->w ; k ++) {
                uint8_t *yp;
                count2 = img->w - k;
                if (bin_array[count2] == 0) {
                    if (ones_count > thresh_lower) {
                        wrong_zeros ++;
                        //Write the pixel to image
                        if (count2 % 2 == 0) {
                            // Even x
                            yp = &buffer[y * 2 * img->w + 2 * count2 + 1];
                        } else {
                            // Uneven x
                            yp = &buffer[y * 2 * img->w + 2 * count2 + 1];
                        }
                        *yp = 255;
                    }
                }
                else if (bin_array[count2] == 1) {
                    ones_count ++;
                }
            }
            next_y_value += STEP;
            Green_pixel_value[green_count] = wrong_zeros + cnt;
            green_count++;

            //REFINE THE FILTER INTO BIGGER CELLS
            storage_value += wrong_zeros + cnt;
            section_count++;
            if(section_count == section_value ){
                //Adding the weight
                weight = 1.f - (fabs(idx_section + 1 - ceil(sections*0.5f))/ceil(sections*0.5f)) * window_scale;
                Section_value[idx_section] = weight * storage_value/(section_value*(img->w - filter_height_cut));
                fprintf(stderr," %f ",Section_value[idx_section]);
                storage_value = 0;
                section_count = 0;
                if(Section_value[idx_section] > Section_value[idx_section - 1]){
                    max_idx = idx_section + 1;
                }
                idx_section++;
            }
        }

    }
    fprintf(stderr,".\n ");
    return(max_idx);
}


void color_object_detector_periodic(void)
{
  static struct color_object_t local_filters[2];
  pthread_mutex_lock(&mutex);
  memcpy(local_filters, global_filters, 2*sizeof(struct color_object_t));
  pthread_mutex_unlock(&mutex);

  if(local_filters[0].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
        local_filters[0].filter_width, local_filters[0].filter_height, local_filters[0].color_count, 0);
    local_filters[0].updated = false;
  }
  if(local_filters[1].updated){
    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION2_ID, local_filters[1].x_c, local_filters[1].y_c,
        local_filters[0].filter_width, local_filters[0].filter_height, local_filters[1].color_count, 1);
    local_filters[1].updated = false;
  }
}
