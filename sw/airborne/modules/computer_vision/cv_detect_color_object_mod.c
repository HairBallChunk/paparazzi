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
 * @file modules/computer_vision/cv_detect_object_mod.h
 * Assumes the object consists of a continuous color and checks
 * if you are over the defined object or not
 */

// headers
#include "modules/computer_vision/cv_detect_color_object_mod.h"
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

// define a print statement that can be used throughout the code
#define PRINT(string,...) fprintf(stderr, "[object_detector->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if OBJECT_DETECTOR_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// defining the mutex variable
static pthread_mutex_t mutex;

// setting variable for frame update (should also be defined in airframe.xml)
#ifndef OFFSET_DETECTOR_FPS1
#define OFFSET_DETECTOR_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif

//Burhan filter settings:
uint8_t Green_percentage[520];
uint8_t R_green_low = 60, G_green_low = 70, B_green_low = 0; // Lower = [65,20,5]
uint8_t R_green_hi = 100, G_green_hi = 200, B_green_hi = 45; // Higher = [95,255,95]
uint8_t gray_threshold = 20;
uint8_t thresh_lower = 0, thresh_upper = 120;
uint16_t STEP = 1;
uint16_t MAX_image_height = 120;
uint16_t filter_height1 = 0;
bool draw = false;

// structure to store filter results
struct offset_detected{
  uint32_t offset;
  bool updated;
};
struct offset_detected offset_filter[1];

// we want this to be a variable called px_offset probably uint32_t like above to sore px_offset instead of count
// Function for filter initialization
uint32_t Burhan_filter(struct image_t *img, uint8_t *Green_percentage, bool draw,
                   uint8_t R_green_low, uint8_t G_green_low, uint8_t B_green_low,
                   uint8_t R_green_hi, uint8_t G_green_hi, uint8_t B_green_hi, uint8_t gray_threshold);

/*
 * offset_detector
 * @param img - input image to process
 * @return img
 */
// this detector specifies the filtered image such that there exists a pointer that can be bound to the current frame
static struct image_t *offset_detector(struct image_t *img)
{
 // function for filter periodic
 uint32_t px_offset = Burhan_filter(img, &Green_percentage, draw, R_green_low, G_green_low, B_green_low,
                   R_green_hi, G_green_hi, B_green_hi, filter_height1);

 VERBOSE_PRINT("px_offset: %d", px_offset);

 pthread_mutex_lock(&mutex);
 offset_filter[0].offset = px_offset;
 offset_filter[0].updated = true;
 pthread_mutex_unlock(&mutex);

  return img;
}



// pointer to be used for video thread
struct image_t *offset_detector1(struct image_t *img);
struct image_t *offset_detector1(struct image_t *img)
{
  return offset_detector(img);
}



void offset_detector_init(void)
{
memset(offset_filter, 0, sizeof(struct offset_detected));
pthread_mutex_init(&mutex, NULL);

#ifdef OFFSET_DETECTOR_CAMERA1    // also defined in airframe
  cv_add_to_device(&OFFSET_DETECTOR_CAMERA1, offset_detector1, OFFSET_DETECTOR_FPS1);
#endif
}



void offset_detector_periodic(void)
{
// if this does not work, check pdf for make clean and make
   static struct offset_detected local_filter[1];
   pthread_mutex_lock(&mutex);
   memcpy(local_filter, offset_filter, sizeof(struct offset_detected));
   pthread_mutex_unlock(&mutex);
   if(local_filter[0].updated){
    AbiSendMsgOFFSET_DETECTION(OFFSET_DETECTION1_ID, local_filter[0].offset);
    local_filter[0].updated = false;
  }
}

/*
 * Burhan filter
 *
 * returns an offset related to the heading that we want
 *
 * @param img - input image to process formatted as YUV422.
 * etc.
 */
uint32_t Burhan_filter(struct image_t *img, uint8_t *Green_percentage, bool draw,
                  uint8_t R_green_low, uint8_t G_green_low, uint8_t B_green_low,
                  uint8_t R_green_hi, uint8_t G_green_hi, uint8_t B_green_hi, uint8_t gray_threshold)
{
    uint16_t size = 0;
    uint16_t ones_count = 0;
    uint16_t wrong_zeros = 0;
    uint8_t flag_start = 0, flag_zero = 0;
    uint8_t end_loop = 120;

    uint32_t cnt = 0;
    uint32_t px_offset = 0;
    uint8_t *buffer = img->buf;
    uint16_t next_y_value = 0;
    uint16_t bin_array[img->w];
    for (uint16_t y = 0; y < img->h; y++) {
        cnt = 0;
        for (uint16_t x = 0; x < img->w; x ++) {
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
//            fprintf(stderr,"I AM HERE %d \n",bin_array[y]);
        }
        Green_percentage++;
        if(y == next_y_value) {
            wrong_zeros = 0;
            ones_count = 0;
            int count2;
//            uint16_t size = find_continous_zeros( &bin_array, bin_array_length, thresh_lower, thresh_upper);
            for (int k = MAX_image_height; k < img->w ; k ++) {
                uint8_t *yp;
                count2 = img->w - k;
                if (bin_array[count2] == 0) {
                    if (ones_count > 0) {
                        wrong_zeros ++;
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
            px_offset += 1;
            return px_offset;
        }
    }
}



