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
 * This function applies the burhan filter on the image
 */

// Own header
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



static pthread_mutex_t mutex;

#ifndef BURHAN_FILTER_FPS1
#define BURHAN_FILTER_FPS1 0 ///< Default FPS (zero means run at camera fps)
#endif

//Burhan filter settings: KEEP THEM!!!
uint8_t R_green_low = 60, G_green_low = 70, B_green_low = 0; // Lower = [65,20,5]
uint8_t R_green_hi = 100, G_green_hi = 200, B_green_hi = 45; // Higher = [95,255,95]
uint8_t gray_threshold = 20;
uint16_t STEP = 20;
uint8_t filter_height_cut = 120;
uint8_t thresh_lower = 5;
uint8_t sections = 13; //NOTICE, IT APPROXIMATE TO THE CLOSEST INTEGER!!!!!
float window_scale = 0.2;
uint8_t print_weights = 0;
uint8_t draw_on_img = 1;


struct communicate_msg{
    int32_t section_idx;
    bool updated;
};


struct communicate_msg global_msg[1];

uint32_t Burhan_filter(struct image_t *img, uint8_t draw_on_img,
                   uint8_t R_green_low, uint8_t G_green_low, uint8_t B_green_low,
                   uint8_t R_green_hi, uint8_t G_green_hi, uint8_t B_green_hi, uint8_t gray_threshold,
                   uint8_t thresh_lower, uint8_t filter_height_cut, uint8_t sections,
                   float window_scale,  uint8_t print_weights);


static struct image_t *Burhan_fcn(struct image_t *img, uint8_t filter)
{

  //Using the Burhan filter
  uint32_t max_idx = Burhan_filter(img, draw_on_img, R_green_low, G_green_low, B_green_low,
                  R_green_hi, G_green_hi, B_green_hi, gray_threshold, thresh_lower, filter_height_cut, sections,
                  window_scale, print_weights);


  pthread_mutex_lock(&mutex);
  global_msg[0].section_idx = max_idx;
  global_msg[0].updated = true;
  pthread_mutex_unlock(&mutex);

  return img;
}

struct image_t *burhan_filter1(struct image_t *img);
struct image_t *burhan_filter1(struct image_t *img)
{
  return Burhan_fcn(img, 1);
}


void burhan_filter_init(void)
{

    memset(global_msg, 0, sizeof(struct communicate_msg));
    pthread_mutex_init(&mutex, NULL);

#ifdef BURHAN_FILTER_CAMERA1
  cv_add_to_device(&BURHAN_FILTER_CAMERA1, burhan_filter1, BURHAN_FILTER_FPS1);
#endif
}


uint32_t Burhan_filter(struct image_t *img, uint8_t draw,
                   uint8_t R_green_low, uint8_t G_green_low, uint8_t B_green_low,
                   uint8_t R_green_hi, uint8_t G_green_hi, uint8_t B_green_hi,
                   uint8_t gray_threshold, uint8_t thresh_lower, uint8_t filter_height_cut, uint8_t sections,
                   float window_scale, uint8_t print_weights){

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
    max_idx = -1;
    Section_value[-1] = 0;


    for (uint16_t y = 0; y < img->h; y += STEP) {
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
                        if(draw) {
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
            if(section_count == section_value){
                //Adding the weight
                weight = 1.f - (fabs(idx_section + 1 - ceil(sections*0.5f))/ceil(sections*0.5f)) * window_scale;
                Section_value[idx_section] = weight * storage_value/(section_value*(img->w - filter_height_cut));
                storage_value = 0;
                section_count = 0;
                if(Section_value[idx_section] > Section_value[max_idx] ){
                    max_idx = idx_section;
                }
                idx_section++;
            }
        }

    }
    if(print_weights){
        fprintf(stderr,"WEIGHT FOR EACH SECTION :  ");
        for (int i = 0 ; i < sections ; i++){
            fprintf(stderr," %f ",Section_value[i]);
        }
        fprintf(stderr,".\n ");
        fprintf(stderr,"OUR MAX SECTION IS : %d \n ",max_idx + 1);
    }

    return(max_idx + 1);
}


void burhan_filter_periodic(void)
{

  static struct communicate_msg local_msg[1];
    pthread_mutex_lock(&mutex);
    memcpy( local_msg , global_msg , sizeof(struct communicate_msg));
    pthread_mutex_unlock(&mutex);


  if(local_msg[0].updated){
      //ADD YOUR ABI MESSAGE HERE (FOR MICHIEL)
//    AbiSendMsgVISUAL_DETECTION(COLOR_OBJECT_DETECTION1_ID, local_filters[0].x_c, local_filters[0].y_c,
//        local_filters[0].filter_width, local_filters[0].filter_height, local_filters[0].color_count, 0);
      local_msg[0].updated = false;
  }

}
