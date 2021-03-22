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
#include "modules/computer_vision/Burhan_filter_module_fcn.h"
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
float weight_green_input = 0.5;
float weight_grad_input = 0.5;

//OBSTACLE FAILSAFE PARAMETERS
uint8_t failsafe_obstacle = 0;
float min_green_thre = 0.2;
int min_sections_failsafe = 3;


struct communicate_msg{
    uint32_t section_idx;
    float green_count;
    uint8_t failsafe_obstacle;
    bool updated;
};

struct vision_msg{
	uint32_t section_idx;
	float green_frac;
	uint8_t failsafe_obstacle_msg;
};


struct communicate_msg global_msg[1];

void Burhan_filter(struct image_t *img, uint8_t draw_on_img,
                   uint8_t R_green_low, uint8_t G_green_low, uint8_t B_green_low,
                   uint8_t R_green_hi, uint8_t G_green_hi, uint8_t B_green_hi, uint8_t gray_threshold,
                   uint8_t thresh_lower, uint8_t filter_height_cut, uint8_t sections,
                   float window_scale,  uint8_t print_weights,float weight_green, float weight_grad, struct vision_msg *vision_msg_in);


struct image_t *Burhan_fcn(struct image_t *img)
{
  struct vision_msg vision_in;
  //Using the Burhan filter
  Burhan_filter(img, draw_on_img, R_green_low, G_green_low, B_green_low,
                  R_green_hi, G_green_hi, B_green_hi, gray_threshold, thresh_lower, filter_height_cut, sections,
                  window_scale, print_weights, weight_green_input, weight_grad_input, &vision_in);

  pthread_mutex_lock(&mutex);
  global_msg[0].section_idx = vision_in.section_idx;
  global_msg[0].green_count = vision_in.green_frac;
  global_msg[0].failsafe_obstacle = vision_in.failsafe_obstacle_msg;
  global_msg[0].updated = true;
  pthread_mutex_unlock(&mutex);
//  fprintf(stderr, "The value of green_count IN THE FUNCTION MODULE is = %f \n",global_msg[0].green_count);

  return img;
}

struct image_t *burhan_filter1(struct image_t *img);
struct image_t *burhan_filter1(struct image_t *img)
{
  return Burhan_fcn(img);
}

void burhan_filter_init(void)
{

    memset(global_msg, 0, sizeof(struct communicate_msg));
    pthread_mutex_init(&mutex, NULL);

#ifdef BURHAN_FILTER_CAMERA1
  cv_add_to_device(&BURHAN_FILTER_CAMERA1, burhan_filter1, BURHAN_FILTER_FPS1);
#endif
}


void Burhan_filter(struct image_t *img, uint8_t draw,
                   uint8_t R_green_low, uint8_t G_green_low, uint8_t B_green_low,
                   uint8_t R_green_hi, uint8_t G_green_hi, uint8_t B_green_hi,
                   uint8_t gray_threshold, uint8_t thresh_lower, uint8_t filter_height_cut, uint8_t sections,
                   float window_scale, uint8_t print_weights, float weight_green, float weight_grad, struct vision_msg *vision_msg_in){

    uint16_t ones_count = 0;
    uint16_t wrong_zeros = 0;
    uint8_t *buffer = img->buf;
    uint16_t next_y_value = 0;
    uint16_t bin_array[img->w];
    uint16_t Green_pixel_value[(int) img->h/STEP];
    float Section_value[sections], weight = 0;
    float gradients[sections - 2];
    float d_gradients[sections - 4];
    float weighted_sum[sections - 2];
    uint16_t idx_section = 0, section_count = 0;
    uint16_t section_value = 0, storage_value = 0, max_idx = 0;


    section_value = (int) img->h/STEP/sections;


    for (uint16_t y = 0; y < img->h; y += STEP) {
        uint8_t cnt = 0;
        uint16_t green_count;
        for (uint16_t x = 0; x < filter_height_cut ; x ++) {
            // Check if the color is inside the specified values
            uint8_t *yp, *up, *vp;
            uint8_t pixel_b, pixel_g, pixel_r, pixel_value_local_gray;
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
                    if (ones_count >= thresh_lower) {
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
                //weight = 1.f - (fabs(idx_section + 1 - ceil(sections*0.5f))/ceil(sections*0.5f)) * window_scale;
                //Section_value[idx_section] = weight * storage_value/(section_value*(img->w - filter_height_cut));
                Section_value[idx_section] = (float)storage_value/((float)(section_value*(img->w - filter_height_cut)));

                storage_value = 0;
                section_count = 0;
                if(Section_value[idx_section] > Section_value[max_idx] ){
                    max_idx = idx_section;
                }
                idx_section++;
            }
        }
    }

    //SEE IF THERE ARE MORE THAN THREE CONSECUTIVE SECTIONS WITH VALUE BELOW A THRESHOLD:
    int counter_min = 0;
    int m_old = 0;
    for (int m = 0 ; m < sections ; m++){
        if(Section_value[m] < min_green_thre){
            if((m - m_old) < 2)
            counter_min++;
            m_old = m;
        }
    }
    max_idx = 0;

    if(counter_min >= min_sections_failsafe){
        failsafe_obstacle = 1;
    }
    else{
        failsafe_obstacle = 0;
    }
//    fprintf(stderr,"FAILSAFE OBSTACLE IS : %d \n ",failsafe_obstacle);

    for (int n = 0; n < sections - 2; n++){
		gradients[n] = fmax(fabs(Section_value[n+1] - Section_value[n]), fabs(Section_value[n+2] - Section_value[n+1]));
	}

    for (int m = 0; m < sections - 4; m++){
    	d_gradients[m] = 1.f - fmax(fabs(gradients[m+1] - gradients[m]), fabs(gradients[m+2] - gradients[m+1]));

    	weighted_sum[m] = weight_grad*d_gradients[m] + weight_green*Section_value[m+2]; //store weighted sum in array

		if(weighted_sum[m] > weighted_sum[max_idx]){
			max_idx = m;
		}
    }

    float grn_count = Section_value[max_idx+2];

    ////// ALEX ADDITION REPLACES MAX_SECTION_IDX ////////
//    float minimum_green_threshold = 0.25;
//    float centroid_num = 0;
//	float centroid_den = 0;
//
//	float max_centroid_area = 0;
//	float max_centroid_idx = 0;
//
//	for (int i = 0; i < 13; i++){
//		if (Section_value[i] >= minimum_green_threshold){
//			float idx = (float) i;
//			centroid_num += idx * Section_value[i];
//			centroid_den += Section_value[i];
//		}
//		else{
//			if (centroid_den > max_centroid_area){
//				max_centroid_area = centroid_den;
//				max_centroid_idx = centroid_num / centroid_den;
//			}
//			centroid_num = 0;
//			centroid_den = 0;
//		}
//	}
//	int max_centroid_idx_returned = (int) (max_centroid_idx + 0.5); // we round then cast to int


//    uint32_t *local_pointer_int;
//    local_pointer_int = &vision_msg_in->section_idx;
//    *local_pointer_int = max_centroid_idx_returned +1;

    uint32_t *local_pointer_int;
    local_pointer_int = &vision_msg_in->section_idx;
    *local_pointer_int = max_idx +1;

    float *local_pointer_float;
    local_pointer_float = &vision_msg_in->green_frac;
    *local_pointer_float = grn_count;

    uint8_t *local_pointer_uint8_t;
    local_pointer_uint8_t = &vision_msg_in->failsafe_obstacle_msg;
    *local_pointer_uint8_t = failsafe_obstacle;

    if(print_weights){
        fprintf(stderr,"WEIGHT FOR EACH SECTION :  ");
        for (int i = 0 ; i < sections ; i++){
            fprintf(stderr," %f ",Section_value[i]);
        }
        fprintf(stderr,".\n ");
        fprintf(stderr,"OUR MAX SECTION IS : %d \n ",max_idx + 1);
    }



}


void burhan_filter_periodic(void)
{

  static struct communicate_msg local_msg[1];
    pthread_mutex_lock(&mutex);
    memcpy( local_msg , global_msg , sizeof(struct communicate_msg));
    pthread_mutex_unlock(&mutex);


  if(local_msg[0].updated){
      //ABI MESSAGE
    AbiSendMsgBURHAN_FILTER(BURHAN_FILTER_ABI_ID, local_msg[0].section_idx , local_msg[0].green_count,
                            local_msg[0].failsafe_obstacle);
//        local_filters[0].filter_width, local_filters[0].filter_height, local_filters[0].color_count, 0);
      local_msg[0].updated = false;
  }

}
