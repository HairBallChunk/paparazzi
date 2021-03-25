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
#define BURHAN_FILTER_FPS1 10 ///< Default FPS (zero means run at camera fps)
#endif

//Burhan filter settings: KEEP THEM!!!
uint8_t R_green_low = 60, G_green_low = 70, B_green_low = 0; // Lower = [65,20,5]
uint8_t R_green_hi = 100, G_green_hi = 200, B_green_hi = 45; // Higher = [95,255,95]
uint8_t gray_threshold = 20;
uint16_t STEP = 20;
uint8_t filter_height_cut = 120;
uint8_t thresh_lower = 3;
uint8_t sections = 13; //NOTICE, IT APPROXIMATE TO THE CLOSEST INTEGER!!!!!
uint8_t print_weights = 0;
uint8_t draw_on_img = 1;
float weight_green_input = 0.5;
float weight_grad_input = 0.5;

//OBSTACLE FAILSAFE PARAMETERS
uint8_t failsafe_obstacle = 0;
float bin_threshold = 0.5f;
int min_sections_failsafe = 5;
int Failsafe_increment_int = 4;
float sum_left = 0, sum_right = 0;
uint8_t close_pole = 0;
float min_green_thre = 0.3;
uint8_t double_gradient_bool = 0;

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
                   uint8_t print_weights,float weight_green, float weight_grad, struct vision_msg *vision_msg_in);

struct image_t *Burhan_fcn(struct image_t *img);
struct image_t *Burhan_fcn(struct image_t *img)
{
  struct vision_msg vision_in;
  //Using the Burhan filter
  Burhan_filter(img, draw_on_img, R_green_low, G_green_low, B_green_low,
                  R_green_hi, G_green_hi, B_green_hi, gray_threshold, thresh_lower, filter_height_cut, sections,
                  print_weights, weight_green_input, weight_grad_input, &vision_in);

  pthread_mutex_lock(&mutex);
  global_msg[0].section_idx = vision_in.section_idx;
  global_msg[0].green_count = vision_in.green_frac;
  global_msg[0].failsafe_obstacle = vision_in.failsafe_obstacle_msg;
  global_msg[0].updated = true;
  pthread_mutex_unlock(&mutex);
//  fprintf(stderr, "The value of green_count IN THE FUNCTION MODULE is = %f \n",global_msg[0].green_count);

  return img;
}

// Struct to store HSV pixel values
struct hsv{
	uint8_t H;
	uint8_t S;
	uint8_t V;
};

//Function to convert RGB to HSV
void RGB2HSV(int r,int g,int b, struct hsv *hsv_in){

	float cmax,cmin,diff;
	float r_norm,g_norm,b_norm;

	float h_calc,s_calc,v_calc;

	r_norm = (float)r/255.f;
	g_norm = (float)g/255.f;
	b_norm = (float)b/255.f;

	cmax = fmax(r_norm,fmax(g_norm,b_norm));
	cmin = fmin(r_norm,fmin(g_norm,b_norm));
	diff = cmax - cmin;

	if (cmax == cmin){
		h_calc = 0.f;
	}
	else if (cmax == r_norm){
		h_calc = fmod(((60*((g_norm - b_norm)/diff)) + 360), 360.0);
	}
	else if (cmax == g_norm){
		h_calc = fmod(((60*((b_norm - r_norm)/diff)) + 120), 360.0);
	}
	else if (cmax == b_norm){
		h_calc = fmod(((60*((r_norm - g_norm)/diff)) + 240), 360.0);
	}

	if (cmax == 0){
		s_calc = 0.0;
	}else{
		s_calc = (diff/cmax)*100;
	}

	v_calc = cmax*100;

	uint32_t *h_ptr;
	h_ptr = &hsv_in->H;
	*h_ptr = (int)h_calc;

	uint32_t *s_ptr;
	s_ptr = &hsv_in->S;
	*s_ptr = (int)s_calc;

	uint32_t *v_ptr;
	v_ptr = &hsv_in->V;
	*v_ptr = (int)v_calc;

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

void Burhan_filter(struct image_t *img, uint8_t draw,
                   uint8_t R_green_low, uint8_t G_green_low, uint8_t B_green_low,
                   uint8_t R_green_hi, uint8_t G_green_hi, uint8_t B_green_hi,
                   uint8_t gray_threshold, uint8_t thresh_lower, uint8_t filter_height_cut, uint8_t sections,
                   uint8_t print_weights, float weight_green, float weight_grad, struct vision_msg *vision_msg_in) {

    uint16_t ones_count = 0;
    uint16_t wrong_zeros = 0;
    uint8_t *buffer = img->buf;
    uint16_t bin_array[img->w];
    float Section_value[sections];
    float weighted_sum[sections - 2];
    uint16_t idx_section = 0, section_count = 0;
    uint16_t section_value = 0, storage_value = 0, max_idx = (int) (sections - 1) / 2, max_idx_green = (int) (sections - 1) / 2;
    Section_value[max_idx] = 0;
    section_value = (int) img->h / STEP / sections; // This is the number of STEPS in a section.
    float d_gradients[sections - 4];
    float gradients[sections - 2];

    // Struct to store HSV values
    struct hsv HSV;

    for (uint16_t y = 0; y < img->h; y += STEP) {
        uint8_t cnt = 0;
        for (uint16_t x = 0; x < img->w; x++) {
            // Check if the color is inside the specified values
            uint8_t *yp, *up, *vp;
            uint8_t pixel_b, pixel_g, pixel_r, pixel_value_local_gray;
            uint8_t pixel_h, pixel_s, pixel_v;

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
            pixel_b = (int) fmin(255.f, 1.164f * (*yp - 16) + 2.018f * (*up - 128));
            pixel_g = (int) fmin(255.f, 1.164f * (*yp - 16) - 0.813f * (*vp - 128) - 0.391f * (*up - 128));
            pixel_r = (int) fmin(255.f, 1.164f * (*yp - 16) + 1.596f * (*vp - 128));


            RGB2HSV(pixel_r, pixel_g, pixel_b, &HSV);

            pixel_h = HSV.H;
            pixel_s = HSV.S;
            pixel_v = HSV.V;

            //CREATE THE BITWISE_AND TOGETHER WITH THE INRANGE FCN
            if ((pixel_b >= B_green_low) && (pixel_b <= B_green_hi) &&
                (pixel_g >= G_green_low) && (pixel_g <= G_green_hi) &&
                (pixel_r >= R_green_low) && (pixel_r <= R_green_hi)) {

              //Keep the YUV in case the RGB doen't work properly
//            if ( (*up >= 0) && (*up <= 120) &&
//                 (*vp >= 0 ) && (*vp <= 120 ) &&
//                 (*yp >= 50 ) && (*yp <= 200 )) {

            // Use this if condition when switched over to HSV filter
//			if ((pixel_h >= H_green_low) && (pixel_h <= H_green_hi) &&
//				(pixel_s >= S_green_low) && (pixel_s <= S_green_hi) &&
//				(pixel_v >= V_green_low) && (pixel_v <= V_green_hi)) {

                //IMPLEMENTING THE BGR TO GRAYSCALE
                pixel_value_local_gray = (int) 0.3f * pixel_r + 0.59f * pixel_g + 0.11f * pixel_b;
            } else {
                pixel_value_local_gray = 0;
            }
            //IMPLEMENTING THE THRESHOLD FCN
            if (pixel_value_local_gray >= gray_threshold) {
                cnt++; //count the green pixels above the gray threshold
                bin_array[x] = 1;
                if (draw) {
                    *yp = 255; // make pixel brighter in image
                }
            } else {
                bin_array[x] = 0;
            }
        }

        // FIND CONTINUOUS ZEROS FUNCTION
        wrong_zeros = 0;
        ones_count = 0;
        int count2;
        for (int k = filter_height_cut; k < img->w; k++) {
            uint8_t *yp;
            count2 = img->w - k;
            if (bin_array[count2] == 0) {
                if (ones_count >= thresh_lower) {
                    wrong_zeros++;
                    //Write the pixel to image
                    if (draw) {
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
            } else if (bin_array[count2] == 1) {
                ones_count++;
            }
        }
        //REFINE THE FILTER INTO BIGGER CELLS
        storage_value += wrong_zeros + cnt;
        section_count++;
        if (section_count == section_value) {
            Section_value[idx_section] = (float) storage_value / ((float) (section_value * (img->w - filter_height_cut))); // Normalize the green count in section
            storage_value = 0; // reset counts
            section_count = 0;
            if (Section_value[idx_section] > Section_value[max_idx_green]) {
                max_idx_green = idx_section; //find section with most green pixels
            }
            idx_section++;
        }
    }

//======================================================================= Navigation algorithms ===============================================================================//

////======================================Burhan single gradient navigation============================================//
    if(!double_gradient_bool) {
        for (int n = 0; n < sections - 2; n++) {
            gradients[n] = 1.f - fmax(fabs(Section_value[n + 1] - Section_value[n]),
                                      fabs(Section_value[n + 2] - Section_value[n + 1]));

            weighted_sum[n] =
                    weight_grad * gradients[n] + weight_green * Section_value[n + 1]; //store weighted sum in array

            if (weighted_sum[n] > weighted_sum[max_idx]) {
                max_idx = n;
            }
        }
    }
    float grn_count = Section_value[max_idx + 1];
//// ======================================Burhan double gradient navigation=============================================//
    if (double_gradient_bool) {
        for (int n = 0; n < sections - 2; n++) {
            gradients[n] = fmax(fabs(Section_value[n + 1] - Section_value[n]),
                                fabs(Section_value[n + 2] - Section_value[n + 1]));
        }

        for (int m = 0; m < sections - 4; m++) {
            d_gradients[m] = 1.f - fmax(fabs(gradients[m + 1] - gradients[m]), fabs(gradients[m + 2] - gradients[m + 1]));

            weighted_sum[m] =
                    weight_grad * d_gradients[m] + weight_green * Section_value[m + 2]; //store weighted sum in array

            if (weighted_sum[m] > weighted_sum[max_idx]) {
                max_idx = m;
            }
        }
        grn_count = Section_value[max_idx + 2];
    }

//=============================================================== Navigation Fail Safes ===============================================================================//


////======================================= BURHAN detect close poles - fail safe================================================//

	int center = (int) (sections-1)/2; // center index of Section_value starting at 0

//	float sum_total;
    sum_left = 0;
    sum_right = 0;
    close_pole = 0;
    failsafe_obstacle = 0;

	for (int r = center - min_sections_failsafe; r < center + min_sections_failsafe + 1; r++ ){
		if (r < center){
			sum_left += Section_value[r];
		}
		if (r > center){
			sum_right += Section_value[r];
		}
	}


	if (sum_left < (float) bin_threshold*sum_right && Section_value[max_idx_green] > min_green_thre){
		max_idx = center + Failsafe_increment_int;
		close_pole = 1;
        failsafe_obstacle = 1;
	}
	else if (sum_right < (float) bin_threshold*sum_left && Section_value[max_idx_green] > min_green_thre){
		max_idx = center - Failsafe_increment_int;
		close_pole = 1;
        failsafe_obstacle = 1;
    }


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

    //Print target heading on the image
    int x_target = max_idx * section_value * STEP + STEP;
    for (int k = 0; k < img->w; k++){
        uint8_t *yp, *up, *vp;
        if (k % 2 == 0) {
            // Even x
            up = &buffer[x_target * 2 * img->w + 2 * k];      // U
            vp = &buffer[x_target * 2 * img->w + 2 * k + 2];  // V
            yp = &buffer[x_target * 2 * img->w + 2 * k + 1];    // Y1
        } else {
            // Uneven x
            up = &buffer[x_target * 2 * img->w + 2 * k - 2];  // U
            vp = &buffer[x_target * 2 * img->w + 2 * k];      // V
            yp = &buffer[x_target * 2 * img->w + 2 * k + 1];    // Y1
        }
        *up = 84;
        *vp = 255;
        *yp = 76;
        if (close_pole == 1){ ///Draw a blue line if in close pole fail safe mode
        	*up = 255;
        	*vp = 107;
        	*yp = 29;
        }
    }


}
