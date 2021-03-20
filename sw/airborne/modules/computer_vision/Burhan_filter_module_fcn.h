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
 * @file modules/computer_vision/cv_detect_color_object.h
 * Assumes the color_object consists of a continuous color and checks
 * if you are over the defined color_object or not
 */

#ifndef COLOR_OBJECT_DETECTOR_CV_H
#define COLOR_OBJECT_DETECTOR_CV_H

#include <stdint.h>
#include <stdbool.h>

//Burhan filter settings
extern uint8_t R_green_low, G_green_low, B_green_low;
extern uint8_t R_green_hi, G_green_hi, B_green_hi;
extern uint8_t gray_threshold;
extern uint16_t STEP;
extern uint8_t filter_height_cut;
extern uint8_t thresh_lower;
extern uint8_t sections;
extern float window_scale;
extern uint8_t print_weights;
extern uint8_t draw_on_img;


// Module functions
extern void burhan_filter_init(void);
extern void burhan_filter_periodic(void);

#endif /* COLOR_OBJECT_DETECTOR_CV_H */
