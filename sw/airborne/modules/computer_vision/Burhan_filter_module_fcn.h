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
 * @file modules/computer_vision/Burhan_filter_module_fcn.h
 */

#ifndef BURHAN_FILTER_MODULE_FCN_H
#define BURHAN_FILTER_MODULE_FCN_H

#include <stdint.h>
#include <stdbool.h>

//Burhan filter settings
extern uint16_t H_low, H_hi;
extern uint16_t S_low, S_hi;
extern uint16_t V_low, V_hi;
extern uint8_t gray_threshold;
extern uint16_t STEP;
extern uint8_t filter_height_cut;
extern uint8_t thresh_lower;
extern uint8_t sections;
extern float weight_green_input;
extern float weight_grad_input;
extern uint8_t print_weights;
extern uint8_t draw_on_img;
extern float bin_threshold;
extern int min_sections_failsafe;
extern int Failsafe_increment_int;
extern float min_green_thre;
extern uint8_t double_gradient_bool;

// Module functions
extern void burhan_filter_init(void);
extern void burhan_filter_periodic(void);

#endif /* BURHAN_FILTER_MODULE_FCN_H */
