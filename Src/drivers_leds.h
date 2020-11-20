/*
 * drivers_leds.h
 *
 *  Created on: Nov 19, 2020
 *      Author: ludivineo
 */

#ifndef DRIVERS_LEDS_H_
#define DRIVERS_LEDS_H_
#ifndef MAIN_H_
#include "main.h"
#endif /* MAIN_H_ */


#define	T0H 1
#define T1H 10
#define T0L 9
#define T1L	2
#define LINES_NBR 8
#define COLS_NBR 8
#define SIZE_RECEIVED_FRAME 10
#define LEDS_NBR 64

uint8_t ascii_to_int(char charachter);
uint8_t assemble_2int(uint8_t int1, uint8_t int2);
int assemble_couleur(uint8_t red_value, uint8_t green_value, uint8_t blue_value);
void send_frame_to_leds_1D(int target_led);
void send_frame_to_leds_2D(int led_col, int led_line);
void set_white();
void set_all_leds_2D(int r_value, int g_value, int b_value);
void set_all_leds_1D(int r_value, int g_value, int b_value);
void execute_led_msg_2D (char msg [9]);
int get_target_led(int x, int y);
void execute_led_msg_1D (char *msg);
void set_one_led (int x, int y, int color_value);
void blink_one_led();





#endif /* DRIVERS_LEDS_H_ */
