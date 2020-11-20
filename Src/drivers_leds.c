/*
 * drivers_leds.c
 *
 *  Created on: Nov 18, 2020
 *      Author: ludivineo
 */
#include "drivers_leds.h"

int matrix [LINES_NBR][COLS_NBR] = {0};
int matrix_1D [LEDS_NBR] = {0};


void set_one_led (int x, int y, int color_value){
	int target = get_target_led(x, y);
	matrix_1D[target]=color_value;
}

uint8_t ascii_to_int(char charachter){
	uint8_t int_result;
	if (charachter == '0'){
		int_result = 0;
	}
	else if (charachter == '1'){
		int_result = 1;
	}
	else if (charachter == '2'){
		int_result = 2;
	}
	else if (charachter == '3'){
		int_result = 3;
	}
	else if (charachter == '4'){
		int_result = 4;
	}
	else if (charachter == '5'){
		int_result = 5;
	}
	else if (charachter == '6'){
		int_result = 6;
	}
	else if (charachter == '7'){
		int_result = 7;
	}
	else if (charachter == '8'){
		int_result = 8;
	}
	else if (charachter == '9'){
		int_result = 9;
	}
	else if (charachter == 'A'){
		int_result = 10;
	}
	else if (charachter == 'B'){
		int_result = 11;
	}
	else if (charachter == 'C'){
		int_result = 12;
	}
	else if (charachter == 'D'){
		int_result = 13;
	}
	else if (charachter == 'E'){
		int_result = 14;
	}
	else if (charachter == 'F'){
		int_result = 15;
	}
	return int_result;
}

uint8_t assemble_2int(uint8_t int1, uint8_t int2){
	uint8_t int_result = (int1*16)+int2;
	return int_result;
}


int assemble_couleur(uint8_t red_value, uint8_t green_value, uint8_t blue_value){
	int color_value = 0;
	color_value = (green_value << 16 | red_value << 8 | blue_value);
	return color_value;
}


void send_frame_to_leds_1D(int target_led){
	vTaskSuspendAll();
	SysTick->CTRL &= ~1;
	TIM1->CR1 &= ~TIM_CR1_CEN;
	HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 0);
	for (int j = 0; j < target_led+1; j++){
		int masque = 0x800000;
		for (int i = 23; i >= 1 ; i --){
			int high1 = T1H;
			int low1 = T1L;
			int high0 = T0H;
			int low0 = T0L;
			if (matrix_1D[j] & masque){
				HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 1);
				while(high1--);
				HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 0);
				while(low1--);
			}
			else{
				HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 1);
				while(high0--);
				HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 0);
				while(low0--);
			}
			masque >>= 1;
		}
		HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 1);
		HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 0);
	}
	TIM1->CR1 |= TIM_CR1_CEN;
	SysTick->CTRL |= 1;
	xTaskResumeAll();

}

void send_frame_to_leds_2D(int led_col, int led_line){
	int col_stop = COLS_NBR-1;
	vTaskSuspendAll();
	HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 0);
	for (int line = 0; line <= led_line; line++){
		if(line == led_line){
			col_stop = led_col;
		}
		for (int col = 0; col < col_stop-1 ; col++){
			int masque = 0x800000;
			for (int i = 23; i >= 0 ; i --){
				int high1 = T1H;
				int low1 = T1L;
				int high0 = T0H;
				int low0 = T0L;
				if (matrix[line][col] & masque){
					HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 1);
					while(high1--);
					HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 0);
					while(low1--);
				}
				else{
					HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 1);
					while(high0--);
					HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 0);
					while(low0--);
				}
				masque >>= 1;
			}
		}
		HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 1);
		HAL_GPIO_WritePin(LEDS_MODULE_GPIO_Port, LEDS_MODULE_Pin, 0);
	}
	xTaskResumeAll();
}

void set_white(){
	for (int lignes = 0; lignes< 8;lignes++){
		for (int col = 0; col<8; col++){
			matrix[lignes][col] = assemble_couleur(10,0,0);
		}
	}
}

void set_all_leds_2D(int r_value, int g_value, int b_value){
	// pour matrix 2D
	for (int lignes = 0; lignes< 8;lignes++){
		for (int col = 0; col<8; col++){
			matrix [lignes][col] = assemble_couleur(r_value, g_value, b_value);
		}
	}

}


void set_all_leds_1D(int r_value, int g_value, int b_value){
	for (int i = 0; i<(LEDS_NBR); i++){
		matrix_1D[i] = assemble_couleur(r_value, g_value, b_value);
	}
}

void execute_led_msg_2D (char msg [9]){
	//extraction des coord
	int x = ascii_to_int(msg[1]);
	int y = ascii_to_int(msg[2]);
	// extraction des composants de la couleur
	uint8_t g_value;
	uint8_t r_value;
	uint8_t b_value;
	r_value = assemble_2int(ascii_to_int(msg[3]), ascii_to_int(msg[4]));
	g_value = assemble_2int(ascii_to_int(msg[5]), ascii_to_int(msg[6]));
	b_value = assemble_2int(ascii_to_int(msg[7]), ascii_to_int(msg[8]));
	//assemblage de la couleur
	int color_value = assemble_couleur(r_value, g_value, b_value);
	//MàJ du tableau
	matrix[x][y] = color_value;
	send_frame_to_leds_2D(x, y);

}

int get_target_led(int x, int y){
	// passage des coord en "base1"
	int target_led = (y*LINES_NBR)+(x+1);
	// retour en "base0"
	return target_led-1;
}


void execute_led_msg_1D (char *msg){
	//extraction des coord
	int x = ascii_to_int(msg[1]);
	int y = ascii_to_int(msg[2]);
	int target = get_target_led(x, y);
	// extraction des composants de la couleur
	uint8_t g_value;
	uint8_t r_value;
	uint8_t b_value;
	r_value = assemble_2int(ascii_to_int(msg[3]), ascii_to_int(msg[4]));
	g_value = assemble_2int(ascii_to_int(msg[5]), ascii_to_int(msg[6]));
	b_value = assemble_2int(ascii_to_int(msg[7]), ascii_to_int(msg[8]));
	//assemblage de la couleur
	int color_value = assemble_couleur(r_value, g_value, b_value);
	//MàJ du tableau
	matrix_1D[target] = color_value;
	//envoi de la trame jusqu'à target led
	send_frame_to_leds_1D(target);

}
void blink_one_led(){
	int color0 = 50;
	int color1 = 10000;
	int color2 = 10000000;
	int count = 0;
	while (count <= 2){
		if (count == 0){
			set_one_led(1,1,color0);
		}
		else if (count == 1){
			set_one_led(1,1,color1);
		}
		else if (count == 2){
			set_one_led(1,1,color2);
		}
		send_frame_to_leds_1D(63);
		count +=1;
		osDelay(1000);
	}
}




