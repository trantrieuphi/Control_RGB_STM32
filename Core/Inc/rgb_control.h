/*
 * rgb_control.h
 *
 *  Created on: Oct 12, 2023
 *      Author: trant
 */

#ifndef INC_RGB_CONTROL_H_s
#define INC_RGB_CONTROL_H_
#include "sys/_stdint.h"

// struct to store an RGB color
typedef struct rgb {
	uint8_t r;
	uint8_t g;
	uint8_t b;
}rgb_t;

//struct to store one NEOPixel LED
typedef struct neopixel {
	uint16_t g[8];
	uint16_t r[8];
	uint16_t b[8];
}neopixel_t;

// 1 chu ki(T) 1.25us
// logic 1 tuong ung voi 0.8us/T muc cao
// logic 0 tuong ung voi 0.4us/T muc cao
// sys frequency = 20MHz, prescaler = 0 => timer frequency = 20MHz/(0+1) = 20MHz
// counter period = 24 => pwm frequency = 20MHz/(24+1) = 800kHz( Data transfer time = 1.25us)
// logic 1: CCR = 15 => (15/24)*1.25 = 0.8us
// logic 0: CCR = 7 => (7/24)*1.25 = 0.4us
#define LED_LOGIC_1 20
#define LED_LOGIC_0 10

void reset_all_leds(neopixel_t * leds, uint16_t number_leds);
void set_all_leds(neopixel_t * leds, uint16_t number_leds);
void set_specific_led(neopixel_t * leds,uint16_t led_position, rgb_t color);
void set_pattern_led(neopixel_t * leds,rgb_t *pattern, uint8_t num_leds);

#endif /* INC_RGB_CONTROL_H_ */
