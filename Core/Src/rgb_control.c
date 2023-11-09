/*
 * rgb_control.c
 *
 *  Created on: Oct 12, 2023
 *      Author: trant
 */
#include "rgb_control.h"

//func to reset all leds rgb
void reset_all_leds(neopixel_t * leds, uint16_t number_leds)
{
	for(uint8_t i = 0; i < number_leds; i++)
	{
		for(uint8_t j = 0; j < 8; j++)
		{
			(leds + i)->r[j] = LED_LOGIC_0;
			(leds + i)->g[j] = LED_LOGIC_0;
			(leds + i)->b[j] = LED_LOGIC_0;
		}
	}
}

//func to set all leds rgb
void set_all_leds(neopixel_t * leds, uint16_t number_leds){
	for(uint8_t i = 0; i < number_leds; i++)
	{
		for(uint8_t j = 0; j < 8; j++)
		{
			(leds + i)->r[j] = LED_LOGIC_1;
			(leds + i)->g[j] = LED_LOGIC_1;
			(leds + i)->b[j] = LED_LOGIC_1;
		}
	}
}

//func to set a specific color to a particular LED
void set_specific_led(neopixel_t * leds, uint16_t led_position, rgb_t color)
{
	for(uint8_t i = 0; i < 8; i++)
	{
		if(color.r & 0x1 <<i){
			(leds + led_position)->r[7-i] = LED_LOGIC_1;
		} else {
			(leds + led_position)->r[7-i] = LED_LOGIC_0;
		}

		if(color.g & 0x1 <<i){
			(leds + led_position)->g[7-i] = LED_LOGIC_1;
		} else {
			(leds + led_position)->g[7-i] = LED_LOGIC_0;
		}

		if(color.b & 0x1 <<i){
			(leds + led_position)->b[7-i] = LED_LOGIC_1;
		} else {
			(leds + led_position)->b[7-i] = LED_LOGIC_0;
		}
	}
}

// func to set pattern LED
void set_pattern_led(neopixel_t * leds,rgb_t *pattern, uint8_t num_leds)
{
	for(uint8_t i = 0; i < num_leds; i++)
	{
		set_specific_led(leds, i, *(pattern + i));
	}
}


