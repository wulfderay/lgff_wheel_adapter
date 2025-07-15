#pragma once
/*******************************************************************************
 * LED control
 *******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <pins_arduino.h>
#include "hardware/gpio.h"


#define LED_GREEN       PIN_LED
#define LED_BLUE        6
#define LED_RED         7
#define LED_YELLOW      8

void init_leds(void);
void set_led(uint8_t led, bool value);
void leds_display_val(uint8_t value);


void clear_led_strip(void);
void update_led_strip(uint8_t leds_buffer);
void debug_led_strip(uint16_t pid);
