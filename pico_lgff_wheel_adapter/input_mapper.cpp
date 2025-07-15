#include "input_mapper.h"
#include <cstdint>
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "pico/stdlib.h"

#include "Arduino.h"
#include "enums.h"
#include "usb_descriptors.h"
#include "wheel_ids.h"
#include "reports.h"
#include "config.h"

/*private functions */
static uint8_t dampen_steering_sensitivity_8(uint8_t input, float dampening);

void reset_generic_report(generic_report_t * generic_report) {
  memset(generic_report, 0, sizeof(generic_report_t));
  generic_report->hat = 0x08;
  generic_report->wheel_8 = 0x7F;
  generic_report->wheel_10 = 0x7F;
  generic_report->wheel_14 = 0x7F;
  generic_report->wheel_16 = 0x7F;
  generic_report->gasPedal_8 = 0xFF;
  generic_report->brakePedal_8 = 0xFF;
  generic_report->clutchPedal_8 = 0xFF;
  generic_report->gasPedal_16 = 0xFFFF;
  generic_report->brakePedal_16 = 0xFFFF;
  generic_report->clutchPedal_16 = 0xFFFF;
  generic_report->shifter_x = 0x80;
  generic_report->shifter_y = 0x80;
}


void map_input(uint8_t const* report, generic_report_t * generic_report, uint16_t vid, uint16_t pid) {

  if (pid == pid_df) { // Driving Force. most logitech wheels will start in this mode

    // map the received report to the generic report
    df_report_t* input_report = (df_report_t*)report;

    generic_report->wheel_precision = wheel_10bits;
    generic_report->pedals_precision_16bits = false;

    generic_report->wheel_10 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = input_report->brakePedal;

    generic_report->hat = input_report->hat;
    generic_report->cross = input_report->cross;
    generic_report->square = input_report->square;
    generic_report->circle = input_report->circle;
    generic_report->triangle = input_report->triangle;
    generic_report->R1 = input_report->R1;
    generic_report->L1 = input_report->L1;      
    generic_report->R2 = input_report->R2;
    generic_report->L2 = input_report->L2;
    generic_report->R3 = input_report->R3;
    generic_report->L3 = input_report->L3;
    generic_report->select = input_report->select;
    generic_report->start = input_report->start;

  } else if (pid == pid_dfp) { // Driving Force Pro UNTESTED
    
    // map the received report to the generic report
    dfp_report_t* input_report = (dfp_report_t*)report;
    
    generic_report->wheel_precision = wheel_14bits;
    generic_report->pedals_precision_16bits = false;
    
    generic_report->wheel_14 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = input_report->brakePedal;

    generic_report->hat = input_report->hat;
    generic_report->cross = input_report->cross;
    generic_report->square = input_report->square;
    generic_report->circle = input_report->circle;
    generic_report->triangle = input_report->triangle;
    generic_report->R1 = input_report->R1;
    generic_report->L1 = input_report->L1;
    generic_report->R2 = input_report->R2;
    generic_report->L2 = input_report->L2;
    generic_report->R3 = input_report->R3;
    generic_report->L3 = input_report->L3;
    generic_report->select = input_report->select;
    generic_report->start = input_report->start;
    generic_report->gear_minus = input_report->gear_minus;
    generic_report->gear_plus = input_report->gear_plus;

  } else if (pid == pid_dfgt) { // Driving Force GT

    // map the received report to the generic report
    dfgt_report_t* input_report = (dfgt_report_t*)report;

    generic_report->wheel_precision = wheel_14bits;
    generic_report->pedals_precision_16bits = false;
    
    generic_report->wheel_14 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = input_report->brakePedal;

    generic_report->hat = input_report->hat;
    generic_report->cross = input_report->cross;
    generic_report->square = input_report->square;
    generic_report->circle = input_report->circle;
    generic_report->triangle = input_report->triangle;
    generic_report->R1 = input_report->R1;
    generic_report->L1 = input_report->L1;
    generic_report->R2 = input_report->R2;
    generic_report->L2 = input_report->L2;
    generic_report->R3 = input_report->R3;
    generic_report->L3 = input_report->L3;
    generic_report->select = input_report->select;
    generic_report->start = input_report->start;
    generic_report->gear_minus = input_report->gear_minus;
    generic_report->gear_plus = input_report->gear_plus;
    generic_report->dial_cw = input_report->dial_cw;
    generic_report->dial_ccw = input_report->dial_ccw;
    generic_report->enter = input_report->enter;
    generic_report->plus = input_report->plus;
    generic_report->minus = input_report->minus;
    generic_report->horn = input_report->horn;
    generic_report->PS = input_report->PS;

  } else if (pid == pid_g25) { // G25

    // map the received report to output report
    g25_report_t* input_report = (g25_report_t*)report;

    generic_report->wheel_precision = wheel_14bits;
    generic_report->pedals_precision_16bits = false;
    
    generic_report->wheel_14 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = input_report->brakePedal;
    generic_report->clutchPedal_8 = input_report->clutchPedal;

    generic_report->hat = input_report->hat;
    generic_report->cross = input_report->cross;
    generic_report->square = input_report->square;
    generic_report->circle = input_report->circle;
    generic_report->triangle = input_report->triangle;
    generic_report->R1 = input_report->R1;
    generic_report->L1 = input_report->L1;
    generic_report->R2 = input_report->R2;
    generic_report->L2 = input_report->L2;
    generic_report->R3 = input_report->R3;
    generic_report->L3 = input_report->L3;
    generic_report->select = input_report->select;
    generic_report->start = input_report->start;
    generic_report->shifter_x = input_report->shifter_x;
    generic_report->shifter_y = input_report->shifter_y;
    generic_report->shifter_1 = input_report->shifter_1;
    generic_report->shifter_2 = input_report->shifter_2;
    generic_report->shifter_3 = input_report->shifter_3;
    generic_report->shifter_4 = input_report->shifter_4;
    generic_report->shifter_5 = input_report->shifter_5;
    generic_report->shifter_6 = input_report->shifter_6;
    generic_report->shifter_r = input_report->shifter_r;
    generic_report->shifter_stick_down = input_report->shifter_stick_down;
    
  } else if (pid == pid_g27) { // G27

    // map the received report to output report
    g27_report_t* input_report = (g27_report_t*)report;

    generic_report->wheel_precision = wheel_14bits;
    generic_report->pedals_precision_16bits = false;
    
    generic_report->wheel_14 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = input_report->brakePedal;
    generic_report->clutchPedal_8 = input_report->clutchPedal;

    generic_report->hat = input_report->hat;
    generic_report->cross = input_report->cross;
    generic_report->square = input_report->square;
    generic_report->circle = input_report->circle;
    generic_report->triangle = input_report->triangle;
    generic_report->R1 = input_report->R1;
    generic_report->L1 = input_report->L1;
    generic_report->R2 = input_report->R2;
    generic_report->L2 = input_report->L2;
    generic_report->R3 = input_report->R3;
    generic_report->L3 = input_report->L3;
    generic_report->R4 = input_report->R4;
    generic_report->L4 = input_report->L4;
    generic_report->R5 = input_report->R5;
    generic_report->L5 = input_report->L5;
    generic_report->select = input_report->select;
    generic_report->start = input_report->start;
    generic_report->shifter_x = input_report->shifter_x;
    generic_report->shifter_y = input_report->shifter_y;
    generic_report->shifter_1 = input_report->shifter_1;
    generic_report->shifter_2 = input_report->shifter_2;
    generic_report->shifter_3 = input_report->shifter_3;
    generic_report->shifter_4 = input_report->shifter_4;
    generic_report->shifter_5 = input_report->shifter_5;
    generic_report->shifter_6 = input_report->shifter_6;
    generic_report->shifter_r = input_report->shifter_r;
    generic_report->shifter_stick_down = input_report->shifter_stick_down;

  } else if (pid == pid_g29 || pid == pid_g923) { // G29 or G923

    // map the received report to output report
    g29_report_t* input_report = (g29_report_t*)report;

    generic_report->wheel_precision = wheel_16bits;
    generic_report->pedals_precision_16bits = false;

    generic_report->wheel_16 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = input_report->brakePedal;
    generic_report->clutchPedal_8 = input_report->clutchPedal;

    generic_report->hat = input_report->hat;
    generic_report->cross = input_report->cross;
    generic_report->square = input_report->square;
    generic_report->circle = input_report->circle;
    generic_report->triangle = input_report->triangle;
    generic_report->R1 = input_report->R1;
    generic_report->L1 = input_report->L1;
    generic_report->R2 = input_report->R2;
    generic_report->L2 = input_report->L2;
    generic_report->select = input_report->share;
    generic_report->start = input_report->options;
    generic_report->R3 = input_report->R3;
    generic_report->L3 = input_report->L3;
    generic_report->shifter_1 = input_report->shifter_1;
    generic_report->shifter_2 = input_report->shifter_2;
    generic_report->shifter_3 = input_report->shifter_3;
    generic_report->shifter_4 = input_report->shifter_4;
    generic_report->shifter_5 = input_report->shifter_5;
    generic_report->shifter_6 = input_report->shifter_6;
    generic_report->shifter_r = input_report->shifter_r;
    generic_report->plus = input_report->plus;
    generic_report->minus = input_report->minus;
    generic_report->dial_cw = input_report->dial_cw;
    generic_report->dial_ccw = input_report->dial_ccw;
    generic_report->enter = input_report->enter;
    generic_report->PS = input_report->PS;
    generic_report->shifter_x = input_report->shifter_x;
    generic_report->shifter_y = input_report->shifter_y;
    generic_report->shifter_stick_down = input_report->shifter_stick_down;

  } else if (pid == pid_fgp) { // Formula GP

    // map the received report to output report
    fgp_report_t* input_report = (fgp_report_t*)report;

    generic_report->wheel_precision = wheel_8bits;
    generic_report->pedals_precision_16bits = false;
    
    generic_report->wheel_8 = dampen_steering_sensitivity_8(input_report->wheel, STEERING_CENTER_DAMPEN);
    generic_report->gasPedal_8 =  input_report->brakePedal; // yep, I know. 
    generic_report->brakePedal_8 = 255 - input_report->gasPedal;

    generic_report->hat = 0x8;
    generic_report->cross = input_report->button_top_left;
    generic_report->square = input_report->button_top_right;
    generic_report->circle = input_report->button_bot_left;
    generic_report->triangle = input_report->button_bot_right;
    generic_report->R1 = input_report->rightPaddle;
    generic_report->L1 = input_report->leftPaddle;

  } else if (pid == pid_ffgp) { // Formula Force GP

    // map the received report to output report
    ffgp_report_t* input_report = (ffgp_report_t*)report;

    generic_report->wheel_precision = wheel_10bits;
    generic_report->pedals_precision_16bits = false;
    
    generic_report->wheel_10 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = input_report->brakePedal;

    generic_report->hat = 0x8;
    generic_report->cross = input_report->button_top_left;
    generic_report->square = input_report->button_top_right;
    generic_report->circle = input_report->button_bot_left;
    generic_report->triangle = input_report->button_bot_right;
    generic_report->R1 = input_report->rightPaddle;
    generic_report->L1 = input_report->leftPaddle;
    
  } else if (pid == pid_momofo) { // Momo Force

    // map the received report to output report
    momofo_report_t* input_report = (momofo_report_t*)report;

    generic_report->wheel_precision = wheel_8bits;
    generic_report->pedals_precision_16bits = false;
    
    generic_report->wheel_8 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = ~input_report->brakePedal;

    generic_report->hat = 0x8;
    generic_report->cross = input_report->cross;
    generic_report->square = input_report->square;
    generic_report->circle = input_report->circle;
    generic_report->triangle = input_report->triangle;
    generic_report->R1 = input_report->R1;
    generic_report->L1 = input_report->L1;
    generic_report->select = input_report->select;
    generic_report->start = input_report->start;
    
  } else if (pid == pid_momora) { // Momo Racing

    // map the received report to output report
    momora_report_t* input_report = (momora_report_t*)report;

    generic_report->wheel_precision = wheel_8bits;
    generic_report->pedals_precision_16bits = false;
    
    generic_report->wheel_8 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = input_report->brakePedal;

    generic_report->hat = 0x8;
    generic_report->cross = input_report->cross;
    generic_report->square = input_report->square;
    generic_report->circle = input_report->circle;
    generic_report->triangle = input_report->triangle;
    generic_report->R1 = input_report->R1;
    generic_report->L1 = input_report->L1;
    generic_report->select = input_report->select;
    generic_report->start = input_report->start;
    generic_report->gear_minus = input_report->gear_minus;
    generic_report->gear_plus = input_report->gear_plus;
    
  } else if (pid == pid_sfw) { // Speed Force Wireless

    // map the received report to output report
    sfw_report_t* input_report = (sfw_report_t*)report;

    generic_report->wheel_precision = wheel_10bits;
    generic_report->pedals_precision_16bits = false;

    if (input_report->hat_u) {
      if (input_report->hat_l)
        generic_report->hat = 0x7;
      else if (input_report->hat_r)
        generic_report->hat = 0x1;
      else
        generic_report->hat = 0x0;
    } else if (input_report->hat_d) {
      if (input_report->hat_l)
        generic_report->hat = 0x5;
      else if (input_report->hat_r)
        generic_report->hat = 0x3;
      else
        generic_report->hat = 0x4;
    } else if (input_report->hat_l) {
      generic_report->hat = 0x6;
    } else if (input_report->hat_r) {
      generic_report->hat = 0x2;
    } else {
      generic_report->hat = 0x8;
    }
    
    generic_report->wheel_10 = input_report->wheel;
    generic_report->gasPedal_8 = input_report->gasPedal;
    generic_report->brakePedal_8 = input_report->brakePedal;
    
    generic_report->cross = input_report->b;
    generic_report->square = input_report->one;
    generic_report->circle = input_report->a;
    generic_report->triangle = input_report->two;
    generic_report->select = input_report->minus;
    generic_report->start = input_report->plus;
    generic_report->PS = input_report->home;

    // if using external pedals, reuse the analog paddles as L1/R1
    #ifdef EXTERNAL_PEDAL_TYPE
      generic_report->R1 = input_report->gasPedal < 127;
      generic_report->L1 = input_report->brakePedal < 127;
    #endif
    
  }

}

/**
  * Dampen steering sensitivity based on a configurable dampening factor.
  * 
  * @param input The raw steering input value (0-255).
  * @param dampening The dampening factor (1.0 = no dampening, higher values increase dampening).
  * @return The dampened steering value (0-255).
  * 
  * This function applies a non-linear curve to the steering input to reduce sensitivity
  * at the extremes while maintaining responsiveness around the center. The dampening factor
  * controls how aggressively this curve is applied.
*/

static uint8_t dampen_steering_sensitivity_8(uint8_t input, float dampening) {
    // Clamp dampening to reasonable range (1.0 = no dampening, higher = more dampening)
    dampening = max(1.0f, dampening);
    
    // Convert to -1.0 to 1.0 range for easier math
    float normalized = (input - 127.0f) / 127.0f;
    
    // Take absolute value to handle both directions the same
    float abs_norm = fabs(normalized);
    
    // Apply curve with configurable dampening
    // dampening parameter controls how aggressive the dampening is
    float curved = powf(abs_norm, dampening) * (dampening - (dampening - 1.0f) * abs_norm);
    
    // Restore direction
    float result = (normalized < 0) ? -curved : curved;
    
    // Convert back to 0-255 range
    return (uint8_t)(result * 127.0f + 127.0f);
}