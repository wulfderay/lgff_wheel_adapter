#include "led.h"
#include "wheel_ids.h"

#ifdef BOARD_RGB_PIN
  #include "src/Pico_WS2812/WS2812.hpp" //https://github.com/ForsakenNGS/Pico_WS2812
  WS2812 ledStrip(
      BOARD_RGB_PIN,      // Data line is connected to pin 0. (GP0)
      1,                  // Strip is 1 LEDs long.
      pio0,               // Use PIO 0 for creating the state machine.
      3,                  // Index of the state machine that will be created for controlling the LED strip
      WS2812::FORMAT_GRB  // Pixel format used by the LED strip
  );
  static uint8_t last_leds = 0;
#endif

void init_leds(void){
  // Don't set up the built in one (green). That's done elsewhere
  // also, the WS2812 is not set up here, but implicitly above.

  gpio_init(LED_BLUE);
  gpio_set_dir(LED_BLUE, 1);
  gpio_put(LED_BLUE, 0);

  gpio_init(LED_RED);
  gpio_set_dir(LED_RED, 1);
  gpio_put(LED_RED, 0);

  gpio_init(LED_YELLOW);
  gpio_set_dir(LED_YELLOW, 1);
  gpio_put(LED_YELLOW, 0);

  
}

void set_led(uint8_t led, bool value) {
  switch (led){
    case LED_GREEN:
      gpio_put(LED_GREEN, value);
      break;
    case LED_RED:
      gpio_put(LED_RED, value);
      break;
    case LED_YELLOW:
      gpio_put(LED_YELLOW, value);
      break;
    case LED_BLUE:
      gpio_put(LED_BLUE, value);
      break;
  }
}

void leds_display_val(uint8_t value){
  // Display the value on the LEDs
  // 0b0000 = all off
  // 0b0001 = blue
  // 0b0010 = yellow
  // 0b0100 = red
  // 0b1000 = green
  
  // note that the green LED is used as a general status, so it may not
  // stay set by this function.

  set_led(LED_BLUE, value & (1<<0));
  set_led(LED_YELLOW, value & (1<<1));
  set_led(LED_RED, value & (1<<2));
  set_led(LED_GREEN, value & (1<<3));
}


void clear_led_strip(void){
  #ifdef BOARD_RGB_PIN
    ledStrip.fill( WS2812::RGB(0, 0, 0) );
    ledStrip.show();
  #endif
}

void update_led_strip(uint8_t leds_buffer){
  #ifdef BOARD_RGB_PIN
    if(last_leds != leds_buffer) {
      if (leds_buffer & (1<<4))
        ledStrip.fill( WS2812::RGB(60, 00, 0) );
      else if (leds_buffer & (1<<3))
        ledStrip.fill( WS2812::RGB(60, 50, 0) );
      else if (leds_buffer & (1<<2))
        ledStrip.fill( WS2812::RGB(40, 30, 0) );
      else if (leds_buffer & (1<<1))
        ledStrip.fill( WS2812::RGB(10, 40, 0) );
      else if (leds_buffer & 1)
        ledStrip.fill( WS2812::RGB(0, 20, 0) );
      else
        ledStrip.fill( WS2812::RGB(0, 0, 0) );
      last_leds = leds_buffer;
      ledStrip.show();
    }
  #endif
}

void debug_led_strip(uint16_t pid){

  #ifdef BOARD_RGB_PIN
   // set rgb led color. using this for debug
    if (pid == pid_df)
      ledStrip.fill( WS2812::RGB(20, 0, 0) ); //red
    else if (pid == pid_dfp)
      ledStrip.fill( WS2812::RGB(0, 20, 0) ); //green
    else if (pid == pid_dfgt)
      ledStrip.fill( WS2812::RGB(0, 0, 20) ); //blue
    else if (pid == pid_g25)
      ledStrip.fill( WS2812::RGB(20, 20, 0) ); //yellow
    else if (pid == pid_g27)
      ledStrip.fill( WS2812::RGB(20, 0, 20) ); //pink
    else if (pid == pid_g29)
      ledStrip.fill( WS2812::RGB(20, 20, 20) ); //white
    ledStrip.show();
  #endif
}