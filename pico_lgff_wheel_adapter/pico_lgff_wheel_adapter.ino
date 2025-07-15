/*******************************************************************************
 * Logitech FFB Wheel Adapter 
 * By Matheus Fraguas (sonik-br)
 * https://github.com/sonik-br/lgff_wheel_adapter
 * 
 * Settings are on the config.h file
 *  
 * Credits:
 * Heavily based on matlo's work on GIMX
 * I'm very grateful for it being released as a open source software
 * https://github.com/matlo
 * 
 * Other useful sources of information
 * https://opensource.logitech.com/wiki/Technical_Information/
 * https://github.com/mathijsvandenberg/g29emu
 * https://github.com/berarma/new-lg4ff
 * https://www.lfs.net/forum/thread/74115-LTWheelConf----Setup-Logitech-DFP-G25-G27-on-linux
 * 
 *******************************************************************************/

// pio-usb is required for rp2040 host
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "pico/stdlib.h"

#include "enums.h"
#include "wheel_ids.h"
#include "wheel_commands.h"
#include "input_mapper.h"
#include "output_mapper.h"
#include "config.h"
#include "reports.h"
#include "usb_descriptors.h"
#include "led.h"

// Define the output mode with default value
lg_wheel_output_type output_mode = WHEEL_T_DEFAULT;

// validation
#if defined(EXTERNAL_PEDAL_TYPE) && (!defined(PEDAL_GAS) || !defined(PEDAL_BRAKE))
  #error Need to set PEDAL_GAS and PEDAL_BRAKE analog pins
#endif
#if !defined(USE_TINYUSB) || defined(USE_TINYUSB_HOST)
  #error USB Stack must be configured as "Tools -> USB Stack -> Adafruit TinyUSB"
#endif
#if F_CPU != 120000000 && F_CPU != 240000000
  #error PIO USB require CPU Speed must be 120 or 240 MHz
#endif


// USB Host object
Adafruit_USBH_Host USBHost;

// USB Device object
Adafruit_USBD_HID usb_hid(NULL, 0, HID_ITF_PROTOCOL_NONE, 10, true);

// connected device
uint8_t wheel_addr = 0;
uint8_t wheel_idx = 0;
bool wheel_supports_cmd = 0;

// initialization step
uint8_t mode_step = 0;
uint8_t dongle_step = 0;
lg_wheel_type change_mode_to = NATIVE;
init_stage_status init_stage = DISCONNECTED;

// received command buffer (ffb, leds)
uint8_t cmd_buffer[7] = { 0x00 };
volatile uint8_t leds_buffer = { 0x00 };

#ifdef EXTERNAL_PEDAL_TYPE
  uint32_t external_pedals_values = ~0x0;
#endif

// report to hold input from any wheel
generic_report_t generic_report;

output_mapper_t output_report;

// holds the device description
tusb_desc_device_t desc;



// persisted variables. got the idea from:
// https://github.com/sanjay900/Santroller/blob/master/src/pico/main.cpp
#include "hardware/watchdog.h"
#define PERSISTED_OUTPUT_MODE_VALID 0x3A2F
static uint32_t __uninitialized_ram(persisted_output_mode);
static uint32_t __uninitialized_ram(persisted_output_mode_valid);

void reset_usb(void) {
  persisted_output_mode = output_mode;
  persisted_output_mode_valid = PERSISTED_OUTPUT_MODE_VALID;
  reboot();
}
void reboot(void) {
  watchdog_enable(1, false);
  for (;;) {
  }
}

void handle_bcd_device(uint16_t vid, uint16_t pid, uint16_t bcdDevice) {
  /*
  Notes for G929:
  The Logitech G929 and G923 Playstation wheels start out by default in Playstation Force Feedback Mode. To switch to
  Classic Mode, issue the following byte sequence as an output report to the USB endpoint:
  Logitech G29 Playstation 3: Send this report first 0x21, 0x09, 0x00, 0x02, 0x00, 0x00, 0x10, 0x00
                              Send this report second 0xf8, 0x09, 0x05, 0x01
  Logitech G29 Playstation 4 0x30, 0xf8, 0x09, 0x05, 0x01
  Logitech G923 Playstation 0x30, 0xf8, 0x09, 0x05, 0x01
  */

//  uint16_t vid;
//  uint16_t pid;
//  tuh_vid_pid_get(wheel_addr, &vid, &pid);

  // check if wheel is in compatibility or native mode
  if (pid_g923ps == pid) // G923 in PS mode
    change_mode_to = G923;
  else if ( ((pid_df == pid) || (pid_dfp == pid) || (pid_g25 == pid)) && (0x1350 == (bcdDevice & 0xfff0)) ) // G29 in compatibility mode
    change_mode_to = G29;
  else if ( ((pid_df == pid) || (pid_dfp == pid)) && (0x1300 == (bcdDevice & 0xff00)) ) // DFGT in compatibility mode
    change_mode_to = DFGT;
  else if ( ((pid_df == pid) || (pid_dfp == pid) || (pid_g25 == pid)) && (0x1230 == (bcdDevice & 0xfff0)) ) // G27 in compatibility mode
    change_mode_to = G27;
  else if ( ((pid_df == pid) || (pid_dfp == pid)) && (0x1200 == (bcdDevice & 0xff00)) ) // G25 in compatibility mode
    change_mode_to = G25;
  else if ( (pid_df == pid) && (0x1000 == (bcdDevice & 0xf000)) ) // DFP in compatibility mode
    change_mode_to = DFP;
  else // native mode
    change_mode_to = NATIVE;

  // force a specific mode
  if (force_input_mode != NATIVE) {

    // try to best match input/output mode
    if (auto_mode && force_input_mode == AUTO) {
      // use the previous detected native mode (change_mode_to)
      auto detected_input = change_mode_to;
      switch (detected_input) {
        case DFP:
        {
          if (output_mode == WHEEL_T_DF)
            change_mode_to = DF;
          break;
        }
        case DFGT:
        case G25:
        {
          if (output_mode == WHEEL_T_DF)
            change_mode_to = DF;
          else if (output_mode == WHEEL_T_DFP)
            change_mode_to = DFP;
          break;
        }
        case G27:
        {
          if (output_mode == WHEEL_T_DF)
            change_mode_to = DF;
          else if (output_mode == WHEEL_T_DFP)
            change_mode_to = DFP;
          else if (output_mode == WHEEL_T_G25)
            change_mode_to = G25;
          break;
        }
        case G29:
        {
//          if (output_mode == WHEEL_T_DF)
//            change_mode_to = DF;
          if (output_mode == WHEEL_T_DFP)
            change_mode_to = DFP;
          else if (output_mode == WHEEL_T_DFGT)
            change_mode_to = DFGT;
          else if (output_mode == WHEEL_T_G27)
            change_mode_to = G27;
          break;
        }
      }//end switch

      // force it into DF mode when emulating FGP, FFGP and SFW
      if (output_mode == WHEEL_T_FGP ||output_mode == WHEEL_T_FFGP ||output_mode == WHEEL_T_SFW) {
        // todo implement: only if it supports emulating another mode? 
        change_mode_to = DF;
      }

      //check if it's already in the correct mode, then skip mode change (by setting NATIVE)
      if (   (pid == pid_df   && (output_mode == WHEEL_T_DF || output_mode == WHEEL_T_FGP || output_mode == WHEEL_T_FFGP || output_mode == WHEEL_T_SFW))
          || (pid == pid_dfp  && output_mode == WHEEL_T_DFP)
          || (pid == pid_dfgt && output_mode == WHEEL_T_DFGT)
          || (pid == pid_g25  && output_mode == WHEEL_T_G25)
          || (pid == pid_g27  && output_mode == WHEEL_T_G27) ) {
        change_mode_to = NATIVE;
      }
    } else {
      if (force_input_mode == DF && pid != pid_df)
        change_mode_to = force_input_mode;
      else if (force_input_mode == DFP && pid != pid_dfp)
        change_mode_to = force_input_mode;
      else if (force_input_mode == DFGT && pid != pid_dfgt)
        change_mode_to = force_input_mode;
      else if (force_input_mode == G25 && pid != pid_g25)
        change_mode_to = force_input_mode;
      else if (force_input_mode == G27 && pid != pid_g27)
        change_mode_to = force_input_mode;
      else if (force_input_mode == G29 && pid != pid_g29)
        change_mode_to = force_input_mode;
      else if (force_input_mode == G923 && pid != pid_g923)
        change_mode_to = force_input_mode;
      else if (pid == pid_g923ps) // G923PS can only be changed to G923Classic
        change_mode_to = G923;
      else
        change_mode_to = NATIVE;
    }
  }

  debug_led_strip(pid);

  // set next stage
  if (pid_sfw == pid)
    init_stage = CONFIGURING_DONGLE;
  else
    init_stage = SENDING_CMDS;
  leds_display_val(init_stage);
}

// receive commands from host and keep them to pass to device later
void hid_set_report_callback(uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
  if (bufsize < sizeof(cmd_buffer))
    return;

  // autocentering command compatible with Formula Force EX 
//  if(buffer[0] == 0xfe && buffer[1] == 0x03)
//  {
//    //set_led(LED_BUILTIN, HIGH);
//    //return;
//  }


// On Momo Racing, logitech profiler puts the device in raw mode. also sends longer commands
// 0x07, 0x03, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved, do not use
// 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // turn on Raw Mode
// 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // led
// On Momo Force it also sends some longer commands
// 81, 0b, 82, 83, bb, 00, cc, 00,
  
  if(buffer[0] == 0xf8) { // Extended Command
    /*
    0x01 Change Mode to Driving Force Pro
    0x02 Change Wheel Range to 200 Degrees
    0x03 Change Wheel Range to 900 Degrees
    0x06 Unknown. Some PS2 game sends it
    0x09 Change Device Mode
    0x0a Revert Identity
    0x10 Switch to G25 Identity with USB Detach
    0x11 Switch to G25 Identity without USB Detach
    0x12 Set RPM LEDs
    0x81 Wheel Range Change
    */
    uint8_t ext_cmd = buffer[1];

    if (ext_cmd == 0x12) { // Set RPM LEDs
      // todo use memcpy?
      // todo check if connected device does not support leds, then don't send?
      // todo momo force/racing does have two leds.
      leds_buffer = buffer[2];
    } else if (ext_cmd == 0x02 || ext_cmd == 0x03 || ext_cmd == 0x81) { // Wheel Range Change
      // skip for now. need to test
//      return;
    } else if (ext_cmd == 0x01 || ext_cmd == 0x10 || ext_cmd == 0x11 || ext_cmd == 0x09 || ext_cmd == 0x06) { //mode change commands
      // skip as we curently can't change our output mode at runtime
      
      if (!auto_mode)
        return;

      // testing
      bool detach = false;
      lg_wheel_output_type new_mode = output_mode;

      if (ext_cmd == 0x01) { // Change Mode to Driving Force Pro
        new_mode = WHEEL_T_DFP;
        detach = true;
      } else if (ext_cmd == 0x06) { // Unknow command. Looks to be "Change Mode to Driving Force Pro without USB Detach"
        return;
        //new_mode = WHEEL_T_DFP;
        //detach = false;
      } else if (ext_cmd == 0x10) { // Switch to G25 Identity with USB Detach
        new_mode = WHEEL_T_G25;
        detach = true;
      } else if (ext_cmd == 0x11) { // Switch to G25 Identity without USB Detach
        new_mode = WHEEL_T_G25;
        detach = false;
      } else if (ext_cmd == 0x09) { // Change Device Mode
        detach = buffer[3];
        if (buffer[2] == 0x00) { //  Logitech Driving Force EX
          new_mode = WHEEL_T_DF;
        } else if (buffer[2] == 0x01) { // Logitech Driving Force Pro
          new_mode = WHEEL_T_DFP;
        } else if (buffer[2] == 0x02) { // Logitech G25 Racing Wheel
          new_mode = WHEEL_T_G25;
        } else if (buffer[2] == 0x03) { // Logitech Driving Force GT
          new_mode = WHEEL_T_DFGT;
        } else if (buffer[2] == 0x04) { // Logitech G27 Racing Wheel
          new_mode = WHEEL_T_G27;
        }
      }

      // forcing detach
      detach = true;

      if (new_mode != output_mode) {
        if (detach)
          tud_disconnect();
        delay(2000); // testing. probably not needed.
        output_mode = new_mode;
        if (detach) {
          reset_usb();
        }
      }
      
      return;
    }
  } else { // non extended command

    uint8_t cmd = buffer[0] & 0xf;
    
    // todo handle spring effect differences for Logitech Formula Force GP and Driving Force series?
    // friction only supported on DFP, G25, DFGT, G27 (also g29, g920?)
    if (cmd == 0x00 || cmd == 0x01) { // Download Force, Download and Play Force
      // code below does nothing... yet.
      if (buffer[1] == 0x01) { // Spring
        // todo check if connected devices needs translation
        uint8_t k1 = (buffer[4] & 0x07);
        uint8_t k2 = (buffer[4] & 0x70) >> 4;
        if (k1 == 0x05)
          k1 = 0x06;
        else if (k1 == 0x06)
          k1 = 0x05;
        if (k2 == 0x05)
          k2 = 0x06;
        else if (k2 == 0x06)
          k2 = 0x05;
        //buffer[4] = (k2<<4) | (k1);
      } else if (buffer[1] == 0x03) { // Auto-Centering Spring
        uint8_t k1 = (buffer[3] & 0x07);
        uint8_t k2 = (buffer[4] & 0x07);
        if (k1 == 0x05)
          k1 = 0x06;
        else if (k1 == 0x06)
          k1 = 0x05;
        if (k2 == 0x05)
          k2 = 0x06;
        else if (k2 == 0x06)
          k2 = 0x05;
//        buffer[3] = k1;
//        buffer[4] = k2;
      } else if (buffer[1] == 0x0e) { // Friction
      } else if (buffer[1] == 0x0b) { // High-Resolution Spring (all devices supports?)
      } else if (buffer[1] == 0x0c) { // High-Resolution Damper (all devices supports?)
      } else if (buffer[1] == 0x0d) { // High-Resolution Auto-Centering Spring (all devices supports?)
      }
    } // END Download Force, Download and Play Force
    else if (cmd == 0x07) { // Unknown (seen on logitech profiler with momo racing)
      return; // skip
    } else if (cmd == 0x08) { // Normal mode
      return; // skip
    } else if (cmd == 0x0b) { // Raw mode
      return; // skip
    }

  }

  memcpy(cmd_buffer, buffer, sizeof(cmd_buffer));
}
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report_desc, uint16_t desc_len) {
  uint16_t vid;
  uint16_t pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  if ((vid == 0x046d) && ((pid & 0xff00) == 0xc200)) { // device is a logitech wheel
    wheel_addr = dev_addr;
    wheel_idx = idx;
    mode_step = 0;
    dongle_step = 0;

    wheel_supports_cmd = (pid != pid_fgp);

    init_stage = DISCONNECTED;
    leds_display_val(init_stage);
    // Get Device Descriptor
//    tuh_descriptor_get_device(dev_addr, &desc, 18, receive_device_descriptor, 0);
    tusb_desc_device_t desc_device;
    for (uint8_t i = 0; i < 5; ++i) {
      if (tuh_descriptor_get_device(dev_addr, &desc_device, 18, NULL, 0)) {
        handle_bcd_device(vid, pid, desc_device.bcdDevice);
        break;
      }
      delay(10);
    }
      
    
    //set_led(LED_BUILTIN, HIGH);
    //tuh_hid_receive_report(dev_addr, idx);
  }
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx) {
  if (dev_addr == wheel_addr && idx == wheel_idx) {
    wheel_addr = 0;
    wheel_idx = 0;
    mode_step = 0;
    dongle_step = 0;
    
    // set next stage
    init_stage = DISCONNECTED;
    leds_display_val(init_stage);
    change_mode_to = NATIVE;
    
//    tud_disconnect(); // disconnect to host
    reset_generic_report(&generic_report);
    
    set_led(LED_BUILTIN, LOW);
    clear_led_strip();
    
//    persisted_output_mode_valid = 0x0;
//    reboot();
    
  }
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len) {
  // safe check
  if (len > 0 && dev_addr == wheel_addr && idx == wheel_idx) {

    reset_generic_report(&generic_report);
    
    
    uint16_t vid, pid;
    tuh_vid_pid_get(wheel_addr, &vid, &pid);
    // map the received report to generic_output
    map_input(report, &generic_report, vid, pid);

//    // now map the generic_output to the output_mode
//    map_output();

  }

  // receive next report
  tuh_hid_receive_report(dev_addr, idx);
}


void setup() {
  Serial.end();

  rp2040.enableDoubleResetBootloader();

  //Configure led pin
  #ifdef LED_BUILTIN
    gpio_init(LED_BUILTIN);
    gpio_set_dir(LED_BUILTIN, 1);
    gpio_put(LED_BUILTIN, LOW);
  #endif

  #ifdef LEDS_EXTRA
    init_leds();
    set_led(LED_BLUE, HIGH);
    set_led(LED_RED, HIGH);
    set_led(LED_YELLOW, HIGH);
    delay(500);
    set_led(LED_BLUE, LOW);
    set_led(LED_RED, LOW);
    set_led(LED_YELLOW, LOW);
  #endif
  //using arduino code to keep it simple
  #ifdef EXTERNAL_PEDAL_TYPE
    pinMode(PEDAL_GAS, INPUT);
    pinMode(PEDAL_BRAKE, INPUT);
  #endif

  #if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
    // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
    TinyUSB_Device_Init(0);
  #endif

  // change output mode if it was persisted during a reboot
  if (persisted_output_mode_valid == PERSISTED_OUTPUT_MODE_VALID)
    output_mode = static_cast<lg_wheel_output_type>(persisted_output_mode);

  // set usb device properties
  TinyUSBDevice.setManufacturerDescriptor("Logitech");
  //usb_hid.setPollInterval(10);
  //usb_hid.enableOutEndpoint(true);

  switch (output_mode) {
    case WHEEL_T_FGP:
      TinyUSBDevice.setID(0x046d, pid_fgp);
      TinyUSBDevice.setProductDescriptor(usb_fgp_string_product);
      TinyUSBDevice.setVersion(usb_fgp_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_fgp_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_fgp_hid_report, sizeof(desc_fgp_hid_report));
      break;
    case WHEEL_T_FFGP:
      TinyUSBDevice.setID(0x046d, pid_ffgp);
      TinyUSBDevice.setProductDescriptor(usb_ffgp_string_product);
      TinyUSBDevice.setVersion(usb_ffgp_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_ffgp_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_ffgp_hid_report, sizeof(desc_ffgp_hid_report));
      break;
    case WHEEL_T_DF:
      TinyUSBDevice.setID(0x046d, pid_df);
      TinyUSBDevice.setProductDescriptor(usb_df_string_product);
      TinyUSBDevice.setVersion(usb_df_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_df_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_df_hid_report, sizeof(desc_df_hid_report));
      break;
    case WHEEL_T_DFP:
      TinyUSBDevice.setID(0x046d, pid_dfp);
      TinyUSBDevice.setProductDescriptor(usb_dfp_string_product);
      TinyUSBDevice.setVersion(usb_dfp_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_dfp_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_dfp_hid_report, sizeof(desc_dfp_hid_report));
      break;
    case WHEEL_T_DFGT:
      TinyUSBDevice.setID(0x046d, pid_dfgt);
      TinyUSBDevice.setProductDescriptor(usb_dfgt_string_product);
      TinyUSBDevice.setVersion(usb_dfgt_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_dfgt_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_dfgt_hid_report, sizeof(desc_dfgt_hid_report));
      break;
    case WHEEL_T_G25:
      TinyUSBDevice.setID(0x046d, pid_g25);
      TinyUSBDevice.setProductDescriptor(usb_g25_string_product);
      TinyUSBDevice.setVersion(usb_g25_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_g25_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_g25_hid_report, sizeof(desc_g25_hid_report));
      break;
    case WHEEL_T_G27:
      TinyUSBDevice.setID(0x046d, pid_g27);
      TinyUSBDevice.setProductDescriptor(usb_g27_string_product);
      TinyUSBDevice.setVersion(usb_g27_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_g27_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_g27_hid_report, sizeof(desc_g27_hid_report));
      break;
    case WHEEL_T_MOMOFO:
      TinyUSBDevice.setID(0x046d, pid_momofo);
      TinyUSBDevice.setProductDescriptor(usb_momoforce_string_product);
      TinyUSBDevice.setVersion(usb_momoforce_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_momoforce_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_momoforce_hid_report, sizeof(desc_momoforce_hid_report));
      break;
    case WHEEL_T_MOMORA:
      TinyUSBDevice.setID(0x046d, pid_momora);
      TinyUSBDevice.setProductDescriptor(usb_momoracing_string_product);
      TinyUSBDevice.setVersion(usb_momoracing_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_momoracing_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_momoracing_hid_report, sizeof(desc_momoracing_hid_report));
      break;
    case WHEEL_T_SFW:
      TinyUSBDevice.setID(0x046d, pid_sfw);
      TinyUSBDevice.setProductDescriptor(usb_sfw_string_product);
      TinyUSBDevice.setVersion(usb_sfw_bcd_version); // Set bcdUSB version e.g 1.0, 2.0, 2.1
      TinyUSBDevice.setDeviceVersion(usb_sfw_bcd_device_version); // Set bcdDevice version
      usb_hid.setReportDescriptor(desc_sfw_hid_report, sizeof(desc_sfw_hid_report));
      // bMaxPower 0x1C (56 mA), bInterval 0x02
      break;
  }

  // override version info
  if (auto_mode && persisted_output_mode_valid != PERSISTED_OUTPUT_MODE_VALID) {
    // automode allways start as a Driving Force. But keep the desired Version info
    // only possible with some devices.
    switch (output_mode) {
      case WHEEL_T_DFP:
      case WHEEL_T_DFGT:
      case WHEEL_T_G25:
      case WHEEL_T_G27: {
        output_mode = WHEEL_T_DF;
        TinyUSBDevice.setID(0x046d, pid_df);
        TinyUSBDevice.setProductDescriptor(usb_df_string_product);
        usb_hid.setReportDescriptor(desc_df_hid_report, sizeof(desc_df_hid_report));
        break;
      }
    }
  }

  usb_hid.setReportCallback(NULL, hid_set_report_callback);

  // Leave Device disconnected until a wheel is connected in Host
  tud_disconnect();
  usb_hid.begin();
//
//  // wait until device mounted
//  while ( !TinyUSBDevice.mounted() ) delay(1);


  // USB Host (PIO)
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  /* https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/46 */
  pio_cfg.sm_tx      = 3;
  pio_cfg.sm_rx      = 2;
  pio_cfg.sm_eop     = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch      = 9;
#endif /* ARDUINO_RASPBERRY_PI_PICO_W */

  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  USBHost.begin(1);
}

void loop() {
  static uint8_t last_cmd_buffer[7] { 0x00 };
  static generic_report_t last_report { 0x00 };
  static uint32_t last_millis = 0;

  // TinyUSBDevice.task(); // no need to call here. Arduino Core handles this
  
  USBHost.task();

  //initialization

  if (init_stage == CONFIGURING_DONGLE) { // initialize wii wireless dongle. todo check if command was success

    const uint8_t dongle_cmd_init_comm[]   = { 0xAF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t dongle_cmd_change_addr[] = { 0xB2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    static uint8_t dongle_buffer[8] { 0x0 };

    if (last_millis == 0) { // force an initial delay
      last_millis = millis();
    } else if (millis() - last_millis > 200) { // delay commands. the dongle needs a longer delay than usual
      if (dongle_step == 0) {
        memcpy(dongle_buffer, dongle_cmd_init_comm, sizeof(dongle_buffer));
        if(tuh_hid_set_report(wheel_addr, wheel_idx, 0x00, HID_REPORT_TYPE_FEATURE, dongle_buffer, sizeof(dongle_buffer))) {
          ++dongle_step;
        }
      } else if(dongle_step == 1) {
        memcpy(dongle_buffer, dongle_cmd_change_addr, sizeof(dongle_buffer));
        dongle_buffer[1] = random(0, 255); // random address
        dongle_buffer[2] = random(0, 255);
        if(tuh_hid_set_report(wheel_addr, wheel_idx, 0x00, HID_REPORT_TYPE_FEATURE, dongle_buffer, sizeof(dongle_buffer))) {
          ++dongle_step;
        }
      } else { 
        init_stage = SENDING_CMDS;
        leds_display_val(init_stage);
      }
      last_millis = millis();
    }

  } else if (init_stage == SENDING_CMDS) {

    const struct init_mode_commands *cmd_mode;
    
    if (change_mode_to == DF)
      cmd_mode = &cmd_mode_df;
    else if (change_mode_to == DFP)
      cmd_mode = &cmd_mode_dfp;
    else if (change_mode_to == DFGT)
      cmd_mode = &cmd_mode_dfgt;
    else if (change_mode_to == G25)
      cmd_mode = &cmd_mode_g25;
    else if (change_mode_to == G27)
      cmd_mode = &cmd_mode_g27;
    else if (change_mode_to == G29)
      cmd_mode = &cmd_mode_g29;
    else if (change_mode_to == G923)
      cmd_mode = &cmd_mode_g923;
    else {
      uint16_t vid, pid;
      tuh_vid_pid_get(wheel_addr, &vid, &pid);
      if (pid_g923 == pid) // Set range
        cmd_mode = &cmd_mode_g923_range;
      else
        cmd_mode = &cmd_mode_none;
    }

    if (last_millis == 0) { // force an initial delay
      last_millis = millis();
    } else if (millis() - last_millis > 20) { // delay commands
      if (!wheel_supports_cmd) // skip commands
        mode_step = 255;
      if (mode_step < cmd_mode->cmd_count) {

        // should skip command? skip if all bytes are zero
        bool skip_command = true;
        for (uint8_t i = 0; i < 7; ++i) {
          if (cmd_mode->cmd[(7*(mode_step)) + i]) {
            skip_command = false;
            break;
          }
        }

        uint8_t report_id = (change_mode_to == G923) ? 0x30 : 0x0;

        if (skip_command) {
          ++mode_step;
        }
        else if (tuh_hid_send_report(wheel_addr, wheel_idx, report_id, &cmd_mode->cmd[7*(mode_step)], 7)) {
          ++mode_step;
        }
      }
      // after all initialization commands are sent, disconnect/reconnect the device to force host re-enumeration
      if(mode_step >= cmd_mode->cmd_count) {
        last_millis = 0;
        init_stage = RESTARTING; // set next stage
        
        // no mode change was sent. wheel must be in native mode now. starts receiving inputs!
        if (change_mode_to == NATIVE) {
          init_stage = READY; // set next stage
          leds_display_val(init_stage);
          set_led(LED_BUILTIN, LOW);
          delay(1000);
          set_led(LED_BUILTIN, HIGH);
          tud_connect(); //connect to host
          // wait until device mounted
          while ( !TinyUSBDevice.mounted() ) delay(1);
        
          tuh_hid_receive_report(wheel_addr, wheel_idx);
        }
        return;
      }
      last_millis = millis();
    }
  }
  
  //if using external pedals, override values
  bool external_pedals_updated = false;
  #ifdef EXTERNAL_PEDAL_TYPE
    // GAS
    static uint8_t last_external_gas = 255;
    static uint16_t gas_min = 4096;
    static uint16_t gas_max = 0;
    uint16_t gas_now = analogRead(PEDAL_GAS);
    gas_min = min(gas_min, gas_now);
    gas_max = max(gas_max, gas_now);
    uint8_t external_gas_value = map(gas_now, gas_min, gas_max, 0, 255);
    if (last_external_gas != external_gas_value)
      external_pedals_updated = true;
    last_external_gas = external_gas_value;

    // BRAKE
    static uint8_t last_external_brake = 255;
    static uint16_t brake_min = 4096;
    static uint16_t brake_max = 0;
    uint16_t brake_now = analogRead(PEDAL_BRAKE);
    brake_min = min(brake_min, brake_now);
    brake_max = max(brake_max, brake_now);
    uint8_t external_brake_value = map(brake_now, brake_min, brake_max, 0, 255);
    if (last_external_brake != external_brake_value)
      external_pedals_updated = true;
    last_external_brake = external_brake_value;
    
    external_pedals_values = (external_brake_value << 8) | external_gas_value;


  #endif

  // input report was updated?
  bool report_was_updated = false;
  if (memcmp(&last_report, &generic_report, sizeof(generic_report))) {
    report_was_updated = true;
    memcpy(&last_report, &generic_report, sizeof(generic_report));
  } else if (external_pedals_updated) {
    report_was_updated = true;
  }

  // map generic_output to the output_mode
  if (report_was_updated)
    map_output(output_mode, generic_report, &output_report );

  // send hid report to host
  if (report_was_updated && usb_hid.ready()) {
    switch (output_mode) {
      case WHEEL_T_FGP:
        usb_hid.sendReport(0, &output_report.report.fgp, sizeof(output_report.report.fgp));
        break;
      case WHEEL_T_FFGP:
        usb_hid.sendReport(0, &output_report.report.ffgp, sizeof(output_report.report.ffgp));
        break;
      case WHEEL_T_DF:
        usb_hid.sendReport(0, &output_report.report.df, sizeof(output_report.report.df));
        break;
      case WHEEL_T_DFP:
        usb_hid.sendReport(0, &output_report.report.dfp, sizeof(output_report.report.dfp));
        break;
      case WHEEL_T_DFGT:
        usb_hid.sendReport(0, &output_report.report.dfgt, sizeof(output_report.report.dfgt));
        break;
      case WHEEL_T_G25:
        usb_hid.sendReport(0, &output_report.report.g25, sizeof(output_report.report.g25));
        break;
      case WHEEL_T_G27:
        usb_hid.sendReport(0, &output_report.report.g27, sizeof(output_report.report.g27));
        break;
      case WHEEL_T_MOMOFO:
        usb_hid.sendReport(0, &output_report.report.momofo, sizeof(output_report.report.momofo));
        break;
      case WHEEL_T_MOMORA:
          usb_hid.sendReport(0, &output_report.report.momora, sizeof(output_report.report.momora));
        break;
      case WHEEL_T_SFW:
        usb_hid.sendReport(0, &output_report.report.sfw, sizeof(output_report.report.sfw));
        break;
    }
  }
  
  // send command to device
  if (memcmp(last_cmd_buffer, cmd_buffer, sizeof(cmd_buffer))) {
    if (init_stage == READY && wheel_addr && wheel_supports_cmd) {
      tuh_hid_send_report(wheel_addr, wheel_idx, 0, cmd_buffer, sizeof(cmd_buffer));
    }
    memcpy(last_cmd_buffer, cmd_buffer, sizeof(cmd_buffer));
  }
  
  update_led_strip(leds_buffer);

}




