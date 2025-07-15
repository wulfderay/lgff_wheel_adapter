#pragma once

// output mode
enum lg_wheel_output_type {
  WHEEL_T_FGP, //WingMan Formula GP (no FFB)  
  WHEEL_T_FFGP, //WingMan Formula Force GP (GT Force)
  WHEEL_T_DF, //Driving Force
  WHEEL_T_DFP, //Driving Force Pro
  WHEEL_T_DFGT, //Driving Force GT
  WHEEL_T_G25, 
  WHEEL_T_G27, //(need to test) endpoint size 16, interval 2
  WHEEL_T_SFW, //Speed Force Wireless
  WHEEL_T_MOMOFO,
  WHEEL_T_MOMORA, // logitech profiler puts the wheel in raw mode and this is not implemented here
};

enum init_stage_status {
  DISCONNECTED,
  CONFIGURING_DONGLE,
  SENDING_CMDS,
  RESTARTING,
  READY
};

enum lg_wheel_type {
  NATIVE,
  AUTO,
  DF,
  DFP,
  DFGT,
  G25,
  G27,
  G29,
  G923,
};

// external pedal type
enum external_pedal_type {
  PEDAL_G25 // Also G27, G29, G923? ... Both pedals are high when in rest position
  //PEDAL_DFGT
};

//enum wheel_precision {
//  wheel_10bits = 0,
//  wheel_14bits,
//  wheel_16bits,
//};
const uint8_t wheel_8bits  = 0;
const uint8_t wheel_10bits = 1;
const uint8_t wheel_14bits = 2;
const uint8_t wheel_16bits = 3;
