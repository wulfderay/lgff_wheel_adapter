#include "output_mapper.h"
#include "enums.h"
#include "config.h"
#include <cstring>


// default report values
// WingMan Formula GP
fgp_report_t default_fgp_report {
  .wheel = 0x7f,
  .pedals = 0x7f,
  .gasPedal = 0xff,
  .brakePedal = 0xff,
  
};

// WingMan Formula Force GP (GT Force)
ffgp_report_t default_ffgp_report {
  .wheel = 0x7f,
  .pedals = 0x7f,
  .gasPedal = 0xff,
  .brakePedal = 0xff,
  
};

//Driving Force
df_report_t default_df_report {
  .wheel = 0x7f,
  //.pedal_connected = 1, // drivehub: clear. need to confirm from real a device
  //.power_connected = 1, // drivehub: set. need to confirm from real a device
  .pedals = 0x7f,
  .hat = 0x08,
  .calibated = 1, // drivehub: set
  //.unknown = 1, // drivehub: set
  .gasPedal = 0xff,
  .brakePedal = 0xff
};

//Driving Force Pro (GT Force Pro)
dfp_report_t default_dfp_report {
  .wheel = 0x7f,
  .hat = 0x08,
  .pedals = 0x7f,
  .gasPedal = 0xff,
  .brakePedal = 0xff,
  .pedal_connected = 1,
  .power_connected = 1,
  .calibated = 1,
  .unknown = 1,
};

//Driving Force GT
dfgt_report_t default_dfgt_report {
  .hat = 0x08,
  .pedal_connected = 1,
  .power_connected = 1,
  .calibated = 1,
  .unknown = 0x5,
  .wheel = 0x7f,
  .gasPedal = 0xff,
  .brakePedal = 0xff,
};

//G25 Racing Wheel
g25_report_t default_g25_report {
  .hat = 0x08,
  .pedal_disconnected = 0,
  .power_connected = 1,
  .wheel = 0x7f,
  .gasPedal = 0xff,
  .brakePedal = 0xff,
  .clutchPedal = 0xff,
  .shifter_x = 0x80,
  .shifter_y = 0x80,
  .shifter = 1,
  .unknown = 1
};

//G27 Racing Wheel
g27_report_t default_g27_report {
  .hat = 0x08,
  .wheel = 0x7f,
  .gasPedal = 0xff,
  .brakePedal = 0xff,
  .clutchPedal = 0xff,
  .shifter_x = 0x80,
  .shifter_y = 0x80,
  .pedal_disconnected = 0,
  .calibrated = 1,
  .shifter_connected = 1,
  .unknown = 1,
};

//Speed Force Wireless
sfw_report_t default_sfw_report {
  .wheel = 0x7f,
  .connected = 1, // 0 when not connecded? or not paired?
  .gasPedal = 0xff,
  .brakePedal = 0xff
};

//Momo Force
momofo_report_t default_momofo_report {
  .wheel = 0x7f,
  .pedals = 0x7f,
  .gasPedal = 0xff,
  .brakePedal = 0x00
};

//Momo Racing
momora_report_t default_momora_report {
  .wheel = 0x7f,
  .pedals = 0x7f,
  .gasPedal = 0xff,
  .brakePedal = 0xff
};


void map_output(lg_wheel_output_type output_mode, generic_report_t generic_report, output_mapper_t *output_report) {
  // shift axis values
  const bool pedals_output_precision_16bits = false;
  uint8_t wheel_output_precision;

  switch (output_mode) {
    case WHEEL_T_FGP:
    case WHEEL_T_MOMOFO:
      wheel_output_precision = wheel_8bits;
      break;
    case WHEEL_T_FFGP:
    case WHEEL_T_DF:
    case WHEEL_T_SFW:
    case WHEEL_T_MOMORA:
      wheel_output_precision = wheel_10bits;
      break;
    case WHEEL_T_DFP:
    case WHEEL_T_DFGT:
    case WHEEL_T_G25:
    case WHEEL_T_G27:
      wheel_output_precision = wheel_14bits;
      break;
    default:
      wheel_output_precision = wheel_14bits;
      break;
  }

  uint16_t wheel;
  uint16_t gas;
  uint16_t brake;
  uint16_t clutch;

  if (wheel_output_precision == generic_report.wheel_precision) { // no conversion
    if (generic_report.wheel_precision == wheel_8bits) {
      wheel = generic_report.wheel_8;
    } else if (generic_report.wheel_precision == wheel_10bits) {
      wheel = generic_report.wheel_10;
    } else if (generic_report.wheel_precision == wheel_14bits) {
      wheel = generic_report.wheel_14;
    } else if (generic_report.wheel_precision == wheel_16bits) {
      wheel = generic_report.wheel_16;
    }
  } else if (generic_report.wheel_precision == wheel_8bits) {
    if (wheel_output_precision == wheel_10bits) {
      wheel = generic_report.wheel_8 << 2;
    } else if (wheel_output_precision == wheel_14bits) {
      wheel = generic_report.wheel_8 << 4;
    } else if (wheel_output_precision == wheel_16bits) {
      wheel = generic_report.wheel_8 << 6;
    }
  } else if (generic_report.wheel_precision == wheel_10bits) {
    if (wheel_output_precision == wheel_8bits) {
      wheel = generic_report.wheel_10 >> 2;
    } else if (wheel_output_precision == wheel_14bits) {
      wheel = generic_report.wheel_10 << 4;
    } else if (wheel_output_precision == wheel_16bits) {
      wheel = generic_report.wheel_10 << 6;
    }
  } else if (generic_report.wheel_precision == wheel_14bits) {
    if (wheel_output_precision == wheel_8bits) {
      wheel = generic_report.wheel_14 >> 6;
    }else if (wheel_output_precision == wheel_10bits) {
      wheel = generic_report.wheel_14 >> 4;
    } else if (wheel_output_precision == wheel_16bits) {
      wheel = generic_report.wheel_14 << 2;
    }
  } else if (generic_report.wheel_precision == wheel_16bits) {
    if (wheel_output_precision == wheel_8bits) {
      wheel = generic_report.wheel_16 >> 8;
    } else if (wheel_output_precision == wheel_10bits) {
      wheel = generic_report.wheel_16 >> 6;
    } else if (wheel_output_precision == wheel_14bits) {
      wheel = generic_report.wheel_16 >> 2;
    }
  }

  //if using external pedals, override input.
  #ifdef EXTERNAL_PEDAL_TYPE
    generic_report.pedals_precision_16bits = false;
    generic_report.gasPedal_8 = external_pedals_values & 0xFF;
    generic_report.brakePedal_8 = (external_pedals_values >> 8) & 0xFF;
  #endif

  if (pedals_output_precision_16bits == generic_report.pedals_precision_16bits) { // no conversion
    if (generic_report.pedals_precision_16bits) {
      gas = generic_report.gasPedal_16;
      brake = generic_report.brakePedal_16;
      clutch = generic_report.clutchPedal_16;
    } else {
      gas = generic_report.gasPedal_8;
      brake = generic_report.brakePedal_8;
      clutch = generic_report.clutchPedal_8;
    }
  } else if (pedals_output_precision_16bits && !generic_report.pedals_precision_16bits) {
      gas = generic_report.gasPedal_8 << 8;
      brake = generic_report.brakePedal_8 << 8;
      clutch = generic_report.clutchPedal_8 << 8;
  } else if (!pedals_output_precision_16bits && generic_report.pedals_precision_16bits) {
      gas = generic_report.gasPedal_16 >> 8;
      brake = generic_report.brakePedal_16 >> 8;
      clutch = generic_report.clutchPedal_16 >> 8;
  }

  switch (output_mode) {
    case WHEEL_T_FGP:
      map_fgp_out(output_report, generic_report, wheel, gas, brake, clutch);
      break;
    case WHEEL_T_FFGP:
      map_ffgp_out(output_report, generic_report, wheel, gas, brake, clutch);
      break;
    case WHEEL_T_DF:
      map_df_out(output_report, generic_report, wheel, gas, brake, clutch);
      break;
    case WHEEL_T_DFP:
      map_dfp_out(output_report, generic_report, wheel, gas, brake, clutch);
      break;
    case WHEEL_T_DFGT:
      map_dfgt_out(output_report, generic_report, wheel, gas, brake, clutch);
      break;
    case WHEEL_T_G25:
      map_g25_out(output_report, generic_report, wheel, gas, brake, clutch);
      break;
    case WHEEL_T_G27:
      map_g27_out(output_report, generic_report, wheel, gas, brake, clutch);
      break;
    case WHEEL_T_MOMOFO:
      map_momofo_out(output_report, generic_report, wheel, gas, brake, clutch);
      break;
    case WHEEL_T_MOMORA:
      map_momora_out(output_report, generic_report, wheel, gas, brake, clutch);
      break;
    case WHEEL_T_SFW:
      map_sfw_out(output_report, generic_report, wheel, gas, brake, clutch);
  }
}

void map_fgp_out(output_mapper_t *output, generic_report_t generic_report,uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_FGP;

  // Copy the default report values
  memcpy(&output->report.fgp, &default_fgp_report, sizeof(output->report.fgp));

  output->report.fgp.wheel = wheel;
  output->report.fgp.gasPedal = gas;
  output->report.fgp.brakePedal = brake;
  output->report.fgp.button_top_left = generic_report.cross;
  output->report.fgp.button_top_right = generic_report.square;
  output->report.fgp.button_bot_left = generic_report.circle;
  output->report.fgp.button_bot_right = generic_report.triangle;
  output->report.fgp.rightPaddle = generic_report.R1;
  output->report.fgp.leftPaddle = generic_report.L1;

  //combined pedals. mid: 0x7F. gas pulls to 0x0, brake pulls to 0xFF;
  output->report.fgp.pedals = (~(output->report.fgp.brakePedal>>1) - ~(output->report.fgp.gasPedal>>1)) + 0x7f;

  // Copy the generic report values
  output->generic_report = generic_report;

  // Update the global out_fgp_report with the values from our output mapper
  //out_fgp_report = output->report.fgp;
}

void map_ffgp_out(output_mapper_t *output, generic_report_t generic_report,uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_FFGP;

  memcpy(&output->report.ffgp, &default_ffgp_report, sizeof(output->report.ffgp));
  
  output->report.ffgp.wheel = wheel;
  output->report.ffgp.gasPedal = gas;
  output->report.ffgp.brakePedal = brake;
  output->report.ffgp.button_top_left = generic_report.cross;
  output->report.ffgp.button_top_right = generic_report.square;
  output->report.ffgp.button_bot_left = generic_report.circle;
  output->report.ffgp.button_bot_right = generic_report.triangle;
  output->report.ffgp.rightPaddle = generic_report.R1;
  output->report.ffgp.leftPaddle = generic_report.L1;

  
  //combined pedals. mid: 0x7F. gas pulls to 0x0, brake pulls to 0xFF;
  output->report.ffgp.pedals = (~(output->report.ffgp.brakePedal>>1) - ~(output->report.ffgp.gasPedal>>1)) + 0x7f;
  // Copy the generic report values
  output->generic_report = generic_report;
  //out_ffgp_report = output->report.ffgp;
}

void map_df_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_DF;
  
  // Copy the default report values
  memcpy(&output->report.df, &default_df_report, sizeof(output->report.df));

  output->report.df.wheel = wheel;
  output->report.df.gasPedal = gas;
  output->report.df.brakePedal = brake;
  output->report.df.cross = generic_report.cross;
  output->report.df.square = generic_report.square;
  output->report.df.circle = generic_report.circle;
  output->report.df.triangle = generic_report.triangle;
  output->report.df.R1 = generic_report.R1;
  output->report.df.L1 = generic_report.L1;

  output->report.df.hat = generic_report.hat;
  output->report.df.R2 = generic_report.R2;
  output->report.df.L2 = generic_report.L2;
  output->report.df.select = generic_report.select;
  output->report.df.start = generic_report.start;
  output->report.df.R3 = generic_report.R3;
  output->report.df.L3 = generic_report.L3;

  //combined pedals. mid: 0x7F. gas pulls to 0x0, brake pulls to 0xFF;
  output->report.df.pedals = (~(output->report.df.brakePedal>>1) - ~(output->report.df.gasPedal>>1)) + 0x7f;

  // Copy the generic report values
  output->generic_report = generic_report;
  //out_df_report = output->report.df;
}

void map_dfp_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_DFP;
  // Copy the default report values
  memcpy(&output->report.dfp, &default_dfp_report, sizeof(output->report.dfp));
  output->report.dfp.wheel = wheel;
  output->report.dfp.gasPedal = gas;
  output->report.dfp.brakePedal = brake;
  output->report.dfp.cross = generic_report.cross;
  output->report.dfp.square = generic_report.square;
  output->report.dfp.circle = generic_report.circle;
  output->report.dfp.triangle = generic_report.triangle;
  output->report.dfp.R1 = generic_report.R1;
  output->report.dfp.L1 = generic_report.L1;

  output->report.dfp.hat = generic_report.hat;
  output->report.dfp.R2 = generic_report.R2;
  output->report.dfp.L2 = generic_report.L2;
  output->report.dfp.select = generic_report.select;
  output->report.dfp.start = generic_report.start;
  output->report.dfp.R3 = generic_report.R3;
  output->report.dfp.L3 = generic_report.L3;

  //combined pedals. mid: 0x7F. gas pulls to 0x0, brake pulls to 0xFF;
  output->report.dfp.pedals = (~(output->report.dfp.brakePedal>>1) - ~(output->report.dfp.gasPedal>>1)) + 0x7f;

  output->report.dfp.gear_minus = generic_report.gear_minus;
  output->report.dfp.gear_plus = generic_report.gear_plus;

  // Copy the generic report values
  output->generic_report = generic_report;
  //out_dfp_report = output->report.dfp;
}

void map_dfgt_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_DFGT;
  
  // Copy the default report values
  memcpy(&output->report.dfgt, &default_dfgt_report, sizeof(output->report.dfgt));
  output->report.dfgt.wheel = wheel;
  output->report.dfgt.gasPedal = gas;
  output->report.dfgt.brakePedal = brake;
  output->report.dfgt.cross = generic_report.cross;
  output->report.dfgt.square = generic_report.square;
  output->report.dfgt.circle = generic_report.circle;
  output->report.dfgt.triangle = generic_report.triangle;
  output->report.dfgt.R1 = generic_report.R1;
  output->report.dfgt.L1 = generic_report.L1;

  output->report.dfgt.hat = generic_report.hat;
  output->report.dfgt.R2 = generic_report.R2;
  output->report.dfgt.L2 = generic_report.L2;
  output->report.dfgt.select = generic_report.select;
  output->report.dfgt.start = generic_report.start;
  output->report.dfgt.R3 = generic_report.R3;
  output->report.dfgt.L3 = generic_report.L3;

  output->report.dfgt.gear_minus = generic_report.gear_minus;
  output->report.dfgt.gear_plus = generic_report.gear_plus;

  output->report.dfgt.enter = generic_report.enter;
  output->report.dfgt.plus = generic_report.plus;
  output->report.dfgt.dial_cw = generic_report.dial_cw;
  output->report.dfgt.dial_ccw = generic_report.dial_ccw;
  output->report.dfgt.minus = generic_report.minus;
  output->report.dfgt.horn = generic_report.horn;
  output->report.dfgt.PS = generic_report.PS;

  // Copy the generic report values
  output->generic_report = generic_report;
  //out_dfgt_report = output->report.dfgt;

}

void map_g25_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_G25;

  // Copy the default report values
  memcpy(&output->report.g25, &default_g25_report, sizeof(output->report.g25));
  output->report.g25.wheel = wheel;
  output->report.g25.gasPedal = gas;
  output->report.g25.brakePedal = brake;
  output->report.g25.cross = generic_report.cross;
  output->report.g25.square = generic_report.square;
  output->report.g25.circle = generic_report.circle;
  output->report.g25.triangle = generic_report.triangle;
  output->report.g25.R1 = generic_report.R1;
  output->report.g25.L1 = generic_report.L1;

  output->report.g25.hat = generic_report.hat;
  output->report.g25.R2 = generic_report.R2;
  output->report.g25.L2 = generic_report.L2;
  output->report.g25.select = generic_report.select;
  output->report.g25.start = generic_report.start;
  output->report.g25.R3 = generic_report.R3;
  output->report.g25.L3 = generic_report.L3;

  output->report.g25.clutchPedal = clutch;
  output->report.g25.shifter_x = generic_report.shifter_x;
  output->report.g25.shifter_y = generic_report.shifter_y;
  output->report.g25.shifter_1 = generic_report.shifter_1;
  output->report.g25.shifter_2 = generic_report.shifter_2;
  output->report.g25.shifter_3 = generic_report.shifter_3;
  output->report.g25.shifter_4 = generic_report.shifter_4;
  output->report.g25.shifter_5 = generic_report.shifter_5;
  output->report.g25.shifter_6 = generic_report.shifter_6;
  output->report.g25.shifter_r = generic_report.shifter_r;
  output->report.g25.shifter_stick_down = generic_report.shifter_stick_down;

  // Copy the generic report values
  output->generic_report = generic_report;
  //out_g25_report = output->report.g25;
}

void map_g27_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_G27;

  // Copy the default report values
  memcpy(&output->report.g27, &default_g27_report, sizeof(output->report.g27));
  output->report.g27.wheel = wheel;
  output->report.g27.gasPedal = gas;
  output->report.g27.brakePedal = brake;
  output->report.g27.cross = generic_report.cross;
  output->report.g27.square = generic_report.square;
  output->report.g27.circle = generic_report.circle;
  output->report.g27.triangle = generic_report.triangle;
  output->report.g27.R1 = generic_report.R1;
  output->report.g27.L1 = generic_report.L1;

  output->report.g27.hat = generic_report.hat;
  output->report.g27.R2 = generic_report.R2;
  output->report.g27.L2 = generic_report.L2;
  output->report.g27.select = generic_report.select;
  output->report.g27.start = generic_report.start;
  output->report.g27.R3 = generic_report.R3;
  output->report.g27.L3 = generic_report.L3;

  output->report.g27.R4 = generic_report.R4;
  output->report.g27.L4 = generic_report.L4;
  output->report.g27.R5 = generic_report.R5;
  output->report.g27.L5 = generic_report.L5;

  output->report.g27.clutchPedal = clutch;
  output->report.g27.shifter_x = generic_report.shifter_x;
  output->report.g27.shifter_y = generic_report.shifter_y;
  output->report.g27.shifter_1 = generic_report.shifter_1;
  output->report.g27.shifter_2 = generic_report.shifter_2;
  output->report.g27.shifter_3 = generic_report.shifter_3;
  output->report.g27.shifter_4 = generic_report.shifter_4;
  output->report.g27.shifter_5 = generic_report.shifter_5;
  output->report.g27.shifter_6 = generic_report.shifter_6;
  output->report.g27.shifter_r = generic_report.shifter_r;
  output->report.g27.shifter_stick_down = generic_report.shifter_stick_down;

  // Copy the generic report values
  output->generic_report = generic_report;
  //out_g27_report = output->report.g27;
}

void map_momofo_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_MOMOFO;

  // Copy the default report values
  memcpy(&output->report.momofo, &default_momofo_report, sizeof(output->report.momofo));
  output->report.momofo.wheel = wheel;
  output->report.momofo.gasPedal = gas;
  output->report.momofo.brakePedal = ~brake; // this is correct?
  output->report.momofo.cross = generic_report.cross;
  output->report.momofo.square = generic_report.square;
  output->report.momofo.circle = generic_report.circle;
  output->report.momofo.triangle = generic_report.triangle;
  output->report.momofo.R1 = generic_report.R1;
  output->report.momofo.L1 = generic_report.L1;
  output->report.momofo.select = generic_report.select;
  output->report.momofo.start = generic_report.start;

  //combined pedals. mid: 0x7F. gas pulls to 0x0, brake pulls to 0xFF;
  //output->report.momofo.pedals = (~(output->report.momofo.brakePedal>>1) - ~(output->report.momofo.gasPedal>>1)) + 0x7f;

  // Copy the generic report values
  output->generic_report = generic_report;
  //out_momofo_report = output->report.momofo;
}

void map_momora_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_MOMORA;

  // Copy the default report values
  memcpy(&output->report.momora, &default_momora_report, sizeof(output->report.momora));
  output->report.momora.wheel = wheel;
  output->report.momora.gasPedal = gas;
  output->report.momora.brakePedal = brake; // this is correct?
  output->report.momora.cross = generic_report.cross;
  output->report.momora.square = generic_report.square;
  output->report.momora.circle = generic_report.circle;
  output->report.momora.triangle = generic_report.triangle;
  output->report.momora.R1 = generic_report.R1;
  output->report.momora.L1 = generic_report.L1;
  output->report.momora.select = generic_report.select;
  output->report.momora.start = generic_report.start;

  output->report.momora.gear_minus = generic_report.gear_minus;
  output->report.momora.gear_plus = generic_report.gear_plus;

  //combined pedals. mid: 0x7F. gas pulls to 0x0, brake pulls to 0xFF;
  //out_momora_report.pedals = (~(out_momora_report.brakePedal>>1) - ~(out_momora_report.gasPedal>>1)) + 0x7f;

  // Copy the generic report values
  output->generic_report = generic_report;
  //out_momora_report = output->report.momora;
}

void map_sfw_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch) {
  output->type = REPORT_TYPE_SFW;

  // Copy the default report values
  memcpy(&output->report.sfw, &default_sfw_report, sizeof(output->report.sfw));
  output->report.sfw.wheel = wheel;
  output->report.sfw.gasPedal = gas;
  output->report.sfw.brakePedal = brake;
  output->report.sfw.b = generic_report.cross;
  output->report.sfw.one = generic_report.square;
  output->report.sfw.a = generic_report.circle;
  output->report.sfw.two = generic_report.triangle;

  output->report.sfw.hat_u = 0;
  output->report.sfw.hat_d = 0;
  output->report.sfw.hat_l = 0;
  output->report.sfw.hat_r = 0;
  switch (generic_report.hat) {
    case 0x0:
      output->report.sfw.hat_u = 1;
      break;
    case 0x1:
      output->report.sfw.hat_u = 1;
      output->report.sfw.hat_r = 1;
      break;
    case 0x2:
      output->report.sfw.hat_r = 1;
      break;
    case 0x3:
      output->report.sfw.hat_d = 1;
      output->report.sfw.hat_r = 1;
      break;
    case 0x4:
      output->report.sfw.hat_d = 1;
      break;
    case 0x5:
      output->report.sfw.hat_d = 1;
      output->report.sfw.hat_l = 1;
      break;
    case 0x6:
      output->report.sfw.hat_l = 1;
      break;
    case 0x7:
      output->report.sfw.hat_u = 1;
      output->report.sfw.hat_l = 1;
      break;
    default:
      break;
  }

  output->report.sfw.minus = generic_report.select;
  output->report.sfw.plus = generic_report.start;
  output->report.sfw.home = generic_report.PS;

  // Copy the generic report values
  output->generic_report = generic_report;
  //out_sfw_report = output->report.sfw;
}

