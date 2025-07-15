#pragma once

#include <stdint.h>
#include "common.h"
#include "reports.h"
#include "enums.h"

// Enum to identify which type of report is currently active in the union
typedef enum {
    REPORT_TYPE_FGP,
    REPORT_TYPE_FFGP,
    REPORT_TYPE_DF,
    REPORT_TYPE_DFP,
    REPORT_TYPE_DFGT,
    REPORT_TYPE_G25,
    REPORT_TYPE_G27,
    REPORT_TYPE_G29,
    REPORT_TYPE_SFW,
    REPORT_TYPE_MOMOFO,
    REPORT_TYPE_MOMORA
} output_report_type_t;

// The main output mapper struct containing generic report, type signifier, and union of all output reports
typedef struct TU_ATTR_PACKED {
    generic_report_t generic_report;  // Generic report that contains all possible inputs
    output_report_type_t type;       // Type signifier for the union
    union {
        fgp_report_t fgp;           // WingMan Formula GP
        ffgp_report_t ffgp;         // WingMan Formula Force GP
        df_report_t df;             // Driving Force
        dfp_report_t dfp;           // Driving Force Pro
        dfgt_report_t dfgt;         // Driving Force GT
        g25_report_t g25;           // G25 Racing Wheel
        g27_report_t g27;           // G27 Racing Wheel
        g29_report_t g29;           // G29 Racing Wheel
        sfw_report_t sfw;           // Speed Force Wireless
        momofo_report_t momofo;     // MOMO Force
        momora_report_t momora;      // MOMO Racing
    } report;                       // Union of all possible output reports
} output_mapper_t;

void map_output(lg_wheel_output_type output_mode, generic_report_t generic_report, output_mapper_t *output_report); // Function to map the generic report to the specific output report based on the current output mode

void map_fgp_out(output_mapper_t *output, generic_report_t generic_report,uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);
void map_ffgp_out(output_mapper_t *output, generic_report_t generic_report,uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);
void map_df_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);
void map_dfp_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);
void map_dfgt_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);
void map_g25_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);
void map_g27_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);
void map_momofo_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);
void map_momora_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);

void map_sfw_out(output_mapper_t *output, generic_report_t generic_report, uint16_t wheel, uint16_t gas, uint16_t brake, uint16_t clutch);
