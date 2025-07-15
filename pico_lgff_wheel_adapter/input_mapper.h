#pragma once

#include "reports.h"

void reset_generic_report(generic_report_t * generic_report);
void map_input(uint8_t const* report, generic_report_t * generic_report, uint16_t vid, uint16_t pid);