/*
 * report.h
 *
 *  Created on: Sep 29, 2022
 *      Author: janoko
 */

#ifndef INC_REPORT_H_
#define INC_REPORT_H_

#include <stdint.h>

#define REPORT_PREFIX {0xEA, 0x19}


typedef struct {
  struct __attribute__((packed)) {
    uint8_t prefix[2];
    uint16_t version;
  } header;

  struct __attribute__((packed)) {
    uint8_t status;
    uint8_t faults;
  } charger;

  struct __attribute__((packed)) {
    uint32_t  id;
    uint8_t   status;
    uint16_t  faults;
    int16_t   voltage;
    int16_t   current;
    uint8_t   SOC;
    uint8_t   SOH;
    uint8_t   temperature;
  } bms;

} ReportFrame_t;

extern ReportFrame_t tmpReportFrame;

void Report_ResetFrame(ReportFrame_t *reportFrame);

#endif /* INC_REPORT_H_ */
