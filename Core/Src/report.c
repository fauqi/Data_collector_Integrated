/*
 * report.c
 *
 *  Created on: Sep 29, 2022
 *      Author: janoko
 */

#include "report.h"
#include <string.h>

ReportFrame_t tmpReportFrame = {
    .header = {
        .prefix = REPORT_PREFIX,
        .version = 1,
    }
};

void Report_ResetFrame(ReportFrame_t *reportFrame) {
  memset(&tmpReportFrame.charger, 0, sizeof(tmpReportFrame.charger));
  memset(&tmpReportFrame.bms, 0, sizeof(tmpReportFrame.bms));
}
