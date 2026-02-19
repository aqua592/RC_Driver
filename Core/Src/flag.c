/*
 * flag.c
 *
 *  Created on: Dec 6, 2024
 *      Author: HALAB_G
 */

#include "flag.h"

struct CONTROL_FLAG FLAG = {

              .READY = 1,
              .FAULT = 0,
              .FAULT_CLEAR = 0,
              .INV_RUN = 1,
              .INV_OLC = 0,
			  .INV_VOLC = 0,
              .INV_ALIGN = 0,
              .INV_NLC = 0,
			  .TS_MODE = 0,
			  .TS_MODE_STOP = 0,
			  .DUTY_TEST = 0,
			  .Param_Estimation = 0,
			  .HALL_POS_TEST = 0
};
