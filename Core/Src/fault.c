/*
 * fault.c
 *
 *  Created on: Dec 6, 2024
 *      Author: HALAB_G
 */


#include "inv.h"
#include "control.h"
#include "fault.h"
#include "flag.h"

uint16_t FaultCnt = 0;
struct FAULT_VAL FLTVAL;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	PwmSwOff();
	HardWareFault();
}

void HardWareFault() {

	PwmSwOff();

	FLAG.FAULT = 1;
	FLAG.READY = 0;

    FLTVAL.Vdc = INV.Vdc;
    FLTVAL.Idc = INV.Idc;
    FLTVAL.Ia = INV.Ia;
    FLTVAL.Ib = INV.Ib;
    FLTVAL.Ic = INV.Ic;
    FLTVAL.Wrpm = INV.Wrpm;

	FaultCnt++;
}


void SoftWareFault() {

	PwmSwOff();

	FLAG.READY = 0;
    if (FLAG.FAULT != 1) FLAG.FAULT = 2;

    FLTVAL.Vdc = INV.Vdc;
    FLTVAL.Idc = INV.Idc;
    FLTVAL.Ia = INV.Ia;
    FLTVAL.Ib = INV.Ib;
    FLTVAL.Ic = INV.Ic;
    FLTVAL.Wrpm = INV.Wrpm;

	FaultCnt++;
}

