/*
 * control.c
 *
 *  Created on: Dec 6, 2024
 *      Author: HALAB_G
 */


#include "control.h"
#include "inv.h"
#include "variable.h"
#include "fault.h"
#include "flag.h"
#include "adc.h"


uint32_t ControlCnt;

extern uint8_t T_buffer[0];
//extern SPI_HandleTypeDef hspi3;
extern ADC_HandleTypeDef hadc1;


void Control(void) {

	ControlCnt++;

	//INV.ADC_A_debug = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	//INV.ADC_B_debug = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	//INV.ADC_C_debug = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
	//INV.ADC_Vdc_debug = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);

	//HAL_ADCEx_InjectedStart(&hadc1);

	adc1Val[0] = hadc1.Instance->JDR1; // Ia
	adc1Val[1] = hadc1.Instance->JDR2; // Ib
	adc1Val[2] = hadc1.Instance->JDR3; // Ic
	adc1Val[3] = hadc1.Instance->JDR4; // Vdc

	if (!AdInitFlag)
		Offset();
	else {
		AdcProcess();
	}

//	if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 5) == HAL_OK)
//	{
//	    INV.ADC_A_debug = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//	    INV.ADC_B_debug = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
//	    INV.ADC_C_debug = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
//	    INV.ADC_Vdc_debug = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
//
//	}

    if (ABS(INV.Ia) >= 80.f)  {SoftWareFault();}
    if (ABS(INV.Ib) >= 80.f)  {SoftWareFault();}
    if (ABS(INV.Ic) >= 80.f)  {SoftWareFault();}
//    if (ABS(INV.Wrpm) >= 1000.f)      {SoftWareFault();}

    UpdateController(&INV);

    Hallsensor_Observer(&INV);

	if (FLAG.READY && !(FLAG.FAULT)) {

		if (FLAG.INV_RUN) {
			SpeedControl(&INV);
			Vref_GenControl(&INV);
//			CurrentControl(&INV);
		}
		else if(FLAG.HALL_POS_TEST){ //  Hall_position Test
			HallPosition_Test(&INV);
			PwmSwOn();
			PWM_BUF_ON;
		}
		// use for inverter duty test
		else if(FLAG.DUTY_TEST) {
//			CS_LOW;

			INV.Duty_A = LIMIT(INV.Duty_A, 0.f, 0.95f);
			INV.Duty_B = LIMIT(INV.Duty_B, 0.f, 0.95f);
			INV.Duty_C = LIMIT(INV.Duty_C, 0.f, 0.95f);

			INV.Duty_A = 0.5;
			INV.Duty_B = 0.5;
			INV.Duty_C = 0.5;

			PwmDutyUpt();
			PwmSwOn();
			PWM_BUF_ON;
//			if (!FLAG.FAULT) PWM_BUF_ON;
//			HAL_SPI_Transmit(&hspi3, T_buffer, 1, 10);

//			if (FLAG.FAULT) PWM_BUF_OFF;

		}
		// use for current control test
		else if (FLAG.INV_OLC) {
//			INV.Wrpm_ref_set_OLC = mode_change_rpm;
//			INV.Idsr_ref_OLC = -0.1f;
//			INV.Iqsr_ref_OLC = 2.0f;
			//TorqueControl(&INV);
			//INV.init_align_done = 1;
			OpenLoopControl(&INV);
			CurrentControl(&INV);
		}

		else if (FLAG.INV_Vref_Gen) {
			Vref_GenControl(&INV);
		}

		else if (FLAG.INV_VOLC) {
//			INV.Wrpm_ref_set_OLC = mode_change_rpm;
//			INV.Idsr_ref_OLC = -0.1f;
//			INV.Iqsr_ref_OLC = 2.0f;
			//TorqueControl(&INV);
			//INV.init_align_done = 1;
			VoltageOpenLoopControl(&INV);
//			CurrentControl(&INV);
		}

		else if(FLAG.Param_Estimation){
			VoltageInjection_SquareWave(&INV);
		}
		else if (FLAG.INV_ALIGN) {
			Align(&INV);
		}

		// reset code
		else{
			PwmSwOff();
			PwmDutyUpt();
			ResetController(&INV);

			FLAG.INV_RUN = 0;
			FLAG.INV_OLC = 0;
			FLAG.INV_VOLC = 0;
			FLAG.INV_ALIGN = 0;
			FLAG.INV_NLC = 0;
			FLAG.TS_MODE = 0;
		}
	}

	// reset code
	else {
		PwmSwOff();
		PwmDutyUpt();

		ResetController(&INV);

		FLAG.INV_RUN = 0;
		FLAG.INV_OLC = 0;
		FLAG.INV_VOLC = 0;
		FLAG.INV_ALIGN = 0;
		FLAG.INV_NLC = 0;
		FLAG.TS_MODE = 0;
		FLAG.READY = 0;

	}
	//DAC
	//END_TICK_MANAGER(ControlTime);
}
