/*
 * inv.c
 *
 *  Created on: Nov 29, 2024
 *      Author: HALAB_G
 */

#include "hrtim.h"
#include "inv.h"
#include "flag.h"
#include "variable.h"
#include "speed_observer.h"
#include "math.h"

extern HRTIM_HandleTypeDef hhrtim1;
extern TIM_HandleTypeDef htim3;
struct INVERTER INV;
extern float dutyCycle;

int Mode_align = 0;
float Time_align = 0.;

int cnt_inj = 0;

float W_LPF_set = 50.f;

void InitParameter(struct INVERTER *INV, float Rs, float Ld, float Lq, float Lamf, float PP, float Jm, float Bm, float Idsr_align, float Is_rated,float Is_limit,float Wrpm_rated,float Te_rated) {

	INV->Rs = Rs;
	INV->Ld = Ld;
	INV->Lq = Lq;
	INV->Ls = Ld;
	INV->Lamf = Lamf;
	INV->PP = PP;
	INV->INV_PP = 1./INV->PP;
	INV->Kt = 1.5*INV->PP*INV->Lamf;
	INV->INV_Kt = 1./INV->Kt;
	INV->Jm = Jm;
	INV->INV_Jm = 1./INV->Jm;
	INV->Bm = Bm;
	INV->Idsr_align = Idsr_align;
	INV->Is_rated = Is_rated;
	INV->Is_limit = Is_limit;
	INV->Te_rated = INV->Kt * INV->Is_rated;
	INV->Te_limit = INV->Kt * INV->Is_limit;
	INV->Wrpm_rated = Wrpm_rated;
	INV->Te_rated = Te_rated;

	INV->MTPA_Te_gap = 0.105263157894737f;
	INV->MTPA_Te_max = 2.f;
	INV->MTPA_Te_gap_INV = 1.f/ 0.105263157894737f;
}

//void InitSpeedController(struct INVERTER *INV, float Wsc, float zeta) {
//
//	INV->Wsc = Wsc;
//	INV->zeta_sc = zeta;
//
//	INV->Kp_sc = INV->Jm*INV->Wsc;
//	INV->Ki_sc = INV->Kp_sc * (INV->Wsc * 0.3f);		// Wpi = Wsc/5;
//	INV->Ka_sc = 1./fmaxf(INV->Kp_sc, 1.e-9);
//
//	INV->Ki_scale = 1.f;
//
//	INV->Wrpm = 0., INV->Wrm = 0., INV->Wr = 0.;
//	INV->Wrpm_ref_set = 0., INV->Wrpm_ref_set_old = 0., INV->Wrm_ref_set = 0.;
//	INV->Wrpm_ref = 0., INV->Wrm_ref = 0.;
//	INV->dWrm = 15000.f*RPM2RM*Tsamp;       //50rpm/s
//	INV->Wrm_err = 0.;
//	INV->Te_ref_integ = 0.;
//	INV->Te_ref_ff = 0.;
//	INV->Te_ref_unsat = 0.;
//	INV->Te_ref = 0.;
//	INV->Te_ref_aw = 0.;
//
//}

void InitCurrentController(struct INVERTER *INV, float Wcc) {

	INV->Wcc = Wcc;
	INV->Kpd_cc = INV->Wcc*INV->Ld;
	INV->Kpq_cc = INV->Wcc*INV->Lq;
	INV->Kid_cc = INV->Wcc*INV->Rs;
	INV->Kiq_cc = INV->Wcc*INV->Rs;

	//INV->Kpd_cc = 1e-6;
	//INV->Kpq_cc = 1e-6;
	//INV->Kid_cc = 0.01;
	//INV->Kiq_cc = 0.01;

	INV->Kad_cc = 1./fmaxf(INV->Kpd_cc, 1.e-9);
	INV->Kaq_cc = 1./fmaxf(INV->Kpq_cc, 1.e-9);

	INV->Ractive = INV->Rs * 1.;

	INV->Idss = 0., INV->Iqss = 0.;
	INV->Idsr = 0., INV->Iqsr = 0.;
	INV->Idsr_err = 0., INV->Iqsr_err = 0.;
	INV->Vdsr_ref_integ = 0., INV->Vqsr_ref_integ = 0.;
	INV->Vdsr_ref_ff = 0., INV->Vqsr_ref_ff = 0.;
	INV->Vdsr_ref_unsat = 0., INV->Vqsr_ref_unsat = 0.;
	INV->Vdsr_ref = 0., INV->Vqsr_ref = 0., INV->Vmag_ref = 0.;
	INV->Vdsr_ref_old = 0., INV->Vqsr_ref_old = 0.;
	INV->Vdsr_ref_aw = 0., INV->Vqsr_ref_aw = 0.;
	INV->Vdss_ref = 0., INV->Vqss_ref = 0.;
	INV->Vas_ref = 0., INV->Vbs_ref = 0., INV->Vcs_ref = 0.;
	INV->Vmax = 0., INV->Vmin = 0., INV->Voffset = 0.;
	INV->Van_ref = 0., INV->Vbn_ref = 0., INV->Vcn_ref = 0.;
	INV->Duty_A = 0., INV->Duty_B = 0., INV->Duty_C = 0.;

	// Open-loop control
	INV->Idsr_ref_OLC = 0.;
	INV->Iqsr_ref_OLC = 0.;
	INV->Vdsr_ref_OLC = 0.;
	INV->Thetar_OLC = 0.;
	INV->Thetar_OLC_buffer = 0.;
	INV->Wrpm_ref_OLC = 0.;
	INV->Wrpm_ref_set_OLC = 0.;
	INV->Wrpm_slope_OLC = 5.f;
	INV->Idsr_slope_OLC = 20.f;
	INV->Idsr_ref_set_OLC = 0.;

	// Nonlinearity compensation
	INV->Idsr_NLC = 0.1;//0.02;
	INV->Idss_ref = 0., INV->Iqss_ref = 0.;
	INV->Ia_ref = 0., INV->Ib_ref = 0., INV->Ic_ref = 0.;
	INV->A_NLC = 3.f; //7.f; //2.00146f;//0.;
	INV->B_NLC = 4.f;//38.9431f;
	INV->C_NLC = 0.;
	INV->Va_NLC = 0., INV->Vb_NLC = 0., INV->Vc_NLC = 0.;

	// Angle
	INV->Thetar = 0.;
	INV->Thetar_adv = 0.;
	INV->Thetar_offset = 0.;
	INV->sinThetar = 0.;
	INV->cosThetar = 0.;
	INV->sinThetar_adv = 0.;
	INV->cosThetar_adv = 0.;
	INV->init_align_done = 0;
	// Duty test
	INV->TestDuty_A = 0.f, INV->TestDuty_B = 0.f, INV->TestDuty_C = 0.f;

	// Hall Position Test
	INV->Duty_Test = 0.01;

	// Sensorless
//	INV->Tmin = 13e-6;
//	INV->Tmin_ad = 2e-6; // From Starting point
//	INV->T_del = 10e-6; // From Starting point to End Point
//	INV->Vd_min = (2.f / 3.f) * INV->Vdc * 2.f * INV->Tmin * Fsamp;
}

void UpdateController(struct INVERTER *INV){

	INV->Kpd_cc = INV->Wcc*INV->Ld;
	INV->Kpq_cc = INV->Wcc*INV->Lq;
	INV->Kid_cc = INV->Wcc*INV->Rs;
	INV->Kiq_cc = INV->Wcc*INV->Rs;

	INV->Kad_cc = 1./fmaxf(INV->Kpd_cc, 1.e-9);
	INV->Kaq_cc = 1./fmaxf(INV->Kpq_cc, 1.e-9);

    INV->Kp_sc = INV->Jm*INV->Wsc;
    INV->Ki_scale = 0.25f;
    INV->Ki_sc = INV->Kp_sc*INV->Wsc*INV->Ki_scale;
    INV->Ka_sc = 1.f/fmaxf(INV->Kp_sc, 1.e-9);


}

void Init_Spd_PLL(struct INVERTER* INV, float Ws){

	//W_SPD_PLL = 2.f * PI * 20.f;		// 2. * PI * 10.;
	INV->W_PLL = Ws;
	INV->Kp_PLL = 2.f * 0.707 * INV->W_PLL;
	INV->Ki_PLL = INV->W_PLL * INV->W_PLL;
	INV->integ_Thetar_PLL = 0.;

	INV->W_PLL_Hall = PI2 * 10.f;
    INV->Kp_PLL_Hall = 2.f * 0.707 * INV->W_PLL_Hall;
    INV->Ki_PLL_Hall = INV->W_PLL_Hall * INV->W_PLL_Hall;
    INV->integ_PLL_Hall = 0.;

}

void ResetController(struct INVERTER *INV) {

	INV->Wrm_ref_set = 0.;
	INV->Wrpm_ref_set = 0.;
	INV->Wrpm_ref_set_old = 0.;
	INV->Wrm_ref = 0.;
	INV->Wrpm_ref = 0.;

	INV->Te_ref_integ = 0.;
	INV->Te_ref_aw = 0.;

	INV->Idsr_ref_integ = 0.;
	INV->Idsr_ref_aw = 0.;

	INV->Te_ref_sat = 0.;
	INV->Te_ref = 0.;

	INV->Idsr_ref = 0.;
	INV->Iqsr_ref = 0.;

	INV->Idsr_ref_OLC = 0.;
	INV->Iqsr_ref_OLC = 0.;
	INV->Vdsr_ref_OLC = 0.;
	INV->Vqsr_ref_OLC = 0.;
	INV->Thetar_OLC = 0.;
	INV->Wrpm_ref_OLC = 0.;
	INV->Wrpm_ref_set_OLC = 0.;
	INV->Wrpm_slope_OLC = 5.f;

	INV->Vdsr_ref_integ = 0.;
	INV->Vqsr_ref_integ = 0.;

	INV->Vdsr_ref_unsat = 0;
	INV->Vqsr_ref_unsat = 0;

	INV->Vdsr_ref_aw = 0.;
	INV->Vqsr_ref_aw = 0.;

	INV->TestDuty_A = 0.;
	INV->TestDuty_B = 0.;
	INV->TestDuty_C = 0.;

	INV->alpha_LPF = 0;

}

//void GenerateTorqueReference(struct INVERTER* INV) {
//	INV->Te_ref = (dutyCycle - 0.15) * 20 * INV->Te_max;
//
//	if (ABS(INV->Te_ref) < 0.05 * INV->Te_max) INV->Te_ref_set = 0.;
//	else INV->Te_ref_set = INV->Te_ref;
//}

void TorqueControl(struct INVERTER *INV) {

	INV->Te_ref = (dutyCycle - 0.15) * 20 * INV->Te_rated;

	if (ABS(INV->Te_ref) < 0.05 * INV->Te_rated) INV->Te_ref = 0.;

	// only q-axis current
	INV->Idsr_ref = 0.;
	INV->Iqsr_ref_unsat = INV->Te_ref*INV->INV_Kt;
	INV->Iqsr_max = sqrtf(INV->Is_limit*INV->Is_limit - INV->Idsr_ref*INV->Idsr_ref);
	INV->Iqsr_ref = LIMIT(INV->Iqsr_ref_unsat, -INV->Iqsr_max, INV->Iqsr_max);

	// MTPA operation
	//	if (fabs(INV->Te_ref) < INV->MTPA_Te_max) {
	//		scaled_Te_ref = fabs(INV->Te_ref) * INV->MTPA_Te_gap_INV;
	//		MTPA_index = floor(scaled_Te_ref);
	//		MTPA_ratio = scaled_Te_ref - MTPA_index;
	//	}
	//	else{
	//		MTPA_index = 20;
	//		MTPA_ratio = 1.;
	//	}
	//
	//	INV->Idsr_ref = Id_MTPA[MTPA_index]  * (1. - MTPA_ratio) + Id_MTPA[MTPA_index + 1] * MTPA_ratio;
	//	INV->Iqsr_ref = (Iq_MTPA[MTPA_index] * (1. - MTPA_ratio) + Iq_MTPA[MTPA_index + 1] * MTPA_ratio) * SIGN(INV->Te_ref);
}

void InitSpeedController(struct INVERTER *INV, float Wsc, float zeta) {

    INV->Wsc = Wsc;
    INV->zeta_sc = zeta;

    INV->Kp_sc = INV->Jm*INV->Wsc;
    INV->Ki_scale = 0.25f;
    INV->Ki_sc = INV->Kp_sc*INV->Wsc*INV->Ki_scale;
    INV->Ka_sc = 1.f/fmaxf(INV->Kp_sc, 1.e-9);

    INV->Wrpm = 0., INV->Wrm = 0., INV->Wr = 0.;
    INV->Wrpm_ref_set = 0., INV->Wrm_ref_set = 0.;
    INV->Wrpm_ref = 0., INV->Wrm_ref = 0.;
//    INV->dWrm = 100.f*RPM2RM*Tsamp;       // 300 rpm/s
//    INV->dWrm = 360.f*RPM2RM*Tsamp;       // 300 rpm/s
//    INV->dWrm = 2000.f*RPM2RM*Tsamp;       // 300 rpm/s
    INV->dWrm = 3000.f*RPM2RM*Tsamp;       // 300 rpm/s
    INV->Wrm_err = 0.;
    INV->Te_ref_integ = 0.;
    INV->Te_ref_ff = 0.;
    INV->Te_ref_unsat = 0.;
    INV->Te_ref = 0.;
    INV->Te_ref_aw = 0.;

}

void CurrentControl(struct INVERTER *INV) {

	if (FLAG.INV_RUN) {
	    INV->Iqsr_ref_unsat = INV->Te_ref*INV->INV_Kt;
	    INV->Iqsr_ref = LIMIT(INV->Iqsr_ref_unsat, -3.*INV->Is_rated, 3.*INV->Is_rated);
	    INV->Idsr_ref = 0;
	}

	INV->Vdsr_ref_old = INV->Vdsr_ref;
	INV->Vqsr_ref_old = INV->Vqsr_ref;

    //INV->Thetar_OLC = BOUND_PI(INV->Thetar_OLC + INV->Wrpm_ref_OLC*RPM2RM*INV->PP*Tsamp);
//    INV->Thetar = INV->Thetar_OLC;

	INV->Idss = (2.f * INV->Ia - INV->Ib - INV->Ic) * INV_3;
	INV->Iqss = (INV->Ib - INV->Ic) * INV_SQRT3;

	Vdss_ref_set = (2.f * Van - Vbn - Vcn) * INV_3;
	Vqss_ref_set = INV_SQRT3 * (Vbn - Vcn);

	//■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
	//■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
	//■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
//	if (INV->init_align_done) {
//		EXT_1.Sin_Thetar_EXT = SIN(EXT_1.Thetar_EXT, EXT_1.Thetar_EXT*EXT_1.Thetar_EXT);
//		EXT_1.Cos_Thetar_EXT = COS(EXT_1.Thetar_EXT*EXT_1.Thetar_EXT);
//
//
//		EXT_1.Vdss_ref_EXT = (EXT_1.Vdss_ref_old + Vdss_ref_set) * 0.5f;
//		EXT_1.Vqss_ref_EXT = (EXT_1.Vqss_ref_old + Vqss_ref_set) * 0.5f;
//
//		EXT_1.Vdss_ref_old = Vdss_ref_set;
//		EXT_1.Vqss_ref_old = Vqss_ref_set;
//
//		EXT_1.Vdse_ref_EXT = EXT_1.Vdss_ref_EXT * EXT_1.Cos_Thetar_EXT + EXT_1.Vqss_ref_EXT * EXT_1.Sin_Thetar_EXT;
//		EXT_1.Vqse_ref_EXT = EXT_1.Vdss_ref_EXT * (-EXT_1.Sin_Thetar_EXT) + EXT_1.Vqss_ref_EXT * EXT_1.Cos_Thetar_EXT;
//
//		EXT_1.Idse_EXT = INV->Idss * EXT_1.Cos_Thetar_EXT + INV->Iqss * EXT_1.Sin_Thetar_EXT;
//		EXT_1.Iqse_EXT = INV->Idss * (-EXT_1.Sin_Thetar_EXT) + INV->Iqss * EXT_1.Cos_Thetar_EXT;
//
//		EXT_1.Err_Idse_EXT = EXT_1.Idse_EXT - EXT_1.Idse_EXT_est;
//		EXT_1.Err_Iqse_EXT = EXT_1.Iqse_EXT - EXT_1.Iqse_EXT_est;
//
//		EXT_1.integ_Idse_EXT_est += Tsamp * EXT_1.Kid_EXT * EXT_1.Err_Idse_EXT;
//		EXT_1.integ_Iqse_EXT_est += Tsamp * EXT_1.Kiq_EXT * EXT_1.Err_Iqse_EXT;
//
//		EXT_1.EEMFd_est = -(EXT_1.Err_Idse_EXT * EXT_1.Kpd_EXT + EXT_1.integ_Idse_EXT_est);
//		EXT_1.EEMFq_est = -(EXT_1.Err_Iqse_EXT * EXT_1.Kpq_EXT + EXT_1.integ_Iqse_EXT_est);
//
//		EXT_1.Vdse_FF_EXT = (EXT_1.Vdse_ref_EXT)+EXT_1.Wr_EXT * EXT_1.Lq_hat * EXT_1.Iqse_EXT;
//		EXT_1.Vqse_FF_EXT = (EXT_1.Vqse_ref_EXT)-EXT_1.Wr_EXT * EXT_1.Lq_hat * EXT_1.Idse_EXT;
//		//Vdse_FF_EXT = (Vdse_ref_EXT)+Wr_EXT_f * Lq_hat * Iqse_EXT;
//		//Vqse_FF_EXT = (Vqse_ref_EXT)-Wr_EXT_f * Lq_hat * Idse_EXT;
//
//		EXT_1.Idse_EXT_est += EXT_1.INV_Ld_hat * Tsamp * (EXT_1.Vdse_FF_EXT - EXT_1.EEMFd_est - EXT_1.Rs_hat * EXT_1.Idse_EXT);
//		EXT_1.Iqse_EXT_est += EXT_1.INV_Ld_hat * Tsamp * (EXT_1.Vqse_FF_EXT - EXT_1.EEMFq_est - EXT_1.Rs_hat * EXT_1.Iqse_EXT);
//
//		if (fabs(EXT_1.EEMFq_est) < 0.05f) {
//			EXT_1.EEMFq_est = 0.05f;
//		}
//
//		EXT_1.Err_Thetar_EXT = atan2f(-EXT_1.EEMFd_est, EXT_1.EEMFq_est);
//		EXT_1.EEMF_mag = sqrt(EXT_1.EEMFd_est * EXT_1.EEMFd_est + EXT_1.EEMFq_est * EXT_1.EEMFq_est);
//		EXT_1.Lamf_est = EXT_1.EEMFq_est / INV->Wr - (INV->Ld-INV->Lq) * INV->Idsr;
//
////		PLL_Adaptive(&PLL_EXT, BOUND_PI(EXT_1.Err_Thetar_EXT - PLL_EXT.Thetar_est));
//
//		////////////////////////////////PI type
//		EXT_1.Thetar_EXT_old = EXT_1.Thetar_EXT;
//
////		EXT_1.Thetar_EXT_PLL_input = BOUND_PI(EXT_1.Err_Thetar_EXT - EXT_1.Thetar_EXT);
////		INV->integ_Thetar_PLL += Tsamp * INV->Ki_PLL * EXT_1.Thetar_EXT_PLL_input;
////		EXT_1.Wr_EXT = EXT_1.k_debug * (INV->integ_Thetar_PLL + EXT_1.Thetar_EXT_PLL_input * INV->Kp_PLL);
//
//		INV->integ_Thetar_PLL += Tsamp * INV->Ki_PLL * EXT_1.Err_Thetar_EXT;
//		EXT_1.Wr_EXT = EXT_1.k_debug * (INV->integ_Thetar_PLL + EXT_1.Err_Thetar_EXT * INV->Kp_PLL);
//
//		EXT_1.Thetar_EXT += EXT_1.Wr_EXT * Tsamp ;
//		EXT_1.Thetar_EXT = BOUND_PI(EXT_1.Thetar_EXT);
//
//		EXT_1.Thetarm_EXT += (EXT_1.Wr_EXT * INV->INV_PP) * Tsamp;
//		EXT_1.Thetarm_EXT = BOUND_PI(EXT_1.Thetarm_EXT );
//
//		EXT_1.W_LPF = PI2 * W_LPF_set;
//		EXT_1.a_LPF = EXT_1.W_LPF / (EXT_1.W_LPF + Fsamp);
//		EXT_1.Wr_EXT_f = EXT_1.a_LPF * EXT_1.Wr_EXT + (1.f-EXT_1.a_LPF) * EXT_1.Wr_EXT_f;
//		EXT_1.Wrm_EXT_f = EXT_1.Wr_EXT_f * INV->INV_PP;
//		EXT_1.Wrpm_EXT_f = EXT_1.Wrm_EXT_f * RM2RPM;
//	}

	if (Theta_mode == 1){
		INV->Thetar = INV->Thetar_OLC;
		INV->Wr = INV->Wrpm_ref_OLC *RPM2RM * INV->PP;
	}

	else {
		INV->Thetar = INV->Thetar_est_Hall;
		INV->Wr = INV->Wr_est_Hall;
	}

//	INV->Thetar_err = EXT_1.Thetar_EXT_old - INV->Thetar_OLC;

//	INV->Thetar_err_ENC = BOUND_PI(INV->Thetar_ENC - INV->Thetar_est_ENC);
//	INV->integ_Thetar_PLL += Tsamp * INV->Ki_PLL * INV->Thetar_err_ENC;
//	INV->Wr_est_ENC = INV->Kp_PLL * INV->Thetar_err_ENC + INV->integ_Thetar_PLL;
//	INV->Thetar_est_ENC += Tsamp * INV->Wr_est_ENC;
//	INV->Thetar_est_ENC = BOUND_PI(INV->Thetar_est_ENC);

//	INV->Thetar = INV->Thetar_OLC;
//	INV->Wr = INV->Wrpm_ref_OLC*RPM2RM*INV->PP;

//	switch (Theta_mode) {
//	case 0: // OLC
//		INV->Thetar = INV->Thetar_OLC;
//		INV->Wr = INV->Wrpm_ref_OLC*RPM2RM*INV->PP;
//		break;
//	case 1: // ENC
//		if(Align_done == 1){
//			INV->Thetar = INV->Thetar_ENC;
//			INV->Wr = INV->Wr_est_ENC;
//		}
//		else {
//			INV->Thetar = 0.f;
//			INV->Wr = 0.f;
//		}
//		break;
//	case 2: // Sensorless (Mix)
//		if(Align_done == 1){
//			INV->Thetar = EXT_1.Thetar_EXT_old;
//			INV->Wr = EXT_1.Wr_EXT_f;
//		}
//		else{
//			INV->Thetar = 0.f;
//			INV->Wr = 0.f;
//		}
//		break;
//	}
	INV->Wrm = INV->Wr * INV->INV_PP;
	INV->Wrpm = INV->Wrm * RM2RPM;

	#if ANGLE_COMPENSATION == ON
	    INV->Thetar_adv = BOUND_PI(INV->Thetar + 1.5f*INV->Wr*Tsamp);
	#else
	    INV->Thetar_adv = INV->Thetar;
	#endif

	//■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
	//■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
	//■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■

    INV->cosThetar = COS(INV->Thetar * INV->Thetar);
    INV->sinThetar = SIN(INV->Thetar, INV->Thetar * INV->Thetar);

    INV->Thetar_adv = BOUND_PI(INV->Thetar + 1.5f*INV->Wr*Tsamp);

    INV->cosThetar_adv = COS(INV->Thetar_adv * INV->Thetar_adv);
    INV->sinThetar_adv = SIN(INV->Thetar_adv, INV->Thetar_adv * INV->Thetar_adv);

    INV->Idsr = INV->Idss*INV->cosThetar + INV->Iqss*INV->sinThetar;
    INV->Iqsr = -INV->Idss*INV->sinThetar + INV->Iqss*INV->cosThetar;

    INV->Idsr_err = INV->Idsr_ref - INV->Idsr;
    INV->Iqsr_err = INV->Iqsr_ref - INV->Iqsr;

	INV->Vdsr_ref_integ += INV->Kid_cc * (INV->Idsr_err - INV->Kad_cc * INV->Vdsr_ref_aw) * Tsamp;
	INV->Vqsr_ref_integ += INV->Kiq_cc * (INV->Iqsr_err - INV->Kaq_cc * INV->Vqsr_ref_aw) * Tsamp;
	INV->Vdsr_ref_ff = -INV->Wr * INV->Lq * INV->Iqsr_ref;
	INV->Vqsr_ref_ff = INV->Wr * (INV->Ld * INV->Idsr_ref + INV->Lamf);
	INV->Vdsr_ref_unsat_old = INV->Kpd_cc * INV->Idsr_err + INV->Vdsr_ref_integ + INV->Vdsr_ref_ff - INV->Ractive * INV->Idsr;
	INV->Vqsr_ref_unsat_old = INV->Kpq_cc * INV->Iqsr_err + INV->Vqsr_ref_integ + INV->Vqsr_ref_ff - INV->Ractive * INV->Iqsr;

	INV->Vdsr_ref_unsat = INV->alpha_LPF * INV->Vdsr_ref_unsat + (1 - INV->alpha_LPF) * INV->Vdsr_ref_unsat_old;
	INV->Vqsr_ref_unsat = INV->alpha_LPF * INV->Vqsr_ref_unsat + (1 - INV->alpha_LPF) * INV->Vqsr_ref_unsat_old;

//	INV.Vdc = ADC5_Result * ADC_Gain_Vdc;
//	INV.Vdc_control = 0.999f * INV.Vdc_control + 0.001f * INV.Vdc;

//	INV->Vdsr_ref_integ += (INV->Kid_cc*(INV->Idsr_err - INV->Kad_cc*INV->Vdsr_ref_aw) - INV->Wr*INV->Kpd_cc*(INV->Iqsr_err - INV->Kaq_cc*INV->Vqsr_ref_aw))*Tsamp;
//	INV->Vqsr_ref_integ += (INV->Kiq_cc*(INV->Iqsr_err - INV->Kaq_cc*INV->Vqsr_ref_aw) + INV->Wr*INV->Kpq_cc*(INV->Idsr_err - INV->Kad_cc*INV->Vdsr_ref_aw))*Tsamp;
//	INV->Vdsr_ref_ff = 0.f;
//	INV->Vqsr_ref_ff = INV->Wr*INV->Lamf;
//	INV->Vdsr_ref_unsat = INV->Kpd_cc*INV->Idsr_err + INV->Vdsr_ref_integ + INV->Vdsr_ref_ff;
//	INV->Vqsr_ref_unsat = INV->Kpq_cc*INV->Iqsr_err + INV->Vqsr_ref_integ + INV->Vqsr_ref_ff;

	INV->Vdsr_ref = LIMIT(INV->Vdsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
	INV->Vqsr_ref = LIMIT(INV->Vqsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
	//arm_sqrt_f32(INV->Vdsr_ref*INV->Vdsr_ref + INV->Vqsr_ref*INV->Vqsr_ref, &INV->Vmag_ref);
	INV->Vdsr_ref_aw = INV->Vdsr_ref_unsat - INV->Vdsr_ref;
	INV->Vqsr_ref_aw = INV->Vqsr_ref_unsat - INV->Vqsr_ref;
	INV->Vdss_ref = INV->Vdsr_ref*INV->cosThetar_adv - INV->Vqsr_ref*INV->sinThetar_adv;
	INV->Vqss_ref = INV->Vdsr_ref*INV->sinThetar_adv + INV->Vqsr_ref*INV->cosThetar_adv;

    INV->Vas_ref = INV->Vdss_ref;
    INV->Vbs_ref = -0.5f*INV->Vdss_ref + SQRT3HALF*INV->Vqss_ref;
    INV->Vcs_ref = -0.5f*INV->Vdss_ref - SQRT3HALF*INV->Vqss_ref;

    INV->Vmax = INV->Vas_ref;
    if (INV->Vmax < INV->Vbs_ref) INV->Vmax = INV->Vbs_ref;
    if (INV->Vmax < INV->Vcs_ref) INV->Vmax = INV->Vcs_ref;

    INV->Vmin = INV->Vas_ref;
    if (INV->Vmin > INV->Vbs_ref) INV->Vmin = INV->Vbs_ref;
    if (INV->Vmin > INV->Vcs_ref) INV->Vmin = INV->Vcs_ref;

    INV->Voffset = -0.5f * (INV->Vmax + INV->Vmin);

    INV->Idss_ref = INV->Idsr_ref*INV->cosThetar_adv - INV->Iqsr_ref*INV->sinThetar_adv;
    INV->Iqss_ref = INV->Idsr_ref*INV->sinThetar_adv + INV->Iqsr_ref*INV->cosThetar_adv;

    INV->Ia_ref = INV->Idss_ref;
    INV->Ib_ref = -0.5f*INV->Idss_ref + SQRT3HALF*INV->Iqss_ref;
    INV->Ic_ref = -0.5f*INV->Idss_ref - SQRT3HALF*INV->Iqss_ref;

    INV->Va_NLC = INV->A_NLC*atanf(INV->B_NLC*INV->Ia_ref);
    INV->Vb_NLC = INV->A_NLC*atanf(INV->B_NLC*INV->Ib_ref);
    INV->Vc_NLC = INV->A_NLC*atanf(INV->B_NLC*INV->Ic_ref);

    INV->Van_ref = LIMIT(INV->Vas_ref + INV->Voffset + INV->Va_NLC, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
    INV->Vbn_ref = LIMIT(INV->Vbs_ref + INV->Voffset + INV->Vb_NLC, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
    INV->Vcn_ref = LIMIT(INV->Vcs_ref + INV->Voffset + INV->Vc_NLC, -0.5f*INV->Vdc, 0.5f*INV->Vdc);

    Van = INV->Van_ref;
    Vbn = INV->Vbn_ref;
    Vcn = INV->Vcn_ref;

    INV->Duty_A = (INV->Van_ref*INV->INV_Vdc + 0.5f);
    INV->Duty_B = (INV->Vbn_ref*INV->INV_Vdc + 0.5f);
    INV->Duty_C = (INV->Vcn_ref*INV->INV_Vdc + 0.5f);

    INV->Duty_A = LIMIT(INV->Duty_A, 0.f, 1.f);
    INV->Duty_B = LIMIT(INV->Duty_B, 0.f, 1.f);
    INV->Duty_C = LIMIT(INV->Duty_C, 0.f, 1.f);

	PwmSwOn();
	PwmDutyUpt();

	if (!FLAG.FAULT) PWM_BUF_ON;

}

void CurrentControl_OLC(struct INVERTER *INV) {

	INV->Vdsr_ref_old = INV->Vdsr_ref;
	INV->Vqsr_ref_old = INV->Vqsr_ref;

	INV->Idss = (2.f * INV->Ia - INV->Ib - INV->Ic) * INV_3;
	INV->Iqss = (INV->Ib - INV->Ic) * INV_SQRT3;

	INV->Thetar = INV->Thetar_OLC;
	INV->Wr = INV->Wrpm_ref_OLC*RPM2RM*INV->PP;
	INV->Wrm = INV->Wr * INV->INV_PP;
	INV->Wrpm = INV->Wrm * RM2RPM;

	#if ANGLE_COMPENSATION == ON
	    INV->Thetar_adv = BOUND_PI(INV->Thetar + 1.5f*INV->Wr*Tsamp);
	#else
	    INV->Thetar_adv = INV->Thetar;
	#endif

    INV->cosThetar = COS(INV->Thetar * INV->Thetar);
    INV->sinThetar = SIN(INV->Thetar, INV->Thetar * INV->Thetar);

//    INV->Thetar_adv = BOUND_PI(INV->Thetar + 1.5f*INV->Wr*Tsamp);

    INV->cosThetar_adv = COS(INV->Thetar_adv * INV->Thetar_adv);
    INV->sinThetar_adv = SIN(INV->Thetar_adv, INV->Thetar_adv * INV->Thetar_adv);

    INV->Idsr = INV->Idss*INV->cosThetar + INV->Iqss*INV->sinThetar;
    INV->Iqsr = -INV->Idss*INV->sinThetar + INV->Iqss*INV->cosThetar;

    INV->Idsr_err = INV->Idsr_ref - INV->Idsr;
    INV->Iqsr_err = INV->Iqsr_ref - INV->Iqsr;

	INV->Vdsr_ref_integ += (INV->Kid_cc*(INV->Idsr_err - INV->Kad_cc*INV->Vdsr_ref_aw) - INV->Wr*INV->Kpd_cc*(INV->Iqsr_err - INV->Kaq_cc*INV->Vqsr_ref_aw))*Tsamp;
	INV->Vqsr_ref_integ += (INV->Kiq_cc*(INV->Iqsr_err - INV->Kaq_cc*INV->Vqsr_ref_aw) + INV->Wr*INV->Kpq_cc*(INV->Idsr_err - INV->Kad_cc*INV->Vdsr_ref_aw))*Tsamp;
	INV->Vdsr_ref_ff = 0.f;
	INV->Vqsr_ref_ff = INV->Wr*INV->Lamf;
	INV->Vdsr_ref_unsat = INV->Kpd_cc*INV->Idsr_err + INV->Vdsr_ref_integ + INV->Vdsr_ref_ff;
	INV->Vqsr_ref_unsat = INV->Kpq_cc*INV->Iqsr_err + INV->Vqsr_ref_integ + INV->Vqsr_ref_ff;
	INV->Vdsr_ref = LIMIT(INV->Vdsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
	INV->Vqsr_ref = LIMIT(INV->Vqsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
	//arm_sqrt_f32(INV->Vdsr_ref*INV->Vdsr_ref + INV->Vqsr_ref*INV->Vqsr_ref, &INV->Vmag_ref);
	INV->Vdsr_ref_aw = INV->Vdsr_ref_unsat - INV->Vdsr_ref;
	INV->Vqsr_ref_aw = INV->Vqsr_ref_unsat - INV->Vqsr_ref;
	INV->Vdss_ref = INV->Vdsr_ref*INV->cosThetar_adv - INV->Vqsr_ref*INV->sinThetar_adv;
	INV->Vqss_ref = INV->Vdsr_ref*INV->sinThetar_adv + INV->Vqsr_ref*INV->cosThetar_adv;

    INV->Vas_ref = INV->Vdss_ref;
    INV->Vbs_ref = -0.5f*INV->Vdss_ref + SQRT3HALF*INV->Vqss_ref;
    INV->Vcs_ref = -0.5f*INV->Vdss_ref - SQRT3HALF*INV->Vqss_ref;

    INV->Vmax = INV->Vas_ref;
    if (INV->Vmax < INV->Vbs_ref) INV->Vmax = INV->Vbs_ref;
    if (INV->Vmax < INV->Vcs_ref) INV->Vmax = INV->Vcs_ref;

    INV->Vmin = INV->Vas_ref;
    if (INV->Vmin > INV->Vbs_ref) INV->Vmin = INV->Vbs_ref;
    if (INV->Vmin > INV->Vcs_ref) INV->Vmin = INV->Vcs_ref;

    INV->Voffset = -0.5f * (INV->Vmax + INV->Vmin);

    INV->Idss_ref = INV->Idsr_ref*INV->cosThetar_adv - INV->Iqsr_ref*INV->sinThetar_adv;
    INV->Iqss_ref = INV->Idsr_ref*INV->sinThetar_adv + INV->Iqsr_ref*INV->cosThetar_adv;

    INV->Ia_ref = INV->Idss_ref;
    INV->Ib_ref = -0.5f*INV->Idss_ref + SQRT3HALF*INV->Iqss_ref;
    INV->Ic_ref = -0.5f*INV->Idss_ref - SQRT3HALF*INV->Iqss_ref;

    INV->Va_NLC = INV->A_NLC*atanf(INV->B_NLC*INV->Ia_ref);
    INV->Vb_NLC = INV->A_NLC*atanf(INV->B_NLC*INV->Ib_ref);
    INV->Vc_NLC = INV->A_NLC*atanf(INV->B_NLC*INV->Ic_ref);

    INV->Van_ref = LIMIT(INV->Vas_ref + INV->Voffset + INV->Va_NLC, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
    INV->Vbn_ref = LIMIT(INV->Vbs_ref + INV->Voffset + INV->Vb_NLC, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
    INV->Vcn_ref = LIMIT(INV->Vcs_ref + INV->Voffset + INV->Vc_NLC, -0.5f*INV->Vdc, 0.5f*INV->Vdc);

    INV->Duty_A = (INV->Van_ref*INV->INV_Vdc + 0.5f);
    INV->Duty_B = (INV->Vbn_ref*INV->INV_Vdc + 0.5f);
    INV->Duty_C = (INV->Vcn_ref*INV->INV_Vdc + 0.5f);

    INV->Duty_A = LIMIT(INV->Duty_A, 0.f, 1.f);
    INV->Duty_B = LIMIT(INV->Duty_B, 0.f, 1.f);
    INV->Duty_C = LIMIT(INV->Duty_C, 0.f, 1.f);

	PwmDutyUpt();
	PwmSwOn();

	if (!FLAG.FAULT) PWM_BUF_ON;

}

void SpeedControl(struct INVERTER *INV) {

	INV->Wrpm_ref_set = (dutyCycle - 0.15) * 20 * INV->Wrpm_rated;
	if (ABS(INV->Wrpm_ref_set) < 0.05 * INV->Wrpm_rated) INV->Wrpm_ref_set = 0.;

    INV->Wrm_ref_set = INV->Wrpm_ref_set*RPM2RM;
    if (INV->Wrm_ref < INV->Wrm_ref_set - INV->dWrm) {
        INV->Wrm_ref += INV->dWrm;
    } else if (INV->Wrm_ref > INV->Wrm_ref_set + INV->dWrm) {
        INV->Wrm_ref -= INV->dWrm;
    } else {
        INV->Wrm_ref = INV->Wrm_ref_set;
    }
    INV->Wrpm_ref = INV->Wrm_ref*RM2RPM;

    INV->Wrm_err = INV->Wrm_ref - INV->Wrm;

    INV->Te_ref_integ += INV->Ki_sc*(INV->Wrm_err - INV->Ka_sc*INV->Te_ref_aw)*Tsamp;
//    INV->Te_ref_ff = 0.f;
//    INV->Te_ref_unsat = INV->Kp_sc*INV->Wrm_err + INV->Te_ref_integ + INV->Te_ref_ff;
    INV->Te_ref_unsat = INV->Kp_sc*INV->Wrm_err + INV->Te_ref_integ;

    INV->Te_ref = LIMIT(INV->Te_ref_unsat, -INV->Te_rated, INV->Te_rated);
    INV->Te_ref_aw = INV->Te_ref_unsat - INV->Te_ref;

}


void OpenLoopControl(struct INVERTER *INV) {

    if     (INV->Idsr_ref_set_OLC > INV->Idsr_ref_OLC + Tsamp * INV->Idsr_slope_OLC)       INV->Idsr_ref_OLC += Tsamp * INV->Idsr_slope_OLC;
    else if(INV->Idsr_ref_set_OLC < INV->Idsr_ref_OLC - Tsamp * INV->Idsr_slope_OLC)       INV->Idsr_ref_OLC -= Tsamp * INV->Idsr_slope_OLC;
    else													                               INV->Idsr_ref_OLC =  INV->Idsr_ref_set_OLC;

    INV->Idsr_ref = INV->Idsr_ref_OLC;
    INV->Iqsr_ref = INV->Iqsr_ref_OLC;

//    INV->Idsr_ref = 0.;
//    INV->Iqsr_ref = INV->Te_ref / INV->Kt;

    if     (INV->Wrpm_ref_set_OLC > INV->Wrpm_ref_OLC + Tsamp * INV->Wrpm_slope_OLC)       INV->Wrpm_ref_OLC += Tsamp * INV->Wrpm_slope_OLC;
    else if(INV->Wrpm_ref_set_OLC < INV->Wrpm_ref_OLC - Tsamp * INV->Wrpm_slope_OLC)       INV->Wrpm_ref_OLC -= Tsamp * INV->Wrpm_slope_OLC;
    else													                               INV->Wrpm_ref_OLC =  INV->Wrpm_ref_set_OLC;

    INV->Thetar_OLC = BOUND_PI(INV->Thetar_OLC + INV->Wrpm_ref_OLC*RPM2RM*INV->PP*Tsamp);
//    INV->Thetar_OLC = 0;

}

void Vref_GenControl(struct INVERTER *INV) {

	if(Theta_mode == 1) {
	    if     (INV->Idsr_ref_set_OLC > INV->Idsr_ref_OLC + Tsamp * INV->Idsr_slope_OLC)       INV->Idsr_ref_OLC += Tsamp * INV->Idsr_slope_OLC;
	    else if(INV->Idsr_ref_set_OLC < INV->Idsr_ref_OLC - Tsamp * INV->Idsr_slope_OLC)       INV->Idsr_ref_OLC -= Tsamp * INV->Idsr_slope_OLC;
	    else													                               INV->Idsr_ref_OLC =  INV->Idsr_ref_set_OLC;

	    INV->Idsr_ref = INV->Idsr_ref_OLC;
	    INV->Iqsr_ref = INV->Iqsr_ref_OLC;

	    if     (INV->Wrpm_ref_set_OLC > INV->Wrpm_ref_OLC + Tsamp * INV->Wrpm_slope_OLC)       INV->Wrpm_ref_OLC += Tsamp * INV->Wrpm_slope_OLC;
	    else if(INV->Wrpm_ref_set_OLC < INV->Wrpm_ref_OLC - Tsamp * INV->Wrpm_slope_OLC)       INV->Wrpm_ref_OLC -= Tsamp * INV->Wrpm_slope_OLC;
	    else													                               INV->Wrpm_ref_OLC =  INV->Wrpm_ref_set_OLC;

	    INV->Wr_ref_OLC = INV->Wrpm_ref_OLC *RPM2RM *INV->PP;

	    INV->Vdsr_ref_OLC = INV->Rs * INV->Idsr_ref_OLC - INV->Wr_ref_OLC * (INV->Lq * INV->Iqsr_ref_OLC);
	    INV->Vqsr_ref_OLC = INV->Rs * INV->Iqsr_ref_OLC + INV->Wr_ref_OLC * (INV->Ld * INV->Idsr_ref_OLC + INV->Lamf);

	    INV->Vdsr_ref_unsat = INV->Vdsr_ref_OLC;
		INV->Vqsr_ref_unsat = INV->Vqsr_ref_OLC;

		INV->Thetar_OLC = BOUND_PI(INV->Thetar_OLC + INV->Wr_ref_OLC * Tsamp);
		INV->Thetar = INV->Thetar_OLC;
	}

	else {

		INV->Iqsr_ref_unsat = INV->Te_ref*INV->INV_Kt;
		INV->Iqsr_ref = LIMIT(INV->Iqsr_ref_unsat, -1.3*INV->Is_rated, 1.3*INV->Is_rated);
		INV->Idsr_ref = 0;

	    INV->Vdsr_ref = INV->Rs * INV->Idsr_ref - INV->Wrm_ref * INV->PP * (INV->Lq * INV->Iqsr_ref);
	    INV->Vqsr_ref = INV->Rs * INV->Iqsr_ref + INV->Wrm_ref * INV->PP * (INV->Ld * INV->Idsr_ref + INV->Lamf);

	    INV->Vdsr_ref_unsat = INV->Vdsr_ref;
		INV->Vqsr_ref_unsat = INV->Vqsr_ref;

		INV->Thetar = INV->Thetar_est_Hall;

	}

//    INV->Idss = (2.f * INV->Ia - INV->Ib - INV->Ic) * INV_3;
//    INV->Iqss = (INV->Ib - INV->Ic) * INV_SQRT3;

//    Vdss_ref_set = (2.f * Van - Vbn - Vcn) * INV_3;
//    Vqss_ref_set = INV_SQRT3 * (Vbn - Vcn);

//	if (INV->init_align_done) {
//		EXT_1.Sin_Thetar_EXT = SIN(EXT_1.Thetar_EXT, EXT_1.Thetar_EXT*EXT_1.Thetar_EXT);
//		EXT_1.Cos_Thetar_EXT = COS(EXT_1.Thetar_EXT*EXT_1.Thetar_EXT);
//
//
//		EXT_1.Vdss_ref_EXT = (EXT_1.Vdss_ref_old + Vdss_ref_set) * 0.5f;
//		EXT_1.Vqss_ref_EXT = (EXT_1.Vqss_ref_old + Vqss_ref_set) * 0.5f;
//
//		EXT_1.Vdss_ref_old = Vdss_ref_set;
//		EXT_1.Vqss_ref_old = Vqss_ref_set;
//
//		EXT_1.Vdse_ref_EXT = EXT_1.Vdss_ref_EXT * EXT_1.Cos_Thetar_EXT + EXT_1.Vqss_ref_EXT * EXT_1.Sin_Thetar_EXT;
//		EXT_1.Vqse_ref_EXT = EXT_1.Vdss_ref_EXT * (-EXT_1.Sin_Thetar_EXT) + EXT_1.Vqss_ref_EXT * EXT_1.Cos_Thetar_EXT;
//
//		EXT_1.Idse_EXT = INV->Idss * EXT_1.Cos_Thetar_EXT + INV->Iqss * EXT_1.Sin_Thetar_EXT;
//		EXT_1.Iqse_EXT = INV->Idss * (-EXT_1.Sin_Thetar_EXT) + INV->Iqss * EXT_1.Cos_Thetar_EXT;
//
//		EXT_1.Err_Idse_EXT = EXT_1.Idse_EXT - EXT_1.Idse_EXT_est;
//		EXT_1.Err_Iqse_EXT = EXT_1.Iqse_EXT - EXT_1.Iqse_EXT_est;
//
//		EXT_1.integ_Idse_EXT_est += Tsamp * EXT_1.Kid_EXT * EXT_1.Err_Idse_EXT;
//		EXT_1.integ_Iqse_EXT_est += Tsamp * EXT_1.Kiq_EXT * EXT_1.Err_Iqse_EXT;
//
//		EXT_1.EEMFd_est = -(EXT_1.Err_Idse_EXT * EXT_1.Kpd_EXT + EXT_1.integ_Idse_EXT_est);
//		EXT_1.EEMFq_est = -(EXT_1.Err_Iqse_EXT * EXT_1.Kpq_EXT + EXT_1.integ_Iqse_EXT_est);
//
//		EXT_1.Vdse_FF_EXT = (EXT_1.Vdse_ref_EXT)+EXT_1.Wr_EXT * EXT_1.Lq_hat * EXT_1.Iqse_EXT;
//		EXT_1.Vqse_FF_EXT = (EXT_1.Vqse_ref_EXT)-EXT_1.Wr_EXT * EXT_1.Lq_hat * EXT_1.Idse_EXT;
//		//Vdse_FF_EXT = (Vdse_ref_EXT)+Wr_EXT_f * Lq_hat * Iqse_EXT;
//		//Vqse_FF_EXT = (Vqse_ref_EXT)-Wr_EXT_f * Lq_hat * Idse_EXT;
//
//		EXT_1.Idse_EXT_est += EXT_1.INV_Ld_hat * Tsamp * (EXT_1.Vdse_FF_EXT - EXT_1.EEMFd_est - EXT_1.Rs_hat * EXT_1.Idse_EXT);
//		EXT_1.Iqse_EXT_est += EXT_1.INV_Ld_hat * Tsamp * (EXT_1.Vqse_FF_EXT - EXT_1.EEMFq_est - EXT_1.Rs_hat * EXT_1.Iqse_EXT);
//
//		if (fabs(EXT_1.EEMFq_est) < 0.05f) {
//			EXT_1.EEMFq_est = 0.05f;
//		}
//
//		EXT_1.Err_Thetar_EXT = atan2f(-EXT_1.EEMFd_est, EXT_1.EEMFq_est);
//		EXT_1.EEMF_mag = sqrt(EXT_1.EEMFd_est * EXT_1.EEMFd_est + EXT_1.EEMFq_est * EXT_1.EEMFq_est);
//		EXT_1.Lamf_est = EXT_1.EEMFq_est / INV->Wr - (INV->Ld-INV->Lq) * INV->Idsr;
//
////		PLL_Adaptive(&PLL_EXT, BOUND_PI(EXT_1.Err_Thetar_EXT - PLL_EXT.Thetar_est));
//
//		////////////////////////////////PI type
//		EXT_1.Thetar_EXT_old = EXT_1.Thetar_EXT;
//
////		EXT_1.Thetar_EXT_PLL_input = BOUND_PI(EXT_1.Err_Thetar_EXT - EXT_1.Thetar_EXT);
////		INV->integ_Thetar_PLL += Tsamp * INV->Ki_PLL * EXT_1.Thetar_EXT_PLL_input;
////		EXT_1.Wr_EXT = EXT_1.k_debug * (INV->integ_Thetar_PLL + EXT_1.Thetar_EXT_PLL_input * INV->Kp_PLL);
//
//		INV->integ_Thetar_PLL += Tsamp * INV->Ki_PLL * EXT_1.Err_Thetar_EXT;
//		EXT_1.Wr_EXT = EXT_1.k_debug * (INV->integ_Thetar_PLL + EXT_1.Err_Thetar_EXT * INV->Kp_PLL);
//
//		EXT_1.Thetar_EXT += EXT_1.Wr_EXT * Tsamp ;
//		EXT_1.Thetar_EXT = BOUND_PI(EXT_1.Thetar_EXT);
//
//		EXT_1.Thetarm_EXT += (EXT_1.Wr_EXT * INV->INV_PP) * Tsamp;
//		EXT_1.Thetarm_EXT = BOUND_PI(EXT_1.Thetarm_EXT );
//
//		EXT_1.W_LPF = PI2 * W_LPF_set;
//		EXT_1.a_LPF = EXT_1.W_LPF / (EXT_1.W_LPF + Fsamp);
//		EXT_1.Wr_EXT_f = EXT_1.a_LPF * EXT_1.Wr_EXT + (1.f-EXT_1.a_LPF) * EXT_1.Wr_EXT_f;
//		EXT_1.Wrm_EXT_f = EXT_1.Wr_EXT_f * INV->INV_PP;
//		EXT_1.Wrpm_EXT_f = EXT_1.Wrm_EXT_f * RM2RPM;
//	}

//	if (Theta_mode){
//		INV->Thetar = EXT_1.Thetar_EXT_old;
//		INV->Wr = EXT_1.Wr_EXT_f;
//	}
//
//	else {
//		INV->Thetar = INV->Thetar_OLC;
//		INV->Wr = INV->Wrpm_ref_OLC *RPM2RM * INV->PP;
//	}

//	INV->Thetar_err = EXT_1.Thetar_EXT_old - INV->Thetar_OLC;

    INV->Thetar_adv = BOUND_PI(INV->Thetar + 1.5*INV->Wr*Tsamp);

    INV->cosThetar = COS(INV->Thetar * INV->Thetar);
    INV->sinThetar = SIN(INV->Thetar, INV->Thetar * INV->Thetar);

    INV->cosThetar_adv = COS(INV->Thetar_adv * INV->Thetar_adv);
    INV->sinThetar_adv = SIN(INV->Thetar_adv, INV->Thetar_adv * INV->Thetar_adv);

    INV->Vdsr_ref = LIMIT(INV->Vdsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
    INV->Vqsr_ref = LIMIT(INV->Vqsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
//    arm_sqrt_f32(INV->Vdsr_ref*INV->Vdsr_ref + INV->Vqsr_ref*INV->Vqsr_ref,  &INV->Vmag_ref );
    INV->Vdsr_ref_aw = INV->Vdsr_ref_unsat - INV->Vdsr_ref;
    INV->Vqsr_ref_aw = INV->Vqsr_ref_unsat - INV->Vqsr_ref;
    INV->Vdss_ref = INV->Vdsr_ref*INV->cosThetar_adv - INV->Vqsr_ref*INV->sinThetar_adv;
    INV->Vqss_ref = INV->Vdsr_ref*INV->sinThetar_adv + INV->Vqsr_ref*INV->cosThetar_adv;

    INV->Vas_ref = INV->Vdss_ref;
    INV->Vbs_ref = -0.5f*INV->Vdss_ref + SQRT3HALF*INV->Vqss_ref;
    INV->Vcs_ref = -0.5f*INV->Vdss_ref - SQRT3HALF*INV->Vqss_ref;

    INV->Vmax = INV->Vas_ref;
    if (INV->Vmax < INV->Vbs_ref) INV->Vmax = INV->Vbs_ref;
    if (INV->Vmax < INV->Vcs_ref) INV->Vmax = INV->Vcs_ref;

    INV->Vmin = INV->Vas_ref;
    if (INV->Vmin > INV->Vbs_ref) INV->Vmin = INV->Vbs_ref;
    if (INV->Vmin > INV->Vcs_ref) INV->Vmin = INV->Vcs_ref;

    INV->Voffset = -0.5f * (INV->Vmax + INV->Vmin);

    INV->Van_ref = INV->Vas_ref + INV->Voffset;
    INV->Vbn_ref = INV->Vbs_ref + INV->Voffset;
    INV->Vcn_ref = INV->Vcs_ref + INV->Voffset;

    INV->Van_ref = LIMIT(INV->Vas_ref + INV->Voffset, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
    INV->Vbn_ref = LIMIT(INV->Vbs_ref + INV->Voffset, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
    INV->Vcn_ref = LIMIT(INV->Vcs_ref + INV->Voffset, -0.5f*INV->Vdc, 0.5f*INV->Vdc);

    Van = INV->Van_ref;
    Vbn = INV->Vbn_ref;
    Vcn = INV->Vcn_ref;

    INV->Duty_A = (INV->Van_ref*INV->INV_Vdc + 0.5f);
    INV->Duty_B = (INV->Vbn_ref*INV->INV_Vdc + 0.5f);
    INV->Duty_C = (INV->Vcn_ref*INV->INV_Vdc + 0.5f);

    INV->Duty_A = LIMIT(INV->Duty_A, 0.f, 1.f);
    INV->Duty_B = LIMIT(INV->Duty_B, 0.f, 1.f);
    INV->Duty_C = LIMIT(INV->Duty_C, 0.f, 1.f);

	PwmSwOn();
	PwmDutyUpt();

	if (!FLAG.FAULT) PWM_BUF_ON;

}
void VoltageOpenLoopControl(struct INVERTER *INV) {

    INV->Vdsr_ref_unsat = INV->Vdsr_ref_OLC;
    INV->Vqsr_ref_unsat = INV->Vqsr_ref_OLC;

    INV->Thetar_OLC = BOUND_PI(INV->Thetar_OLC + INV->Wrpm_ref_OLC*RPM2RM*INV->PP*Tsamp);

    INV->Thetar = INV->Thetar_OLC;

    INV->Idss = (2.f * INV->Ia - INV->Ib - INV->Ic) * INV_3;
    INV->Iqss = (INV->Ib - INV->Ic) * INV_SQRT3;

    Vdss_ref_set = (2.f * Van - Vbn - Vcn) * INV_3;
    Vqss_ref_set = INV_SQRT3 * (Vbn - Vcn);

    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
    //■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■■
//    if (INV->init_align_done) {
//    	EXT_1.Sin_Thetar_EXT = SIN(EXT_1.Thetar_EXT, EXT_1.Thetar_EXT*EXT_1.Thetar_EXT);
//    	EXT_1.Cos_Thetar_EXT = COS(EXT_1.Thetar_EXT*EXT_1.Thetar_EXT);
//
//
//    	EXT_1.Vdss_ref_EXT = (EXT_1.Vdss_ref_old + Vdss_ref_set) * 0.5f;
//    	EXT_1.Vqss_ref_EXT = (EXT_1.Vqss_ref_old + Vqss_ref_set) * 0.5f;
//
//    	EXT_1.Vdss_ref_old = Vdss_ref_set;
//    	EXT_1.Vqss_ref_old = Vqss_ref_set;
//
//    	EXT_1.Vdse_ref_EXT = EXT_1.Vdss_ref_EXT * EXT_1.Cos_Thetar_EXT + EXT_1.Vqss_ref_EXT * EXT_1.Sin_Thetar_EXT;
//    	EXT_1.Vqse_ref_EXT = EXT_1.Vdss_ref_EXT * (-EXT_1.Sin_Thetar_EXT) + EXT_1.Vqss_ref_EXT * EXT_1.Cos_Thetar_EXT;
//
//    	EXT_1.Idse_EXT = INV->Idss * EXT_1.Cos_Thetar_EXT + INV->Iqss * EXT_1.Sin_Thetar_EXT;
//    	EXT_1.Iqse_EXT = INV->Idss * (-EXT_1.Sin_Thetar_EXT) + INV->Iqss * EXT_1.Cos_Thetar_EXT;
//
//    	EXT_1.Err_Idse_EXT = EXT_1.Idse_EXT - EXT_1.Idse_EXT_est;
//    	EXT_1.Err_Iqse_EXT = EXT_1.Iqse_EXT - EXT_1.Iqse_EXT_est;
//
//    	EXT_1.integ_Idse_EXT_est += Tsamp * EXT_1.Kid_EXT * EXT_1.Err_Idse_EXT;
//    	EXT_1.integ_Iqse_EXT_est += Tsamp * EXT_1.Kiq_EXT * EXT_1.Err_Iqse_EXT;
//
//    	EXT_1.EEMFd_est = -(EXT_1.Err_Idse_EXT * EXT_1.Kpd_EXT + EXT_1.integ_Idse_EXT_est);
//    	EXT_1.EEMFq_est = -(EXT_1.Err_Iqse_EXT * EXT_1.Kpq_EXT + EXT_1.integ_Iqse_EXT_est);
//
//    	EXT_1.Vdse_FF_EXT = (EXT_1.Vdse_ref_EXT)+EXT_1.Wr_EXT * EXT_1.Lq_hat * EXT_1.Iqse_EXT;
//    	EXT_1.Vqse_FF_EXT = (EXT_1.Vqse_ref_EXT)-EXT_1.Wr_EXT * EXT_1.Lq_hat * EXT_1.Idse_EXT;
//    	//Vdse_FF_EXT = (Vdse_ref_EXT)+Wr_EXT_f * Lq_hat * Iqse_EXT;
//    	//Vqse_FF_EXT = (Vqse_ref_EXT)-Wr_EXT_f * Lq_hat * Idse_EXT;
//
//    	EXT_1.Idse_EXT_est += EXT_1.INV_Ld_hat * Tsamp * (EXT_1.Vdse_FF_EXT - EXT_1.EEMFd_est - EXT_1.Rs_hat * EXT_1.Idse_EXT);
//    	EXT_1.Iqse_EXT_est += EXT_1.INV_Ld_hat * Tsamp * (EXT_1.Vqse_FF_EXT - EXT_1.EEMFq_est - EXT_1.Rs_hat * EXT_1.Iqse_EXT);
//
//    	if (fabs(EXT_1.EEMFq_est) < 0.05f) {
//    		EXT_1.EEMFq_est = 0.05f;
//    	}
//
//    	EXT_1.Err_Thetar_EXT = atan2f(-EXT_1.EEMFd_est, EXT_1.EEMFq_est);
//    	EXT_1.Lamf_est = EXT_1.EEMFq_est / INV->Wr - (INV->Ld-INV->Lq) * INV->Idsr;


//    	INV->W_PLL = Ws;
//    	INV->Kp_PLL = 2.f * 0.707 * INV->W_PLL;
//    	INV->Ki_PLL = INV->W_PLL * INV->W_PLL;
//    	INV->integ_Thetar_PLL = 0.;
//

    	////////////////////////////////PI type
//    	EXT_1.Thetar_EXT_old = EXT_1.Thetar_EXT;
//    	INV->integ_Thetar_PLL += Tsamp * INV->Ki_PLL * EXT_1.Err_Thetar_EXT;
//    	EXT_1.Wr_EXT = INV->integ_Thetar_PLL + EXT_1.Err_Thetar_EXT * INV->Kp_PLL;
//    	EXT_1.Thetar_EXT += EXT_1.Wr_EXT * Tsamp;
//    	EXT_1.Thetar_EXT = BOUND_PI(EXT_1.Thetar_EXT);
//
//    	EXT_1.Thetarm_EXT += (EXT_1.Wr_EXT * INV->INV_PP) * Tsamp;
//    	EXT_1.Thetarm_EXT = BOUND_PI(EXT_1.Thetarm_EXT );
//
//    	EXT_1.W_LPF = PI2 * W_LPF_set;
//    	EXT_1.a_LPF = EXT_1.W_LPF / (EXT_1.W_LPF + Fsamp);
//    	EXT_1.Wr_EXT_f = EXT_1.a_LPF * EXT_1.Wr_EXT + (1.f-EXT_1.a_LPF) * EXT_1.Wr_EXT_f;
//    	EXT_1.Wrm_EXT_f = EXT_1.Wr_EXT_f * INV->INV_PP;
//    	EXT_1.Wrpm_EXT_f = EXT_1.Wrm_EXT_f * RM2RPM;
//    }

    if (Theta_mode){
    	INV->Thetar = EXT_1.Thetar_EXT_old;
    	INV->Wr = EXT_1.Wr_EXT_f;
    }

    else {
    	INV->Thetar = INV->Thetar_OLC;
    	INV->Wr = INV->Wrpm_ref_OLC *RPM2RM * INV->PP;
    }

//    INV->Thetar_err = BOUND_PI(EXT_1.Thetar_EXT_old - INV->Thetar_OLC);


//    INV->Thetar_OLC = BOUND_PI(INV->Thetar_OLC + INV->Wrpm_ref_OLC*RPM2RM*INV->PP*Tsamp);
//    INV->Thetar = INV->Thetar_OLC;

#if ANGLE_COMPENSATION == ON
    INV->Thetar_adv = BOUND_PI(INV->Thetar + 1.5*INV->Wr*Tsamp);
#else
    INV->Thetar_adv = INV->Thetar;
#endif

    INV->Thetar_adv = BOUND_PI(INV->Thetar + 1.5*INV->Wr*Tsamp);

    INV->cosThetar = COS(INV->Thetar * INV->Thetar);
    INV->sinThetar = SIN(INV->Thetar, INV->Thetar * INV->Thetar);

    INV->cosThetar_adv = COS(INV->Thetar_adv * INV->Thetar_adv);
    INV->sinThetar_adv = SIN(INV->Thetar_adv, INV->Thetar_adv * INV->Thetar_adv);

    INV->Vdsr_ref = LIMIT(INV->Vdsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
    INV->Vqsr_ref = LIMIT(INV->Vqsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
//    arm_sqrt_f32(INV->Vdsr_ref*INV->Vdsr_ref + INV->Vqsr_ref*INV->Vqsr_ref,  &INV->Vmag_ref );
    INV->Vdsr_ref_aw = INV->Vdsr_ref_unsat - INV->Vdsr_ref;
    INV->Vqsr_ref_aw = INV->Vqsr_ref_unsat - INV->Vqsr_ref;
    INV->Vdss_ref = INV->Vdsr_ref*INV->cosThetar_adv - INV->Vqsr_ref*INV->sinThetar_adv;
    INV->Vqss_ref = INV->Vdsr_ref*INV->sinThetar_adv + INV->Vqsr_ref*INV->cosThetar_adv;

    INV->Vas_ref = INV->Vdss_ref;
    INV->Vbs_ref = -0.5f*INV->Vdss_ref + SQRT3HALF*INV->Vqss_ref;
    INV->Vcs_ref = -0.5f*INV->Vdss_ref - SQRT3HALF*INV->Vqss_ref;

    INV->Vmax = INV->Vas_ref;
    if (INV->Vmax < INV->Vbs_ref) INV->Vmax = INV->Vbs_ref;
    if (INV->Vmax < INV->Vcs_ref) INV->Vmax = INV->Vcs_ref;

    INV->Vmin = INV->Vas_ref;
    if (INV->Vmin > INV->Vbs_ref) INV->Vmin = INV->Vbs_ref;
    if (INV->Vmin > INV->Vcs_ref) INV->Vmin = INV->Vcs_ref;

    INV->Voffset = -0.5f * (INV->Vmax + INV->Vmin);

    INV->Van_ref = INV->Vas_ref + INV->Voffset;
    INV->Vbn_ref = INV->Vbs_ref + INV->Voffset;
    INV->Vcn_ref = INV->Vcs_ref + INV->Voffset;

    INV->Van_ref = LIMIT(INV->Vas_ref + INV->Voffset, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
    INV->Vbn_ref = LIMIT(INV->Vbs_ref + INV->Voffset, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
    INV->Vcn_ref = LIMIT(INV->Vcs_ref + INV->Voffset, -0.5f*INV->Vdc, 0.5f*INV->Vdc);

    Van = INV->Van_ref;
    Vbn = INV->Vbn_ref;
    Vcn = INV->Vcn_ref;

    INV->Duty_A = (INV->Van_ref*INV->INV_Vdc + 0.5f);
    INV->Duty_B = (INV->Vbn_ref*INV->INV_Vdc + 0.5f);
    INV->Duty_C = (INV->Vcn_ref*INV->INV_Vdc + 0.5f);

    INV->Duty_A = LIMIT(INV->Duty_A, 0.f, 1.f);
    INV->Duty_B = LIMIT(INV->Duty_B, 0.f, 1.f);
    INV->Duty_C = LIMIT(INV->Duty_C, 0.f, 1.f);

	PwmSwOn();
	PwmDutyUpt();

	if (!FLAG.FAULT) PWM_BUF_ON;

}

void VoltageInjection_SquareWave(struct INVERTER *INV) {

	INV->Vmag_inj = 1;

	INV->Thetar = INV->Thetar_OLC;
	INV->Thetar_adv = INV->Thetar_OLC;

	INV->cosThetar = 1;
	INV->sinThetar = 0;
	INV->cosThetar_adv = 1;
	INV->sinThetar_adv = 0;

	if (cnt_inj >= 2) {
		// 다음 스위칭 주기에 +d 주입
		INV->Vdsr_ref_unsat = INV->Vmag_inj;
		INV->Vqsr_ref_unsat = 0;
		cnt_inj = 0;
	}
	else {
		// 다음 스위칭 주기에 -d 주입
		INV->Vdsr_ref_unsat = -INV->Vmag_inj;
		INV->Vqsr_ref_unsat = 0;
		cnt_inj = 1;
	}

	cnt_inj++;

	INV->Vdsr_ref = LIMIT(INV->Vdsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
	INV->Vqsr_ref = LIMIT(INV->Vqsr_ref_unsat, -INV->Vdc*INV_SQRT3, INV->Vdc*INV_SQRT3);
	INV->Vdsr_ref_aw = INV->Vdsr_ref_unsat - INV->Vdsr_ref;
	INV->Vqsr_ref_aw = INV->Vqsr_ref_unsat - INV->Vqsr_ref;
	INV->Vdss_ref = INV->Vdsr_ref*INV->cosThetar_adv - INV->Vqsr_ref*INV->sinThetar_adv;
	INV->Vqss_ref = INV->Vdsr_ref*INV->sinThetar_adv + INV->Vqsr_ref*INV->cosThetar_adv;

	INV->Vas_ref = INV->Vdss_ref;
	INV->Vbs_ref = -0.5f*INV->Vdss_ref + SQRT3HALF*INV->Vqss_ref;
	INV->Vcs_ref = -0.5f*INV->Vdss_ref - SQRT3HALF*INV->Vqss_ref;

	INV->Vmax = INV->Vas_ref;
	if (INV->Vmax < INV->Vbs_ref) INV->Vmax = INV->Vbs_ref;
	if (INV->Vmax < INV->Vcs_ref) INV->Vmax = INV->Vcs_ref;

	INV->Vmin = INV->Vas_ref;
	if (INV->Vmin > INV->Vbs_ref) INV->Vmin = INV->Vbs_ref;
	if (INV->Vmin > INV->Vcs_ref) INV->Vmin = INV->Vcs_ref;

	INV->Voffset = -0.5f * (INV->Vmax + INV->Vmin);

	INV->Idss_ref = INV->Idsr_ref*INV->cosThetar_adv - INV->Iqsr_ref*INV->sinThetar_adv;
	INV->Iqss_ref = INV->Idsr_ref*INV->sinThetar_adv + INV->Iqsr_ref*INV->cosThetar_adv;

	INV->Ia_ref = INV->Idss_ref;
	INV->Ib_ref = -0.5f*INV->Idss_ref + SQRT3HALF*INV->Iqss_ref;
	INV->Ic_ref = -0.5f*INV->Idss_ref - SQRT3HALF*INV->Iqss_ref;

	INV->Va_NLC = INV->A_NLC*atanf(INV->B_NLC*INV->Ia_ref);
	INV->Vb_NLC = INV->A_NLC*atanf(INV->B_NLC*INV->Ib_ref);
	INV->Vc_NLC = INV->A_NLC*atanf(INV->B_NLC*INV->Ic_ref);

	INV->Van_ref = LIMIT(INV->Vas_ref + INV->Voffset + INV->Va_NLC, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
	INV->Vbn_ref = LIMIT(INV->Vbs_ref + INV->Voffset + INV->Vb_NLC, -0.5f*INV->Vdc, 0.5f*INV->Vdc);
	INV->Vcn_ref = LIMIT(INV->Vcs_ref + INV->Voffset + INV->Vc_NLC, -0.5f*INV->Vdc, 0.5f*INV->Vdc);

	INV->Duty_A = (INV->Van_ref*INV->INV_Vdc + 0.5f);
	INV->Duty_B = (INV->Vbn_ref*INV->INV_Vdc + 0.5f);
	INV->Duty_C = (INV->Vcn_ref*INV->INV_Vdc + 0.5f);

	INV->Duty_A = LIMIT(INV->Duty_A, 0.f, 1.f);
	INV->Duty_B = LIMIT(INV->Duty_B, 0.f, 1.f);
	INV->Duty_C = LIMIT(INV->Duty_C, 0.f, 1.f);

	PwmDutyUpt();
	PwmSwOn();

	if (!FLAG.FAULT) PWM_BUF_ON;

}

void Hallsensor_Observer(struct INVERTER *INV){ // speed calculation

//	INV->hallSensorValue = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

	INV->Thetar_err_Hall = BOUND_PI(INV->Thetar_Hall_PLL - INV->Thetar_est_Hall);
//	INV->Thetar_Hall_PLL = BOUND_PI(INV->Thetar_Hall_PLL);
//	INV->Thetar_err_Hall = INV->Thetar_Hall_PLL - INV->Thetar_est_Hall;
	INV->integ_PLL_Hall += Tsamp * INV->Ki_PLL_Hall * INV->Thetar_err_Hall;
	INV->Wr_est_Hall = INV->Kp_PLL_Hall * INV->Thetar_err_Hall + INV->integ_PLL_Hall;
	INV->Thetar_est_Hall += Tsamp * INV->Wr_est_Hall;
	INV->Thetar_est_Hall = BOUND_PI(INV->Thetar_est_Hall);

//	if (fabs(INV->Wr_est_Hall) < 20){
//		INV->Wr_est_Hall = 0;
//	}

	INV->Wr = INV->Wr_est_Hall;
	INV->Wrm = INV->Wr * INV->INV_PP;
	INV->Wrpm = INV->Wrm*Rm2Rpm;

	INV->hall_a = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	INV->hall_b = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
	INV->hall_c = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

	INV->hall_state = GetHallSensorState(INV->hall_a,INV->hall_b,INV->hall_c);

	switch(INV->hall_state){
		case 6:
			INV->Thetar_Hall_PLL = 0.;
			break;
		case 4:
			INV->Thetar_Hall_PLL = PIOF3;
			break;
		case 5:
			INV->Thetar_Hall_PLL = 2*PIOF3;
			break;
		case 1:
			INV->Thetar_Hall_PLL = 3*PIOF3;
			break;
		case 3:
			INV->Thetar_Hall_PLL = -2*PIOF3;
			break;
		case 2:
			INV->Thetar_Hall_PLL = -PIOF3;
			break;
	}

//	if (INV->hall_state != INV->hall_state_old){
//		switch(INV->hall_state_old){ //CCW(hall: 3,2,6,4,5,1)
//		case 1:
//			switch (INV->hall_state){
//			case 3:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL + PIOF3;
//
//				break;
//			case 5:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL - PIOF3;
//				break;
//			}
//			break;
//		case 2:
//			switch (INV->hall_state){
//			case 6:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL + PIOF3;
//				break;
//			case 3:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL - PIOF3;
//				break;
//			}
//			break;
//		case 3:
//			switch (INV->hall_state){
//			case 2:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL + PIOF3;
//				break;
//			case 1:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL - PIOF3;
//				break;
//			}
//			break;
//		case 4:
//			switch (INV->hall_state){
//			case 5:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL + PIOF3;
//				break;
//			case 6:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL - PIOF3;
//				break;
//			}
//			break;
//		case 5:
//			switch (INV->hall_state){
//			case 1:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL + PIOF3;
//				break;
//			case 4:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL - PIOF3;
//				break;
//			}
//			break;
//		case 6:
//			switch (INV->hall_state){
//			case 4:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL + PIOF3;
//				break;
//			case 2:
//				INV->Thetar_Hall_PLL = INV->Thetar_Hall_PLL - PIOF3;
//				break;
//			}
//			break;
//		}
//	}
//	INV->hall_state_old = INV->hall_state;
}

uint8_t GetHallSensorState(GPIO_PinState hall_sensor_1, GPIO_PinState hall_sensor_2, GPIO_PinState hall_sensor_3)
{
    uint8_t hall_state = 0;

    // Shift and combine the states to form a 3-bit value
    hall_state |= (hall_sensor_1 == GPIO_PIN_SET) ? 0x01 : 0x00;
    hall_state |= (hall_sensor_2 == GPIO_PIN_SET) ? 0x02 : 0x00;
    hall_state |= (hall_sensor_3 == GPIO_PIN_SET) ? 0x04 : 0x00;

    return hall_state;
}

void HallPosition_Test(struct INVERTER *INV){
	//INV->Duty_Test = 0.01;
	switch (INV->duty_state){
	case 1: // vector [1 0 0]
		INV->Duty_A = INV->Duty_Test;
		INV->Duty_B = 0;
		INV->Duty_C = 0;
		break;
	case 2: // vector [1 1 0]
		INV->Duty_A = INV->Duty_Test;
		INV->Duty_B = INV->Duty_Test;
		INV->Duty_C = 0;
		break;
	case 3: // vector [0 1 0]
		INV->Duty_A = 0;
		INV->Duty_B = INV->Duty_Test;
		INV->Duty_C = 0;
		break;
	case 4: // vector [0 1 1]
		INV->Duty_A = 0;
		INV->Duty_B = INV->Duty_Test;
		INV->Duty_C = INV->Duty_Test;
		break;
	case 5: // vector [0 0 1]
		INV->Duty_A = 0;
		INV->Duty_B = 0;
		INV->Duty_C = INV->Duty_Test;
		break;
	case 6: // vector [1 0 1]
		INV->Duty_A = INV->Duty_Test;
		INV->Duty_B = 0;
		INV->Duty_C = INV->Duty_Test;
		break;
	default: // vector [0 0 0]
		INV->Duty_A = 0;
		INV->Duty_B = 0;
		INV->Duty_C = 0;
		break;
	}
	PwmDutyUpt();
	PwmSwOn();
}

void Align(struct INVERTER *INV) {

    INV->Thetar = 0.;

    switch (Mode_align) {
    case 0:
        Align_done = 0;
        INV->Thetar_offset = 0.;
        INV->Idsr_ref = INV->Idsr_align;
        Time_align = 0.;
        Mode_align++;
        break;
    case 1:
        CurrentControl(INV);
		PwmDutyUpt();
		PwmSwOn();
        if (!FLAG.FAULT) {PWM_BUF_ON;}

        if (Time_align >= 1.) {

//            INV->Thetar_offset = 0.999*INV->Thetar_offset + 0.001*BOUND_PI(htim2.Instance->CNT*PI2*INV_ENC_CNT_MAX*INV->PP);	// IIR 필터 (3초 동안 계산)
//            INV->Thetar_offset = BOUND_PI(htim2.Instance->CNT*PI2*INV_ENC_CNT_MAX*INV->PP);
        }
        if (Time_align >= 4.) Mode_align++;
        break;
    case 2:		// 0 전류 제어를 하고 끄기
        INV->Idsr_ref = 0.;

        CurrentControl(INV);
		PwmDutyUpt();
		PwmSwOn();
        if (!FLAG.FAULT) {PWM_BUF_ON;}

        if (Time_align >= 5.) Mode_align++;
        break;
    case 3:
        Mode_align = 0;
        Align_done = 1;
        FLAG.INV_ALIGN = 0;
        break;
    default:
        break;
    }
    Time_align += Tsamp;

}

// 2. PwmSwOn() 함수 교체
void PwmSwOn(void) {
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
        HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);

    HAL_HRTIM_WaveformCounterStart(&hhrtim1,
        HRTIM_TIMERID_TIMER_A |
        HRTIM_TIMERID_TIMER_B |
        HRTIM_TIMERID_TIMER_C);
}

// 3. PwmSwOff() 함수 교체
void PwmSwOff(void) {
		PWM_BUF_OFF;
		INV.Duty_A = 0.f;
		INV.Duty_B = 0.f;
		INV.Duty_C = 0.f;

    HAL_HRTIM_WaveformOutputStop(&hhrtim1,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
        HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);

    HAL_HRTIM_WaveformCounterStop(&hhrtim1,
        HRTIM_TIMERID_TIMER_A |
        HRTIM_TIMERID_TIMER_B |
        HRTIM_TIMERID_TIMER_C);
}

void PwmDutyUpt(void) {
		uint32_t period = hhrtim1.Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR;

	    uint32_t duty_a = (uint32_t)(INV.Duty_A * (float)period);
	    uint32_t duty_b = (uint32_t)(INV.Duty_B * (float)period);
	    uint32_t duty_c = (uint32_t)(INV.Duty_C * (float)period);

	    if(duty_a > period) duty_a = period;
	    if(duty_b > period) duty_b = period;
	    if(duty_c > period) duty_c = period;

    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                          HRTIM_COMPAREUNIT_1, duty_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                          HRTIM_COMPAREUNIT_1, duty_b);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C,
                          HRTIM_COMPAREUNIT_1, duty_c);
}

