/*
 * speed_observer.c
 *
 *  Created on: Feb 12, 2025
 *      Author: HALAB_G
 */



#include "speed_observer.h"
#include "math.h"
#include "variable.h"

struct EXT_Sensorless EXT_1;

void InitSpeedObserver_(struct SPEED_OBSERVER* SPDOBS, float Beta, float PP, float Ld, float Lq, float Lamf, float Jm, float Bm) {

    SPDOBS->Thetarm_est = 0.f;
    SPDOBS->Thetar_est = 0.f;
    SPDOBS->Wrm_est = 0.f;
    SPDOBS->Wr_est = 0.f;
    SPDOBS->Wrm_est_fb = 0.f;

    SPDOBS->PP = PP;
    SPDOBS->INV_PP = 1.f/PP;

    SPDOBS->Ld_hat = Ld;
    SPDOBS->Lq_hat = Lq;
    SPDOBS->Lamf_hat = Lamf;

    SPDOBS->Jm_hat = Jm;
    SPDOBS->Bm_hat = Bm;
    SPDOBS->INV_Jm_hat = 1.f/Jm;

    //SPDOBS->Tsamp = 1.f/5e3;

    //Triple Pole
	//SPDOBS->Wso = -Beta;
	//SPDOBS->l1 = -3.f * SPDOBS->Wso - SPDOBS->Bm_hat * SPDOBS->INV_Jm_hat;
	//SPDOBS->l2 = 3.f * SPDOBS->Wso * SPDOBS->Wso + 3. * SPDOBS->Wso * SPDOBS->Bm_hat * SPDOBS->INV_Jm_hat + SPDOBS->Bm_hat * SPDOBS->Bm_hat * SPDOBS->INV_Jm_hat * SPDOBS->INV_Jm_hat;
	//SPDOBS->l3 = SPDOBS->Wso * SPDOBS->Wso * SPDOBS->Wso * SPDOBS->Jm_hat;

	//Butterworth
	SPDOBS->Wso = -Beta;
	SPDOBS->l1 = - 2.f * SPDOBS->Wso - SPDOBS->Bm_hat * SPDOBS->INV_Jm_hat;
	SPDOBS->l2 = 2.f * SPDOBS->Wso * SPDOBS->Wso - SPDOBS->l1* SPDOBS->Bm_hat* SPDOBS->INV_Jm_hat;
	SPDOBS->l3 = SPDOBS->Wso * SPDOBS->Wso * SPDOBS->Wso * SPDOBS->Jm_hat;


    SPDOBS->K1 = SPDOBS->l1;
    SPDOBS->K2 = SPDOBS->Jm_hat * SPDOBS->l2;
    SPDOBS->K3 = -SPDOBS->l3;

    SPDOBS->Thetarm_err = 0.f;
    SPDOBS->Wrm_err = 0.f;
    SPDOBS->Te_est = 0.f;
    SPDOBS->Tl_est = 0.f;
    SPDOBS->Te_ff = 0.f;
    SPDOBS->integ_double = 0.f;

    SPDOBS->integ_Wrm_est = 0.f;

    SPDOBS->Tload_est = 0.f;

}

void SpeedObserver_4_34(struct SPEED_OBSERVER* SPDOBS, float Err_Thetar, float Idse_ff, float Iqse_ff) {

    SPDOBS->Thetarm_err = Err_Thetar * SPDOBS->INV_PP;

    SPDOBS->Te_est = SPDOBS->K2 * SPDOBS->Thetarm_err;
    SPDOBS->Tl_est += SPDOBS->K3 * SPDOBS->Thetarm_err * Tsamp;
    SPDOBS->Te_ff = 1.5f * SPDOBS->PP * (SPDOBS->Lamf_hat * Iqse_ff + (SPDOBS->Ld_hat - SPDOBS->Lq_hat) * Idse_ff * Iqse_ff);
    //SPDOBS->Te_ff = 0.;

    SPDOBS->integ_Wrm_est += (SPDOBS->Te_est + SPDOBS->Te_ff + SPDOBS->Tl_est - SPDOBS->Bm_hat * SPDOBS->Wrm_est_fb) * SPDOBS->INV_Jm_hat * Tsamp;
    SPDOBS->Wrm_est = SPDOBS->integ_Wrm_est;
    SPDOBS->Wrm_est_fb = SPDOBS->integ_Wrm_est;
    SPDOBS->Wr_est = SPDOBS->Wrm_est * SPDOBS->PP;

    SPDOBS->Thetarm_est += (SPDOBS->Wrm_est + SPDOBS->K1 * SPDOBS->Thetarm_err) * Tsamp;
    SPDOBS->Thetarm_est = BOUND_PI(SPDOBS->Thetarm_est);
    SPDOBS->Thetar_est = BOUND_PI(SPDOBS->PP * SPDOBS->Thetarm_est);

    SPDOBS->Tload_est = -SPDOBS->Tl_est;

}


void SpeedObserver_4_35(struct SPEED_OBSERVER* SPDOBS, float Err_Thetar, float Idse_ff, float Iqse_ff) {

	SPDOBS->Thetarm_err = Err_Thetar * SPDOBS->INV_PP;

	SPDOBS->Te_est = SPDOBS->K2 * SPDOBS->Thetarm_err;
	SPDOBS->Tl_est += SPDOBS->K3 * SPDOBS->Thetarm_err * Tsamp;
	SPDOBS->Te_ff = 1.5f * SPDOBS->PP * (SPDOBS->Lamf_hat * Iqse_ff + (SPDOBS->Ld_hat - SPDOBS->Lq_hat) * Idse_ff * Iqse_ff);
	//SPDOBS->Te_ff = 0.;

	SPDOBS->integ_Wrm_est += (SPDOBS->Te_est + SPDOBS->Te_ff + SPDOBS->Tl_est - SPDOBS->Bm_hat * SPDOBS->Wrm_est_fb) * SPDOBS->INV_Jm_hat * Tsamp;
	SPDOBS->Wrm_est = SPDOBS->integ_Wrm_est + SPDOBS->K1 * SPDOBS->Thetarm_err;
	SPDOBS->Wrm_est_fb = SPDOBS->integ_Wrm_est;
	SPDOBS->Wr_est = SPDOBS->Wrm_est * SPDOBS->PP;

	SPDOBS->Thetarm_est += (SPDOBS->Wrm_est) * Tsamp;
	SPDOBS->Thetarm_est = BOUND_PI(SPDOBS->Thetarm_est);
	SPDOBS->Thetar_est = BOUND_PI(SPDOBS->PP * SPDOBS->Thetarm_est);

	SPDOBS->Tload_est = -SPDOBS->Tl_est;
}


void initExtended_Sensorless_Synchronous_Frame(struct EXT_Sensorless* EXT, float Wc, float Rs, float Ld, float Lq) {
    EXT->Rs_hat = Rs;
    EXT->Ld_hat = Ld;
    EXT->Lq_hat = Lq;
    EXT->INV_Ld_hat = 1./Ld;

    EXT->k_debug = 1.f;

    EXT->Wec = 1.0f*Wc;
    EXT->Kpd_EXT = EXT->Ld_hat * EXT->Wec;
    EXT->Kid_EXT = EXT->Rs_hat * EXT->Wec;
    EXT->Kpq_EXT = EXT->Ld_hat * EXT->Wec;
    EXT->Kiq_EXT = EXT->Rs_hat * EXT->Wec;

    //EXT->Tsamp = 1.f / 5.e3;


    EXT->Vdss_ref_EXT = 0.f;
    EXT->Vqss_ref_EXT = 0.f;
    EXT->Vdss_ref_old = 0.f;
    EXT->Vqss_ref_old = 0.f;

    EXT->Err_Thetar_EXT = 0.f;
    EXT->Wr_EXT = 0.f;
    EXT->Thetar_EXT = 0.f;
    EXT->Thetar_EXT_old = 0.f;
    EXT->Sin_Thetar_EXT = 0.f;
    EXT->Cos_Thetar_EXT = 0.f;
    EXT->Wr_EXT_f = 0.f;

    EXT->Thetarm_EXT = 0.f;
    EXT->Thetarm_EXT_old = 0.f;
    EXT->Sin_Thetarm_EXT = 0.f;
    EXT->Cos_Thetarm_EXT = 0.f;

    EXT->Vdse_ref_EXT = 0.f;
    EXT->Vqse_ref_EXT = 0.f;
    EXT->Idse_EXT = 0.f;
    EXT->Iqse_EXT = 0.f;
    EXT->Err_Idse_EXT = 0.f;
    EXT->Err_Iqse_EXT = 0.f;
    EXT->Idse_EXT_est = 0.f;
    EXT->Iqse_EXT_est = 0.f;
    EXT->integ_Idse_EXT_est = 0.f;
    EXT->integ_Iqse_EXT_est = 0.f;
    EXT->EEMFd_est = 0.f;
    EXT->EEMFq_est = 0.f;
    EXT->Vdse_FF_EXT = 0.f;
    EXT->Vqse_FF_EXT = 0.f;

}

void EXT_SS_Sync(struct EXT_Sensorless* EXT, float Vdss, float Vqss, float Idss, float Iqss) {
    EXT->Kpd_EXT = EXT->Ld_hat * EXT->Wec;
    EXT->Kid_EXT = EXT->Rs_hat * EXT->Wec;
    EXT->Kpq_EXT = EXT->Ld_hat * EXT->Wec;
    EXT->Kiq_EXT = EXT->Rs_hat * EXT->Wec;

    EXT->Sin_Thetar_EXT = sinf(EXT->Thetar_EXT);
    EXT->Cos_Thetar_EXT = cosf(EXT->Thetar_EXT);

    EXT->Vdss_ref_EXT = (EXT->Vdss_ref_old + Vdss) * 0.5f;
    EXT->Vqss_ref_EXT = (EXT->Vqss_ref_old + Vqss) * 0.5f;

    //EXT->Vdss_ref_EXT = EXT->Vdss_ref_old;
    //EXT->Vqss_ref_EXT = EXT->Vqss_ref_old;

    EXT->Vdss_ref_old = Vdss;
    EXT->Vqss_ref_old = Vqss;

    EXT->Vdse_ref_EXT = EXT->Vdss_ref_EXT * EXT->Cos_Thetar_EXT + EXT->Vqss_ref_EXT * EXT->Sin_Thetar_EXT;
    EXT->Vqse_ref_EXT = EXT->Vdss_ref_EXT * (-EXT->Sin_Thetar_EXT) + EXT->Vqss_ref_EXT * EXT->Cos_Thetar_EXT;

    EXT->Idse_EXT = Idss * EXT->Cos_Thetar_EXT + Iqss * EXT->Sin_Thetar_EXT;
    EXT->Iqse_EXT = Idss * (-EXT->Sin_Thetar_EXT) + Iqss * EXT->Cos_Thetar_EXT;

    EXT->Err_Idse_EXT = EXT->Idse_EXT - EXT->Idse_EXT_est;
    EXT->Err_Iqse_EXT = EXT->Iqse_EXT - EXT->Iqse_EXT_est;

    EXT->integ_Idse_EXT_est += Tsamp * EXT->Kid_EXT * EXT->Err_Idse_EXT;
    EXT->integ_Iqse_EXT_est += Tsamp * EXT->Kiq_EXT * EXT->Err_Iqse_EXT;

    EXT->EEMFd_est = -(EXT->Err_Idse_EXT * EXT->Kpd_EXT + EXT->integ_Idse_EXT_est);
    EXT->EEMFq_est = -(EXT->Err_Iqse_EXT * EXT->Kpq_EXT + EXT->integ_Iqse_EXT_est);

    EXT->Vdse_FF_EXT = (EXT->Vdse_ref_EXT)+ EXT->Wr_EXT * EXT->Lq_hat * EXT->Iqse_EXT;
    EXT->Vqse_FF_EXT = (EXT->Vqse_ref_EXT)- EXT->Wr_EXT * EXT->Lq_hat * EXT->Idse_EXT;
    //EXT->Vdse_FF_EXT = (EXT->Vdse_ref_EXT)+EXT->Wr_EXT_f * EXT->Lq_hat * EXT->Iqse_EXT;
    //EXT->Vqse_FF_EXT = (EXT->Vqse_ref_EXT)-EXT->Wr_EXT_f * EXT->Lq_hat * EXT->Idse_EXT;

    EXT->Idse_EXT_est += EXT->INV_Ld_hat * Tsamp * (EXT->Vdse_FF_EXT - EXT->EEMFd_est - EXT->Rs_hat * EXT->Idse_EXT);
    EXT->Iqse_EXT_est += EXT->INV_Ld_hat * Tsamp * (EXT->Vqse_FF_EXT - EXT->EEMFq_est - EXT->Rs_hat * EXT->Iqse_EXT);

    if (fabs(EXT->EEMFq_est) < 1.f) {
        EXT->EEMFq_est = 1.f;
    }

    EXT->Err_Thetar_EXT = atan2f(-EXT->EEMFd_est, EXT->EEMFq_est);
    EXT->Thetar_EXT_old = EXT->Thetar_EXT;
    EXT->Thetarm_EXT_old = EXT->Thetarm_EXT;

}

