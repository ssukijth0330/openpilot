#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6945638239760000243);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3968798334777133982);
void gnss_H_mod_fun(double *state, double *out_246944569381823836);
void gnss_f_fun(double *state, double dt, double *out_2648529891379557822);
void gnss_F_fun(double *state, double dt, double *out_2252592465799141843);
void gnss_h_6(double *state, double *sat_pos, double *out_4163819229610976683);
void gnss_H_6(double *state, double *sat_pos, double *out_2287387615128234836);
void gnss_h_20(double *state, double *sat_pos, double *out_4847015344803285583);
void gnss_H_20(double *state, double *sat_pos, double *out_2751431159818563852);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_9041024282100407462);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8047877791999041801);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_9041024282100407462);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8047877791999041801);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}