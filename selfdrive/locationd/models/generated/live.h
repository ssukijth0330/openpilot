#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6682464975442471769);
void live_err_fun(double *nom_x, double *delta_x, double *out_8729540985056067358);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4100406716172023109);
void live_H_mod_fun(double *state, double *out_4365735452148030976);
void live_f_fun(double *state, double dt, double *out_7233734678308439105);
void live_F_fun(double *state, double dt, double *out_7186407748472172171);
void live_h_4(double *state, double *unused, double *out_747960034792833697);
void live_H_4(double *state, double *unused, double *out_2836961591093737002);
void live_h_9(double *state, double *unused, double *out_5143917515407147576);
void live_H_9(double *state, double *unused, double *out_2595771944464146357);
void live_h_10(double *state, double *unused, double *out_4990416931212113379);
void live_H_10(double *state, double *unused, double *out_1563174650098063786);
void live_h_12(double *state, double *unused, double *out_8819626307844828718);
void live_H_12(double *state, double *unused, double *out_2182494816938224793);
void live_h_35(double *state, double *unused, double *out_2516966990548027135);
void live_H_35(double *state, double *unused, double *out_4928057849263238502);
void live_h_32(double *state, double *unused, double *out_5871713862586783818);
void live_H_32(double *state, double *unused, double *out_8161168332066886053);
void live_h_13(double *state, double *unused, double *out_6162076028028181008);
void live_H_13(double *state, double *unused, double *out_2827166593172103927);
void live_h_14(double *state, double *unused, double *out_5143917515407147576);
void live_H_14(double *state, double *unused, double *out_2595771944464146357);
void live_h_33(double *state, double *unused, double *out_1322511510347290919);
void live_H_33(double *state, double *unused, double *out_8078614853902096106);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}