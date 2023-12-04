#pragma once
#include "rednose/helpers/ekf.h"
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
void live_H(double *in_vec, double *out_3015933982250180603);
void live_err_fun(double *nom_x, double *delta_x, double *out_8400730981033182085);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8387381869745561018);
void live_H_mod_fun(double *state, double *out_991325598626428970);
void live_f_fun(double *state, double dt, double *out_5768235778925541051);
void live_F_fun(double *state, double dt, double *out_3259261285910579428);
void live_h_4(double *state, double *unused, double *out_2250168671993795102);
void live_H_4(double *state, double *unused, double *out_6297306697124902286);
void live_h_9(double *state, double *unused, double *out_3907296490369071169);
void live_H_9(double *state, double *unused, double *out_4862218441320201860);
void live_h_10(double *state, double *unused, double *out_7839317026457642480);
void live_H_10(double *state, double *unused, double *out_3692584655362384114);
void live_h_12(double *state, double *unused, double *out_5188725649563288731);
void live_H_12(double *state, double *unused, double *out_83951679917830710);
void live_h_35(double *state, double *unused, double *out_9028906405635524646);
void live_H_35(double *state, double *unused, double *out_4384417936227673826);
void live_h_32(double *state, double *unused, double *out_1799345417115305318);
void live_H_32(double *state, double *unused, double *out_6209455692394341728);
void live_h_13(double *state, double *unused, double *out_5425232971149804905);
void live_H_13(double *state, double *unused, double *out_1411400110440861016);
void live_h_14(double *state, double *unused, double *out_3907296490369071169);
void live_H_14(double *state, double *unused, double *out_4862218441320201860);
void live_h_33(double *state, double *unused, double *out_238084949913907181);
void live_H_33(double *state, double *unused, double *out_1233860931588816222);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}