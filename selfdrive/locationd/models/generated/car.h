#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3994003449420885177);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2836171143531724637);
void car_H_mod_fun(double *state, double *out_4300544645246884683);
void car_f_fun(double *state, double dt, double *out_4085036891446952713);
void car_F_fun(double *state, double dt, double *out_7682267835197150451);
void car_h_25(double *state, double *unused, double *out_567823590266453003);
void car_H_25(double *state, double *unused, double *out_3999943544051968747);
void car_h_24(double *state, double *unused, double *out_8280871722589836637);
void car_H_24(double *state, double *unused, double *out_1744095468928151118);
void car_h_30(double *state, double *unused, double *out_5044895097204645462);
void car_H_30(double *state, double *unused, double *out_8527639874179576945);
void car_h_26(double *state, double *unused, double *out_5648210981881122979);
void car_H_26(double *state, double *unused, double *out_7741446862926024971);
void car_h_27(double *state, double *unused, double *out_4433849677394392642);
void car_H_27(double *state, double *unused, double *out_7744340887729549760);
void car_h_29(double *state, double *unused, double *out_7425066469838863811);
void car_H_29(double *state, double *unused, double *out_8017408529865184761);
void car_h_28(double *state, double *unused, double *out_1821975044564654612);
void car_H_28(double *state, double *unused, double *out_5346936526774836281);
void car_h_31(double *state, double *unused, double *out_7195441666554219438);
void car_H_31(double *state, double *unused, double *out_8367654965159376447);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}