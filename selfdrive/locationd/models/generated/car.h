#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_233190742517833586);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2532411625762108203);
void car_H_mod_fun(double *state, double *out_7276270444402064385);
void car_f_fun(double *state, double dt, double *out_2962122646812813231);
void car_F_fun(double *state, double dt, double *out_8634689501083839799);
void car_h_25(double *state, double *unused, double *out_8857349483120821556);
void car_H_25(double *state, double *unused, double *out_5144420576223059326);
void car_h_24(double *state, double *unused, double *out_5794944961364335702);
void car_H_24(double *state, double *unused, double *out_1431151230368458775);
void car_h_30(double *state, double *unused, double *out_3675661563917981687);
void car_H_30(double *state, double *unused, double *out_1772269765268557429);
void car_h_26(double *state, double *unused, double *out_3072345679241504170);
void car_H_26(double *state, double *unused, double *out_8885923895097115550);
void car_h_27(double *state, double *unused, double *out_3855676215459975911);
void car_H_27(double *state, double *unused, double *out_402493546531867482);
void car_h_29(double *state, double *unused, double *out_2732151711016319252);
void car_H_29(double *state, double *unused, double *out_2115856273401418515);
void car_h_28(double *state, double *unused, double *out_3252360330783498394);
void car_H_28(double *state, double *unused, double *out_7198255290470949089);
void car_h_31(double *state, double *unused, double *out_5951945062334707710);
void car_H_31(double *state, double *unused, double *out_5113774614346098898);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}