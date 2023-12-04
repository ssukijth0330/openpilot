#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_233190742517833586) {
   out_233190742517833586[0] = delta_x[0] + nom_x[0];
   out_233190742517833586[1] = delta_x[1] + nom_x[1];
   out_233190742517833586[2] = delta_x[2] + nom_x[2];
   out_233190742517833586[3] = delta_x[3] + nom_x[3];
   out_233190742517833586[4] = delta_x[4] + nom_x[4];
   out_233190742517833586[5] = delta_x[5] + nom_x[5];
   out_233190742517833586[6] = delta_x[6] + nom_x[6];
   out_233190742517833586[7] = delta_x[7] + nom_x[7];
   out_233190742517833586[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2532411625762108203) {
   out_2532411625762108203[0] = -nom_x[0] + true_x[0];
   out_2532411625762108203[1] = -nom_x[1] + true_x[1];
   out_2532411625762108203[2] = -nom_x[2] + true_x[2];
   out_2532411625762108203[3] = -nom_x[3] + true_x[3];
   out_2532411625762108203[4] = -nom_x[4] + true_x[4];
   out_2532411625762108203[5] = -nom_x[5] + true_x[5];
   out_2532411625762108203[6] = -nom_x[6] + true_x[6];
   out_2532411625762108203[7] = -nom_x[7] + true_x[7];
   out_2532411625762108203[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7276270444402064385) {
   out_7276270444402064385[0] = 1.0;
   out_7276270444402064385[1] = 0;
   out_7276270444402064385[2] = 0;
   out_7276270444402064385[3] = 0;
   out_7276270444402064385[4] = 0;
   out_7276270444402064385[5] = 0;
   out_7276270444402064385[6] = 0;
   out_7276270444402064385[7] = 0;
   out_7276270444402064385[8] = 0;
   out_7276270444402064385[9] = 0;
   out_7276270444402064385[10] = 1.0;
   out_7276270444402064385[11] = 0;
   out_7276270444402064385[12] = 0;
   out_7276270444402064385[13] = 0;
   out_7276270444402064385[14] = 0;
   out_7276270444402064385[15] = 0;
   out_7276270444402064385[16] = 0;
   out_7276270444402064385[17] = 0;
   out_7276270444402064385[18] = 0;
   out_7276270444402064385[19] = 0;
   out_7276270444402064385[20] = 1.0;
   out_7276270444402064385[21] = 0;
   out_7276270444402064385[22] = 0;
   out_7276270444402064385[23] = 0;
   out_7276270444402064385[24] = 0;
   out_7276270444402064385[25] = 0;
   out_7276270444402064385[26] = 0;
   out_7276270444402064385[27] = 0;
   out_7276270444402064385[28] = 0;
   out_7276270444402064385[29] = 0;
   out_7276270444402064385[30] = 1.0;
   out_7276270444402064385[31] = 0;
   out_7276270444402064385[32] = 0;
   out_7276270444402064385[33] = 0;
   out_7276270444402064385[34] = 0;
   out_7276270444402064385[35] = 0;
   out_7276270444402064385[36] = 0;
   out_7276270444402064385[37] = 0;
   out_7276270444402064385[38] = 0;
   out_7276270444402064385[39] = 0;
   out_7276270444402064385[40] = 1.0;
   out_7276270444402064385[41] = 0;
   out_7276270444402064385[42] = 0;
   out_7276270444402064385[43] = 0;
   out_7276270444402064385[44] = 0;
   out_7276270444402064385[45] = 0;
   out_7276270444402064385[46] = 0;
   out_7276270444402064385[47] = 0;
   out_7276270444402064385[48] = 0;
   out_7276270444402064385[49] = 0;
   out_7276270444402064385[50] = 1.0;
   out_7276270444402064385[51] = 0;
   out_7276270444402064385[52] = 0;
   out_7276270444402064385[53] = 0;
   out_7276270444402064385[54] = 0;
   out_7276270444402064385[55] = 0;
   out_7276270444402064385[56] = 0;
   out_7276270444402064385[57] = 0;
   out_7276270444402064385[58] = 0;
   out_7276270444402064385[59] = 0;
   out_7276270444402064385[60] = 1.0;
   out_7276270444402064385[61] = 0;
   out_7276270444402064385[62] = 0;
   out_7276270444402064385[63] = 0;
   out_7276270444402064385[64] = 0;
   out_7276270444402064385[65] = 0;
   out_7276270444402064385[66] = 0;
   out_7276270444402064385[67] = 0;
   out_7276270444402064385[68] = 0;
   out_7276270444402064385[69] = 0;
   out_7276270444402064385[70] = 1.0;
   out_7276270444402064385[71] = 0;
   out_7276270444402064385[72] = 0;
   out_7276270444402064385[73] = 0;
   out_7276270444402064385[74] = 0;
   out_7276270444402064385[75] = 0;
   out_7276270444402064385[76] = 0;
   out_7276270444402064385[77] = 0;
   out_7276270444402064385[78] = 0;
   out_7276270444402064385[79] = 0;
   out_7276270444402064385[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2962122646812813231) {
   out_2962122646812813231[0] = state[0];
   out_2962122646812813231[1] = state[1];
   out_2962122646812813231[2] = state[2];
   out_2962122646812813231[3] = state[3];
   out_2962122646812813231[4] = state[4];
   out_2962122646812813231[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2962122646812813231[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2962122646812813231[7] = state[7];
   out_2962122646812813231[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8634689501083839799) {
   out_8634689501083839799[0] = 1;
   out_8634689501083839799[1] = 0;
   out_8634689501083839799[2] = 0;
   out_8634689501083839799[3] = 0;
   out_8634689501083839799[4] = 0;
   out_8634689501083839799[5] = 0;
   out_8634689501083839799[6] = 0;
   out_8634689501083839799[7] = 0;
   out_8634689501083839799[8] = 0;
   out_8634689501083839799[9] = 0;
   out_8634689501083839799[10] = 1;
   out_8634689501083839799[11] = 0;
   out_8634689501083839799[12] = 0;
   out_8634689501083839799[13] = 0;
   out_8634689501083839799[14] = 0;
   out_8634689501083839799[15] = 0;
   out_8634689501083839799[16] = 0;
   out_8634689501083839799[17] = 0;
   out_8634689501083839799[18] = 0;
   out_8634689501083839799[19] = 0;
   out_8634689501083839799[20] = 1;
   out_8634689501083839799[21] = 0;
   out_8634689501083839799[22] = 0;
   out_8634689501083839799[23] = 0;
   out_8634689501083839799[24] = 0;
   out_8634689501083839799[25] = 0;
   out_8634689501083839799[26] = 0;
   out_8634689501083839799[27] = 0;
   out_8634689501083839799[28] = 0;
   out_8634689501083839799[29] = 0;
   out_8634689501083839799[30] = 1;
   out_8634689501083839799[31] = 0;
   out_8634689501083839799[32] = 0;
   out_8634689501083839799[33] = 0;
   out_8634689501083839799[34] = 0;
   out_8634689501083839799[35] = 0;
   out_8634689501083839799[36] = 0;
   out_8634689501083839799[37] = 0;
   out_8634689501083839799[38] = 0;
   out_8634689501083839799[39] = 0;
   out_8634689501083839799[40] = 1;
   out_8634689501083839799[41] = 0;
   out_8634689501083839799[42] = 0;
   out_8634689501083839799[43] = 0;
   out_8634689501083839799[44] = 0;
   out_8634689501083839799[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8634689501083839799[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8634689501083839799[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8634689501083839799[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8634689501083839799[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8634689501083839799[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8634689501083839799[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8634689501083839799[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8634689501083839799[53] = -9.8000000000000007*dt;
   out_8634689501083839799[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8634689501083839799[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8634689501083839799[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8634689501083839799[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8634689501083839799[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8634689501083839799[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8634689501083839799[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8634689501083839799[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8634689501083839799[62] = 0;
   out_8634689501083839799[63] = 0;
   out_8634689501083839799[64] = 0;
   out_8634689501083839799[65] = 0;
   out_8634689501083839799[66] = 0;
   out_8634689501083839799[67] = 0;
   out_8634689501083839799[68] = 0;
   out_8634689501083839799[69] = 0;
   out_8634689501083839799[70] = 1;
   out_8634689501083839799[71] = 0;
   out_8634689501083839799[72] = 0;
   out_8634689501083839799[73] = 0;
   out_8634689501083839799[74] = 0;
   out_8634689501083839799[75] = 0;
   out_8634689501083839799[76] = 0;
   out_8634689501083839799[77] = 0;
   out_8634689501083839799[78] = 0;
   out_8634689501083839799[79] = 0;
   out_8634689501083839799[80] = 1;
}
void h_25(double *state, double *unused, double *out_8857349483120821556) {
   out_8857349483120821556[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5144420576223059326) {
   out_5144420576223059326[0] = 0;
   out_5144420576223059326[1] = 0;
   out_5144420576223059326[2] = 0;
   out_5144420576223059326[3] = 0;
   out_5144420576223059326[4] = 0;
   out_5144420576223059326[5] = 0;
   out_5144420576223059326[6] = 1;
   out_5144420576223059326[7] = 0;
   out_5144420576223059326[8] = 0;
}
void h_24(double *state, double *unused, double *out_5794944961364335702) {
   out_5794944961364335702[0] = state[4];
   out_5794944961364335702[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1431151230368458775) {
   out_1431151230368458775[0] = 0;
   out_1431151230368458775[1] = 0;
   out_1431151230368458775[2] = 0;
   out_1431151230368458775[3] = 0;
   out_1431151230368458775[4] = 1;
   out_1431151230368458775[5] = 0;
   out_1431151230368458775[6] = 0;
   out_1431151230368458775[7] = 0;
   out_1431151230368458775[8] = 0;
   out_1431151230368458775[9] = 0;
   out_1431151230368458775[10] = 0;
   out_1431151230368458775[11] = 0;
   out_1431151230368458775[12] = 0;
   out_1431151230368458775[13] = 0;
   out_1431151230368458775[14] = 1;
   out_1431151230368458775[15] = 0;
   out_1431151230368458775[16] = 0;
   out_1431151230368458775[17] = 0;
}
void h_30(double *state, double *unused, double *out_3675661563917981687) {
   out_3675661563917981687[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1772269765268557429) {
   out_1772269765268557429[0] = 0;
   out_1772269765268557429[1] = 0;
   out_1772269765268557429[2] = 0;
   out_1772269765268557429[3] = 0;
   out_1772269765268557429[4] = 1;
   out_1772269765268557429[5] = 0;
   out_1772269765268557429[6] = 0;
   out_1772269765268557429[7] = 0;
   out_1772269765268557429[8] = 0;
}
void h_26(double *state, double *unused, double *out_3072345679241504170) {
   out_3072345679241504170[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8885923895097115550) {
   out_8885923895097115550[0] = 0;
   out_8885923895097115550[1] = 0;
   out_8885923895097115550[2] = 0;
   out_8885923895097115550[3] = 0;
   out_8885923895097115550[4] = 0;
   out_8885923895097115550[5] = 0;
   out_8885923895097115550[6] = 0;
   out_8885923895097115550[7] = 1;
   out_8885923895097115550[8] = 0;
}
void h_27(double *state, double *unused, double *out_3855676215459975911) {
   out_3855676215459975911[0] = state[3];
}
void H_27(double *state, double *unused, double *out_402493546531867482) {
   out_402493546531867482[0] = 0;
   out_402493546531867482[1] = 0;
   out_402493546531867482[2] = 0;
   out_402493546531867482[3] = 1;
   out_402493546531867482[4] = 0;
   out_402493546531867482[5] = 0;
   out_402493546531867482[6] = 0;
   out_402493546531867482[7] = 0;
   out_402493546531867482[8] = 0;
}
void h_29(double *state, double *unused, double *out_2732151711016319252) {
   out_2732151711016319252[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2115856273401418515) {
   out_2115856273401418515[0] = 0;
   out_2115856273401418515[1] = 1;
   out_2115856273401418515[2] = 0;
   out_2115856273401418515[3] = 0;
   out_2115856273401418515[4] = 0;
   out_2115856273401418515[5] = 0;
   out_2115856273401418515[6] = 0;
   out_2115856273401418515[7] = 0;
   out_2115856273401418515[8] = 0;
}
void h_28(double *state, double *unused, double *out_3252360330783498394) {
   out_3252360330783498394[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7198255290470949089) {
   out_7198255290470949089[0] = 1;
   out_7198255290470949089[1] = 0;
   out_7198255290470949089[2] = 0;
   out_7198255290470949089[3] = 0;
   out_7198255290470949089[4] = 0;
   out_7198255290470949089[5] = 0;
   out_7198255290470949089[6] = 0;
   out_7198255290470949089[7] = 0;
   out_7198255290470949089[8] = 0;
}
void h_31(double *state, double *unused, double *out_5951945062334707710) {
   out_5951945062334707710[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5113774614346098898) {
   out_5113774614346098898[0] = 0;
   out_5113774614346098898[1] = 0;
   out_5113774614346098898[2] = 0;
   out_5113774614346098898[3] = 0;
   out_5113774614346098898[4] = 0;
   out_5113774614346098898[5] = 0;
   out_5113774614346098898[6] = 0;
   out_5113774614346098898[7] = 0;
   out_5113774614346098898[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_233190742517833586) {
  err_fun(nom_x, delta_x, out_233190742517833586);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2532411625762108203) {
  inv_err_fun(nom_x, true_x, out_2532411625762108203);
}
void car_H_mod_fun(double *state, double *out_7276270444402064385) {
  H_mod_fun(state, out_7276270444402064385);
}
void car_f_fun(double *state, double dt, double *out_2962122646812813231) {
  f_fun(state,  dt, out_2962122646812813231);
}
void car_F_fun(double *state, double dt, double *out_8634689501083839799) {
  F_fun(state,  dt, out_8634689501083839799);
}
void car_h_25(double *state, double *unused, double *out_8857349483120821556) {
  h_25(state, unused, out_8857349483120821556);
}
void car_H_25(double *state, double *unused, double *out_5144420576223059326) {
  H_25(state, unused, out_5144420576223059326);
}
void car_h_24(double *state, double *unused, double *out_5794944961364335702) {
  h_24(state, unused, out_5794944961364335702);
}
void car_H_24(double *state, double *unused, double *out_1431151230368458775) {
  H_24(state, unused, out_1431151230368458775);
}
void car_h_30(double *state, double *unused, double *out_3675661563917981687) {
  h_30(state, unused, out_3675661563917981687);
}
void car_H_30(double *state, double *unused, double *out_1772269765268557429) {
  H_30(state, unused, out_1772269765268557429);
}
void car_h_26(double *state, double *unused, double *out_3072345679241504170) {
  h_26(state, unused, out_3072345679241504170);
}
void car_H_26(double *state, double *unused, double *out_8885923895097115550) {
  H_26(state, unused, out_8885923895097115550);
}
void car_h_27(double *state, double *unused, double *out_3855676215459975911) {
  h_27(state, unused, out_3855676215459975911);
}
void car_H_27(double *state, double *unused, double *out_402493546531867482) {
  H_27(state, unused, out_402493546531867482);
}
void car_h_29(double *state, double *unused, double *out_2732151711016319252) {
  h_29(state, unused, out_2732151711016319252);
}
void car_H_29(double *state, double *unused, double *out_2115856273401418515) {
  H_29(state, unused, out_2115856273401418515);
}
void car_h_28(double *state, double *unused, double *out_3252360330783498394) {
  h_28(state, unused, out_3252360330783498394);
}
void car_H_28(double *state, double *unused, double *out_7198255290470949089) {
  H_28(state, unused, out_7198255290470949089);
}
void car_h_31(double *state, double *unused, double *out_5951945062334707710) {
  h_31(state, unused, out_5951945062334707710);
}
void car_H_31(double *state, double *unused, double *out_5113774614346098898) {
  H_31(state, unused, out_5113774614346098898);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
