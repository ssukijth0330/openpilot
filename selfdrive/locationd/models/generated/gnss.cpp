#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6945638239760000243) {
   out_6945638239760000243[0] = delta_x[0] + nom_x[0];
   out_6945638239760000243[1] = delta_x[1] + nom_x[1];
   out_6945638239760000243[2] = delta_x[2] + nom_x[2];
   out_6945638239760000243[3] = delta_x[3] + nom_x[3];
   out_6945638239760000243[4] = delta_x[4] + nom_x[4];
   out_6945638239760000243[5] = delta_x[5] + nom_x[5];
   out_6945638239760000243[6] = delta_x[6] + nom_x[6];
   out_6945638239760000243[7] = delta_x[7] + nom_x[7];
   out_6945638239760000243[8] = delta_x[8] + nom_x[8];
   out_6945638239760000243[9] = delta_x[9] + nom_x[9];
   out_6945638239760000243[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3968798334777133982) {
   out_3968798334777133982[0] = -nom_x[0] + true_x[0];
   out_3968798334777133982[1] = -nom_x[1] + true_x[1];
   out_3968798334777133982[2] = -nom_x[2] + true_x[2];
   out_3968798334777133982[3] = -nom_x[3] + true_x[3];
   out_3968798334777133982[4] = -nom_x[4] + true_x[4];
   out_3968798334777133982[5] = -nom_x[5] + true_x[5];
   out_3968798334777133982[6] = -nom_x[6] + true_x[6];
   out_3968798334777133982[7] = -nom_x[7] + true_x[7];
   out_3968798334777133982[8] = -nom_x[8] + true_x[8];
   out_3968798334777133982[9] = -nom_x[9] + true_x[9];
   out_3968798334777133982[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_246944569381823836) {
   out_246944569381823836[0] = 1.0;
   out_246944569381823836[1] = 0;
   out_246944569381823836[2] = 0;
   out_246944569381823836[3] = 0;
   out_246944569381823836[4] = 0;
   out_246944569381823836[5] = 0;
   out_246944569381823836[6] = 0;
   out_246944569381823836[7] = 0;
   out_246944569381823836[8] = 0;
   out_246944569381823836[9] = 0;
   out_246944569381823836[10] = 0;
   out_246944569381823836[11] = 0;
   out_246944569381823836[12] = 1.0;
   out_246944569381823836[13] = 0;
   out_246944569381823836[14] = 0;
   out_246944569381823836[15] = 0;
   out_246944569381823836[16] = 0;
   out_246944569381823836[17] = 0;
   out_246944569381823836[18] = 0;
   out_246944569381823836[19] = 0;
   out_246944569381823836[20] = 0;
   out_246944569381823836[21] = 0;
   out_246944569381823836[22] = 0;
   out_246944569381823836[23] = 0;
   out_246944569381823836[24] = 1.0;
   out_246944569381823836[25] = 0;
   out_246944569381823836[26] = 0;
   out_246944569381823836[27] = 0;
   out_246944569381823836[28] = 0;
   out_246944569381823836[29] = 0;
   out_246944569381823836[30] = 0;
   out_246944569381823836[31] = 0;
   out_246944569381823836[32] = 0;
   out_246944569381823836[33] = 0;
   out_246944569381823836[34] = 0;
   out_246944569381823836[35] = 0;
   out_246944569381823836[36] = 1.0;
   out_246944569381823836[37] = 0;
   out_246944569381823836[38] = 0;
   out_246944569381823836[39] = 0;
   out_246944569381823836[40] = 0;
   out_246944569381823836[41] = 0;
   out_246944569381823836[42] = 0;
   out_246944569381823836[43] = 0;
   out_246944569381823836[44] = 0;
   out_246944569381823836[45] = 0;
   out_246944569381823836[46] = 0;
   out_246944569381823836[47] = 0;
   out_246944569381823836[48] = 1.0;
   out_246944569381823836[49] = 0;
   out_246944569381823836[50] = 0;
   out_246944569381823836[51] = 0;
   out_246944569381823836[52] = 0;
   out_246944569381823836[53] = 0;
   out_246944569381823836[54] = 0;
   out_246944569381823836[55] = 0;
   out_246944569381823836[56] = 0;
   out_246944569381823836[57] = 0;
   out_246944569381823836[58] = 0;
   out_246944569381823836[59] = 0;
   out_246944569381823836[60] = 1.0;
   out_246944569381823836[61] = 0;
   out_246944569381823836[62] = 0;
   out_246944569381823836[63] = 0;
   out_246944569381823836[64] = 0;
   out_246944569381823836[65] = 0;
   out_246944569381823836[66] = 0;
   out_246944569381823836[67] = 0;
   out_246944569381823836[68] = 0;
   out_246944569381823836[69] = 0;
   out_246944569381823836[70] = 0;
   out_246944569381823836[71] = 0;
   out_246944569381823836[72] = 1.0;
   out_246944569381823836[73] = 0;
   out_246944569381823836[74] = 0;
   out_246944569381823836[75] = 0;
   out_246944569381823836[76] = 0;
   out_246944569381823836[77] = 0;
   out_246944569381823836[78] = 0;
   out_246944569381823836[79] = 0;
   out_246944569381823836[80] = 0;
   out_246944569381823836[81] = 0;
   out_246944569381823836[82] = 0;
   out_246944569381823836[83] = 0;
   out_246944569381823836[84] = 1.0;
   out_246944569381823836[85] = 0;
   out_246944569381823836[86] = 0;
   out_246944569381823836[87] = 0;
   out_246944569381823836[88] = 0;
   out_246944569381823836[89] = 0;
   out_246944569381823836[90] = 0;
   out_246944569381823836[91] = 0;
   out_246944569381823836[92] = 0;
   out_246944569381823836[93] = 0;
   out_246944569381823836[94] = 0;
   out_246944569381823836[95] = 0;
   out_246944569381823836[96] = 1.0;
   out_246944569381823836[97] = 0;
   out_246944569381823836[98] = 0;
   out_246944569381823836[99] = 0;
   out_246944569381823836[100] = 0;
   out_246944569381823836[101] = 0;
   out_246944569381823836[102] = 0;
   out_246944569381823836[103] = 0;
   out_246944569381823836[104] = 0;
   out_246944569381823836[105] = 0;
   out_246944569381823836[106] = 0;
   out_246944569381823836[107] = 0;
   out_246944569381823836[108] = 1.0;
   out_246944569381823836[109] = 0;
   out_246944569381823836[110] = 0;
   out_246944569381823836[111] = 0;
   out_246944569381823836[112] = 0;
   out_246944569381823836[113] = 0;
   out_246944569381823836[114] = 0;
   out_246944569381823836[115] = 0;
   out_246944569381823836[116] = 0;
   out_246944569381823836[117] = 0;
   out_246944569381823836[118] = 0;
   out_246944569381823836[119] = 0;
   out_246944569381823836[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2648529891379557822) {
   out_2648529891379557822[0] = dt*state[3] + state[0];
   out_2648529891379557822[1] = dt*state[4] + state[1];
   out_2648529891379557822[2] = dt*state[5] + state[2];
   out_2648529891379557822[3] = state[3];
   out_2648529891379557822[4] = state[4];
   out_2648529891379557822[5] = state[5];
   out_2648529891379557822[6] = dt*state[7] + state[6];
   out_2648529891379557822[7] = dt*state[8] + state[7];
   out_2648529891379557822[8] = state[8];
   out_2648529891379557822[9] = state[9];
   out_2648529891379557822[10] = state[10];
}
void F_fun(double *state, double dt, double *out_2252592465799141843) {
   out_2252592465799141843[0] = 1;
   out_2252592465799141843[1] = 0;
   out_2252592465799141843[2] = 0;
   out_2252592465799141843[3] = dt;
   out_2252592465799141843[4] = 0;
   out_2252592465799141843[5] = 0;
   out_2252592465799141843[6] = 0;
   out_2252592465799141843[7] = 0;
   out_2252592465799141843[8] = 0;
   out_2252592465799141843[9] = 0;
   out_2252592465799141843[10] = 0;
   out_2252592465799141843[11] = 0;
   out_2252592465799141843[12] = 1;
   out_2252592465799141843[13] = 0;
   out_2252592465799141843[14] = 0;
   out_2252592465799141843[15] = dt;
   out_2252592465799141843[16] = 0;
   out_2252592465799141843[17] = 0;
   out_2252592465799141843[18] = 0;
   out_2252592465799141843[19] = 0;
   out_2252592465799141843[20] = 0;
   out_2252592465799141843[21] = 0;
   out_2252592465799141843[22] = 0;
   out_2252592465799141843[23] = 0;
   out_2252592465799141843[24] = 1;
   out_2252592465799141843[25] = 0;
   out_2252592465799141843[26] = 0;
   out_2252592465799141843[27] = dt;
   out_2252592465799141843[28] = 0;
   out_2252592465799141843[29] = 0;
   out_2252592465799141843[30] = 0;
   out_2252592465799141843[31] = 0;
   out_2252592465799141843[32] = 0;
   out_2252592465799141843[33] = 0;
   out_2252592465799141843[34] = 0;
   out_2252592465799141843[35] = 0;
   out_2252592465799141843[36] = 1;
   out_2252592465799141843[37] = 0;
   out_2252592465799141843[38] = 0;
   out_2252592465799141843[39] = 0;
   out_2252592465799141843[40] = 0;
   out_2252592465799141843[41] = 0;
   out_2252592465799141843[42] = 0;
   out_2252592465799141843[43] = 0;
   out_2252592465799141843[44] = 0;
   out_2252592465799141843[45] = 0;
   out_2252592465799141843[46] = 0;
   out_2252592465799141843[47] = 0;
   out_2252592465799141843[48] = 1;
   out_2252592465799141843[49] = 0;
   out_2252592465799141843[50] = 0;
   out_2252592465799141843[51] = 0;
   out_2252592465799141843[52] = 0;
   out_2252592465799141843[53] = 0;
   out_2252592465799141843[54] = 0;
   out_2252592465799141843[55] = 0;
   out_2252592465799141843[56] = 0;
   out_2252592465799141843[57] = 0;
   out_2252592465799141843[58] = 0;
   out_2252592465799141843[59] = 0;
   out_2252592465799141843[60] = 1;
   out_2252592465799141843[61] = 0;
   out_2252592465799141843[62] = 0;
   out_2252592465799141843[63] = 0;
   out_2252592465799141843[64] = 0;
   out_2252592465799141843[65] = 0;
   out_2252592465799141843[66] = 0;
   out_2252592465799141843[67] = 0;
   out_2252592465799141843[68] = 0;
   out_2252592465799141843[69] = 0;
   out_2252592465799141843[70] = 0;
   out_2252592465799141843[71] = 0;
   out_2252592465799141843[72] = 1;
   out_2252592465799141843[73] = dt;
   out_2252592465799141843[74] = 0;
   out_2252592465799141843[75] = 0;
   out_2252592465799141843[76] = 0;
   out_2252592465799141843[77] = 0;
   out_2252592465799141843[78] = 0;
   out_2252592465799141843[79] = 0;
   out_2252592465799141843[80] = 0;
   out_2252592465799141843[81] = 0;
   out_2252592465799141843[82] = 0;
   out_2252592465799141843[83] = 0;
   out_2252592465799141843[84] = 1;
   out_2252592465799141843[85] = dt;
   out_2252592465799141843[86] = 0;
   out_2252592465799141843[87] = 0;
   out_2252592465799141843[88] = 0;
   out_2252592465799141843[89] = 0;
   out_2252592465799141843[90] = 0;
   out_2252592465799141843[91] = 0;
   out_2252592465799141843[92] = 0;
   out_2252592465799141843[93] = 0;
   out_2252592465799141843[94] = 0;
   out_2252592465799141843[95] = 0;
   out_2252592465799141843[96] = 1;
   out_2252592465799141843[97] = 0;
   out_2252592465799141843[98] = 0;
   out_2252592465799141843[99] = 0;
   out_2252592465799141843[100] = 0;
   out_2252592465799141843[101] = 0;
   out_2252592465799141843[102] = 0;
   out_2252592465799141843[103] = 0;
   out_2252592465799141843[104] = 0;
   out_2252592465799141843[105] = 0;
   out_2252592465799141843[106] = 0;
   out_2252592465799141843[107] = 0;
   out_2252592465799141843[108] = 1;
   out_2252592465799141843[109] = 0;
   out_2252592465799141843[110] = 0;
   out_2252592465799141843[111] = 0;
   out_2252592465799141843[112] = 0;
   out_2252592465799141843[113] = 0;
   out_2252592465799141843[114] = 0;
   out_2252592465799141843[115] = 0;
   out_2252592465799141843[116] = 0;
   out_2252592465799141843[117] = 0;
   out_2252592465799141843[118] = 0;
   out_2252592465799141843[119] = 0;
   out_2252592465799141843[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_4163819229610976683) {
   out_4163819229610976683[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2287387615128234836) {
   out_2287387615128234836[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2287387615128234836[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2287387615128234836[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2287387615128234836[3] = 0;
   out_2287387615128234836[4] = 0;
   out_2287387615128234836[5] = 0;
   out_2287387615128234836[6] = 1;
   out_2287387615128234836[7] = 0;
   out_2287387615128234836[8] = 0;
   out_2287387615128234836[9] = 0;
   out_2287387615128234836[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_4847015344803285583) {
   out_4847015344803285583[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2751431159818563852) {
   out_2751431159818563852[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2751431159818563852[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2751431159818563852[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2751431159818563852[3] = 0;
   out_2751431159818563852[4] = 0;
   out_2751431159818563852[5] = 0;
   out_2751431159818563852[6] = 1;
   out_2751431159818563852[7] = 0;
   out_2751431159818563852[8] = 0;
   out_2751431159818563852[9] = 1;
   out_2751431159818563852[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_9041024282100407462) {
   out_9041024282100407462[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8047877791999041801) {
   out_8047877791999041801[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[6] = 0;
   out_8047877791999041801[7] = 1;
   out_8047877791999041801[8] = 0;
   out_8047877791999041801[9] = 0;
   out_8047877791999041801[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_9041024282100407462) {
   out_9041024282100407462[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8047877791999041801) {
   out_8047877791999041801[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8047877791999041801[6] = 0;
   out_8047877791999041801[7] = 1;
   out_8047877791999041801[8] = 0;
   out_8047877791999041801[9] = 0;
   out_8047877791999041801[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6945638239760000243) {
  err_fun(nom_x, delta_x, out_6945638239760000243);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_3968798334777133982) {
  inv_err_fun(nom_x, true_x, out_3968798334777133982);
}
void gnss_H_mod_fun(double *state, double *out_246944569381823836) {
  H_mod_fun(state, out_246944569381823836);
}
void gnss_f_fun(double *state, double dt, double *out_2648529891379557822) {
  f_fun(state,  dt, out_2648529891379557822);
}
void gnss_F_fun(double *state, double dt, double *out_2252592465799141843) {
  F_fun(state,  dt, out_2252592465799141843);
}
void gnss_h_6(double *state, double *sat_pos, double *out_4163819229610976683) {
  h_6(state, sat_pos, out_4163819229610976683);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2287387615128234836) {
  H_6(state, sat_pos, out_2287387615128234836);
}
void gnss_h_20(double *state, double *sat_pos, double *out_4847015344803285583) {
  h_20(state, sat_pos, out_4847015344803285583);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2751431159818563852) {
  H_20(state, sat_pos, out_2751431159818563852);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_9041024282100407462) {
  h_7(state, sat_pos_vel, out_9041024282100407462);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8047877791999041801) {
  H_7(state, sat_pos_vel, out_8047877791999041801);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_9041024282100407462) {
  h_21(state, sat_pos_vel, out_9041024282100407462);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8047877791999041801) {
  H_21(state, sat_pos_vel, out_8047877791999041801);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
