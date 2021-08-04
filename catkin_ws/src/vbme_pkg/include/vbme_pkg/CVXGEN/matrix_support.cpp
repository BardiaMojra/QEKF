/* Produced by CVXGEN, 2020-09-14 19:22:53 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
//~ #include "solver.h"
#include "vbme_pkg/CVXGEN/solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
  lhs[10] = 0;
  lhs[11] = 0;
  lhs[12] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.A[0])-rhs[1]*(-params.A[10])-rhs[2]*(-params.A[20])-rhs[3]*(-params.A[30])-rhs[4]*(-params.A[40])-rhs[5]*(-params.A[50])-rhs[6]*(-params.A[60])-rhs[7]*(-params.A[70])-rhs[8]*(-params.A[80])-rhs[9]*(-params.A[90])-rhs[10]*(-params.A[100])-rhs[11]*(-params.A[110])-rhs[12]*(-params.A[120]);
  lhs[1] = -rhs[0]*(-params.A[1])-rhs[1]*(-params.A[11])-rhs[2]*(-params.A[21])-rhs[3]*(-params.A[31])-rhs[4]*(-params.A[41])-rhs[5]*(-params.A[51])-rhs[6]*(-params.A[61])-rhs[7]*(-params.A[71])-rhs[8]*(-params.A[81])-rhs[9]*(-params.A[91])-rhs[10]*(-params.A[101])-rhs[11]*(-params.A[111])-rhs[12]*(-params.A[121]);
  lhs[2] = -rhs[0]*(-params.A[2])-rhs[1]*(-params.A[12])-rhs[2]*(-params.A[22])-rhs[3]*(-params.A[32])-rhs[4]*(-params.A[42])-rhs[5]*(-params.A[52])-rhs[6]*(-params.A[62])-rhs[7]*(-params.A[72])-rhs[8]*(-params.A[82])-rhs[9]*(-params.A[92])-rhs[10]*(-params.A[102])-rhs[11]*(-params.A[112])-rhs[12]*(-params.A[122]);
  lhs[3] = -rhs[0]*(-params.A[3])-rhs[1]*(-params.A[13])-rhs[2]*(-params.A[23])-rhs[3]*(-params.A[33])-rhs[4]*(-params.A[43])-rhs[5]*(-params.A[53])-rhs[6]*(-params.A[63])-rhs[7]*(-params.A[73])-rhs[8]*(-params.A[83])-rhs[9]*(-params.A[93])-rhs[10]*(-params.A[103])-rhs[11]*(-params.A[113])-rhs[12]*(-params.A[123]);
  lhs[4] = -rhs[0]*(-params.A[4])-rhs[1]*(-params.A[14])-rhs[2]*(-params.A[24])-rhs[3]*(-params.A[34])-rhs[4]*(-params.A[44])-rhs[5]*(-params.A[54])-rhs[6]*(-params.A[64])-rhs[7]*(-params.A[74])-rhs[8]*(-params.A[84])-rhs[9]*(-params.A[94])-rhs[10]*(-params.A[104])-rhs[11]*(-params.A[114])-rhs[12]*(-params.A[124]);
  lhs[5] = -rhs[0]*(-params.A[5])-rhs[1]*(-params.A[15])-rhs[2]*(-params.A[25])-rhs[3]*(-params.A[35])-rhs[4]*(-params.A[45])-rhs[5]*(-params.A[55])-rhs[6]*(-params.A[65])-rhs[7]*(-params.A[75])-rhs[8]*(-params.A[85])-rhs[9]*(-params.A[95])-rhs[10]*(-params.A[105])-rhs[11]*(-params.A[115])-rhs[12]*(-params.A[125]);
  lhs[6] = -rhs[0]*(-params.A[6])-rhs[1]*(-params.A[16])-rhs[2]*(-params.A[26])-rhs[3]*(-params.A[36])-rhs[4]*(-params.A[46])-rhs[5]*(-params.A[56])-rhs[6]*(-params.A[66])-rhs[7]*(-params.A[76])-rhs[8]*(-params.A[86])-rhs[9]*(-params.A[96])-rhs[10]*(-params.A[106])-rhs[11]*(-params.A[116])-rhs[12]*(-params.A[126]);
  lhs[7] = -rhs[0]*(-params.A[7])-rhs[1]*(-params.A[17])-rhs[2]*(-params.A[27])-rhs[3]*(-params.A[37])-rhs[4]*(-params.A[47])-rhs[5]*(-params.A[57])-rhs[6]*(-params.A[67])-rhs[7]*(-params.A[77])-rhs[8]*(-params.A[87])-rhs[9]*(-params.A[97])-rhs[10]*(-params.A[107])-rhs[11]*(-params.A[117])-rhs[12]*(-params.A[127]);
  lhs[8] = -rhs[0]*(-params.A[8])-rhs[1]*(-params.A[18])-rhs[2]*(-params.A[28])-rhs[3]*(-params.A[38])-rhs[4]*(-params.A[48])-rhs[5]*(-params.A[58])-rhs[6]*(-params.A[68])-rhs[7]*(-params.A[78])-rhs[8]*(-params.A[88])-rhs[9]*(-params.A[98])-rhs[10]*(-params.A[108])-rhs[11]*(-params.A[118])-rhs[12]*(-params.A[128]);
  lhs[9] = -rhs[0]*(-params.A[9])-rhs[1]*(-params.A[19])-rhs[2]*(-params.A[29])-rhs[3]*(-params.A[39])-rhs[4]*(-params.A[49])-rhs[5]*(-params.A[59])-rhs[6]*(-params.A[69])-rhs[7]*(-params.A[79])-rhs[8]*(-params.A[89])-rhs[9]*(-params.A[99])-rhs[10]*(-params.A[109])-rhs[11]*(-params.A[119])-rhs[12]*(-params.A[129]);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.A[0])-rhs[1]*(-params.A[1])-rhs[2]*(-params.A[2])-rhs[3]*(-params.A[3])-rhs[4]*(-params.A[4])-rhs[5]*(-params.A[5])-rhs[6]*(-params.A[6])-rhs[7]*(-params.A[7])-rhs[8]*(-params.A[8])-rhs[9]*(-params.A[9]);
  lhs[1] = -rhs[0]*(-params.A[10])-rhs[1]*(-params.A[11])-rhs[2]*(-params.A[12])-rhs[3]*(-params.A[13])-rhs[4]*(-params.A[14])-rhs[5]*(-params.A[15])-rhs[6]*(-params.A[16])-rhs[7]*(-params.A[17])-rhs[8]*(-params.A[18])-rhs[9]*(-params.A[19]);
  lhs[2] = -rhs[0]*(-params.A[20])-rhs[1]*(-params.A[21])-rhs[2]*(-params.A[22])-rhs[3]*(-params.A[23])-rhs[4]*(-params.A[24])-rhs[5]*(-params.A[25])-rhs[6]*(-params.A[26])-rhs[7]*(-params.A[27])-rhs[8]*(-params.A[28])-rhs[9]*(-params.A[29]);
  lhs[3] = -rhs[0]*(-params.A[30])-rhs[1]*(-params.A[31])-rhs[2]*(-params.A[32])-rhs[3]*(-params.A[33])-rhs[4]*(-params.A[34])-rhs[5]*(-params.A[35])-rhs[6]*(-params.A[36])-rhs[7]*(-params.A[37])-rhs[8]*(-params.A[38])-rhs[9]*(-params.A[39]);
  lhs[4] = -rhs[0]*(-params.A[40])-rhs[1]*(-params.A[41])-rhs[2]*(-params.A[42])-rhs[3]*(-params.A[43])-rhs[4]*(-params.A[44])-rhs[5]*(-params.A[45])-rhs[6]*(-params.A[46])-rhs[7]*(-params.A[47])-rhs[8]*(-params.A[48])-rhs[9]*(-params.A[49]);
  lhs[5] = -rhs[0]*(-params.A[50])-rhs[1]*(-params.A[51])-rhs[2]*(-params.A[52])-rhs[3]*(-params.A[53])-rhs[4]*(-params.A[54])-rhs[5]*(-params.A[55])-rhs[6]*(-params.A[56])-rhs[7]*(-params.A[57])-rhs[8]*(-params.A[58])-rhs[9]*(-params.A[59]);
  lhs[6] = -rhs[0]*(-params.A[60])-rhs[1]*(-params.A[61])-rhs[2]*(-params.A[62])-rhs[3]*(-params.A[63])-rhs[4]*(-params.A[64])-rhs[5]*(-params.A[65])-rhs[6]*(-params.A[66])-rhs[7]*(-params.A[67])-rhs[8]*(-params.A[68])-rhs[9]*(-params.A[69]);
  lhs[7] = -rhs[0]*(-params.A[70])-rhs[1]*(-params.A[71])-rhs[2]*(-params.A[72])-rhs[3]*(-params.A[73])-rhs[4]*(-params.A[74])-rhs[5]*(-params.A[75])-rhs[6]*(-params.A[76])-rhs[7]*(-params.A[77])-rhs[8]*(-params.A[78])-rhs[9]*(-params.A[79]);
  lhs[8] = -rhs[0]*(-params.A[80])-rhs[1]*(-params.A[81])-rhs[2]*(-params.A[82])-rhs[3]*(-params.A[83])-rhs[4]*(-params.A[84])-rhs[5]*(-params.A[85])-rhs[6]*(-params.A[86])-rhs[7]*(-params.A[87])-rhs[8]*(-params.A[88])-rhs[9]*(-params.A[89]);
  lhs[9] = -rhs[0]*(-params.A[90])-rhs[1]*(-params.A[91])-rhs[2]*(-params.A[92])-rhs[3]*(-params.A[93])-rhs[4]*(-params.A[94])-rhs[5]*(-params.A[95])-rhs[6]*(-params.A[96])-rhs[7]*(-params.A[97])-rhs[8]*(-params.A[98])-rhs[9]*(-params.A[99]);
  lhs[10] = -rhs[0]*(-params.A[100])-rhs[1]*(-params.A[101])-rhs[2]*(-params.A[102])-rhs[3]*(-params.A[103])-rhs[4]*(-params.A[104])-rhs[5]*(-params.A[105])-rhs[6]*(-params.A[106])-rhs[7]*(-params.A[107])-rhs[8]*(-params.A[108])-rhs[9]*(-params.A[109]);
  lhs[11] = -rhs[0]*(-params.A[110])-rhs[1]*(-params.A[111])-rhs[2]*(-params.A[112])-rhs[3]*(-params.A[113])-rhs[4]*(-params.A[114])-rhs[5]*(-params.A[115])-rhs[6]*(-params.A[116])-rhs[7]*(-params.A[117])-rhs[8]*(-params.A[118])-rhs[9]*(-params.A[119]);
  lhs[12] = -rhs[0]*(-params.A[120])-rhs[1]*(-params.A[121])-rhs[2]*(-params.A[122])-rhs[3]*(-params.A[123])-rhs[4]*(-params.A[124])-rhs[5]*(-params.A[125])-rhs[6]*(-params.A[126])-rhs[7]*(-params.A[127])-rhs[8]*(-params.A[128])-rhs[9]*(-params.A[129]);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.Q[0])+rhs[1]*(2*params.Q[13])+rhs[2]*(2*params.Q[26])+rhs[3]*(2*params.Q[39])+rhs[4]*(2*params.Q[52])+rhs[5]*(2*params.Q[65])+rhs[6]*(2*params.Q[78])+rhs[7]*(2*params.Q[91])+rhs[8]*(2*params.Q[104])+rhs[9]*(2*params.Q[117])+rhs[10]*(2*params.Q[130])+rhs[11]*(2*params.Q[143])+rhs[12]*(2*params.Q[156]);
  lhs[1] = rhs[0]*(2*params.Q[1])+rhs[1]*(2*params.Q[14])+rhs[2]*(2*params.Q[27])+rhs[3]*(2*params.Q[40])+rhs[4]*(2*params.Q[53])+rhs[5]*(2*params.Q[66])+rhs[6]*(2*params.Q[79])+rhs[7]*(2*params.Q[92])+rhs[8]*(2*params.Q[105])+rhs[9]*(2*params.Q[118])+rhs[10]*(2*params.Q[131])+rhs[11]*(2*params.Q[144])+rhs[12]*(2*params.Q[157]);
  lhs[2] = rhs[0]*(2*params.Q[2])+rhs[1]*(2*params.Q[15])+rhs[2]*(2*params.Q[28])+rhs[3]*(2*params.Q[41])+rhs[4]*(2*params.Q[54])+rhs[5]*(2*params.Q[67])+rhs[6]*(2*params.Q[80])+rhs[7]*(2*params.Q[93])+rhs[8]*(2*params.Q[106])+rhs[9]*(2*params.Q[119])+rhs[10]*(2*params.Q[132])+rhs[11]*(2*params.Q[145])+rhs[12]*(2*params.Q[158]);
  lhs[3] = rhs[0]*(2*params.Q[3])+rhs[1]*(2*params.Q[16])+rhs[2]*(2*params.Q[29])+rhs[3]*(2*params.Q[42])+rhs[4]*(2*params.Q[55])+rhs[5]*(2*params.Q[68])+rhs[6]*(2*params.Q[81])+rhs[7]*(2*params.Q[94])+rhs[8]*(2*params.Q[107])+rhs[9]*(2*params.Q[120])+rhs[10]*(2*params.Q[133])+rhs[11]*(2*params.Q[146])+rhs[12]*(2*params.Q[159]);
  lhs[4] = rhs[0]*(2*params.Q[4])+rhs[1]*(2*params.Q[17])+rhs[2]*(2*params.Q[30])+rhs[3]*(2*params.Q[43])+rhs[4]*(2*params.Q[56])+rhs[5]*(2*params.Q[69])+rhs[6]*(2*params.Q[82])+rhs[7]*(2*params.Q[95])+rhs[8]*(2*params.Q[108])+rhs[9]*(2*params.Q[121])+rhs[10]*(2*params.Q[134])+rhs[11]*(2*params.Q[147])+rhs[12]*(2*params.Q[160]);
  lhs[5] = rhs[0]*(2*params.Q[5])+rhs[1]*(2*params.Q[18])+rhs[2]*(2*params.Q[31])+rhs[3]*(2*params.Q[44])+rhs[4]*(2*params.Q[57])+rhs[5]*(2*params.Q[70])+rhs[6]*(2*params.Q[83])+rhs[7]*(2*params.Q[96])+rhs[8]*(2*params.Q[109])+rhs[9]*(2*params.Q[122])+rhs[10]*(2*params.Q[135])+rhs[11]*(2*params.Q[148])+rhs[12]*(2*params.Q[161]);
  lhs[6] = rhs[0]*(2*params.Q[6])+rhs[1]*(2*params.Q[19])+rhs[2]*(2*params.Q[32])+rhs[3]*(2*params.Q[45])+rhs[4]*(2*params.Q[58])+rhs[5]*(2*params.Q[71])+rhs[6]*(2*params.Q[84])+rhs[7]*(2*params.Q[97])+rhs[8]*(2*params.Q[110])+rhs[9]*(2*params.Q[123])+rhs[10]*(2*params.Q[136])+rhs[11]*(2*params.Q[149])+rhs[12]*(2*params.Q[162]);
  lhs[7] = rhs[0]*(2*params.Q[7])+rhs[1]*(2*params.Q[20])+rhs[2]*(2*params.Q[33])+rhs[3]*(2*params.Q[46])+rhs[4]*(2*params.Q[59])+rhs[5]*(2*params.Q[72])+rhs[6]*(2*params.Q[85])+rhs[7]*(2*params.Q[98])+rhs[8]*(2*params.Q[111])+rhs[9]*(2*params.Q[124])+rhs[10]*(2*params.Q[137])+rhs[11]*(2*params.Q[150])+rhs[12]*(2*params.Q[163]);
  lhs[8] = rhs[0]*(2*params.Q[8])+rhs[1]*(2*params.Q[21])+rhs[2]*(2*params.Q[34])+rhs[3]*(2*params.Q[47])+rhs[4]*(2*params.Q[60])+rhs[5]*(2*params.Q[73])+rhs[6]*(2*params.Q[86])+rhs[7]*(2*params.Q[99])+rhs[8]*(2*params.Q[112])+rhs[9]*(2*params.Q[125])+rhs[10]*(2*params.Q[138])+rhs[11]*(2*params.Q[151])+rhs[12]*(2*params.Q[164]);
  lhs[9] = rhs[0]*(2*params.Q[9])+rhs[1]*(2*params.Q[22])+rhs[2]*(2*params.Q[35])+rhs[3]*(2*params.Q[48])+rhs[4]*(2*params.Q[61])+rhs[5]*(2*params.Q[74])+rhs[6]*(2*params.Q[87])+rhs[7]*(2*params.Q[100])+rhs[8]*(2*params.Q[113])+rhs[9]*(2*params.Q[126])+rhs[10]*(2*params.Q[139])+rhs[11]*(2*params.Q[152])+rhs[12]*(2*params.Q[165]);
  lhs[10] = rhs[0]*(2*params.Q[10])+rhs[1]*(2*params.Q[23])+rhs[2]*(2*params.Q[36])+rhs[3]*(2*params.Q[49])+rhs[4]*(2*params.Q[62])+rhs[5]*(2*params.Q[75])+rhs[6]*(2*params.Q[88])+rhs[7]*(2*params.Q[101])+rhs[8]*(2*params.Q[114])+rhs[9]*(2*params.Q[127])+rhs[10]*(2*params.Q[140])+rhs[11]*(2*params.Q[153])+rhs[12]*(2*params.Q[166]);
  lhs[11] = rhs[0]*(2*params.Q[11])+rhs[1]*(2*params.Q[24])+rhs[2]*(2*params.Q[37])+rhs[3]*(2*params.Q[50])+rhs[4]*(2*params.Q[63])+rhs[5]*(2*params.Q[76])+rhs[6]*(2*params.Q[89])+rhs[7]*(2*params.Q[102])+rhs[8]*(2*params.Q[115])+rhs[9]*(2*params.Q[128])+rhs[10]*(2*params.Q[141])+rhs[11]*(2*params.Q[154])+rhs[12]*(2*params.Q[167]);
  lhs[12] = rhs[0]*(2*params.Q[12])+rhs[1]*(2*params.Q[25])+rhs[2]*(2*params.Q[38])+rhs[3]*(2*params.Q[51])+rhs[4]*(2*params.Q[64])+rhs[5]*(2*params.Q[77])+rhs[6]*(2*params.Q[90])+rhs[7]*(2*params.Q[103])+rhs[8]*(2*params.Q[116])+rhs[9]*(2*params.Q[129])+rhs[10]*(2*params.Q[142])+rhs[11]*(2*params.Q[155])+rhs[12]*(2*params.Q[168]);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = 0;
  work.q[7] = 0;
  work.q[8] = 0;
  work.q[9] = 0;
  work.q[10] = 0;
  work.q[11] = 0;
  work.q[12] = 0;
}
void fillh(void) {
  work.h[0] = -0.0001;
  work.h[1] = -0.0001;
  work.h[2] = -0.0001;
  work.h[3] = -0.0001;
  work.h[4] = -0.0001;
  work.h[5] = -0.0001;
  work.h[6] = -0.0001;
  work.h[7] = -0.0001;
  work.h[8] = -0.0001;
  work.h[9] = -0.0001;
}
void fillb(void) {
}
void pre_ops(void) {
}
