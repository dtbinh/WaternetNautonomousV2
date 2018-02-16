/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 8];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 8 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 8 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 8 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 8 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 8 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 8 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 8 + 7];

acadoWorkspace.state[88] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[89] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.state[90] = acadoVariables.od[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 8] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 8 + 8];
acadoWorkspace.d[lRun1 * 8 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 8 + 9];
acadoWorkspace.d[lRun1 * 8 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 8 + 10];
acadoWorkspace.d[lRun1 * 8 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 8 + 11];
acadoWorkspace.d[lRun1 * 8 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 8 + 12];
acadoWorkspace.d[lRun1 * 8 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 8 + 13];
acadoWorkspace.d[lRun1 * 8 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 8 + 14];
acadoWorkspace.d[lRun1 * 8 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 8 + 15];

acadoWorkspace.evGx[lRun1 * 64] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 64 + 1] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 64 + 2] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 64 + 3] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 64 + 4] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 64 + 5] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 64 + 6] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 64 + 7] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 64 + 8] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 64 + 9] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 64 + 10] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 64 + 11] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 64 + 12] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 64 + 13] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 64 + 14] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 64 + 15] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 64 + 16] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 64 + 17] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 64 + 18] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 64 + 19] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 64 + 20] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 64 + 21] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 64 + 22] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 64 + 23] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 64 + 24] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 64 + 25] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 64 + 26] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 64 + 27] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 64 + 28] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 64 + 29] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 64 + 30] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 64 + 31] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 64 + 32] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 64 + 33] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 64 + 34] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 64 + 35] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 64 + 36] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 64 + 37] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 64 + 38] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 64 + 39] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 64 + 40] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 64 + 41] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 64 + 42] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 64 + 43] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 64 + 44] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 64 + 45] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 64 + 46] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 64 + 47] = acadoWorkspace.state[55];
acadoWorkspace.evGx[lRun1 * 64 + 48] = acadoWorkspace.state[56];
acadoWorkspace.evGx[lRun1 * 64 + 49] = acadoWorkspace.state[57];
acadoWorkspace.evGx[lRun1 * 64 + 50] = acadoWorkspace.state[58];
acadoWorkspace.evGx[lRun1 * 64 + 51] = acadoWorkspace.state[59];
acadoWorkspace.evGx[lRun1 * 64 + 52] = acadoWorkspace.state[60];
acadoWorkspace.evGx[lRun1 * 64 + 53] = acadoWorkspace.state[61];
acadoWorkspace.evGx[lRun1 * 64 + 54] = acadoWorkspace.state[62];
acadoWorkspace.evGx[lRun1 * 64 + 55] = acadoWorkspace.state[63];
acadoWorkspace.evGx[lRun1 * 64 + 56] = acadoWorkspace.state[64];
acadoWorkspace.evGx[lRun1 * 64 + 57] = acadoWorkspace.state[65];
acadoWorkspace.evGx[lRun1 * 64 + 58] = acadoWorkspace.state[66];
acadoWorkspace.evGx[lRun1 * 64 + 59] = acadoWorkspace.state[67];
acadoWorkspace.evGx[lRun1 * 64 + 60] = acadoWorkspace.state[68];
acadoWorkspace.evGx[lRun1 * 64 + 61] = acadoWorkspace.state[69];
acadoWorkspace.evGx[lRun1 * 64 + 62] = acadoWorkspace.state[70];
acadoWorkspace.evGx[lRun1 * 64 + 63] = acadoWorkspace.state[71];

acadoWorkspace.evGu[lRun1 * 16] = acadoWorkspace.state[72];
acadoWorkspace.evGu[lRun1 * 16 + 1] = acadoWorkspace.state[73];
acadoWorkspace.evGu[lRun1 * 16 + 2] = acadoWorkspace.state[74];
acadoWorkspace.evGu[lRun1 * 16 + 3] = acadoWorkspace.state[75];
acadoWorkspace.evGu[lRun1 * 16 + 4] = acadoWorkspace.state[76];
acadoWorkspace.evGu[lRun1 * 16 + 5] = acadoWorkspace.state[77];
acadoWorkspace.evGu[lRun1 * 16 + 6] = acadoWorkspace.state[78];
acadoWorkspace.evGu[lRun1 * 16 + 7] = acadoWorkspace.state[79];
acadoWorkspace.evGu[lRun1 * 16 + 8] = acadoWorkspace.state[80];
acadoWorkspace.evGu[lRun1 * 16 + 9] = acadoWorkspace.state[81];
acadoWorkspace.evGu[lRun1 * 16 + 10] = acadoWorkspace.state[82];
acadoWorkspace.evGu[lRun1 * 16 + 11] = acadoWorkspace.state[83];
acadoWorkspace.evGu[lRun1 * 16 + 12] = acadoWorkspace.state[84];
acadoWorkspace.evGu[lRun1 * 16 + 13] = acadoWorkspace.state[85];
acadoWorkspace.evGu[lRun1 * 16 + 14] = acadoWorkspace.state[86];
acadoWorkspace.evGu[lRun1 * 16 + 15] = acadoWorkspace.state[87];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 8;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = u[0];
out[9] = u[1];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 8];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 8 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 8 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 8 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 8 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 8 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 8 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 8 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[9] = acadoVariables.u[runObj * 2 + 1];
acadoWorkspace.objValueIn[10] = acadoVariables.od[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 10] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 10 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 10 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 10 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 10 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 10 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 10 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 10 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 10 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 10 + 9] = acadoWorkspace.objValueOut[9];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acadoWorkspace.objValueIn[4] = acadoVariables.x[84];
acadoWorkspace.objValueIn[5] = acadoVariables.x[85];
acadoWorkspace.objValueIn[6] = acadoVariables.x[86];
acadoWorkspace.objValueIn[7] = acadoVariables.x[87];
acadoWorkspace.objValueIn[8] = acadoVariables.od[10];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7];
dNew[1] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3] + Gx1[12]*dOld[4] + Gx1[13]*dOld[5] + Gx1[14]*dOld[6] + Gx1[15]*dOld[7];
dNew[2] += + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7];
dNew[3] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7];
dNew[4] += + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7];
dNew[5] += + Gx1[40]*dOld[0] + Gx1[41]*dOld[1] + Gx1[42]*dOld[2] + Gx1[43]*dOld[3] + Gx1[44]*dOld[4] + Gx1[45]*dOld[5] + Gx1[46]*dOld[6] + Gx1[47]*dOld[7];
dNew[6] += + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7];
dNew[7] += + Gx1[56]*dOld[0] + Gx1[57]*dOld[1] + Gx1[58]*dOld[2] + Gx1[59]*dOld[3] + Gx1[60]*dOld[4] + Gx1[61]*dOld[5] + Gx1[62]*dOld[6] + Gx1[63]*dOld[7];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
Gx2[49] = Gx1[49];
Gx2[50] = Gx1[50];
Gx2[51] = Gx1[51];
Gx2[52] = Gx1[52];
Gx2[53] = Gx1[53];
Gx2[54] = Gx1[54];
Gx2[55] = Gx1[55];
Gx2[56] = Gx1[56];
Gx2[57] = Gx1[57];
Gx2[58] = Gx1[58];
Gx2[59] = Gx1[59];
Gx2[60] = Gx1[60];
Gx2[61] = Gx1[61];
Gx2[62] = Gx1[62];
Gx2[63] = Gx1[63];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[24] + Gx1[4]*Gx2[32] + Gx1[5]*Gx2[40] + Gx1[6]*Gx2[48] + Gx1[7]*Gx2[56];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[25] + Gx1[4]*Gx2[33] + Gx1[5]*Gx2[41] + Gx1[6]*Gx2[49] + Gx1[7]*Gx2[57];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[18] + Gx1[3]*Gx2[26] + Gx1[4]*Gx2[34] + Gx1[5]*Gx2[42] + Gx1[6]*Gx2[50] + Gx1[7]*Gx2[58];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[19] + Gx1[3]*Gx2[27] + Gx1[4]*Gx2[35] + Gx1[5]*Gx2[43] + Gx1[6]*Gx2[51] + Gx1[7]*Gx2[59];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[28] + Gx1[4]*Gx2[36] + Gx1[5]*Gx2[44] + Gx1[6]*Gx2[52] + Gx1[7]*Gx2[60];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[21] + Gx1[3]*Gx2[29] + Gx1[4]*Gx2[37] + Gx1[5]*Gx2[45] + Gx1[6]*Gx2[53] + Gx1[7]*Gx2[61];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[22] + Gx1[3]*Gx2[30] + Gx1[4]*Gx2[38] + Gx1[5]*Gx2[46] + Gx1[6]*Gx2[54] + Gx1[7]*Gx2[62];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[23] + Gx1[3]*Gx2[31] + Gx1[4]*Gx2[39] + Gx1[5]*Gx2[47] + Gx1[6]*Gx2[55] + Gx1[7]*Gx2[63];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[8] + Gx1[10]*Gx2[16] + Gx1[11]*Gx2[24] + Gx1[12]*Gx2[32] + Gx1[13]*Gx2[40] + Gx1[14]*Gx2[48] + Gx1[15]*Gx2[56];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[9] + Gx1[10]*Gx2[17] + Gx1[11]*Gx2[25] + Gx1[12]*Gx2[33] + Gx1[13]*Gx2[41] + Gx1[14]*Gx2[49] + Gx1[15]*Gx2[57];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[10] + Gx1[10]*Gx2[18] + Gx1[11]*Gx2[26] + Gx1[12]*Gx2[34] + Gx1[13]*Gx2[42] + Gx1[14]*Gx2[50] + Gx1[15]*Gx2[58];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[11] + Gx1[10]*Gx2[19] + Gx1[11]*Gx2[27] + Gx1[12]*Gx2[35] + Gx1[13]*Gx2[43] + Gx1[14]*Gx2[51] + Gx1[15]*Gx2[59];
Gx3[12] = + Gx1[8]*Gx2[4] + Gx1[9]*Gx2[12] + Gx1[10]*Gx2[20] + Gx1[11]*Gx2[28] + Gx1[12]*Gx2[36] + Gx1[13]*Gx2[44] + Gx1[14]*Gx2[52] + Gx1[15]*Gx2[60];
Gx3[13] = + Gx1[8]*Gx2[5] + Gx1[9]*Gx2[13] + Gx1[10]*Gx2[21] + Gx1[11]*Gx2[29] + Gx1[12]*Gx2[37] + Gx1[13]*Gx2[45] + Gx1[14]*Gx2[53] + Gx1[15]*Gx2[61];
Gx3[14] = + Gx1[8]*Gx2[6] + Gx1[9]*Gx2[14] + Gx1[10]*Gx2[22] + Gx1[11]*Gx2[30] + Gx1[12]*Gx2[38] + Gx1[13]*Gx2[46] + Gx1[14]*Gx2[54] + Gx1[15]*Gx2[62];
Gx3[15] = + Gx1[8]*Gx2[7] + Gx1[9]*Gx2[15] + Gx1[10]*Gx2[23] + Gx1[11]*Gx2[31] + Gx1[12]*Gx2[39] + Gx1[13]*Gx2[47] + Gx1[14]*Gx2[55] + Gx1[15]*Gx2[63];
Gx3[16] = + Gx1[16]*Gx2[0] + Gx1[17]*Gx2[8] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[24] + Gx1[20]*Gx2[32] + Gx1[21]*Gx2[40] + Gx1[22]*Gx2[48] + Gx1[23]*Gx2[56];
Gx3[17] = + Gx1[16]*Gx2[1] + Gx1[17]*Gx2[9] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[25] + Gx1[20]*Gx2[33] + Gx1[21]*Gx2[41] + Gx1[22]*Gx2[49] + Gx1[23]*Gx2[57];
Gx3[18] = + Gx1[16]*Gx2[2] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[26] + Gx1[20]*Gx2[34] + Gx1[21]*Gx2[42] + Gx1[22]*Gx2[50] + Gx1[23]*Gx2[58];
Gx3[19] = + Gx1[16]*Gx2[3] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[27] + Gx1[20]*Gx2[35] + Gx1[21]*Gx2[43] + Gx1[22]*Gx2[51] + Gx1[23]*Gx2[59];
Gx3[20] = + Gx1[16]*Gx2[4] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[20] + Gx1[19]*Gx2[28] + Gx1[20]*Gx2[36] + Gx1[21]*Gx2[44] + Gx1[22]*Gx2[52] + Gx1[23]*Gx2[60];
Gx3[21] = + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[21] + Gx1[19]*Gx2[29] + Gx1[20]*Gx2[37] + Gx1[21]*Gx2[45] + Gx1[22]*Gx2[53] + Gx1[23]*Gx2[61];
Gx3[22] = + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[22] + Gx1[19]*Gx2[30] + Gx1[20]*Gx2[38] + Gx1[21]*Gx2[46] + Gx1[22]*Gx2[54] + Gx1[23]*Gx2[62];
Gx3[23] = + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[15] + Gx1[18]*Gx2[23] + Gx1[19]*Gx2[31] + Gx1[20]*Gx2[39] + Gx1[21]*Gx2[47] + Gx1[22]*Gx2[55] + Gx1[23]*Gx2[63];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[24] + Gx1[28]*Gx2[32] + Gx1[29]*Gx2[40] + Gx1[30]*Gx2[48] + Gx1[31]*Gx2[56];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[25] + Gx1[28]*Gx2[33] + Gx1[29]*Gx2[41] + Gx1[30]*Gx2[49] + Gx1[31]*Gx2[57];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[18] + Gx1[27]*Gx2[26] + Gx1[28]*Gx2[34] + Gx1[29]*Gx2[42] + Gx1[30]*Gx2[50] + Gx1[31]*Gx2[58];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[19] + Gx1[27]*Gx2[27] + Gx1[28]*Gx2[35] + Gx1[29]*Gx2[43] + Gx1[30]*Gx2[51] + Gx1[31]*Gx2[59];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[12] + Gx1[26]*Gx2[20] + Gx1[27]*Gx2[28] + Gx1[28]*Gx2[36] + Gx1[29]*Gx2[44] + Gx1[30]*Gx2[52] + Gx1[31]*Gx2[60];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[13] + Gx1[26]*Gx2[21] + Gx1[27]*Gx2[29] + Gx1[28]*Gx2[37] + Gx1[29]*Gx2[45] + Gx1[30]*Gx2[53] + Gx1[31]*Gx2[61];
Gx3[30] = + Gx1[24]*Gx2[6] + Gx1[25]*Gx2[14] + Gx1[26]*Gx2[22] + Gx1[27]*Gx2[30] + Gx1[28]*Gx2[38] + Gx1[29]*Gx2[46] + Gx1[30]*Gx2[54] + Gx1[31]*Gx2[62];
Gx3[31] = + Gx1[24]*Gx2[7] + Gx1[25]*Gx2[15] + Gx1[26]*Gx2[23] + Gx1[27]*Gx2[31] + Gx1[28]*Gx2[39] + Gx1[29]*Gx2[47] + Gx1[30]*Gx2[55] + Gx1[31]*Gx2[63];
Gx3[32] = + Gx1[32]*Gx2[0] + Gx1[33]*Gx2[8] + Gx1[34]*Gx2[16] + Gx1[35]*Gx2[24] + Gx1[36]*Gx2[32] + Gx1[37]*Gx2[40] + Gx1[38]*Gx2[48] + Gx1[39]*Gx2[56];
Gx3[33] = + Gx1[32]*Gx2[1] + Gx1[33]*Gx2[9] + Gx1[34]*Gx2[17] + Gx1[35]*Gx2[25] + Gx1[36]*Gx2[33] + Gx1[37]*Gx2[41] + Gx1[38]*Gx2[49] + Gx1[39]*Gx2[57];
Gx3[34] = + Gx1[32]*Gx2[2] + Gx1[33]*Gx2[10] + Gx1[34]*Gx2[18] + Gx1[35]*Gx2[26] + Gx1[36]*Gx2[34] + Gx1[37]*Gx2[42] + Gx1[38]*Gx2[50] + Gx1[39]*Gx2[58];
Gx3[35] = + Gx1[32]*Gx2[3] + Gx1[33]*Gx2[11] + Gx1[34]*Gx2[19] + Gx1[35]*Gx2[27] + Gx1[36]*Gx2[35] + Gx1[37]*Gx2[43] + Gx1[38]*Gx2[51] + Gx1[39]*Gx2[59];
Gx3[36] = + Gx1[32]*Gx2[4] + Gx1[33]*Gx2[12] + Gx1[34]*Gx2[20] + Gx1[35]*Gx2[28] + Gx1[36]*Gx2[36] + Gx1[37]*Gx2[44] + Gx1[38]*Gx2[52] + Gx1[39]*Gx2[60];
Gx3[37] = + Gx1[32]*Gx2[5] + Gx1[33]*Gx2[13] + Gx1[34]*Gx2[21] + Gx1[35]*Gx2[29] + Gx1[36]*Gx2[37] + Gx1[37]*Gx2[45] + Gx1[38]*Gx2[53] + Gx1[39]*Gx2[61];
Gx3[38] = + Gx1[32]*Gx2[6] + Gx1[33]*Gx2[14] + Gx1[34]*Gx2[22] + Gx1[35]*Gx2[30] + Gx1[36]*Gx2[38] + Gx1[37]*Gx2[46] + Gx1[38]*Gx2[54] + Gx1[39]*Gx2[62];
Gx3[39] = + Gx1[32]*Gx2[7] + Gx1[33]*Gx2[15] + Gx1[34]*Gx2[23] + Gx1[35]*Gx2[31] + Gx1[36]*Gx2[39] + Gx1[37]*Gx2[47] + Gx1[38]*Gx2[55] + Gx1[39]*Gx2[63];
Gx3[40] = + Gx1[40]*Gx2[0] + Gx1[41]*Gx2[8] + Gx1[42]*Gx2[16] + Gx1[43]*Gx2[24] + Gx1[44]*Gx2[32] + Gx1[45]*Gx2[40] + Gx1[46]*Gx2[48] + Gx1[47]*Gx2[56];
Gx3[41] = + Gx1[40]*Gx2[1] + Gx1[41]*Gx2[9] + Gx1[42]*Gx2[17] + Gx1[43]*Gx2[25] + Gx1[44]*Gx2[33] + Gx1[45]*Gx2[41] + Gx1[46]*Gx2[49] + Gx1[47]*Gx2[57];
Gx3[42] = + Gx1[40]*Gx2[2] + Gx1[41]*Gx2[10] + Gx1[42]*Gx2[18] + Gx1[43]*Gx2[26] + Gx1[44]*Gx2[34] + Gx1[45]*Gx2[42] + Gx1[46]*Gx2[50] + Gx1[47]*Gx2[58];
Gx3[43] = + Gx1[40]*Gx2[3] + Gx1[41]*Gx2[11] + Gx1[42]*Gx2[19] + Gx1[43]*Gx2[27] + Gx1[44]*Gx2[35] + Gx1[45]*Gx2[43] + Gx1[46]*Gx2[51] + Gx1[47]*Gx2[59];
Gx3[44] = + Gx1[40]*Gx2[4] + Gx1[41]*Gx2[12] + Gx1[42]*Gx2[20] + Gx1[43]*Gx2[28] + Gx1[44]*Gx2[36] + Gx1[45]*Gx2[44] + Gx1[46]*Gx2[52] + Gx1[47]*Gx2[60];
Gx3[45] = + Gx1[40]*Gx2[5] + Gx1[41]*Gx2[13] + Gx1[42]*Gx2[21] + Gx1[43]*Gx2[29] + Gx1[44]*Gx2[37] + Gx1[45]*Gx2[45] + Gx1[46]*Gx2[53] + Gx1[47]*Gx2[61];
Gx3[46] = + Gx1[40]*Gx2[6] + Gx1[41]*Gx2[14] + Gx1[42]*Gx2[22] + Gx1[43]*Gx2[30] + Gx1[44]*Gx2[38] + Gx1[45]*Gx2[46] + Gx1[46]*Gx2[54] + Gx1[47]*Gx2[62];
Gx3[47] = + Gx1[40]*Gx2[7] + Gx1[41]*Gx2[15] + Gx1[42]*Gx2[23] + Gx1[43]*Gx2[31] + Gx1[44]*Gx2[39] + Gx1[45]*Gx2[47] + Gx1[46]*Gx2[55] + Gx1[47]*Gx2[63];
Gx3[48] = + Gx1[48]*Gx2[0] + Gx1[49]*Gx2[8] + Gx1[50]*Gx2[16] + Gx1[51]*Gx2[24] + Gx1[52]*Gx2[32] + Gx1[53]*Gx2[40] + Gx1[54]*Gx2[48] + Gx1[55]*Gx2[56];
Gx3[49] = + Gx1[48]*Gx2[1] + Gx1[49]*Gx2[9] + Gx1[50]*Gx2[17] + Gx1[51]*Gx2[25] + Gx1[52]*Gx2[33] + Gx1[53]*Gx2[41] + Gx1[54]*Gx2[49] + Gx1[55]*Gx2[57];
Gx3[50] = + Gx1[48]*Gx2[2] + Gx1[49]*Gx2[10] + Gx1[50]*Gx2[18] + Gx1[51]*Gx2[26] + Gx1[52]*Gx2[34] + Gx1[53]*Gx2[42] + Gx1[54]*Gx2[50] + Gx1[55]*Gx2[58];
Gx3[51] = + Gx1[48]*Gx2[3] + Gx1[49]*Gx2[11] + Gx1[50]*Gx2[19] + Gx1[51]*Gx2[27] + Gx1[52]*Gx2[35] + Gx1[53]*Gx2[43] + Gx1[54]*Gx2[51] + Gx1[55]*Gx2[59];
Gx3[52] = + Gx1[48]*Gx2[4] + Gx1[49]*Gx2[12] + Gx1[50]*Gx2[20] + Gx1[51]*Gx2[28] + Gx1[52]*Gx2[36] + Gx1[53]*Gx2[44] + Gx1[54]*Gx2[52] + Gx1[55]*Gx2[60];
Gx3[53] = + Gx1[48]*Gx2[5] + Gx1[49]*Gx2[13] + Gx1[50]*Gx2[21] + Gx1[51]*Gx2[29] + Gx1[52]*Gx2[37] + Gx1[53]*Gx2[45] + Gx1[54]*Gx2[53] + Gx1[55]*Gx2[61];
Gx3[54] = + Gx1[48]*Gx2[6] + Gx1[49]*Gx2[14] + Gx1[50]*Gx2[22] + Gx1[51]*Gx2[30] + Gx1[52]*Gx2[38] + Gx1[53]*Gx2[46] + Gx1[54]*Gx2[54] + Gx1[55]*Gx2[62];
Gx3[55] = + Gx1[48]*Gx2[7] + Gx1[49]*Gx2[15] + Gx1[50]*Gx2[23] + Gx1[51]*Gx2[31] + Gx1[52]*Gx2[39] + Gx1[53]*Gx2[47] + Gx1[54]*Gx2[55] + Gx1[55]*Gx2[63];
Gx3[56] = + Gx1[56]*Gx2[0] + Gx1[57]*Gx2[8] + Gx1[58]*Gx2[16] + Gx1[59]*Gx2[24] + Gx1[60]*Gx2[32] + Gx1[61]*Gx2[40] + Gx1[62]*Gx2[48] + Gx1[63]*Gx2[56];
Gx3[57] = + Gx1[56]*Gx2[1] + Gx1[57]*Gx2[9] + Gx1[58]*Gx2[17] + Gx1[59]*Gx2[25] + Gx1[60]*Gx2[33] + Gx1[61]*Gx2[41] + Gx1[62]*Gx2[49] + Gx1[63]*Gx2[57];
Gx3[58] = + Gx1[56]*Gx2[2] + Gx1[57]*Gx2[10] + Gx1[58]*Gx2[18] + Gx1[59]*Gx2[26] + Gx1[60]*Gx2[34] + Gx1[61]*Gx2[42] + Gx1[62]*Gx2[50] + Gx1[63]*Gx2[58];
Gx3[59] = + Gx1[56]*Gx2[3] + Gx1[57]*Gx2[11] + Gx1[58]*Gx2[19] + Gx1[59]*Gx2[27] + Gx1[60]*Gx2[35] + Gx1[61]*Gx2[43] + Gx1[62]*Gx2[51] + Gx1[63]*Gx2[59];
Gx3[60] = + Gx1[56]*Gx2[4] + Gx1[57]*Gx2[12] + Gx1[58]*Gx2[20] + Gx1[59]*Gx2[28] + Gx1[60]*Gx2[36] + Gx1[61]*Gx2[44] + Gx1[62]*Gx2[52] + Gx1[63]*Gx2[60];
Gx3[61] = + Gx1[56]*Gx2[5] + Gx1[57]*Gx2[13] + Gx1[58]*Gx2[21] + Gx1[59]*Gx2[29] + Gx1[60]*Gx2[37] + Gx1[61]*Gx2[45] + Gx1[62]*Gx2[53] + Gx1[63]*Gx2[61];
Gx3[62] = + Gx1[56]*Gx2[6] + Gx1[57]*Gx2[14] + Gx1[58]*Gx2[22] + Gx1[59]*Gx2[30] + Gx1[60]*Gx2[38] + Gx1[61]*Gx2[46] + Gx1[62]*Gx2[54] + Gx1[63]*Gx2[62];
Gx3[63] = + Gx1[56]*Gx2[7] + Gx1[57]*Gx2[15] + Gx1[58]*Gx2[23] + Gx1[59]*Gx2[31] + Gx1[60]*Gx2[39] + Gx1[61]*Gx2[47] + Gx1[62]*Gx2[55] + Gx1[63]*Gx2[63];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8] + Gx1[5]*Gu1[10] + Gx1[6]*Gu1[12] + Gx1[7]*Gu1[14];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9] + Gx1[5]*Gu1[11] + Gx1[6]*Gu1[13] + Gx1[7]*Gu1[15];
Gu2[2] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[8] + Gx1[13]*Gu1[10] + Gx1[14]*Gu1[12] + Gx1[15]*Gu1[14];
Gu2[3] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[11] + Gx1[14]*Gu1[13] + Gx1[15]*Gu1[15];
Gu2[4] = + Gx1[16]*Gu1[0] + Gx1[17]*Gu1[2] + Gx1[18]*Gu1[4] + Gx1[19]*Gu1[6] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[14];
Gu2[5] = + Gx1[16]*Gu1[1] + Gx1[17]*Gu1[3] + Gx1[18]*Gu1[5] + Gx1[19]*Gu1[7] + Gx1[20]*Gu1[9] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[15];
Gu2[6] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[2] + Gx1[26]*Gu1[4] + Gx1[27]*Gu1[6] + Gx1[28]*Gu1[8] + Gx1[29]*Gu1[10] + Gx1[30]*Gu1[12] + Gx1[31]*Gu1[14];
Gu2[7] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[3] + Gx1[26]*Gu1[5] + Gx1[27]*Gu1[7] + Gx1[28]*Gu1[9] + Gx1[29]*Gu1[11] + Gx1[30]*Gu1[13] + Gx1[31]*Gu1[15];
Gu2[8] = + Gx1[32]*Gu1[0] + Gx1[33]*Gu1[2] + Gx1[34]*Gu1[4] + Gx1[35]*Gu1[6] + Gx1[36]*Gu1[8] + Gx1[37]*Gu1[10] + Gx1[38]*Gu1[12] + Gx1[39]*Gu1[14];
Gu2[9] = + Gx1[32]*Gu1[1] + Gx1[33]*Gu1[3] + Gx1[34]*Gu1[5] + Gx1[35]*Gu1[7] + Gx1[36]*Gu1[9] + Gx1[37]*Gu1[11] + Gx1[38]*Gu1[13] + Gx1[39]*Gu1[15];
Gu2[10] = + Gx1[40]*Gu1[0] + Gx1[41]*Gu1[2] + Gx1[42]*Gu1[4] + Gx1[43]*Gu1[6] + Gx1[44]*Gu1[8] + Gx1[45]*Gu1[10] + Gx1[46]*Gu1[12] + Gx1[47]*Gu1[14];
Gu2[11] = + Gx1[40]*Gu1[1] + Gx1[41]*Gu1[3] + Gx1[42]*Gu1[5] + Gx1[43]*Gu1[7] + Gx1[44]*Gu1[9] + Gx1[45]*Gu1[11] + Gx1[46]*Gu1[13] + Gx1[47]*Gu1[15];
Gu2[12] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[2] + Gx1[50]*Gu1[4] + Gx1[51]*Gu1[6] + Gx1[52]*Gu1[8] + Gx1[53]*Gu1[10] + Gx1[54]*Gu1[12] + Gx1[55]*Gu1[14];
Gu2[13] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[3] + Gx1[50]*Gu1[5] + Gx1[51]*Gu1[7] + Gx1[52]*Gu1[9] + Gx1[53]*Gu1[11] + Gx1[54]*Gu1[13] + Gx1[55]*Gu1[15];
Gu2[14] = + Gx1[56]*Gu1[0] + Gx1[57]*Gu1[2] + Gx1[58]*Gu1[4] + Gx1[59]*Gu1[6] + Gx1[60]*Gu1[8] + Gx1[61]*Gu1[10] + Gx1[62]*Gu1[12] + Gx1[63]*Gu1[14];
Gu2[15] = + Gx1[56]*Gu1[1] + Gx1[57]*Gu1[3] + Gx1[58]*Gu1[5] + Gx1[59]*Gu1[7] + Gx1[60]*Gu1[9] + Gx1[61]*Gu1[11] + Gx1[62]*Gu1[13] + Gx1[63]*Gu1[15];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10] + Gu1[12]*Gu2[12] + Gu1[14]*Gu2[14];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11] + Gu1[12]*Gu2[13] + Gu1[14]*Gu2[15];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10] + Gu1[13]*Gu2[12] + Gu1[15]*Gu2[14];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11] + Gu1[13]*Gu2[13] + Gu1[15]*Gu2[15];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = (real_t)1.0000000000000000e-03;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = (real_t)1.0000000000000000e-03;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)2.5000000000000000e+02*dOld[0];
dNew[1] = + (real_t)2.5000000000000000e+02*dOld[1];
dNew[2] = + (real_t)9.9999999999999995e-07*dOld[2];
dNew[3] = + (real_t)9.9999999999999995e-07*dOld[3];
dNew[4] = + (real_t)9.9999999999999995e-07*dOld[4];
dNew[5] = + (real_t)9.9999999999999995e-07*dOld[5];
dNew[6] = + (real_t)9.9999999999999995e-07*dOld[6];
dNew[7] = + (real_t)9.9999999999999995e-07*dOld[7];
}

void acado_multQN1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)9.9999999999999995e-07*dOld[0];
dNew[1] = + (real_t)9.9999999999999995e-07*dOld[1];
dNew[2] = + (real_t)9.9999999999999995e-07*dOld[2];
dNew[3] = + (real_t)9.9999999999999995e-07*dOld[3];
dNew[4] = + (real_t)9.9999999999999995e-07*dOld[4];
dNew[5] = + (real_t)9.9999999999999995e-07*dOld[5];
dNew[6] = + (real_t)9.9999999999999995e-07*dOld[6];
dNew[7] = + (real_t)9.9999999999999995e-07*dOld[7];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + (real_t)1.0000000000000000e-03*Dy1[8];
RDy1[1] = + (real_t)1.0000000000000000e-03*Dy1[9];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)2.5000000000000000e+02*Dy1[0];
QDy1[1] = + (real_t)2.5000000000000000e+02*Dy1[1];
QDy1[2] = + (real_t)9.9999999999999995e-07*Dy1[2];
QDy1[3] = + (real_t)9.9999999999999995e-07*Dy1[3];
QDy1[4] = + (real_t)9.9999999999999995e-07*Dy1[4];
QDy1[5] = + (real_t)9.9999999999999995e-07*Dy1[5];
QDy1[6] = + (real_t)9.9999999999999995e-07*Dy1[6];
QDy1[7] = + (real_t)9.9999999999999995e-07*Dy1[7];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4] + E1[10]*QDy1[5] + E1[12]*QDy1[6] + E1[14]*QDy1[7];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4] + E1[11]*QDy1[5] + E1[13]*QDy1[6] + E1[15]*QDy1[7];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[8] + E1[4]*Gx1[16] + E1[6]*Gx1[24] + E1[8]*Gx1[32] + E1[10]*Gx1[40] + E1[12]*Gx1[48] + E1[14]*Gx1[56];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[9] + E1[4]*Gx1[17] + E1[6]*Gx1[25] + E1[8]*Gx1[33] + E1[10]*Gx1[41] + E1[12]*Gx1[49] + E1[14]*Gx1[57];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[10] + E1[4]*Gx1[18] + E1[6]*Gx1[26] + E1[8]*Gx1[34] + E1[10]*Gx1[42] + E1[12]*Gx1[50] + E1[14]*Gx1[58];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[11] + E1[4]*Gx1[19] + E1[6]*Gx1[27] + E1[8]*Gx1[35] + E1[10]*Gx1[43] + E1[12]*Gx1[51] + E1[14]*Gx1[59];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[12] + E1[4]*Gx1[20] + E1[6]*Gx1[28] + E1[8]*Gx1[36] + E1[10]*Gx1[44] + E1[12]*Gx1[52] + E1[14]*Gx1[60];
H101[5] += + E1[0]*Gx1[5] + E1[2]*Gx1[13] + E1[4]*Gx1[21] + E1[6]*Gx1[29] + E1[8]*Gx1[37] + E1[10]*Gx1[45] + E1[12]*Gx1[53] + E1[14]*Gx1[61];
H101[6] += + E1[0]*Gx1[6] + E1[2]*Gx1[14] + E1[4]*Gx1[22] + E1[6]*Gx1[30] + E1[8]*Gx1[38] + E1[10]*Gx1[46] + E1[12]*Gx1[54] + E1[14]*Gx1[62];
H101[7] += + E1[0]*Gx1[7] + E1[2]*Gx1[15] + E1[4]*Gx1[23] + E1[6]*Gx1[31] + E1[8]*Gx1[39] + E1[10]*Gx1[47] + E1[12]*Gx1[55] + E1[14]*Gx1[63];
H101[8] += + E1[1]*Gx1[0] + E1[3]*Gx1[8] + E1[5]*Gx1[16] + E1[7]*Gx1[24] + E1[9]*Gx1[32] + E1[11]*Gx1[40] + E1[13]*Gx1[48] + E1[15]*Gx1[56];
H101[9] += + E1[1]*Gx1[1] + E1[3]*Gx1[9] + E1[5]*Gx1[17] + E1[7]*Gx1[25] + E1[9]*Gx1[33] + E1[11]*Gx1[41] + E1[13]*Gx1[49] + E1[15]*Gx1[57];
H101[10] += + E1[1]*Gx1[2] + E1[3]*Gx1[10] + E1[5]*Gx1[18] + E1[7]*Gx1[26] + E1[9]*Gx1[34] + E1[11]*Gx1[42] + E1[13]*Gx1[50] + E1[15]*Gx1[58];
H101[11] += + E1[1]*Gx1[3] + E1[3]*Gx1[11] + E1[5]*Gx1[19] + E1[7]*Gx1[27] + E1[9]*Gx1[35] + E1[11]*Gx1[43] + E1[13]*Gx1[51] + E1[15]*Gx1[59];
H101[12] += + E1[1]*Gx1[4] + E1[3]*Gx1[12] + E1[5]*Gx1[20] + E1[7]*Gx1[28] + E1[9]*Gx1[36] + E1[11]*Gx1[44] + E1[13]*Gx1[52] + E1[15]*Gx1[60];
H101[13] += + E1[1]*Gx1[5] + E1[3]*Gx1[13] + E1[5]*Gx1[21] + E1[7]*Gx1[29] + E1[9]*Gx1[37] + E1[11]*Gx1[45] + E1[13]*Gx1[53] + E1[15]*Gx1[61];
H101[14] += + E1[1]*Gx1[6] + E1[3]*Gx1[14] + E1[5]*Gx1[22] + E1[7]*Gx1[30] + E1[9]*Gx1[38] + E1[11]*Gx1[46] + E1[13]*Gx1[54] + E1[15]*Gx1[62];
H101[15] += + E1[1]*Gx1[7] + E1[3]*Gx1[15] + E1[5]*Gx1[23] + E1[7]*Gx1[31] + E1[9]*Gx1[39] + E1[11]*Gx1[47] + E1[13]*Gx1[55] + E1[15]*Gx1[63];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 16; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
dNew[5] += + E1[10]*U1[0] + E1[11]*U1[1];
dNew[6] += + E1[12]*U1[0] + E1[13]*U1[1];
dNew[7] += + E1[14]*U1[0] + E1[15]*U1[1];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)2.5000000000000000e+02*Gx1[0];
Gx2[1] = + (real_t)2.5000000000000000e+02*Gx1[1];
Gx2[2] = + (real_t)2.5000000000000000e+02*Gx1[2];
Gx2[3] = + (real_t)2.5000000000000000e+02*Gx1[3];
Gx2[4] = + (real_t)2.5000000000000000e+02*Gx1[4];
Gx2[5] = + (real_t)2.5000000000000000e+02*Gx1[5];
Gx2[6] = + (real_t)2.5000000000000000e+02*Gx1[6];
Gx2[7] = + (real_t)2.5000000000000000e+02*Gx1[7];
Gx2[8] = + (real_t)2.5000000000000000e+02*Gx1[8];
Gx2[9] = + (real_t)2.5000000000000000e+02*Gx1[9];
Gx2[10] = + (real_t)2.5000000000000000e+02*Gx1[10];
Gx2[11] = + (real_t)2.5000000000000000e+02*Gx1[11];
Gx2[12] = + (real_t)2.5000000000000000e+02*Gx1[12];
Gx2[13] = + (real_t)2.5000000000000000e+02*Gx1[13];
Gx2[14] = + (real_t)2.5000000000000000e+02*Gx1[14];
Gx2[15] = + (real_t)2.5000000000000000e+02*Gx1[15];
Gx2[16] = + (real_t)9.9999999999999995e-07*Gx1[16];
Gx2[17] = + (real_t)9.9999999999999995e-07*Gx1[17];
Gx2[18] = + (real_t)9.9999999999999995e-07*Gx1[18];
Gx2[19] = + (real_t)9.9999999999999995e-07*Gx1[19];
Gx2[20] = + (real_t)9.9999999999999995e-07*Gx1[20];
Gx2[21] = + (real_t)9.9999999999999995e-07*Gx1[21];
Gx2[22] = + (real_t)9.9999999999999995e-07*Gx1[22];
Gx2[23] = + (real_t)9.9999999999999995e-07*Gx1[23];
Gx2[24] = + (real_t)9.9999999999999995e-07*Gx1[24];
Gx2[25] = + (real_t)9.9999999999999995e-07*Gx1[25];
Gx2[26] = + (real_t)9.9999999999999995e-07*Gx1[26];
Gx2[27] = + (real_t)9.9999999999999995e-07*Gx1[27];
Gx2[28] = + (real_t)9.9999999999999995e-07*Gx1[28];
Gx2[29] = + (real_t)9.9999999999999995e-07*Gx1[29];
Gx2[30] = + (real_t)9.9999999999999995e-07*Gx1[30];
Gx2[31] = + (real_t)9.9999999999999995e-07*Gx1[31];
Gx2[32] = + (real_t)9.9999999999999995e-07*Gx1[32];
Gx2[33] = + (real_t)9.9999999999999995e-07*Gx1[33];
Gx2[34] = + (real_t)9.9999999999999995e-07*Gx1[34];
Gx2[35] = + (real_t)9.9999999999999995e-07*Gx1[35];
Gx2[36] = + (real_t)9.9999999999999995e-07*Gx1[36];
Gx2[37] = + (real_t)9.9999999999999995e-07*Gx1[37];
Gx2[38] = + (real_t)9.9999999999999995e-07*Gx1[38];
Gx2[39] = + (real_t)9.9999999999999995e-07*Gx1[39];
Gx2[40] = + (real_t)9.9999999999999995e-07*Gx1[40];
Gx2[41] = + (real_t)9.9999999999999995e-07*Gx1[41];
Gx2[42] = + (real_t)9.9999999999999995e-07*Gx1[42];
Gx2[43] = + (real_t)9.9999999999999995e-07*Gx1[43];
Gx2[44] = + (real_t)9.9999999999999995e-07*Gx1[44];
Gx2[45] = + (real_t)9.9999999999999995e-07*Gx1[45];
Gx2[46] = + (real_t)9.9999999999999995e-07*Gx1[46];
Gx2[47] = + (real_t)9.9999999999999995e-07*Gx1[47];
Gx2[48] = + (real_t)9.9999999999999995e-07*Gx1[48];
Gx2[49] = + (real_t)9.9999999999999995e-07*Gx1[49];
Gx2[50] = + (real_t)9.9999999999999995e-07*Gx1[50];
Gx2[51] = + (real_t)9.9999999999999995e-07*Gx1[51];
Gx2[52] = + (real_t)9.9999999999999995e-07*Gx1[52];
Gx2[53] = + (real_t)9.9999999999999995e-07*Gx1[53];
Gx2[54] = + (real_t)9.9999999999999995e-07*Gx1[54];
Gx2[55] = + (real_t)9.9999999999999995e-07*Gx1[55];
Gx2[56] = + (real_t)9.9999999999999995e-07*Gx1[56];
Gx2[57] = + (real_t)9.9999999999999995e-07*Gx1[57];
Gx2[58] = + (real_t)9.9999999999999995e-07*Gx1[58];
Gx2[59] = + (real_t)9.9999999999999995e-07*Gx1[59];
Gx2[60] = + (real_t)9.9999999999999995e-07*Gx1[60];
Gx2[61] = + (real_t)9.9999999999999995e-07*Gx1[61];
Gx2[62] = + (real_t)9.9999999999999995e-07*Gx1[62];
Gx2[63] = + (real_t)9.9999999999999995e-07*Gx1[63];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)9.9999999999999995e-07*Gx1[0];
Gx2[1] = + (real_t)9.9999999999999995e-07*Gx1[1];
Gx2[2] = + (real_t)9.9999999999999995e-07*Gx1[2];
Gx2[3] = + (real_t)9.9999999999999995e-07*Gx1[3];
Gx2[4] = + (real_t)9.9999999999999995e-07*Gx1[4];
Gx2[5] = + (real_t)9.9999999999999995e-07*Gx1[5];
Gx2[6] = + (real_t)9.9999999999999995e-07*Gx1[6];
Gx2[7] = + (real_t)9.9999999999999995e-07*Gx1[7];
Gx2[8] = + (real_t)9.9999999999999995e-07*Gx1[8];
Gx2[9] = + (real_t)9.9999999999999995e-07*Gx1[9];
Gx2[10] = + (real_t)9.9999999999999995e-07*Gx1[10];
Gx2[11] = + (real_t)9.9999999999999995e-07*Gx1[11];
Gx2[12] = + (real_t)9.9999999999999995e-07*Gx1[12];
Gx2[13] = + (real_t)9.9999999999999995e-07*Gx1[13];
Gx2[14] = + (real_t)9.9999999999999995e-07*Gx1[14];
Gx2[15] = + (real_t)9.9999999999999995e-07*Gx1[15];
Gx2[16] = + (real_t)9.9999999999999995e-07*Gx1[16];
Gx2[17] = + (real_t)9.9999999999999995e-07*Gx1[17];
Gx2[18] = + (real_t)9.9999999999999995e-07*Gx1[18];
Gx2[19] = + (real_t)9.9999999999999995e-07*Gx1[19];
Gx2[20] = + (real_t)9.9999999999999995e-07*Gx1[20];
Gx2[21] = + (real_t)9.9999999999999995e-07*Gx1[21];
Gx2[22] = + (real_t)9.9999999999999995e-07*Gx1[22];
Gx2[23] = + (real_t)9.9999999999999995e-07*Gx1[23];
Gx2[24] = + (real_t)9.9999999999999995e-07*Gx1[24];
Gx2[25] = + (real_t)9.9999999999999995e-07*Gx1[25];
Gx2[26] = + (real_t)9.9999999999999995e-07*Gx1[26];
Gx2[27] = + (real_t)9.9999999999999995e-07*Gx1[27];
Gx2[28] = + (real_t)9.9999999999999995e-07*Gx1[28];
Gx2[29] = + (real_t)9.9999999999999995e-07*Gx1[29];
Gx2[30] = + (real_t)9.9999999999999995e-07*Gx1[30];
Gx2[31] = + (real_t)9.9999999999999995e-07*Gx1[31];
Gx2[32] = + (real_t)9.9999999999999995e-07*Gx1[32];
Gx2[33] = + (real_t)9.9999999999999995e-07*Gx1[33];
Gx2[34] = + (real_t)9.9999999999999995e-07*Gx1[34];
Gx2[35] = + (real_t)9.9999999999999995e-07*Gx1[35];
Gx2[36] = + (real_t)9.9999999999999995e-07*Gx1[36];
Gx2[37] = + (real_t)9.9999999999999995e-07*Gx1[37];
Gx2[38] = + (real_t)9.9999999999999995e-07*Gx1[38];
Gx2[39] = + (real_t)9.9999999999999995e-07*Gx1[39];
Gx2[40] = + (real_t)9.9999999999999995e-07*Gx1[40];
Gx2[41] = + (real_t)9.9999999999999995e-07*Gx1[41];
Gx2[42] = + (real_t)9.9999999999999995e-07*Gx1[42];
Gx2[43] = + (real_t)9.9999999999999995e-07*Gx1[43];
Gx2[44] = + (real_t)9.9999999999999995e-07*Gx1[44];
Gx2[45] = + (real_t)9.9999999999999995e-07*Gx1[45];
Gx2[46] = + (real_t)9.9999999999999995e-07*Gx1[46];
Gx2[47] = + (real_t)9.9999999999999995e-07*Gx1[47];
Gx2[48] = + (real_t)9.9999999999999995e-07*Gx1[48];
Gx2[49] = + (real_t)9.9999999999999995e-07*Gx1[49];
Gx2[50] = + (real_t)9.9999999999999995e-07*Gx1[50];
Gx2[51] = + (real_t)9.9999999999999995e-07*Gx1[51];
Gx2[52] = + (real_t)9.9999999999999995e-07*Gx1[52];
Gx2[53] = + (real_t)9.9999999999999995e-07*Gx1[53];
Gx2[54] = + (real_t)9.9999999999999995e-07*Gx1[54];
Gx2[55] = + (real_t)9.9999999999999995e-07*Gx1[55];
Gx2[56] = + (real_t)9.9999999999999995e-07*Gx1[56];
Gx2[57] = + (real_t)9.9999999999999995e-07*Gx1[57];
Gx2[58] = + (real_t)9.9999999999999995e-07*Gx1[58];
Gx2[59] = + (real_t)9.9999999999999995e-07*Gx1[59];
Gx2[60] = + (real_t)9.9999999999999995e-07*Gx1[60];
Gx2[61] = + (real_t)9.9999999999999995e-07*Gx1[61];
Gx2[62] = + (real_t)9.9999999999999995e-07*Gx1[62];
Gx2[63] = + (real_t)9.9999999999999995e-07*Gx1[63];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)2.5000000000000000e+02*Gu1[0];
Gu2[1] = + (real_t)2.5000000000000000e+02*Gu1[1];
Gu2[2] = + (real_t)2.5000000000000000e+02*Gu1[2];
Gu2[3] = + (real_t)2.5000000000000000e+02*Gu1[3];
Gu2[4] = + (real_t)9.9999999999999995e-07*Gu1[4];
Gu2[5] = + (real_t)9.9999999999999995e-07*Gu1[5];
Gu2[6] = + (real_t)9.9999999999999995e-07*Gu1[6];
Gu2[7] = + (real_t)9.9999999999999995e-07*Gu1[7];
Gu2[8] = + (real_t)9.9999999999999995e-07*Gu1[8];
Gu2[9] = + (real_t)9.9999999999999995e-07*Gu1[9];
Gu2[10] = + (real_t)9.9999999999999995e-07*Gu1[10];
Gu2[11] = + (real_t)9.9999999999999995e-07*Gu1[11];
Gu2[12] = + (real_t)9.9999999999999995e-07*Gu1[12];
Gu2[13] = + (real_t)9.9999999999999995e-07*Gu1[13];
Gu2[14] = + (real_t)9.9999999999999995e-07*Gu1[14];
Gu2[15] = + (real_t)9.9999999999999995e-07*Gu1[15];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)9.9999999999999995e-07*Gu1[0];
Gu2[1] = + (real_t)9.9999999999999995e-07*Gu1[1];
Gu2[2] = + (real_t)9.9999999999999995e-07*Gu1[2];
Gu2[3] = + (real_t)9.9999999999999995e-07*Gu1[3];
Gu2[4] = + (real_t)9.9999999999999995e-07*Gu1[4];
Gu2[5] = + (real_t)9.9999999999999995e-07*Gu1[5];
Gu2[6] = + (real_t)9.9999999999999995e-07*Gu1[6];
Gu2[7] = + (real_t)9.9999999999999995e-07*Gu1[7];
Gu2[8] = + (real_t)9.9999999999999995e-07*Gu1[8];
Gu2[9] = + (real_t)9.9999999999999995e-07*Gu1[9];
Gu2[10] = + (real_t)9.9999999999999995e-07*Gu1[10];
Gu2[11] = + (real_t)9.9999999999999995e-07*Gu1[11];
Gu2[12] = + (real_t)9.9999999999999995e-07*Gu1[12];
Gu2[13] = + (real_t)9.9999999999999995e-07*Gu1[13];
Gu2[14] = + (real_t)9.9999999999999995e-07*Gu1[14];
Gu2[15] = + (real_t)9.9999999999999995e-07*Gu1[15];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void acado_condensePrep(  )
{
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.d[ 8 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 64 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 16 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.E[ 32 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.d[ 16 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGx[ 128 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.E[ 64 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.E[ 80 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.d[ 24 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGx[ 192 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.E[ 112 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 128 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 144 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.d[ 32 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.evGx[ 256 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 160 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.E[ 176 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 208 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.E[ 224 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.d[ 40 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.evGx[ 320 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.E[ 256 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 272 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.E[ 304 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.E[ 320 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.d[ 48 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.evGx[ 384 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.E[ 352 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.E[ 368 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 384 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.E[ 400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.E[ 416 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.E[ 432 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.d[ 56 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.evGx[ 448 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 448 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.E[ 464 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 496 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.E[ 512 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.E[ 528 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 544 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 112 ]), &(acadoWorkspace.E[ 560 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.d[ 64 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.evGx[ 512 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.E[ 592 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 608 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.E[ 624 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.E[ 640 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.E[ 656 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.E[ 672 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.E[ 688 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 128 ]), &(acadoWorkspace.E[ 704 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.d[ 72 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.evGx[ 576 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.E[ 720 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.E[ 736 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.E[ 752 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.E[ 768 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.E[ 784 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.E[ 800 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.E[ 816 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.E[ 832 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.E[ 848 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.E[ 864 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 448 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 736 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 752 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 784 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 800 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 832 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 848 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 16 ]), &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 160 ]), &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 448 ]), &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 112 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 176 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 256 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 352 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 464 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 592 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 736 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 16 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 128 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 272 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 368 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 608 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 752 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 32 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 208 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 496 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 224 ]), &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 304 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 512 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 640 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 784 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 64 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 416 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 656 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 800 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 80 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 544 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 560 ]), &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 688 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 832 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 112 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 704 ]), &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 848 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 128 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 144 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 16 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 160 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 448 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 736 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 752 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 768 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 784 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 800 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 816 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 832 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 848 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QE[ 32 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 64 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 112 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 176 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 256 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 352 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 464 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 592 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 736 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 752 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 768 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 784 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 800 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 816 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 832 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 848 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QE[ 80 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 128 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 272 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 368 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 608 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 752 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 768 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 784 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 800 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 816 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 832 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 848 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 208 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 496 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 784 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 800 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 816 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 832 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 848 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QE[ 224 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 304 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 400 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 512 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 640 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 784 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 800 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 816 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 832 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 848 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QE[ 320 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 416 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 656 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 800 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 816 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 832 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 848 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 544 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 832 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 848 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QE[ 560 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 688 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 832 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 848 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QE[ 704 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 848 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QE[ 864 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_multQ1d( acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.d[ 8 ]), &(acadoWorkspace.Qd[ 8 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 16 ]), &(acadoWorkspace.Qd[ 16 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 32 ]), &(acadoWorkspace.Qd[ 32 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 40 ]), &(acadoWorkspace.Qd[ 40 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 56 ]), &(acadoWorkspace.Qd[ 56 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 64 ]), &(acadoWorkspace.Qd[ 64 ]) );
acado_multQN1d( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 16 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 160 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 336 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 448 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 576 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 720 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 32 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 64 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 112 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 176 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 256 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 352 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 464 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 592 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 736 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 80 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 128 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 272 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 368 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 608 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 752 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 208 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 496 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 224 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 304 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 400 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 512 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 640 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 784 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 320 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 416 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 656 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 800 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 544 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 560 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 688 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 832 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 704 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 848 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.g[ 18 ]) );
acadoWorkspace.lb[0] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[19];
acadoWorkspace.ub[0] = (real_t)5.0000000000000000e+01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)5.0000000000000000e+01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)5.0000000000000000e+01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)5.0000000000000000e+01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)5.0000000000000000e+01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)5.0000000000000000e+01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)5.0000000000000000e+01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)5.0000000000000000e+01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)5.0000000000000000e+01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)5.0000000000000000e+01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)5.0000000000000000e+01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)5.0000000000000000e+01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)5.0000000000000000e+01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)5.0000000000000000e+01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)5.0000000000000000e+01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)5.0000000000000000e+01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)5.0000000000000000e+01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)5.0000000000000000e+01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)5.0000000000000000e+01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)5.0000000000000000e+01 - acadoVariables.u[19];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.Dy[80] -= acadoVariables.y[80];
acadoWorkspace.Dy[81] -= acadoVariables.y[81];
acadoWorkspace.Dy[82] -= acadoVariables.y[82];
acadoWorkspace.Dy[83] -= acadoVariables.y[83];
acadoWorkspace.Dy[84] -= acadoVariables.y[84];
acadoWorkspace.Dy[85] -= acadoVariables.y[85];
acadoWorkspace.Dy[86] -= acadoVariables.y[86];
acadoWorkspace.Dy[87] -= acadoVariables.y[87];
acadoWorkspace.Dy[88] -= acadoVariables.y[88];
acadoWorkspace.Dy[89] -= acadoVariables.y[89];
acadoWorkspace.Dy[90] -= acadoVariables.y[90];
acadoWorkspace.Dy[91] -= acadoVariables.y[91];
acadoWorkspace.Dy[92] -= acadoVariables.y[92];
acadoWorkspace.Dy[93] -= acadoVariables.y[93];
acadoWorkspace.Dy[94] -= acadoVariables.y[94];
acadoWorkspace.Dy[95] -= acadoVariables.y[95];
acadoWorkspace.Dy[96] -= acadoVariables.y[96];
acadoWorkspace.Dy[97] -= acadoVariables.y[97];
acadoWorkspace.Dy[98] -= acadoVariables.y[98];
acadoWorkspace.Dy[99] -= acadoVariables.y[99];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];
acadoWorkspace.DyN[7] -= acadoVariables.yN[7];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 18 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 72 ]) );

acadoWorkspace.QDy[80] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[81] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[82] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[83] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[84] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[4];
acadoWorkspace.QDy[85] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[86] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[87] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[7];

acadoWorkspace.QDy[8] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[84] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[85] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[86] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[87] += acadoWorkspace.Qd[79];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 448 ]), &(acadoWorkspace.QDy[ 64 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.QDy[ 16 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 32 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.QDy[ 40 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 56 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.QDy[ 64 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QDy[ 80 ]), &(acadoWorkspace.g[ 18 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[1] += + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[2] += + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[3] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[4] += + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[5] += + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[6] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[7] += + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[8] += + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[9] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[10] += + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[11] += + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[12] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[13] += + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[14] += + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[15] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[16] += + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[17] += + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[18] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[7];
acadoWorkspace.g[19] += + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[7];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];
acadoVariables.x[5] += acadoWorkspace.Dx0[5];
acadoVariables.x[6] += acadoWorkspace.Dx0[6];
acadoVariables.x[7] += acadoWorkspace.Dx0[7];

acadoVariables.x[8] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[0];
acadoVariables.x[9] += + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[1];
acadoVariables.x[10] += + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[2];
acadoVariables.x[11] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[3];
acadoVariables.x[12] += + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[4];
acadoVariables.x[13] += + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[5];
acadoVariables.x[14] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[6];
acadoVariables.x[15] += + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[7];
acadoVariables.x[16] += + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[8];
acadoVariables.x[17] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[9];
acadoVariables.x[18] += + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[10];
acadoVariables.x[19] += + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[11];
acadoVariables.x[20] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[12];
acadoVariables.x[21] += + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[13];
acadoVariables.x[22] += + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[14];
acadoVariables.x[23] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[15];
acadoVariables.x[24] += + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[16];
acadoVariables.x[25] += + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[17];
acadoVariables.x[26] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[18];
acadoVariables.x[27] += + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[19];
acadoVariables.x[28] += + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[20];
acadoVariables.x[29] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[21];
acadoVariables.x[30] += + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[22];
acadoVariables.x[31] += + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[23];
acadoVariables.x[32] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[24];
acadoVariables.x[33] += + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[25];
acadoVariables.x[34] += + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[26];
acadoVariables.x[35] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[27];
acadoVariables.x[36] += + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[28];
acadoVariables.x[37] += + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[29];
acadoVariables.x[38] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[30];
acadoVariables.x[39] += + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[31];
acadoVariables.x[40] += + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[32];
acadoVariables.x[41] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[33];
acadoVariables.x[42] += + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[34];
acadoVariables.x[43] += + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[35];
acadoVariables.x[44] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[36];
acadoVariables.x[45] += + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[37];
acadoVariables.x[46] += + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[38];
acadoVariables.x[47] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[39];
acadoVariables.x[48] += + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[40];
acadoVariables.x[49] += + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[41];
acadoVariables.x[50] += + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[42];
acadoVariables.x[51] += + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[43];
acadoVariables.x[52] += + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[44];
acadoVariables.x[53] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[45];
acadoVariables.x[54] += + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[46];
acadoVariables.x[55] += + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[47];
acadoVariables.x[56] += + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[48];
acadoVariables.x[57] += + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[49];
acadoVariables.x[58] += + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[50];
acadoVariables.x[59] += + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[51];
acadoVariables.x[60] += + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[52];
acadoVariables.x[61] += + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[53];
acadoVariables.x[62] += + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[54];
acadoVariables.x[63] += + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[55];
acadoVariables.x[64] += + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[56];
acadoVariables.x[65] += + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[57];
acadoVariables.x[66] += + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[58];
acadoVariables.x[67] += + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[59];
acadoVariables.x[68] += + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[60];
acadoVariables.x[69] += + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[61];
acadoVariables.x[70] += + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[62];
acadoVariables.x[71] += + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[63];
acadoVariables.x[72] += + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[64];
acadoVariables.x[73] += + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[65];
acadoVariables.x[74] += + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[66];
acadoVariables.x[75] += + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[67];
acadoVariables.x[76] += + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[550]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[551]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[68];
acadoVariables.x[77] += + acadoWorkspace.evGx[552]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[553]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[554]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[555]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[556]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[557]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[558]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[559]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[69];
acadoVariables.x[78] += + acadoWorkspace.evGx[560]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[561]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[562]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[563]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[564]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[565]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[566]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[567]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[70];
acadoVariables.x[79] += + acadoWorkspace.evGx[568]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[569]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[575]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[71];
acadoVariables.x[80] += + acadoWorkspace.evGx[576]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[577]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[578]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[579]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[580]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[581]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[582]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[583]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[72];
acadoVariables.x[81] += + acadoWorkspace.evGx[584]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[585]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[586]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[587]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[588]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[589]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[590]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[591]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[73];
acadoVariables.x[82] += + acadoWorkspace.evGx[592]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[593]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[594]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[74];
acadoVariables.x[83] += + acadoWorkspace.evGx[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[604]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[605]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[606]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[607]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[75];
acadoVariables.x[84] += + acadoWorkspace.evGx[608]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[609]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[610]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[611]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[612]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[613]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[614]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[615]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[76];
acadoVariables.x[85] += + acadoWorkspace.evGx[616]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[617]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[618]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[619]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[77];
acadoVariables.x[86] += + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[625]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[626]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[627]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[628]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[629]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[630]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[631]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[78];
acadoVariables.x[87] += + acadoWorkspace.evGx[632]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[633]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[634]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[635]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[636]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[637]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[638]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[639]*acadoWorkspace.Dx0[7] + acadoWorkspace.d[79];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 8 ]) );
acado_multEDu( &(acadoWorkspace.E[ 16 ]), acadoWorkspace.x, &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 32 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 16 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 64 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), acadoWorkspace.x, &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 112 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 128 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 32 ]) );
acado_multEDu( &(acadoWorkspace.E[ 160 ]), acadoWorkspace.x, &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 176 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 208 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 224 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 40 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), acadoWorkspace.x, &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 256 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 272 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 304 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 336 ]), acadoWorkspace.x, &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 352 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 368 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 416 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 56 ]) );
acado_multEDu( &(acadoWorkspace.E[ 448 ]), acadoWorkspace.x, &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 464 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 496 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 512 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 544 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 560 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 64 ]) );
acado_multEDu( &(acadoWorkspace.E[ 576 ]), acadoWorkspace.x, &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 592 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 608 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 640 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 656 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 688 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 704 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 720 ]), acadoWorkspace.x, &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 736 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 752 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 784 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 800 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 832 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 848 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 80 ]) );
acado_multEDu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 80 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 8];
acadoWorkspace.state[1] = acadoVariables.x[index * 8 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 8 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 8 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 8 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 8 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 8 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 8 + 7];
acadoWorkspace.state[88] = acadoVariables.u[index * 2];
acadoWorkspace.state[89] = acadoVariables.u[index * 2 + 1];
acadoWorkspace.state[90] = acadoVariables.od[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 8 + 8] = acadoWorkspace.state[0];
acadoVariables.x[index * 8 + 9] = acadoWorkspace.state[1];
acadoVariables.x[index * 8 + 10] = acadoWorkspace.state[2];
acadoVariables.x[index * 8 + 11] = acadoWorkspace.state[3];
acadoVariables.x[index * 8 + 12] = acadoWorkspace.state[4];
acadoVariables.x[index * 8 + 13] = acadoWorkspace.state[5];
acadoVariables.x[index * 8 + 14] = acadoWorkspace.state[6];
acadoVariables.x[index * 8 + 15] = acadoWorkspace.state[7];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 8] = acadoVariables.x[index * 8 + 8];
acadoVariables.x[index * 8 + 1] = acadoVariables.x[index * 8 + 9];
acadoVariables.x[index * 8 + 2] = acadoVariables.x[index * 8 + 10];
acadoVariables.x[index * 8 + 3] = acadoVariables.x[index * 8 + 11];
acadoVariables.x[index * 8 + 4] = acadoVariables.x[index * 8 + 12];
acadoVariables.x[index * 8 + 5] = acadoVariables.x[index * 8 + 13];
acadoVariables.x[index * 8 + 6] = acadoVariables.x[index * 8 + 14];
acadoVariables.x[index * 8 + 7] = acadoVariables.x[index * 8 + 15];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[80] = xEnd[0];
acadoVariables.x[81] = xEnd[1];
acadoVariables.x[82] = xEnd[2];
acadoVariables.x[83] = xEnd[3];
acadoVariables.x[84] = xEnd[4];
acadoVariables.x[85] = xEnd[5];
acadoVariables.x[86] = xEnd[6];
acadoVariables.x[87] = xEnd[7];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[80];
acadoWorkspace.state[1] = acadoVariables.x[81];
acadoWorkspace.state[2] = acadoVariables.x[82];
acadoWorkspace.state[3] = acadoVariables.x[83];
acadoWorkspace.state[4] = acadoVariables.x[84];
acadoWorkspace.state[5] = acadoVariables.x[85];
acadoWorkspace.state[6] = acadoVariables.x[86];
acadoWorkspace.state[7] = acadoVariables.x[87];
if (uEnd != 0)
{
acadoWorkspace.state[88] = uEnd[0];
acadoWorkspace.state[89] = uEnd[1];
}
else
{
acadoWorkspace.state[88] = acadoVariables.u[18];
acadoWorkspace.state[89] = acadoVariables.u[19];
}
acadoWorkspace.state[90] = acadoVariables.od[10];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[80] = acadoWorkspace.state[0];
acadoVariables.x[81] = acadoWorkspace.state[1];
acadoVariables.x[82] = acadoWorkspace.state[2];
acadoVariables.x[83] = acadoWorkspace.state[3];
acadoVariables.x[84] = acadoWorkspace.state[4];
acadoVariables.x[85] = acadoWorkspace.state[5];
acadoVariables.x[86] = acadoWorkspace.state[6];
acadoVariables.x[87] = acadoWorkspace.state[7];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[18] = uEnd[0];
acadoVariables.u[19] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 10 */
real_t tmpDy[ 10 ];

/** Row vector of size: 8 */
real_t tmpDyN[ 8 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 8];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 8 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 8 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 8 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 8 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 8 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 8 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 8 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[9] = acadoVariables.u[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[10] = acadoVariables.od[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 10] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 10];
acadoWorkspace.Dy[lRun1 * 10 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 10 + 1];
acadoWorkspace.Dy[lRun1 * 10 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 10 + 2];
acadoWorkspace.Dy[lRun1 * 10 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 10 + 3];
acadoWorkspace.Dy[lRun1 * 10 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 10 + 4];
acadoWorkspace.Dy[lRun1 * 10 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 10 + 5];
acadoWorkspace.Dy[lRun1 * 10 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 10 + 6];
acadoWorkspace.Dy[lRun1 * 10 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 10 + 7];
acadoWorkspace.Dy[lRun1 * 10 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 10 + 8];
acadoWorkspace.Dy[lRun1 * 10 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 10 + 9];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[80];
acadoWorkspace.objValueIn[1] = acadoVariables.x[81];
acadoWorkspace.objValueIn[2] = acadoVariables.x[82];
acadoWorkspace.objValueIn[3] = acadoVariables.x[83];
acadoWorkspace.objValueIn[4] = acadoVariables.x[84];
acadoWorkspace.objValueIn[5] = acadoVariables.x[85];
acadoWorkspace.objValueIn[6] = acadoVariables.x[86];
acadoWorkspace.objValueIn[7] = acadoVariables.x[87];
acadoWorkspace.objValueIn[8] = acadoVariables.od[10];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 10]*(real_t)2.5000000000000000e+02;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 10 + 1]*(real_t)2.5000000000000000e+02;
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 10 + 2]*(real_t)9.9999999999999995e-07;
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 10 + 3]*(real_t)9.9999999999999995e-07;
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 10 + 4]*(real_t)9.9999999999999995e-07;
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 10 + 5]*(real_t)9.9999999999999995e-07;
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 10 + 6]*(real_t)9.9999999999999995e-07;
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 10 + 7]*(real_t)9.9999999999999995e-07;
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 10 + 8]*(real_t)1.0000000000000000e-03;
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 10 + 9]*(real_t)1.0000000000000000e-03;
objVal += + acadoWorkspace.Dy[lRun1 * 10]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 10 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 10 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 10 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 10 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 10 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 10 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 10 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 10 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 10 + 9]*tmpDy[9];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)9.9999999999999995e-07;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)9.9999999999999995e-07;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)9.9999999999999995e-07;
tmpDyN[3] = + acadoWorkspace.DyN[3]*(real_t)9.9999999999999995e-07;
tmpDyN[4] = + acadoWorkspace.DyN[4]*(real_t)9.9999999999999995e-07;
tmpDyN[5] = + acadoWorkspace.DyN[5]*(real_t)9.9999999999999995e-07;
tmpDyN[6] = + acadoWorkspace.DyN[6]*(real_t)9.9999999999999995e-07;
tmpDyN[7] = + acadoWorkspace.DyN[7]*(real_t)9.9999999999999995e-07;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7];

objVal *= 0.5;
return objVal;
}

