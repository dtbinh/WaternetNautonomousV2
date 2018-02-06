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
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 6 + 5];

acadoWorkspace.state[54] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[55] = acadoVariables.u[lRun1 * 2 + 1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 6] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 6 + 6];
acadoWorkspace.d[lRun1 * 6 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 6 + 7];
acadoWorkspace.d[lRun1 * 6 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 6 + 8];
acadoWorkspace.d[lRun1 * 6 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 6 + 9];
acadoWorkspace.d[lRun1 * 6 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 6 + 10];
acadoWorkspace.d[lRun1 * 6 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 6 + 11];

acadoWorkspace.evGx[lRun1 * 36] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 36 + 1] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 36 + 2] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 36 + 3] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 36 + 4] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 36 + 5] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 36 + 6] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 36 + 7] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 36 + 8] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 36 + 9] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 36 + 10] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 36 + 11] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 36 + 12] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 36 + 13] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 36 + 14] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 36 + 15] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 36 + 16] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 36 + 17] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 36 + 18] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 36 + 19] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 36 + 20] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 36 + 21] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 36 + 22] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 36 + 23] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 36 + 24] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 36 + 25] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 36 + 26] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 36 + 27] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 36 + 28] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 36 + 29] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 36 + 30] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 36 + 31] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 36 + 32] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 36 + 33] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 36 + 34] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 36 + 35] = acadoWorkspace.state[41];

acadoWorkspace.evGu[lRun1 * 12] = acadoWorkspace.state[42];
acadoWorkspace.evGu[lRun1 * 12 + 1] = acadoWorkspace.state[43];
acadoWorkspace.evGu[lRun1 * 12 + 2] = acadoWorkspace.state[44];
acadoWorkspace.evGu[lRun1 * 12 + 3] = acadoWorkspace.state[45];
acadoWorkspace.evGu[lRun1 * 12 + 4] = acadoWorkspace.state[46];
acadoWorkspace.evGu[lRun1 * 12 + 5] = acadoWorkspace.state[47];
acadoWorkspace.evGu[lRun1 * 12 + 6] = acadoWorkspace.state[48];
acadoWorkspace.evGu[lRun1 * 12 + 7] = acadoWorkspace.state[49];
acadoWorkspace.evGu[lRun1 * 12 + 8] = acadoWorkspace.state[50];
acadoWorkspace.evGu[lRun1 * 12 + 9] = acadoWorkspace.state[51];
acadoWorkspace.evGu[lRun1 * 12 + 10] = acadoWorkspace.state[52];
acadoWorkspace.evGu[lRun1 * 12 + 11] = acadoWorkspace.state[53];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
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
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[7] = acadoVariables.u[runObj * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 6] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 6 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 6 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 6 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 6 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 6 + 5] = acadoWorkspace.objValueOut[5];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.x[124];
acadoWorkspace.objValueIn[5] = acadoVariables.x[125];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5];
dNew[1] += + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2] + Gx1[9]*dOld[3] + Gx1[10]*dOld[4] + Gx1[11]*dOld[5];
dNew[2] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5];
dNew[3] += + Gx1[18]*dOld[0] + Gx1[19]*dOld[1] + Gx1[20]*dOld[2] + Gx1[21]*dOld[3] + Gx1[22]*dOld[4] + Gx1[23]*dOld[5];
dNew[4] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5];
dNew[5] += + Gx1[30]*dOld[0] + Gx1[31]*dOld[1] + Gx1[32]*dOld[2] + Gx1[33]*dOld[3] + Gx1[34]*dOld[4] + Gx1[35]*dOld[5];
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
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[24] + Gx1[5]*Gx2[30];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[25] + Gx1[5]*Gx2[31];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[20] + Gx1[4]*Gx2[26] + Gx1[5]*Gx2[32];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[15] + Gx1[3]*Gx2[21] + Gx1[4]*Gx2[27] + Gx1[5]*Gx2[33];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[22] + Gx1[4]*Gx2[28] + Gx1[5]*Gx2[34];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[23] + Gx1[4]*Gx2[29] + Gx1[5]*Gx2[35];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[6] + Gx1[8]*Gx2[12] + Gx1[9]*Gx2[18] + Gx1[10]*Gx2[24] + Gx1[11]*Gx2[30];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[7] + Gx1[8]*Gx2[13] + Gx1[9]*Gx2[19] + Gx1[10]*Gx2[25] + Gx1[11]*Gx2[31];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[8] + Gx1[8]*Gx2[14] + Gx1[9]*Gx2[20] + Gx1[10]*Gx2[26] + Gx1[11]*Gx2[32];
Gx3[9] = + Gx1[6]*Gx2[3] + Gx1[7]*Gx2[9] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[21] + Gx1[10]*Gx2[27] + Gx1[11]*Gx2[33];
Gx3[10] = + Gx1[6]*Gx2[4] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[22] + Gx1[10]*Gx2[28] + Gx1[11]*Gx2[34];
Gx3[11] = + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[23] + Gx1[10]*Gx2[29] + Gx1[11]*Gx2[35];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[12] + Gx1[15]*Gx2[18] + Gx1[16]*Gx2[24] + Gx1[17]*Gx2[30];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[13] + Gx1[15]*Gx2[19] + Gx1[16]*Gx2[25] + Gx1[17]*Gx2[31];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[8] + Gx1[14]*Gx2[14] + Gx1[15]*Gx2[20] + Gx1[16]*Gx2[26] + Gx1[17]*Gx2[32];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[9] + Gx1[14]*Gx2[15] + Gx1[15]*Gx2[21] + Gx1[16]*Gx2[27] + Gx1[17]*Gx2[33];
Gx3[16] = + Gx1[12]*Gx2[4] + Gx1[13]*Gx2[10] + Gx1[14]*Gx2[16] + Gx1[15]*Gx2[22] + Gx1[16]*Gx2[28] + Gx1[17]*Gx2[34];
Gx3[17] = + Gx1[12]*Gx2[5] + Gx1[13]*Gx2[11] + Gx1[14]*Gx2[17] + Gx1[15]*Gx2[23] + Gx1[16]*Gx2[29] + Gx1[17]*Gx2[35];
Gx3[18] = + Gx1[18]*Gx2[0] + Gx1[19]*Gx2[6] + Gx1[20]*Gx2[12] + Gx1[21]*Gx2[18] + Gx1[22]*Gx2[24] + Gx1[23]*Gx2[30];
Gx3[19] = + Gx1[18]*Gx2[1] + Gx1[19]*Gx2[7] + Gx1[20]*Gx2[13] + Gx1[21]*Gx2[19] + Gx1[22]*Gx2[25] + Gx1[23]*Gx2[31];
Gx3[20] = + Gx1[18]*Gx2[2] + Gx1[19]*Gx2[8] + Gx1[20]*Gx2[14] + Gx1[21]*Gx2[20] + Gx1[22]*Gx2[26] + Gx1[23]*Gx2[32];
Gx3[21] = + Gx1[18]*Gx2[3] + Gx1[19]*Gx2[9] + Gx1[20]*Gx2[15] + Gx1[21]*Gx2[21] + Gx1[22]*Gx2[27] + Gx1[23]*Gx2[33];
Gx3[22] = + Gx1[18]*Gx2[4] + Gx1[19]*Gx2[10] + Gx1[20]*Gx2[16] + Gx1[21]*Gx2[22] + Gx1[22]*Gx2[28] + Gx1[23]*Gx2[34];
Gx3[23] = + Gx1[18]*Gx2[5] + Gx1[19]*Gx2[11] + Gx1[20]*Gx2[17] + Gx1[21]*Gx2[23] + Gx1[22]*Gx2[29] + Gx1[23]*Gx2[35];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[6] + Gx1[26]*Gx2[12] + Gx1[27]*Gx2[18] + Gx1[28]*Gx2[24] + Gx1[29]*Gx2[30];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[7] + Gx1[26]*Gx2[13] + Gx1[27]*Gx2[19] + Gx1[28]*Gx2[25] + Gx1[29]*Gx2[31];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[14] + Gx1[27]*Gx2[20] + Gx1[28]*Gx2[26] + Gx1[29]*Gx2[32];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[15] + Gx1[27]*Gx2[21] + Gx1[28]*Gx2[27] + Gx1[29]*Gx2[33];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[22] + Gx1[28]*Gx2[28] + Gx1[29]*Gx2[34];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[23] + Gx1[28]*Gx2[29] + Gx1[29]*Gx2[35];
Gx3[30] = + Gx1[30]*Gx2[0] + Gx1[31]*Gx2[6] + Gx1[32]*Gx2[12] + Gx1[33]*Gx2[18] + Gx1[34]*Gx2[24] + Gx1[35]*Gx2[30];
Gx3[31] = + Gx1[30]*Gx2[1] + Gx1[31]*Gx2[7] + Gx1[32]*Gx2[13] + Gx1[33]*Gx2[19] + Gx1[34]*Gx2[25] + Gx1[35]*Gx2[31];
Gx3[32] = + Gx1[30]*Gx2[2] + Gx1[31]*Gx2[8] + Gx1[32]*Gx2[14] + Gx1[33]*Gx2[20] + Gx1[34]*Gx2[26] + Gx1[35]*Gx2[32];
Gx3[33] = + Gx1[30]*Gx2[3] + Gx1[31]*Gx2[9] + Gx1[32]*Gx2[15] + Gx1[33]*Gx2[21] + Gx1[34]*Gx2[27] + Gx1[35]*Gx2[33];
Gx3[34] = + Gx1[30]*Gx2[4] + Gx1[31]*Gx2[10] + Gx1[32]*Gx2[16] + Gx1[33]*Gx2[22] + Gx1[34]*Gx2[28] + Gx1[35]*Gx2[34];
Gx3[35] = + Gx1[30]*Gx2[5] + Gx1[31]*Gx2[11] + Gx1[32]*Gx2[17] + Gx1[33]*Gx2[23] + Gx1[34]*Gx2[29] + Gx1[35]*Gx2[35];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8] + Gx1[5]*Gu1[10];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9] + Gx1[5]*Gu1[11];
Gu2[2] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[8]*Gu1[4] + Gx1[9]*Gu1[6] + Gx1[10]*Gu1[8] + Gx1[11]*Gu1[10];
Gu2[3] = + Gx1[6]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[8]*Gu1[5] + Gx1[9]*Gu1[7] + Gx1[10]*Gu1[9] + Gx1[11]*Gu1[11];
Gu2[4] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[15]*Gu1[6] + Gx1[16]*Gu1[8] + Gx1[17]*Gu1[10];
Gu2[5] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[15]*Gu1[7] + Gx1[16]*Gu1[9] + Gx1[17]*Gu1[11];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[2] + Gx1[20]*Gu1[4] + Gx1[21]*Gu1[6] + Gx1[22]*Gu1[8] + Gx1[23]*Gu1[10];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[5] + Gx1[21]*Gu1[7] + Gx1[22]*Gu1[9] + Gx1[23]*Gu1[11];
Gu2[8] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[2] + Gx1[26]*Gu1[4] + Gx1[27]*Gu1[6] + Gx1[28]*Gu1[8] + Gx1[29]*Gu1[10];
Gu2[9] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[3] + Gx1[26]*Gu1[5] + Gx1[27]*Gu1[7] + Gx1[28]*Gu1[9] + Gx1[29]*Gu1[11];
Gu2[10] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[2] + Gx1[32]*Gu1[4] + Gx1[33]*Gu1[6] + Gx1[34]*Gu1[8] + Gx1[35]*Gu1[10];
Gu2[11] = + Gx1[30]*Gu1[1] + Gx1[31]*Gu1[3] + Gx1[32]*Gu1[5] + Gx1[33]*Gu1[7] + Gx1[34]*Gu1[9] + Gx1[35]*Gu1[11];
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
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10];
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] = 0.0;
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] = 0.0;
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] = 0.0;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 2)] = acadoWorkspace.H[(iCol * 80) + (iRow * 2)];
acadoWorkspace.H[(iRow * 80) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 80) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 2 + 1)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = +dOld[0];
dNew[1] = +dOld[1];
dNew[2] = + (real_t)9.9999999999999995e-07*dOld[2];
dNew[3] = + (real_t)9.9999999999999995e-07*dOld[3];
dNew[4] = + (real_t)9.9999999999999995e-07*dOld[4];
dNew[5] = + (real_t)9.9999999999999995e-07*dOld[5];
}

void acado_multQN1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)1.0000000000000000e+06*dOld[0];
dNew[1] = + (real_t)1.0000000000000000e+06*dOld[1];
dNew[2] = + (real_t)9.9999999999999995e-07*dOld[2];
dNew[3] = + (real_t)9.9999999999999995e-07*dOld[3];
dNew[4] = + (real_t)9.9999999999999995e-07*dOld[4];
dNew[5] = + (real_t)9.9999999999999995e-07*dOld[5];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = 0.0;
;
RDy1[1] = 0.0;
;
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = +Dy1[0];
QDy1[1] = +Dy1[1];
QDy1[2] = + (real_t)9.9999999999999995e-07*Dy1[2];
QDy1[3] = + (real_t)9.9999999999999995e-07*Dy1[3];
QDy1[4] = + (real_t)9.9999999999999995e-07*Dy1[4];
QDy1[5] = + (real_t)9.9999999999999995e-07*Dy1[5];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4] + E1[10]*QDy1[5];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4] + E1[11]*QDy1[5];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[6] + E1[4]*Gx1[12] + E1[6]*Gx1[18] + E1[8]*Gx1[24] + E1[10]*Gx1[30];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[7] + E1[4]*Gx1[13] + E1[6]*Gx1[19] + E1[8]*Gx1[25] + E1[10]*Gx1[31];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[8] + E1[4]*Gx1[14] + E1[6]*Gx1[20] + E1[8]*Gx1[26] + E1[10]*Gx1[32];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[9] + E1[4]*Gx1[15] + E1[6]*Gx1[21] + E1[8]*Gx1[27] + E1[10]*Gx1[33];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[10] + E1[4]*Gx1[16] + E1[6]*Gx1[22] + E1[8]*Gx1[28] + E1[10]*Gx1[34];
H101[5] += + E1[0]*Gx1[5] + E1[2]*Gx1[11] + E1[4]*Gx1[17] + E1[6]*Gx1[23] + E1[8]*Gx1[29] + E1[10]*Gx1[35];
H101[6] += + E1[1]*Gx1[0] + E1[3]*Gx1[6] + E1[5]*Gx1[12] + E1[7]*Gx1[18] + E1[9]*Gx1[24] + E1[11]*Gx1[30];
H101[7] += + E1[1]*Gx1[1] + E1[3]*Gx1[7] + E1[5]*Gx1[13] + E1[7]*Gx1[19] + E1[9]*Gx1[25] + E1[11]*Gx1[31];
H101[8] += + E1[1]*Gx1[2] + E1[3]*Gx1[8] + E1[5]*Gx1[14] + E1[7]*Gx1[20] + E1[9]*Gx1[26] + E1[11]*Gx1[32];
H101[9] += + E1[1]*Gx1[3] + E1[3]*Gx1[9] + E1[5]*Gx1[15] + E1[7]*Gx1[21] + E1[9]*Gx1[27] + E1[11]*Gx1[33];
H101[10] += + E1[1]*Gx1[4] + E1[3]*Gx1[10] + E1[5]*Gx1[16] + E1[7]*Gx1[22] + E1[9]*Gx1[28] + E1[11]*Gx1[34];
H101[11] += + E1[1]*Gx1[5] + E1[3]*Gx1[11] + E1[5]*Gx1[17] + E1[7]*Gx1[23] + E1[9]*Gx1[29] + E1[11]*Gx1[35];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 12; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
dNew[5] += + E1[10]*U1[0] + E1[11]*U1[1];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = +Gx1[0];
Gx2[1] = +Gx1[1];
Gx2[2] = +Gx1[2];
Gx2[3] = +Gx1[3];
Gx2[4] = +Gx1[4];
Gx2[5] = +Gx1[5];
Gx2[6] = +Gx1[6];
Gx2[7] = +Gx1[7];
Gx2[8] = +Gx1[8];
Gx2[9] = +Gx1[9];
Gx2[10] = +Gx1[10];
Gx2[11] = +Gx1[11];
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
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e+06*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e+06*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000000e+06*Gx1[2];
Gx2[3] = + (real_t)1.0000000000000000e+06*Gx1[3];
Gx2[4] = + (real_t)1.0000000000000000e+06*Gx1[4];
Gx2[5] = + (real_t)1.0000000000000000e+06*Gx1[5];
Gx2[6] = + (real_t)1.0000000000000000e+06*Gx1[6];
Gx2[7] = + (real_t)1.0000000000000000e+06*Gx1[7];
Gx2[8] = + (real_t)1.0000000000000000e+06*Gx1[8];
Gx2[9] = + (real_t)1.0000000000000000e+06*Gx1[9];
Gx2[10] = + (real_t)1.0000000000000000e+06*Gx1[10];
Gx2[11] = + (real_t)1.0000000000000000e+06*Gx1[11];
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
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = +Gu1[0];
Gu2[1] = +Gu1[1];
Gu2[2] = +Gu1[2];
Gu2[3] = +Gu1[3];
Gu2[4] = + (real_t)9.9999999999999995e-07*Gu1[4];
Gu2[5] = + (real_t)9.9999999999999995e-07*Gu1[5];
Gu2[6] = + (real_t)9.9999999999999995e-07*Gu1[6];
Gu2[7] = + (real_t)9.9999999999999995e-07*Gu1[7];
Gu2[8] = + (real_t)9.9999999999999995e-07*Gu1[8];
Gu2[9] = + (real_t)9.9999999999999995e-07*Gu1[9];
Gu2[10] = + (real_t)9.9999999999999995e-07*Gu1[10];
Gu2[11] = + (real_t)9.9999999999999995e-07*Gu1[11];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e+06*Gu1[0];
Gu2[1] = + (real_t)1.0000000000000000e+06*Gu1[1];
Gu2[2] = + (real_t)1.0000000000000000e+06*Gu1[2];
Gu2[3] = + (real_t)1.0000000000000000e+06*Gu1[3];
Gu2[4] = + (real_t)9.9999999999999995e-07*Gu1[4];
Gu2[5] = + (real_t)9.9999999999999995e-07*Gu1[5];
Gu2[6] = + (real_t)9.9999999999999995e-07*Gu1[6];
Gu2[7] = + (real_t)9.9999999999999995e-07*Gu1[7];
Gu2[8] = + (real_t)9.9999999999999995e-07*Gu1[8];
Gu2[9] = + (real_t)9.9999999999999995e-07*Gu1[9];
Gu2[10] = + (real_t)9.9999999999999995e-07*Gu1[10];
Gu2[11] = + (real_t)9.9999999999999995e-07*Gu1[11];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[6] + Hx[2]*Gx[12] + Hx[3]*Gx[18] + Hx[4]*Gx[24] + Hx[5]*Gx[30];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[7] + Hx[2]*Gx[13] + Hx[3]*Gx[19] + Hx[4]*Gx[25] + Hx[5]*Gx[31];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[8] + Hx[2]*Gx[14] + Hx[3]*Gx[20] + Hx[4]*Gx[26] + Hx[5]*Gx[32];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[9] + Hx[2]*Gx[15] + Hx[3]*Gx[21] + Hx[4]*Gx[27] + Hx[5]*Gx[33];
A01[4] = + Hx[0]*Gx[4] + Hx[1]*Gx[10] + Hx[2]*Gx[16] + Hx[3]*Gx[22] + Hx[4]*Gx[28] + Hx[5]*Gx[34];
A01[5] = + Hx[0]*Gx[5] + Hx[1]*Gx[11] + Hx[2]*Gx[17] + Hx[3]*Gx[23] + Hx[4]*Gx[29] + Hx[5]*Gx[35];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 40) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6] + Hx[4]*E[8] + Hx[5]*E[10];
acadoWorkspace.A[(row * 40) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7] + Hx[4]*E[9] + Hx[5]*E[11];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 67. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[2]));
a[1] = (sin(xd[2]));
a[2] = (((xd[0]-(real_t)(1.0000000000000000e+01))*a[0])+(xd[1]*a[1]));
a[3] = (sin(xd[2]));
a[4] = (cos(xd[2]));
a[5] = ((((real_t)(0.0000000000000000e+00)-(xd[0]-(real_t)(1.0000000000000000e+01)))*a[3])+(xd[1]*a[4]));
a[6] = (((a[2]/(real_t)(3.0000000000000000e+00))*(a[2]/(real_t)(3.0000000000000000e+00)))+((a[5]/(real_t)(3.0000000000000000e+00))*(a[5]/(real_t)(3.0000000000000000e+00))));
a[7] = ((real_t)(1.0000000000000000e+00)/(real_t)(3.0000000000000000e+00));
a[8] = (a[0]*a[7]);
a[9] = (a[2]/(real_t)(3.0000000000000000e+00));
a[10] = (a[8]*a[9]);
a[11] = ((real_t)(1.0000000000000000e+00)/(real_t)(3.0000000000000000e+00));
a[12] = (a[0]*a[11]);
a[13] = (a[2]/(real_t)(3.0000000000000000e+00));
a[14] = (a[12]*a[13]);
a[15] = (a[10]+a[14]);
a[16] = (real_t)(-1.0000000000000000e+00);
a[17] = (a[16]*a[3]);
a[18] = ((real_t)(1.0000000000000000e+00)/(real_t)(3.0000000000000000e+00));
a[19] = (a[17]*a[18]);
a[20] = (a[5]/(real_t)(3.0000000000000000e+00));
a[21] = (a[19]*a[20]);
a[22] = ((real_t)(1.0000000000000000e+00)/(real_t)(3.0000000000000000e+00));
a[23] = (a[17]*a[22]);
a[24] = (a[5]/(real_t)(3.0000000000000000e+00));
a[25] = (a[23]*a[24]);
a[26] = (a[21]+a[25]);
a[27] = (a[15]+a[26]);
a[28] = (a[1]*a[7]);
a[29] = (a[28]*a[9]);
a[30] = (a[1]*a[11]);
a[31] = (a[30]*a[13]);
a[32] = (a[29]+a[31]);
a[33] = (a[4]*a[18]);
a[34] = (a[33]*a[20]);
a[35] = (a[4]*a[22]);
a[36] = (a[35]*a[24]);
a[37] = (a[34]+a[36]);
a[38] = (a[32]+a[37]);
a[39] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[40] = (xd[0]-(real_t)(1.0000000000000000e+01));
a[41] = (a[39]*a[40]);
a[42] = (cos(xd[2]));
a[43] = (a[42]*xd[1]);
a[44] = (a[41]+a[43]);
a[45] = (a[44]*a[7]);
a[46] = (a[45]*a[9]);
a[47] = (a[44]*a[11]);
a[48] = (a[47]*a[13]);
a[49] = (a[46]+a[48]);
a[50] = (cos(xd[2]));
a[51] = ((real_t)(0.0000000000000000e+00)-(xd[0]-(real_t)(1.0000000000000000e+01)));
a[52] = (a[50]*a[51]);
a[53] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[54] = (a[53]*xd[1]);
a[55] = (a[52]+a[54]);
a[56] = (a[55]*a[18]);
a[57] = (a[56]*a[20]);
a[58] = (a[55]*a[22]);
a[59] = (a[58]*a[24]);
a[60] = (a[57]+a[59]);
a[61] = (a[49]+a[60]);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);
a[65] = (real_t)(0.0000000000000000e+00);
a[66] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[6];
out[1] = a[27];
out[2] = a[38];
out[3] = a[61];
out[4] = a[62];
out[5] = a[63];
out[6] = a[64];
out[7] = a[65];
out[8] = a[66];
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
int lRun1;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.d[ 6 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 36 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 12 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 24 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.d[ 12 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGx[ 72 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 36 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 48 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 60 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.d[ 18 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGx[ 108 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 84 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 96 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.E[ 108 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.d[ 24 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGx[ 144 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 132 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 156 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 168 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.d[ 30 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGx[ 180 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 204 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.E[ 216 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.E[ 228 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.E[ 240 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.d[ 36 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGx[ 216 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 252 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 264 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.E[ 276 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 312 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.E[ 324 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.d[ 42 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGx[ 252 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.E[ 348 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 372 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.E[ 384 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.E[ 396 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.E[ 408 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.E[ 420 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.d[ 48 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGx[ 288 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 432 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.E[ 444 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 456 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.E[ 468 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.E[ 492 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.E[ 504 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.E[ 516 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.E[ 528 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.d[ 54 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGx[ 324 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 540 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.E[ 552 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.E[ 564 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 588 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.E[ 600 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.E[ 612 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.E[ 624 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.E[ 636 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.E[ 648 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 360 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.d[ 60 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGx[ 360 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.E[ 660 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.E[ 672 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.E[ 684 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.E[ 696 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.E[ 708 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.E[ 720 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.E[ 732 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.E[ 744 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.E[ 756 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.E[ 768 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.E[ 780 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 396 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.d[ 66 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.evGx[ 396 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.E[ 792 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.E[ 804 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.E[ 816 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.E[ 828 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.E[ 840 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.E[ 852 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.E[ 864 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.E[ 876 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.E[ 888 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.E[ 900 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.E[ 912 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.E[ 924 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 66 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.d[ 72 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.evGx[ 432 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.E[ 936 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.E[ 948 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.E[ 960 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.E[ 972 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.E[ 984 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.E[ 996 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.E[ 1008 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.E[ 1020 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.E[ 1032 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.E[ 1044 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.E[ 1056 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.E[ 1068 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.E[ 1080 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 468 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.d[ 78 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGx[ 468 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.E[ 1092 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.E[ 1104 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.E[ 1116 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.E[ 1128 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.E[ 1140 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.E[ 1152 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.E[ 1164 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.E[ 1176 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.E[ 1188 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.E[ 1200 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.E[ 1212 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.E[ 1224 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.E[ 1236 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.E[ 1248 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 504 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 78 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.d[ 84 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.evGx[ 504 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.E[ 1260 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.E[ 1272 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.E[ 1284 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.E[ 1296 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.E[ 1308 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.E[ 1320 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.E[ 1332 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.E[ 1344 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.E[ 1356 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.E[ 1368 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.E[ 1380 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.E[ 1392 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.E[ 1404 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.E[ 1416 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.E[ 1428 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 540 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.d[ 90 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.evGx[ 540 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.E[ 1440 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.E[ 1452 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.E[ 1464 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.E[ 1476 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.E[ 1488 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.E[ 1500 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.E[ 1512 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.E[ 1524 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.E[ 1536 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.E[ 1548 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.E[ 1560 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.E[ 1572 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.E[ 1584 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.E[ 1596 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.E[ 1608 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.E[ 1620 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.d[ 96 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.evGx[ 576 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.E[ 1632 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.E[ 1644 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.E[ 1656 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.E[ 1668 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.E[ 1680 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.E[ 1692 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.E[ 1704 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.E[ 1716 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.E[ 1728 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.E[ 1740 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.E[ 1752 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.E[ 1764 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.E[ 1776 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.E[ 1788 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.E[ 1800 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.E[ 1812 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.E[ 1824 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 612 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.d[ 102 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGx[ 612 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.E[ 1836 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.E[ 1848 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.E[ 1860 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.E[ 1872 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.E[ 1884 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.E[ 1896 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.E[ 1908 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.E[ 1920 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.E[ 1932 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.E[ 1944 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.E[ 1956 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.E[ 1968 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.E[ 1980 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.E[ 1992 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.E[ 2004 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.E[ 2016 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.E[ 2028 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 204 ]), &(acadoWorkspace.E[ 2040 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 102 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.d[ 108 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.evGx[ 648 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.E[ 2052 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.E[ 2064 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.E[ 2076 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.E[ 2088 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.E[ 2100 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.E[ 2112 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.E[ 2124 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.E[ 2136 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.E[ 2148 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.E[ 2160 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.E[ 2172 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.E[ 2184 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.E[ 2196 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.E[ 2208 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.E[ 2220 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.E[ 2232 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.E[ 2244 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.E[ 2256 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.E[ 2268 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 684 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.d[ 114 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGx[ 684 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.E[ 2280 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.E[ 2292 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.E[ 2304 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.E[ 2316 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.E[ 2328 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.E[ 2340 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.E[ 2352 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.E[ 2364 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.E[ 2376 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.E[ 2388 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.E[ 2400 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.E[ 2412 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.E[ 2424 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.E[ 2436 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.E[ 2448 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.E[ 2460 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.E[ 2472 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.E[ 2484 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.E[ 2496 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 228 ]), &(acadoWorkspace.E[ 2508 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 660 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 684 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 804 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 948 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1092 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1116 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1284 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1452 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1644 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1656 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1836 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1848 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1860 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2052 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2076 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2280 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2292 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2304 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2316 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2328 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2340 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2352 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2364 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2376 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2388 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2400 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2412 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2424 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2436 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2448 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2460 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2472 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QE[ 2484 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2496 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 2508 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 252 ]), &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 660 ]), &(acadoWorkspace.evGx[ 360 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 792 ]), &(acadoWorkspace.evGx[ 396 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 936 ]), &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1092 ]), &(acadoWorkspace.evGx[ 468 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1260 ]), &(acadoWorkspace.evGx[ 504 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1440 ]), &(acadoWorkspace.evGx[ 540 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1632 ]), &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1836 ]), &(acadoWorkspace.evGx[ 612 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2052 ]), &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 2280 ]), &(acadoWorkspace.evGx[ 684 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 348 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 444 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 804 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 948 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1272 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1452 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1644 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1848 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2064 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2292 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 564 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 684 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1116 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1284 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1464 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1656 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1860 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2076 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2304 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 372 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 468 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 696 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 828 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 972 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1128 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1476 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1668 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1872 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2088 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2316 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 588 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 708 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 840 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 984 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1140 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1308 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1680 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1884 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2100 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2328 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 396 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 492 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 852 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 996 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1320 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1500 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1692 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1896 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2112 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2340 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 60 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 408 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 612 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 732 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1008 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1164 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1332 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1512 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1704 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1908 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2124 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2352 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 72 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 516 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 744 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 876 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1020 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1176 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1344 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1524 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1716 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1920 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2136 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2364 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 84 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 636 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 756 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 888 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1032 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1188 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1356 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1728 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1932 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2148 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2376 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 96 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 900 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1044 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1368 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1548 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1740 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1944 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2160 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2388 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 780 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1212 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1380 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1560 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1752 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1956 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2172 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2400 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 120 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 924 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1068 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1224 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1572 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1764 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1968 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2184 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2412 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 132 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1236 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1404 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1776 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1980 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2196 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2424 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 144 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1248 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1416 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1596 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1788 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1992 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2208 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2436 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 156 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1428 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1608 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1800 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2004 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2220 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2448 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 168 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1620 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1812 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2016 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2232 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2460 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 180 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1824 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2028 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2244 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2472 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 192 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 204 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2040 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.H10[ 204 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2256 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 204 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2484 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 204 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 216 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2268 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 216 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2496 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 216 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 228 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 2508 ]), &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.H10[ 228 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 660 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 792 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 936 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1092 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1260 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1440 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1632 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1836 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2052 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2280 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 804 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 948 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1452 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1644 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1848 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2292 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 684 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1116 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1284 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1656 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1860 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2076 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2304 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2316 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 0, 10 );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 0, 11 );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 0, 12 );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 0, 13 );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 0, 14 );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 0, 15 );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 0, 16 );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 0, 17 );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 0, 18 );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 0, 19 );
acado_setBlockH11( 0, 19, &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 672 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 804 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 948 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1104 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1272 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1452 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1644 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1848 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2064 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2292 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 684 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1116 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1284 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1656 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1860 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2076 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2304 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2316 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 1, 10 );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 1, 11 );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 1, 12 );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 1, 13 );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 1, 14 );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 1, 15 );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 1, 16 );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 1, 17 );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 1, 18 );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 1, 19 );
acado_setBlockH11( 1, 19, &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 684 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 816 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 960 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1116 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1284 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1464 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1656 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1860 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2076 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2304 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2316 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 2, 10 );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 2, 11 );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 2, 12 );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 2, 13 );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 2, 14 );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 2, 15 );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 2, 16 );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 2, 17 );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 2, 18 );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 2, 19 );
acado_setBlockH11( 2, 19, &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 696 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 828 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1128 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1476 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1668 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1872 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2088 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2316 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 3, 10 );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 3, 11 );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 3, 12 );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 3, 13 );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 3, 14 );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 3, 15 );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 3, 16 );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 3, 17 );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 3, 18 );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 3, 19 );
acado_setBlockH11( 3, 19, &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 708 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 840 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 984 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1140 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1308 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1488 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1680 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1884 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2100 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2328 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 4, 10 );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 4, 11 );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 4, 12 );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 4, 13 );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 4, 14 );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 4, 15 );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 4, 16 );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 4, 17 );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 4, 18 );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 4, 19 );
acado_setBlockH11( 4, 19, &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 720 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 852 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 996 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1152 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1320 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1500 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1692 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1896 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2112 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2340 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 5, 10 );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 5, 11 );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 5, 12 );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 5, 13 );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 5, 14 );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 5, 15 );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 5, 16 );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 5, 17 );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 5, 18 );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 5, 19 );
acado_setBlockH11( 5, 19, &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 732 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1008 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1164 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1332 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1512 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1704 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1908 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2124 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2352 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 6, 10 );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 6, 11 );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 6, 12 );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 6, 13 );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 6, 14 );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 6, 15 );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 6, 16 );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 6, 17 );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 6, 18 );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 6, 19 );
acado_setBlockH11( 6, 19, &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 744 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 876 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1020 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1176 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1344 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1524 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1716 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1920 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2136 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2364 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 7, 10 );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 7, 11 );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 7, 12 );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 7, 13 );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 7, 14 );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 7, 15 );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 7, 16 );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 7, 17 );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 7, 18 );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 7, 19 );
acado_setBlockH11( 7, 19, &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QE[ 636 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 888 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1032 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1356 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1536 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1728 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1932 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2148 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2376 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 8, 10 );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 8, 11 );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 8, 12 );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 8, 13 );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 8, 14 );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 8, 15 );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 8, 16 );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 8, 17 );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 8, 18 );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 8, 19 );
acado_setBlockH11( 8, 19, &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 768 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 900 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1044 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1200 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1368 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1548 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1740 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1944 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2160 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2388 ]) );

acado_zeroBlockH11( 9, 10 );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 9, 11 );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 9, 12 );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 9, 13 );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 9, 14 );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 9, 15 );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 9, 16 );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 9, 17 );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 9, 18 );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 9, 19 );
acado_setBlockH11( 9, 19, &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 10, 10 );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QE[ 780 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 912 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1056 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1212 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1380 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1560 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1752 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1956 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2172 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2400 ]) );

acado_zeroBlockH11( 10, 11 );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 10, 12 );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 10, 13 );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 10, 14 );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 10, 15 );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 10, 16 );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 10, 17 );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 10, 18 );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 10, 19 );
acado_setBlockH11( 10, 19, &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 11, 11 );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.QE[ 924 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.QE[ 1068 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1224 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1392 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1572 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1764 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1968 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2184 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2412 ]) );

acado_zeroBlockH11( 11, 12 );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 11, 13 );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 11, 14 );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 11, 15 );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 11, 16 );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 11, 17 );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 11, 18 );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 11, 19 );
acado_setBlockH11( 11, 19, &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 12, 12 );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.QE[ 1236 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1584 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1776 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 1980 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2196 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2424 ]) );

acado_zeroBlockH11( 12, 13 );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 12, 14 );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 12, 15 );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 12, 16 );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 12, 17 );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 12, 18 );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 12, 19 );
acado_setBlockH11( 12, 19, &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 13, 13 );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QE[ 1248 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1416 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QE[ 1596 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1788 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 1992 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2208 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2436 ]) );

acado_zeroBlockH11( 13, 14 );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 13, 15 );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 13, 16 );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 13, 17 );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 13, 18 );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 13, 19 );
acado_setBlockH11( 13, 19, &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 14, 14 );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.QE[ 1428 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1608 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QE[ 1800 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2004 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2220 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2448 ]) );

acado_zeroBlockH11( 14, 15 );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 14, 16 );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 14, 17 );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 14, 18 );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 14, 19 );
acado_setBlockH11( 14, 19, &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 15, 15 );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.QE[ 1620 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.QE[ 1812 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2016 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2232 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2460 ]) );

acado_zeroBlockH11( 15, 16 );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 15, 17 );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 15, 18 );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 15, 19 );
acado_setBlockH11( 15, 19, &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 16, 16 );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QE[ 1824 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.QE[ 2028 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QE[ 2244 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2472 ]) );

acado_zeroBlockH11( 16, 17 );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 16, 18 );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 16, 19 );
acado_setBlockH11( 16, 19, &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 17, 17 );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.QE[ 2040 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2256 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QE[ 2484 ]) );

acado_zeroBlockH11( 17, 18 );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 17, 19 );
acado_setBlockH11( 17, 19, &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 18, 18 );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.QE[ 2268 ]) );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2496 ]) );

acado_zeroBlockH11( 18, 19 );
acado_setBlockH11( 18, 19, &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QE[ 2508 ]) );

acado_setBlockH11_R1( 19, 19 );
acado_setBlockH11( 19, 19, &(acadoWorkspace.E[ 2508 ]), &(acadoWorkspace.QE[ 2508 ]) );


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
acado_copyHTH( 10, 0 );
acado_copyHTH( 10, 1 );
acado_copyHTH( 10, 2 );
acado_copyHTH( 10, 3 );
acado_copyHTH( 10, 4 );
acado_copyHTH( 10, 5 );
acado_copyHTH( 10, 6 );
acado_copyHTH( 10, 7 );
acado_copyHTH( 10, 8 );
acado_copyHTH( 10, 9 );
acado_copyHTH( 11, 0 );
acado_copyHTH( 11, 1 );
acado_copyHTH( 11, 2 );
acado_copyHTH( 11, 3 );
acado_copyHTH( 11, 4 );
acado_copyHTH( 11, 5 );
acado_copyHTH( 11, 6 );
acado_copyHTH( 11, 7 );
acado_copyHTH( 11, 8 );
acado_copyHTH( 11, 9 );
acado_copyHTH( 11, 10 );
acado_copyHTH( 12, 0 );
acado_copyHTH( 12, 1 );
acado_copyHTH( 12, 2 );
acado_copyHTH( 12, 3 );
acado_copyHTH( 12, 4 );
acado_copyHTH( 12, 5 );
acado_copyHTH( 12, 6 );
acado_copyHTH( 12, 7 );
acado_copyHTH( 12, 8 );
acado_copyHTH( 12, 9 );
acado_copyHTH( 12, 10 );
acado_copyHTH( 12, 11 );
acado_copyHTH( 13, 0 );
acado_copyHTH( 13, 1 );
acado_copyHTH( 13, 2 );
acado_copyHTH( 13, 3 );
acado_copyHTH( 13, 4 );
acado_copyHTH( 13, 5 );
acado_copyHTH( 13, 6 );
acado_copyHTH( 13, 7 );
acado_copyHTH( 13, 8 );
acado_copyHTH( 13, 9 );
acado_copyHTH( 13, 10 );
acado_copyHTH( 13, 11 );
acado_copyHTH( 13, 12 );
acado_copyHTH( 14, 0 );
acado_copyHTH( 14, 1 );
acado_copyHTH( 14, 2 );
acado_copyHTH( 14, 3 );
acado_copyHTH( 14, 4 );
acado_copyHTH( 14, 5 );
acado_copyHTH( 14, 6 );
acado_copyHTH( 14, 7 );
acado_copyHTH( 14, 8 );
acado_copyHTH( 14, 9 );
acado_copyHTH( 14, 10 );
acado_copyHTH( 14, 11 );
acado_copyHTH( 14, 12 );
acado_copyHTH( 14, 13 );
acado_copyHTH( 15, 0 );
acado_copyHTH( 15, 1 );
acado_copyHTH( 15, 2 );
acado_copyHTH( 15, 3 );
acado_copyHTH( 15, 4 );
acado_copyHTH( 15, 5 );
acado_copyHTH( 15, 6 );
acado_copyHTH( 15, 7 );
acado_copyHTH( 15, 8 );
acado_copyHTH( 15, 9 );
acado_copyHTH( 15, 10 );
acado_copyHTH( 15, 11 );
acado_copyHTH( 15, 12 );
acado_copyHTH( 15, 13 );
acado_copyHTH( 15, 14 );
acado_copyHTH( 16, 0 );
acado_copyHTH( 16, 1 );
acado_copyHTH( 16, 2 );
acado_copyHTH( 16, 3 );
acado_copyHTH( 16, 4 );
acado_copyHTH( 16, 5 );
acado_copyHTH( 16, 6 );
acado_copyHTH( 16, 7 );
acado_copyHTH( 16, 8 );
acado_copyHTH( 16, 9 );
acado_copyHTH( 16, 10 );
acado_copyHTH( 16, 11 );
acado_copyHTH( 16, 12 );
acado_copyHTH( 16, 13 );
acado_copyHTH( 16, 14 );
acado_copyHTH( 16, 15 );
acado_copyHTH( 17, 0 );
acado_copyHTH( 17, 1 );
acado_copyHTH( 17, 2 );
acado_copyHTH( 17, 3 );
acado_copyHTH( 17, 4 );
acado_copyHTH( 17, 5 );
acado_copyHTH( 17, 6 );
acado_copyHTH( 17, 7 );
acado_copyHTH( 17, 8 );
acado_copyHTH( 17, 9 );
acado_copyHTH( 17, 10 );
acado_copyHTH( 17, 11 );
acado_copyHTH( 17, 12 );
acado_copyHTH( 17, 13 );
acado_copyHTH( 17, 14 );
acado_copyHTH( 17, 15 );
acado_copyHTH( 17, 16 );
acado_copyHTH( 18, 0 );
acado_copyHTH( 18, 1 );
acado_copyHTH( 18, 2 );
acado_copyHTH( 18, 3 );
acado_copyHTH( 18, 4 );
acado_copyHTH( 18, 5 );
acado_copyHTH( 18, 6 );
acado_copyHTH( 18, 7 );
acado_copyHTH( 18, 8 );
acado_copyHTH( 18, 9 );
acado_copyHTH( 18, 10 );
acado_copyHTH( 18, 11 );
acado_copyHTH( 18, 12 );
acado_copyHTH( 18, 13 );
acado_copyHTH( 18, 14 );
acado_copyHTH( 18, 15 );
acado_copyHTH( 18, 16 );
acado_copyHTH( 18, 17 );
acado_copyHTH( 19, 0 );
acado_copyHTH( 19, 1 );
acado_copyHTH( 19, 2 );
acado_copyHTH( 19, 3 );
acado_copyHTH( 19, 4 );
acado_copyHTH( 19, 5 );
acado_copyHTH( 19, 6 );
acado_copyHTH( 19, 7 );
acado_copyHTH( 19, 8 );
acado_copyHTH( 19, 9 );
acado_copyHTH( 19, 10 );
acado_copyHTH( 19, 11 );
acado_copyHTH( 19, 12 );
acado_copyHTH( 19, 13 );
acado_copyHTH( 19, 14 );
acado_copyHTH( 19, 15 );
acado_copyHTH( 19, 16 );
acado_copyHTH( 19, 17 );
acado_copyHTH( 19, 18 );

acado_multQ1d( acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.Qd[ 6 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.Qd[ 18 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.Qd[ 42 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.Qd[ 54 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 66 ]), &(acadoWorkspace.Qd[ 66 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 78 ]), &(acadoWorkspace.Qd[ 78 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.Qd[ 84 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.Qd[ 90 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.Qd[ 96 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 102 ]), &(acadoWorkspace.Qd[ 102 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.Qd[ 108 ]) );
acado_multQN1d( &(acadoWorkspace.d[ 114 ]), &(acadoWorkspace.Qd[ 114 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 12 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 36 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 252 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 336 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 540 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 660 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 792 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 936 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1092 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1260 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1440 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1632 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1836 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2052 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 2280 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 348 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 444 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 672 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 804 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 948 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1104 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1272 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1452 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1644 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1848 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2064 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2292 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 564 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 684 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 816 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 960 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1116 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1284 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1464 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1656 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1860 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2076 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2304 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 372 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 468 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 696 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 828 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 972 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1128 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1476 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1668 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1872 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2088 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2316 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 588 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 708 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 840 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 984 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1140 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1308 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1488 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1680 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1884 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2100 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2328 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 396 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 492 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 720 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 852 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 996 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1152 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1320 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1500 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1692 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1896 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2112 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2340 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 408 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 612 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 732 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1008 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1164 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1332 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1512 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1704 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1908 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2124 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2352 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 516 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 744 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 876 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1020 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1176 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1344 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1524 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1716 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1920 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2136 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2364 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 636 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 756 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 888 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1032 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1188 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1356 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1536 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1728 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1932 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2148 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2376 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 768 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 900 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1044 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1200 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1368 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1548 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1740 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1944 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2160 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2388 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 780 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 912 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1056 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1212 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1380 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1560 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1752 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1956 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2172 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2400 ]), &(acadoWorkspace.g[ 20 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 924 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1068 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1224 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1392 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1572 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1764 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1968 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2184 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2412 ]), &(acadoWorkspace.g[ 22 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1236 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1404 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1584 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1776 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1980 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2196 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2424 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1248 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1416 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1596 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1788 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1992 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2208 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2436 ]), &(acadoWorkspace.g[ 26 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1428 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1608 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1800 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2004 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2220 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2448 ]), &(acadoWorkspace.g[ 28 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1620 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1812 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2016 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2232 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2460 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1824 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2028 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2244 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2472 ]), &(acadoWorkspace.g[ 32 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2040 ]), &(acadoWorkspace.g[ 34 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2256 ]), &(acadoWorkspace.g[ 34 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2484 ]), &(acadoWorkspace.g[ 34 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2268 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2496 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 2508 ]), &(acadoWorkspace.g[ 38 ]) );
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
acadoWorkspace.lb[20] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-5.0000000000000000e+01 - acadoVariables.u[39];
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
acadoWorkspace.ub[20] = (real_t)5.0000000000000000e+01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)5.0000000000000000e+01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)5.0000000000000000e+01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)5.0000000000000000e+01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)5.0000000000000000e+01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)5.0000000000000000e+01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)5.0000000000000000e+01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)5.0000000000000000e+01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)5.0000000000000000e+01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)5.0000000000000000e+01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)5.0000000000000000e+01 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)5.0000000000000000e+01 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)5.0000000000000000e+01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)5.0000000000000000e+01 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)5.0000000000000000e+01 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)5.0000000000000000e+01 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)5.0000000000000000e+01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)5.0000000000000000e+01 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)5.0000000000000000e+01 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)5.0000000000000000e+01 - acadoVariables.u[39];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.x[lRun1 * 6 + 5];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[7] = acadoVariables.u[lRun1 * 2 + 1];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1 * 6] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun1 * 6 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 6 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 6 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 6 + 4] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 6 + 5] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHu[lRun1 * 2] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHu[lRun1 * 2 + 1] = acadoWorkspace.conValueOut[8];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];

acado_multHxC( &(acadoWorkspace.evHx[ 6 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 6 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.A01[ 12 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.A01[ 18 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.A01[ 24 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A01[ 30 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.A01[ 36 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.A01[ 42 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.A01[ 48 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.A01[ 54 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.A01[ 66 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.A01[ 72 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.A01[ 78 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.A01[ 84 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.A01[ 90 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.A01[ 96 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.A01[ 102 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.A01[ 108 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.A01[ 114 ]) );

acado_multHxE( &(acadoWorkspace.evHx[ 6 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.E[ 12 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.E[ 24 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.E[ 36 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.E[ 48 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.E[ 60 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 72 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 84 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 96 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 108 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.E[ 120 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.E[ 132 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.E[ 144 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.E[ 156 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.E[ 168 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 180 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 192 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 204 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 216 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 228 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 240 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.E[ 252 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.E[ 264 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.E[ 276 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.E[ 288 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.E[ 300 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.E[ 312 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.E[ 324 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 336 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 348 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 360 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 372 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 384 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 396 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 408 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 420 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.E[ 432 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.E[ 444 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.E[ 456 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.E[ 468 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.E[ 480 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.E[ 492 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.E[ 504 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.E[ 516 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.E[ 528 ]), 9, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 540 ]), 10, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 552 ]), 10, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 564 ]), 10, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 576 ]), 10, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 588 ]), 10, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 600 ]), 10, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 612 ]), 10, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 624 ]), 10, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 636 ]), 10, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 648 ]), 10, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 660 ]), 11, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 672 ]), 11, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 684 ]), 11, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 696 ]), 11, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 708 ]), 11, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 720 ]), 11, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 732 ]), 11, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 744 ]), 11, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 756 ]), 11, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 768 ]), 11, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.E[ 780 ]), 11, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 792 ]), 12, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 804 ]), 12, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 816 ]), 12, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 828 ]), 12, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 840 ]), 12, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 852 ]), 12, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 864 ]), 12, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 876 ]), 12, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 888 ]), 12, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 900 ]), 12, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 912 ]), 12, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 924 ]), 12, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 936 ]), 13, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 948 ]), 13, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 960 ]), 13, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 972 ]), 13, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 984 ]), 13, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 996 ]), 13, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 1008 ]), 13, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 1020 ]), 13, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 1032 ]), 13, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 1044 ]), 13, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 1056 ]), 13, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 1068 ]), 13, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.E[ 1080 ]), 13, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1092 ]), 14, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1104 ]), 14, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1116 ]), 14, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1128 ]), 14, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1140 ]), 14, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1152 ]), 14, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1164 ]), 14, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1176 ]), 14, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1188 ]), 14, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1200 ]), 14, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1212 ]), 14, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1224 ]), 14, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1236 ]), 14, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 1248 ]), 14, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1260 ]), 15, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1272 ]), 15, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1284 ]), 15, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1296 ]), 15, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1308 ]), 15, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1320 ]), 15, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1332 ]), 15, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1344 ]), 15, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1356 ]), 15, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1368 ]), 15, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1380 ]), 15, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1392 ]), 15, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1404 ]), 15, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1416 ]), 15, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 1428 ]), 15, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1440 ]), 16, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1452 ]), 16, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1464 ]), 16, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1476 ]), 16, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1488 ]), 16, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1500 ]), 16, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1512 ]), 16, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1524 ]), 16, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1536 ]), 16, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1548 ]), 16, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1560 ]), 16, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1572 ]), 16, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1584 ]), 16, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1596 ]), 16, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1608 ]), 16, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 1620 ]), 16, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1632 ]), 17, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1644 ]), 17, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1656 ]), 17, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1668 ]), 17, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1680 ]), 17, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1692 ]), 17, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1704 ]), 17, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1716 ]), 17, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1728 ]), 17, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1740 ]), 17, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1752 ]), 17, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1764 ]), 17, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1776 ]), 17, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1788 ]), 17, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1800 ]), 17, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1812 ]), 17, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.E[ 1824 ]), 17, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1836 ]), 18, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1848 ]), 18, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1860 ]), 18, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1872 ]), 18, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1884 ]), 18, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1896 ]), 18, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1908 ]), 18, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1920 ]), 18, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1932 ]), 18, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1944 ]), 18, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1956 ]), 18, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1968 ]), 18, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1980 ]), 18, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 1992 ]), 18, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 2004 ]), 18, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 2016 ]), 18, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 2028 ]), 18, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 2040 ]), 18, 17 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2052 ]), 19, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2064 ]), 19, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2076 ]), 19, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2088 ]), 19, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2100 ]), 19, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2112 ]), 19, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2124 ]), 19, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2136 ]), 19, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2148 ]), 19, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2160 ]), 19, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2172 ]), 19, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2184 ]), 19, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2196 ]), 19, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2208 ]), 19, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2220 ]), 19, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2232 ]), 19, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2244 ]), 19, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2256 ]), 19, 17 );
acado_multHxE( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.E[ 2268 ]), 19, 18 );

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[42] = acadoWorkspace.evHu[2];
acadoWorkspace.A[43] = acadoWorkspace.evHu[3];
acadoWorkspace.A[84] = acadoWorkspace.evHu[4];
acadoWorkspace.A[85] = acadoWorkspace.evHu[5];
acadoWorkspace.A[126] = acadoWorkspace.evHu[6];
acadoWorkspace.A[127] = acadoWorkspace.evHu[7];
acadoWorkspace.A[168] = acadoWorkspace.evHu[8];
acadoWorkspace.A[169] = acadoWorkspace.evHu[9];
acadoWorkspace.A[210] = acadoWorkspace.evHu[10];
acadoWorkspace.A[211] = acadoWorkspace.evHu[11];
acadoWorkspace.A[252] = acadoWorkspace.evHu[12];
acadoWorkspace.A[253] = acadoWorkspace.evHu[13];
acadoWorkspace.A[294] = acadoWorkspace.evHu[14];
acadoWorkspace.A[295] = acadoWorkspace.evHu[15];
acadoWorkspace.A[336] = acadoWorkspace.evHu[16];
acadoWorkspace.A[337] = acadoWorkspace.evHu[17];
acadoWorkspace.A[378] = acadoWorkspace.evHu[18];
acadoWorkspace.A[379] = acadoWorkspace.evHu[19];
acadoWorkspace.A[420] = acadoWorkspace.evHu[20];
acadoWorkspace.A[421] = acadoWorkspace.evHu[21];
acadoWorkspace.A[462] = acadoWorkspace.evHu[22];
acadoWorkspace.A[463] = acadoWorkspace.evHu[23];
acadoWorkspace.A[504] = acadoWorkspace.evHu[24];
acadoWorkspace.A[505] = acadoWorkspace.evHu[25];
acadoWorkspace.A[546] = acadoWorkspace.evHu[26];
acadoWorkspace.A[547] = acadoWorkspace.evHu[27];
acadoWorkspace.A[588] = acadoWorkspace.evHu[28];
acadoWorkspace.A[589] = acadoWorkspace.evHu[29];
acadoWorkspace.A[630] = acadoWorkspace.evHu[30];
acadoWorkspace.A[631] = acadoWorkspace.evHu[31];
acadoWorkspace.A[672] = acadoWorkspace.evHu[32];
acadoWorkspace.A[673] = acadoWorkspace.evHu[33];
acadoWorkspace.A[714] = acadoWorkspace.evHu[34];
acadoWorkspace.A[715] = acadoWorkspace.evHu[35];
acadoWorkspace.A[756] = acadoWorkspace.evHu[36];
acadoWorkspace.A[757] = acadoWorkspace.evHu[37];
acadoWorkspace.A[798] = acadoWorkspace.evHu[38];
acadoWorkspace.A[799] = acadoWorkspace.evHu[39];
acadoWorkspace.lbA[0] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = (real_t)1.0000000000000000e+00 - acadoWorkspace.evH[19];

acadoWorkspace.ubA[0] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[19];

acado_macHxd( &(acadoWorkspace.evHx[ 6 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 1 ]), &(acadoWorkspace.ubA[ 1 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.lbA[ 5 ]), &(acadoWorkspace.ubA[ 5 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 42 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.lbA[ 7 ]), &(acadoWorkspace.ubA[ 7 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 54 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 66 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 11 ]), &(acadoWorkspace.ubA[ 11 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.d[ 66 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 78 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.lbA[ 13 ]), &(acadoWorkspace.ubA[ 13 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.d[ 78 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.d[ 90 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 102 ]), &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.lbA[ 17 ]), &(acadoWorkspace.ubA[ 17 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.d[ 102 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 114 ]), &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.lbA[ 19 ]), &(acadoWorkspace.ubA[ 19 ]) );

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];

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
acadoWorkspace.Dy[100] -= acadoVariables.y[100];
acadoWorkspace.Dy[101] -= acadoVariables.y[101];
acadoWorkspace.Dy[102] -= acadoVariables.y[102];
acadoWorkspace.Dy[103] -= acadoVariables.y[103];
acadoWorkspace.Dy[104] -= acadoVariables.y[104];
acadoWorkspace.Dy[105] -= acadoVariables.y[105];
acadoWorkspace.Dy[106] -= acadoVariables.y[106];
acadoWorkspace.Dy[107] -= acadoVariables.y[107];
acadoWorkspace.Dy[108] -= acadoVariables.y[108];
acadoWorkspace.Dy[109] -= acadoVariables.y[109];
acadoWorkspace.Dy[110] -= acadoVariables.y[110];
acadoWorkspace.Dy[111] -= acadoVariables.y[111];
acadoWorkspace.Dy[112] -= acadoVariables.y[112];
acadoWorkspace.Dy[113] -= acadoVariables.y[113];
acadoWorkspace.Dy[114] -= acadoVariables.y[114];
acadoWorkspace.Dy[115] -= acadoVariables.y[115];
acadoWorkspace.Dy[116] -= acadoVariables.y[116];
acadoWorkspace.Dy[117] -= acadoVariables.y[117];
acadoWorkspace.Dy[118] -= acadoVariables.y[118];
acadoWorkspace.Dy[119] -= acadoVariables.y[119];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.g[ 38 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 66 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 102 ]), &(acadoWorkspace.QDy[ 102 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.QDy[ 114 ]) );

acadoWorkspace.QDy[120] = + (real_t)1.0000000000000000e+06*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[121] = + (real_t)1.0000000000000000e+06*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[122] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[123] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[124] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[4];
acadoWorkspace.QDy[125] = + (real_t)9.9999999999999995e-07*acadoWorkspace.DyN[5];

acadoWorkspace.QDy[6] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[84] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[85] += acadoWorkspace.Qd[79];
acadoWorkspace.QDy[86] += acadoWorkspace.Qd[80];
acadoWorkspace.QDy[87] += acadoWorkspace.Qd[81];
acadoWorkspace.QDy[88] += acadoWorkspace.Qd[82];
acadoWorkspace.QDy[89] += acadoWorkspace.Qd[83];
acadoWorkspace.QDy[90] += acadoWorkspace.Qd[84];
acadoWorkspace.QDy[91] += acadoWorkspace.Qd[85];
acadoWorkspace.QDy[92] += acadoWorkspace.Qd[86];
acadoWorkspace.QDy[93] += acadoWorkspace.Qd[87];
acadoWorkspace.QDy[94] += acadoWorkspace.Qd[88];
acadoWorkspace.QDy[95] += acadoWorkspace.Qd[89];
acadoWorkspace.QDy[96] += acadoWorkspace.Qd[90];
acadoWorkspace.QDy[97] += acadoWorkspace.Qd[91];
acadoWorkspace.QDy[98] += acadoWorkspace.Qd[92];
acadoWorkspace.QDy[99] += acadoWorkspace.Qd[93];
acadoWorkspace.QDy[100] += acadoWorkspace.Qd[94];
acadoWorkspace.QDy[101] += acadoWorkspace.Qd[95];
acadoWorkspace.QDy[102] += acadoWorkspace.Qd[96];
acadoWorkspace.QDy[103] += acadoWorkspace.Qd[97];
acadoWorkspace.QDy[104] += acadoWorkspace.Qd[98];
acadoWorkspace.QDy[105] += acadoWorkspace.Qd[99];
acadoWorkspace.QDy[106] += acadoWorkspace.Qd[100];
acadoWorkspace.QDy[107] += acadoWorkspace.Qd[101];
acadoWorkspace.QDy[108] += acadoWorkspace.Qd[102];
acadoWorkspace.QDy[109] += acadoWorkspace.Qd[103];
acadoWorkspace.QDy[110] += acadoWorkspace.Qd[104];
acadoWorkspace.QDy[111] += acadoWorkspace.Qd[105];
acadoWorkspace.QDy[112] += acadoWorkspace.Qd[106];
acadoWorkspace.QDy[113] += acadoWorkspace.Qd[107];
acadoWorkspace.QDy[114] += acadoWorkspace.Qd[108];
acadoWorkspace.QDy[115] += acadoWorkspace.Qd[109];
acadoWorkspace.QDy[116] += acadoWorkspace.Qd[110];
acadoWorkspace.QDy[117] += acadoWorkspace.Qd[111];
acadoWorkspace.QDy[118] += acadoWorkspace.Qd[112];
acadoWorkspace.QDy[119] += acadoWorkspace.Qd[113];
acadoWorkspace.QDy[120] += acadoWorkspace.Qd[114];
acadoWorkspace.QDy[121] += acadoWorkspace.Qd[115];
acadoWorkspace.QDy[122] += acadoWorkspace.Qd[116];
acadoWorkspace.QDy[123] += acadoWorkspace.Qd[117];
acadoWorkspace.QDy[124] += acadoWorkspace.Qd[118];
acadoWorkspace.QDy[125] += acadoWorkspace.Qd[119];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 660 ]), &(acadoWorkspace.QDy[ 66 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 792 ]), &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 936 ]), &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1092 ]), &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1260 ]), &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1440 ]), &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1632 ]), &(acadoWorkspace.QDy[ 102 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.QDy[ 114 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 2280 ]), &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.QDy[ 66 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QDy[ 78 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.QDy[ 84 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.QDy[ 96 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.QDy[ 102 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.QDy[ 114 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 2508 ]), &(acadoWorkspace.QDy[ 120 ]), &(acadoWorkspace.g[ 38 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[1] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[2] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[3] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[4] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[5] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[6] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[7] += + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[8] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[9] += + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[10] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[11] += + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[12] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[13] += + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[14] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[15] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[16] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[17] += + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[18] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[19] += + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[20] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[21] += + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[22] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[23] += + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[24] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[25] += + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[26] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[27] += + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[28] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[29] += + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[30] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[31] += + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[32] += + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[33] += + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[34] += + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[35] += + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[36] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[37] += + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[38] += + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[39] += + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[5];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[5];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[5];
acadoWorkspace.lbA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[19] -= acadoWorkspace.pacA01Dx0[19];

acadoWorkspace.ubA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[19] -= acadoWorkspace.pacA01Dx0[19];

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
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];
acadoVariables.x[5] += acadoWorkspace.Dx0[5];

acadoVariables.x[6] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[0];
acadoVariables.x[7] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[1];
acadoVariables.x[8] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[2];
acadoVariables.x[9] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[3];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[4];
acadoVariables.x[11] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[5];
acadoVariables.x[12] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[6];
acadoVariables.x[13] += + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[7];
acadoVariables.x[14] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[8];
acadoVariables.x[15] += + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[9];
acadoVariables.x[16] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[10];
acadoVariables.x[17] += + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[11];
acadoVariables.x[18] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[12];
acadoVariables.x[19] += + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[13];
acadoVariables.x[20] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[14];
acadoVariables.x[21] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[15];
acadoVariables.x[22] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[16];
acadoVariables.x[23] += + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[17];
acadoVariables.x[24] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[18];
acadoVariables.x[25] += + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[19];
acadoVariables.x[26] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[20];
acadoVariables.x[27] += + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[21];
acadoVariables.x[28] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[22];
acadoVariables.x[29] += + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[23];
acadoVariables.x[30] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[24];
acadoVariables.x[31] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[25];
acadoVariables.x[32] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[26];
acadoVariables.x[33] += + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[27];
acadoVariables.x[34] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[28];
acadoVariables.x[35] += + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[29];
acadoVariables.x[36] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[30];
acadoVariables.x[37] += + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[31];
acadoVariables.x[38] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[32];
acadoVariables.x[39] += + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[33];
acadoVariables.x[40] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[34];
acadoVariables.x[41] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[35];
acadoVariables.x[42] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[36];
acadoVariables.x[43] += + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[37];
acadoVariables.x[44] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[38];
acadoVariables.x[45] += + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[39];
acadoVariables.x[46] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[40];
acadoVariables.x[47] += + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[41];
acadoVariables.x[48] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[42];
acadoVariables.x[49] += + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[43];
acadoVariables.x[50] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[44];
acadoVariables.x[51] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[45];
acadoVariables.x[52] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[46];
acadoVariables.x[53] += + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[47];
acadoVariables.x[54] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[48];
acadoVariables.x[55] += + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[49];
acadoVariables.x[56] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[50];
acadoVariables.x[57] += + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[51];
acadoVariables.x[58] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[52];
acadoVariables.x[59] += + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[53];
acadoVariables.x[60] += + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[54];
acadoVariables.x[61] += + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[55];
acadoVariables.x[62] += + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[56];
acadoVariables.x[63] += + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[57];
acadoVariables.x[64] += + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[58];
acadoVariables.x[65] += + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[59];
acadoVariables.x[66] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[60];
acadoVariables.x[67] += + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[61];
acadoVariables.x[68] += + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[62];
acadoVariables.x[69] += + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[63];
acadoVariables.x[70] += + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[64];
acadoVariables.x[71] += + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[65];
acadoVariables.x[72] += + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[66];
acadoVariables.x[73] += + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[67];
acadoVariables.x[74] += + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[68];
acadoVariables.x[75] += + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[69];
acadoVariables.x[76] += + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[70];
acadoVariables.x[77] += + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[71];
acadoVariables.x[78] += + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[72];
acadoVariables.x[79] += + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[73];
acadoVariables.x[80] += + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[74];
acadoVariables.x[81] += + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[75];
acadoVariables.x[82] += + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[76];
acadoVariables.x[83] += + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[77];
acadoVariables.x[84] += + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[78];
acadoVariables.x[85] += + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[79];
acadoVariables.x[86] += + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[80];
acadoVariables.x[87] += + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[81];
acadoVariables.x[88] += + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[82];
acadoVariables.x[89] += + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[83];
acadoVariables.x[90] += + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[84];
acadoVariables.x[91] += + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[85];
acadoVariables.x[92] += + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[86];
acadoVariables.x[93] += + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[87];
acadoVariables.x[94] += + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[88];
acadoVariables.x[95] += + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[89];
acadoVariables.x[96] += + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[90];
acadoVariables.x[97] += + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[550]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[551]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[91];
acadoVariables.x[98] += + acadoWorkspace.evGx[552]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[553]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[554]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[555]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[556]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[557]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[92];
acadoVariables.x[99] += + acadoWorkspace.evGx[558]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[559]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[560]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[561]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[562]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[563]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[93];
acadoVariables.x[100] += + acadoWorkspace.evGx[564]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[565]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[566]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[567]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[568]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[569]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[94];
acadoVariables.x[101] += + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[575]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[95];
acadoVariables.x[102] += + acadoWorkspace.evGx[576]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[577]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[578]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[579]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[580]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[581]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[96];
acadoVariables.x[103] += + acadoWorkspace.evGx[582]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[583]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[584]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[585]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[586]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[587]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[97];
acadoVariables.x[104] += + acadoWorkspace.evGx[588]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[589]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[590]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[591]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[592]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[593]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[98];
acadoVariables.x[105] += + acadoWorkspace.evGx[594]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[99];
acadoVariables.x[106] += + acadoWorkspace.evGx[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[604]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[605]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[100];
acadoVariables.x[107] += + acadoWorkspace.evGx[606]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[607]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[608]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[609]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[610]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[611]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[101];
acadoVariables.x[108] += + acadoWorkspace.evGx[612]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[613]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[614]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[615]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[616]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[617]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[102];
acadoVariables.x[109] += + acadoWorkspace.evGx[618]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[619]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[103];
acadoVariables.x[110] += + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[625]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[626]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[627]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[628]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[629]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[104];
acadoVariables.x[111] += + acadoWorkspace.evGx[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[632]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[633]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[634]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[635]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[105];
acadoVariables.x[112] += + acadoWorkspace.evGx[636]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[637]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[638]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[639]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[640]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[641]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[106];
acadoVariables.x[113] += + acadoWorkspace.evGx[642]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[643]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[644]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[645]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[646]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[647]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[107];
acadoVariables.x[114] += + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[650]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[651]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[652]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[653]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[108];
acadoVariables.x[115] += + acadoWorkspace.evGx[654]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[655]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[656]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[657]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[658]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[659]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[109];
acadoVariables.x[116] += + acadoWorkspace.evGx[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[664]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[665]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[110];
acadoVariables.x[117] += + acadoWorkspace.evGx[666]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[667]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[668]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[669]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[111];
acadoVariables.x[118] += + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[675]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[676]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[677]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[112];
acadoVariables.x[119] += + acadoWorkspace.evGx[678]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[679]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[680]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[681]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[682]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[683]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[113];
acadoVariables.x[120] += + acadoWorkspace.evGx[684]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[685]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[686]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[687]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[688]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[689]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[114];
acadoVariables.x[121] += + acadoWorkspace.evGx[690]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[691]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[692]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[693]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[694]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[695]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[115];
acadoVariables.x[122] += + acadoWorkspace.evGx[696]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[697]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[698]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[699]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[700]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[701]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[116];
acadoVariables.x[123] += + acadoWorkspace.evGx[702]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[703]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[704]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[705]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[706]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[707]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[117];
acadoVariables.x[124] += + acadoWorkspace.evGx[708]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[709]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[710]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[711]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[712]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[713]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[118];
acadoVariables.x[125] += + acadoWorkspace.evGx[714]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[715]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[716]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[717]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[718]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[719]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[119];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 12 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 36 ]), acadoWorkspace.x, &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), acadoWorkspace.x, &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), acadoWorkspace.x, &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 252 ]), acadoWorkspace.x, &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 336 ]), acadoWorkspace.x, &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 432 ]), acadoWorkspace.x, &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 540 ]), acadoWorkspace.x, &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 636 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 660 ]), acadoWorkspace.x, &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 672 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 684 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 696 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 708 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 720 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 732 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 744 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 768 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 780 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 66 ]) );
acado_multEDu( &(acadoWorkspace.E[ 792 ]), acadoWorkspace.x, &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 804 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 816 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 828 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 840 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 852 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 876 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 888 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 900 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 912 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 924 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 936 ]), acadoWorkspace.x, &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 948 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 960 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 984 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 996 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1008 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1020 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1032 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1044 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1056 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1068 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 78 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1092 ]), acadoWorkspace.x, &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1104 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1116 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1128 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1140 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1152 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1164 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1176 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1200 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1212 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1224 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1236 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1248 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 84 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1260 ]), acadoWorkspace.x, &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1272 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1284 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1308 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1320 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1332 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1344 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1356 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1368 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1380 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1392 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1416 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1428 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1440 ]), acadoWorkspace.x, &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1452 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1464 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1476 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1488 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1500 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1524 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1536 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1548 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1560 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1572 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1584 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1596 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1608 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 96 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1632 ]), acadoWorkspace.x, &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1644 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1656 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1668 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1680 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1692 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1704 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1716 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1740 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1752 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1764 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1776 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1788 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1800 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1812 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1824 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 102 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1836 ]), acadoWorkspace.x, &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1848 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1860 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1872 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1884 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1896 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1908 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1920 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1932 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1944 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1956 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1968 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1980 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1992 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2004 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2016 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2028 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2040 ]), &(acadoWorkspace.x[ 34 ]), &(acadoVariables.x[ 108 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2052 ]), acadoWorkspace.x, &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2064 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2076 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2088 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2100 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2112 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2124 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2136 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2148 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2172 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2184 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2196 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2208 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2220 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2232 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2244 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2256 ]), &(acadoWorkspace.x[ 34 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 114 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2280 ]), acadoWorkspace.x, &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2292 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2304 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2316 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2328 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2340 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2352 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2364 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2388 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2400 ]), &(acadoWorkspace.x[ 20 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2412 ]), &(acadoWorkspace.x[ 22 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2424 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2436 ]), &(acadoWorkspace.x[ 26 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2448 ]), &(acadoWorkspace.x[ 28 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2460 ]), &(acadoWorkspace.x[ 30 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2472 ]), &(acadoWorkspace.x[ 32 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.x[ 34 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2496 ]), &(acadoWorkspace.x[ 36 ]), &(acadoVariables.x[ 120 ]) );
acado_multEDu( &(acadoWorkspace.E[ 2508 ]), &(acadoWorkspace.x[ 38 ]), &(acadoVariables.x[ 120 ]) );
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
for (index = 0; index < 20; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 6];
acadoWorkspace.state[1] = acadoVariables.x[index * 6 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 6 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 6 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 6 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 6 + 5];
acadoWorkspace.state[54] = acadoVariables.u[index * 2];
acadoWorkspace.state[55] = acadoVariables.u[index * 2 + 1];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 6 + 6] = acadoWorkspace.state[0];
acadoVariables.x[index * 6 + 7] = acadoWorkspace.state[1];
acadoVariables.x[index * 6 + 8] = acadoWorkspace.state[2];
acadoVariables.x[index * 6 + 9] = acadoWorkspace.state[3];
acadoVariables.x[index * 6 + 10] = acadoWorkspace.state[4];
acadoVariables.x[index * 6 + 11] = acadoWorkspace.state[5];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 6] = acadoVariables.x[index * 6 + 6];
acadoVariables.x[index * 6 + 1] = acadoVariables.x[index * 6 + 7];
acadoVariables.x[index * 6 + 2] = acadoVariables.x[index * 6 + 8];
acadoVariables.x[index * 6 + 3] = acadoVariables.x[index * 6 + 9];
acadoVariables.x[index * 6 + 4] = acadoVariables.x[index * 6 + 10];
acadoVariables.x[index * 6 + 5] = acadoVariables.x[index * 6 + 11];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[120] = xEnd[0];
acadoVariables.x[121] = xEnd[1];
acadoVariables.x[122] = xEnd[2];
acadoVariables.x[123] = xEnd[3];
acadoVariables.x[124] = xEnd[4];
acadoVariables.x[125] = xEnd[5];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[120];
acadoWorkspace.state[1] = acadoVariables.x[121];
acadoWorkspace.state[2] = acadoVariables.x[122];
acadoWorkspace.state[3] = acadoVariables.x[123];
acadoWorkspace.state[4] = acadoVariables.x[124];
acadoWorkspace.state[5] = acadoVariables.x[125];
if (uEnd != 0)
{
acadoWorkspace.state[54] = uEnd[0];
acadoWorkspace.state[55] = uEnd[1];
}
else
{
acadoWorkspace.state[54] = acadoVariables.u[38];
acadoWorkspace.state[55] = acadoVariables.u[39];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[120] = acadoWorkspace.state[0];
acadoVariables.x[121] = acadoWorkspace.state[1];
acadoVariables.x[122] = acadoWorkspace.state[2];
acadoVariables.x[123] = acadoWorkspace.state[3];
acadoVariables.x[124] = acadoWorkspace.state[4];
acadoVariables.x[125] = acadoWorkspace.state[5];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[38] = uEnd[0];
acadoVariables.u[39] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39];
kkt = fabs( kkt );
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index + 40];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 6 */
real_t tmpDy[ 6 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[7] = acadoVariables.u[lRun1 * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 6] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 6];
acadoWorkspace.Dy[lRun1 * 6 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 6 + 1];
acadoWorkspace.Dy[lRun1 * 6 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 6 + 2];
acadoWorkspace.Dy[lRun1 * 6 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 6 + 3];
acadoWorkspace.Dy[lRun1 * 6 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 6 + 4];
acadoWorkspace.Dy[lRun1 * 6 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 6 + 5];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acadoWorkspace.objValueIn[3] = acadoVariables.x[123];
acadoWorkspace.objValueIn[4] = acadoVariables.x[124];
acadoWorkspace.objValueIn[5] = acadoVariables.x[125];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 6];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 6 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 6 + 2]*(real_t)9.9999999999999995e-07;
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 6 + 3]*(real_t)9.9999999999999995e-07;
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 6 + 4]*(real_t)9.9999999999999995e-07;
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 6 + 5]*(real_t)9.9999999999999995e-07;
objVal += + acadoWorkspace.Dy[lRun1 * 6]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 6 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 6 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 6 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 6 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 6 + 5]*tmpDy[5];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000000e+06;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)1.0000000000000000e+06;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)9.9999999999999995e-07;
tmpDyN[3] = + acadoWorkspace.DyN[3]*(real_t)9.9999999999999995e-07;
tmpDyN[4] = + acadoWorkspace.DyN[4]*(real_t)9.9999999999999995e-07;
tmpDyN[5] = + acadoWorkspace.DyN[5]*(real_t)9.9999999999999995e-07;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}

