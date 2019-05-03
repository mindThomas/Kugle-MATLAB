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


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 10;

/* Compute outputs: */
out[0] = ((real_t)(5.0000000000000000e-01)*xd[8]);
out[1] = ((real_t)(5.0000000000000000e-01)*xd[9]);
out[2] = xd[4];
out[3] = xd[5];
out[4] = ((real_t)(1.4528899999999993e+01)*xd[1]);
out[5] = ((real_t)(-1.4528899999999993e+01)*xd[0]);
out[6] = xd[7];
out[7] = u[2];
out[8] = u[0];
out[9] = u[1];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(5.0000000000000000e-01);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(5.0000000000000000e-01);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(1.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(1.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(1.4528899999999993e+01);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(-1.4528899999999993e+01);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(1.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(1.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(1.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(1.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim10_triangular( real_t* const A, real_t* const b )
{

b[9] = b[9]/A[99];
b[8] -= + A[89]*b[9];
b[8] = b[8]/A[88];
b[7] -= + A[79]*b[9];
b[7] -= + A[78]*b[8];
b[7] = b[7]/A[77];
b[6] -= + A[69]*b[9];
b[6] -= + A[68]*b[8];
b[6] -= + A[67]*b[7];
b[6] = b[6]/A[66];
b[5] -= + A[59]*b[9];
b[5] -= + A[58]*b[8];
b[5] -= + A[57]*b[7];
b[5] -= + A[56]*b[6];
b[5] = b[5]/A[55];
b[4] -= + A[49]*b[9];
b[4] -= + A[48]*b[8];
b[4] -= + A[47]*b[7];
b[4] -= + A[46]*b[6];
b[4] -= + A[45]*b[5];
b[4] = b[4]/A[44];
b[3] -= + A[39]*b[9];
b[3] -= + A[38]*b[8];
b[3] -= + A[37]*b[7];
b[3] -= + A[36]*b[6];
b[3] -= + A[35]*b[5];
b[3] -= + A[34]*b[4];
b[3] = b[3]/A[33];
b[2] -= + A[29]*b[9];
b[2] -= + A[28]*b[8];
b[2] -= + A[27]*b[7];
b[2] -= + A[26]*b[6];
b[2] -= + A[25]*b[5];
b[2] -= + A[24]*b[4];
b[2] -= + A[23]*b[3];
b[2] = b[2]/A[22];
b[1] -= + A[19]*b[9];
b[1] -= + A[18]*b[8];
b[1] -= + A[17]*b[7];
b[1] -= + A[16]*b[6];
b[1] -= + A[15]*b[5];
b[1] -= + A[14]*b[4];
b[1] -= + A[13]*b[3];
b[1] -= + A[12]*b[2];
b[1] = b[1]/A[11];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim10_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 10; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (9); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*10+i]);
	for( j=(i+1); j < 10; j++ ) {
		temp = fabs(A[j*10+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 10; ++k)
{
	acadoWorkspace.rk_dim10_swap = A[i*10+k];
	A[i*10+k] = A[indexMax*10+k];
	A[indexMax*10+k] = acadoWorkspace.rk_dim10_swap;
}
	acadoWorkspace.rk_dim10_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim10_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*10+i];
	for( j=i+1; j < 10; j++ ) {
		A[j*10+i] = -A[j*10+i]/A[i*10+i];
		for( k=i+1; k < 10; k++ ) {
			A[j*10+k] += A[j*10+i] * A[i*10+k];
		}
		b[j] += A[j*10+i] * b[i];
	}
}
det *= A[99];
det = fabs(det);
acado_solve_dim10_triangular( A, b );
return det;
}

void acado_solve_dim10_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim10_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim10_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim10_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim10_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim10_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim10_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim10_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim10_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim10_bPerm[8] = b[rk_perm[8]];
acadoWorkspace.rk_dim10_bPerm[9] = b[rk_perm[9]];
acadoWorkspace.rk_dim10_bPerm[1] += A[10]*acadoWorkspace.rk_dim10_bPerm[0];

acadoWorkspace.rk_dim10_bPerm[2] += A[20]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[2] += A[21]*acadoWorkspace.rk_dim10_bPerm[1];

acadoWorkspace.rk_dim10_bPerm[3] += A[30]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[3] += A[31]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[3] += A[32]*acadoWorkspace.rk_dim10_bPerm[2];

acadoWorkspace.rk_dim10_bPerm[4] += A[40]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[4] += A[41]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[4] += A[42]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[4] += A[43]*acadoWorkspace.rk_dim10_bPerm[3];

acadoWorkspace.rk_dim10_bPerm[5] += A[50]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[5] += A[51]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[5] += A[52]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[5] += A[53]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[5] += A[54]*acadoWorkspace.rk_dim10_bPerm[4];

acadoWorkspace.rk_dim10_bPerm[6] += A[60]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[6] += A[61]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[6] += A[62]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[6] += A[63]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[6] += A[64]*acadoWorkspace.rk_dim10_bPerm[4];
acadoWorkspace.rk_dim10_bPerm[6] += A[65]*acadoWorkspace.rk_dim10_bPerm[5];

acadoWorkspace.rk_dim10_bPerm[7] += A[70]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[7] += A[71]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[7] += A[72]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[7] += A[73]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[7] += A[74]*acadoWorkspace.rk_dim10_bPerm[4];
acadoWorkspace.rk_dim10_bPerm[7] += A[75]*acadoWorkspace.rk_dim10_bPerm[5];
acadoWorkspace.rk_dim10_bPerm[7] += A[76]*acadoWorkspace.rk_dim10_bPerm[6];

acadoWorkspace.rk_dim10_bPerm[8] += A[80]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[8] += A[81]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[8] += A[82]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[8] += A[83]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[8] += A[84]*acadoWorkspace.rk_dim10_bPerm[4];
acadoWorkspace.rk_dim10_bPerm[8] += A[85]*acadoWorkspace.rk_dim10_bPerm[5];
acadoWorkspace.rk_dim10_bPerm[8] += A[86]*acadoWorkspace.rk_dim10_bPerm[6];
acadoWorkspace.rk_dim10_bPerm[8] += A[87]*acadoWorkspace.rk_dim10_bPerm[7];

acadoWorkspace.rk_dim10_bPerm[9] += A[90]*acadoWorkspace.rk_dim10_bPerm[0];
acadoWorkspace.rk_dim10_bPerm[9] += A[91]*acadoWorkspace.rk_dim10_bPerm[1];
acadoWorkspace.rk_dim10_bPerm[9] += A[92]*acadoWorkspace.rk_dim10_bPerm[2];
acadoWorkspace.rk_dim10_bPerm[9] += A[93]*acadoWorkspace.rk_dim10_bPerm[3];
acadoWorkspace.rk_dim10_bPerm[9] += A[94]*acadoWorkspace.rk_dim10_bPerm[4];
acadoWorkspace.rk_dim10_bPerm[9] += A[95]*acadoWorkspace.rk_dim10_bPerm[5];
acadoWorkspace.rk_dim10_bPerm[9] += A[96]*acadoWorkspace.rk_dim10_bPerm[6];
acadoWorkspace.rk_dim10_bPerm[9] += A[97]*acadoWorkspace.rk_dim10_bPerm[7];
acadoWorkspace.rk_dim10_bPerm[9] += A[98]*acadoWorkspace.rk_dim10_bPerm[8];


acado_solve_dim10_triangular( A, acadoWorkspace.rk_dim10_bPerm );
b[0] = acadoWorkspace.rk_dim10_bPerm[0];
b[1] = acadoWorkspace.rk_dim10_bPerm[1];
b[2] = acadoWorkspace.rk_dim10_bPerm[2];
b[3] = acadoWorkspace.rk_dim10_bPerm[3];
b[4] = acadoWorkspace.rk_dim10_bPerm[4];
b[5] = acadoWorkspace.rk_dim10_bPerm[5];
b[6] = acadoWorkspace.rk_dim10_bPerm[6];
b[7] = acadoWorkspace.rk_dim10_bPerm[7];
b[8] = acadoWorkspace.rk_dim10_bPerm[8];
b[9] = acadoWorkspace.rk_dim10_bPerm[9];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 1.6666666666666666e-02 };


/* Fixed step size:0.0333333 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[10] = rk_eta[170];
acadoWorkspace.rk_xxx[11] = rk_eta[171];
acadoWorkspace.rk_xxx[12] = rk_eta[172];
acadoWorkspace.rk_xxx[13] = rk_eta[173];
acadoWorkspace.rk_xxx[14] = rk_eta[174];
acadoWorkspace.rk_xxx[15] = rk_eta[175];
acadoWorkspace.rk_xxx[16] = rk_eta[176];
acadoWorkspace.rk_xxx[17] = rk_eta[177];
acadoWorkspace.rk_xxx[18] = rk_eta[178];
acadoWorkspace.rk_xxx[19] = rk_eta[179];
acadoWorkspace.rk_xxx[20] = rk_eta[180];
acadoWorkspace.rk_xxx[21] = rk_eta[181];
acadoWorkspace.rk_xxx[22] = rk_eta[182];
acadoWorkspace.rk_xxx[23] = rk_eta[183];
acadoWorkspace.rk_xxx[24] = rk_eta[184];
acadoWorkspace.rk_xxx[25] = rk_eta[185];
acadoWorkspace.rk_xxx[26] = rk_eta[186];
acadoWorkspace.rk_xxx[27] = rk_eta[187];
acadoWorkspace.rk_xxx[28] = rk_eta[188];
acadoWorkspace.rk_xxx[29] = rk_eta[189];
acadoWorkspace.rk_xxx[30] = rk_eta[190];
acadoWorkspace.rk_xxx[31] = rk_eta[191];
acadoWorkspace.rk_xxx[32] = rk_eta[192];
acadoWorkspace.rk_xxx[33] = rk_eta[193];
acadoWorkspace.rk_xxx[34] = rk_eta[194];
acadoWorkspace.rk_xxx[35] = rk_eta[195];
acadoWorkspace.rk_xxx[36] = rk_eta[196];
acadoWorkspace.rk_xxx[37] = rk_eta[197];
acadoWorkspace.rk_xxx[38] = rk_eta[198];
acadoWorkspace.rk_xxx[39] = rk_eta[199];
acadoWorkspace.rk_xxx[40] = rk_eta[200];
acadoWorkspace.rk_xxx[41] = rk_eta[201];
acadoWorkspace.rk_xxx[42] = rk_eta[202];
acadoWorkspace.rk_xxx[43] = rk_eta[203];
acadoWorkspace.rk_xxx[44] = rk_eta[204];
acadoWorkspace.rk_xxx[45] = rk_eta[205];
acadoWorkspace.rk_xxx[46] = rk_eta[206];
acadoWorkspace.rk_xxx[47] = rk_eta[207];
acadoWorkspace.rk_xxx[48] = rk_eta[208];
acadoWorkspace.rk_xxx[49] = rk_eta[209];
acadoWorkspace.rk_xxx[50] = rk_eta[210];
acadoWorkspace.rk_xxx[51] = rk_eta[211];
acadoWorkspace.rk_xxx[52] = rk_eta[212];
acadoWorkspace.rk_xxx[53] = rk_eta[213];
acadoWorkspace.rk_xxx[54] = rk_eta[214];
acadoWorkspace.rk_xxx[55] = rk_eta[215];
acadoWorkspace.rk_xxx[56] = rk_eta[216];
acadoWorkspace.rk_xxx[57] = rk_eta[217];
acadoWorkspace.rk_xxx[58] = rk_eta[218];
acadoWorkspace.rk_xxx[59] = rk_eta[219];

for (run = 0; run < 3; ++run)
{
if( run > 0 ) {
for (i = 0; i < 10; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 16] = rk_eta[i * 10 + 10];
acadoWorkspace.rk_diffsPrev2[i * 16 + 1] = rk_eta[i * 10 + 11];
acadoWorkspace.rk_diffsPrev2[i * 16 + 2] = rk_eta[i * 10 + 12];
acadoWorkspace.rk_diffsPrev2[i * 16 + 3] = rk_eta[i * 10 + 13];
acadoWorkspace.rk_diffsPrev2[i * 16 + 4] = rk_eta[i * 10 + 14];
acadoWorkspace.rk_diffsPrev2[i * 16 + 5] = rk_eta[i * 10 + 15];
acadoWorkspace.rk_diffsPrev2[i * 16 + 6] = rk_eta[i * 10 + 16];
acadoWorkspace.rk_diffsPrev2[i * 16 + 7] = rk_eta[i * 10 + 17];
acadoWorkspace.rk_diffsPrev2[i * 16 + 8] = rk_eta[i * 10 + 18];
acadoWorkspace.rk_diffsPrev2[i * 16 + 9] = rk_eta[i * 10 + 19];
acadoWorkspace.rk_diffsPrev2[i * 16 + 10] = rk_eta[i * 6 + 110];
acadoWorkspace.rk_diffsPrev2[i * 16 + 11] = rk_eta[i * 6 + 111];
acadoWorkspace.rk_diffsPrev2[i * 16 + 12] = rk_eta[i * 6 + 112];
acadoWorkspace.rk_diffsPrev2[i * 16 + 13] = rk_eta[i * 6 + 113];
acadoWorkspace.rk_diffsPrev2[i * 16 + 14] = rk_eta[i * 6 + 114];
acadoWorkspace.rk_diffsPrev2[i * 16 + 15] = rk_eta[i * 6 + 115];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 10; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 160 ]) );
for (j = 0; j < 10; ++j)
{
tmp_index1 = (run1 * 10) + (j);
acadoWorkspace.rk_A[tmp_index1 * 10] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 8] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 9] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 9)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 10) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 10] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 10 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 10 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 10 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 10 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 10 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 10 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 10 + 7] = acadoWorkspace.rk_kkk[run1 + 7] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 10 + 8] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[8];
acadoWorkspace.rk_b[run1 * 10 + 9] = acadoWorkspace.rk_kkk[run1 + 9] - acadoWorkspace.rk_rhsTemp[9];
}
det = acado_solve_dim10_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 10];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 10 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 10 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 10 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 10 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 10 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 10 + 6];
acadoWorkspace.rk_kkk[j + 7] += acadoWorkspace.rk_b[j * 10 + 7];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 10 + 8];
acadoWorkspace.rk_kkk[j + 9] += acadoWorkspace.rk_b[j * 10 + 9];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 10; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 10] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 10 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 10 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 10 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 10 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 10 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 10 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 10 + 7] = acadoWorkspace.rk_kkk[run1 + 7] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 10 + 8] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[8];
acadoWorkspace.rk_b[run1 * 10 + 9] = acadoWorkspace.rk_kkk[run1 + 9] - acadoWorkspace.rk_rhsTemp[9];
}
acado_solve_dim10_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 10];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 10 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 10 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 10 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 10 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 10 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 10 + 6];
acadoWorkspace.rk_kkk[j + 7] += acadoWorkspace.rk_b[j * 10 + 7];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 10 + 8];
acadoWorkspace.rk_kkk[j + 9] += acadoWorkspace.rk_b[j * 10 + 9];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 10; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 160 ]) );
for (j = 0; j < 10; ++j)
{
tmp_index1 = (run1 * 10) + (j);
acadoWorkspace.rk_A[tmp_index1 * 10] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 8] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 8)];
acadoWorkspace.rk_A[tmp_index1 * 10 + 9] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 160) + (j * 16 + 9)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 10) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 10; ++run1)
{
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_b[i * 10] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1)];
acadoWorkspace.rk_b[i * 10 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1 + 16)];
acadoWorkspace.rk_b[i * 10 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1 + 32)];
acadoWorkspace.rk_b[i * 10 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1 + 48)];
acadoWorkspace.rk_b[i * 10 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1 + 64)];
acadoWorkspace.rk_b[i * 10 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1 + 80)];
acadoWorkspace.rk_b[i * 10 + 6] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1 + 96)];
acadoWorkspace.rk_b[i * 10 + 7] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1 + 112)];
acadoWorkspace.rk_b[i * 10 + 8] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1 + 128)];
acadoWorkspace.rk_b[i * 10 + 9] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (run1 + 144)];
}
if( 0 == run1 ) {
det = acado_solve_dim10_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
}
 else {
acado_solve_dim10_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 10];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 10 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 10 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 10 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 10 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 10 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 10 + 6];
acadoWorkspace.rk_diffK[i + 7] = acadoWorkspace.rk_b[i * 10 + 7];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 10 + 8];
acadoWorkspace.rk_diffK[i + 9] = acadoWorkspace.rk_b[i * 10 + 9];
}
for (i = 0; i < 10; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 16) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 16) + (run1)] += + acadoWorkspace.rk_diffK[i]*(real_t)3.3333333333333333e-02;
}
}
for (run1 = 0; run1 < 6; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 10; ++j)
{
tmp_index1 = (i * 10) + (j);
tmp_index2 = (run1) + (j * 16);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 160) + (tmp_index2 + 10)];
}
}
acado_solve_dim10_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim10_perm );
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 10];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 10 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 10 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 10 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 10 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 10 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 10 + 6];
acadoWorkspace.rk_diffK[i + 7] = acadoWorkspace.rk_b[i * 10 + 7];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 10 + 8];
acadoWorkspace.rk_diffK[i + 9] = acadoWorkspace.rk_b[i * 10 + 9];
}
for (i = 0; i < 10; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 16) + (run1 + 10)] = + acadoWorkspace.rk_diffK[i]*(real_t)3.3333333333333333e-02;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)3.3333333333333333e-02;
rk_eta[1] += + acadoWorkspace.rk_kkk[1]*(real_t)3.3333333333333333e-02;
rk_eta[2] += + acadoWorkspace.rk_kkk[2]*(real_t)3.3333333333333333e-02;
rk_eta[3] += + acadoWorkspace.rk_kkk[3]*(real_t)3.3333333333333333e-02;
rk_eta[4] += + acadoWorkspace.rk_kkk[4]*(real_t)3.3333333333333333e-02;
rk_eta[5] += + acadoWorkspace.rk_kkk[5]*(real_t)3.3333333333333333e-02;
rk_eta[6] += + acadoWorkspace.rk_kkk[6]*(real_t)3.3333333333333333e-02;
rk_eta[7] += + acadoWorkspace.rk_kkk[7]*(real_t)3.3333333333333333e-02;
rk_eta[8] += + acadoWorkspace.rk_kkk[8]*(real_t)3.3333333333333333e-02;
rk_eta[9] += + acadoWorkspace.rk_kkk[9]*(real_t)3.3333333333333333e-02;
if( run == 0 ) {
for (i = 0; i < 10; ++i)
{
for (j = 0; j < 10; ++j)
{
tmp_index2 = (j) + (i * 10);
rk_eta[tmp_index2 + 10] = acadoWorkspace.rk_diffsNew2[(i * 16) + (j)];
}
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 110] = acadoWorkspace.rk_diffsNew2[(i * 16) + (j + 10)];
}
}
}
else {
for (i = 0; i < 10; ++i)
{
for (j = 0; j < 10; ++j)
{
tmp_index2 = (j) + (i * 10);
rk_eta[tmp_index2 + 10] = + acadoWorkspace.rk_diffsNew2[i * 16]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 10] += + acadoWorkspace.rk_diffsNew2[i * 16 + 1]*acadoWorkspace.rk_diffsPrev2[j + 16];
rk_eta[tmp_index2 + 10] += + acadoWorkspace.rk_diffsNew2[i * 16 + 2]*acadoWorkspace.rk_diffsPrev2[j + 32];
rk_eta[tmp_index2 + 10] += + acadoWorkspace.rk_diffsNew2[i * 16 + 3]*acadoWorkspace.rk_diffsPrev2[j + 48];
rk_eta[tmp_index2 + 10] += + acadoWorkspace.rk_diffsNew2[i * 16 + 4]*acadoWorkspace.rk_diffsPrev2[j + 64];
rk_eta[tmp_index2 + 10] += + acadoWorkspace.rk_diffsNew2[i * 16 + 5]*acadoWorkspace.rk_diffsPrev2[j + 80];
rk_eta[tmp_index2 + 10] += + acadoWorkspace.rk_diffsNew2[i * 16 + 6]*acadoWorkspace.rk_diffsPrev2[j + 96];
rk_eta[tmp_index2 + 10] += + acadoWorkspace.rk_diffsNew2[i * 16 + 7]*acadoWorkspace.rk_diffsPrev2[j + 112];
rk_eta[tmp_index2 + 10] += + acadoWorkspace.rk_diffsNew2[i * 16 + 8]*acadoWorkspace.rk_diffsPrev2[j + 128];
rk_eta[tmp_index2 + 10] += + acadoWorkspace.rk_diffsNew2[i * 16 + 9]*acadoWorkspace.rk_diffsPrev2[j + 144];
}
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 110] = acadoWorkspace.rk_diffsNew2[(i * 16) + (j + 10)];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16]*acadoWorkspace.rk_diffsPrev2[j + 10];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16 + 1]*acadoWorkspace.rk_diffsPrev2[j + 26];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16 + 2]*acadoWorkspace.rk_diffsPrev2[j + 42];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16 + 3]*acadoWorkspace.rk_diffsPrev2[j + 58];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16 + 4]*acadoWorkspace.rk_diffsPrev2[j + 74];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16 + 5]*acadoWorkspace.rk_diffsPrev2[j + 90];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16 + 6]*acadoWorkspace.rk_diffsPrev2[j + 106];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16 + 7]*acadoWorkspace.rk_diffsPrev2[j + 122];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16 + 8]*acadoWorkspace.rk_diffsPrev2[j + 138];
rk_eta[tmp_index2 + 110] += + acadoWorkspace.rk_diffsNew2[i * 16 + 9]*acadoWorkspace.rk_diffsPrev2[j + 154];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 3.3333333333333331e-01;
}
for (i = 0; i < 10; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



