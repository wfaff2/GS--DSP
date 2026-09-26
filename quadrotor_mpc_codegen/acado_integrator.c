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


real_t rk_dim14_swap;

/** Column vector of size: 14 */
real_t rk_dim14_bPerm[ 14 ];

/** Column vector of size: 6 */
real_t auxVar[ 6 ];

real_t rk_ttt;

/** Row vector of size: 94 */
real_t rk_xxx[ 94 ];

/** Matrix of size: 7 x 2 (row major format) */
real_t rk_kkk[ 14 ];

/** Matrix of size: 14 x 14 (row major format) */
real_t rk_A[ 196 ];

/** Column vector of size: 14 */
real_t rk_b[ 14 ];

/** Row vector of size: 14 */
int rk_dim14_perm[ 14 ];

/** Column vector of size: 7 */
real_t rk_rhsTemp[ 7 ];

/** Matrix of size: 2 x 98 (row major format) */
real_t rk_diffsTemp2[ 196 ];

/** Matrix of size: 7 x 2 (row major format) */
real_t rk_diffK[ 14 ];

/** Matrix of size: 7 x 14 (row major format) */
real_t rk_diffsPrev2[ 98 ];

/** Matrix of size: 7 x 14 (row major format) */
real_t rk_diffsNew2[ 98 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim14_perm, rk_A, rk_b, rk_diffsPrev2, rk_diffsNew2, rk_diffsTemp2, rk_dim14_swap, rk_dim14_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 7;
const real_t* od = in + 14;

/* Compute outputs: */
out[0] = xd[3];
out[1] = xd[4];
out[2] = xd[5];
out[3] = (((u[0]-xd[3])/(real_t)(2.0000000000000001e-01))+(((((real_t)(-2.0000000000000000e+00)*((od[69]*xd[5])-(od[70]*xd[4])))+((real_t)(0.0000000000000000e+00)-((od[72]*xd[2])-(od[73]*xd[1]))))+((real_t)(0.0000000000000000e+00)-((od[69]*((od[68]*xd[1])-(od[69]*xd[0])))-(od[70]*((od[70]*xd[0])-(od[68]*xd[2]))))))+((real_t)(0.0000000000000000e+00)-od[74])));
out[4] = (((u[1]-xd[4])/(real_t)(2.0000000000000001e-01))+(((((real_t)(-2.0000000000000000e+00)*((od[70]*xd[3])-(od[68]*xd[5])))+((real_t)(0.0000000000000000e+00)-((od[73]*xd[0])-(od[71]*xd[2]))))+((real_t)(0.0000000000000000e+00)-((od[70]*((od[69]*xd[2])-(od[70]*xd[1])))-(od[68]*((od[68]*xd[1])-(od[69]*xd[0]))))))+((real_t)(0.0000000000000000e+00)-od[75])));
out[5] = (((u[2]-xd[5])/(real_t)(2.0000000000000001e-01))+(((((real_t)(-2.0000000000000000e+00)*((od[68]*xd[4])-(od[69]*xd[3])))+((real_t)(0.0000000000000000e+00)-((od[71]*xd[1])-(od[72]*xd[0]))))+((real_t)(0.0000000000000000e+00)-((od[68]*((od[70]*xd[0])-(od[68]*xd[2])))-(od[69]*((od[69]*xd[2])-(od[70]*xd[1]))))))+((real_t)(0.0000000000000000e+00)-od[76])));
out[6] = u[3];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 14;
/* Vector of auxiliary variables; number of elements: 6. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.0000000000000001e-01));
a[1] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.0000000000000001e-01));
a[2] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.0000000000000001e-01));
a[3] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.0000000000000001e-01));
a[4] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.0000000000000001e-01));
a[5] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.0000000000000001e-01));

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(1.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(1.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(1.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = ((real_t)(0.0000000000000000e+00)-((od[69]*((real_t)(0.0000000000000000e+00)-od[69]))-(od[70]*od[70])));
out[43] = (((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-od[73]))+((real_t)(0.0000000000000000e+00)-(od[69]*od[68])));
out[44] = (((real_t)(0.0000000000000000e+00)-od[72])+((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-(od[70]*((real_t)(0.0000000000000000e+00)-od[68])))));
out[45] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[0]);
out[46] = ((real_t)(-2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-od[70]));
out[47] = ((real_t)(-2.0000000000000000e+00)*od[69]);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = a[1];
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (((real_t)(0.0000000000000000e+00)-od[73])+((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-(od[68]*((real_t)(0.0000000000000000e+00)-od[69])))));
out[57] = ((real_t)(0.0000000000000000e+00)-((od[70]*((real_t)(0.0000000000000000e+00)-od[70]))-(od[68]*od[68])));
out[58] = (((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-od[71]))+((real_t)(0.0000000000000000e+00)-(od[70]*od[69])));
out[59] = ((real_t)(-2.0000000000000000e+00)*od[70]);
out[60] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[2]);
out[61] = ((real_t)(-2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-od[68]));
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = a[3];
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-od[72]))+((real_t)(0.0000000000000000e+00)-(od[68]*od[70])));
out[71] = (((real_t)(0.0000000000000000e+00)-od[71])+((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-(od[69]*((real_t)(0.0000000000000000e+00)-od[70])))));
out[72] = ((real_t)(0.0000000000000000e+00)-((od[68]*((real_t)(0.0000000000000000e+00)-od[68]))-(od[69]*od[69])));
out[73] = ((real_t)(-2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-od[69]));
out[74] = ((real_t)(-2.0000000000000000e+00)*od[68]);
out[75] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[4]);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = a[5];
out[80] = (real_t)(0.0000000000000000e+00);
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
out[94] = (real_t)(1.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim14_triangular( real_t* const A, real_t* const b )
{

b[13] = b[13]/A[195];
b[12] -= + A[181]*b[13];
b[12] = b[12]/A[180];
b[11] -= + A[167]*b[13];
b[11] -= + A[166]*b[12];
b[11] = b[11]/A[165];
b[10] -= + A[153]*b[13];
b[10] -= + A[152]*b[12];
b[10] -= + A[151]*b[11];
b[10] = b[10]/A[150];
b[9] -= + A[139]*b[13];
b[9] -= + A[138]*b[12];
b[9] -= + A[137]*b[11];
b[9] -= + A[136]*b[10];
b[9] = b[9]/A[135];
b[8] -= + A[125]*b[13];
b[8] -= + A[124]*b[12];
b[8] -= + A[123]*b[11];
b[8] -= + A[122]*b[10];
b[8] -= + A[121]*b[9];
b[8] = b[8]/A[120];
b[7] -= + A[111]*b[13];
b[7] -= + A[110]*b[12];
b[7] -= + A[109]*b[11];
b[7] -= + A[108]*b[10];
b[7] -= + A[107]*b[9];
b[7] -= + A[106]*b[8];
b[7] = b[7]/A[105];
b[6] -= + A[97]*b[13];
b[6] -= + A[96]*b[12];
b[6] -= + A[95]*b[11];
b[6] -= + A[94]*b[10];
b[6] -= + A[93]*b[9];
b[6] -= + A[92]*b[8];
b[6] -= + A[91]*b[7];
b[6] = b[6]/A[90];
b[5] -= + A[83]*b[13];
b[5] -= + A[82]*b[12];
b[5] -= + A[81]*b[11];
b[5] -= + A[80]*b[10];
b[5] -= + A[79]*b[9];
b[5] -= + A[78]*b[8];
b[5] -= + A[77]*b[7];
b[5] -= + A[76]*b[6];
b[5] = b[5]/A[75];
b[4] -= + A[69]*b[13];
b[4] -= + A[68]*b[12];
b[4] -= + A[67]*b[11];
b[4] -= + A[66]*b[10];
b[4] -= + A[65]*b[9];
b[4] -= + A[64]*b[8];
b[4] -= + A[63]*b[7];
b[4] -= + A[62]*b[6];
b[4] -= + A[61]*b[5];
b[4] = b[4]/A[60];
b[3] -= + A[55]*b[13];
b[3] -= + A[54]*b[12];
b[3] -= + A[53]*b[11];
b[3] -= + A[52]*b[10];
b[3] -= + A[51]*b[9];
b[3] -= + A[50]*b[8];
b[3] -= + A[49]*b[7];
b[3] -= + A[48]*b[6];
b[3] -= + A[47]*b[5];
b[3] -= + A[46]*b[4];
b[3] = b[3]/A[45];
b[2] -= + A[41]*b[13];
b[2] -= + A[40]*b[12];
b[2] -= + A[39]*b[11];
b[2] -= + A[38]*b[10];
b[2] -= + A[37]*b[9];
b[2] -= + A[36]*b[8];
b[2] -= + A[35]*b[7];
b[2] -= + A[34]*b[6];
b[2] -= + A[33]*b[5];
b[2] -= + A[32]*b[4];
b[2] -= + A[31]*b[3];
b[2] = b[2]/A[30];
b[1] -= + A[27]*b[13];
b[1] -= + A[26]*b[12];
b[1] -= + A[25]*b[11];
b[1] -= + A[24]*b[10];
b[1] -= + A[23]*b[9];
b[1] -= + A[22]*b[8];
b[1] -= + A[21]*b[7];
b[1] -= + A[20]*b[6];
b[1] -= + A[19]*b[5];
b[1] -= + A[18]*b[4];
b[1] -= + A[17]*b[3];
b[1] -= + A[16]*b[2];
b[1] = b[1]/A[15];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
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

real_t acado_solve_dim14_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 14; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (13); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*14+i]);
	for( j=(i+1); j < 14; j++ ) {
		temp = fabs(A[j*14+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 14; ++k)
{
	rk_dim14_swap = A[i*14+k];
	A[i*14+k] = A[indexMax*14+k];
	A[indexMax*14+k] = rk_dim14_swap;
}
	rk_dim14_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim14_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*14+i];
	for( j=i+1; j < 14; j++ ) {
		A[j*14+i] = -A[j*14+i]/A[i*14+i];
		for( k=i+1; k < 14; k++ ) {
			A[j*14+k] += A[j*14+i] * A[i*14+k];
		}
		b[j] += A[j*14+i] * b[i];
	}
}
det *= A[195];
det = fabs(det);
acado_solve_dim14_triangular( A, b );
return det;
}

void acado_solve_dim14_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim14_bPerm[0] = b[rk_perm[0]];
rk_dim14_bPerm[1] = b[rk_perm[1]];
rk_dim14_bPerm[2] = b[rk_perm[2]];
rk_dim14_bPerm[3] = b[rk_perm[3]];
rk_dim14_bPerm[4] = b[rk_perm[4]];
rk_dim14_bPerm[5] = b[rk_perm[5]];
rk_dim14_bPerm[6] = b[rk_perm[6]];
rk_dim14_bPerm[7] = b[rk_perm[7]];
rk_dim14_bPerm[8] = b[rk_perm[8]];
rk_dim14_bPerm[9] = b[rk_perm[9]];
rk_dim14_bPerm[10] = b[rk_perm[10]];
rk_dim14_bPerm[11] = b[rk_perm[11]];
rk_dim14_bPerm[12] = b[rk_perm[12]];
rk_dim14_bPerm[13] = b[rk_perm[13]];
rk_dim14_bPerm[1] += A[14]*rk_dim14_bPerm[0];

rk_dim14_bPerm[2] += A[28]*rk_dim14_bPerm[0];
rk_dim14_bPerm[2] += A[29]*rk_dim14_bPerm[1];

rk_dim14_bPerm[3] += A[42]*rk_dim14_bPerm[0];
rk_dim14_bPerm[3] += A[43]*rk_dim14_bPerm[1];
rk_dim14_bPerm[3] += A[44]*rk_dim14_bPerm[2];

rk_dim14_bPerm[4] += A[56]*rk_dim14_bPerm[0];
rk_dim14_bPerm[4] += A[57]*rk_dim14_bPerm[1];
rk_dim14_bPerm[4] += A[58]*rk_dim14_bPerm[2];
rk_dim14_bPerm[4] += A[59]*rk_dim14_bPerm[3];

rk_dim14_bPerm[5] += A[70]*rk_dim14_bPerm[0];
rk_dim14_bPerm[5] += A[71]*rk_dim14_bPerm[1];
rk_dim14_bPerm[5] += A[72]*rk_dim14_bPerm[2];
rk_dim14_bPerm[5] += A[73]*rk_dim14_bPerm[3];
rk_dim14_bPerm[5] += A[74]*rk_dim14_bPerm[4];

rk_dim14_bPerm[6] += A[84]*rk_dim14_bPerm[0];
rk_dim14_bPerm[6] += A[85]*rk_dim14_bPerm[1];
rk_dim14_bPerm[6] += A[86]*rk_dim14_bPerm[2];
rk_dim14_bPerm[6] += A[87]*rk_dim14_bPerm[3];
rk_dim14_bPerm[6] += A[88]*rk_dim14_bPerm[4];
rk_dim14_bPerm[6] += A[89]*rk_dim14_bPerm[5];

rk_dim14_bPerm[7] += A[98]*rk_dim14_bPerm[0];
rk_dim14_bPerm[7] += A[99]*rk_dim14_bPerm[1];
rk_dim14_bPerm[7] += A[100]*rk_dim14_bPerm[2];
rk_dim14_bPerm[7] += A[101]*rk_dim14_bPerm[3];
rk_dim14_bPerm[7] += A[102]*rk_dim14_bPerm[4];
rk_dim14_bPerm[7] += A[103]*rk_dim14_bPerm[5];
rk_dim14_bPerm[7] += A[104]*rk_dim14_bPerm[6];

rk_dim14_bPerm[8] += A[112]*rk_dim14_bPerm[0];
rk_dim14_bPerm[8] += A[113]*rk_dim14_bPerm[1];
rk_dim14_bPerm[8] += A[114]*rk_dim14_bPerm[2];
rk_dim14_bPerm[8] += A[115]*rk_dim14_bPerm[3];
rk_dim14_bPerm[8] += A[116]*rk_dim14_bPerm[4];
rk_dim14_bPerm[8] += A[117]*rk_dim14_bPerm[5];
rk_dim14_bPerm[8] += A[118]*rk_dim14_bPerm[6];
rk_dim14_bPerm[8] += A[119]*rk_dim14_bPerm[7];

rk_dim14_bPerm[9] += A[126]*rk_dim14_bPerm[0];
rk_dim14_bPerm[9] += A[127]*rk_dim14_bPerm[1];
rk_dim14_bPerm[9] += A[128]*rk_dim14_bPerm[2];
rk_dim14_bPerm[9] += A[129]*rk_dim14_bPerm[3];
rk_dim14_bPerm[9] += A[130]*rk_dim14_bPerm[4];
rk_dim14_bPerm[9] += A[131]*rk_dim14_bPerm[5];
rk_dim14_bPerm[9] += A[132]*rk_dim14_bPerm[6];
rk_dim14_bPerm[9] += A[133]*rk_dim14_bPerm[7];
rk_dim14_bPerm[9] += A[134]*rk_dim14_bPerm[8];

rk_dim14_bPerm[10] += A[140]*rk_dim14_bPerm[0];
rk_dim14_bPerm[10] += A[141]*rk_dim14_bPerm[1];
rk_dim14_bPerm[10] += A[142]*rk_dim14_bPerm[2];
rk_dim14_bPerm[10] += A[143]*rk_dim14_bPerm[3];
rk_dim14_bPerm[10] += A[144]*rk_dim14_bPerm[4];
rk_dim14_bPerm[10] += A[145]*rk_dim14_bPerm[5];
rk_dim14_bPerm[10] += A[146]*rk_dim14_bPerm[6];
rk_dim14_bPerm[10] += A[147]*rk_dim14_bPerm[7];
rk_dim14_bPerm[10] += A[148]*rk_dim14_bPerm[8];
rk_dim14_bPerm[10] += A[149]*rk_dim14_bPerm[9];

rk_dim14_bPerm[11] += A[154]*rk_dim14_bPerm[0];
rk_dim14_bPerm[11] += A[155]*rk_dim14_bPerm[1];
rk_dim14_bPerm[11] += A[156]*rk_dim14_bPerm[2];
rk_dim14_bPerm[11] += A[157]*rk_dim14_bPerm[3];
rk_dim14_bPerm[11] += A[158]*rk_dim14_bPerm[4];
rk_dim14_bPerm[11] += A[159]*rk_dim14_bPerm[5];
rk_dim14_bPerm[11] += A[160]*rk_dim14_bPerm[6];
rk_dim14_bPerm[11] += A[161]*rk_dim14_bPerm[7];
rk_dim14_bPerm[11] += A[162]*rk_dim14_bPerm[8];
rk_dim14_bPerm[11] += A[163]*rk_dim14_bPerm[9];
rk_dim14_bPerm[11] += A[164]*rk_dim14_bPerm[10];

rk_dim14_bPerm[12] += A[168]*rk_dim14_bPerm[0];
rk_dim14_bPerm[12] += A[169]*rk_dim14_bPerm[1];
rk_dim14_bPerm[12] += A[170]*rk_dim14_bPerm[2];
rk_dim14_bPerm[12] += A[171]*rk_dim14_bPerm[3];
rk_dim14_bPerm[12] += A[172]*rk_dim14_bPerm[4];
rk_dim14_bPerm[12] += A[173]*rk_dim14_bPerm[5];
rk_dim14_bPerm[12] += A[174]*rk_dim14_bPerm[6];
rk_dim14_bPerm[12] += A[175]*rk_dim14_bPerm[7];
rk_dim14_bPerm[12] += A[176]*rk_dim14_bPerm[8];
rk_dim14_bPerm[12] += A[177]*rk_dim14_bPerm[9];
rk_dim14_bPerm[12] += A[178]*rk_dim14_bPerm[10];
rk_dim14_bPerm[12] += A[179]*rk_dim14_bPerm[11];

rk_dim14_bPerm[13] += A[182]*rk_dim14_bPerm[0];
rk_dim14_bPerm[13] += A[183]*rk_dim14_bPerm[1];
rk_dim14_bPerm[13] += A[184]*rk_dim14_bPerm[2];
rk_dim14_bPerm[13] += A[185]*rk_dim14_bPerm[3];
rk_dim14_bPerm[13] += A[186]*rk_dim14_bPerm[4];
rk_dim14_bPerm[13] += A[187]*rk_dim14_bPerm[5];
rk_dim14_bPerm[13] += A[188]*rk_dim14_bPerm[6];
rk_dim14_bPerm[13] += A[189]*rk_dim14_bPerm[7];
rk_dim14_bPerm[13] += A[190]*rk_dim14_bPerm[8];
rk_dim14_bPerm[13] += A[191]*rk_dim14_bPerm[9];
rk_dim14_bPerm[13] += A[192]*rk_dim14_bPerm[10];
rk_dim14_bPerm[13] += A[193]*rk_dim14_bPerm[11];
rk_dim14_bPerm[13] += A[194]*rk_dim14_bPerm[12];


acado_solve_dim14_triangular( A, rk_dim14_bPerm );
b[0] = rk_dim14_bPerm[0];
b[1] = rk_dim14_bPerm[1];
b[2] = rk_dim14_bPerm[2];
b[3] = rk_dim14_bPerm[3];
b[4] = rk_dim14_bPerm[4];
b[5] = rk_dim14_bPerm[5];
b[6] = rk_dim14_bPerm[6];
b[7] = rk_dim14_bPerm[7];
b[8] = rk_dim14_bPerm[8];
b[9] = rk_dim14_bPerm[9];
b[10] = rk_dim14_bPerm[10];
b[11] = rk_dim14_bPerm[11];
b[12] = rk_dim14_bPerm[12];
b[13] = rk_dim14_bPerm[13];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 6.2500000000000003e-03, 1.3466878364870323e-02, 
-9.6687836487032168e-04, 6.2500000000000003e-03 };


/* Fixed step size:0.025 */
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

rk_ttt = 0.0000000000000000e+00;
rk_xxx[7] = rk_eta[105];
rk_xxx[8] = rk_eta[106];
rk_xxx[9] = rk_eta[107];
rk_xxx[10] = rk_eta[108];
rk_xxx[11] = rk_eta[109];
rk_xxx[12] = rk_eta[110];
rk_xxx[13] = rk_eta[111];
rk_xxx[14] = rk_eta[112];
rk_xxx[15] = rk_eta[113];
rk_xxx[16] = rk_eta[114];
rk_xxx[17] = rk_eta[115];
rk_xxx[18] = rk_eta[116];
rk_xxx[19] = rk_eta[117];
rk_xxx[20] = rk_eta[118];
rk_xxx[21] = rk_eta[119];
rk_xxx[22] = rk_eta[120];
rk_xxx[23] = rk_eta[121];
rk_xxx[24] = rk_eta[122];
rk_xxx[25] = rk_eta[123];
rk_xxx[26] = rk_eta[124];
rk_xxx[27] = rk_eta[125];
rk_xxx[28] = rk_eta[126];
rk_xxx[29] = rk_eta[127];
rk_xxx[30] = rk_eta[128];
rk_xxx[31] = rk_eta[129];
rk_xxx[32] = rk_eta[130];
rk_xxx[33] = rk_eta[131];
rk_xxx[34] = rk_eta[132];
rk_xxx[35] = rk_eta[133];
rk_xxx[36] = rk_eta[134];
rk_xxx[37] = rk_eta[135];
rk_xxx[38] = rk_eta[136];
rk_xxx[39] = rk_eta[137];
rk_xxx[40] = rk_eta[138];
rk_xxx[41] = rk_eta[139];
rk_xxx[42] = rk_eta[140];
rk_xxx[43] = rk_eta[141];
rk_xxx[44] = rk_eta[142];
rk_xxx[45] = rk_eta[143];
rk_xxx[46] = rk_eta[144];
rk_xxx[47] = rk_eta[145];
rk_xxx[48] = rk_eta[146];
rk_xxx[49] = rk_eta[147];
rk_xxx[50] = rk_eta[148];
rk_xxx[51] = rk_eta[149];
rk_xxx[52] = rk_eta[150];
rk_xxx[53] = rk_eta[151];
rk_xxx[54] = rk_eta[152];
rk_xxx[55] = rk_eta[153];
rk_xxx[56] = rk_eta[154];
rk_xxx[57] = rk_eta[155];
rk_xxx[58] = rk_eta[156];
rk_xxx[59] = rk_eta[157];
rk_xxx[60] = rk_eta[158];
rk_xxx[61] = rk_eta[159];
rk_xxx[62] = rk_eta[160];
rk_xxx[63] = rk_eta[161];
rk_xxx[64] = rk_eta[162];
rk_xxx[65] = rk_eta[163];
rk_xxx[66] = rk_eta[164];
rk_xxx[67] = rk_eta[165];
rk_xxx[68] = rk_eta[166];
rk_xxx[69] = rk_eta[167];
rk_xxx[70] = rk_eta[168];
rk_xxx[71] = rk_eta[169];
rk_xxx[72] = rk_eta[170];
rk_xxx[73] = rk_eta[171];
rk_xxx[74] = rk_eta[172];
rk_xxx[75] = rk_eta[173];
rk_xxx[76] = rk_eta[174];
rk_xxx[77] = rk_eta[175];
rk_xxx[78] = rk_eta[176];
rk_xxx[79] = rk_eta[177];
rk_xxx[80] = rk_eta[178];
rk_xxx[81] = rk_eta[179];
rk_xxx[82] = rk_eta[180];
rk_xxx[83] = rk_eta[181];
rk_xxx[84] = rk_eta[182];
rk_xxx[85] = rk_eta[183];
rk_xxx[86] = rk_eta[184];
rk_xxx[87] = rk_eta[185];
rk_xxx[88] = rk_eta[186];
rk_xxx[89] = rk_eta[187];
rk_xxx[90] = rk_eta[188];
rk_xxx[91] = rk_eta[189];
rk_xxx[92] = rk_eta[190];
rk_xxx[93] = rk_eta[191];

for (run = 0; run < 4; ++run)
{
if( run > 0 ) {
for (i = 0; i < 7; ++i)
{
rk_diffsPrev2[i * 14] = rk_eta[i * 7 + 7];
rk_diffsPrev2[i * 14 + 1] = rk_eta[i * 7 + 8];
rk_diffsPrev2[i * 14 + 2] = rk_eta[i * 7 + 9];
rk_diffsPrev2[i * 14 + 3] = rk_eta[i * 7 + 10];
rk_diffsPrev2[i * 14 + 4] = rk_eta[i * 7 + 11];
rk_diffsPrev2[i * 14 + 5] = rk_eta[i * 7 + 12];
rk_diffsPrev2[i * 14 + 6] = rk_eta[i * 7 + 13];
rk_diffsPrev2[i * 14 + 7] = rk_eta[i * 7 + 56];
rk_diffsPrev2[i * 14 + 8] = rk_eta[i * 7 + 57];
rk_diffsPrev2[i * 14 + 9] = rk_eta[i * 7 + 58];
rk_diffsPrev2[i * 14 + 10] = rk_eta[i * 7 + 59];
rk_diffsPrev2[i * 14 + 11] = rk_eta[i * 7 + 60];
rk_diffsPrev2[i * 14 + 12] = rk_eta[i * 7 + 61];
rk_diffsPrev2[i * 14 + 13] = rk_eta[i * 7 + 62];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 7; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 98 ]) );
for (j = 0; j < 7; ++j)
{
tmp_index1 = (run1 * 7) + (j);
rk_A[tmp_index1 * 14] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14)];
rk_A[tmp_index1 * 14 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 1)];
rk_A[tmp_index1 * 14 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 2)];
rk_A[tmp_index1 * 14 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 3)];
rk_A[tmp_index1 * 14 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 4)];
rk_A[tmp_index1 * 14 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 5)];
rk_A[tmp_index1 * 14 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 6)];
if( 0 == run1 ) rk_A[(tmp_index1 * 14) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 14 + 7] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14)];
rk_A[tmp_index1 * 14 + 8] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 1)];
rk_A[tmp_index1 * 14 + 9] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 2)];
rk_A[tmp_index1 * 14 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 3)];
rk_A[tmp_index1 * 14 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 4)];
rk_A[tmp_index1 * 14 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 5)];
rk_A[tmp_index1 * 14 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 6)];
if( 1 == run1 ) rk_A[(tmp_index1 * 14) + (j + 7)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 7] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 7 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 7 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 7 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 7 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 7 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 7 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
}
det = acado_solve_dim14_system( rk_A, rk_b, rk_dim14_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 7];
rk_kkk[j + 2] += rk_b[j * 7 + 1];
rk_kkk[j + 4] += rk_b[j * 7 + 2];
rk_kkk[j + 6] += rk_b[j * 7 + 3];
rk_kkk[j + 8] += rk_b[j * 7 + 4];
rk_kkk[j + 10] += rk_b[j * 7 + 5];
rk_kkk[j + 12] += rk_b[j * 7 + 6];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 7; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 7] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 7 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 7 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 7 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 7 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 7 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 7 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
}
acado_solve_dim14_system_reuse( rk_A, rk_b, rk_dim14_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 7];
rk_kkk[j + 2] += rk_b[j * 7 + 1];
rk_kkk[j + 4] += rk_b[j * 7 + 2];
rk_kkk[j + 6] += rk_b[j * 7 + 3];
rk_kkk[j + 8] += rk_b[j * 7 + 4];
rk_kkk[j + 10] += rk_b[j * 7 + 5];
rk_kkk[j + 12] += rk_b[j * 7 + 6];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 7; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 98 ]) );
for (j = 0; j < 7; ++j)
{
tmp_index1 = (run1 * 7) + (j);
rk_A[tmp_index1 * 14] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14)];
rk_A[tmp_index1 * 14 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 1)];
rk_A[tmp_index1 * 14 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 2)];
rk_A[tmp_index1 * 14 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 3)];
rk_A[tmp_index1 * 14 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 4)];
rk_A[tmp_index1 * 14 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 5)];
rk_A[tmp_index1 * 14 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 6)];
if( 0 == run1 ) rk_A[(tmp_index1 * 14) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 14 + 7] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14)];
rk_A[tmp_index1 * 14 + 8] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 1)];
rk_A[tmp_index1 * 14 + 9] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 2)];
rk_A[tmp_index1 * 14 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 3)];
rk_A[tmp_index1 * 14 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 4)];
rk_A[tmp_index1 * 14 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 5)];
rk_A[tmp_index1 * 14 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 98) + (j * 14 + 6)];
if( 1 == run1 ) rk_A[(tmp_index1 * 14) + (j + 7)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 7; ++run1)
{
for (i = 0; i < 2; ++i)
{
rk_b[i * 7] = - rk_diffsTemp2[(i * 98) + (run1)];
rk_b[i * 7 + 1] = - rk_diffsTemp2[(i * 98) + (run1 + 14)];
rk_b[i * 7 + 2] = - rk_diffsTemp2[(i * 98) + (run1 + 28)];
rk_b[i * 7 + 3] = - rk_diffsTemp2[(i * 98) + (run1 + 42)];
rk_b[i * 7 + 4] = - rk_diffsTemp2[(i * 98) + (run1 + 56)];
rk_b[i * 7 + 5] = - rk_diffsTemp2[(i * 98) + (run1 + 70)];
rk_b[i * 7 + 6] = - rk_diffsTemp2[(i * 98) + (run1 + 84)];
}
if( 0 == run1 ) {
det = acado_solve_dim14_system( rk_A, rk_b, rk_dim14_perm );
}
 else {
acado_solve_dim14_system_reuse( rk_A, rk_b, rk_dim14_perm );
}
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 7];
rk_diffK[i + 2] = rk_b[i * 7 + 1];
rk_diffK[i + 4] = rk_b[i * 7 + 2];
rk_diffK[i + 6] = rk_b[i * 7 + 3];
rk_diffK[i + 8] = rk_b[i * 7 + 4];
rk_diffK[i + 10] = rk_b[i * 7 + 5];
rk_diffK[i + 12] = rk_b[i * 7 + 6];
}
for (i = 0; i < 7; ++i)
{
rk_diffsNew2[(i * 14) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 14) + (run1)] += + rk_diffK[i * 2]*(real_t)1.2500000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)1.2500000000000001e-02;
}
}
for (run1 = 0; run1 < 7; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index1 = (i * 7) + (j);
tmp_index2 = (run1) + (j * 14);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 98) + (tmp_index2 + 7)];
}
}
acado_solve_dim14_system_reuse( rk_A, rk_b, rk_dim14_perm );
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 7];
rk_diffK[i + 2] = rk_b[i * 7 + 1];
rk_diffK[i + 4] = rk_b[i * 7 + 2];
rk_diffK[i + 6] = rk_b[i * 7 + 3];
rk_diffK[i + 8] = rk_b[i * 7 + 4];
rk_diffK[i + 10] = rk_b[i * 7 + 5];
rk_diffK[i + 12] = rk_b[i * 7 + 6];
}
for (i = 0; i < 7; ++i)
{
rk_diffsNew2[(i * 14) + (run1 + 7)] = + rk_diffK[i * 2]*(real_t)1.2500000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)1.2500000000000001e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)1.2500000000000001e-02 + rk_kkk[1]*(real_t)1.2500000000000001e-02;
rk_eta[1] += + rk_kkk[2]*(real_t)1.2500000000000001e-02 + rk_kkk[3]*(real_t)1.2500000000000001e-02;
rk_eta[2] += + rk_kkk[4]*(real_t)1.2500000000000001e-02 + rk_kkk[5]*(real_t)1.2500000000000001e-02;
rk_eta[3] += + rk_kkk[6]*(real_t)1.2500000000000001e-02 + rk_kkk[7]*(real_t)1.2500000000000001e-02;
rk_eta[4] += + rk_kkk[8]*(real_t)1.2500000000000001e-02 + rk_kkk[9]*(real_t)1.2500000000000001e-02;
rk_eta[5] += + rk_kkk[10]*(real_t)1.2500000000000001e-02 + rk_kkk[11]*(real_t)1.2500000000000001e-02;
rk_eta[6] += + rk_kkk[12]*(real_t)1.2500000000000001e-02 + rk_kkk[13]*(real_t)1.2500000000000001e-02;
if( run == 0 ) {
for (i = 0; i < 7; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index2 = (j) + (i * 7);
rk_eta[tmp_index2 + 7] = rk_diffsNew2[(i * 14) + (j)];
}
for (j = 0; j < 7; ++j)
{
tmp_index2 = (j) + (i * 7);
rk_eta[tmp_index2 + 56] = rk_diffsNew2[(i * 14) + (j + 7)];
}
}
}
else {
for (i = 0; i < 7; ++i)
{
for (j = 0; j < 7; ++j)
{
tmp_index2 = (j) + (i * 7);
rk_eta[tmp_index2 + 7] = + rk_diffsNew2[i * 14]*rk_diffsPrev2[j];
rk_eta[tmp_index2 + 7] += + rk_diffsNew2[i * 14 + 1]*rk_diffsPrev2[j + 14];
rk_eta[tmp_index2 + 7] += + rk_diffsNew2[i * 14 + 2]*rk_diffsPrev2[j + 28];
rk_eta[tmp_index2 + 7] += + rk_diffsNew2[i * 14 + 3]*rk_diffsPrev2[j + 42];
rk_eta[tmp_index2 + 7] += + rk_diffsNew2[i * 14 + 4]*rk_diffsPrev2[j + 56];
rk_eta[tmp_index2 + 7] += + rk_diffsNew2[i * 14 + 5]*rk_diffsPrev2[j + 70];
rk_eta[tmp_index2 + 7] += + rk_diffsNew2[i * 14 + 6]*rk_diffsPrev2[j + 84];
}
for (j = 0; j < 7; ++j)
{
tmp_index2 = (j) + (i * 7);
rk_eta[tmp_index2 + 56] = rk_diffsNew2[(i * 14) + (j + 7)];
rk_eta[tmp_index2 + 56] += + rk_diffsNew2[i * 14]*rk_diffsPrev2[j + 7];
rk_eta[tmp_index2 + 56] += + rk_diffsNew2[i * 14 + 1]*rk_diffsPrev2[j + 21];
rk_eta[tmp_index2 + 56] += + rk_diffsNew2[i * 14 + 2]*rk_diffsPrev2[j + 35];
rk_eta[tmp_index2 + 56] += + rk_diffsNew2[i * 14 + 3]*rk_diffsPrev2[j + 49];
rk_eta[tmp_index2 + 56] += + rk_diffsNew2[i * 14 + 4]*rk_diffsPrev2[j + 63];
rk_eta[tmp_index2 + 56] += + rk_diffsNew2[i * 14 + 5]*rk_diffsPrev2[j + 77];
rk_eta[tmp_index2 + 56] += + rk_diffsNew2[i * 14 + 6]*rk_diffsPrev2[j + 91];
}
}
}
resetIntegrator = 0;
rk_ttt += 2.5000000000000000e-01;
}
for (i = 0; i < 7; ++i)
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



