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


real_t rk_dim12_swap;

/** Column vector of size: 12 */
real_t rk_dim12_bPerm[ 12 ];

/** Column vector of size: 6 */
real_t auxVar[ 6 ];

real_t rk_ttt;

/** Row vector of size: 47 */
real_t rk_xxx[ 47 ];

/** Matrix of size: 6 x 2 (row major format) */
real_t rk_kkk[ 12 ];

/** Matrix of size: 12 x 12 (row major format) */
real_t rk_A[ 144 ];

/** Column vector of size: 12 */
real_t rk_b[ 12 ];

/** Row vector of size: 12 */
int rk_dim12_perm[ 12 ];

/** Column vector of size: 6 */
real_t rk_rhsTemp[ 6 ];

/** Matrix of size: 2 x 72 (row major format) */
real_t rk_diffsTemp2[ 144 ];

/** Matrix of size: 6 x 2 (row major format) */
real_t rk_diffK[ 12 ];

/** Matrix of size: 6 x 12 (row major format) */
real_t rk_diffsPrev2[ 72 ];

/** Matrix of size: 6 x 12 (row major format) */
real_t rk_diffsNew2[ 72 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim12_perm, rk_A, rk_b, rk_diffsPrev2, rk_diffsNew2, rk_diffsTemp2, rk_dim12_swap, rk_dim12_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;
const real_t* od = in + 12;

/* Compute outputs: */
out[0] = xd[3];
out[1] = xd[4];
out[2] = xd[5];
out[3] = (((u[0]-xd[3])/(real_t)(1.0000000000000001e-01))+(((((real_t)(-2.0000000000000000e+00)*((od[27]*xd[5])-(od[28]*xd[4])))+((real_t)(0.0000000000000000e+00)-((od[30]*xd[2])-(od[31]*xd[1]))))+((real_t)(0.0000000000000000e+00)-((od[27]*((od[26]*xd[1])-(od[27]*xd[0])))-(od[28]*((od[28]*xd[0])-(od[26]*xd[2]))))))+((real_t)(0.0000000000000000e+00)-od[32])));
out[4] = (((u[1]-xd[4])/(real_t)(1.0000000000000001e-01))+(((((real_t)(-2.0000000000000000e+00)*((od[28]*xd[3])-(od[26]*xd[5])))+((real_t)(0.0000000000000000e+00)-((od[31]*xd[0])-(od[29]*xd[2]))))+((real_t)(0.0000000000000000e+00)-((od[28]*((od[27]*xd[2])-(od[28]*xd[1])))-(od[26]*((od[26]*xd[1])-(od[27]*xd[0]))))))+((real_t)(0.0000000000000000e+00)-od[33])));
out[5] = (((u[2]-xd[5])/(real_t)(1.4999999999999999e-01))+(((((real_t)(-2.0000000000000000e+00)*((od[26]*xd[4])-(od[27]*xd[3])))+((real_t)(0.0000000000000000e+00)-((od[29]*xd[1])-(od[30]*xd[0]))))+((real_t)(0.0000000000000000e+00)-((od[26]*((od[28]*xd[0])-(od[26]*xd[2])))-(od[27]*((od[27]*xd[2])-(od[28]*xd[1]))))))+((real_t)(0.0000000000000000e+00)-od[34])));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 12;
/* Vector of auxiliary variables; number of elements: 6. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.0000000000000001e-01));
a[1] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.0000000000000001e-01));
a[2] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.0000000000000001e-01));
a[3] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.0000000000000001e-01));
a[4] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.4999999999999999e-01));
a[5] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.4999999999999999e-01));

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
out[16] = (real_t)(1.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
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
out[29] = (real_t)(1.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = ((real_t)(0.0000000000000000e+00)-((od[27]*((real_t)(0.0000000000000000e+00)-od[27]))-(od[28]*od[28])));
out[37] = (((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-od[31]))+((real_t)(0.0000000000000000e+00)-(od[27]*od[26])));
out[38] = (((real_t)(0.0000000000000000e+00)-od[30])+((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-(od[28]*((real_t)(0.0000000000000000e+00)-od[26])))));
out[39] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[0]);
out[40] = ((real_t)(-2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-od[28]));
out[41] = ((real_t)(-2.0000000000000000e+00)*od[27]);
out[42] = a[1];
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (((real_t)(0.0000000000000000e+00)-od[31])+((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-(od[26]*((real_t)(0.0000000000000000e+00)-od[27])))));
out[49] = ((real_t)(0.0000000000000000e+00)-((od[28]*((real_t)(0.0000000000000000e+00)-od[28]))-(od[26]*od[26])));
out[50] = (((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-od[29]))+((real_t)(0.0000000000000000e+00)-(od[28]*od[27])));
out[51] = ((real_t)(-2.0000000000000000e+00)*od[28]);
out[52] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[2]);
out[53] = ((real_t)(-2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-od[26]));
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = a[3];
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-od[30]))+((real_t)(0.0000000000000000e+00)-(od[26]*od[28])));
out[61] = (((real_t)(0.0000000000000000e+00)-od[29])+((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-(od[27]*((real_t)(0.0000000000000000e+00)-od[28])))));
out[62] = ((real_t)(0.0000000000000000e+00)-((od[26]*((real_t)(0.0000000000000000e+00)-od[26]))-(od[27]*od[27])));
out[63] = ((real_t)(-2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-od[27]));
out[64] = ((real_t)(-2.0000000000000000e+00)*od[26]);
out[65] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[4]);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = a[5];
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim12_triangular( real_t* const A, real_t* const b )
{

b[11] = b[11]/A[143];
b[10] -= + A[131]*b[11];
b[10] = b[10]/A[130];
b[9] -= + A[119]*b[11];
b[9] -= + A[118]*b[10];
b[9] = b[9]/A[117];
b[8] -= + A[107]*b[11];
b[8] -= + A[106]*b[10];
b[8] -= + A[105]*b[9];
b[8] = b[8]/A[104];
b[7] -= + A[95]*b[11];
b[7] -= + A[94]*b[10];
b[7] -= + A[93]*b[9];
b[7] -= + A[92]*b[8];
b[7] = b[7]/A[91];
b[6] -= + A[83]*b[11];
b[6] -= + A[82]*b[10];
b[6] -= + A[81]*b[9];
b[6] -= + A[80]*b[8];
b[6] -= + A[79]*b[7];
b[6] = b[6]/A[78];
b[5] -= + A[71]*b[11];
b[5] -= + A[70]*b[10];
b[5] -= + A[69]*b[9];
b[5] -= + A[68]*b[8];
b[5] -= + A[67]*b[7];
b[5] -= + A[66]*b[6];
b[5] = b[5]/A[65];
b[4] -= + A[59]*b[11];
b[4] -= + A[58]*b[10];
b[4] -= + A[57]*b[9];
b[4] -= + A[56]*b[8];
b[4] -= + A[55]*b[7];
b[4] -= + A[54]*b[6];
b[4] -= + A[53]*b[5];
b[4] = b[4]/A[52];
b[3] -= + A[47]*b[11];
b[3] -= + A[46]*b[10];
b[3] -= + A[45]*b[9];
b[3] -= + A[44]*b[8];
b[3] -= + A[43]*b[7];
b[3] -= + A[42]*b[6];
b[3] -= + A[41]*b[5];
b[3] -= + A[40]*b[4];
b[3] = b[3]/A[39];
b[2] -= + A[35]*b[11];
b[2] -= + A[34]*b[10];
b[2] -= + A[33]*b[9];
b[2] -= + A[32]*b[8];
b[2] -= + A[31]*b[7];
b[2] -= + A[30]*b[6];
b[2] -= + A[29]*b[5];
b[2] -= + A[28]*b[4];
b[2] -= + A[27]*b[3];
b[2] = b[2]/A[26];
b[1] -= + A[23]*b[11];
b[1] -= + A[22]*b[10];
b[1] -= + A[21]*b[9];
b[1] -= + A[20]*b[8];
b[1] -= + A[19]*b[7];
b[1] -= + A[18]*b[6];
b[1] -= + A[17]*b[5];
b[1] -= + A[16]*b[4];
b[1] -= + A[15]*b[3];
b[1] -= + A[14]*b[2];
b[1] = b[1]/A[13];
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

real_t acado_solve_dim12_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 12; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (11); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*12+i]);
	for( j=(i+1); j < 12; j++ ) {
		temp = fabs(A[j*12+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 12; ++k)
{
	rk_dim12_swap = A[i*12+k];
	A[i*12+k] = A[indexMax*12+k];
	A[indexMax*12+k] = rk_dim12_swap;
}
	rk_dim12_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim12_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*12+i];
	for( j=i+1; j < 12; j++ ) {
		A[j*12+i] = -A[j*12+i]/A[i*12+i];
		for( k=i+1; k < 12; k++ ) {
			A[j*12+k] += A[j*12+i] * A[i*12+k];
		}
		b[j] += A[j*12+i] * b[i];
	}
}
det *= A[143];
det = fabs(det);
acado_solve_dim12_triangular( A, b );
return det;
}

void acado_solve_dim12_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim12_bPerm[0] = b[rk_perm[0]];
rk_dim12_bPerm[1] = b[rk_perm[1]];
rk_dim12_bPerm[2] = b[rk_perm[2]];
rk_dim12_bPerm[3] = b[rk_perm[3]];
rk_dim12_bPerm[4] = b[rk_perm[4]];
rk_dim12_bPerm[5] = b[rk_perm[5]];
rk_dim12_bPerm[6] = b[rk_perm[6]];
rk_dim12_bPerm[7] = b[rk_perm[7]];
rk_dim12_bPerm[8] = b[rk_perm[8]];
rk_dim12_bPerm[9] = b[rk_perm[9]];
rk_dim12_bPerm[10] = b[rk_perm[10]];
rk_dim12_bPerm[11] = b[rk_perm[11]];
rk_dim12_bPerm[1] += A[12]*rk_dim12_bPerm[0];

rk_dim12_bPerm[2] += A[24]*rk_dim12_bPerm[0];
rk_dim12_bPerm[2] += A[25]*rk_dim12_bPerm[1];

rk_dim12_bPerm[3] += A[36]*rk_dim12_bPerm[0];
rk_dim12_bPerm[3] += A[37]*rk_dim12_bPerm[1];
rk_dim12_bPerm[3] += A[38]*rk_dim12_bPerm[2];

rk_dim12_bPerm[4] += A[48]*rk_dim12_bPerm[0];
rk_dim12_bPerm[4] += A[49]*rk_dim12_bPerm[1];
rk_dim12_bPerm[4] += A[50]*rk_dim12_bPerm[2];
rk_dim12_bPerm[4] += A[51]*rk_dim12_bPerm[3];

rk_dim12_bPerm[5] += A[60]*rk_dim12_bPerm[0];
rk_dim12_bPerm[5] += A[61]*rk_dim12_bPerm[1];
rk_dim12_bPerm[5] += A[62]*rk_dim12_bPerm[2];
rk_dim12_bPerm[5] += A[63]*rk_dim12_bPerm[3];
rk_dim12_bPerm[5] += A[64]*rk_dim12_bPerm[4];

rk_dim12_bPerm[6] += A[72]*rk_dim12_bPerm[0];
rk_dim12_bPerm[6] += A[73]*rk_dim12_bPerm[1];
rk_dim12_bPerm[6] += A[74]*rk_dim12_bPerm[2];
rk_dim12_bPerm[6] += A[75]*rk_dim12_bPerm[3];
rk_dim12_bPerm[6] += A[76]*rk_dim12_bPerm[4];
rk_dim12_bPerm[6] += A[77]*rk_dim12_bPerm[5];

rk_dim12_bPerm[7] += A[84]*rk_dim12_bPerm[0];
rk_dim12_bPerm[7] += A[85]*rk_dim12_bPerm[1];
rk_dim12_bPerm[7] += A[86]*rk_dim12_bPerm[2];
rk_dim12_bPerm[7] += A[87]*rk_dim12_bPerm[3];
rk_dim12_bPerm[7] += A[88]*rk_dim12_bPerm[4];
rk_dim12_bPerm[7] += A[89]*rk_dim12_bPerm[5];
rk_dim12_bPerm[7] += A[90]*rk_dim12_bPerm[6];

rk_dim12_bPerm[8] += A[96]*rk_dim12_bPerm[0];
rk_dim12_bPerm[8] += A[97]*rk_dim12_bPerm[1];
rk_dim12_bPerm[8] += A[98]*rk_dim12_bPerm[2];
rk_dim12_bPerm[8] += A[99]*rk_dim12_bPerm[3];
rk_dim12_bPerm[8] += A[100]*rk_dim12_bPerm[4];
rk_dim12_bPerm[8] += A[101]*rk_dim12_bPerm[5];
rk_dim12_bPerm[8] += A[102]*rk_dim12_bPerm[6];
rk_dim12_bPerm[8] += A[103]*rk_dim12_bPerm[7];

rk_dim12_bPerm[9] += A[108]*rk_dim12_bPerm[0];
rk_dim12_bPerm[9] += A[109]*rk_dim12_bPerm[1];
rk_dim12_bPerm[9] += A[110]*rk_dim12_bPerm[2];
rk_dim12_bPerm[9] += A[111]*rk_dim12_bPerm[3];
rk_dim12_bPerm[9] += A[112]*rk_dim12_bPerm[4];
rk_dim12_bPerm[9] += A[113]*rk_dim12_bPerm[5];
rk_dim12_bPerm[9] += A[114]*rk_dim12_bPerm[6];
rk_dim12_bPerm[9] += A[115]*rk_dim12_bPerm[7];
rk_dim12_bPerm[9] += A[116]*rk_dim12_bPerm[8];

rk_dim12_bPerm[10] += A[120]*rk_dim12_bPerm[0];
rk_dim12_bPerm[10] += A[121]*rk_dim12_bPerm[1];
rk_dim12_bPerm[10] += A[122]*rk_dim12_bPerm[2];
rk_dim12_bPerm[10] += A[123]*rk_dim12_bPerm[3];
rk_dim12_bPerm[10] += A[124]*rk_dim12_bPerm[4];
rk_dim12_bPerm[10] += A[125]*rk_dim12_bPerm[5];
rk_dim12_bPerm[10] += A[126]*rk_dim12_bPerm[6];
rk_dim12_bPerm[10] += A[127]*rk_dim12_bPerm[7];
rk_dim12_bPerm[10] += A[128]*rk_dim12_bPerm[8];
rk_dim12_bPerm[10] += A[129]*rk_dim12_bPerm[9];

rk_dim12_bPerm[11] += A[132]*rk_dim12_bPerm[0];
rk_dim12_bPerm[11] += A[133]*rk_dim12_bPerm[1];
rk_dim12_bPerm[11] += A[134]*rk_dim12_bPerm[2];
rk_dim12_bPerm[11] += A[135]*rk_dim12_bPerm[3];
rk_dim12_bPerm[11] += A[136]*rk_dim12_bPerm[4];
rk_dim12_bPerm[11] += A[137]*rk_dim12_bPerm[5];
rk_dim12_bPerm[11] += A[138]*rk_dim12_bPerm[6];
rk_dim12_bPerm[11] += A[139]*rk_dim12_bPerm[7];
rk_dim12_bPerm[11] += A[140]*rk_dim12_bPerm[8];
rk_dim12_bPerm[11] += A[141]*rk_dim12_bPerm[9];
rk_dim12_bPerm[11] += A[142]*rk_dim12_bPerm[10];


acado_solve_dim12_triangular( A, rk_dim12_bPerm );
b[0] = rk_dim12_bPerm[0];
b[1] = rk_dim12_bPerm[1];
b[2] = rk_dim12_bPerm[2];
b[3] = rk_dim12_bPerm[3];
b[4] = rk_dim12_bPerm[4];
b[5] = rk_dim12_bPerm[5];
b[6] = rk_dim12_bPerm[6];
b[7] = rk_dim12_bPerm[7];
b[8] = rk_dim12_bPerm[8];
b[9] = rk_dim12_bPerm[9];
b[10] = rk_dim12_bPerm[10];
b[11] = rk_dim12_bPerm[11];
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
rk_xxx[6] = rk_eta[78];
rk_xxx[7] = rk_eta[79];
rk_xxx[8] = rk_eta[80];
rk_xxx[9] = rk_eta[81];
rk_xxx[10] = rk_eta[82];
rk_xxx[11] = rk_eta[83];
rk_xxx[12] = rk_eta[84];
rk_xxx[13] = rk_eta[85];
rk_xxx[14] = rk_eta[86];
rk_xxx[15] = rk_eta[87];
rk_xxx[16] = rk_eta[88];
rk_xxx[17] = rk_eta[89];
rk_xxx[18] = rk_eta[90];
rk_xxx[19] = rk_eta[91];
rk_xxx[20] = rk_eta[92];
rk_xxx[21] = rk_eta[93];
rk_xxx[22] = rk_eta[94];
rk_xxx[23] = rk_eta[95];
rk_xxx[24] = rk_eta[96];
rk_xxx[25] = rk_eta[97];
rk_xxx[26] = rk_eta[98];
rk_xxx[27] = rk_eta[99];
rk_xxx[28] = rk_eta[100];
rk_xxx[29] = rk_eta[101];
rk_xxx[30] = rk_eta[102];
rk_xxx[31] = rk_eta[103];
rk_xxx[32] = rk_eta[104];
rk_xxx[33] = rk_eta[105];
rk_xxx[34] = rk_eta[106];
rk_xxx[35] = rk_eta[107];
rk_xxx[36] = rk_eta[108];
rk_xxx[37] = rk_eta[109];
rk_xxx[38] = rk_eta[110];
rk_xxx[39] = rk_eta[111];
rk_xxx[40] = rk_eta[112];
rk_xxx[41] = rk_eta[113];
rk_xxx[42] = rk_eta[114];
rk_xxx[43] = rk_eta[115];
rk_xxx[44] = rk_eta[116];
rk_xxx[45] = rk_eta[117];
rk_xxx[46] = rk_eta[118];

for (run = 0; run < 4; ++run)
{
if( run > 0 ) {
for (i = 0; i < 6; ++i)
{
rk_diffsPrev2[i * 12] = rk_eta[i * 6 + 6];
rk_diffsPrev2[i * 12 + 1] = rk_eta[i * 6 + 7];
rk_diffsPrev2[i * 12 + 2] = rk_eta[i * 6 + 8];
rk_diffsPrev2[i * 12 + 3] = rk_eta[i * 6 + 9];
rk_diffsPrev2[i * 12 + 4] = rk_eta[i * 6 + 10];
rk_diffsPrev2[i * 12 + 5] = rk_eta[i * 6 + 11];
rk_diffsPrev2[i * 12 + 6] = rk_eta[i * 6 + 42];
rk_diffsPrev2[i * 12 + 7] = rk_eta[i * 6 + 43];
rk_diffsPrev2[i * 12 + 8] = rk_eta[i * 6 + 44];
rk_diffsPrev2[i * 12 + 9] = rk_eta[i * 6 + 45];
rk_diffsPrev2[i * 12 + 10] = rk_eta[i * 6 + 46];
rk_diffsPrev2[i * 12 + 11] = rk_eta[i * 6 + 47];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 72 ]) );
for (j = 0; j < 6; ++j)
{
tmp_index1 = (run1 * 6) + (j);
rk_A[tmp_index1 * 12] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12)];
rk_A[tmp_index1 * 12 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 1)];
rk_A[tmp_index1 * 12 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 2)];
rk_A[tmp_index1 * 12 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 3)];
rk_A[tmp_index1 * 12 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 4)];
rk_A[tmp_index1 * 12 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 5)];
if( 0 == run1 ) rk_A[(tmp_index1 * 12) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 12 + 6] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12)];
rk_A[tmp_index1 * 12 + 7] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 1)];
rk_A[tmp_index1 * 12 + 8] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 2)];
rk_A[tmp_index1 * 12 + 9] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 3)];
rk_A[tmp_index1 * 12 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 4)];
rk_A[tmp_index1 * 12 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 5)];
if( 1 == run1 ) rk_A[(tmp_index1 * 12) + (j + 6)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 6] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 6 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 6 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 6 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 6 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 6 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
}
det = acado_solve_dim12_system( rk_A, rk_b, rk_dim12_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 6];
rk_kkk[j + 2] += rk_b[j * 6 + 1];
rk_kkk[j + 4] += rk_b[j * 6 + 2];
rk_kkk[j + 6] += rk_b[j * 6 + 3];
rk_kkk[j + 8] += rk_b[j * 6 + 4];
rk_kkk[j + 10] += rk_b[j * 6 + 5];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 6] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 6 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 6 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 6 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 6 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 6 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
}
acado_solve_dim12_system_reuse( rk_A, rk_b, rk_dim12_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 6];
rk_kkk[j + 2] += rk_b[j * 6 + 1];
rk_kkk[j + 4] += rk_b[j * 6 + 2];
rk_kkk[j + 6] += rk_b[j * 6 + 3];
rk_kkk[j + 8] += rk_b[j * 6 + 4];
rk_kkk[j + 10] += rk_b[j * 6 + 5];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 6; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 72 ]) );
for (j = 0; j < 6; ++j)
{
tmp_index1 = (run1 * 6) + (j);
rk_A[tmp_index1 * 12] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12)];
rk_A[tmp_index1 * 12 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 1)];
rk_A[tmp_index1 * 12 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 2)];
rk_A[tmp_index1 * 12 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 3)];
rk_A[tmp_index1 * 12 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 4)];
rk_A[tmp_index1 * 12 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 5)];
if( 0 == run1 ) rk_A[(tmp_index1 * 12) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 12 + 6] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12)];
rk_A[tmp_index1 * 12 + 7] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 1)];
rk_A[tmp_index1 * 12 + 8] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 2)];
rk_A[tmp_index1 * 12 + 9] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 3)];
rk_A[tmp_index1 * 12 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 4)];
rk_A[tmp_index1 * 12 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 72) + (j * 12 + 5)];
if( 1 == run1 ) rk_A[(tmp_index1 * 12) + (j + 6)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 6; ++run1)
{
for (i = 0; i < 2; ++i)
{
rk_b[i * 6] = - rk_diffsTemp2[(i * 72) + (run1)];
rk_b[i * 6 + 1] = - rk_diffsTemp2[(i * 72) + (run1 + 12)];
rk_b[i * 6 + 2] = - rk_diffsTemp2[(i * 72) + (run1 + 24)];
rk_b[i * 6 + 3] = - rk_diffsTemp2[(i * 72) + (run1 + 36)];
rk_b[i * 6 + 4] = - rk_diffsTemp2[(i * 72) + (run1 + 48)];
rk_b[i * 6 + 5] = - rk_diffsTemp2[(i * 72) + (run1 + 60)];
}
if( 0 == run1 ) {
det = acado_solve_dim12_system( rk_A, rk_b, rk_dim12_perm );
}
 else {
acado_solve_dim12_system_reuse( rk_A, rk_b, rk_dim12_perm );
}
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 6];
rk_diffK[i + 2] = rk_b[i * 6 + 1];
rk_diffK[i + 4] = rk_b[i * 6 + 2];
rk_diffK[i + 6] = rk_b[i * 6 + 3];
rk_diffK[i + 8] = rk_b[i * 6 + 4];
rk_diffK[i + 10] = rk_b[i * 6 + 5];
}
for (i = 0; i < 6; ++i)
{
rk_diffsNew2[(i * 12) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 12) + (run1)] += + rk_diffK[i * 2]*(real_t)1.2500000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)1.2500000000000001e-02;
}
}
for (run1 = 0; run1 < 6; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index1 = (i * 6) + (j);
tmp_index2 = (run1) + (j * 12);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 72) + (tmp_index2 + 6)];
}
}
acado_solve_dim12_system_reuse( rk_A, rk_b, rk_dim12_perm );
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 6];
rk_diffK[i + 2] = rk_b[i * 6 + 1];
rk_diffK[i + 4] = rk_b[i * 6 + 2];
rk_diffK[i + 6] = rk_b[i * 6 + 3];
rk_diffK[i + 8] = rk_b[i * 6 + 4];
rk_diffK[i + 10] = rk_b[i * 6 + 5];
}
for (i = 0; i < 6; ++i)
{
rk_diffsNew2[(i * 12) + (run1 + 6)] = + rk_diffK[i * 2]*(real_t)1.2500000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)1.2500000000000001e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)1.2500000000000001e-02 + rk_kkk[1]*(real_t)1.2500000000000001e-02;
rk_eta[1] += + rk_kkk[2]*(real_t)1.2500000000000001e-02 + rk_kkk[3]*(real_t)1.2500000000000001e-02;
rk_eta[2] += + rk_kkk[4]*(real_t)1.2500000000000001e-02 + rk_kkk[5]*(real_t)1.2500000000000001e-02;
rk_eta[3] += + rk_kkk[6]*(real_t)1.2500000000000001e-02 + rk_kkk[7]*(real_t)1.2500000000000001e-02;
rk_eta[4] += + rk_kkk[8]*(real_t)1.2500000000000001e-02 + rk_kkk[9]*(real_t)1.2500000000000001e-02;
rk_eta[5] += + rk_kkk[10]*(real_t)1.2500000000000001e-02 + rk_kkk[11]*(real_t)1.2500000000000001e-02;
if( run == 0 ) {
for (i = 0; i < 6; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 6] = rk_diffsNew2[(i * 12) + (j)];
}
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 42] = rk_diffsNew2[(i * 12) + (j + 6)];
}
}
}
else {
for (i = 0; i < 6; ++i)
{
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 6] = + rk_diffsNew2[i * 12]*rk_diffsPrev2[j];
rk_eta[tmp_index2 + 6] += + rk_diffsNew2[i * 12 + 1]*rk_diffsPrev2[j + 12];
rk_eta[tmp_index2 + 6] += + rk_diffsNew2[i * 12 + 2]*rk_diffsPrev2[j + 24];
rk_eta[tmp_index2 + 6] += + rk_diffsNew2[i * 12 + 3]*rk_diffsPrev2[j + 36];
rk_eta[tmp_index2 + 6] += + rk_diffsNew2[i * 12 + 4]*rk_diffsPrev2[j + 48];
rk_eta[tmp_index2 + 6] += + rk_diffsNew2[i * 12 + 5]*rk_diffsPrev2[j + 60];
}
for (j = 0; j < 6; ++j)
{
tmp_index2 = (j) + (i * 6);
rk_eta[tmp_index2 + 42] = rk_diffsNew2[(i * 12) + (j + 6)];
rk_eta[tmp_index2 + 42] += + rk_diffsNew2[i * 12]*rk_diffsPrev2[j + 6];
rk_eta[tmp_index2 + 42] += + rk_diffsNew2[i * 12 + 1]*rk_diffsPrev2[j + 18];
rk_eta[tmp_index2 + 42] += + rk_diffsNew2[i * 12 + 2]*rk_diffsPrev2[j + 30];
rk_eta[tmp_index2 + 42] += + rk_diffsNew2[i * 12 + 3]*rk_diffsPrev2[j + 42];
rk_eta[tmp_index2 + 42] += + rk_diffsNew2[i * 12 + 4]*rk_diffsPrev2[j + 54];
rk_eta[tmp_index2 + 42] += + rk_diffsNew2[i * 12 + 5]*rk_diffsPrev2[j + 66];
}
}
}
resetIntegrator = 0;
rk_ttt += 2.5000000000000000e-01;
}
for (i = 0; i < 6; ++i)
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



