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


#include <acado_common.h>

#include "qpOASES_e/QProblem.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
#include "qpOASES_e/extras/SolutionAnalysis.h"
#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

static int acado_nWSR;

/*
 * Optional diagnostic capture of the exact condensed QP handed to qpOASES-e.
 * The hook is disabled unless CONI_MPC_QP_CAPTURE names a writable file.
 * It records only diagnostic copies and does not alter solver inputs, options,
 * or initialization.  The companion reader owns the fixed array layout below.
 */
static FILE* coni_qp_capture = 0;
static uint64_t coni_qp_capture_index = 0;
static int coni_qp_capture_initialized = 0;

static void coni_qp_write(const void* data, size_t size, size_t count) {
  if (coni_qp_capture != 0) (void)fwrite(data, size, count, coni_qp_capture);
}

static void coni_qp_capture_open(void) {
  if (coni_qp_capture_initialized) return;
  coni_qp_capture_initialized = 1;
  const char* path = getenv("CONI_MPC_QP_CAPTURE");
  if (path == 0 || path[0] == '\0') return;
  coni_qp_capture = fopen(path, "wb");
  if (coni_qp_capture == 0) return;
  const uint64_t magic = UINT64_C(0x3150514d4f434343); /* CCCOMQP1 */
  const uint32_t version = 1;
  const uint32_t nv = ACADO_QP_NV;
	const uint32_t nc = ACADO_NPAC * ACADO_N;
  const uint32_t nx_nodes = ACADO_NX * (ACADO_N + 1);
  const uint32_t nu_nodes = ACADO_NU * ACADO_N;
  const uint32_t nod_nodes = ACADO_NOD * (ACADO_N + 1);
  coni_qp_write(&magic, sizeof(magic), 1);
  coni_qp_write(&version, sizeof(version), 1);
  coni_qp_write(&nv, sizeof(nv), 1);
  coni_qp_write(&nc, sizeof(nc), 1);
  coni_qp_write(&nx_nodes, sizeof(nx_nodes), 1);
  coni_qp_write(&nu_nodes, sizeof(nu_nodes), 1);
  coni_qp_write(&nod_nodes, sizeof(nod_nodes), 1);
}

static void coni_qp_capture_record(int status, double objective, double kkt) {
  coni_qp_capture_open();
  if (coni_qp_capture == 0) return;
  const uint64_t index = coni_qp_capture_index++;
  const int32_t solver_status = (int32_t)status;
  const int32_t nWSR = (int32_t)acado_nWSR;
  coni_qp_write(&index, sizeof(index), 1);
  coni_qp_write(&solver_status, sizeof(solver_status), 1);
  coni_qp_write(&nWSR, sizeof(nWSR), 1);
  coni_qp_write(&objective, sizeof(objective), 1);
  coni_qp_write(&kkt, sizeof(kkt), 1);
  /* Pre-feedback ACADO variables: RTI linearization/reference context. */
  coni_qp_write(acadoVariables.x, sizeof(real_t), sizeof(acadoVariables.x) / sizeof(real_t));
  coni_qp_write(acadoVariables.u, sizeof(real_t), sizeof(acadoVariables.u) / sizeof(real_t));
  coni_qp_write(acadoVariables.x0, sizeof(real_t), sizeof(acadoVariables.x0) / sizeof(real_t));
  coni_qp_write(acadoVariables.od, sizeof(real_t), sizeof(acadoVariables.od) / sizeof(real_t));
  coni_qp_write(acadoVariables.y, sizeof(real_t), sizeof(acadoVariables.y) / sizeof(real_t));
  coni_qp_write(acadoVariables.yN, sizeof(real_t), sizeof(acadoVariables.yN) / sizeof(real_t));
  coni_qp_write(acadoVariables.W, sizeof(real_t), sizeof(acadoVariables.W) / sizeof(real_t));
  coni_qp_write(acadoVariables.WN, sizeof(real_t), sizeof(acadoVariables.WN) / sizeof(real_t));
  coni_qp_write(acadoVariables.lbValues, sizeof(real_t), sizeof(acadoVariables.lbValues) / sizeof(real_t));
  coni_qp_write(acadoVariables.ubValues, sizeof(real_t), sizeof(acadoVariables.ubValues) / sizeof(real_t));
  coni_qp_write(acadoVariables.lbAValues, sizeof(real_t), sizeof(acadoVariables.lbAValues) / sizeof(real_t));
  coni_qp_write(acadoVariables.ubAValues, sizeof(real_t), sizeof(acadoVariables.ubAValues) / sizeof(real_t));
  /* Exact condensed qpOASES-e arrays and returned primal/dual solution. */
  coni_qp_write(acadoWorkspace.H, sizeof(real_t), sizeof(acadoWorkspace.H) / sizeof(real_t));
  coni_qp_write(acadoWorkspace.g, sizeof(real_t), sizeof(acadoWorkspace.g) / sizeof(real_t));
  coni_qp_write(acadoWorkspace.A, sizeof(real_t), sizeof(acadoWorkspace.A) / sizeof(real_t));
  coni_qp_write(acadoWorkspace.lb, sizeof(real_t), sizeof(acadoWorkspace.lb) / sizeof(real_t));
  coni_qp_write(acadoWorkspace.ub, sizeof(real_t), sizeof(acadoWorkspace.ub) / sizeof(real_t));
  coni_qp_write(acadoWorkspace.lbA, sizeof(real_t), sizeof(acadoWorkspace.lbA) / sizeof(real_t));
  coni_qp_write(acadoWorkspace.ubA, sizeof(real_t), sizeof(acadoWorkspace.ubA) / sizeof(real_t));
  coni_qp_write(acadoWorkspace.x, sizeof(real_t), sizeof(acadoWorkspace.x) / sizeof(real_t));
  coni_qp_write(acadoWorkspace.y, sizeof(real_t), sizeof(acadoWorkspace.y) / sizeof(real_t));
  fflush(coni_qp_capture);
}




#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1
static SolutionAnalysis acado_sa;
#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

int acado_solve( void )
{

	returnValue retVal;
	QProblem qp;
	Options options;

	acado_nWSR = QPOASES_NWSRMAX;

	QProblemCON( &qp, 140,240,HST_POSDEF );
	Options_setToMPC( &options );
	QProblem_setOptions( &qp,options );

	retVal = QProblem_init( &qp, acadoWorkspace.H, acadoWorkspace.g, acadoWorkspace.A, acadoWorkspace.lb, acadoWorkspace.ub, acadoWorkspace.lbA, acadoWorkspace.ubA, &acado_nWSR,0 );
	retVal = qpOASES_getSimpleStatus( retVal,0 );

    /* only use solution on success, if iteration limit has been reached or if QP is infeasible! */
    if ( ( retVal == 0 ) || ( retVal == 1 ) || ( retVal == -2 ) )
    {
        QProblem_getPrimalSolution( &qp,acadoWorkspace.x );
        QProblem_getDualSolution( &qp,acadoWorkspace.y );
    }

	double objective = 0.0;
	for (int i = 0; i < ACADO_QP_NV; ++i) {
		objective += acadoWorkspace.g[i] * acadoWorkspace.x[i];
		for (int j = 0; j < ACADO_QP_NV; ++j)
			objective += 0.5 * acadoWorkspace.x[i] * acadoWorkspace.H[i*ACADO_QP_NV+j] * acadoWorkspace.x[j];
	}
	coni_qp_capture_record((int)retVal, objective, acado_getKKT());

#if ACADO_COMPUTE_COVARIANCE_MATRIX == 1

	if (retVal != SUCCESSFUL_RETURN)
		return (int)retVal;

	retVal = acado_sa.getHessianInverse( &qp,var );

#endif /* ACADO_COMPUTE_COVARIANCE_MATRIX */

	return (int)retVal;
}

int acado_getNWSR( void )
{
	return acado_nWSR;
}

const char* acado_getErrorString( int error )
{
	return MessageHandling_getErrorCodeMessage( qpOASES_getGlobalMessageHandler(),error );
}
