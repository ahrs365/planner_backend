/*
 *	This file is part of qpDUNES.
 *
 *	qpDUNES -- A DUal NEwton Strategy for convex quadratic programming.
 *	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
 *	All rights reserved.
 *
 *	qpDUNES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpDUNES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qp42; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file include/qp/stage_qp_solver_qpoases.h
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2012
 */


#ifndef QP42_STAGE_QP_SOLVER_QPOASES_H
#define QP42_STAGE_QP_SOLVER_QPOASES_H

#ifdef __MATLAB__
	#include "qp/matrix_vector.h"
#endif /* __MATLAB__ */

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */


#include "qp/types.h"
#if !defined(__STATIC_MEMORY__)
	#include "qp/qpData.h"
#else
	#include "qp/qpDataStatic.h"
#endif
#include "qp/qpdunes_utils.h"

#include "qp/matrix_vector.h"

/** ... */
qpoasesObject_t* qpOASES_constructor(	qpData_t* qpData,
										int_t nV,
										int_t nC );


/** ... */
void qpOASES_destructor( qpoasesObject_t** qpoasesObject );


/** ... */
return_t qpOASES_setup( qpData_t* qpData,
						qpoasesObject_t* qpoasesObject,
						interval_t* interval
						);

/* ----------------------------------------------
 * major QP data upadate (note that qpOASES
 * immediate resolves the QP here)
 *
#>>>>>>                                           */
return_t qpOASES_dataUpdate( 	qpData_t* const qpData,
								qpoasesObject_t* const qpoasesObject,
								interval_t* const interval,
								boolean_t H_changed,
								boolean_t zLow_changed,
								boolean_t zUpp_changed,
								boolean_t D_changed,
								boolean_t dLow_changed,
								boolean_t dUpp_changed,
								int_t* nQpoasesIter
								);


/** ... */
return_t qpOASES_updateDualGuess(	qpData_t* const qpData,
									interval_t* const interval,
									const z_vector_t* const lambdaK,
									const z_vector_t* const lambdaK1
									);


/** ... */
return_t qpOASES_hotstart( 	qpData_t* qpData,
							qpoasesObject_t* qpoasesObject,
							interval_t* interval,
							z_vector_t* q,
							int_t* const numQpoasesIter,
							boolean_t logHomotopy
							);


/* ----------------------------------------------
 * Get dual solution and translate it to qpDUNES
 * style
 *
#>>>>>>                                           */
return_t qpOASES_getDualSol( 	qpData_t* qpData,
								interval_t* interval,
								qpoasesObject_t* qpoasesObject,
								d2_vector_t* mu
								);


/** ... */
return_t qpOASES_getCholZTHZ( 	qpData_t* qpData,
								qpoasesObject_t* qpoasesObject,
								zz_matrix_t* cholZTHZ 				);


/* ----------------------------------------------
 * Get null-space basis matrix Z
 *
 *		                                          */
return_t qpOASES_getZT( qpData_t* qpData,
						qpoasesObject_t* qpoasesObject,
						int_t* nFree,
						zz_matrix_t* ZT 				);


return_t qpOASES_getPrimalDualVariables( qpData_t* qpData,
										 qpoasesObject_t* qpoasesObject 	);


/* ----------------------------------------------
 * Get number of active constraints and bounds
 *
 *												  */
int_t qpOASES_getNbrActConstr( 	qpoasesObject_t* qpoasesObject
								);

/* ----------------------------------------------
 * gets the step size to the first active set change
 * if it is shorter than an incumbent step size
 * initially in alphaMin
 *
 *		                                           */
return_t qpOASES_getMinStepsize(	const qpData_t* const qpData,
									const interval_t* const interval,
									real_t* alphaMin );


/** ... */
return_t qpOASES_doStep( qpData_t* const qpData,
						 qpoasesObject_t* qpoasesObject,
						 interval_t* const interval,
						 real_t alpha,
						 z_vector_t* const z,
						 d2_vector_t* const mu,
						 z_vector_t* const q,
						 real_t* const p				);


/* ----------------------------------------------
 * evaluate stage objective at given alpha values
 * along homotopy, based on precomputed piecewise
 * quadratic parameterization
 *
#>>>>>>                                           */
return_t qpOASES_evalAddParametricObjFctn(	qpData_t* const qpData,
											interval_t* const interval,
											large_vector_t* resVec,
											large_vector_t* alphaVec,
											int_t nBasePoints
											);


/* ----------------------------------------------
 * return stage objective derivative at given
 * alpha value along homotopy, based on
 * precomputed piecewise quadratic parameterization
 *
#>>>>>>                                           */
real_t qpOASES_getParametricObjFctnGrad(	qpData_t* const qpData,
											interval_t* const interval,
											real_t alpha
											);


/* ----------------------------------------------
 * return stage objective second derivative at
 * given alpha value (in given direction) along
 * homotopy, based on precomputed piecewise
 * quadratic parameterization
 *
#>>>>>>                                           */
real_t qpOASES_getParametricObjFctnHess(	qpData_t* const qpData,
											interval_t* const interval,
											real_t alpha,
											int_t direction
											);


#ifdef __cplusplus
} /* extern C */
#endif /* __cplusplus */

#endif	/* QP42_STAGE_QP_SOLVER_QPOASES_H */


/*
 *	end of file
 */
