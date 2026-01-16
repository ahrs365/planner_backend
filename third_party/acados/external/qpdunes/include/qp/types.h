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
 *	License along with qpDUNES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file include/qp/types.h
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 *
 *	Declaration of all non-built-in unit types.
 */


#ifndef QPDUNES_TYPES_H
#define QPDUNES_TYPES_H

#ifdef USE_ACADOS_TYPES
#include "acados/utils/types.h"
#endif

#ifdef __MATLAB__
#include "matrix.h"	/* for mwSize types */
#endif


#define __USE_ASSERTS__ 					/* check return values of routines rigorously (useful for debugging) */
#undef __USE_ASSERTS__

//#define __SUPPRESS_ALL_OUTPUT__			/* no printing */
//#undef __SUPPRESS_ALL_OUTPUT__

#define __SUPPRESS_ALL_WARNINGS__			/* do not display warnings */
#undef __SUPPRESS_ALL_WARNINGS__

//#define __MEASURE_TIMINGS__					/* measure computation times */
#undef __MEASURE_TIMINGS__

#define __ANALYZE_FACTORIZATION__			/* log inverse Newton Hessian for analysis */
#undef __ANALYZE_FACTORIZATION__

#define __QPDUNES_PARALLEL__				/* use openMP parallelization */
#undef __QPDUNES_PARALLEL__

#define __DO_UNIT_TESTS__					/* include some unit tests */
#undef __DO_UNIT_TESTS__

#define PRINTING_PRECISION 14

#ifdef __MATLAB__
	#define MAX_STR_LEN 2560
#endif


/** colored/formatted terminal output */
#ifndef __MATLAB__
	#define COL_STD  "\033[0m"
	#define COL_SUCC "\033[92m"
	#define COL_WARN "\033[93m"
	#define COL_ERR  "\033[91m"
#else
	#define COL_STD  " ***"
	#define COL_SUCC "*** "
	#define COL_WARN "*** "
	#define COL_ERR  "*** "
#endif


/** simple types */
#ifndef __MATLAB__
	#ifndef uint_t
		typedef unsigned int uint_t;
	#endif
	#ifndef USE_ACADOS_TYPES
		#ifndef int_t
			typedef int int_t;
		#endif
		#ifndef real_t
			#ifdef __USE_SINGLE_PRECISION__
				typedef float real_t;
			#else
				typedef double real_t;
			#endif	/* __USE_SINGLE_PRECISION__ */
		#endif
	#endif  /* USE_ACADOS_TYPES */
#else	/* __MATLAB__ */
	typedef int int_t;
	typedef unsigned int uint_t;
	typedef double real_t;
#endif	/* __MATLAB__ */


#if !defined(__STATIC_MEMORY__)
	#define _NX_ (qpData->nX)
	#define _NU_ (qpData->nU)
	#define _NZ_ (qpData->nZ)
	#define _NI_ (qpData->nI)
	#define _NDTTL_ (qpData->nDttl)
#endif
#define _NV( I ) (qpData->intervals[ I ]->nV)
#define _ND( I ) (qpData->intervals[ I ]->nD)
#define _NV_ (interval->nV)
#define _ND_ (interval->nD)



/** MATRIX ACCESS */
/*                                                block offset   row offset   column offset (0=diag,-1=supDiag)   column */
#define accHessian( K, L, I, J )	hessian->data[ (K)*2*_NX_*_NX_  + (I)*2*_NX_   + (1+L)*_NX_                         + (J) ]
/*                                                        block offset   row offset   column offset (0=diag,-1=supDiag)   column */
#define accCholHessian( K, L, I, J )	cholHessian->data[ (K)*2*_NX_*_NX_  + (I)*2*_NX_   + (1+L)*_NX_                         + (J) ]

#define accUnconstrainedHessian( K, L, I, J )	qpData->unconstrainedHessian.data[ (K)*2*_NX_*_NX_ + (I)*2*_NX_ + (1+L)*_NX_ + (J) ]

#define accCholUnconstrainedHessian( K, L, I, J )	qpData->cholUnconstrainedHessian.data[ (K)*2*_NX_*_NX_ + (I)*2*_NX_ + (1+L)*_NX_ + (J) ]


#define accH( I, J )	H->data[ (I)*nV + (J) ]

#define accC( I, J )	C->data[ (I)*_NZ_ + (J) ]

#define accM( I, J, DIM )	M[ (I)*(DIM) + (J) ]		/**< generic low level matrix access */
#define accMT( I, J, DIM )	M[ (J)*(DIM) + (I) ]		/**< generic low level transposed matrix access */
#define accL( I, J, DIM )	L[ (I)*(DIM) + (J) ]		/**< generic low level lower triangular matrix access */


/** Advanced types */

/** A simple Boolean type */
#ifdef __APPLE__
	typedef unsigned int boolean_t;
	#ifndef QPDUNES_FALSE
		#define QPDUNES_FALSE 0
	#endif
	#ifndef QPDUNES_TRUE
		#define QPDUNES_TRUE 1
	#endif
#else
	typedef enum
	{
		QPDUNES_FALSE,					/**< ... */
		QPDUNES_TRUE					/**< ... */
	} boolean_t;
#endif


/** Matrix sparsity type */
typedef enum
{
	QPDUNES_MATRIX_UNDEFINED,		/**< ... */
	QPDUNES_DENSE,					/**< ... */
	QPDUNES_SPARSE,					/**< ... */
	QPDUNES_DIAGONAL,				/**< ... */
	QPDUNES_IDENTITY,				/**< ... */
	QPDUNES_ALLZEROS				/**< ... */
} sparsityType_t;



/** Log level */
typedef enum
{
	QPDUNES_LOG_OFF = 0,					/**< ... */
	QPDUNES_LOG_ITERATIONS,					/**< ... */
	QPDUNES_LOG_ALL_DATA					/**< ... */
} logLevel_t;


/** Local QP solver list */
typedef enum
{
	QPDUNES_STAGE_QP_SOLVER_UNDEFINED,		/**< ... */
	QPDUNES_STAGE_QP_SOLVER_CLIPPING,		/**< ... */
	QPDUNES_STAGE_QP_SOLVER_QPOASES			/**< ... */
} qp_solver_t;


/** Newton Hessian regularization types */
typedef enum
{
	QPDUNES_REG_LEVENBERG_MARQUARDT,				/**< 0 = ... */
	QPDUNES_REG_NORMALIZED_LEVENBERG_MARQUARDT,		/**< 1 = ... */
	QPDUNES_REG_SINGULAR_DIRECTIONS,				/**< 2 = regularize only in singular directions during Cholesky factorization */
	QPDUNES_REG_UNCONSTRAINED_HESSIAN,				/**< 3 = use unconstrained Hessian instead */
	QPDUNES_REG_ADD_UNCONSTRAINED_HESSIAN,			/**< 4 = add unconstrained Hessian multiplied by options.regParam */
	QPDUNES_REG_ADD_UNCONSTRAINED_HESSIAN_DIAG,		/**< 5 = add diagonal entried of the unconstrained Hessian, multiplied by options.regParam */
	QPDUNES_REG_GRADIENT_STEP						/**< 6 = ... */
} nwtnHssnRegType_t;


/** Newton Hessian factorization  algorithms */
typedef enum
{
	QPDUNES_NH_FAC_BAND_FORWARD,		/**< 0 = ... */
	QPDUNES_NH_FAC_BAND_REVERSE			/**< 1 = ... */
} nwtnHssnFacAlg_t;


/** Line search types */
typedef enum
{
	QPDUNES_LS_BACKTRACKING_LS,						/**< 0 = ... */
	QPDUNES_LS_BACKTRACKING_LS_WITH_AS_CHANGE,		/**< 1 = fast backtracking until progress, bisection until AS change */
	QPDUNES_LS_GOLDEN_SECTION_LS,					/**< 2 = ... */
	QPDUNES_LS_GRADIENT_BISECTION_LS,				/**< 3 = ... */
	QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS,	/**< 4 = fast backtracking first, then gradient based bisection for refinement */
	QPDUNES_LS_GRID_LS,								/**< 5 = evaluate objective function on a grid and take minimum */
	QPDUNES_LS_ACCELERATED_GRID_LS,					/**< 6 = fast backtracking first, then grid search for refinement */
	QPDUNES_LS_HOMOTOPY_GRID_SEARCH					/**< 7 = Grid search utilizing precomputed homotoppy parameterization */
} lineSearchType_t;


/** Error codes */
typedef enum
{
	QPDUNES_UNTERMINATED,
	QPDUNES_OK = 0,							/**< ... */

	QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND,
	QPDUNES_SUCC_SUBOPTIMAL_TERMINATION,
	QPDUNES_ERR_STAGE_QP_INFEASIBLE,
	QPDUNES_ERR_STAGE_COUPLING_INFEASIBLE,

	QPDUNES_ERR_UNKNOWN_ERROR,
	QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE,	/**< ... */
	QPDUNES_ERR_UNKNOWN_LS_TYPE,
	QPDUNES_ERR_INVALID_ARGUMENT,
	QPDUNES_ERR_ITERATION_LIMIT_REACHED,
	QPDUNES_ERR_DIVISION_BY_ZERO,
	QPDUNES_ERR_NUMBER_OF_MAX_LINESEARCH_ITERATIONS_REACHED,
	QPDUNES_ERR_DECEEDED_MIN_LINESEARCH_STEPSIZE,
	QPDUNES_ERR_EXCEEDED_MAX_LINESEARCH_STEPSIZE,
	QPDUNES_ERR_NEWTON_SYSTEM_NO_ASCENT_DIRECTION,
	QPDUNES_NOTICE_NEWTON_MATRIX_NOT_SET_UP
} return_t;


#endif	/* QPDUNES_TYPES_H */


/*
 *	end of file
 */
