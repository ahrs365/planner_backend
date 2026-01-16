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
 *	\file include/qp/qpDataStatic.h
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 *
 *	Declaration of all QP data types in a static fashion.
 */


#ifndef QPDUNES_QPDATASTATIC_H
#define QPDUNES_QPDATASTATIC_H

#include <qpDimensions.h>



/**
 *	\brief Matrix of size _NX_ by _NX_
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 */
typedef struct
{
	/** matrix sparsity specification */
	sparsityType_t sparsityType;

	/** matrix data array */
	real_t data[_NX_*_NX_];
} xx_matrix_t;


/**
 *	\brief Matrix of size _NX_ by _NU_
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 */
typedef struct
{
	/** matrix sparsity specification */
	sparsityType_t sparsityType;

	/** matrix data array */
	real_t data[_NX_*_NU_];
} xu_matrix_t;


/**
 *	\brief Matrix of size _NX_ by _NZ_
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 */
typedef struct
{
	/** matrix sparsity specification */
	sparsityType_t sparsityType;

	/** matrix data array */
	real_t data[_NX_*_NZ_];
} xz_matrix_t;


/**
 *	\brief Matrix of size _NU_ by _NX_
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 */
typedef struct
{
	/** matrix sparsity specification */
	sparsityType_t sparsityType;

	/** matrix data array */
	real_t data[_NU_*_NX_];
} ux_matrix_t;


/**
 *	\brief Matrix of size _NU_ by _NU_
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 */
typedef struct
{
	/** matrix sparsity specification */
	sparsityType_t sparsityType;

	/** matrix data array */
	real_t data[_NU_*_NU_];
} uu_matrix_t;


/**
 *	\brief Matrix of size _NZ_ by _NX_
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 */
typedef struct
{
	/** matrix sparsity specification */
	sparsityType_t sparsityType;

	/** matrix data array */
	real_t data[_NZ_*_NX_];
} zx_matrix_t;


/**
 *	\brief Matrix of size _NZ_ by _NZ_
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 */
typedef struct
{
	/** matrix sparsity specification */
	sparsityType_t sparsityType;

	/** matrix data array */
	real_t data[_NZ_*_NZ_];
} zz_matrix_t;


/**
 *	\brief Matrix of size _NZ_ by _NZ_
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 */
typedef struct
{
	/** matrix sparsity specification */
	sparsityType_t sparsityType;

	/** matrix data array */
	real_t data[_NZ_*_NZ_];
todo! check where v is used!
} vv_matrix_t;


/**
 *	\brief Matrix of size _NDMAX_ by _NZ_
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 */
typedef struct
{
	/** matrix sparsity specification */
	sparsityType_t sparsityType;

	/** matrix data array */
	real_t data[_NDMAX_*_NZ_];
} dz_matrix_t;




/**
 * Special Newton hessian storage format:
 *
 *  A symmetric block tri-diagonal Newton Hessian
 * 		[ D L'       ]
 * 		[ L D L'     ]
 * 		[   L D L'   ]
 * 		[      ...   ]
 * 		[        L D ]
 *
 *  is stored as
 * 		[ - D ]
 * 		[ L D ]
 * 		[ L D ]
 * 		[ ... ]
 * 		[ L D ]
 *
 */
typedef struct
{
	/** matrix data array */
	real_t data[_NI_*2*_NX_];
} xn2x_matrix_t;

//typedef matrix_t xnxn_matrix_t;



/**
 *	\brief generic vector data type
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
typedef struct
{
	/** vector property flags */
	boolean_t isDefined;
	boolean_t hasChanged;

	/** vector data array */
	real_t* data;
} vector_t;

typedef vector_t x_vector_t;
typedef vector_t u_vector_t;
typedef vector_t z_vector_t;
typedef vector_t d_vector_t;
typedef vector_t d2_vector_t;
typedef vector_t d2n1_vector_t;
typedef vector_t xn_vector_t;
typedef vector_t zn_vector_t;
typedef vector_t zn1_vector_t;
typedef vector_t large_vector_t;



/**
 *	\brief generic integer vector data type
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
typedef struct
{
	/** vector property flags */
	boolean_t isDefined;
	boolean_t hasChanged;

	/** vector data array */
	int_t* data;
} intVector_t;

typedef intVector_t zn_intVector_t;


/**
 *	\brief function pointers for qpOASES methods
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2013
 */
//typedef int (*qpoasesInitFctnPntr)( const real_t* const _H, const real_t* const _g, const real_t* const _A,
//							const real_t* const _lb, const real_t* const _ub,
//							const real_t* const _lbA, const real_t* const _ubA,
//							int& nWSR );
//typedef int (*qpoasesHotstartFctnPntr)( const real_t* const g_new,
//								const real_t* const lb_new, const real_t* const ub_new,
//								const real_t* const lbA_new, const real_t* const ubA_new,
//								int& nWSR );
//typedef void (*qpoasesGetPrimalSolutionFctnPntr)( real_t* const xOpt );
//typedef real_t (*qpoasesGetObjValFctnPntr)();


/**
 *	\brief pointer to qpOASES object for C++ method access
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2013
 */
typedef void qpoases_t;
typedef void qpoasesOptions_t;
typedef struct
{
	qpoases_t* qpoases;
	qpoasesOptions_t* options;
} qpoasesObject_t;


/**
 *	\brief struct with auxiliary data for QPOASES QP solver
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2013
 */
typedef struct
{
	qpoasesObject_t* qpoasesObject;

	/* workspace */
	z_vector_t qFullStep;			/**< linear term corresponding to full-step in lambda */
	real_t pFullStep;				/**< constant term corresponding to full-step in lambda */
} qpSolverQpoases_t;



/**
 *	\brief struct with auxiliary data for clipping QP solver
 *
 *	...
 *
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2013
 */
typedef struct
{
	z_vector_t zUnconstrained;	/**< unconstrained primal solution for current lambda guess */
	z_vector_t dz;				/**< delta z - update in primal variables corresponding to a full step deltaLambda */

	/* workspace */
	z_vector_t qStep;			/**< step in linear term for line search */
	real_t pStep;				/**< step in constant term for line search */
} qpSolverClipping_t;


/**
 *	\brief Hessian interval data type and dynamic constraint interval data type
 *
 *	Datatype for one interval of the QP data.
 *
 *  Structure of the Hessian block:
 *   (x)' (Q  S) (x)
 *   (u)  (S' R) (u)
 *
 *  Structure of interval dynamics:
 *   x_{k+1} = A_k*x_k + B_k*u_k + c_k
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
typedef struct
{
	uint_t id;					/**< stage index */


	/* dimensions */
	uint_t nD;					/**< number of constraints */
	uint_t nV;					/**< number of variables */


	/* primal objective function */
	vv_matrix_t H;				/**< Hessian */
	vv_matrix_t cholH;			/**< inverse of Hessian */
	real_t HQNorm;				/**< norm of Q-part of Hessian */     //FIXME: choose which norm to compute exactly, etc.

	z_vector_t g;				/**< primal gradient block */


	/* dualized objective function */
	z_vector_t q;				/**< linear objective function term after dualization */
	real_t p;					/**< constant objective function term after dualization */


	/* dynamic system */
	xz_matrix_t C;				/**< u-part of constraint matrix */
	x_vector_t  c;				/**< constant part */


	/* constraints */
	z_vector_t  zLow;			/**< lower variable bound */
	z_vector_t  zUpp;			/**< upper variable bound */
	dz_matrix_t D;				/**< full constraint matrix */
	d_vector_t  dLow;			/**< constraint lower bound */
	d_vector_t  dUpp;			/**< constraint upper bound */


	/* primal QP solution */
	z_vector_t z;				/**< full primal solution for current lambda guess */
	real_t optObjVal;			/**< objective value */


	/* dual QP solution */
	d2_vector_t y;				/**< stage constraint multiplier vector  */
	d2_vector_t yPrev;			/**< previous stage constraint multiplier vector (needed to detect AS changes)  */


	/* QP solver */
	qp_solver_t qpSolverSpecification;		/**< dedicated QP solver */

	qpSolverClipping_t qpSolverClipping;	/**< workspace for clipping QP solver */
	qpSolverQpoases_t qpSolverQpoases;		/**< pointer to qpOASES object */

	boolean_t rebuildHessianBlock;				/**< indicator flag whether an active set change occurred on this
										     	 interval during the current iteration */


	/* memory for objective function parameterization (used optionally in line search) */
	large_vector_t parametricObjFctn_alpha;
	large_vector_t parametricObjFctn_f;
	large_vector_t parametricObjFctn_fPrime;
	large_vector_t parametricObjFctn_fPrimePrime;
	int_t parametricObjFctn_nBasePoints;

	large_vector_t parametricObjFctn_fSum;			/**< full dual objective value at alpha values of this stage */


	/* workspace */
	x_vector_t lambdaK;		/**<  */
	x_vector_t lambdaK1;	/**<  */

	x_vector_t xVecTmp;			/**<  */
	u_vector_t uVecTmp;			/**<  */
	z_vector_t zVecTmp;			/**<  */

} interval_t;



/**
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
typedef struct
{
	/* iteration limits */
	int_t maxIter;
	int_t maxNumLineSearchIterations;			/**< maximum number of line search steps in solution of Newton system */
	int_t maxNumLineSearchRefinementIterations;	/**< maximum number of refinement line search steps to find point with AS change */
	int_t maxNumQpoasesIterations;				/**< maximum number of qpOASES working set recalculations */

	/* printing */
	int_t printLevel;							/**< Amount of information printed:   0 = no output
																					  1 = only errors and success
																					  2 = additionally iterations and warnings
																					  3 = debug information
																					  */
	/* logging */
	logLevel_t logLevel;						/**< Amount of information logged */

	int_t printIntervalHeader;
	boolean_t printIterationTiming;
	boolean_t printLineSearchTiming;
	/* TODO: use print precision here for variable precision matrix and vector printing */

	/* numerical tolerances */
	real_t stationarityTolerance;
	real_t equalityTolerance;
	real_t newtonHessDiagRegTolerance;	/**< Tolerance on diagonal elements of Newton Hessian before regularizing */
	real_t activenessTolerance;			/**< Tolerance in constraint violation before a constraint is considered active => needed to avoid weakly active constraints, which might yield suboptimal Hessian directions */
	/* TODO: check if ZERO needed, and whether equality tolerance is not enough */
	real_t QPDUNES_ZERO;					/** Numerical value of zero (for situations in which it would be unreasonable to compare with 0.0). Has to be positive. */
	real_t QPDUNES_INFTY;					/**< Numerical value of infinity (e.g. for non-existing bounds). Has to be positive. */
	real_t ascentCurvatureTolerance;	/**< Tolerance when a step is called a zero curvature step */

	/* additional options */
	int_t nbrInitialGradientSteps;			/**< after the first Newton step a number of cheaper gradient
											 	 steps with line search can be used to drive the method
											 	 faster to the solution */
	boolean_t checkForInfeasibility;		/**< perform checks for infeasibility of the problem */
	boolean_t allowSuboptimalTermination;	/**< permits regular termination after reaching iteration limit (when dual still suboptimal) */

	/* regularization options */
	nwtnHssnRegType_t regType;
	real_t regParam;					/**< Levenberg-Marquardt relaxation parameter */

	nwtnHssnFacAlg_t nwtnHssnFacAlg;

	/* line search options */
	lineSearchType_t lsType;
	real_t lineSearchReductionFactor;
	real_t lineSearchIncreaseFactor;
	real_t lineSearchMinAbsProgress;
	real_t lineSearchMinRelProgress;
	real_t lineSearchStationarityTolerance;
	real_t lineSearchMaxStepSize;
	int_t lineSearchNbrGridPoints;		/**< number of grid points for grid line search */

	/* qpOASES options */
	real_t qpOASES_terminationTolerance;

} qpOptions_t;



/**
 *	\brief log type for single iteration
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
typedef struct
{
	/* data: vectors, matrices */
	xn_vector_t lambda;
	xn_vector_t deltaLambda;

	xn2x_matrix_t hessian;
	xn2x_matrix_t cholHessian;
	xn_vector_t gradient;

	zn1_vector_t dz;
	zn1_vector_t zUnconstrained;
	zn1_vector_t z;

	d2n1_vector_t y;

	xn_vector_t regDirections; 		/* log the regularized directions delta, for Newton system (H+diag(delta))*lambda = -g */

	#ifdef __ANALYZE_FACTORIZATION__
	xnxn_matrix_t invHessian;
	#endif

	/* timings */
	real_t tIt;
	real_t tNwtnSetup;
	real_t tNwtnSolve;
	real_t tQP;
	real_t tLineSearch;

	/* statuses */
	real_t gradNorm;
	real_t lambdaNorm;
	real_t stepNorm;
	real_t stepSize;
	real_t objVal;

	uint_t nActConstr;
	uint_t nChgdConstr;
	int_t hessRefactorIdx;


	/* flags, etc. */
	uint_t itNbr;
	boolean_t isHessianRegularized;
	uint_t numLineSearchIter;
	int_t* numQpoasesIter;

} itLog_t;


/**
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
typedef struct
{
	/* Problem dimensions */
	uint_t nI;
	uint_t nX;
	uint_t nU;
	uint_t nZ;
	uint_t nDttl;				/**< total number of local constraints */

	/* Problem data */
	interval_t** intervals;

	/* options */
	qpOptions_t qpOptions;

	/* iterations log */
	itLog_t* itLog;

	int_t numIter;

} log_t;



/**
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
typedef struct
{
	/* variables */
	uint_t nI;
	uint_t nX;
	uint_t nU;
	uint_t nZ;
	uint_t nDttl;				/**< total number of local constraints */

	interval_t** intervals;		/**< array of pointers to interval structs; double pointer for more efficient shifting */

	xn_vector_t lambda;
	xn_vector_t deltaLambda;

	xn2x_matrix_t hessian;
	xn2x_matrix_t cholHessian;
	xn_vector_t gradient;

	xn2x_matrix_t cholDefaultHessian;

//	int_t* ieqStatus;
//	int_t* prevIeqStatus;

//	real_t nActConstr;		/* TODO: move to history/log object */
//	real_t nChgdConstr;		/* TODO: move to history object */

	real_t alpha;
	real_t optObjVal;

	qpOptions_t options;

	/* flags, etc. */
	/* TODO: move to log object */
//	boolean_t isHessianRegularized;
//	int_t numIter;
//	int_t numLineSearchIter;

	/* workspace */
	x_vector_t xVecTmp;			/**<  */
	u_vector_t uVecTmp;			/**<  */
	z_vector_t zVecTmp;			/**<  */
	xn_vector_t xnVecTmp;		/**<  */
	xn_vector_t xnVecTmp2;		/**<  */

	xx_matrix_t xxMatTmp;		/**<  */
	xx_matrix_t xxMatTmp2;		/**<  */
	ux_matrix_t uxMatTmp;		/**<  */
	xz_matrix_t xzMatTmp;		/**<  */
	zx_matrix_t zxMatTmp;		/**<  */
	zz_matrix_t zzMatTmp;		/**<  */
	zz_matrix_t zzMatTmp2;		/**<  */

	/* log */
	log_t log;

} qpData_t;



#endif /* QPDUNES_QPDATASTATIC_H */



/*
 *	end of file
 */
