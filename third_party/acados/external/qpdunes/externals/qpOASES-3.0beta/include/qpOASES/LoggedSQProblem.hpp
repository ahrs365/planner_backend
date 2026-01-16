/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2012 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file include/qpOASES/LoggedSQProblem.hpp
 *	\author Janick Frasch
 *	\version 3.0
 *	\date 2007-2014
 *
 *	Wrapper of SQProblem for additional qpDUNES functionality.
 */



#ifndef QPOASES_LOGGEDSQPROBLEM_HPP
#define QPOASES_LOGGEDSQPROBLEM_HPP


#include <qpOASES/SQProblem.hpp>


BEGIN_NAMESPACE_QPOASES


/** 
 *	\brief Wrapper of SQProblem for additional qpDUNES functionality.
 *
 *
 *	\author Janick Frasch
 *	\version 3.0
 *	\date 2007-2014
 */
class LoggedSQProblem : public SQProblem
{

	/*
	 *	PUBLIC MEMBER FUNCTIONS
	 */
	public:
//		/** Default constructor. */
//		SQProblem( );
//
		/** Constructor which takes the QP dimension and Hessian type
		 *  information. If the Hessian is the zero (i.e. HST_ZERO) or the
		 *  identity matrix (i.e. HST_IDENTITY), respectively, no memory
		 *  is allocated for it and a NULL pointer can be passed for it
		 *  to the init() functions. */
		LoggedSQProblem(	int _nV,	  							/**< Number of variables. */
							int _nC,  								/**< Number of constraints. */
							HessianType _hessianType = HST_UNKNOWN	/**< Type of Hessian matrix. */
							);
//
//		/** Copy constructor (deep copy). */
//		SQProblem(	const SQProblem& rhs	/**< Rhs object. */
//					);
//
//		/** Destructor. */
//		virtual ~SQProblem( );


	/** Solves QProblem using online active set strategy.
	 *  Note: This function internally calls solveQP/solveRegularisedQP
	 *        for solving an initialised QP!
	 *	\return SUCCESSFUL_RETURN \n
	 			RET_MAX_NWSR_REACHED \n
	 			RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED \n
				RET_HOTSTART_FAILED \n
				RET_SHIFT_DETERMINATION_FAILED \n
				RET_STEPDIRECTION_DETERMINATION_FAILED \n
				RET_STEPLENGTH_DETERMINATION_FAILED \n
				RET_HOMOTOPY_STEP_FAILED \n
				RET_HOTSTART_STOPPED_INFEASIBILITY \n
				RET_HOTSTART_STOPPED_UNBOUNDEDNESS */
	returnValue hotstart_withHomotopyLogging(	const real_t* const g_new,		/**< Gradient of neighbouring QP to be solved. */
							const real_t* const lb_new,		/**< Lower bounds of neighbouring QP to be solved. \n
												 			 	 If no lower bounds exist, a NULL pointer can be passed. */
							const real_t* const ub_new,		/**< Upper bounds of neighbouring QP to be solved. \n
												 			 	 If no upper bounds exist, a NULL pointer can be passed. */
							const real_t* const lbA_new,	/**< Lower constraints' bounds of neighbouring QP to be solved. \n
												 			 	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
							const real_t* const ubA_new,	/**< Upper constraints' bounds of neighbouring QP to be solved. \n
												 			 	 If no upper constraints' bounds exist, a NULL pointer can be passed. */
							real_t* const parametricObjFctn_alpha,			/**< log for homotopy kinks (active set changes) */
							real_t* const parametricObjFctn_f,				/**< log for objective value */
							real_t* const parametricObjFctn_fPrime,			/**< log for objective derivative in homotopy direction */
							real_t* const parametricObjFctn_fPrimePrime,	/**< log for objective second derivative in homotopy direction */
							int& nWSR,						/**< Input: Maximum number of working set recalculations; \n
														 		 Output: Number of performed working set recalculations. */
							real_t* const cputime = 0		/**< Input: Maximum CPU time allowed for QP solution. \n
															 	 Output: CPU time spend for QP solution (or to perform nWSR iterations). */
							);


	/** Returns the Cholesky factor R of the (projected) Hessian
	 *  (i.e. R^T*R = Z^T*H*Z).
	 *	\return SUCCESSFUL_RETURN \n
	 *			RET_QP_NOT_SOLVED */
	void getR(	real_t** const ROut	/**< Output: Pointer to upper triangular Cholesky factor of projected Hessian (after QP has been solved). */
				) const;

	/** Returns the projection matrix Z of the projected Hessian
	 *  Z^T*H*Z.
	 *	\return SUCCESSFUL_RETURN \n
	 *			RET_QP_NOT_SOLVED */
	void getQT(	real_t** const QTOut,	/**< Output: pointer to transposed optimal Hessian projection matrix  (after QP has been solved). */
				int* nZ					/**< Output: dimension of the null space. */
				) const;



	/*
	 *	PROTECTED MEMBER FUNCTIONS
	 */
	protected:

	/** Solves QProblem using online active set strategy.
	 *  Note: This function is internally called by all hotstart functions!
	 *	\return SUCCESSFUL_RETURN \n
	 			RET_MAX_NWSR_REACHED \n
	 			RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED \n
				RET_HOTSTART_FAILED \n
				RET_SHIFT_DETERMINATION_FAILED \n
				RET_STEPDIRECTION_DETERMINATION_FAILED \n
				RET_STEPLENGTH_DETERMINATION_FAILED \n
				RET_HOMOTOPY_STEP_FAILED \n
				RET_HOTSTART_STOPPED_INFEASIBILITY \n
				RET_HOTSTART_STOPPED_UNBOUNDEDNESS */
	returnValue solveQP_withHomotopyLogging(	const real_t* const g_new,		/**< Gradient of neighbouring QP to be solved. */
							const real_t* const lb_new,	/**< Lower bounds of neighbouring QP to be solved. \n
												 			 	 If no lower bounds exist, a NULL pointer can be passed. */
							const real_t* const ub_new,		/**< Upper bounds of neighbouring QP to be solved. \n
												 			 	 If no upper bounds exist, a NULL pointer can be passed. */
							const real_t* const lbA_new,	/**< Lower constraints' bounds of neighbouring QP to be solved. \n
												 			 	 If no lower constraints' bounds exist, a NULL pointer can be passed. */
							const real_t* const ubA_new,	/**< Upper constraints' bounds of neighbouring QP to be solved. \n
												 			 	 If no upper constraints' bounds exist, a NULL pointer can be passed. */
							real_t* const parametricObjFctn_alpha,			/**< log for homotopy kinks (active set changes) */
							real_t* const parametricObjFctn_f,				/**< log for objective value */
							real_t* const parametricObjFctn_fPrime,			/**< log for objective derivative in homotopy direction */
							real_t* const parametricObjFctn_fPrimePrime,	/**< log for objective second derivative in homotopy direction */
							int& nWSR,						/**< Input: Maximum number of working set recalculations; \n
														 		 Output: Number of performed working set recalculations. */
							real_t* const cputime,			/**< Input: Maximum CPU time allowed for QP solution. \n
															 	 Output: CPU time spend for QP solution (or to perform nWSR iterations). */
							int  nWSRperformed = 0			/**< Number of working set recalculations already performed to solve
																 this QP within previous solveQP() calls. This number is
																 always zero, except for successive calls from solveRegularisedQP()
																 or when using the far bound strategy. */
							);


	/** Removes a bounds from active set. ADAPTED FOR QPDUNES TO CLEAN ARRAYS IN A SAFE, REUSABLE WAY
	 *	\return SUCCESSFUL_RETURN \n
				RET_BOUND_NOT_ACTIVE \n
				RET_HESSIAN_NOT_SPD \n
				RET_REMOVEBOUND_FAILED */
	returnValue removeBound(	int number,								/**< Number of bound to be removed from active set. */
								BooleanType updateCholesky,				/**< Flag indicating if Cholesky decomposition shall be updated. */
								BooleanType allowFlipping = BT_FALSE,	/**< Flag indicating if flipping bounds are allowed. */
								BooleanType ensureNZC = BT_FALSE		/**< Flag indicating if non-zero curvature is ensured by exchange rules. */
								);


	/*
	 *	PROTECTED MEMBER VARIABLES
	 */
	protected:

};


END_NAMESPACE_QPOASES


#endif	/* QPOASES_LOGGEDSQPROBLEM_HPP */


/*
 *	end of file
 */
