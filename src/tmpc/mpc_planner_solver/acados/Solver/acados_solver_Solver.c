/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "Solver_model/Solver_model.h"


#include "Solver_constraints/Solver_constraints.h"
#include "Solver_cost/Solver_cost.h"



#include "acados_solver_Solver.h"

#define NX     SOLVER_NX
#define NZ     SOLVER_NZ
#define NU     SOLVER_NU
#define NP     SOLVER_NP
#define NY0    SOLVER_NY0
#define NY     SOLVER_NY
#define NYN    SOLVER_NYN

#define NBX    SOLVER_NBX
#define NBX0   SOLVER_NBX0
#define NBU    SOLVER_NBU
#define NG     SOLVER_NG
#define NBXN   SOLVER_NBXN
#define NGN    SOLVER_NGN

#define NH     SOLVER_NH
#define NHN    SOLVER_NHN
#define NH0    SOLVER_NH0
#define NPHI   SOLVER_NPHI
#define NPHIN  SOLVER_NPHIN
#define NPHI0  SOLVER_NPHI0
#define NR     SOLVER_NR

#define NS     SOLVER_NS
#define NS0    SOLVER_NS0
#define NSN    SOLVER_NSN

#define NSBX   SOLVER_NSBX
#define NSBU   SOLVER_NSBU
#define NSH0   SOLVER_NSH0
#define NSH    SOLVER_NSH
#define NSHN   SOLVER_NSHN
#define NSG    SOLVER_NSG
#define NSPHI0 SOLVER_NSPHI0
#define NSPHI  SOLVER_NSPHI
#define NSPHIN SOLVER_NSPHIN
#define NSGN   SOLVER_NSGN
#define NSBXN  SOLVER_NSBXN



// ** solver data **

Solver_solver_capsule * Solver_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(Solver_solver_capsule));
    Solver_solver_capsule *capsule = (Solver_solver_capsule *) capsule_mem;

    return capsule;
}


int Solver_acados_free_capsule(Solver_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int Solver_acados_create(Solver_solver_capsule* capsule)
{
    int N_shooting_intervals = SOLVER_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return Solver_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int Solver_acados_update_time_steps(Solver_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "Solver_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for Solver_acados_create: step 1
 */
void Solver_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;

    nlp_solver_plan->nlp_cost[0] = EXTERNAL;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = LINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = MIRROR;
}


static ocp_nlp_dims* Solver_acados_create_setup_dimensions(Solver_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    nbxe[0] = 5;
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);

    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for Solver_acados_create: step 3
 */
void Solver_acados_create_setup_functions(Solver_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__ ); \
    } while(false)


    // constraints.constr_type == "BGH" and dims.nh > 0
    capsule->nl_constr_h_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun_jac[i], Solver_constr_h_fun_jac_uxt_zt);
    }
    capsule->nl_constr_h_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun[i], Solver_constr_h_fun);
    }
    
    capsule->nl_constr_h_fun_jac_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun_jac_hess[i], Solver_constr_h_fun_jac_uxt_zt_hess);
    }
    

    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun, Solver_cost_ext_cost_0_fun);
    MAP_CASADI_FNC(ext_cost_0_fun_jac, Solver_cost_ext_cost_0_fun_jac);
    MAP_CASADI_FNC(ext_cost_0_fun_jac_hess, Solver_cost_ext_cost_0_fun_jac_hess);

    

    




    // explicit ode
    capsule->expl_vde_forw = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_vde_forw[i], Solver_expl_vde_forw);
    }

    capsule->expl_ode_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_fun[i], Solver_expl_ode_fun);
    }

    capsule->expl_vde_adj = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_vde_adj[i], Solver_expl_vde_adj);
    }
    capsule->expl_ode_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_hess[i], Solver_expl_ode_hess);
    }


    // external cost
    capsule->ext_cost_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun[i], Solver_cost_ext_cost_fun);
    }

    capsule->ext_cost_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun_jac[i], Solver_cost_ext_cost_fun_jac);
    }

    capsule->ext_cost_fun_jac_hess = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun_jac_hess[i], Solver_cost_ext_cost_fun_jac_hess);
    }

    

    

#undef MAP_CASADI_FNC
}


/**
 * Internal function for Solver_acados_create: step 4
 */
void Solver_acados_create_set_default_parameters(Solver_solver_capsule* capsule) {
    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));

    for (int i = 0; i <= N; i++) {
        Solver_acados_update_params(capsule, i, p, NP);
    }
    free(p);
}


/**
 * Internal function for Solver_acados_create: step 5
 */
void Solver_acados_setup_nlp_in(Solver_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps

    if (new_time_steps)
    {
        Solver_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {double time_step = 0.2;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->expl_vde_forw[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_vde_adj", &capsule->expl_vde_adj[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "expl_ode_hess", &capsule->expl_ode_hess[i]);
    }

    /**** Cost ****/
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun", &capsule->ext_cost_0_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac", &capsule->ext_cost_0_fun_jac);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac_hess", &capsule->ext_cost_0_fun_jac_hess);
    
    
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun", &capsule->ext_cost_fun[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac", &capsule->ext_cost_fun_jac[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac_hess", &capsule->ext_cost_fun_jac_hess[i-1]);
        
        
    }






    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(5 * sizeof(int));
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);








    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    
    lbu[0] = -2;
    ubu[0] = 2;
    lbu[1] = -0.8;
    ubu[1] = 0.8;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);








    // x
    int* idxbx = malloc(NBX * sizeof(int));
    
    idxbx[0] = 0;
    idxbx[1] = 1;
    idxbx[2] = 2;
    idxbx[3] = 3;
    idxbx[4] = 4;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    
    lbx[0] = -2000;
    ubx[0] = 2000;
    lbx[1] = -2000;
    ubx[1] = 2000;
    lbx[2] = -12.566370614359172;
    ubx[2] = 12.566370614359172;
    lbx[3] = -0.01;
    ubx[3] = 3;
    lbx[4] = -1;
    ubx[4] = 10000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);




    // set up nonlinear constraints for stage 1 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;

    
    lh[0] = -1000000000000000;
    lh[1] = -1000000000000000;
    lh[2] = -1000000000000000;
    lh[3] = -1000000000000000;
    lh[4] = -1000000000000000;
    lh[5] = -1000000000000000;
    lh[6] = -1000000000000000;
    lh[7] = -1000000000000000;
    lh[8] = -1000000000000000;
    lh[9] = -1000000000000000;
    lh[10] = -1000000000000000;
    lh[11] = -1000000000000000;
    lh[12] = -1000000000000000;
    lh[13] = -1000000000000000;
    lh[14] = -1000000000000000;
    lh[15] = -1000000000000000;
    lh[16] = -1000000000000000;
    lh[17] = -1000000000000000;
    lh[18] = -1000000000000000;
    lh[19] = -1000000000000000;
    lh[20] = -1000000000000000;
    lh[21] = -1000000000000000;
    lh[22] = -1000000000000000;
    lh[23] = -1000000000000000;
    lh[24] = -1000000000000000;
    lh[25] = -1000000000000000;
    lh[26] = -1000000000000000;
    lh[27] = -1000000000000000;
    lh[28] = -1000000000000000;
    lh[29] = -1000000000000000;
    lh[30] = -1000000000000000;
    lh[31] = -1000000000000000;
    lh[32] = -1000000000000000;
    lh[33] = -1000000000000000;
    lh[34] = -1000000000000000;
    lh[35] = -1000000000000000;
    lh[36] = -1000000000000000;
    lh[37] = -1000000000000000;
    lh[38] = -1000000000000000;
    lh[39] = -1000000000000000;
    lh[40] = -1000000000000000;
    lh[41] = -1000000000000000;
    lh[42] = -1000000000000000;
    lh[43] = -1000000000000000;
    lh[44] = -1000000000000000;
    lh[45] = -1000000000000000;
    lh[46] = -1000000000000000;
    lh[47] = -1000000000000000;
    lh[48] = -1000000000000000;
    lh[49] = -1000000000000000;
    lh[50] = -1000000000000000;
    lh[51] = -1000000000000000;
    lh[52] = -1000000000000000;
    lh[53] = -1000000000000000;
    lh[54] = -1000000000000000;
    lh[55] = -1000000000000000;
    lh[56] = -1000000000000000;
    lh[57] = -1000000000000000;
    lh[58] = -1000000000000000;
    lh[59] = -1000000000000000;
    lh[60] = -1000000000000000;
    lh[61] = -1000000000000000;
    lh[62] = -1000000000000000;
    lh[63] = -1000000000000000;
    lh[64] = -1000000000000000;
    lh[65] = -1000000000000000;
    lh[66] = -1000000000000000;
    lh[67] = -1000000000000000;
    lh[68] = -1000000000000000;
    lh[69] = -1000000000000000;
    lh[70] = -1000000000000000;
    lh[71] = -1000000000000000;
    lh[72] = -1000000000000000;
    lh[73] = -1000000000000000;
    lh[74] = -1000000000000000;
    lh[75] = -1000000000000000;
    lh[76] = -1000000000000000;
    lh[77] = -1000000000000000;
    lh[78] = -1000000000000000;
    lh[79] = -1000000000000000;
    lh[80] = -1000000000000000;
    lh[81] = -1000000000000000;
    lh[82] = -1000000000000000;
    lh[83] = -1000000000000000;
    lh[84] = -1000000000000000;
    lh[85] = -1000000000000000;
    lh[86] = -1000000000000000;
    lh[87] = -1000000000000000;
    lh[88] = -1000000000000000;
    lh[89] = -1000000000000000;
    lh[90] = -1000000000000000;
    lh[91] = -1000000000000000;
    lh[92] = -1000000000000000;
    lh[93] = -1000000000000000;
    lh[94] = -1000000000000000;
    lh[95] = -1000000000000000;
    lh[96] = -1000000000000000;
    lh[97] = -1000000000000000;
    lh[98] = -1000000000000000;
    lh[99] = -1000000000000000;
    lh[100] = 1;
    lh[101] = 1;
    lh[102] = 1;
    lh[103] = 1;
    lh[104] = 1;
    lh[105] = 1;
    lh[106] = 1;
    lh[107] = 1;
    lh[108] = 1;
    lh[109] = 1;
    lh[110] = 1;
    lh[111] = 1;
    lh[112] = 1;
    lh[113] = 1;
    lh[114] = 1;
    lh[115] = 1;
    lh[116] = 1;
    lh[117] = 1;
    lh[118] = 1;
    lh[119] = 1;
    lh[120] = 1;
    lh[121] = 1;
    lh[122] = 1;
    lh[123] = 1;
    lh[124] = 1;
    lh[125] = 1;
    lh[126] = 1;
    lh[127] = 1;
    lh[128] = 1;
    lh[129] = 1;
    lh[130] = 1;
    lh[131] = 1;
    lh[132] = 1;
    lh[133] = 1;
    lh[134] = 1;
    lh[135] = 1;
    lh[136] = 1;
    lh[137] = 1;
    lh[138] = 1;
    lh[139] = 1;
    lh[140] = 1;
    lh[141] = 1;
    lh[142] = 1;
    lh[143] = 1;
    lh[144] = 1;
    lh[145] = 1;
    lh[146] = 1;
    lh[147] = 1;
    lh[148] = 1;
    lh[149] = 1;
    lh[150] = 1;
    lh[151] = 1;
    lh[152] = 1;
    lh[153] = 1;
    lh[154] = 1;
    lh[155] = 1;
    lh[156] = 1;
    lh[157] = 1;
    lh[158] = 1;
    lh[159] = 1;
    lh[160] = 1;
    lh[161] = 1;
    lh[162] = 1;
    lh[163] = 1;
    lh[164] = 1;
    lh[165] = 1;
    lh[166] = 1;
    lh[167] = 1;
    lh[168] = 1;
    lh[169] = 1;
    lh[170] = 1;
    lh[171] = 1;
    lh[172] = 1;
    lh[173] = 1;
    lh[174] = 1;
    lh[175] = 1;
    lh[176] = 1;
    lh[177] = 1;
    lh[178] = 1;
    lh[179] = 1;
    lh[180] = 1;
    lh[181] = 1;
    lh[182] = 1;
    lh[183] = 1;
    lh[184] = 1;
    lh[185] = 1;
    lh[186] = 1;
    lh[187] = 1;
    lh[188] = 1;
    lh[189] = 1;
    lh[190] = 1;
    lh[191] = 1;
    lh[192] = 1;
    lh[193] = 1;
    lh[194] = 1;
    lh[195] = 1;
    lh[196] = 1;
    lh[197] = 1;
    lh[198] = 1;
    lh[199] = 1;

    
    uh[100] = 1000000000000000;
    uh[101] = 1000000000000000;
    uh[102] = 1000000000000000;
    uh[103] = 1000000000000000;
    uh[104] = 1000000000000000;
    uh[105] = 1000000000000000;
    uh[106] = 1000000000000000;
    uh[107] = 1000000000000000;
    uh[108] = 1000000000000000;
    uh[109] = 1000000000000000;
    uh[110] = 1000000000000000;
    uh[111] = 1000000000000000;
    uh[112] = 1000000000000000;
    uh[113] = 1000000000000000;
    uh[114] = 1000000000000000;
    uh[115] = 1000000000000000;
    uh[116] = 1000000000000000;
    uh[117] = 1000000000000000;
    uh[118] = 1000000000000000;
    uh[119] = 1000000000000000;
    uh[120] = 1000000000000000;
    uh[121] = 1000000000000000;
    uh[122] = 1000000000000000;
    uh[123] = 1000000000000000;
    uh[124] = 1000000000000000;
    uh[125] = 1000000000000000;
    uh[126] = 1000000000000000;
    uh[127] = 1000000000000000;
    uh[128] = 1000000000000000;
    uh[129] = 1000000000000000;
    uh[130] = 1000000000000000;
    uh[131] = 1000000000000000;
    uh[132] = 1000000000000000;
    uh[133] = 1000000000000000;
    uh[134] = 1000000000000000;
    uh[135] = 1000000000000000;
    uh[136] = 1000000000000000;
    uh[137] = 1000000000000000;
    uh[138] = 1000000000000000;
    uh[139] = 1000000000000000;
    uh[140] = 1000000000000000;
    uh[141] = 1000000000000000;
    uh[142] = 1000000000000000;
    uh[143] = 1000000000000000;
    uh[144] = 1000000000000000;
    uh[145] = 1000000000000000;
    uh[146] = 1000000000000000;
    uh[147] = 1000000000000000;
    uh[148] = 1000000000000000;
    uh[149] = 1000000000000000;
    uh[150] = 1000000000000000;
    uh[151] = 1000000000000000;
    uh[152] = 1000000000000000;
    uh[153] = 1000000000000000;
    uh[154] = 1000000000000000;
    uh[155] = 1000000000000000;
    uh[156] = 1000000000000000;
    uh[157] = 1000000000000000;
    uh[158] = 1000000000000000;
    uh[159] = 1000000000000000;
    uh[160] = 1000000000000000;
    uh[161] = 1000000000000000;
    uh[162] = 1000000000000000;
    uh[163] = 1000000000000000;
    uh[164] = 1000000000000000;
    uh[165] = 1000000000000000;
    uh[166] = 1000000000000000;
    uh[167] = 1000000000000000;
    uh[168] = 1000000000000000;
    uh[169] = 1000000000000000;
    uh[170] = 1000000000000000;
    uh[171] = 1000000000000000;
    uh[172] = 1000000000000000;
    uh[173] = 1000000000000000;
    uh[174] = 1000000000000000;
    uh[175] = 1000000000000000;
    uh[176] = 1000000000000000;
    uh[177] = 1000000000000000;
    uh[178] = 1000000000000000;
    uh[179] = 1000000000000000;
    uh[180] = 1000000000000000;
    uh[181] = 1000000000000000;
    uh[182] = 1000000000000000;
    uh[183] = 1000000000000000;
    uh[184] = 1000000000000000;
    uh[185] = 1000000000000000;
    uh[186] = 1000000000000000;
    uh[187] = 1000000000000000;
    uh[188] = 1000000000000000;
    uh[189] = 1000000000000000;
    uh[190] = 1000000000000000;
    uh[191] = 1000000000000000;
    uh[192] = 1000000000000000;
    uh[193] = 1000000000000000;
    uh[194] = 1000000000000000;
    uh[195] = 1000000000000000;
    uh[196] = 1000000000000000;
    uh[197] = 1000000000000000;
    uh[198] = 1000000000000000;
    uh[199] = 1000000000000000;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i-1]);
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i-1]);
        
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i,
                                      "nl_constr_h_fun_jac_hess", &capsule->nl_constr_h_fun_jac_hess[i-1]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
    }
    free(luh);



    /* terminal constraints */













}


static void Solver_acados_create_set_opts(Solver_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    int nlp_solver_exact_hessian = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess", &nlp_solver_exact_hessian);

    int exact_hess_dyn = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_dyn", &exact_hess_dyn);

    int exact_hess_cost = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_cost", &exact_hess_cost);

    int exact_hess_constr = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_constr", &exact_hess_constr);int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization", "fixed_step");
int with_solution_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params", &with_solution_sens_wrt_params);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    int full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "full_step_dual", &full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);

    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 30;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);
    double reg_epsilon = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "reg_epsilon", &reg_epsilon);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");




    int as_rti_iter = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_iter", &as_rti_iter);

    int as_rti_level = 4;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "as_rti_level", &as_rti_level);

    int rti_log_residuals = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_log_residuals", &rti_log_residuals);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);


    double qp_solver_tol_stat = 0.00001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_stat", &qp_solver_tol_stat);
    double qp_solver_tol_eq = 0.00001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_eq", &qp_solver_tol_eq);
    double qp_solver_tol_ineq = 0.00001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_ineq", &qp_solver_tol_ineq);
    double qp_solver_tol_comp = 0.00001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_tol_comp", &qp_solver_tol_comp);
    int qp_solver_warm_start = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_warm_start", &qp_solver_warm_start);

    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
}


/**
 * Internal function for Solver_acados_create: step 7
 */
void Solver_acados_set_nlp_out(Solver_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for Solver_acados_create: step 8
 */
//void Solver_acados_create_8_create_solver(Solver_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for Solver_acados_create: step 9
 */
int Solver_acados_create_precompute(Solver_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int Solver_acados_create_with_discretization(Solver_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != SOLVER_N && !new_time_steps) {
        fprintf(stderr, "Solver_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, SOLVER_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    Solver_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = Solver_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    Solver_acados_create_set_opts(capsule);

    // 4) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 5) setup functions, nlp_in and default parameters
    Solver_acados_create_setup_functions(capsule);
    Solver_acados_setup_nlp_in(capsule, N, new_time_steps);
    Solver_acados_create_set_default_parameters(capsule);

    // 6) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    Solver_acados_set_nlp_out(capsule);

    // 8) do precomputations
    int status = Solver_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int Solver_acados_update_qp_solver_cond_N(Solver_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from Solver_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);

    // -> 9) do precomputations
    int status = Solver_acados_create_precompute(capsule);
    return status;
}


int Solver_acados_reset(Solver_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+2*NS0+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NH0+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}




int Solver_acados_update_params(Solver_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 1055;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int Solver_acados_update_params_sparse(Solver_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int Solver_acados_set_p_global_and_precompute_dependencies(Solver_solver_capsule* capsule, double* data, int data_len)
{

    printf("p_global is not defined, Solver_acados_set_p_global_and_precompute_dependencies does nothing.\n");
}




int Solver_acados_solve(Solver_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


void Solver_acados_batch_solve(Solver_solver_capsule ** capsules, int N_batch)
{

    for (int i = 0; i < N_batch; i++)
    {
        ocp_nlp_solve(capsules[i]->nlp_solver, capsules[i]->nlp_in, capsules[i]->nlp_out);
    }


    return;
}


int Solver_acados_free(Solver_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->expl_vde_forw[i]);
        external_function_external_param_casadi_free(&capsule->expl_ode_fun[i]);
        external_function_external_param_casadi_free(&capsule->expl_vde_adj[i]);
        external_function_external_param_casadi_free(&capsule->expl_ode_hess[i]);
    }
    free(capsule->expl_vde_adj);
    free(capsule->expl_vde_forw);
    free(capsule->expl_ode_fun);
    free(capsule->expl_ode_hess);

    // cost
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac);
    external_function_external_param_casadi_free(&capsule->ext_cost_0_fun_jac_hess);
    
    
    for (int i = 0; i < N - 1; i++)
    {
        external_function_external_param_casadi_free(&capsule->ext_cost_fun[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->ext_cost_fun_jac_hess[i]);
        
        
    }
    free(capsule->ext_cost_fun);
    free(capsule->ext_cost_fun_jac);
    free(capsule->ext_cost_fun_jac_hess);

    // constraints
    for (int i = 0; i < N-1; i++)
    {
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac_hess[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);
    free(capsule->nl_constr_h_fun_jac_hess);



    return 0;
}


void Solver_acados_print_stats(Solver_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[1200];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            tmp_int = (int) stat[i + j * nrow];
            printf("%d\t", tmp_int);
        }
        printf("\n");
    }
}

int Solver_acados_custom_update(Solver_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *Solver_acados_get_nlp_in(Solver_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *Solver_acados_get_nlp_out(Solver_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *Solver_acados_get_sens_out(Solver_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *Solver_acados_get_nlp_solver(Solver_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *Solver_acados_get_nlp_config(Solver_solver_capsule* capsule) { return capsule->nlp_config; }
void *Solver_acados_get_nlp_opts(Solver_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *Solver_acados_get_nlp_dims(Solver_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *Solver_acados_get_nlp_plan(Solver_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
