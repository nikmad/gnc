/*
Author: Nikhil Madduri
Created: 31/Jul/2020
Modified: 01/Aug/2020 till 08/Oct/2020
*/

#include <stdio.h>
#include "math_util.h"
#include "functions_vtol.h"
#include "parameters_vtol.h"

void true_states(struct states states_out, struct force_n_moments fm_in, float* states_estimated)
{

// estimate states (using real state data)
    float pnhat    = states_out.pn;
    float pehat    = states_out.pe;
    float hhat     = -states_out.pd;
    float Vahat    = fm_in.Va;
    float alphahat = fm_in.alpha;
    float betahat  = fm_in.beta;
    float phihat   = states_out.phi;
    float thetahat = states_out.theta;
    float chihat   = atan2f(fm_in.Va*sinf(states_out.psi)+fm_in.w_e, fm_in.Va*cosf(states_out.psi)+fm_in.w_n);
    float phat     = states_out.p;
    float qhat     = states_out.q;
    float rhat     = states_out.r;
    float Vghat    = sqrtf(powf(fm_in.Va*cosf(states_out.psi)+fm_in.w_n,2) + powf(fm_in.Va*sinf(states_out.psi)+fm_in.w_e,2));
    float wnhat    = fm_in.w_n;
    float wehat    = fm_in.w_e;
    float psihat   = states_out.psi;
    float bxhat    = 0;
    float byhat    = 0;
    float bzhat    = 0;
    
    float xhat[] = {pnhat,
        pehat,
        hhat,
        Vahat,
        alphahat,
        betahat,
        phihat,
        thetahat,
        chihat,
        phat,
        qhat,
        rhat,
        Vghat,
        wnhat,
        wehat,
        psihat,
        bxhat,
        byhat,
        bzhat};

    for(int i=0;i<19;i++)
    {
        *(states_estimated+i)      = xhat[i];
    }

}
