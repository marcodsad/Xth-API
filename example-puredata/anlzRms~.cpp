#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>
#define WINDOW 512

static t_class *anlzRms_tilde_class;

typedef struct _anlzRms_tilde
{
    t_object x_obj;
    xth *xthInst;
    xthConfig config;

    t_float sr;
    t_float n;

    int dspTicks;

    t_sample *signal;

    t_outlet *x_output;
    t_float x_f;

} t_anlzRms_tilde;


/* ------------------------ anlzRms~ -------------------------------- */

static void *anlzRms_tilde_new(t_symbol *s, int argc, t_atom *argv)
{
    t_anlzRms_tilde *x = (t_anlzRms_tilde *)pd_new(anlzRms_tilde_class);
	int i;

	x->xthInst = new xth(x->config);

	x->x_output = outlet_new(&x->x_obj, &s_float);
	s=s;
	
	x->sr = 44100.0;
	x->n = 64.0;
	x->dspTicks = 0;

	x->signal = (t_sample *)t_getbytes((WINDOW+x->n) * sizeof(t_sample));

 	for(i=0; i<(WINDOW+x->n); i++)
		x->signal[i] = 0.0;
    
    return (x);
}


static void anlzRms_tilde_destructor(t_anlzRms_tilde *x)
{
    t_freebytes(x->signal, (WINDOW+x->n)*sizeof(t_sample));
    post("curveDraw destructor...");
}


static t_int *anlzRms_tilde_perform(t_int *w)
{
    int i, n;
    t_float dbVal;

    t_anlzRms_tilde *x = (t_anlzRms_tilde *)(w[1]);

    t_sample *in = (t_float *)(w[2]);
    n = w[3];
 			
 	// shift signal buffer contents back.
	for(i=0; i<WINDOW; i++)
		x->signal[i] = x->signal[i+n];
	
	// write new block to end of signal buffer.
	for(i=0; i<n; i++)
		x->signal[(int)WINDOW+i] = in[i];
		
	
	if(x->dspTicks++ >= 3)
	{
		dbVal = x->xthInst->anlzRms(x->signal, 512);
		outlet_float(x->x_output, dbVal);
		x->dspTicks = 0;
	}
		
    return (w+4);
}


static void anlzRms_tilde_dsp(t_anlzRms_tilde *x, t_signal **sp)
{
	int i;
	
	dsp_add(
		anlzRms_tilde_perform,
		3,
		x,
		sp[0]->s_vec,
		sp[0]->s_n
	); 
};


extern "C"
{
	void anlzRms_tilde_setup(void)
	{
		anlzRms_tilde_class = 
		class_new(
			gensym("anlzRms~"),
			(t_newmethod)anlzRms_tilde_new,
			(t_method)anlzRms_tilde_destructor,
			sizeof(t_anlzRms_tilde),
			CLASS_DEFAULT, 
			A_GIMME,
			0
		);

		CLASS_MAINSIGNALIN(anlzRms_tilde_class, t_anlzRms_tilde, x_f);
	
		class_addmethod(
			anlzRms_tilde_class,
			(t_method)anlzRms_tilde_dsp,
			gensym("dsp"),
			A_DEFFLOAT,
			0
		);
	}
}