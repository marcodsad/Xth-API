#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>
#define WINDOW 441 // based on roughly 10ms snapshot rate

static t_class *anlzSnap_tilde_class;

typedef struct _anlzSnap_tilde
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

} t_anlzSnap_tilde;


/* ------------------------ anlzSnap~ -------------------------------- */

static void *anlzSnap_tilde_new(t_symbol *s, int argc, t_atom *argv)
{
    t_anlzSnap_tilde *x = (t_anlzSnap_tilde *)pd_new(anlzSnap_tilde_class);
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


static void anlzSnap_tilde_destructor(t_anlzSnap_tilde *x)
{
    t_freebytes(x->signal, (WINDOW+x->n)*sizeof(t_sample));
    post("curveDraw destructor...");
}


static t_int *anlzSnap_tilde_perform(t_int *w)
{
    int i, n;
    t_float val;

    t_anlzSnap_tilde *x = (t_anlzSnap_tilde *)(w[1]);

    t_sample *in = (t_float *)(w[2]);
    n = w[3];
 			
 	// shift signal buffer contents back.
	for(i=0; i<WINDOW; i++)
		x->signal[i] = x->signal[i+n];
	
	// write new block to end of signal buffer.
	for(i=0; i<n; i++)
		x->signal[(int)WINDOW+i] = in[i];
		
	
	// 7 ticks for 441 samples = ~10ms
	if(x->dspTicks++ >= 6)
	{
		val = x->xthInst->anlzSnap(x->signal, 441);
		outlet_float(x->x_output, val);
		x->dspTicks = 0;
	}
		
    return (w+4);
}


static void anlzSnap_tilde_dsp(t_anlzSnap_tilde *x, t_signal **sp)
{
	int i;
	
	dsp_add(
		anlzSnap_tilde_perform,
		3,
		x,
		sp[0]->s_vec,
		sp[0]->s_n
	); 
};


extern "C"
{
	void anlzSnap_tilde_setup(void)
	{
		anlzSnap_tilde_class = 
		class_new(
			gensym("anlzSnap~"),
			(t_newmethod)anlzSnap_tilde_new,
			(t_method)anlzSnap_tilde_destructor,
			sizeof(t_anlzSnap_tilde),
			CLASS_DEFAULT, 
			A_GIMME,
			0
		);

		CLASS_MAINSIGNALIN(anlzSnap_tilde_class, t_anlzSnap_tilde, x_f);
	
		class_addmethod(
			anlzSnap_tilde_class,
			(t_method)anlzSnap_tilde_dsp,
			gensym("dsp"),
			A_DEFFLOAT,
			0
		);
	}
}