
///////////////////////////////////////////////////////////////////////
//
//  smoothControlParams
//
///////////////////////////////////////////////////////////////////////


#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>

static t_class *smoothControlParams_class;

// Dataspace
typedef struct _smoothControlParams {

    // object 
    t_object  x_obj;
    
    xth *xthInst;
    xthConfig config;

} t_smoothControlParams;




// BUILDING THE OBJECT
// ===================

static void *smoothControlParams_new(t_symbol *s, int argc, t_atom *argv)
{
	t_smoothControlParams *x = (t_smoothControlParams *)pd_new(smoothControlParams_class);

	x->xthInst = new xth(x->config);

	outlet_new(&x->x_obj, &s_float);

	return (void *)x;
}

static void smoothControlParams_destructor(t_smoothControlParams *x)
{
    post("smoothControlParams destructor...");
}

static void smoothControlParams_input(t_smoothControlParams *x, t_float f)
{
	t_float output;

	output = x->xthInst->smoothControlParams(f);
	
	outlet_float(x->x_obj.ob_outlet, output);
}


extern "C"
{
	void smoothControlParams_setup(void) {

		smoothControlParams_class = class_new(gensym("smoothControlParams"),
			(t_newmethod)smoothControlParams_new,
			(t_method)smoothControlParams_destructor,
			sizeof(t_smoothControlParams),
			CLASS_DEFAULT,
			A_GIMME,
			0
		);

		class_addfloat(
			smoothControlParams_class,
			(t_method)smoothControlParams_input
		);
	}
}
