
///////////////////////////////////////////////////////////////////////
//
//  flowAverage
//
///////////////////////////////////////////////////////////////////////


#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>

static t_class *flowAverage_class;

// Dataspace
typedef struct _flowAverage {

    // object 
    t_object  x_obj;
    
    xth *xthInst;
    xthConfig config;

} t_flowAverage;




// BUILDING THE OBJECT
// ===================

static void *flowAverage_new(t_symbol *s, int argc, t_atom *argv)
{
	t_flowAverage *x = (t_flowAverage *)pd_new(flowAverage_class);

	outlet_new(&x->x_obj, &s_float);

	x->xthInst = new xth(x->config);

	return (void *)x;
}

static void flowAverage_destructor(t_flowAverage *x)
{
    post("flowAverage destructor...");
}

static void flowAverage_input(t_flowAverage *x, t_float f)
{
	t_float output;

	output = x->xthInst->flowAverage(f);
	
	outlet_float(x->x_obj.ob_outlet, output);
}


extern "C"
{
	void flowAverage_setup(void) {

		flowAverage_class = class_new(gensym("flowAverage"),
			(t_newmethod)flowAverage_new,
			(t_method)flowAverage_destructor,
			sizeof(t_flowAverage),
			CLASS_DEFAULT,
			A_GIMME,
			0
		);

		class_addfloat(
			flowAverage_class,
			(t_method)flowAverage_input
		);
	}
}
