
///////////////////////////////////////////////////////////////////////
//
//  flowLastMin
//
///////////////////////////////////////////////////////////////////////


#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>

static t_class *flowLastMin_class;

// Dataspace
typedef struct _flowLastMin {

    // object 
    t_object  x_obj;
    
    xth *xthInst;
    xthConfig config;
    xthDataspace dataspace;

} t_flowLastMin;




// BUILDING THE OBJECT
// ===================

static void *flowLastMin_new(t_symbol *s, int argc, t_atom *argv)
{
	t_flowLastMin *x = (t_flowLastMin *)pd_new(flowLastMin_class);

	x->xthInst = new xth(x->config);
	x->dataspace = x->xthInst->getDataspace();

    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("bang"), gensym("reset"));
	outlet_new(&x->x_obj, &s_float);

	return (void *)x;
}

static void flowLastMin_destructor(t_flowLastMin *x)
{
    post("flowLastMin destructor...");
}

static void flowLastMin_input(t_flowLastMin *x, t_float f)
{
	t_float output;

	output = x->xthInst->flowLastMin(f);
	
	outlet_float(x->x_obj.ob_outlet, output);
}

static void flowLastMin_reset(t_flowLastMin *x, t_floatarg dummy)
{
	x->dataspace.flowLastMin = FLT_MAX;
	x->xthInst->setDataspace(x->dataspace);
}

extern "C"
{
	void flowLastMin_setup(void) {

		flowLastMin_class = class_new(gensym("flowLastMin"),
			(t_newmethod)flowLastMin_new,
			(t_method)flowLastMin_destructor,
			sizeof(t_flowLastMin),
			CLASS_DEFAULT,
			A_GIMME,
			0
		);

		class_addfloat(
			flowLastMin_class,
			(t_method)flowLastMin_input
		);

		class_addmethod(
			flowLastMin_class, 
			(t_method)flowLastMin_reset,
			gensym("reset"),
			A_DEFFLOAT,
			0
		);
	}
}
