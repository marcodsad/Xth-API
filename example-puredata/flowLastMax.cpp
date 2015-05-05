
///////////////////////////////////////////////////////////////////////
//
//  flowLastMax
//
///////////////////////////////////////////////////////////////////////


#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>

static t_class *flowLastMax_class;

// Dataspace
typedef struct _flowLastMax {

    // object 
    t_object  x_obj;
    
    xth *xthInst;
    xthConfig config;
    xthDataspace dataspace;

} t_flowLastMax;




// BUILDING THE OBJECT
// ===================

static void *flowLastMax_new(t_symbol *s, int argc, t_atom *argv)
{
	t_flowLastMax *x = (t_flowLastMax *)pd_new(flowLastMax_class);

	x->xthInst = new xth(x->config);
	x->dataspace = x->xthInst->getDataspace();

    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("bang"), gensym("reset"));
	outlet_new(&x->x_obj, &s_float);
	
	return (void *)x;
}

static void flowLastMax_destructor(t_flowLastMax *x)
{
    post("flowLastMax destructor...");
}

static void flowLastMax_input(t_flowLastMax *x, t_float f)
{
	t_float output;

	output = x->xthInst->flowLastMax(f);
	
	outlet_float(x->x_obj.ob_outlet, output);
}

static void flowLastMax_reset(t_flowLastMax *x, t_floatarg dummy)
{
	x->dataspace.flowLastMax = FLT_MIN;
	x->xthInst->setDataspace(x->dataspace);
}

extern "C"
{
	void flowLastMax_setup(void) {

		flowLastMax_class = class_new(gensym("flowLastMax"),
			(t_newmethod)flowLastMax_new,
			(t_method)flowLastMax_destructor,
			sizeof(t_flowLastMax),
			CLASS_DEFAULT,
			A_GIMME,
			0
		);

		class_addfloat(
			flowLastMax_class,
			(t_method)flowLastMax_input
		);

		class_addmethod(
			flowLastMax_class, 
			(t_method)flowLastMax_reset,
			gensym("reset"),
			A_DEFFLOAT,
			0
		);
	}
}
