
///////////////////////////////////////////////////////////////////////
//
//  flowBetween
//
///////////////////////////////////////////////////////////////////////


#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>
#include <float.h>

static t_class *flowBetween_class;

// Dataspace
typedef struct _flowBetween {

    // object 
    t_object  x_obj;

    xth *xthInst;
    xthConfig config;
    xthDataspace dataspace;
    
    t_float x_loLim;
    t_float x_hiLim;
    
	t_outlet *x_inBounds;
	t_outlet *x_outBounds;

} t_flowBetween;




// BUILDING THE OBJECT
// ===================

static void *flowBetween_new(t_symbol *s, int argc, t_atom *argv)
{
	t_flowBetween *x = (t_flowBetween *)pd_new(flowBetween_class);

	x->xthInst = new xth(x->config);
	x->dataspace = x->xthInst->getDataspace();

	switch(argc) {
		case 0:
			x->x_loLim = x->dataspace.flowBetweenBounds[0];
			x->x_hiLim = x->dataspace.flowBetweenBounds[1];
			post("case zero");
			break;		
		case 1:
			x->x_loLim = atom_getfloat(argv);
			x->x_hiLim = x->dataspace.flowBetweenBounds[1];
			x->dataspace.flowBetweenBounds[0] = x->x_loLim;
			x->xthInst->setDataspace(x->dataspace);
			post("case one");
			break;
		case 2:
			x->x_loLim = atom_getfloat(argv);
			x->x_hiLim = atom_getfloat(argv+1);
			x->dataspace.flowBetweenBounds[0] = x->x_loLim;
			x->dataspace.flowBetweenBounds[1] = x->x_hiLim;
			x->xthInst->setDataspace(x->dataspace);
			post("case two");
			break;
		default:
			break;
			post("case default");
	};
	
	post("loLim: %f, hiLim: %f", x->x_loLim, x->x_hiLim);
	
	// need functions for these inlets. they shouldn't be passive
	
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("setLoLim"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("setHiLim"));

	x->x_inBounds = outlet_new(&x->x_obj, &s_float);
	x->x_outBounds = outlet_new(&x->x_obj, &s_float);
	
	return (void *)x;
}

static void flowBetween_destructor(t_flowBetween *x)
{
    post("flowBetween destructor...");
}

static void flowBetween_input(t_flowBetween *x, t_float f)
{
	t_float output;

	output = x->xthInst->flowBetween(f);
	
	if(output<FLT_MAX)
		outlet_float(x->x_inBounds, output);
	else
		outlet_float(x->x_outBounds, f);
}

static void flowBetween_setLoLim(t_flowBetween *x, t_floatarg loLim)
{
	x->x_loLim = loLim;
	x->dataspace.flowBetweenBounds[0] = x->x_loLim;
	x->xthInst->setDataspace(x->dataspace);
}


static void flowBetween_setHiLim(t_flowBetween *x, t_floatarg hiLim)
{
	x->x_hiLim = hiLim;
	x->dataspace.flowBetweenBounds[1] = x->x_hiLim;
	x->xthInst->setDataspace(x->dataspace);
}

static void flowBetween_printXth(t_flowBetween *x, t_floatarg dummy)
{
	x->dataspace = x->xthInst->getDataspace();
	post("loLim: %f\nhiLim: %f", x->dataspace.flowBetweenBounds[0], x->dataspace.flowBetweenBounds[1]);
}

extern "C"
{
	void flowBetween_setup(void) {

		flowBetween_class = class_new(gensym("flowBetween"),
			(t_newmethod)flowBetween_new,
			(t_method)flowBetween_destructor,
			sizeof(t_flowBetween),
			CLASS_DEFAULT,
			A_GIMME,
			0
		);

		class_addfloat(
			flowBetween_class,
			(t_method)flowBetween_input
		);

		class_addmethod(
			flowBetween_class, 
			(t_method)flowBetween_setLoLim,
			gensym("setLoLim"),
			A_DEFFLOAT,
			0
		);
		
		class_addmethod(
			flowBetween_class, 
			(t_method)flowBetween_setHiLim,
			gensym("setHiLim"),
			A_DEFFLOAT,
			0
		);

		class_addmethod(
			flowBetween_class, 
			(t_method)flowBetween_printXth,
			gensym("printXth"),
			A_DEFFLOAT,
			0
		);
	}
}
