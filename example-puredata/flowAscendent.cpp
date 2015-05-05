
///////////////////////////////////////////////////////////////////////
//
//  flowAscendent
//
///////////////////////////////////////////////////////////////////////


#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>

static t_class *flowAscendent_class;

// Dataspace
typedef struct _flowAscendent {

    // object 
    t_object  x_obj;
    
    xth *xthInst;
    xthConfig config;

} t_flowAscendent;




// BUILDING THE OBJECT
// ===================

static void *flowAscendent_new(t_symbol *s, int argc, t_atom *argv)
{
	t_flowAscendent *x = (t_flowAscendent *)pd_new(flowAscendent_class);

	x->xthInst = new xth(x->config);

	outlet_new(&x->x_obj, &s_float);

	return (void *)x;
}

static void flowAscendent_destructor(t_flowAscendent *x)
{
    post("flowAscendent destructor...");
}

static void flowAscendent_input(t_flowAscendent *x, t_float f)
{
	t_float output;

	output = x->xthInst->flowAscendent(f);
	
	if(output)
		outlet_float(x->x_obj.ob_outlet, output);
}


extern "C"
{
	void flowAscendent_setup(void) {

		flowAscendent_class = class_new(gensym("flowAscendent"),
			(t_newmethod)flowAscendent_new,
			(t_method)flowAscendent_destructor,
			sizeof(t_flowAscendent),
			CLASS_DEFAULT,
			A_GIMME,
			0
		);

		class_addfloat(
			flowAscendent_class,
			(t_method)flowAscendent_input
		);
	}
}
