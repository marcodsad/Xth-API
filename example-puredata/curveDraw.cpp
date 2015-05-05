
///////////////////////////////////////////////////////////////////////
//
//  curveDraw
//
///////////////////////////////////////////////////////////////////////


#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>

static t_class *curveDraw_class;

// Dataspace
typedef struct _curveDraw {

    // object 
    t_object  x_obj;
    
    xth *xthInst;
    xthConfig config;

} t_curveDraw;




// BUILDING THE OBJECT
// ===================

static void *curveDraw_new(t_symbol *s, int argc, t_atom *argv)
{
	t_curveDraw *x = (t_curveDraw *)pd_new(curveDraw_class);

	x->xthInst = new xth(x->config);

	outlet_new(&x->x_obj, &s_float);

	return (void *)x;
}

static void curveDraw_destructor(t_curveDraw *x)
{
    post("curveDraw destructor...");
}

static void curveDraw_input(t_curveDraw *x, t_float f)
{
	t_float output;

	output = x->xthInst->curveDraw(f);
	
	outlet_float(x->x_obj.ob_outlet, output);
}


extern "C"
{
	void curveDraw_setup(void) {

		curveDraw_class = class_new(gensym("curveDraw"),
			(t_newmethod)curveDraw_new,
			(t_method)curveDraw_destructor,
			sizeof(t_curveDraw),
			CLASS_DEFAULT,
			A_GIMME,
			0
		);

		class_addfloat(
			curveDraw_class,
			(t_method)curveDraw_input
		);
	}
}
