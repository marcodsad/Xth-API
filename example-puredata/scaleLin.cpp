
///////////////////////////////////////////////////////////////////////
//
//  scaleLin
//
///////////////////////////////////////////////////////////////////////


#include "m_pd.h"
#include "xth.h"
#include <iostream>
#include <math.h>
#include <float.h>

static t_class *scaleLin_class;

// Dataspace
typedef struct _scaleLin {

    // object 
    t_object  x_obj;

    xth *xthInst;
    xthConfig config;
    xthDataspace dataspace;
    
    t_float x_top;
    t_float x_bottom;
    t_float x_topInput;

} t_scaleLin;




// BUILDING THE OBJECT
// ===================

static void *scaleLin_new(t_symbol *s, int argc, t_atom *argv)
{
	t_scaleLin *x = (t_scaleLin *)pd_new(scaleLin_class);
	int i;
	
	x->xthInst = new xth(x->config);
	x->dataspace = x->xthInst->getDataspace();

	switch(argc) {
		case 0:
			x->x_top = 127;
			x->x_bottom = 0;
			x->x_topInput = 400;
			post("case zero");
			break;		
		case 1:
			x->x_top = atom_getfloat(argv);
			x->x_bottom = 0;
			x->x_topInput = 400;
			post("case one");
			break;
		case 2:
			x->x_top = atom_getfloat(argv);
			x->x_bottom = atom_getfloat(argv+1);
			x->x_topInput = 400;
			post("case two");
			break;
		case 3:
			x->x_top = atom_getfloat(argv);
			x->x_bottom = atom_getfloat(argv+1);
			x->x_topInput = atom_getfloat(argv+2);
			post("case three");
			break;
		default:
			break;
			post("case default");
	};
				
	post("top: %f, bottom: %f, topInput: %f", x->x_top, x->x_bottom, x->x_topInput);
		
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("setTop"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("setBottom"));
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("setTopInput"));

	outlet_new(&x->x_obj, &s_float);

	return (void *)x;
}

static void scaleLin_destructor(t_scaleLin *x)
{
    post("scaleLin destructor...");
}

static void scaleLin_input(t_scaleLin *x, t_float f)
{
	t_float output;

	output = x->xthInst->scaleLin(f, x->x_top, x->x_bottom, x->x_topInput);
	
	outlet_float(x->x_obj.ob_outlet, output);

}

static void scaleLin_setTop(t_scaleLin *x, t_floatarg f)
{
	x->x_top = f;
}

static void scaleLin_setBottom(t_scaleLin *x, t_floatarg f)
{
	x->x_bottom = f;
}

static void scaleLin_setTopInput(t_scaleLin *x, t_floatarg f)
{
	x->x_topInput = f;
}

extern "C"
{
	void scaleLin_setup(void) {

		scaleLin_class = class_new(gensym("scaleLin"),
			(t_newmethod)scaleLin_new,
			(t_method)scaleLin_destructor,
			sizeof(t_scaleLin),
			CLASS_DEFAULT,
			A_GIMME,
			0
		);

		class_addfloat(
			scaleLin_class,
			(t_method)scaleLin_input
		);

		class_addmethod(
			scaleLin_class, 
			(t_method)scaleLin_setTop,
			gensym("setTop"),
			A_DEFFLOAT,
			0
		);
		
		class_addmethod(
			scaleLin_class, 
			(t_method)scaleLin_setBottom,
			gensym("setBottom"),
			A_DEFFLOAT,
			0
		);
		
		class_addmethod(
			scaleLin_class, 
			(t_method)scaleLin_setTopInput,
			gensym("setTopInput"),
			A_DEFFLOAT,
			0
		);
	}
}
