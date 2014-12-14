// Function for the dynamics of the problem
// dy/dt = dynamics(y,u,z,p)

// The following are the input and output available variables 
// for the dynamics of your optimal control problem.

// Input :
// time : current time (t)
// normalized_time: t renormalized in [0,1]
// initial_time : time value on the first discretization point
// final_time : time value on the last discretization point
// dim_* is the dimension of next vector in the declaration
// state : vector of state variables
// control : vector of control variables
// algebraicvars : vector of algebraic variables
// optimvars : vector of optimization parameters
// constants : vector of constants

// Output :
// state_dynamics : vector giving the expression of the dynamic of each state variable.

// The functions of your problem have to be written in C++ code
// Remember that the vectors numbering in C++ starts from 0
// (ex: the first component of the vector state is state[0])

// Tdouble variables correspond to values that can change during optimization:
// states, controls, algebraic variables and optimization parameters.
// Values that remain constant during optimization use standard types (double, int, ...).

#include "header_dynamics"
{
	// HERE : description of the function for the dynamics
	// Please give a function or a value for the dynamics of each state variable
	Tdouble p 	= state[0];
	Tdouble q 	= state[1];
	Tdouble r 	= state[2];
	
	Tdouble u1 	= control[0];
	Tdouble u2 	= control[1];
	Tdouble u3 	= control[2];
	 	
	double lambda 	= constants[0];
  	double mu	= constants[1];
	double a1	= constants[2];
	double a2	= constants[3];
	double a3	= constants[4];
	double eps	= constants[5];
	

	state_dynamics[0] = (mu - lambda)*q*r + a1*u1;
	state_dynamics[1] = ((lambda - 1)*r*p + a2*u2)/mu;
	state_dynamics[2] = ((1-mu)*p*q+a3*u3)/lambda;
	state_dynamics[3] = abs(u1)+abs(u2)+abs(u3)+eps*(u1*u1+u2*u2+u3*u3);
}


