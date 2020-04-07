#ifndef _PSO_H_
#define _PSO_H_

#define TRUE 1
#define FALSE !TRUE

#define MIN 0
#define MAX !MIN

#define DEBUG 0
#define RETURN_ERROR -99999999

//default pso parameters
#define DEFAULT_MAX_ITER 100
#define DEFAULT_SWARM_SIZE 30
#define DEFAULT_C1 1.496
#define DEFAULT_C2 1.496
#define DEFAULT_W_MIN 0.3
#define DEFAULT_W_MAX 0.7298

#include <stdlib.h> // for rand()
#include <stdio.h> // for printf
#include <float.h> //DOUBLE_MAX
#include <time.h> // for time()
#include <math.h> // for cos(), pow(), sqrt() etc.
#include <string.h> //memcpy

/*function pointer to objective function. cordinates and dimension size*/
typedef double (*pso_objective_func)(double*, int);

/* Holds global information for algorithm
*/
typedef struct pso_settings_t
{
	int opt_mode;
	int iter_max;

	int dim;
	int size;
	double* x_lo, * x_up;
	double c1;
	double c2;
	double w_min;
	double w_max;
}pso_settings_t;


typedef struct pso_cordinate_t
{
	double* x; //instantaneous position
	double* v; //instantaneous velocity
}pso_cordinate_t;

/*holds the parameters of a single particle*/
typedef struct pso_particle_t
{
	pso_cordinate_t cord;
	double* pbestCord;
	double pbest;
}pso_particle_t;

/* typedef struct pso_context_t
	Holds the swarm state and information
*/
typedef struct pso_context_t
{
	int iter;
	double* gbestCord;
	double gbest;
	double weight;
	pso_particle_t* particle;
	pso_objective_func obj_func;
}pso_context_t;

void pso_settings_weighting_adjust(pso_context_t* context, pso_settings_t* settings);

void pso_swarm_generate(pso_context_t* context, pso_settings_t* settings);
void pso_swarm_destroy(pso_context_t* context, pso_settings_t* settings);
void pso_swarm_evaluate(pso_context_t* context, pso_settings_t* settings);
void pso_swarm_update(pso_context_t* context, pso_settings_t* settings);
int pso_particle_compare(pso_settings_t* settings, double result, double cBest);
int pso_stop(pso_context_t* context, pso_settings_t* settings);
void pso_debug_2d(pso_context_t* context, pso_settings_t* settings);

void deepCopy(double** dst, double** src, int size_t);


/*user functions*/
void pso_run(pso_context_t* context, pso_settings_t* settings);
void pso_init(pso_context_t** context, pso_settings_t** settings, int mode);
void pso_uninit(pso_context_t* context, pso_settings_t* settings);

/*setup funcs*/
void pso_settings_set_goalFunc(pso_context_t* context, pso_objective_func func);
void pso_settings_set_solutionSpace(pso_settings_t** settings, double* range_lower, double* range_upper, int dimension);

/*alter default settings*/
void pso_settings_default(pso_settings_t* settings);

void pso_settings_set_coefficients(pso_settings_t* settings, double _c1, double _c2);
void pso_settings_set_inertia(pso_settings_t* settings, double _w_lo, double _w_up);
void pso_settings_set_swarm(pso_settings_t* settings, int agents);




#endif // PSO_H_
