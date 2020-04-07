#ifndef _PSO_H_
#define _PSO_H_

#define TRUE 1
#define FALSE !TRUE

#define MIN 0
#define MAX !MIN

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
void pso_swarm_evaluate(pso_context_t* context, pso_settings_t* settings);
void pso_swarm_update(pso_context_t* context, pso_settings_t* settings);
int pso_particle_compare(pso_settings_t* settings, double result, double cBest);
int pso_stop(pso_context_t* context, pso_settings_t* settings);
void pso_debug_2d(pso_context_t* context, pso_settings_t* settings);

void deepCopy(double** dst, double** src, int size_t);


/*user functions*/
void pso_run(pso_context_t* context, pso_settings_t* settings);
void pso_init(pso_context_t** context, pso_settings_t** settings, int dimension, int mode);
void pso_settings_default(pso_settings_t* settings, int swarmSize, int maxIter);
void pso_settings_goalFunc_set(pso_context_t* context, pso_objective_func func);

#endif // PSO_H_
