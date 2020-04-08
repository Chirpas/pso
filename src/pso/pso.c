#include "pso.h"

/*double* pso_run(pso_context_t* context, pso_settings_t* settings);
	Function call that runs the optimisation algorithm.

	returns a double* array containing the optimal values for each dimension of the specified
	number of dimensions.
	*/
void pso_run(pso_context_t* context, pso_settings_t* settings)
{
	if (context == NULL)
	{
		printf("Error: 'context' not initialized. Have you called pso_init?\n");
	}
	else if (settings == NULL)
	{
		printf("Error: 'settings' not initialized. Have you called pso_init?\n");
	}
	else if (context->obj_func == NULL)
	{
		printf("Error: 'obj_func' is NULL. Have you called pso_settings_goalFunc_set?\n");
	}
	else
	{
		pso_swarm_generate(context, settings);
		pso_debug_2d(context, settings);

		//main pso loop
		do
		{
			//iterate to next generation
			context->iter++;
			pso_swarm_update(context, settings);
			pso_swarm_evaluate(context, settings);
			pso_debug_2d(context, settings);
		} while (!pso_stop(context, settings));

		//deref memory allocated during run
		pso_swarm_destroy(context, settings);
	}
}

/* void pso_swarm_destroy(pso_context_t* context, pso_settings_t* settings);
	Dereferences all allocated memory for particles during the PSO runtime apart from gbest!. 
*/
void pso_swarm_destroy(pso_context_t* context, pso_settings_t* settings)
{
	for (int i = 0; i < settings->size; i++)
	{
		if (context->particle != NULL)
		{
			free(context->particle[i].cord.x);
			free(context->particle[i].cord.v);
			free(context->particle[i].pbestCord);
		}
	}
	free(context->particle);
}

/* int pso_stop(pso_context_t* context, pso_settings_t* settings)
	Checks if algorithm stop condition has been met.

	returns TRUE (1) if stop condition met. Otherwise returns FALSE (0).
*/
int pso_stop(pso_context_t* context, pso_settings_t* settings)
{
	if (context->iter >= settings->iter_max)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*void pso_init(pso_context_t context, pso_settings_t settings)
	Initialise datastructures & runtime functions
	int mode - optimisation mode -> minimization (MIN) or maximization (MAX)
*/
void pso_init(pso_context_t** context, pso_settings_t** settings, int mode)
{
	*settings = (pso_settings_t*)malloc(sizeof(pso_settings_t));;
	*context = (pso_context_t*)malloc(sizeof(pso_context_t));
	(*context)->iter = 0;
	(*settings)->opt_mode = mode;

	//set initial g
	if(mode == PSO_MIN)
		(*context)->gbest = DBL_MAX;
	else
		(*context)->gbest = 0;

	srand((unsigned int)time(NULL));
}

/* void pso_uninit(pso_context_t* context, pso_settings_t* settings);
	Free all memory allocated by the algorithm
*/
void pso_uninit(pso_context_t* context, pso_settings_t* settings)
{
	free(settings->x_lo);
	free(settings->x_up);
	free(context->gbestCord);
	free(settings);
	free(context);
}

/*void pso_settings_default(pso_settings_t* settings)
	Sets the default settings for the pso algorithm

	pso_settings_t* settings	- Holds all information the algorithm requires
*/
void pso_settings_default(pso_settings_t* settings)
{
	if (settings == NULL)
	{
		printf("Settings = NULL\n");
	}
	else
	{
		//set default values
		settings->c1 = DEFAULT_C1;
		settings->c2 = DEFAULT_C2;
		settings->w_min = DEFAULT_W_MIN;
		settings->w_max = DEFAULT_W_MAX;
		settings->iter_max = DEFAULT_MAX_ITER;
		settings->size = DEFAULT_SWARM_SIZE;
	}
}

/* void pso_settings_solutionSpace_set(pso_settings_t* settings, double* range_lower, double* range_upper, int dimension)
	Sets the search space that the swarm with move over evaluating the goal function. Also sets the dimensionality of the search space
*/
void pso_settings_set_solutionSpace(pso_settings_t** settings, double* range_lower, double* range_upper, int dimension)
{
	if (settings == NULL)
	{
		printf("pso data structures not initialised!\n");
		return;
	}

	(*settings)->dim = dimension;
	(*settings)->x_lo = (double*)malloc(dimension * sizeof(double));
	(*settings)->x_up = (double*)malloc(dimension * sizeof(double));

	for (int i = 0; i < (*settings)->dim; i++)
	{
		(*settings)->x_lo[i] = range_lower[i];
		(*settings)->x_up[i] = range_upper[i];
	}
}

/* void pso_settings_weighting_adjust(pso_context_t* context, pso_settings_t* settings);
	Adjust the weighting coefficient depending on the current iteration. At present, this is only a simple adjustment

	int stage - Iteration at which the weighting reaches its minimum.
*/
void pso_settings_weighting_adjust(pso_context_t* context, pso_settings_t* settings)
{
	int stage = 3 * (context->iter) / 4;
	if (context->iter <= stage)
	{
		context->weight = settings->w_min + (settings->w_max - settings->w_min) * (double)((stage - context->iter) / stage);
	}
	else
	{
		context->weight = settings->w_min;
	}
}


/*void pso_settings_goalFunc_set(pso_context_t* context, pso_objective_func func)
	Passes the address of the goalfunction to be evaluated. Assumes that the object function is of the form
	typedef double (*pso_objective_func)(double*, int);
*/
void pso_settings_set_goalFunc(pso_context_t* context, pso_objective_func func)
{
	context->obj_func = func;
}


/*
	Generates and initialises the initial position of the swarm particles and stores in the context
*/
void pso_swarm_generate(pso_context_t* context, pso_settings_t* settings)
{
	//allocate memory for each particle & set initial random variable
	printf("swarm size: %d\n", settings->size);

	context->particle = (pso_particle_t*)malloc((settings->size)*sizeof(pso_particle_t));
	context->gbestCord = (double*)malloc(settings->dim * sizeof(double));
	double a, b;
	
	for (int i = 0; i < settings->size; i++)
	{
		context->particle[i].cord.x = (double*)malloc((settings->dim) * sizeof(double));
		context->particle[i].cord.v = (double*)malloc((settings->dim) * sizeof(double));
		context->particle[i].pbestCord = (double*)malloc((settings->dim) * sizeof(double));

		//allocate an initial x pos and velocity based on specified cordinate space & set personal best
		for (int j = 0; j < settings->dim; j++)
		{
			a = (rand() % (int)(settings->x_up[j] - settings->x_lo[j] + 1)) + settings->x_lo[j];
			b = (rand() % (int)(settings->x_up[j] - settings->x_lo[j] + 1)) + settings->x_lo[j];
			context->particle[i].cord.x[j] = a;
			context->particle[i].cord.v[j] = (a - b) / 2.0f;
		}
		deepCopy(&(context->particle[i].pbestCord), &(context->particle[i].cord.x), settings->dim);

		if (settings->opt_mode == PSO_MIN)
		{
			context->particle[i].pbest = DBL_MAX;
			context->gbest = DBL_MAX;
		}
		else
		{
			context->particle[i].pbest = 0;
			context->gbest = 0;
		}
	}
	//find gbest and pbest for initial particles
	pso_swarm_evaluate(context, settings);
}


/*void pso_swarm_evaluate(pso_context_t* context, pso_settings_t* settings)
	Evaluates the fitness function of each particle in the swarm and updates the global best
	if one of the particles has a better solution.

	double result - temp variable used to hold the result of the fitness function evaluation
*/
void pso_swarm_evaluate(pso_context_t* context, pso_settings_t* settings)
{
	double result;
	//for each particle in the swarm
	for (int i = 0; i < settings->size; i++)
	{
		/* at present, the only scheme implemented is that particles outside the box wont have their fitness function evaluated. 
		As their motion changes, bringing them closer to the global best and back in the box, they will be re-evaluated. Other possible implementations
		include an elastic and inelastic barrier.*/

		int inside = TRUE;
		//if particle leaves confines of box, ignore it.
		for (int j = 0; j < settings->dim; j++)
		{
			if ((context->particle[i].cord.x[j] > settings->x_up[j]) || context->particle[i].cord.x[j] < settings->x_lo[j])
			{
				inside = FALSE;
				break;
			}
		}
		if (inside == TRUE)
		{
			//evaluate fitness function
			result = context->obj_func(context->particle[i].cord.x, settings->dim);
			//if better than current personal best, update
			if (pso_particle_compare(settings, result, context->particle[i].pbest))
			{
				deepCopy(&(context->particle[i].pbestCord), &(context->particle[i].cord.x), settings->dim);
				context->particle[i].pbest = result;
				//if better than gbest, update gbest
				if (pso_particle_compare(settings, result, context->gbest))
				{
					deepCopy(&(context->gbestCord), &(context->particle[i].cord.x), settings->dim);
					context->gbest = result;
				}
			}
		}
	}
}

/*int pso_particle_compare(pso_settings_t* settings, double result, double cBest)
	Compares a resulting fitness function with the current best under the set optimisation scheme.

	returns TRUE(1) if result is better than cBest. If not, returns FALSE(0)
*/
int pso_particle_compare(pso_settings_t* settings, double result, double cBest)
{
	if (settings->opt_mode == PSO_MIN && result < cBest)
		return TRUE;
	else if (settings->opt_mode == PSO_MAX && result > cBest)
		return TRUE;
	else
		return FALSE;
}

/*void pso_swarm_update(pso_context_t* context, pso_settings_t* settings)
	Updates the position of each particle and increments the generation

	double c1_r, c2_r
*/
void pso_swarm_update(pso_context_t* context, pso_settings_t* settings)
{
	double c1_r, c2_r;
	//update inertia weighting
	pso_settings_weighting_adjust(context, settings);

	//loop over all particles
	for (int i = 0; i < settings->size; i++)
	{
		//for each dimension of each particle
		for (int j = 0; j < settings->dim; j++)
		{
			//calculate exploratory coefficients
			c1_r = settings->c1*(rand() / (double)RAND_MAX);
			c2_r = settings->c2*(rand() / (double)RAND_MAX);

			//update individual dimension velocity, then position
			context->particle[i].cord.v[j] = context->weight * context->particle[i].cord.v[j]
				+ c1_r * (context->particle[i].pbestCord[j] - context->particle[i].cord.x[j])
				+ c2_r * (context->gbestCord[j] - context->particle[i].cord.x[j]);
			context->particle[i].cord.x[j] += context->particle[i].cord.v[j];

			//can implement other boundary conditions here. For now, going to take the approach of ignorinf the particle
			//if it leaves the region. only evaluate fittness when it returns,
		}
	}
}

/* void pso_settings_set_coefficients(pso_settings_t* settings, double _c1, double _c2);
	Sets the exploratory coefficients
	double _c1 - affinity to exploit global
	double _c2 - affinity to exploit local
*/
void pso_settings_set_coefficients(pso_settings_t* settings, double _c1, double _c2)
{
	settings->c1 = _c1;
	settings->c2 = _c2;
}

/*void pso_settings_set_inertia(pso_settings_t* settings, double _w_lo, double _w_up)
	Sets the upper and lower inertial weight boundaries
*/
void pso_settings_set_inertia(pso_settings_t* settings, double _w_lo, double _w_up)
{
	settings->w_min = _w_lo;
	settings->w_max = _w_up;
}

/*void pso_settings_set_swarm(pso_settings_t* settings, int agents)
	Sets the number of agents in the swarm
*/
void pso_settings_set_swarm(pso_settings_t* settings, int agents)
{
	settings->size = agents;
}

/*void pso_debug_2d(pso_context_t* context, pso_settings_t* settings)
	debug function used to print the first two dimensions of each particle in the swarm
*/

void pso_debug_2d(pso_context_t* context, pso_settings_t* settings)
{
	if (PSO_DEBUG == 1)
	{
		printf("printing particles; iteration: %d\n", context->iter);
		for (int i = 0; i < settings->size; i++)
		{
			printf("particle: %d\n", i);
			printf("x : %.2f, %.2f \n", context->particle[i].cord.x[0], context->particle[i].cord.x[1]);
			printf("v : %.2f, %.2f \n", context->particle[i].cord.v[0], context->particle[i].cord.v[1]);
			printf("px: %.2f, %.2f, pVal: %.2f \n\n", context->particle[i].pbestCord[0], context->particle[i].pbestCord[1], context->particle[i].pbest);
		}
	}
}


	/*void deepCopy(double** dst, double** src, int size_t)
	Takes the address of a source and destination double* and performs a deep copy on the source information
*/
void deepCopy(double** dst, double** src, int size_t)
{
	for (int i = 0; i < size_t; i++)
	{
		(*dst)[i] = (*src)[i];
	}
}
