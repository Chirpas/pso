#include "pso.h"

double rosenbrock(double* vec, int dim) {

	double sum = 0;
	int i;
	for (i = 0; i < dim - 1; i++)
		sum += 100 * pow((vec[i + 1] - pow(vec[i], 2)), 2) + pow((1 - vec[i]), 2);

	return sum;

}


void main(void)
{
	//init sata structures & rand gen
	pso_context_t* context = NULL;
	pso_settings_t* settings = NULL;

	int dimensions = 2;
	int agents = 5;
	
	//init
	pso_init(&context, &settings, 2, MIN);

	//set pso parameters
	pso_settings_default(settings, agents, 20);
	pso_settings_goalFunc_set(context, rosenbrock);
	pso_run(context, settings);

	printf("\ngBest : %.2f, %.2f, gVal: %f \n", context->gbestCord[0], context->gbestCord[1], context->gbest);
}