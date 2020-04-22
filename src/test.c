#include "pso.h"

double rosenbrock(double* x, int dim) {

	double sum = 0;
	int i;
	for (i = 0; i < dim - 1; i++)
		sum += 100 * pow((x[i + 1] - pow(x[i], 2)), 2) + pow((1 - x[i]), 2);

	return sum;

}

double dropWave(double* x, int dim)
{
	const int dimMax = 2;
	double returnVal, nume, deno;
	if (dim != 2)
	{
		printf("Invalid number of dimensions. Recieved %i dimensions instead of %i!\n", dim, dimMax);
		returnVal = RETURN_ERROR;
	}
	else
	{
		nume = 1 + cos(12 * sqrt(pow(x[0], 2) + pow(x[1], 2)));
		deno = 0.5 * (pow(x[0], 2) + pow(x[1], 2)) + 2;
		returnVal = -(nume / deno);
	}
	return returnVal;
}


void main(void)
{
	//init sata structures & rand gen
	pso_context_t* context = NULL;
	pso_settings_t* settings = NULL;

	int dimensions = 2;
	double range_lower[5] = { -5, -5};
	double range_upper[5] = { 10, 10};
	
	//init
	pso_init(&context, &settings, PSO_MIN);

	//set pso parameters
	pso_settings_default(settings);
	pso_settings_set_solutionSpace(&settings, range_lower, range_upper, dimensions);
	pso_settings_set_goalFunc(context, rosenbrock);
	pso_settings_set_swarm(settings, 30);
	pso_settings_set_maxIterations(settings, 200);
	//pso_settings_set_minDeltaGBest(settings, 0.000001, 5);
	pso_settings_set_exportGenerations(settings, TRUE);
	pso_run(context, settings);

	for (int i = 0; i <= context->iter; i++)
	{
		pso_debug_coord(context, settings, i);
	}

	pso_uninit(context, settings);
}