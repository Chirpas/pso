C Particle Swarm Optimization (PSO)
===

This is a modular implementation of the PSO algorithm with no external dependancies.

This library was developed due to a lack of flexibility in existing implementations.

# Features
- Generalised for variables with varying search spaces
- Stop criteria includes max iterations and minimal change in global best value

# Future Plans
- Other inertial weight schemes
- Elastic and inelastic boundary collisions (currently wont evaluate until particles return to search space)
- Communication between adjacent particles
- Performance optimisation

# Usage

1. Create a goalfunction of the following form:

    `double (*func_ptr)(double*, int)`
  
    The first parameter takes an array of co-ordinates in the desired search space and the second take the dimensionality of the search space. The returned result is the function value at the provided co-ordinates.
    
2. Initialise the PSO datastructures by passing a reference to pso_init along with the dimensionality of the search space and the optimisation mode:

    `void pso_init(pso_context_t** context, pso_settings_t** settings, int mode);`
    
    `i.e. pso_init(&context, &settings, PSO_MIN);`
    
3. Set the default PSO parameters or set your own.

    `void pso_settings_default(pso_settings_t* settings, int swarmSize, int maxIter);`
    
    Manually setting parameters can be done with the following functions. IF you do this without calling `pso_settings_default`, each  
    of these functions must be called! A safe way to go is to set default parameters, then modify.
    
    `void pso_settings_set_coefficients(pso_settings_t* settings, double _c1, double _c2);`
    `void pso_settings_set_inertia(pso_settings_t* settings, double _w_lo, double _w_up);`
    `void pso_settings_set_swarm(pso_settings_t* settings, int agents);`
    `void pso_settings_set_maxIterations(pso_settings_t* settings, int maxIter);`
    `void pso_settings_set_minDeltaGBest(pso_settings_t* settings, double dGbest, int n);`
    `void pso_settings_set_exportGenerations(pso_settings_t* settings, int export);`

4. Pass a reference to the goalfunction to be optimised

    `void pso_settings_goalFunc_set(pso_context_t* context, pso_objective_func func);`
    
5. Set the solution space boundaries

    `void pso_settings_set_solutionSpace(pso_settings_t** settings, double* range_lower, double* range_upper, int dimension);`

6. Run the algorithm:

    `pso_run(context, settings);`

7. Optimal result can be accessed through:
        
    `context->gBestCord`
    
8. Cleanup. Once finished, call `void pso_uninit(pso_context_t* context, pso_settings_t* settings)` to cleanup any memory allocated by    the algorithm.

# Integration
To integrate into your project, all thats needed is to copy pso.c and pso.h into your working directory. Alternatively, you can generate and link the static library.

At present, premake is used to do this and can be called with `scripts\Win-GenProjects.bat`. This is windows specific and generates a project file for VS2019. I have no plans on modifying this however its easily adjusted to your system and can generate for cmake or a makefile. For further details see https://premake.github.io/

# Example
A simple demo file has been included and will be compiled along with the static library mentioned above.

# Disclaimer
This algorithm has been implemented in such a way to integrate into one of my current projects and at present has been tested only in a limited capacity. I accept no responsibility for any actions it or its implementation may cause... except feedback. I love feedback!
