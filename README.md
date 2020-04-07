C Particle Swarm Optimization (PSO)
===

This is a modular implementation of the PSO algorithm with no external dependancies.

This library was developed due to a lack of flexibility in existing implementations.

# Features
- Generalised for variables with varying seach spaces

# Future Plans
- Other inertial weight schemes
- Elastic and inelastic boundary collisions (currently wont evaluate until particles return to search space)
- Additional setup functions for user customisability
- Improve stop criteria
- Communication between adjacent particles
- Performance optimisation

# Usage

1. Create a goalfunction of the following form:

    'double (\*func_ptr)(double\*, int)'
  
    The first parameter takes an array of co-ordinates in the desired search space and the second take the dimensionality of the search space. The returned result is the function value at the provided co-ordinates.
    
2. Initialise the PSO datastructures by passing a reference to pso_init along with the dimensionality of the search space and the optimisation mode:

    void pso_init(pso_context_t\*\* context, pso_settings_t\*\* settings, int dimension, int mode);
    
    i.e. pso_init(&context, &settings, 2, MIN);
    
3. Set the default PSO parameters or set your own: (manual setting is the next implementation)

    void pso_settings_default(pso_settings_t\* settings, int swarmSize, int maxIter);

4. Pass a reference to the goalfunction to be optimised

    void pso_settings_goalFunc_set(pso_context_t\* context, pso_objective_func func);

5. Run the algorithm:

    pso_run(context, settings);

6. Optimal result can be accessed through:
        
    context->gBestCord
