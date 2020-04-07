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

    `double (\*func_ptr)(double\*, int)`
  
    The first parameter takes an array of co-ordinates in the desired search space and the second take the dimensionality of the search space. The returned result is the function value at the provided co-ordinates.
    
2. Initialise the PSO datastructures by passing a reference to pso_init along with the dimensionality of the search space and the optimisation mode:

    `void pso_init(pso_context_t\*\* context, pso_settings_t\*\* settings, int dimension, int mode);`
    
    `i.e. pso_init(&context, &settings, 2, MIN);`
    
3. Set the default PSO parameters or set your own: (manual setting is the next implementation)

    `void pso_settings_default(pso_settings_t\* settings, int swarmSize, int maxIter);`

4. Pass a reference to the goalfunction to be optimised

    `void pso_settings_goalFunc_set(pso_context_t\* context, pso_objective_func func);`

5. Run the algorithm:

    `pso_run(context, settings);`

6. Optimal result can be accessed through:
        
    `context->gBestCord`

#Integration
To integrate into your project, all thats needed is to copy pso.c and pso.h into your working directory. Alternatively, you can generate and link the static library.

At present, premake is used to do this and can be called with `scripts\Win-GenProjects.bat`. This is windows specific and generates a project file for VS2019. I have no plans on modifying this however its easily adjusted to your system and can generate for cmake or a makefile. For further details see https://premake.github.io/

#Example
A simple demo file has been included and will be compiled along with the static library mentioned above.

#Disclaimer
This algorithm has been implemented in such a way to integrate into one of my current projects and at present has been tested only in a limited capacity. I accept no responsibility for any actions it or its implementation may cause... except feedback. I love feedback!
