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

1. Create a goalfunction of the following form

  double (\*func_ptr)(double\*, int)
  
  The first parameter takes an array of co-ordinates in the desired search space and the second take the dimensionality of the search space. The returned result is the function value at the provided co-ordinates.
