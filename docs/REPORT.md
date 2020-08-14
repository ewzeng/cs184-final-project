Testing math formulas:

$\Delta w = e^{-i\pi}$ 

How does this look?

![equation](https://latex.codecogs.com/gif.latex?\Delta&space;w&space;=&space;e^{-i\pi})

![equation](https://latex.codecogs.com/svg.latex?\Delta&space;w&space;=&space;e^{-i\pi})

How about this in-line? ![equation](https://latex.codecogs.com/svg.latex?\inline&space;\Delta&space;w&space;=&space;e^{-i\pi}) hehehe

# CS184 Final Project Report: 3D Fluid Simulation

## Team Members

- Bill Li <chengxil@>
- Uma Unni <umaunni@>
- Edward Zeng <edwardzeng@>

## Abstract
Efficient and realistic fluid simulation is of interest to many people, in particular to those working in real-time graphics. In this project, we implemented the paper *Position Based Fluids* by Macklin and Muller to create a fast position-based fluid simulation engine.

## Technical Approach
In pseudocode, the simulation algorithm is as follows:
```
/* Simulation Loop */
WHILE
    FOR EACH PARTICLE:
        Apply forces and acceleration
        Computed (estimated) new position and velocity
    
    FOR EACH PARTICLE:
        Compute and store neighboring particles
    
    FOR EACH PARTICLE:
        Apply incompressibility constraint
    
    FOR EACH PARTICLE:
        Detect and handle collisions
    
    FOR EACH PARTICLE:
        Update particle position
    
    FOR EACH PARTICLE:
        Update particle velocity
        Apply vorticity heuristics
        Apply viscosity heuristics
END WHILE
```

We decided to store the neighborhood information of the particles (i.e. the topology of the fluid) in a lookup table `vector <vector<Particle *> *> neighbor_lookup` where the neighbors of the `i`-th particle in a fluid would be `*neighbor_lookup[i]`.

To compute neighboring particles of a given particle `p`, we considered many options such as bounding boxes (as in Project 3-1) and hash grids (as in Project 4). We finally decided on using an highly optimized kd-tree provided by the library *nanoflann* because neighborhood computation was a potential bottleneck for the simulation, so we wanted to be as fast as possible.

In the paper, the incompressibility constraint on the `i`-th particle is expressed as $\rho_i =$ rest density, where $\rho_i$ is the estimated density around the `i`-th particle, and given by the weighted average:

![equation](https://latex.codecogs.com/svg.latex?\rho_i&space;=&space;\sum_j&space;m_jW(p_i-p_j,&space;h),&space;\&space;m_j&space;\&space;\text{and}&space;\&space;p_j&space;\&space;\text{being&space;mass&space;of&space;the&space;j-th&space;neighbor,&space;respectively})

The paper chose the spikey kernel to be the smoothing kernel $W$. In this project, we decided to choose the cubic spline because it was taught in calls, was easier to compute gradients with, and conceptually simple. So:

![equation](https://latex.codecogs.com/svg.latex?W(p,h)&space;=&space;\frac{1}{\pi&space;h^3}\begin{cases}1&space;-&space;\frac{3}{2}\xi^2&space;&plus;&space;\frac{3}{4}\xi^3&space;&&space;0&space;\leq&space;\xi&space;\leq&space;1&space;\\&space;\frac{1}{4}(2&space;-&space;\xi)^3&space;&&space;1&space;\leq&space;\xi&space;\leq&space;2\\&space;0&space;&&space;\text{otherwise}\end{cases})

where ![equation](https://latex.codecogs.com/svg.latex?\inline&space;\xi&space;=&space;\|p\|/h) and `h` is the smoothing radius.

### Challenges

## LALALA
A paragraph summary of the entire project.
Technical approach

## A
A 1-2 page summary of your technical approach, techniques used, algorithms implemented, etc. (use references to papers or other resources for further detail). Highlight how your approach varied from the references used (did you implement a subset, or did you change or enhance anything), the unique decisions you made and why.

## Problems
A description of problems encountered and how you tackled them.

## Lessions Learned
A description of lessons learned.

## Results
Your final images, animations, video of your system (whichever is relevant). You can include results that you think show off what you built but that you did not have time to go over on presentation day.
## References
Contributions from each team member
A clear description of the work contributed by each team member.

Edward Zeng
Bill Li
Uma Unni
Reminder: You must submit your final deliverables on a website that you can choose to keep up after class if you wish to present this work in your portfolio.