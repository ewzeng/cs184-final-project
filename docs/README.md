# CS184 Final Project Design Doc: 3D Fluid Simulation

The finalized project report can be found [here](ewzeng.github.io/cs184-final-project).

## Team Members

- Bill Li <chengxil@>
- Uma Unni <umaunni@>
- Edward Zeng <edwardzeng@>

## Summary
In this project, we implement the paper *Position Based Fluids* to simulate incompressible Newtonian fluids. This is done in two steps: we create a particle-based simulation, and then convert the particles into a mesh to render.

## Problem Description
It is difficult to simulate 3D fluids realistically without a large amount of computing power. This is in particular a challenge for games and graphics applications that are required to run in real-time. Here we implement a Verlet-integration type fluid simulation that promises to be fast and efficient.

## Goals and Deliverables

#### Baseline Deliverables
In this project, we will create many short clips of water splashing inside an invisible container. First, we show clips rendered with only water particles (no meshes), and then we will show clips rendered using meshes in Blender. This will be the key result of the project.

To do this, we will implement a fluid simulation engine that given starting parameters like the number of particles, time between each frame, and initial particle position, will generate a realistic fluid simulation.

Finally, we will create a final report to present and summarize our findings.

#### Aspirations
If we have time, we would like to add support for liquids of different vicosities - not just water. That way, we can simulate many different fluids (possibly at the same time), maybe even possibly honey.

It would also be interesting to simulate cool scernarios like water pouring out of a kettle. Furthermore, we could add support for moving objects (like what happens when a solid ball in dropped into water).

If we have enough computational power and time, we could ray trace water, taking into account both refraction and reflection to build a photorealistic video.

We could also test different inputs and make simplifications to see the minimal amount of computation required to generate an "acceptable" 3D fluid simulation.


#### Analysis and Quality
It is difficult to define a benchmark for the quality of the results because there is no precise benchmark to comapre with, but our results should look realistic to eye and not take too long to render. In particular, the simulation should be stable in the long run, and not diverge.

In doing this project, we hope to answer with our analysis whether or not real-time realistic 3D fluid simulation is a feasible idea on a high-end personal computer or laptop.

## Schedule
Week 1: Read the paper and build the skeleton code for both position based particle simulation and anisotropic kernel surface. Learn how to use blender to render objects. 

Week 2: Try to finish the coding by the beginning of week, start debugging the code if there is any and finish rendering as well as the final report. Can try to implement extra features if there is timing remaining. 

## Resources
[Position Based Fluids](http://mmacklin.com/pbf_sig_preprint.pdf)
[Reconstructing Surfaces of Particle-Based Fluids
Using Anisotropic Kernels](https://www.cc.gatech.edu/~turk/my_papers/sph_surfaces.pdf)
[Marching Cubes Algorithm](http://www.cs.carleton.edu/cs_comps/0405/shape/marching_cubes.html)
[Blender](https://www.blender.org/features/animation/)

We will we using our personal computers to do most of rendering, but we may take advantage of the hive machines if they are available. We will be using Blender to render our meshes.