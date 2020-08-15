# CS184 Final Project Report: 3D Fluid Simulation

## Team Members

- Bill Li <chengxil@>
- Uma Unni <umaunni@>
- Edward Zeng <edwardzeng@>

## Notes
- Our GitHub repo is at https://github.com/ewzeng/cs184-final-project. **Most of our work was done on the `experimental` branch**. (The `master` branch was virtually unused.)
- Because GitHub markdown does not support in-line LaTex well, we use backslashes to denote Greek letters. For example, `\alpha` denotes the Greek letter alpha.

## Abstract
Efficient and realistic fluid simulation is of interest to many people, in particular to those working in real-time graphics. In this project, we implemented the paper *Position Based Fluids* by Macklin and Muller to create a fast position-based fluid simulation engine.

## Technical Approach

### Particle Based Fluid Simulation
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

To compute neighboring particles of a given particle `p`, we considered many options such as bounding boxes (as in Project 3-1) and hash grids (as in Project 4 and in the paper). We finally decided on using an highly optimized kd-tree provided by the library *nanoflann* because neighborhood computation was a potential bottleneck for the simulation, and we wanted to be as fast as possible.

In the paper, the incompressibility constraint on the `i`-th particle is expressed as `\rho_i = \rho_0`, where `\rho_0` is the rest density, and `\rho_i` is the estimated density around the `i`-th particle, and given by the weighted average:

<img src="https://latex.codecogs.com/svg.latex?\rho_i&space;=&space;\sum_j&space;m_jW(p_i-p_j,&space;h),&space;\&space;m_j&space;\&space;\text{and}&space;\&space;p_j&space;\&space;\text{being&space;mass&space;and&space;position&space;of&space;the&space;j-th&space;neighbor,&space;respectively"><br>

The paper chose the spikey kernel to be the smoothing kernel `W`. In this project, we decided to choose the cubic spline because it was taught in class, was easier to compute gradients with, and conceptually simple. So:

<img src="https://latex.codecogs.com/svg.latex?W(x,h)&space;=&space;\frac{1}{\pi&space;h^3}\begin{cases}1&space;-&space;\frac{3}{2}\xi^2&space;&plus;&space;\frac{3}{4}\xi^3&space;&&space;0&space;\leq&space;\xi&space;\leq&space;1&space;\\&space;\frac{1}{4}(2&space;-&space;\xi)^3&space;&&space;1&space;\leq&space;\xi&space;\leq&space;2\\&space;0&space;&&space;\text{otherwise}\end{cases}"><br>

where `\xi = x.norm()/h` and `h` is the smoothing radius.

Enforcing the incompressibility constraint is as simple as computing a positional update for each particle such that

<img src="https://latex.codecogs.com/svg.latex?&space;\sum_i&space;C_i=&space;0,&space;\quad&space;C_i&space;=&space;1&space;-&space;\rho_i&space;/&space;\rho_0&space;"><br>

Fortunately, the paper does most of the math to compute this positional update. For the `i`-th particle, the positional update required is 

<img src="https://latex.codecogs.com/svg.latex?&space;\Delta&space;p_i&space;=&space;\frac{1}{\rho_0}&space;\sum_j(\lambda_i&space;&plus;&space;\lambda_j&space;&plus;&space;s_{corr})\nabla&space;W(p_i-p_j,h)"><br>

where `p_i`, `p_j` are ths positions of the `i`-th, `j`-th particles, `s_corr` is the artificial pressure term, the summation is taken over the neighboring particles of the `i`-th particle, and `\lambda_i` is given by:

<img src="https://latex.codecogs.com/svg.latex?\lambda_i&space;=&space;-&space;\frac{C_i}{\sum_k|\nabla_{p_k}&space;C_i|^2&space;&plus;\varepsilon}"><br>

The `\epsilon` is here to prevent the denominator being 0.

Unforunately, the paper was rather vague on implementing self-collisions, so we had to come up with our own method. Our first idea was to do something similar to Project 4:
```
FOR EVERY NEIGHBOR q OF p:
    sum = 0, cnt = 0
    IF p and q are too close:
        sum += correction, cnt++
p.position += sum/cnt
```
However, with this self-collision, the fluid kept collaspsing on top of itself. We suspect this was because the self-collision code did not make any guarantees of the distances between particles as we were taking an average at the end. So we decided to give each particle a radius, and enforced a more drastic self-collision algorithm:
```
FOR EVERY NEIGHBOR q OF p:
    IF p, q intersect:
        p.position += amount so p and q do not intersect
```
This solved the problem of the fluid collaspsing on top of itself, but as a side effect, the particles of the fluid got extremely bouncy, and we couldn't find a good way to mitigate that.

The paper also worked out the math for computing `s_corr` (artifical pressure for surface tension), vorticity, and viscosity. We implemented these equations as is, only modifying some constants to give us more realistic simulations with our cubic spline kernel. For completeness, we include the equations:

<img src="https://latex.codecogs.com/svg.latex?s_{corr}&space;=&space;-k\left(\frac{W(p_i-p_j,h)}{W([0.2h,&space;0,0],&space;h)}\right)^4">
<br>

<img src="https://latex.codecogs.com/svg.latex?f_i^{voriticity}&space;=&space;\varepsilon&space;\cdot&space;\left(\eta&space;\times&space;\omega_i&space;\right),&space;\&space;\eta&space;=&space;\frac{\nabla&space;\omega_i}{\|\nabla&space;\omega_i\|},&space;\&space;\omega_i&space;=&space;\sum_j(v_j-v_i)&space;\times&space;\nabla_{p_j}W(p_i-p_j,h)">
<br>

<img src="https://latex.codecogs.com/svg.latex?v_i^{new}&space;=&space;v_i&space;&plus;&space;c\sum_i(v_i-v_j)&space;\cdot&space;W(p_i-p_j,h)">
<br>

Here, the second and third equations reflect the impact of applying vorticity and viscosity, respectively. Because our fluid particles were rather bouncy, we opted not to include voritcity (which creates bigger splashes) into the default simulation. (Furthermore, the paper states that vorticity confinement is optional.) Simulation with vorticity confinement implemented can be found in the `fluid-sim` branch of the github repo.


### Mesh Generating Algorithm (Marching Cubes)

In addition to particle simulation, we aspired to create realistic looking videos by converting the particles into a mesh and rendering the mesh. To do this, we wanted to implement the *Marching Cube Algorithm*. The *Marching Cube Algorithm* requires a kernel function `F`, which takes the coordinate `(x,y,z)` as input and outputs a single value `c`. The pseudocode is as follows
```
PARTITION the WORLD SPACE into (n^3) CUBES

FOR EACH CUBE C:
    FOR EACH VERTEX V of C:
        Compute F(V),if F(V) > ISOVALUE, then COLOR V
    Lookup the list of triangles need to draw based on coloring scheme
    Draw the list of triangles needed.
```
Based on this paper [here](http://academy.cba.mit.edu/classes/scanning_printing/MarchingCubes.pdf), there are 14 unique cases up to rotation as follows. 
<br>
![](https://i.imgur.com/p1iajLb.png)
<br>
Observe since each vertex can either be colored or not colored, there are `2^8 = 256` total cases. To compute exactly which index we are using, we can use the following puesdocode
```
lookup = 0
FOR i, vertex IN cube:
    IF f(vertex) > isovalue:
        lookup = lookup | 1 << i
RETURN lookup
```
We used this lookup table right [here](http://paulbourke.net/geometry/polygonise/), which returns triplets of edge numbers, terminated by trailing `-1` at the end of the list. Then for each edge, we placed the vertices by linearly interpolate `f(v1)` and `f(v2)`. We then called OpenGL to render the triangles.

However, when we attempted to implement the *Marching Cubes Algorithm* in this way, we did not have enough time to completely finish, debug and make it work as expected (see the `linux` branch of the github repo for our attempt).

Out of curosity, we exported one of our frames as a point cloud file and then used MeshLab's *Marching Cube Algorithm* to generate the following mesh. If we had more time, we would have generated something similar with our own code and used Blender to apply water textures and shaders to render a video.
<br>
![](https://i.imgur.com/6f02At6.png)


### Other Challenges
When we started the project, we decided to heavily borrow from the code for Project 4. However, it was difficult to make significant changes from code with lots dependencies, and the linker was not very coorperative. So after a few days, we decided to scrap a good amount of the code for Project 4 and write our own interactive renderer. This took a while because we had to first learn how to use OpenGL.

Also, because we were developing on different operating systems, there were various frustrating instances where our code would compile for one OS but fail to compile with others. It was rather difficult to solve these problems.

A third challenge specific was that fluid simulation requires a lot of given parameters, so it was hard to determine a optimal set of constants to create the most realistic simulation. We ended up tweaking a lot of parameters every time we implemented a new constraint, just to keep things looking as realistic as possible.

### Lessions Learned
A description of lessons learned.

## Results
This is our result of simulating a fluid with 1000 particles falling from rest. (Because of our implementation of self collision, the fluid looks rather bouncy.)

![](https://i.imgur.com/k3Ffwmu.gif =640x360)

![](https://i.imgur.com/F3CghOx.gif =640x360)

For the more detailed simulation with 5000 particles, the video can be found [here](https://www.youtube.com/watch?v=fGKn2YpF01U) and [here](https://www.youtube.com/watch?v=87KfAjHhaZE)

These videos were generated by exporting our simulation data to MeshLab's renderer because the aesthetics of our own renderer were too ugly to put on this report. To get a idea of what our renderer looks like, simply make and run `./clothsim` on the `experimental` branch in github (you may need to change the shader path depending on whether you are on Windows or Linux). You'll see what we mean. (However, our renderer is interactive and runs in real-time! Press `p` to pause and `r` to restart the simulation!)

## References and Resources
[Position Based Fluids](http://mmacklin.com/pbf_sig_preprint.pdf)
[Marching Cubes](http://academy.cba.mit.edu/classes/scanning_printing/MarchingCubes.pdf)
[nanoflann](https://github.com/jlblancoc/nanoflann)
[Surface Reconstruction](https://www.cc.gatech.edu/~turk/my_papers/sph_surfaces.pdf)
[Triangulation Table](http://paulbourke.net/geometry/polygonise/)
[Meshlab](https://www.meshlab.net/)
[Learn OpenGL](https://learnopengl.com/) (Super useful when we were building our own renderer)

## Contributions
Edward Zeng
- Built the OpenGL renderer, incorporated the nanoflann library, created the basic structure of the skeleton code, implemented the fluid simulation math and collisions, debugged the fluid simulation math, helped write the final report

Bill Li
- Helped Edward bash out the math behind fluid simulation, spent a lot of time rendering videos and tweaking parameters, researched and worked on implementing the marching cubes algorithm, helped write the final report 

Uma Unni

- Worked on marching cubes algorithm, and helped debug, debug, debug