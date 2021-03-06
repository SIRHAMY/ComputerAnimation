# Computer Animation Projects
Some physics simulation projects I completed for my Computer Animation course. The course number has intentionally been ommitted to make it a little harder for future students to snag my code. That being said, you can pretty easily put 2 and 2 together via my [Linkedin](https://www.linkedin.com/in/hamiltongreene).

The code provided are the main files that I worked on. Most (all?) of the project require external files to run. I have ommitted these as I'm not sure whether or not I have permission to publish/distribute them. In their stead, I've included a short demo and summary for each project to give you a general idea of what's going on.

## Overview
Click on the titles to watch a demo

### [**Project 1: Interpolation**](https://www.youtube.com/watch?v=iAuspv9IAK0)
We had to implement four different kinds of interpolations between custom control points instantiated and removed via mouse clicks. The four interpolations were:

* Linear
* Bezier
* Catmull-Rom
* B-Spline

### [**Project 2: Galileo's Experiment**](https://www.youtube.com/watch?v=Y8ER3qpQ0O0)
Here, we compared the accuracy of the numerical integration methods: explicit Euler, midpoint, and analytical. Analytical utilizes the basic formula for gravity while euler and midpoint are step methods that attempt to estimate the correct velocity based on the current state of the body.

### [**Project 3: Tinkertoy**](https://www.youtube.com/watch?v=_SxRLK4yBvQ)
The goal here was to implement constraints such that the firs ball stays on the circle and the second ball is attached to the first by a rigid line, with forces acting on each other as they should. As you can see, the longer the simulation runs, the more error is introduced until it finally spins out of control.

### [**Project 4: Pinata**](https://www.youtube.com/watch?v=lrPQJfbUBtQ&feature=youtu.be)
We were given skeleton code that provided a box (the pinata) and two rigid bodies - one a cube and the other a sphere. Starting the simulation would result in the rigid bodies falling infinitely.

To Implement:

* Quaternion representation for the orientation of each rigid body
* Calculations for angular motion. 
* Colliding contact between a rigid body and the pinata and between the rigid bodies themselves. 

### [**Project 5: Fun with Fluids**](https://www.youtube.com/watch?v=VrAC1NpH5ao&feature=youtu.be)
The skeleton code provided an implementation of a density field. The white marks show the density, which I'm applying to the space with a left click. Our goal was to advect the velocity vector field using the semi-Lagrange method.

### [**Project 6: Twister**](https://www.youtube.com/watch?v=gSZYbGCMh94&feature=youtu.be)
This project provided a basic marionette figure with a hard-coded inverse kinematic function that allowed the right leg to be moved correctly if you interacted with the marker located on the right foot. The goal was to implement a general inverse kinematic solver that worked for every marker on the marionette.