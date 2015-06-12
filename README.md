# PoseGraphSimulation.jl

Pose-graph simulation in Julia for vehicles driving in street networks, utilizing the Julia OpenStreetMap.jl Package.
PoseGraphSimulation.jl can create a pose-graph data file (similar to the Manhattan world SLAM dataset) to be solved using [iSAM](http://people.csail.mit.edu/kaess/isam/) or another pose-graph solver, and also provides some functions for computing the error of the solution.
This enables Manhattan-world-like datasets to be generated for any route with OpenStreetMap coverage to be solved with iSAM's command line interface.


### References

This code was used to compute the results in my ICRA 2015 paper [1] and thesis [2]:

[1] Steiner T, Huang G, Leonard JJ.  2015.  Location Utility-based Map Reduction. International Conference on Robotics and Automation (ICRA). [PDF](https://marinerobotics.mit.edu/location-utility-based-map-reduction)

[2] Steiner T. 2015. Utility-based Map Reduction for Ground and Flight Vehicle Navigation. Doctoral Dissertation, Massachusetts Institute of Technology. [PDF](http://bit.ly/steiner-thesis)


### Installation

In Julia, run:

```
Pkg.clone("https://github.com/tedsteiner/PoseGraphSimulation.jl.git")
```


### Documentation

See Section 5.2.1 of my [thesis](http://bit.ly/steiner-thesis) for basic usage.

This package is provided without unit tests or detailed documentation, but feel free to open issues for questions not covered in my thesis. 
My hope is that this will be useful to someone, but I currently have no plans to further develop this code. 
If you make improvements or fixes please consider submitting them as pull requests so that others might benefit, as well.

