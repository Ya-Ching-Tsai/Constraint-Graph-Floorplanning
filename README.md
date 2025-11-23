# Constraint Graph Floorplanning

A C++ implementation of floorplanning using Horizontal Constraint Graph (HCG) and Vertical Constraint Graph (VCG). Computes coordinates, slack,critical edges, and optimized area based on HW3 specifications.

---

### Features
- Build HCG & VCG from input
- Lower-left placement (Formula 1)
- Layout width/height computation (Formula 2)
- Upper-right placement (Formula 3)
- Slack computation
- Critical edge detection
- Horizontal-edge shifting for area optimization

---
### Theory Overview
This project implements classical Constraint Graph–based floorplanning, a fundamental method in VLSI physical design for determining legal module placement.
Horizontal Constraint Graph (HCG)
* Represents left–right placement constraints.
* A directed edge A → B indicates that module A must appear to the left of module B.
* Module x-coordinates are computed by finding the longest path in the HCG.

#### Vertical Constraint Graph (VCG)
* Represents bottom–top constraints.
* A directed edge A → B indicates that module A must be placed below module B.
* Module y-coordinates are obtained using the longest path in the VCG.

#### Slack and Critical Edges
* Slack measures how much a constraint can relax without affecting layout legality.
* Edges with zero slack are critical edges and determine the final layout size.
* Identifying critical edges is important for optimization.

#### Horizontal Shifting Optimization
* Uses slack information along HCG edges to shift modules leftward when possible.
* Reduces total layout width while preserving all constraints.
* Mimics early-area–optimization behavior in commercial EDA floorplanning tools.

---
### Compile & Run
#### Compile
> g++ -std=c++11 -O2 -o constraint_graph_floorplan constraint_graph_floorplan.cpp
#### Run
> ./constraint_graph_floorplan <input_case>



