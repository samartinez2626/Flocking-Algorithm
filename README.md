# Flocking Simulation with Boids

This repository contains a complete Python + `pygame` implementation of a classic 2D boids simulation. The project demonstrates how three simple local rules produce believable flocking behavior:

- **Separation** keeps boids from crowding one another.
- **Alignment** nudges boids toward the average heading of nearby neighbors.
- **Cohesion** pulls boids toward the local center of mass.

## Features

- At least 30 boids on screen by default (`40` boids).
- Screen wrapping so the flock stays inside the play space.
- Speed and steering-force limits for stable motion.
- Triangle boids that point in their direction of travel.
- Configurable boid count, radii, max speed, max force, and rule weights.
- Optional extension: left-click to place a goal point for the flock to seek.
- Heads-up display with current settings.

## Requirements

- Python 3.10+
- `pygame`

Install dependencies with:

```bash
pip install -r requirements.txt
```

## Running the Simulation

```bash
python main.py
```

You can tune the simulation from the command line:

```bash
python main.py --num-boids 60 --neighbor-radius 70 --separation-weight 1.8
```

### Controls

- **Left click**: place a goal point for the flock to seek.
- **C**: clear the goal point.
- **H**: show/hide the HUD.
- **Close window**: quit.

## Reflection Questions

### Which rule has the biggest visible effect?
Separation often has the strongest immediate visual impact because it prevents collisions and creates the spacing that makes the flock readable.

### What happens if separation is too strong?
The flock becomes jittery and scattered because boids constantly push away from one another.

### What happens if cohesion is too strong?
Boids over-cluster into dense blobs and may orbit unnaturally around the center of mass.

### Why is flocking emergent behavior?
No boid has a global leader or full knowledge of the flock. Group motion appears from many local interactions.

### What is the time complexity of checking every boid against every other boid?
The straightforward implementation is **O(n²)** per frame.

### How could this be optimized?
Use spatial partitioning such as a uniform grid, quadtree, or spatial hashing so each boid only checks nearby regions.

## File Overview

- `main.py`: full boids simulation.
- `requirements.txt`: Python dependency list.
