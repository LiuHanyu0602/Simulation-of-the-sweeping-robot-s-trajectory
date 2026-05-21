# Sweeping Robot Trajectory Simulation

This repository contains a Python simulation of sweeping robot coverage trajectories inside a square boundary.

The project compares two simple wall-collision behavior models for a robotic vacuum cleaner:

1. Fixed-angle turning after each wall collision.
2. Adaptive random left/right turning based on the distance traveled between consecutive collisions.

## Project Idea

A sweeping robot with limited sensors can still follow a simple closed-loop behavior:

```text
sense -> judge -> decide -> act -> feedback
```

In this simulation, wall collisions are used as feedback. The robot updates its movement direction after each collision, and the resulting path is visualized with Turtle and Matplotlib.

## Simulation Models

### Fixed Turn Model

After each wall collision, the robot turns by a fixed clockwise angle and continues moving.

Script:

```text
scripts/fixed_turn_simulation.py
```

### Adaptive Random Turn Model

After each wall collision, the robot randomly turns left or right. The turn angle is mapped from the distance traveled since the previous collision.

Script:

```text
scripts/adaptive_turn_simulation.py
```

## Repository Layout

```text
.
├── scripts/
│   ├── fixed_turn_simulation.py
│   └── adaptive_turn_simulation.py
├── docs/
│   └── project-notes.md
├── requirements.txt
└── README.md
```

## Requirements

- Python 3
- matplotlib
- turtle, math, and random from the Python standard library

Install the external dependency:

```bash
pip install -r requirements.txt
```

## Run

Fixed turn model:

```bash
python scripts/fixed_turn_simulation.py
```

Adaptive random turn model:

```bash
python scripts/adaptive_turn_simulation.py
```

Each script opens a Turtle animation window and then displays the final trajectory with Matplotlib.

## Notes

This is a completed coursework-style simulation project. The cleanup keeps the original behavior while making the repository easier to browse and run from GitHub.
