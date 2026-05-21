# Project Notes

This project simulates sweeping robot coverage trajectories in a square environment.

## Environment

- Boundary: square area from `-200` to `200` on both axes.
- Step size: `5` units per movement update.
- Stop condition: simulation ends after `150` wall collisions.
- Start position: `(0, 0)`.
- Initial heading: random angle from `0` to `360` degrees.

## Model 1: Fixed Turn

The fixed turn model changes direction by a fixed clockwise angle after every collision.

This model is simple and deterministic after the initial random heading. It is useful as a baseline for comparing coverage patterns.

## Model 2: Adaptive Random Turn

The adaptive model uses the distance between two consecutive collision points to estimate how far the robot traveled before impact. That distance is mapped to a turn angle in the range `90` to `300` degrees.

After each collision, the robot randomly chooses whether to turn left or right by the mapped angle.

## Visualization

Both scripts use:

- Turtle for live trajectory drawing.
- Matplotlib for a final static trajectory plot.

## Clean Repository Choices

The original combined code was split into two runnable scripts without changing the underlying behavior. This makes the repository easier to understand while keeping the project close to the original assignment/demo version.
