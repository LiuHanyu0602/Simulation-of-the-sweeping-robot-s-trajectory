# Simulation-of-the-sweeping-robot-s-trajectory
Comparison of Trajectory Coverage for Two Behavioral Models

**The robotic vacuum cleaner is be able to:**
Clean an area of 1000 sq ft in one charge cycle.
Detect and avoid physical obstacles during operation.
Automatically return to a charging station either after cleaning or when the dust bag exceeds a set weight.
Trigger an audible alarm (e.g., buzzer) when the dust bag weight reaches the preset threshold.


With a very limited number of sensors, we simulated the closed-loop control of ‘sense-judge-decide-act-feedback’ as much as possible in code. This is the basis of the core behavioural logic of a real sweeping robot. This algorithm realizes adaptive steering according to the timing and intensity of collision by dynamically mapping the intervals of consecutive line-bumping events into rotation angles. In addition, when the number and time of alternating left and right collisions are monitored to exceed a threshold value, a backward plus large-angle random steering extrication strategy is triggered.

