# Changelog

## 2026-04-28 - Polaris AMCL Startup and Goal Convergence Fixes

This update addresses two operational problems observed when using Polaris with AMCL:

1. The robot sometimes did not start moving after receiving a goal until it was manually nudged with teleop.
2. The robot slowed down excessively near the final goal and could continue crawling without ever satisfying a clean stop condition.

The startup problem came from Polaris consuming pose information passively. AMCL and the TF tree may not publish a fresh pose sample exactly when a goal is sent, especially when AMCL update thresholds are movement-gated. At the same time, the planner and controller initialized their internal robot pose to `(0, 0, 0)`, which meant they could plan or control from a placeholder pose before localization was actually ready.

The convergence problem came from using the final path index as the stopping condition for open paths. That is brittle because the closest sampled path point may not become the last index at the same time the physical robot is close enough to the goal. The controller now uses distance-to-goal semantics instead.

### Fixed

- Prevented the planner from generating paths from the default `(0, 0, 0)` robot pose before localization is available. The planner now tries to refresh the robot pose from TF immediately before planning and refuses to publish a path until it has a valid pose.
- Prevented the vector field controller from publishing motion commands before it has a valid robot pose. The controller now actively refreshes pose from TF each control cycle and publishes zero vector commands while localization is unavailable.
- Added support for the correctly named `PoseWithCovarianceStamped` pose mode while keeping compatibility with the existing misspelled `PoseWithCovarience` value.
- Fixed near-goal behavior for open paths. The controller now uses a distance-based goal tolerance with hysteresis instead of relying on the closest path index becoming exactly the final path point.
- Added smooth slowdown near the final goal with a configurable minimum tracking speed before the stop tolerance. This prevents the robot from crawling indefinitely just outside the goal while still allowing it to stop cleanly once inside tolerance.
- Corrected the feedback linearization loop timing by adding a configurable `control_period` parameter. The default is now `0.05 s` instead of the previous effective `0.5 s` loop.
- Loaded the existing `path_from_points.yaml` file in Polaris navigation launch files so planner frame parameters are applied during normal launches.

### How It Works

- `path_from_points` keeps a `has_robot_pose` gate. On goal or start-service planning requests, it first asks TF for the latest transform between the configured global frame and robot frame. If TF cannot provide it yet, planning is skipped instead of publishing a path from a fake origin pose.
- `vector_field_control` keeps a `has_pose` gate. The control loop refreshes the robot pose from TF before computing commands. Until a valid pose exists, it publishes a zero vector to the feedback linearization node.
- The controller latches goal completion for open paths once the robot is inside `goal_tolerance`. The latch uses `goal_hysteresis` so small localization noise near the threshold does not repeatedly restart motion.
- The controller scales the vector field speed inside `slowdown_radius` using a smoothstep profile. Outside the final tolerance, the command keeps at least `min_tracking_speed`, avoiding the asymptotic low-speed behavior that made the robot crawl forever.
- Feedback linearization remains responsible only for converting the desired global vector into linear and angular velocity commands. Goal semantics stay in the vector field controller, where path and goal context are available.

### Added Parameters

- `goal_tolerance`: distance threshold for declaring an open-path goal reached.
- `goal_hysteresis`: distance threshold used to keep the reached-goal latch stable.
- `slowdown_radius`: distance from the final goal where speed scheduling starts.
- `min_tracking_speed`: minimum vector speed used outside the goal tolerance.
- `control_period`: feedback linearization loop period.
