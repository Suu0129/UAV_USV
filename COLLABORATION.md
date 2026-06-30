# Parallel Development Guide

## 1. Migration rule

`uav_usv_sim` is the preserved legacy package. Do not delete or move an active implementation until its replacement package:

1. Builds independently.
2. Passes its package tests.
3. Passes an integrated launch smoke test.
4. Has been selected by a bringup configuration.
5. Has survived at least one integration cycle.

Until then, new packages are development boundaries, not replacements.

## 2. Package ownership

| Package | Primary responsibility |
|---|---|
| `uav_usv_interfaces` | Shared messages, services, actions |
| `uav_usv_description` | Geometry, inertia, URDF/Xacro, sensor extrinsics |
| `uav_usv_gazebo` | Worlds, environment, sensors, simulation plugins |
| `uav_usv_usv_control` | USV control and actuator execution |
| `uav_usv_uav_control` | PX4 and UAV control |
| `uav_usv_perception` | LiDAR, camera, AIS, tracking, fusion |
| `uav_usv_localization` | SLAM, GNSS/IMU fusion, localization TF |
| `uav_usv_navigation` | Nav2, costmaps, planning and local control |
| `uav_usv_colregs` | Collision risk and COLREGs decisions |
| `uav_usv_mission` | High-level cooperative task logic |
| `uav_usv_bringup` | Integrated launch and configuration |
| `uav_usv_tests` | Contract and integration tests |

Each package should have one primary owner and at least one reviewer.

## 3. Git workflow

- `main`: always buildable and runnable.
- `develop`: optional integration branch when the team needs staged integration.
- `feature/<package>-<feature>`: normal development.
- `fix/<package>-<issue>`: bug fixes.

Example:

```bash
git switch -c feature/perception-mid360
```

Keep pull requests scoped to one package whenever possible. Shared interface changes require review from the integrator and every affected package owner.

## 4. Definition of done

A pull request must include:

- Purpose and affected package.
- Input/output topics and TF changes.
- Parameters and defaults.
- Build command.
- Test or rosbag reproduction command.
- Evidence that unrelated packages still build.

Never commit `build/`, `install/`, `log/`, Python caches, or personal absolute paths.

Run the shared check before opening a pull request:

```bash
./tools/check_workspace.sh
```

## 5. Integration cadence

Recommended weekly cycle:

1. Freeze interface changes at the start of the cycle.
2. Develop against mocks or recorded bags.
3. Merge package changes after review.
4. Run full build and launch smoke tests.
5. Tag a known-good integration point.

## 6. Worktrees

Use independent worktrees for concurrent tasks:

```bash
git worktree add ../UAV_USV_mid360 feature/perception-mid360
git worktree add ../UAV_USV_nav feature/navigation-dynamic-costmap
```

Give every worktree separate colcon directories under `/var/tmp`.
