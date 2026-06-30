# UAV_USV Workspace

This workspace keeps the original `uav_usv_sim` package runnable while the team incrementally migrates functionality into independently owned ROS 2 packages.

## Compatibility

The original commands remain valid:

```bash
ros2 launch uav_usv_sim uav_usv_px4_sim.launch.py
ros2 launch uav_usv_sim boat_nav2_navigation.launch.py
```

The integration package provides wrappers around the same implementation:

```bash
ros2 launch uav_usv_bringup legacy_px4_sim.launch.py
ros2 launch uav_usv_bringup legacy_boat_nav2.launch.py
ros2 launch uav_usv_bringup legacy_colregs_test.launch.py
```

See [COLLABORATION.md](COLLABORATION.md) and [docs/INTERFACES.md](docs/INTERFACES.md) before developing a module.
