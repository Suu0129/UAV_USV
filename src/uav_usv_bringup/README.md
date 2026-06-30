# uav_usv_bringup

Ownership: system integration team.

This package is the stable user entry point. During migration its launch files include the original `uav_usv_sim` launch files, so existing behavior remains unchanged.

```bash
ros2 launch uav_usv_bringup legacy_px4_sim.launch.py
ros2 launch uav_usv_bringup legacy_boat_nav2.launch.py
ros2 launch uav_usv_bringup legacy_colregs_test.launch.py
```
