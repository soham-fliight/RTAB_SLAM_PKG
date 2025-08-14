# Terminal 1 (launch everything)
```bash
cd ~/Documents/SLAM_Pkg
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zoe_rtabmap.launch.py zoe_rtabmap.launch.py
```


# Terminal 2 rviz2
- Fixed Frame: map
- Add: TF, Path (/rtabmap/path), PointCloud2 (/cloud_map or /cloud_obstacles)


# Testing
## Sources → Sync
```bash
ros2 topic hz  /camera/color/image_rect
ros2 topic hz  /camera/aligned_depth_to_color
ros2 topic hz  /camera/color/camera_info
ros2 topic hz  /rgbd_image
ros2 topic info -v /rgbd_image   # expect publisher rgbd_sync, subscribers rtabmap + rgbd_odometry

## Odometry + Map
ros2 topic echo --once /odom
ros2 topic hz /cloud_obstacles
ros2 topic echo --once /rtabmap/info
```


# Common issues-
## MiDaS drop_path / BEiT errors
### Pin timm and clear cache:

```bash
/usr/bin/python3 -m pip install --user "timm==0.6.12"
rm -rf ~/.cache/torch/hub/intel-isl_MiDaS_master
```
# Exporting ZoeDepth Pkg paths
```bash
export ZOEDEPTH_PATH="$HOME/Documents/Depth_Est/ZoeDepth"
export PYTHONPATH="$ZOEDEPTH_PATH:$PYTHONPATH"
```


# Save Database
```bash
ros2 service call /rtabmap/save_database rtabmap_msgs/srv/SaveDatabase "{filename: '/tmp/zoe_rtabmap.db'}"
```

- Load next time (uses ~/.ros/rtabmap.db by default; copy it there or use --ros-args with params to point elsewhere)


# One-liner bring-up (after files are in place)
```bash
ros2 launch zoe_rtabmap.launch.py zoe_rtabmap.launch.py
```