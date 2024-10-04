# Point Cloud Utils

utilities for working with point clouds; Includes ROS wrappers around PCL filters, cloud sync/merge node and an obstacle detector.

## CustomNodes

- [`rgbd_cloud`](src/rgbd_cloud_node.cpp) - Turns RGB-depth image pairs into point clouds.
- [`cloud_sync`](src/cloud_sync_node.cpp) - Synchronizes multiple point clouds and merges them into a single cloud.
- [`obstacle_detection`](src/obstacle_detection_node.cpp) - Detects obstacles in a point cloud.

### RGBD Cloud Configuration

Topics:
- `color/image_raw` - color image
- `depth/image_raw` - depth image
- `cloud` - point cloud

Both images must arrive with the same timestamp for a cloud to be generated.

Parameters:
```yaml
/**:
  ros__parameters:
    # image transport for color images (default: raw)
    color_transport: compressed
    # image transport for depth images (default: raw)
    depth_transport: compressedDepth
    # horizontal resolution of the point cloud (default: 160)
    cloud_width: 200
    # minimum Z coordinate of a point to be included in the point cloud (default: 0.1)
    min_distance: 0.1
    # maximum Z coordinate of a point to be included in the point cloud (default: 5.0)
    max_distance: 5.0
```

### Cloud Sync Configuration

Topics:
- `inputN` - input point clouds (N = 0, 1, 2, ...)
- `output` - merged point cloud

Parameters:
```yaml
/**:
  ros__parameters:
    # The number of input clouds to synchronize. (default: 0, erroneous value)
    number_of_inputs: 2
    # The frame ID of the output cloud. (default: "base_link")
    output_frame_id: "base_link"
    # Whether to use approximate time synchronization. (default: true)
    approx_sync: true
    # Queue size for the synchronizer. (default: 10)
    queue_size: 10
```

### Obstacle Detection Configuration

Topics:
- `input` - input point cloud
- `output` - obstacle cloud

Parameters:
```yaml
/**:
  ros__parameters:
    # The frame to which transform the point cloud before processing. (default: "odom")
    world_frame: "odom"
    # The radius for normal estimation. (default: 0.2)
    normal_estimation_radius: 0.2
    # The maximum angle between the ground and the normal for a point to be considered ground. (radians, default: 30 degrees)
    max_ground_angle: 0.698

    # The radius for the outlier removal filter. (default: 0.2)
    outlier_removal_radius: 0.2
    # The minimum number of neighbors for the outlier removal filter. (default: 10)
    outlier_removal_min_neighbors: 10

    # The frame of the robot. (default: "base_link")
    robot_frame: "base_link"
    # The minimum height relative to the robot for an obstacle point to be considered. Height is not affected by robot rotation. (default: -1.0)
    min_obstacle_height: -1.0
    # The maximum height for an obstacle point to be considered. (default: 1.0)
    max_obstacle_height: 0.5
```

## PCL Filter Wrappers

- [`radius_outlier_removal`](src/radius_outlier_removal_node.cpp)
- [`statistical_outlier_removal`](src/statistical_outlier_removal_node.cpp)
- [`voxel_grid`](src/voxel_grid_node.cpp)

Those can be chained in a launch file to create a pipeline:
```py
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_cloud_utils',
            executable='rgbd_cloud',
            remappings=[
                ('color/image_raw', '/camera/color/image_raw'),
                ('depth/image_raw', '/camera/depth/image_raw'),
                ('cloud', '/camera/point_cloud/raw'),
            ],
        ),
        Node(
            package='point_cloud_utils',
            executable='voxel_grid',
            remappings=[
                ('input', '/camera/point_cloud/raw'),
                ('output', '/camera/point_cloud/voxel_grid'),
            ],
        ),
        Node(
            package='point_cloud_utils',
            executable='radius_outlier_removal',
            remappings=[
                ('input', '/camera/point_cloud/voxel_grid'),
                ('output', '/camera/point_cloud'),
            ],
        ),
    ])
```
