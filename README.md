# ROS2 sample

Sample code written for [this](https://www.udemy.com/course/ros2-for-beginners/?couponCode=NVDPRODIN35) udemy course.

## Dynamic Launcher

A ROS2 node that can dynamically start and stop other nodes at runtime using services.

### Build

```bash
colcon build --packages-select my_py_pkg
source install/setup.bash
```

### Run the Dynamic Launcher

```bash
ros2 run my_py_pkg dynamic_launcher
```

### Start a Node

```bash
ros2 service call /start_node std_srvs/srv/SetBool "{data: true}"
```

### Stop a Node

```bash
ros2 service call /stop_node std_srvs/srv/SetBool "{data: true}"
```

### Monitor Node Status

```bash
ros2 topic echo /node_status
```

### Check Running Nodes

```bash
ros2 node list
```
