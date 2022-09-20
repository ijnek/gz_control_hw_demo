# Demo for gz_control_hw

[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)
[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_humble.yaml?query=branch:rolling)


To launch gazebo with the robot in it, run:

```sh
ros2 launch gz_control_hw_demo gz.launch.py
```

To start ros2 control, run:
```sh
ros2 launch gz_control_hw_demo control.launch.py
```

Now, publish a command on ``/joint_group_position_controller/commands`` like you would with any joint group position controller:

```sh
ros2 topic pub --once /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "
data:
- -0.5
- -0.5
"
```
