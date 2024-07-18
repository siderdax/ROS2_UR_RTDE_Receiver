# ROS2_UR_RTDE_Receiver
ROS2 Universal Robot RTDE data topic publisher

## Prerequisite
Install ROS2 Humble
See [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

## Installation
```
mkdir /opt/ros/workspace/src
cd /opt/ros/workspace/src
git clone https://github.com/siderdax/ROS2_UR_RTDE_Receiver.git
cd ../
colcon build --symlink-install
```

## Run

### Common
```
source /opt/ros/workspace/install/setup.bash
ros2 run ur_rtde_receiver receiver
```

## Configuration

### Remap topic name
See [Remapping](https://design.ros2.org/articles/static_remapping.html)
```
ros2 run ur_rtde_receiver receiver --ros-args -r ur_rtde_data:=<new_topic_name>
```

### Configuration
1. MQTT hostname
    ```
    ros2 run ur_rtde_receiver receiver --ros-args -p ur_rtde_env.host:=<address> -p 
    ```
    
2. RTDE configuration file path(default: install/ur_rtde_receiver/share/ur_rtde_receiver/config/configuration.xml)
    ```
    ros2 run ur_rtde_receiver receiver --ros-args -p ur_rtde_env.config:=<config file path> 
    ```
    
3. RTDE Frequency
    ```
    ros2 run ur_rtde_receiver receiver --ros-args -p ur_rtde_env.frequency:=<frequency>
    ```
    
4. With paramter file
    1. Create parameter file "params.yaml" and write below
        ```
        /ur_rtde_data:
        ros__parameters:
            ur_rtde_env:
              host: 127.0.0.1
              config: configuration.xml
              frequency: 10
        ```
    2. Run node
        ```
        ros2 run ur_rtde_receiver receiver --ros-args --params-file params.yaml
        ```