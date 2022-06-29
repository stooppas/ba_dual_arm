# Examples for Multi-Arm MoveIt implementation
![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/66126160/172908340-3391bfac-9480-4070-8f43-db529522412d.gif)
## Build
1. Build MoveIt2 from my repository
    
    Install all dependecies from [Build Moveit from Source](https://moveit.ros.org/install-moveit2/source/)
    ```
    sudo apt remove ros-humble-moveit-msgs

    mkdir -p ws_moveit/src && cd ws_moveit/src
    git clone https://github.com/stooppas/moveit2 && git clone https://github.com/stooppas/moveit_msgs -b ros2

    for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
    rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

    cd ..
    source /opt/ros/humble/setup.bash
    colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
2. Install [Niryo One ROS2](https://github.com/stooppas/niryo_one_ros2)
3. Build this
    ```
    mkdir -p ba_ws/src && cd ba_ws/src
    git clone https://github.com/stooppas/ba_dual_arm
    cd .. && source ../ws_moveit
    colcon build
    ```
## Usage 

1. Start all Niryo One Clients
    - The examples need two clients with the namespace "a_bot" and "b_bot"
    - To start: ```ros2 launch niryo_one_bringup start_sim.launch.py ns:=a_bot```

2. Open new terminal and start RViz and move_group with ```ros2 launch niryo_one_moveit_config niryo_one_moveit.launch.py```
    - **This must always be launched, as it is the server that handles the incoming action requests from the examples**
3. Open new terminal and start the examples with ```ros2 launch ba_dual_arm_examples eef_align.launch.py```
