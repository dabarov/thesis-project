# My master thesis project

## Prerequisites

 - Download [src folder](https://drive.google.com/file/d/1W3sZJq_g6yIq7tqLrgCdkY9-GfKc0jDt/view?usp=sharing) and extract it into new workspace.
 - Install `ros_kortex` prerequisites:

```sh
sudo apt install python3 python3-pip
sudo python3 -m pip install conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
```

 - Install ros dependencies (run in workspace folder):

```sh
rosdep install --from-paths src --ignore-src -y
```
 - Build workspace and source it:

```sh
catkin_make
source devel/setup.bash # setup.zsh if you use zsh
```

## Launch environment

### Launch office world

```sh
roslaunch gen3lite_pickplace office_world_spawner.launch
```

### Launch moveit

```sh
roslaunch my_moveit_pkg move_group.launch
```

### Spawn table and objects

```sh
roslaunch gen3lite_pickplace spawn_table.launch
roslaunch gen3lite_pickplace spawn_objects_to_pick.launch
```

### Move gripper to initial position

```sh
rosrun kinova_moveit dev.py 
```

### Run services

 - Run move server:

```sh
rosrun kinova_moveit move_server.py
```

 - Run grab server:

```sh
rosrun kinova_moveit grab_server.py
```

## Usage

### Call move service

```sh
 rosservice call /kinova_moveit/move "offset: 
  x: 0.0  
  y: 0.1  
  z: 0.0  
close: 0.9"
```

```close``` here is position of the gripper. ```x, y, z``` are offsets from the current position of the gripper.

### Call grab and release services

 - To grab certain object use:

```sh
rosservice call /kinova_moveit/grab "targetName: 'orange'"
```

 - To release current object in the gripper run:

```sh
rosservice call /kinova_moveit/release "{}"
```
