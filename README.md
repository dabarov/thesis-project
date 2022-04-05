# My master thesis project

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

Run move server:

```sh
rosrun kinova_moveit move_server.py
```

Run grab server:

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

`close` here is position of the gripper. `x, y, z` are offsets from the current position of the gripper.

### Call grab and release services

To grab certain object use:

```sh
rosservice call /kinova_moveit/grab "targetName: ''"
```

To release current object in the gripper run:

```sh
rosservice call /kinova_moveit/release "{}"
```
