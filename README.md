# kinova-moveit

Assuming you have the workspace. 

  1. Spawn the office environment, tables and objects. 

```sh
roslaunch gen3lite_pickplace office_world_spawner.launch
roslaunch gen3lite_pickplace spawn_table.launch
roslaunch gen3lite_pickplace spawn_objects_to_pick.launch
```

  2. Run all services

```sh
rosrun kinova_moveit api_server.py
```
```sh
rosrun kinova_moveit grab_server.py
```
```sh
rosrun kinova_moveit find_server.py
```

  3. Run client for demo

```sh
rosrun kinova_moveit client
```