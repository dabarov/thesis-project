# My master thesis project

### Installation
In workspace's ```src``` folder:

```sh
git clone --recurse-submodules https://github.com/dabarov/thesis-project
git clone -b melodic-devel https://github.com/Kinovarobotics/ros_kortex
```

In workspace:

```sh
catkin_make
source devel/setup.bash
```

### Usage

**Currently everything works in testing environment. Then it will be tranfered to main files.**

To run simulation:

```sh
roslaunch gen3lite_pickplace simulation.launch   
```

To run control unit (currently in testing file) :

```sh
roslaunch gen3lite_pickplace test.launch
```