# My master thesis project

### Prerequisites
```sh
sudo apt install python3 python3-pip
sudo python3 -m pip install conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
```

### Installation
In workspace's ```src``` folder:

```sh
git clone --recurse-submodules https://github.com/dabarov/thesis-project
git clone -b melodic-devel https://github.com/Kinovarobotics/ros_kortex
```

In workspace:

```sh
rosdep install --from-paths src --ignore-src -y
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