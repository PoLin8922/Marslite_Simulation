socially-store-robot
===
Requirement
---
* Nvidia driver >= 410.48
* Docker >= 19.03.13
* Nvidia-docker >= 2.5.0

Installation
---
Pull docker image
```
docker pull oocami35029287/marslite_simulation:cuda10
```
Pull this github
```
git@github.com:oocami35029287/socially-store-robot.git
```

Make files
---
Before you start, you should compile all the required files.
```bash
source mars_ws/compile_py36.sh
source yolo_ws/compile_py36.sh
```
To run the corrider simulation
---
If you want to navigate robot in the crowd, you should try this.

1. Open padsim simulator in rviz
```bash
#terminal 1
sourcemars
roslaunch pedsim_simulator gym_crowd_environment.launch scene_file:=crossing_corridor.xml
```
2. Open gazebo
```bash
#terminal 2
sourcemars
roslaunch pedsim_gazebo_plugin crossing_corridor.launch
#terminal 3
roslaunch mars_lite_description spawn_mars.launch
```
3. Let pedestrian move in simulation
```bash
rosservice call /pedsim_simulator/unpause_simulation "{}"
```
4. Open pedestrian detection
```bash
#terminal 4
sourceyolo
roslaunch scan yolodetect.launch
```
5. Open amcl
```bash
#terminal 5
sourcemars
roslaunch turtlebot3_navigation globalmap.launch
```
####  Next part haven't been finished yet.
6. To navigate robot
```bash
#terminal 6 (path finding)
roslaunch path_finding astar_path_finding_with_social_proxemics.launch
sourcemars
#terminal 7 (path following)
sourcemars
roslaunch path_tracking path_tracking_autonomous.launch
```

####  Use aliases
Also, you can do the samething above using aliases in docker container.
```bash
#each command in different terminal
cmd1
cmd2
cmd3
cmd4
cmd5
cmd6
```

Only open mars model
---
If you only want to open gazebo model in gazebo you can try this.
```bash
#terminal 1 (open gazebo in mars)
sourcemars
roslaunch mars_lite_description mars_gazebo.launch realsense_enabled:=true world:=world
#terminal 2 (see all state in rviz and open moveit)
sourcemars
roslaunch mars_lite_moveit_config mars_lite_moveit_planning_execution_gz.launch
```
reference
---
1. [socially-aware-walker](https://github.com/coolcat647/socially-aware-walker)
2. [marslite robot](https://github.com/coolcat647/mars_lite_simulation_ws)
