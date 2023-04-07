# subcanopy_flight

## Installation
Install RotorS and rpg_quadrotor_control following these instructions https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/Installation-Guide <br />
If you run into a build error regarding an xacro file, just delete the commented out lines in the relevant file and it should build
```
git clone git@github.com:rachelzh9/subcanopy_flight.git
```

## Running
Launching quadrotor in an empty world with sample trajectory tracking
```
roslaunch rpg_mpc follow_trajectory.launch
```
Launching in empty world with GUI
```
roslaunch rpg_rotors_interface quadrotor_empty_world.launch use_mpc:=true
```
