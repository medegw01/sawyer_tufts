# sawyer_tufts
 
## Introduction:
This package when ran launches gazebo world, a table and a specified block on the table via a python script. The user specifies the block's serial number(0 through 4) and the number of times to run the code. The joint_state is recorded into a rosbag. The naming convention for the rosbag file is:
>                     <sawyer_model>_<Block number>_<Time stamp>.bag
example:
>                      sawyer_model_4_2018-08-01-14-25-14.bag

## Run:
To run the code, the user runs the following in the commandline:
>                     roslaunch sawyer_tufts pick_and_place_tufts.launch num_of_run:="natural number"
example:
>                     roslaunch sawyer_tufts pick_and_place_tufts.launch run_info:=4
 
where:
- sawyer_tufts                -> the name of package
- pick_and_place_tufts.launch -> the launch file for the gazebo world and the demo python script
- num_of_run                  -> number of desired time the robot should pick block. Number should be greater than

## Video demo:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/FynCZzQ-ldQ/0.jpg)](https://www.youtube.com/watch?v=FynCZzQ-ldQ)
