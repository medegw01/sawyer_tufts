# sawyer_tufts
 
## Introduction:
This package when ran launches gazebo world, a table and a specified block on the table via a python script. The user specifies the block's serial number(0 through 4) and the number of times to run the code. The joint_state is recorded into a rosbag. The naming convention for the rosbag file is:
>                     <sawyer__model><Block number>__<Time stamp>.bag
example:
>                      sawyer__model4__2018-08-01-14-25-14.bag

## Run:
To run the code, the user runs the following in the commandline:
>                     roslaunch sawyer_tufts pick_and_place_tufts.launch run_info:="block_number num_of_run"
example:
>                     roslaunch sawyer_tufts pick_and_place_tufts.launch run_info:="4 1"'
 
where:
- sawyer_tufts                -> the name of package
- pick_and_place_tufts.launch -> the launch file for the gazebo world and the demo python script
- run_info                    -> run information which will contain the run arguments
- block_number                -> for this demo, it's from 0 to 4. With 0 being the heaviest and 4, the lightest
- num_of_run                  -> number of desired time the robot should pick block. Number should be greater than

## Video demo:
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/FynCZzQ-ldQ/0.jpg)](https://www.youtube.com/watch?v=FynCZzQ-ldQ)