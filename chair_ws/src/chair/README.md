This repository contains code associated with the airchair project.

The goal here is to support N wheelchairs with different targets on them.
By default the chairs are named chair_a through chair_x. There is a launch file that just launches chair_a and another that launches a and b. Should be clear from those examples.

To drive them around with the keyboard, use


ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=chair_a/cmd_vel 

