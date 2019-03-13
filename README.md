# MonkeyTask
# How to set-up the workspace:
  Follow this:
  http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Don't forget to add the setup.bash files in the ~/.bashrc file!

Set-up teh ssh key:
Follow this link:
https://help.github.com/en/enterprise/2.16/user/articles/connecting-to-github-with-ssh



#  Dependencies
Install SDL libarary: sudo apt-get install libsdl2-dev

# How to set-up the packge

## Installation
```
roscd
cd ../src/
git clone git@github.com:beatricebarra/MonkeyTask.git
cd MonkeyTask
git submodule init
git submodule update
cd ../../
rospack profile
catkin_make
```
If you get an error saying that 
```
 No such file or directory
 #include "kuka_fri_bridge/JointStateImpedance.h"
                                                 ^
```
Don't worry. Repeat the following steps agian, mayble close and open the terminal also!
```
rospack profile
catkin_make
```
# How to run

Run the robot:

In Termianl 1:
```
roscore
```
In Termianl 2:
```
rosrun  fri_iiwa_ros fri_iiwa_example_ros
```
In Termianl 3:
```
roscd
cd ../src/MonkeyTask/miscellaneous/robot-toolkit/
./bin/robot_simulator --config packages/monkeytask/SlowMovementApplication
```

"SlowMovementApplication" is the name of your package!
 stop the robot before turning off.

# How to set-up eclise:
```
cd ~/catkin_ws
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
```
Then you can import it into the eclipse workspace
