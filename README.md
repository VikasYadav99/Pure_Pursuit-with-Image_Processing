# Pure_Pursuit-with-Image_Processing

## Steps to run
1. Extract the folder vac_description in your ROS workspace.
2. Open the terminal and change your directory to the workspace. (cd your_workspace)
3. Run catkin build command. (catkin build or rosbuild)
4. Source the environment. (source devel/setup.bash)
5. Launch the gazebo.launch file. (roslaunch vac_description gazebo.launch)
6. Open new tab in the terminal and run the controls.py script. (rosrun vac_description controls.py)

Now you can see the robot following the pre-defined path.

Program for locomotion and pure pursuit controller are in controls.py, pure_pursuit.py and image_process.py under scripts folder.
