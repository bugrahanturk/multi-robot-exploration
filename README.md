# Multirobot Exploration 

## Project Description
This project implements a Multi-Robot Frontier Exploration System using TurtleBot3 robots in the ROS1 Noetic environment. The system enables four autonomous robots to collaboratively explore an unknown environment, detect frontier regions, and merge their exploration maps in real time.

Key Features:

- Multi-robot coordination using move_base
- Real-time map merging with multirobot-map-merge
- Optimized path planning for efficient exploration
- Intelligent frontier detection and target assignment

## Environment Setup
The node configurations for the turtlebot3_navigation and turtlebot3_gazebo packages from the [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) and [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) repositories should be set up for 4 tb3 robots. The necessary configurations for multirobot have been made in the micromouse_maze environment for testing.

The required node connections following the adjustments are provided in the following tf tree.
<p>
<img src="imgs/tf_tree_last.png" width="874"/>
</p>

```
# Install required ROS package for multi-robot map merging
sudo apt-get install ros-noetic-multirobot-map-merge

# Create a workspace for multi-robot exploration
mkdir -p ~/multi_tb3_fe/src
cd ~/multi_tb3_fe/src

# Clone necessary repositories
git clone https://github.com/bugrahanturk/multi-robot-exploration.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Move necessary folders and clean up
mv multi-robot_exploration_mapping*/micromouse_maze .
mv multi-robot_exploration_mapping*/multirobot_exploration_mapping .
rm -r multi-robot_exploration_mapping*

# Install dependencies
cd ~/multi_tb3_fe/
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# Build the workspace
catkin_make
source ~/.bashrc
```

## Starting the Simulation Environment
Four Turtlebot3 robots will be operated, utilizing the burger model due to the absence of the camera-depth sensor. Therefore, the TURTLEBOT3_MODEL variable in the ./bashrc file should be assigned an appropriate value.
```
gedit ~/.bashrc
```
```
export TURTLEBOT3_MODEL=burger
```
Starting the simulation environment:
```
# Start the simulation environment with four TurtleBot3 robots
roslaunch micromouse_maze micromouse_maze3_multi.launch

# Launch multi-robot map merging (combines exploration maps)
roslaunch turtlebot3_gazebo multi_map_merge.launch

# Start SLAM (Simultaneous Localization and Mapping) for all robots
roslaunch turtlebot3_gazebo multi_turtlebot3_slam.launch

# Run move_base for path planning and navigation
roslaunch turtlebot3_navigation multi_move_base.launch

# Launch RViz for visualization
roslaunch micromouse_maze multi_robot_rviz.launch

# Start the multi-robot exploration algorithm
rosrun multi_robot_exploration multi_robot_exploration
```

Starting the multi_robot_exploration node:
```
rosrun multi_robot_exploration multi_robot_exploration
```

##  Multi-Robot Exploration Process
The multi-robot discovery algorithm consists of the following steps,
1. Frontier Detection →  2. Clustering → 3. Target Assignment → 4. Path Planning → 5. Robot Navigation → 6. Map Merging

**Frontier Detection and Clustering**

Unexplored frontier regions are detected by calling the findFrontierClusters() function. 
Frontier cells, cells neighboring unknown areas, are identified with isFrontierCell().

**Creating clusters**

Connected frontier cells are clustered using CLC with the labelComponent() function.
For each cluster, properties such as centroid and information potential are calculated with calculateClusterProperties(). If frontier regions are not found, the algorithm waits for the next map update.

**Determining Robot Position**

The position of the robots is continuously updated in the map frame using odometry data. This is done with the poseCallback() function and the transformation between the robot's local frame and the global map frame is calculated using tf2.

**Target Assignment**

If frontier clusters are found, the assignTargets() function assigns frontier clusters to existing robots according to their utility value. The utility value is calculated as follows:
Utility = Information Potential - (Distance * ALPHA), this calculation is done with the calculateUtility() function. For each robot, the nearest and accessible target is determined with the findAccessibleTarget() function. The target is sent to the relevant robot using the sendGoal() function.

**Route Planning and Cache Usage**

To optimize path calculations, the algorithm uses the PathCache structure. This structure stores previously computed path lengths between robots and frontier clusters and prevents unnecessary computations. If the cache expires or is not valid, the paths are recalculated with the calculateAllPaths() function.

**Robot Movement and Tracking**

After the target is assigned, a move command is sent to the robot using the move_base action client. The robot's progress to the target is monitored with doneCallback(), activeCallback() and feedbackCallback() functions. These feedbacks allow updating the robot status and reassigning the target if necessary.

**Reassignment and Alternative Destinations**

If a robot fails to reach a destination or encounters obstacles, the algorithm tries alternative destinations within the cluster. If all the alternatives fail, the robot is re-qualified for new destinations.

## Results
Logs:
<p>
<img src="imgs/kesif_debug.png" width="874"/>
</p>


Exploring Start:
<p>
<img src="imgs/baslangic_kesif.png" width="874"/>
</p>

<p>
<img src="imgs/buytuk_alan.png" width="874"/>
</p>

<p>
<img src="imgs/kesif_baslangic_ilerleyis.png" width="874"/>
</p>

<p>
<img src="imgs/buyumus_3.png" width="874"/>
</p>

<p>
<img src="imgs/kesif_sonucu.png" width="874"/>
</p>

<p>
<img src="imgs/kesif_snouc2.png" width="874"/>
</p>
