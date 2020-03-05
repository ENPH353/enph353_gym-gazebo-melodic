# An OpenAI gym extension for using Gazebo known as `gym-gazebo`

**`gym-gazebo` is a complex piece of software for roboticists that puts together simulation tools, robot middlewares (ROS, ROS 2), machine learning and reinforcement learning techniques. All together to create an environment whereto benchmark and develop behaviors with robots. Setting up `gym-gazebo` appropriately requires relevant familiarity with these tools.**

## Installation
Refer to [INSTALL.md](INSTALL.md)

## Usage

### Build and install gym-gazebo

In the root directory of the repository:

```bash
sudo pip install -e .
```

### Running an environment

To run the cartpole environment go to directory where gym-gazebo is contained, then run:
```
source enph353_gym-gazebo/gym_gazebo/envs/ros_ws/devel/setup.bash
cd enph353_gym-gazebo/examples/gazebo_cartpole  
python gazebo_cartpole_v0.py
```

To run the robot environment go to directory where gym-gazebo is contained, then run:
```
source enph353_gym-gazebo/gym_gazebo/envs/ros_ws/devel/setup.bash
cd enph353_gym-gazebo/examples/adeept_awr
python enph_ai_adeept_awr_empty_qlearn.py
```


### Killing background processes

Sometimes, after ending or killing the simulation `gzserver` and `rosmaster` stay on the background, make sure you end them before starting new tests.

We recommend creating an alias to kill those processes.

```bash
echo "alias killgazebogym='killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient'" >> ~/.bashrc
```
