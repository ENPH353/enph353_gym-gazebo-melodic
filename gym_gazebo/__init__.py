import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

## ENPH 353 Adeept Awr envs
register(
    id='Gazebo_ENPH_Ai_Adeept_Awr_Empty-v0',
    entry_point='gym_gazebo.envs.adeept_awr:Gazebo_ENPH_Ai_Adeept_Awr_Empty_Env',
    max_episode_steps=70000,
)
register(
    id='Gazebo_ENPH_Ai_Adeept_Awr_Empty_NN-v0',
    entry_point='gym_gazebo.envs.adeept_awr:Gazebo_ENPH_Ai_Adeept_Awr_Empty_NN_Env',
    max_episode_steps=300,
)

# Turtlebot envs
register(
    id='GazeboMazeTurtlebotLidar-v0',
    entry_point='gym_gazebo.envs.turtlebot:GazeboMazeTurtlebotLidarEnv',
    # More arguments here
)
register(
    id='GazeboCircuitTurtlebotLidar-v0',
    entry_point='gym_gazebo.envs.turtlebot:GazeboCircuitTurtlebotLidarEnv',
    # More arguments here
)
register(
    id='GazeboCircuit2TurtlebotLidar-v0',
    entry_point='gym_gazebo.envs.turtlebot:GazeboCircuit2TurtlebotLidarEnv',
    # More arguments here
)
register(
    id='GazeboCircuit2TurtlebotLidarNn-v0',
    entry_point='gym_gazebo.envs.turtlebot:GazeboCircuit2TurtlebotLidarNnEnv',
    max_episode_steps=1000,
    # More arguments here
)
register(
    id='GazeboCircuit2cTurtlebotCameraNnEnv-v0',
    entry_point='gym_gazebo.envs.turtlebot:GazeboCircuit2cTurtlebotCameraNnEnv',
    # More arguments here
)
register(
    id='GazeboRoundTurtlebotLidar-v0',
    entry_point='gym_gazebo.envs.turtlebot:GazeboRoundTurtlebotLidarEnv',
    # More arguments here
)

# cart pole
register(
    id='GazeboCartPole-v0',
    entry_point='gym_gazebo.envs.gazebo_cartpole:GazeboCartPolev0Env',
)
