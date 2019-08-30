import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)

# Gazebo
# ----------------------------------------

## ENPH 353 Adeept Awr envs
register(
    id='Gazebo_ENPH_Ai_Adeept_Awr_Empty-v0',
    entry_point='gym_gazebo.envs.adeept_awr:Gazebo_ENPH_Ai_Adeept_Awr_Empty_Env',
    max_episode_steps=500,
)
register(
    id='Gazebo_ENPH_Ai_Adeept_Awr_Empty_NN-v0',
    entry_point='gym_gazebo.envs.adeept_awr:Gazebo_ENPH_Ai_Adeept_Awr_Empty_NN_Env',
    max_episode_steps=300,
)

# cart pole
register(
    id='GazeboCartPole-v0',
    entry_point='gym_gazebo.envs.gazebo_cartpole:GazeboCartPolev0Env',
)
