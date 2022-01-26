from gym.envs.registration import register

register(
    id='mycartpole-v0',
    entry_point='gym_cartpole.envs:Cartpole_Env',
)