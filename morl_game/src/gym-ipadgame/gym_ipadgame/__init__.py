from gym.envs.registration import register

register(
    id='ipadgame-v0',
    entry_point='gym_ipadgame.envs:GameEnv',
)
# register(
#     id='foo-extrahard-v0',
#     entry_point='gym_foo.envs:FooExtraHardEnv',
# )
