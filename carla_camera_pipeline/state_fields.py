"""Shared layout of the /carla/ego/state message (no CARLA dependency)."""

STATE_FIELDS = [
    'sim_time',                 # s
    'x', 'y', 'z',              # m (left-handed, UE4)
    'roll', 'pitch', 'yaw',     # deg
    'speed',                    # m/s
    'steer', 'throttle', 'brake',  # labels
]
