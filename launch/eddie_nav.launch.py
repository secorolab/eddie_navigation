from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    return sl.launch_description()
