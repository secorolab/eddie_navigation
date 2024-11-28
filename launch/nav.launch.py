from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    sl.include(
        "eddie_navigation",
        "navigation.launch.py",
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "use_respawn": "true",
            "params_file": sl.find("eddie_navigation", "nav2_params.yaml", "config"),
        },
    )

    return sl.launch_description()