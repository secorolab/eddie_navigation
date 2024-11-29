from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    # launch eddie simulation
    sl.include("eddie_gazebo", "run_sim.launch.py")

    # rviz
    sl.rviz(sl.find("eddie_navigation", "eddie_gz.rviz", "config/rviz"))

    sl.include(
        "eddie_navigation",
        "slam.launch.py",
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "use_lifecycle_manager": "false",
            "slam_params_file": sl.find(
                "eddie_navigation", "mapper_params_online_async.yaml", "config"
            ),
        },
    )

    return sl.launch_description()
