from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    sl.declare_arg("use_sim_time", "True")
    sl.declare_arg("params_file", "nav2_params.yaml")
    sl.declare_arg("initial_x", "0.0")
    sl.declare_arg("initial_y", "0.0")
    sl.declare_arg("initial_yaw", "0.0")

    nav2_yaml = sl.find("eddie_navigation", sl.arg("params_file"), "config")

    sl.node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_yaml, {
            "initial_pose.x": sl.arg("initial_x"),
            "initial_pose.y": sl.arg("initial_y"),
            "initial_pose.yaw": sl.arg("initial_yaw"),
        }],
    )

    sl.node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": sl.arg("use_sim_time")},
            {"autostart": True},
            {"node_names": ["amcl"]},
        ],
    )

    return sl.launch_description()
