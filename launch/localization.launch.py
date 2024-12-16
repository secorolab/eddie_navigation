from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    sl.declare_arg("use_sim_time", "True")

    nav2_yaml = sl.find("eddie_navigation", "nav2_params.yaml", "config")

    sl.node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_yaml],
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
