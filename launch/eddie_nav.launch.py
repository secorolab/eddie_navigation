from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    # launch eddie simulation
    sl.include('eddie_gazebo', 'run_sim.launch.py')

    # rviz
    sl.rviz(sl.find('eddie_navigation', 'eddie_gz.rviz', 'config/rviz'))

    # launch slam
    sl.include('slam_toolbox', 'online_async_launch.py', launch_arguments={
        'use_sim_time': 'true',
        'autostart': 'true',
        # 'params_file': sl.find('eddie_navigation', 'nav2_params.yaml', 'config')
    })

    return sl.launch_description()