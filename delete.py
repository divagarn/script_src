    def disinfect_navigator_start(self, max_x_vel):
        self.disinfect_navigation_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.disinfect_navigation_uuid)
        navigator_args = ['max_x_vel:={}'.format(max_x_vel)]

        launch_file = roslaunch.rlutil.resolve_launch_arguments([DISINFECT_NAVIGATION_LAUNCH_PATH] + navigator_args)
        self.disinfect_navigator_launch = roslaunch.parent.ROSLaunchParent(self.disinfect_navigation_uuid,
                                                                      [(launch_file, navigator_args)])
        self.disinfect_navigator_launch.start()

<arg name="max_x_vel" default="0.2"/>
<include file="$(find your_package_name)/launch/move_base_sanitize.launch">
    <arg name="max_x_vel" value="$(arg max_x_vel)"/>
</include>
