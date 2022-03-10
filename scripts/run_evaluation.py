import roslaunch
import time

class LaunchFile:
    def __init__(self, package, file, args=[]):
        self.file = [(roslaunch.rlutil.resolve_launch_arguments([package, file])[0], args)]
        self.is_launched = False

    def launch(self, uuid):
        self.parent = roslaunch.parent.ROSLaunchParent(uuid, self.file)
        self.parent.start()
        self.is_launched = True
        return self.parent

    def spin(self):
        self.parent.spin_once()

    def shutdown(self):
        print('Shutting down')
        self.parent.shutdown()
        print('Shutdown complete')

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
world_launch = LaunchFile('ur_with_cam_gazebo', 'ur_with_cam.launch', ['world_name:=world19', 'base:=retractable'])
superellipsoid_launch = LaunchFile('capsicum_superellipsoid_detector', 'start_sim.launch')

world_launch.launch(uuid)
time.sleep(5)
superellipsoid_launch.launch(uuid)
time.sleep(5)
superellipsoid_launch.shutdown()
time.sleep(5)
world_launch.shutdown()