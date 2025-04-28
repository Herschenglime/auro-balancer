from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


# shutdown node on exit
def shutdown_on_exit(node):
    return RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[Shutdown()]))


def generate_launch_description():
    servo_node = Node(
        package="auro_balancer", executable="servo_controller", name="servo"
    )
    imu_node = Node(package="auro_balancer", executable="imu_publisher", name="imu")
    dist_node = Node(package="auro_balancer", executable="dist_publisher", name="dist")
    pid_node = Node(package="auro_balancer", executable="kalman_pid", name="pid")

    return LaunchDescription(
        [
            servo_node,
            imu_node,
            dist_node,
            pid_node,
            # If ANY node exits, shutdown everything
            shutdown_on_exit(servo_node),
            shutdown_on_exit(imu_node),
            shutdown_on_exit(dist_node),
            shutdown_on_exit(pid_node),
        ]
    )
