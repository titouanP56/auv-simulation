import rclpy
from rclpy.node import Node
import subprocess
import os

class AutoSaverNode(Node):
    """
    ROS 2 Node to automatically save the OctoMap at regular intervals.
    
    This node triggers a system call to `octomap_saver_node` every 60 seconds
    to ensure the mapping progress is persistently saved without manual intervention.
    It also forces a final save upon keyboard interrupt (Ctrl+C).
    """
    def __init__(self):
        super().__init__('auto_saver_node')
        # ROS 2 Timer - Execution every 60.0 seconds
        self.timer = self.create_timer(60.0, self.save_map)
        self.get_logger().info('AutoSaverNode activated: saving map every 60s.')

    def save_map(self):
        self.get_logger().info('Launching OctoMap save in the background...')
        
        # Deduce the source folder path dynamically via installation
        try:
            from ament_index_python.packages import get_package_prefix
            pkg_prefix = get_package_prefix('auv_perception')
            # pkg_prefix = ".../install/auv_perception", go up 2 levels to enter src
            save_path = os.path.abspath(os.path.join(pkg_prefix, '../../src/auv_perception', 'net_map_autosave.bt'))
        except Exception:
            # Safety fallback
            save_path = os.path.expanduser('~/net_map_autosave.bt')
            
        try:
            # Execute the strict system command as required
            subprocess.run(
                ['ros2', 'run', 'octomap_server', 'octomap_saver_node', '--ros-args', '-p', f'octomap_path:={save_path}'],
                check=True,
                capture_output=True,
                text=True,
                timeout=5.0 # A 5-second timeout prevents the process from blocking indefinitely (very useful during general Ctrl+C)
            )
            self.get_logger().info(f'Success: Map saved as {save_path}')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Map saving took too long (timeout).')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'OctoMap save failed. Code: {e.returncode}, Stderr: {e.stderr}')

def main(args=None):
    rclpy.init(args=args)
    node = AutoSaverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Capture Ctrl+C to force a final save
        node.get_logger().info('Interrupt (Ctrl+C) detected. Launching final save...')
        node.save_map()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
