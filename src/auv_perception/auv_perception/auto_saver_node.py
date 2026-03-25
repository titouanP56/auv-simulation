import rclpy
from rclpy.node import Node
import subprocess
import os

class AutoSaverNode(Node):
    def __init__(self):
        super().__init__('auto_saver_node')
        # Timer ROS 2 - Exécution toutes les 60.0 secondes
        self.timer = self.create_timer(60.0, self.save_map)
        self.get_logger().info('AutoSaverNode activé : sauvegarde de la carte toutes les 60s.')

    def save_map(self):
        self.get_logger().info('Lancement de la sauvegarde OctoMap en arrière-plan...')
        
        # En déduire le chemin dossier source dynamiquement via l'installation
        try:
            from ament_index_python.packages import get_package_prefix
            pkg_prefix = get_package_prefix('auv_perception')
            # pkg_prefix = ".../install/auv_perception", on remonte de 2 crans pour aller dans src
            save_path = os.path.abspath(os.path.join(pkg_prefix, '../../src/auv_perception', 'net_map_autosave.bt'))
        except Exception:
            # Fallback de sûreté
            save_path = os.path.expanduser('~/carte_filet_autosave.bt')
            
        try:
            # Exécution de la commande système stricte comme exigé
            subprocess.run(
                ['ros2', 'run', 'octomap_server', 'octomap_saver_node', '--ros-args', '-p', f'octomap_path:={save_path}'],
                check=True,
                capture_output=True,
                text=True,
                timeout=5.0 # Un timeout de 5 secondes évite que le processus ne bloque indéfiniment (très utile lors du Ctrl+C général)
            )
            self.get_logger().info(f'Succès : Carte sauvegardée sous {save_path}')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('La sauvegarde a pris trop de temps (timeout).')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Échec de la sauvegarde OctoMap. Code: {e.returncode}, Stderr: {e.stderr}')

def main(args=None):
    rclpy.init(args=args)
    node = AutoSaverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Capture du Ctrl+C pour forcer une ultime sauvegarde
        node.get_logger().info('Interruption (Ctrl+C) détectée. Lancement de la sauvegarde finale...')
        node.save_map()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
